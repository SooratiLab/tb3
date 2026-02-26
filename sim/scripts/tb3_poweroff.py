#!/usr/bin/env python3
"""
Shutdown TurtleBot3 robots remotely.

Automatically detects valid SSH usernames and caches them for faster subsequent runs.
Prioritizes passwordless (key-based) SSH connections over password-required connections.

Customization:
    Edit COMMON_SSH_USERS list below to add custom usernames for auto-detection.

Usage:
    python tb3_poweroff.py 1        # Shutdown tb1
    python tb3_poweroff.py 1 2 3    # Shutdown tb1, tb2, tb3
    python tb3_poweroff.py -H tb1   # Shutdown specific host tb1
    python tb3_poweroff.py          # Shutdown all online TurtleBots (auto-detected)
    python tb3_poweroff.py -p pass  # With sudo password
    python tb3_poweroff.py -d       # Dry-run: list devices without shutting down
    python tb3_poweroff.py -v       # Verbose: show SSH detection details
    python tb3_poweroff.py -q       # Quiet: suppress detailed output
    python tb3_poweroff.py -u tb    # Use specific SSH username

    CLI alias: tb3_poweroff, pwf
"""

import sys
import subprocess
import argparse
import time
import csv
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock

# Add scripts directory to path for importing Terminal
SCRIPT_DIR = Path(__file__).parent.resolve()
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)

from terminal import Terminal

# Cache file for storing SSH user information
CACHE_FILE = SCRIPT_DIR / ".quick.csv"

# Common SSH usernames to try for auto-detection
# You can add your own common usernames here if needed
COMMON_SSH_USERS = ['ubuntu', 'tb', 'pi', 'group', 'turtlebot']

# Global lock for synchronized printing when verbose
_print_lock = Lock()


def synchronized_print(*args, **kwargs):
    """Thread-safe print function."""
    with _print_lock:
        print(*args, **kwargs)


def get_tailscale_devices():
    """Get all devices from tailscale status.
    
    Returns:
        List of dicts with keys: ip, hostname, os, is_online
    """
    try:
        proc = subprocess.run(
            ["tailscale", "status"],
            capture_output=True,
            text=True,
            check=True,
        )
    except Exception as e:
        print(f"Failed to run `tailscale status`: {e}")
        return []
    
    devices = []
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        
        ip = parts[0]
        hostname = parts[1]
        os_type = parts[3] if len(parts) > 3 else ""
        
        # Check if offline
        is_online = "offline" not in line.lower()
        
        devices.append({
            "ip": ip,
            "hostname": hostname,
            "os": os_type,
            "is_online": is_online,
        })
    
    return devices


def resolve_ip(ssh_user: str) -> str | None:
    """Resolve TurtleBot IP address using tailscale."""
    devices = get_tailscale_devices()
    for device in devices:
        if device["hostname"] == ssh_user:
            return device["ip"]
    return None


def find_online_turtlebots():
    """Find all online TurtleBot devices from tailscale.
    
    Filters for:
    - Linux devices only
    - Hostname starts with 'tb' or 'ubuntu' (preferred)
    - If none found, fallback to any online Linux device
    - Excludes hostnames starting with 'aice'
    - Device is online (not offline)
    
    Returns:
        List of hostnames (e.g., ['tb1', 'tb10', 'ubuntu'])
    """
    devices = get_tailscale_devices()
    turtlebots = []
    fallback_devices = []
    
    for device in devices:
        hostname = device["hostname"]
        os_type = device["os"]
        is_online = device["is_online"]
        
        # Must be online
        if not is_online:
            continue
        
        # Must be Linux
        if os_type.lower() != "linux":
            continue
        
        # Exclude aice* devices
        if hostname.startswith("aice"):
            continue
        
        if hostname.startswith("tb") or hostname.startswith("ubuntu"):
            turtlebots.append(hostname)
        else:
            fallback_devices.append(hostname)
    
    # If no tb/ubuntu devices found, use fallback
    if not turtlebots and fallback_devices:
        return fallback_devices
    
    return turtlebots


def read_cache() -> dict:
    """Read SSH user cache from CSV file.
    
    Returns:
        Dict mapping hostname to dict with keys: ip, username, passwordless, last_updated
    """
    cache = {}
    if not CACHE_FILE.exists():
        return cache
    
    try:
        with open(CACHE_FILE, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                cache[row['hostname']] = {
                    'ip': row['ip'],
                    'username': row['username'],
                    'passwordless': row.get('passwordless', 'false').lower() == 'true',
                    'last_updated': row.get('last_updated', ''),
                }
    except Exception as e:
        print(f"Warning: Failed to read cache file: {e}")
    
    return cache


def write_cache(cache: dict):
    """Write SSH user cache to CSV file.
    
    Args:
        cache: Dict mapping hostname to dict with keys: ip, username, passwordless, last_updated
    """
    try:
        with open(CACHE_FILE, 'w', newline='') as f:
            fieldnames = ['hostname', 'ip', 'username', 'passwordless', 'last_updated']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            for hostname, info in sorted(cache.items()):
                writer.writerow({
                    'hostname': hostname,
                    'ip': info['ip'],
                    'username': info['username'],
                    'passwordless': str(info.get('passwordless', False)).lower(),
                    'last_updated': info.get('last_updated', ''),
                })
    except Exception as e:
        print(f"Warning: Failed to write cache file: {e}")


def update_cache_entry(hostname: str, ip: str, username: str, passwordless: bool):
    """Update a single cache entry."""
    from datetime import datetime
    
    cache = read_cache()
    cache[hostname] = {
        'ip': ip,
        'username': username,
        'passwordless': passwordless,
        'last_updated': datetime.now().isoformat(),
    }
    write_cache(cache)


def detect_ssh_users(hostname: str, ip: str, verbose: bool = True) -> dict:
    """
    Try to detect valid SSH usernames for the host.
    
    Args:
        hostname: The hostname of the device
        ip: The IP address to connect to
        verbose: Whether to print detection progress
    
    Returns:
        Dict with 'passwordless' and 'password' lists of valid usernames
    """
    # Add common SSH usernames to try, with the hostname itself as the first priority
    common_users = [hostname] + COMMON_SSH_USERS
    
    # Use synchronized print for thread-safe output
    print_fn = synchronized_print if verbose else lambda *args, **kwargs: None
    
    if verbose:
        print_fn(f"[{hostname}] Auto-detecting SSH username...")
    
    passwordless_users = []
    password_users = []
    
    for user in common_users:
        try:
            # First try with BatchMode to quickly test key-based auth
            result = subprocess.run(
                [
                    "ssh",
                    "-o", "ConnectTimeout=2",
                    "-o", "BatchMode=yes",
                    "-o", "StrictHostKeyChecking=no",
                    "-o", "UserKnownHostsFile=/dev/null",
                    "-o", "LogLevel=ERROR",
                    f"{user}@{ip}",
                    "echo OK"
                ],
                capture_output=True,
                timeout=3,
                text=True,
            )
            
            output_combined = result.stdout + result.stderr
            
            if verbose:
                print_fn(f"[{hostname}]   {user}: BatchMode test - rc={result.returncode}, "
                      f"stdout={repr(result.stdout[:50])}, stderr={repr(result.stderr[:50])}")
            
            # Check for password change required in the output
            if ("change your password" in output_combined.lower() or 
                "password expired" in output_combined.lower()):
                if verbose:
                    print_fn(f"[{hostname}]   {user}: requires password change (skipping)")
                continue
            
            # Test successful batch mode for passwordless SSH
            if result.returncode == 0 and "OK" in result.stdout:
                try:
                    verify_terminal = Terminal(
                        name=f"{hostname}-{user}-verify", verbose=False)
                    
                    ssh_cmd = (
                        f"ssh -tt "
                        f"-o ConnectTimeout=3 "
                        f"-o StrictHostKeyChecking=no "
                        f"-o UserKnownHostsFile=/dev/null "
                        f"-o LogLevel=ERROR "
                        f"-o PasswordAuthentication=no "
                        f"{user}@{ip}"
                    )
                    
                    verify_terminal.run(
                        ssh_cmd,
                        detached=True,
                        capture_output=True,
                        print_output=False,
                        register_signals=False,
                    )
                    
                    time.sleep(1)
                    verify_terminal.send_stdin("exit\n")
                    
                    # Poll for output for up to 3 seconds total
                    start_time = time.time()
                    all_output = ""
                    while time.time() - start_time < 3.0:
                        output = verify_terminal.pop_output() or ""
                        all_output += output
                        
                        # Stop if we have significant output or if process has exited
                        if len(all_output) > 50 or not verify_terminal.is_running():
                            break
                        
                        time.sleep(0.3)
                    
                    # Get any remaining output
                    final_output = verify_terminal.pop_output() or ""
                    all_output += final_output
                    
                    verify_terminal.terminate_process()
                    
                    if verbose:
                        print_fn(f"[{hostname}]   {user}: Interactive test - output={repr(all_output[:250])}")
                    
                    # Ignore password change users
                    if ("change your password" in all_output.lower() or 
                        "password expired" in all_output.lower()):
                        if verbose:
                            print_fn(f"[{hostname}]   {user}: requires password change (skipping)")
                        continue
                    
                    # Look for common shell prompts to confirm a real connection
                    got_connection = (
                        "Welcome to" in all_output or
                        "Ubuntu" in all_output or
                        "Linux" in all_output or
                        f"{user}@" in all_output or
                        "Last login" in all_output or
                        any(c in all_output for c in ["$", "#"])
                    )
                    
                    if got_connection:
                        passwordless_users.append(user)
                    elif verbose:
                        print_fn(f"[{hostname}]   {user}: No connection success indicators (skipping)")
                except Exception as e:
                    if verbose:
                        print_fn(f"[{hostname}]   {user}: Interactive test failed: {e} (skipping)")
                    continue
                continue
            
            # Check if connection was denied due to password requirement
            if ("denied" not in result.stderr.lower() and 
                "permission" in result.stderr.lower()):
                password_users.append(user)
        
        # Try next user on errors
        except subprocess.TimeoutExpired:
            continue
        except Exception:
            continue
    
    if verbose:
        if passwordless_users:
            print_fn(f"[{hostname}] Found passwordless users: {', '.join(passwordless_users)}")
        if password_users:
            print_fn(f"[{hostname}] Found password-required users: {', '.join(password_users)}")
        if not passwordless_users and not password_users:
            print_fn(f"[{hostname}] Could not auto-detect any valid usernames")
    
    return {
        'passwordless': passwordless_users,
        'password': password_users,
    }


def shutdown_tb3(
        ssh_user: str, password: str | None = None, timeout: float = 30.0, 
        verbose: bool = True, username: str | None = None) -> bool:
    """
    Shutdown a single TurtleBot3.
    
    Args:
        ssh_user: hostname to shutdown (e.g., 'tb1', 'tb10', 'ubuntu')
        password: optional sudo password if required
        timeout: timeout in seconds for shutdown confirmation
        verbose: whether to print detailed output
        username: SSH username (defaults to 'ubuntu' for tb* devices, hostname otherwise)
    
    Returns True if shutdown command was sent and connection closed.
    """
    ip = resolve_ip(ssh_user)
    if ip is None:
        print(f"[{ssh_user}] Not found in tailscale, skipping")
        return False
    
    users_to_try = []
    # If a specific username is provided, try only that one
    if username is not None:
        users_to_try = [username]
    else:
        # Check cache first
        cache = read_cache()
        if ssh_user in cache and cache[ssh_user]['ip'] == ip:
            cached_user = cache[ssh_user]['username']
            if verbose:
                print(f"[{ssh_user}] Using cached username: {cached_user}")
            users_to_try = [cached_user]
        else:
            detected = detect_ssh_users(ssh_user, ip, verbose=verbose)
            # Prefer passwordless users, then password users
            users_to_try = detected['passwordless'] + detected['password']
            
            # Fallback if detection fails
            if not users_to_try:
                if ssh_user.startswith("tb"):
                    users_to_try = ["ubuntu"]
                else:
                    users_to_try = [ssh_user]
                if verbose:
                    print(f"[{ssh_user}] Using fallback username: {users_to_try[0]}")
    
    # Try each username until one succeeds
    for ssh_username in users_to_try:
        result = _try_shutdown_with_user(
            ssh_user, ssh_username, ip, password, timeout, verbose)
        if result is not None:
            return result
    
    print(f"[{ssh_user}] All username attempts failed")
    return False


def _try_shutdown_with_user(
        ssh_user: str, ssh_username: str, ip: str, password: str | None, 
        timeout: float, verbose: bool) -> bool | None:
    """
    Try to shutdown using a specific username.
    
    Returns:
        True if shutdown succeeded
        False if this user should be skipped (try next user)
        None if a fatal error occurred (don't try other users)
    """
    
    # Use synchronized print for thread-safe output
    print_fn = synchronized_print if verbose else lambda *args, **kwargs: None
    
    if verbose:
        print_fn(f"[{ssh_user}] Trying {ssh_username}@{ip}...")
    
    terminal = Terminal(name=f"{ssh_user}-{ssh_username}-poweroff", verbose=False)
    
    try:
        # Start SSH session with detached mode for real-time interaction
        # -o StrictHostKeyChecking=no: Skip host key verification
        # -o UserKnownHostsFile=/dev/null: Don't save to known_hosts
        # -o LogLevel=ERROR: Suppress SSH info messages
        # -o PasswordAuthentication=no: Disable password prompts if keys don't work
        # -o PreferredAuthentications=publickey: Try publickey first
        ssh_cmd = (
            f"ssh -tt "
            f"-o StrictHostKeyChecking=no "
            f"-o UserKnownHostsFile=/dev/null "
            f"-o LogLevel=ERROR "
        )
        
        # Only add password authentication options if no password is provided
        if password is None:
            ssh_cmd += (
                f"-o PreferredAuthentications=publickey "
                f"-o PasswordAuthentication=no "
            )
        
        ssh_cmd += f"{ssh_username}@{ip}"
        
        terminal.run(
            ssh_cmd,
            detached=True,
            capture_output=True,
            print_output=False,
            register_signals=False,
        )
        
        # Wait for SSH to connect (look for shell prompt or password)
        start = time.time()
        connected = False
        password_prompted = False
        while time.time() - start < 10.0:
            output = terminal.pop_output() or ""
            
            # Check for successful connection
            if (f"{ssh_username}@" in output or "Last login" in output or 
                "$" in output or "#" in output):
                connected = True
                break
            
            # Check for password prompt - be more specific to avoid false positives
            # Only treat as password prompt if password is enabled (i.e., password parameter was provided)
            if (password is not None and 
                ("password:" in output.lower() or "password for" in output.lower())):
                password_prompted = True
                if verbose:
                    print_fn(f"[{ssh_user}] Sending password...")
                terminal.send_stdin(f"{password}\n")
                time.sleep(1)
                continue
            
            # Check for authentication failures when using key-based auth
            if (password is None and 
                ("permission denied" in output.lower() or "authentication failed" in output.lower())):
                if verbose:
                    print_fn(f"[{ssh_user}] {ssh_username}: Key-based authentication failed")
                return False  # Try next user
            
            # Check for connection errors
            if "connection refused" in output.lower() or "no route to host" in output.lower():
                if verbose:
                    print_fn(f"[{ssh_user}] {ssh_username}: Connection refused")
                return None  # Fatal error, don't try other users
            
            # Check for password change required
            if "change your password" in output.lower() or "password expired" in output.lower():
                if verbose:
                    print_fn(f"[{ssh_user}] {ssh_username}: Password change required (skipping)")
                return False  # Try next user
            
            time.sleep(0.2)
        if not connected:
            if verbose:
                print_fn(f"[{ssh_user}] {ssh_username}: Connection timeout")

            final_output = terminal.pop_output() or ""
            if final_output and verbose:
                if ("denied" in final_output.lower() or "failed" in final_output.lower() or 
                    "error" in final_output.lower() or "change" in final_output.lower()):
                    print_fn(f"[{ssh_user}] {ssh_username}: {final_output[:200]}")
            return False  # Try next user
        
        # Connection successful - update cache
        is_passwordless = (password is None)
        update_cache_entry(ssh_user, ip, ssh_username, is_passwordless)
        
        if verbose:
            print_fn(f"[{ssh_user}] Connected, sending poweroff...")
        
        # Send poweroff command
        terminal.send_stdin("sudo poweroff\n")
        
        time.sleep(0.5)
        sudo_output = terminal.pop_output() or ""
        
        if "password" in sudo_output.lower() and password is not None:
            terminal.send_stdin(f"{password}\n")
            if verbose:
                print_fn(f"[{ssh_user}] Sent sudo password")
        elif "password" in sudo_output.lower() and password is None:
            synchronized_print(f"[{ssh_user}] Sudo password required but not provided (use -p flag)")
            return False
        
        # Wait for "Connection to ... closed" message
        start = time.time()
        while time.time() - start < timeout:
            output = terminal.pop_output() or ""
            
            if "connection" in output.lower() and "closed" in output.lower():
                if verbose:
                    print_fn(f"[{ssh_user}] Shutdown confirmed")
                return True
            
            # Also check if process is no longer running
            if not terminal.is_running():
                if verbose:
                    print_fn(f"[{ssh_user}] Shutdown confirmed")
                return True
            
            time.sleep(0.5)
        
        synchronized_print(f"[{ssh_user}] Timeout waiting for shutdown confirmation")
        return False
        
    except Exception as e:
        synchronized_print(f"\n[{ssh_user}] Failed with exception: {e}")
        if verbose:
            import traceback
    except Exception as e:
        synchronized_print(f"[{ssh_user}] Failed with exception: {e}")
        return False


def process_dry_run_host(hostname: str, args) -> tuple[str, str, str | None]:
    """
    Process a single host in dry-run mode.
    
    Returns:
        Tuple of (hostname, ip, display_user_info)
    """
    ip = resolve_ip(hostname)
    
    if not ip:
        return (hostname, None, None)
    
    # Determine SSH username for display
    if args.user:
        display_user = args.user
        user_info = f"{display_user}@{ip}"
    else:
        # Always auto-detect in dry-run mode (don't use cache)
        # In dry-run, always perform detection (show output if verbose flag is set)
        detected = detect_ssh_users(hostname, ip, verbose=args.verbose)
        users = detected['passwordless'] + detected['password']
        
        if users:
            display_user = users[0]
            is_passwordless = display_user in detected['passwordless']
            # Update cache with detected user
            update_cache_entry(hostname, ip, display_user, is_passwordless)
            
            if len(users) > 1:
                user_info = f"{display_user}@{ip} [will try: {', '.join(users)}]"
            else:
                user_info = f"{display_user}@{ip}"
        else:
            # Fallback - detection found no valid users
            if hostname.startswith("tb"):
                display_user = "ubuntu (fallback)"
            else:
                display_user = f"{hostname} (fallback)"
            user_info = f"{display_user}@{ip}"
    
    return (hostname, ip, user_info)


def parse_args():
    """Parse command-line arguments.
    
    Returns:
        argparse.Namespace: Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description="Shutdown TurtleBot3 robots remotely",
        epilog="Examples:\n"
               "  python tb3_poweroff.py 1        # Shutdown tb1\n"
               "  python tb3_poweroff.py 1 2 3    # Shutdown tb1, tb2, tb3\n"
               "  python tb3_poweroff.py all      # Shutdown all known TurtleBots\n"
               "  python tb3_poweroff.py          # Shutdown all online TurtleBots (auto-detected)\n"
               "  python tb3_poweroff.py humble   # Shutdown device named 'humble'\n"
               "  python tb3_poweroff.py -p pass  # With sudo password\n"
               "  python tb3_poweroff.py -q       # Quiet mode\n"
               "  python tb3_poweroff.py --dry-run # List devices that would be shutdown",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "targets",
        nargs="*",
        help="TurtleBot number(s) to shutdown (e.g., 1 2 3), hostname(s) (e.g., humble sooratilab). If omitted, auto-detects online TurtleBots.",
    )
    parser.add_argument(
        "--hostname", "-o",
        type=str,
        default=None,
        help="Shutdown specific hostname (alternative to positional argument)",
    )
    parser.add_argument(
        "--dry-run", "-d",
        action="store_true",
        help="List devices that would be shutdown without actually shutting them down",
    )
    parser.add_argument(
        "--password", "-p",
        type=str,
        default=None,
        help="Sudo password if required for poweroff",
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Suppress detailed output (only show summary)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output (shows detection details)",
    )
    parser.add_argument(
        "--user", "-u",
        type=str,
        default=None,
        help="SSH username (default: 'ubuntu' for tb* devices, hostname otherwise)",
    )
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    # Determine which robots to shutdown
    hostnames = []
    
    # Handle --hostname/-H flag
    if args.hostname:
        hostnames = [args.hostname]
        print(f"Shutting down device: {args.hostname}")
    elif not args.targets:
        # Auto-detect online TurtleBots
        hostnames = find_online_turtlebots()
        if not hostnames:
            print("No online TurtleBots found via tailscale")
            sys.exit(0)
        print(f"Auto-detected online TurtleBots: {hostnames}")
    else:
        # Convert numeric targets to hostnames, or use as-is if string
        for target in args.targets:
            try:
                idx = int(target)
                hostnames.append(f"tb{idx}")
            except ValueError:
                hostnames.append(target)
        print(f"Shutting down devices: {hostnames}")
    
    # Dry-run mode: Detect, resolve IPs, and determine SSH usernames without shutting down
    if args.dry_run:
        print("\n[DRY RUN] The following devices would be shutdown:")
        
        # Run in parallel for multiple hosts
        if len(hostnames) > 1:
            with ThreadPoolExecutor(max_workers=min(len(hostnames), 10)) as executor:
                future_to_hostname = {
                    executor.submit(process_dry_run_host, hostname, args): hostname 
                    for hostname in hostnames
                }
                
                results = []
                for future in as_completed(future_to_hostname):
                    hostname, ip, user_info = future.result()
                    results.append((hostname, ip, user_info))
                
                # Sort results by original hostname order for consistent output
                results_dict = {h: (i, u) for h, i, u in results}
                for hostname in hostnames:
                    ip, user_info = results_dict.get(hostname, (None, None))
                    if not ip:
                        print(f"  - {hostname} (not found in tailscale)")
                    else:
                        print(f"  - {hostname} ({user_info})")
        else:
            for hostname in hostnames:
                hostname_result, ip, user_info = process_dry_run_host(hostname, args)
                if not ip:
                    print(f"  - {hostname} (not found in tailscale)")
                else:
                    print(f"  - {hostname} ({user_info})")
        
        print(f"\nTotal: {len(hostnames)} device(s)")
        sys.exit(0)
    
    results = {}
    verbose_mode = args.verbose or not args.quiet
    
    # Run in parallel for multiple hosts
    if len(hostnames) > 1:
        with ThreadPoolExecutor(max_workers=min(len(hostnames), 10)) as executor:
            future_to_hostname = {
                executor.submit(
                    shutdown_tb3, hostname, args.password, 30.0, verbose_mode, args.user
                ): hostname 
                for hostname in hostnames
            }
            
            for future in as_completed(future_to_hostname):
                hostname = future_to_hostname[future]
                try:
                    results[hostname] = future.result()
                except Exception as e:
                    synchronized_print(f"[{hostname}] Exception in thread: {e}")
                    results[hostname] = False
    else:
        for hostname in hostnames:
            results[hostname] = shutdown_tb3(
                hostname, password=args.password, verbose=verbose_mode, 
                username=args.user)
    
    # Summary only if some failed
    if not all(results.values()):
        print("=" * 40)
        print("Shutdown Summary:")
        for hostname, success in results.items():
            status = "Sent" if success else "Failed"
            print(f"  {hostname}: {status}")
        
        success_count = sum(results.values())
        print(f"Total: {success_count}/{len(results)} shutdown commands sent")


if __name__ == "__main__":
    main()
