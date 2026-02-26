import os
import sys
import signal
import subprocess
import threading
import logging
from pathlib import Path
from copy import deepcopy
from typing import Optional, Mapping

import pty


def atomic_print(output, flush=True, end="\n"):
    """
    Process-safe atomic print using file locking.

    Prevents interleaved output when multiple processes write to stdout.
    Automatically bypasses locking in Jupyter environments.
    """
    if output is None:
        return

    clean_output = str(output).replace("\r", "")
    in_jupyter = "IPython" in sys.modules

    if in_jupyter:
        print(clean_output, end=end, flush=flush)
        return

    try:
        import fcntl
        fcntl.flock(sys.stdout.fileno(), fcntl.LOCK_EX)
        print(clean_output, end=end, flush=flush)
    finally:
        try:
            fcntl.flock(sys.stdout.fileno(), fcntl.LOCK_UN)
        except Exception:
            pass

def verbose(
    info,
    verbose=True,
    decorator="info",
    sep=False,
    filter_none=False,
    n_dashes=50,
    bottom=False,
    line_by_line=False,
    flush=True,
):
    """
    Process-safe verbose printing utility.
    """
    if not verbose:
        return

    decorator = str(decorator).upper()
    prefix = f"[{decorator}] - " if decorator else "\t"

    if not isinstance(info, (str, tuple, list)):
        atomic_print(
            f"[ERROR] Unsupported verbose type: {type(info)}",
            flush=flush,
        )
        return

    orig_info = info
    if isinstance(info, (tuple, list)):
        orig_info = deepcopy(info)
        if filter_none:
            info = " ".join(str(i) for i in info if i is not None)
        else:
            info = " ".join(str(i) for i in info)

    output = ""

    if sep:
        output += "-" * n_dashes + "\n"

    if isinstance(orig_info, (list, tuple)) and line_by_line:
        for i, line in enumerate(orig_info):
            output += f"{prefix}{line}"
            if i < len(orig_info) - 1:
                output += "\n"
    else:
        output += f"{prefix}{info}"

    if bottom:
        output += "\n" + "-" * max(1, n_dashes // 10)

    atomic_print(output, flush=flush)

# Global variable to hold the verbose function reference for the wrapper
_verbose_func = None

def verbose_wrapper(
    info,
    verbose=None,
    print_verbose=None,
    decorator="info",
    sep=False,
    filter_none=False,
    n_dashes=50,
    bottom=False,
    line_by_line=False,
    flush=True,
):
    """
    Wrapper when `verbose` cannot be used directly.
    Accepts both 'verbose' and 'print_verbose' parameters for compatibility.
    If both are provided, 'print_verbose' takes precedence.
    """
    global _verbose_func
    if _verbose_func is None:
        _verbose_func = globals()['verbose']
    
    # Determine which verbose flag to use (print_verbose takes precedence)
    if print_verbose is not None:
        verbose_flag = print_verbose
    elif verbose is not None:
        verbose_flag = verbose
    else:
        verbose_flag = True
    
    _verbose_func(
        info,
        verbose_flag,
        decorator,
        sep,
        filter_none,
        n_dashes,
        bottom,
        line_by_line,
        flush,
    )


# CLI helpers
def get_cli_args(to_update_only=True):
    """
    Return CLI arguments.

    If to_update_only=True:
        returns ["arg1", "arg2"] for --arg1 --arg2
    Else:
        returns full argv (excluding script name)
    """
    if to_update_only:
        return [arg.lstrip("--") for arg in sys.argv if arg.startswith("--")]
    return sys.argv[1:]

def get_cli_arg_value(arg_name, default=None, pop=False, cast_to=None):
    """
    Return value of CLI argument or True if store-true flag.
    
    Args:
        arg_name: Argument name with or without '--' prefix (e.g., 'num_agents' or '--num_agents')
        default: Value to return if argument not found
        pop: If True, remove the argument (and its value if present) from sys.argv
        cast_to: Optional type or callable to cast the value to (e.g., int, float, str, bool)
                 Can be a type object or callable function
    """
    # Normalize arg_name by stripping leading dashes
    arg_name = arg_name.lstrip("-")
    
    args = get_cli_args(to_update_only=False)
    for i, arg in enumerate(args):
        if arg.lstrip("-") == arg_name:
            # Check if this arg has a value (next element exists and doesn't start with -)
            has_value = i + 1 < len(args) and not args[i + 1].startswith("-")
            
            if has_value:
                value = args[i + 1]
                
                # Apply type casting if specified (before popping to preserve original on error)
                if cast_to is not None:
                    try:
                        # Get the name for error messages
                        type_name = getattr(cast_to, '__name__', str(cast_to))
                        value = cast_to(value)
                    except (ValueError, TypeError) as e:
                        raise ValueError(
                            f"Failed to cast --{arg_name}='{args[i + 1]}' "
                            f"to {type_name}: {e}"
                        ) from e
                
                # Remove from sys.argv after successful casting
                if pop:
                    actual_index = i + 1  # +1 because args doesn't include script name
                    sys.argv.pop(actual_index)  # Remove the flag
                    sys.argv.pop(actual_index)  # Remove the value
                
                return value
            else:
                # It's a store-true flag
                if pop:
                    actual_index = i + 1  # +1 because args doesn't include script name
                    sys.argv.pop(actual_index)
                return True
    return default


# Terminal class
class Terminal:
    """
    Subprocess wrapper with PTY-backed detached execution,
    stdin injection, and incremental stdout consumption.
    
    Args:
        signals: Specifies which signals to register handlers for.
            - "all" or True: Register SIGINT and SIGTERM (default)
            - "none" or False or None: Don't register any signal handlers
            - "sigterm" or "SIGTERM": Register only SIGTERM
            - "sigint" or "SIGINT": Register only SIGINT  
            - List of signal constants: e.g., [signal.SIGTERM]
    """

    def __init__(
        self,
        interactive_shell=False,
        stop_on_error=False,
        name="Terminal",
        verbose=True,
        signals="all",
    ):
        self.interactive_shell = interactive_shell
        self.stop_on_error = stop_on_error
        self.verbose = verbose
        self.name = name
        self.signals = signals

        self._process = None
        self.cmd = ""

        self._pty_master_fd = None
        self._detached_stdout = []
        self._detached_stderr = []

        self._stdout_lock = threading.Lock()

        self._signal_handlers_registered = False
        self._original_handlers = {}
        self._handling_signal = False  # Prevent re-entrant signal handling

    # Signal handling
    
    def _get_signals_to_register(self, signals=None):
        """
        Parse the signals parameter and return list of signal constants to register.
        """
        sig_config = signals if signals is not None else self.signals
        
        # Handle string/bool shortcuts
        if sig_config in ("all", True):
            return [signal.SIGINT, signal.SIGTERM]
        elif sig_config in ("none", False, None):
            return []
        elif sig_config in ("sigterm", "SIGTERM"):
            return [signal.SIGTERM]
        elif sig_config in ("sigint", "SIGINT"):
            return [signal.SIGINT]
        elif isinstance(sig_config, (list, tuple)):
            return list(sig_config)
        else:
            # Default to all if unrecognized
            return [signal.SIGINT, signal.SIGTERM]

    def register_signal_handlers(self, signals=None):
        if self._signal_handlers_registered:
            return

        signals_to_register = self._get_signals_to_register(signals)
        
        if not signals_to_register:
            return
        
        self._original_handlers = {}
        for sig in signals_to_register:
            self._original_handlers[sig] = signal.signal(sig, self._signal_handler)
        
        self._signal_handlers_registered = True

    def unregister_signal_handlers(self):
        if not self._signal_handlers_registered:
            return

        for sig, handler in self._original_handlers.items():
            signal.signal(sig, handler)

        self._signal_handlers_registered = False
        self._original_handlers = {}

    def _signal_handler(self, signum, frame):
        # Prevent re-entrant handling and excessive logging
        if self._handling_signal:
            return
        self._handling_signal = True
        
        # Only log in verbose mode
        if self.verbose:
            print(f"[{self.name}] Caught signal {signum}")
        
        self.terminate_process()
        
        # Call the original handler to propagate the signal up the chain
        original = self._original_handlers.get(signum)
        if (original and callable(original) and 
            original not in (signal.SIG_IGN, signal.SIG_DFL)):
            original(signum, frame)
        elif original == signal.SIG_DFL:
            # Re-raise the signal with default behavior
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)

    # Lifecycle

    def is_running(self):
        return self._process is not None and self._process.poll() is None

    def terminate_process(self, timeout=3):
        if self._process is None:
            return False

        proc = self._process
        pid = proc.pid

        if self.verbose:
            print(f"[{self.name}] Terminating process tree rooted at PID {pid}")

        try:
            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                proc.terminate()

            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    proc.kill()
                proc.wait(timeout=1)
        finally:
            self._process = None
            return True

    # Execution

    def run(
        self,
        command,
        *,
        stdin=None,
        capture_output=False,
        print_output=True,
        silent=False,
        detached=False,
        timeout=None,
        register_signals=True,
        signals=None,
    ):
        if silent:
            print_output = False

        if register_signals:
            self.register_signal_handlers(signals=signals)

        self.cmd = command

        try:
            if detached:
                return self._run_detached(command, stdin, print_output)
            elif self.interactive_shell:
                return self._run_interactive(
                    command, stdin, capture_output, print_output, silent, timeout
                )
            else:
                return self._run_standard(
                    command, stdin, capture_output, print_output, silent, timeout
                )
        finally:
            if register_signals and not detached:
                self.unregister_signal_handlers()

    def send_stdin(self, data: str) -> bool:
        if not self.is_running():
            return False

        try:
            if self._pty_master_fd is not None:
                os.write(self._pty_master_fd, data.encode())
                return True
            elif self._process.stdin:
                self._process.stdin.write(data)
                self._process.stdin.flush()
                return True
        except Exception:
            return False

        return False

    # Detached execution with PTY

    def _run_detached(self, command, stdin, print_output):

        if self.verbose:
            print(f"Running (detached): {command}")

        master_fd, slave_fd = pty.openpty()

        self._process = subprocess.Popen(
            command,
            shell=True,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            text=True,
            preexec_fn=os.setsid,
            close_fds=True,
        )

        os.close(slave_fd)
        self._pty_master_fd = master_fd

        with self._stdout_lock:
            self._detached_stdout.clear()

        def pty_reader():
            while True:
                try:
                    data = os.read(master_fd, 1024)
                    if not data:
                        break
                    text = data.decode(errors="ignore")
                    with self._stdout_lock:
                        self._detached_stdout.append(text)
                    if print_output:
                        print(text, end="")
                except OSError:
                    break

        threading.Thread(target=pty_reader, daemon=True).start()

        if isinstance(stdin, str):
            os.write(master_fd, stdin.encode())

        return {
            "pid": self._process.pid,
            "process": self._process,
            "command": command,
            "detached": True,
            "success": None,
        }

    # Standard execution

    def _run_standard(
        self, command, stdin, capture_output, print_output, silent, timeout
    ):
        if self.verbose and not silent:
            print(f"Running: {command}")

        proc = subprocess.Popen(
            command,
            shell=True,
            text=True,
            stdin=subprocess.PIPE if isinstance(stdin, str) else stdin,
            stdout=subprocess.PIPE if (capture_output or print_output) else None,
            stderr=subprocess.PIPE if (capture_output or print_output) else None,
        )

        self._process = proc

        if isinstance(stdin, str):
            proc.stdin.write(stdin)
            proc.stdin.close()

        # If we need to both print and capture, stream the output
        if print_output and capture_output and not silent:
            stdout_lines = []
            stderr_lines = []
            
            import select
            
            # Set streams to non-blocking if available
            if proc.stdout:
                import fcntl
                flags = fcntl.fcntl(proc.stdout, fcntl.F_GETFL)
                fcntl.fcntl(proc.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            if proc.stderr:
                flags = fcntl.fcntl(proc.stderr, fcntl.F_GETFL)
                fcntl.fcntl(proc.stderr, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            
            # Stream output in real-time
            while proc.poll() is None:
                readable, _, _ = select.select([proc.stdout, proc.stderr], [], [], 0.1)
                
                for stream in readable:
                    if stream == proc.stdout:
                        line = stream.readline()
                        if line:
                            stdout_lines.append(line)
                            print(line, end='', flush=True)
                    elif stream == proc.stderr:
                        line = stream.readline()
                        if line:
                            stderr_lines.append(line)
                            print(line, end='', flush=True)
            
            # Read any remaining output
            remaining_stdout = proc.stdout.read()
            if remaining_stdout:
                stdout_lines.append(remaining_stdout)
                print(remaining_stdout, end='', flush=True)
            
            remaining_stderr = proc.stderr.read()
            if remaining_stderr:
                stderr_lines.append(remaining_stderr)
                print(remaining_stderr, end='', flush=True)
            
            stdout = ''.join(stdout_lines)
            stderr = ''.join(stderr_lines)
        else:
            # Original behavior for simple cases
            stdout, stderr = proc.communicate(timeout=timeout)
            
            if print_output and stdout and not silent:
                print(stdout, flush=True)
            if stderr and not silent:
                print(stderr, flush=True)
        
        self._process = None

        return {
            "returncode": proc.returncode,
            "stdout": stdout if capture_output else None,
            "stderr": stderr if capture_output else None,
            "success": proc.returncode == 0,
            "command": command,
        }

    def _run_interactive(
        self, command, stdin, capture_output, print_output, silent, timeout
    ):
        full_cmd = f"bash -i -c 'set +m; {command}'"

        if self.verbose and not silent:
            print(f"Running (interactive): {command}")

        self._process = subprocess.Popen(
            full_cmd,
            shell=True,
            text=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE if capture_output else None,
            stderr=subprocess.PIPE if capture_output else None,
            preexec_fn=os.setsid,
        )

        stdout, stderr = self._process.communicate(
            input=stdin if isinstance(stdin, str) else None,
            timeout=timeout,
        )

        rc = self._process.returncode
        self._process = None

        return {
            "returncode": rc,
            "stdout": stdout if capture_output else None,
            "stderr": stderr if capture_output else None,
            "success": rc == 0,
            "command": command,
        }

    # Output access

    def get_output(self):
        if self._process is None:
            return {
                "stdout": None,
                "stderr": None,
                "returncode": None,
                "running": False,
            }

        running = self._process.poll() is None

        with self._stdout_lock:
            stdout = "".join(self._detached_stdout)

        return {
            "stdout": stdout,
            "stderr": None,
            "returncode": self._process.returncode,
            "running": running,
            "command": self.cmd,
        }

    def pop_output(self) -> str:
        """
        Return and clear accumulated detached stdout.
        This is REQUIRED for streaming consumers.
        """
        with self._stdout_lock:
            out = "".join(self._detached_stdout)
            self._detached_stdout.clear()
        return out

    def close(self):
        self.terminate_process()
        self.unregister_signal_handlers()

    def __enter__(self):
        self.register_signal_handlers()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def __del__(self):
        self.close()

