# Setup Troubleshooting

This guide covers common issues you may encounter when setting up and using the TurtleBot 3, along with their solutions.

## 0. Check Environment Variables Are Exported

Before troubleshooting more complex issues, verify that all required environment variables are set in your current shell:

```bash
# Check ROS 2 environment
echo $ROS_DISTRO
printenv | grep -i ros

# Check TurtleBot 3 model
echo $TURTLEBOT3_MODEL

# Check other relevant variables
printenv | grep -i zenoh
printenv | grep -i rmw
```

If any of these variables are empty or unset, export them as needed. For example:

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=42
export ZENOH_ROUTER_CHECK=true
```

### Manual Exports vs .bashrc

> [!WARNING]
> Running `export` commands manually in a terminal only sets variables for that specific shell session. Once you close the terminal, these settings are lost. To make environment variables persist across sessions, add them to your shell's configuration file (e.g., `.bashrc` for Bash, `.zshrc` for Zsh, or a script in `/etc/profile.d/` for system-wide settings). These files are automatically sourced when you open a new terminal or start a login session, ensuring your environment is always configured correctly.

`.bashrc` and `.zshrc` are scripts that run automatically each time you start a new interactive shell, while `/etc/profile.d/` contains system-wide scripts that are sourced during login. Adding your exports to one of these locations ensures they are available in every new terminal without needing to run the export commands manually each time.

## 2. Old ROS Processes

Stale ROS processes can cause conflicts and unexpected behaviour. If you experience issues with nodes not starting or communicating, try terminating any existing ROS processes:

```bash
pkill -9 -f ros
ros2 daemon stop
```

The `pkill` command forcefully terminates all processes matching "ros", while `ros2 daemon stop` stops the ROS 2 daemon. After running these commands, restart your ROS 2 launch files.

## 3. Environment Not Sourced

If commands like `ros2` or `ros2 launch` are not recognised, your ROS 2 environment may not be sourced. Add the following to your `.bashrc` file:

```bash
source /opt/ros/<distro>/setup.bash
```

Replace `<distro>` with your ROS 2 distribution (e.g., `humble` or `jazzy`). Then reload your terminal or run `source ~/.bashrc`.

## 4. Zenoh Environment Variables Not Exported

Zenoh is used for communication in some configurations. Ensure the Zenoh environment variables are set:

```bash
export ZENOH_ROUTER_CHECK=true
```

Add this to your `.bashrc` if you need Zenoh functionality.

## 5. Incorrect Tailscale IP Address

If you are using Tailscale for network connectivity, ensure you are using the correct IP address:

```bash
tailscale ip
```

Verify that the IP matches what is configured in your setup. If you are connecting to a remote device, double-check that both machines are on the same Tailscale network.

## 6. Middleware Errors

Middleware issues can prevent nodes from communicating. Check your current middleware configuration:

```bash
ros2 doctor --report | grep middleware
```

This displays the middleware currently in use. Common middleware options include CycloneDDS and FastRTPS. If you encounter issues, try switching middleware by setting the `RMW_IMPLEMENTATION` environment variable:

```bash
export RMW_IMPLEMENTATION=rmw_cyclone_dpp_cpp
# or
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 8. Talker–Listener Issues

If the talker–listener demo is not working, verify that:

1. Both machines are on the same network
2. The `ROS_DOMAIN_ID` matches on both machines
3. Firewall settings allow ROS 2 traffic on ports 7400-7402 and 11811

Test with:

```bash
export ROS_DOMAIN_ID=<your-domain-id>
ros2 run demo_nodes_py listener
```

## 9. Timestamp Errors

If you encounter errors related to timestamps on received data, this is usually caused by clock synchronisation issues between machines.

### Checking Clock Status

First, check the clock status on both machines:

```bash
timedatectl status
```

Look for the following in the output:

- `System clock synchronised: yes`
- `NTP service: active`

If either of these shows `no` or `inactive`, clock synchronisation is not enabled.

### Enabling Clock Synchronisation

If the clock is not synchronised, enable NTP:

```bash
sudo timedatectl set-np true
```

Then verify the status again:

```bash
timedatectl status
```

Wait a moment for the synchronisation to complete. Timestamp errors should resolve once both machines have synchronised clocks.


## 10. Camera "Request Cancelled" Error

If you receive a "Request cancelled" error when using the camera, ensure the `TURTLEBOT3_MODEL` environment variable is set correctly:

```bash
export TURTLEBOT3_MODEL=burger
# or
export TURTLEBOT3_MODEL=waffle
```

Add this to your `.bashrc` file to ensure it is set on every terminal session.

---

## Additional Tips

- **Check ROS 2 logs**: Use `ros2 log` commands to view detailed logging information
- **Verify network connectivity**: Use `ping` to test connectivity between machines
- **Restart services**: Sometimes simply restarting the affected nodes resolves the issue
- **Consult official documentation**: Refer to the [ROS 2 documentation](https://docs.ros.org/) for distribution-specific guidance
