# Setup Troubleshooting

This guide covers common issues you may encounter when setting up and using the TurtleBot3, along with their solutions.

## 1. Check Environment Variables Are Exported

Before troubleshooting more complex issues, verify that all required environment variables are set in your current shell:

```bash
# Check ROS 2 environment
echo $ROS_DISTRO
printenv | grep -i ros

# Check TurtleBot3 model
echo $TURTLEBOT3_MODEL

# Check middleware and Fast DDS profile
printenv | grep -i rmw
printenv | grep -i fastrtps
```

Expected values:

```bash
ROS_DISTRO=jazzy
TURTLEBOT3_MODEL=waffle_pi
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/<user>/fastdds_tailscale.xml
```

If any of these variables are empty or wrong, export them as needed:

```bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_tailscale.xml"
```

### Manual Exports vs .bashrc

> [!WARNING]
> Running `export` commands manually in a terminal only sets variables for that specific shell session. Once you close the terminal, these settings are lost. To make environment variables persist across sessions, add them to your shell's configuration file (e.g. `.bashrc` for Bash or `.zshrc` for Zsh), or to a script in `/etc/profile.d/` for system-wide settings.

`.bashrc` and `.zshrc` are scripts that run automatically each time you start a new interactive shell, while `/etc/profile.d/` contains system-wide scripts that are sourced during login. Adding your exports to one of these locations ensures they are available in every new terminal.

## 2. Old ROS Processes

Stale ROS processes can cause conflicts and unexpected behaviour. If nodes do not start or cannot communicate, terminate any existing ROS processes:

```bash
pkill -9 -f ros
ros2 daemon stop
```

In the container, the `tb3_kill_all` helper also stops Gazebo and every ROS 2 node, then restarts the ROS 2 daemon.

## 3. Environment Not Sourced

If commands like `ros2` or `ros2 launch` are not recognised, source the Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
```

Then reload your terminal or run `source ~/.bashrc`. In the container, `/opt/ros/jazzy/setup.bash` is sourced automatically for interactive shells.

## 4. Fast DDS Profile Not Found or Ignored

Physical-robot networking uses the Fast DDS profile described in [Networking](20_networking.md). Check that the path exists:

```bash
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
ls -l "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

Check that the placeholder has been replaced:

```bash
grep PEER_TAILSCALE_IP "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

If that prints a line, the file still contains the placeholder and Fast DDS cannot use it as a peer address. Replace it with the other device's Tailscale IPv4 address.

## 5. Incorrect Tailscale IP Address

If you are using Tailscale, ensure you are using the correct IPv4 address:

```bash
tailscale ip -4
tailscale status
```

Check that the IP in your profile is the **other** device's Tailscale address, not your own. A robot's profile points at the container host; the container's profile points at the robot.

## 6. Middleware Errors

Middleware issues can prevent nodes from communicating. Check your current middleware configuration:

```bash
ros2 doctor --report | grep -i middleware
```

Fast DDS is the default in Jazzy. This is the expected value for the lab:

```text
middleware name: rmw_fastrtps_cpp
```

If a previous session exported Zenoh or Cyclone DDS, start a fresh shell or correct the environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_ROUTER_CHECK
unset ZENOH_CONFIG_OVERRIDE
unset ROS_LOCALHOST_ONLY
```

## 7. Talker–Listener Issues

If the talker–listener demo is not working, verify that:

1. Both machines are on the same Tailscale network.
2. The `ROS_DOMAIN_ID` matches on both machines.
3. Both machines use `FASTRTPS_DEFAULT_PROFILES_FILE` and point it at the other machine's Tailscale IP.
4. Both machines have replaced `PEER_TAILSCALE_IP`.
5. Firewall or ACL settings allow UDP ports `7400-7420` between the two devices.

Test with:

```bash
export ROS_DOMAIN_ID=<your-domain-id>
ros2 run demo_nodes_py listener
```

On the other machine:

```bash
ros2 run demo_nodes_cpp talker
```

Remember that SOTON-IoT does not support multicast, so both ends must use the unicast profile.

## 8. Timestamp Errors

Errors related to timestamps on received data are usually caused by clock synchronisation issues between machines.

Check the clock status on both machines:

```bash
timedatectl status
```

Look for:

- `System clock synchronized: yes`
- `NTP service: active`

If either shows `no` or `inactive`, enable NTP:

```bash
sudo timedatectl set-ntp true
```

Then verify the status again:

```bash
timedatectl status
```

Wait a moment for synchronisation to complete. Timestamp errors should resolve once both machines have synchronised clocks.

## 9. Camera "Request Cancelled" Error

If you receive a "Request cancelled" error when using the camera, ensure the model is set correctly. The lab uses the Waffle Pi:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

Add this to your `.bashrc` on the robot if it is not already set.

## 10. Gazebo Sim Problems

The lab uses **Gazebo Sim Harmonic**, not Gazebo Classic. There are no `gzserver` or `gzclient` commands; use `gz sim`.

**Gazebo starts but `/scan` never appears**

Gazebo's LIDAR needs working rendering. Check the renderer:

```bash
glxinfo -B | grep -i renderer
```

You are looking for a renderer containing `llvmpipe`. If it says `swrast` or reports an error, use software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

Then restart the world.

**A model cannot be found**

Gazebo Sim resolves `model://` URIs through `GZ_SIM_RESOURCE_PATH`, not the old Gazebo Classic `GAZEBO_MODEL_PATH`:

```bash
echo "$GZ_SIM_RESOURCE_PATH"
```

The TurtleBot3 models should be under `/opt/ros/jazzy/share/turtlebot3_gazebo/models`. The `aice_sim` launch file adds its own `models/` directory automatically.

**Stale Gazebo processes**

Stop them with the container helper:

```bash
tb3_kill_all
```

Or find them directly:

```bash
pgrep -af 'gz sim|gz-sim'
```

**The simulation is slow**

Gazebo runs on the CPU in the container. Close the Gazebo GUI and use the base world when you do not need the 3D view. A real-time factor below `1.0` is expected.

---

## Additional Tips

- **Check ROS 2 logs**: use `ros2 topic`, `ros2 node`, and the launch terminal output.
- **Verify network connectivity**: use `tailscale ping` and `ping` to test connectivity.
- **Restart services**: sometimes restarting the affected nodes resolves the issue.
- **Consult official documentation**: refer to the [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/) and [Gazebo Sim documentation](https://gazebosim.org/docs/harmonic/).
