# Networking a TurtleBot3 with Fast DDS over Tailscale

ROS 2 Jazzy uses Fast DDS as its default middleware. On a normal network it discovers other machines with multicast, but the University's **SOTON-IoT** network does not carry multicast traffic. The fix is to keep the default Fast DDS middleware and give it a configuration file that:

- disables multicast discovery; and
- lists the other machine's Tailscale IP address as a unicast discovery endpoint.

This replaces the old Zenoh setup. You do **not** need to install Zenoh.

## Container networking

The supplied `compose.yaml` publishes only the noVNC port and uses Docker's default bridge network. That is enough for simulation, but it does not automatically give the container the host's Tailscale interface, and it does not forward the UDP ports Fast DDS uses.

For physical-robot sessions, the container must be able to reach the robot over Tailscale and the robot must be able to reach the container's host. On Linux the usual approach is Tailscale on the host plus host networking for the container. That is a change to the container setup, not to this repository. If you cannot run the container that way, run the Jazzy environment natively on Ubuntu 24.04 for hardware sessions.

## 1. Set up Tailscale

Do this on the robot and on the computer that runs the container. If Tailscale is installed *inside* the container instead of on the host, the container needs `/dev/net/tun` and `NET_ADMIN`.

1. Create a Tailscale account using your University account, Gmail, or GitHub.
2. Install Tailscale:

   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   ```

3. Add the robot to your tailnet and enable SSH:

   ```bash
   sudo tailscale up --ssh
   ```

4. Add the container host to the same tailnet:

   ```bash
   sudo tailscale up
   ```

5. Find both Tailscale IPv4 addresses:

   ```bash
   tailscale ip -4
   ```

6. Check that the two devices can reach each other:

   ```bash
   tailscale ping <other_device_tailscale_name_or_ip>
   ```

## 2. Create the Fast DDS profile

The profile is in this repository at [`config/fastdds_tailscale.xml`](../config/fastdds_tailscale.xml).

Copy it to the robot and to the container host (or into the container if that is where you run ROS 2). On **each** machine, replace `PEER_TAILSCALE_IP` with the **other** machine's Tailscale IPv4 address:

| Machine | `PEER_TAILSCALE_IP` |
|---|---|
| Robot | Tailscale IP of the container host |
| Container | Tailscale IP of the robot |

For example, on the robot:

```bash
nano ~/fastdds_tailscale.xml
```

Change:

```xml
<address>PEER_TAILSCALE_IP</address>
```

to:

```xml
<address>100.x.y.z</address>
```

The empty locator below disables multicast discovery. Leave it in place:

```xml
<metatrafficUnicastLocatorList>
  <locator/>
</metatrafficUnicastLocatorList>
```

> [!IMPORTANT]
> Both ends must use the profile. If only one end disables multicast, discovery will be unreliable on SOTON-IoT.

## 3. Configure each ROS 2 shell

Run the following on the robot and in the container. Fast DDS is the default RMW in Jazzy; setting `RMW_IMPLEMENTATION` explicitly prevents a leftover Zenoh or Cyclone DDS setting from a previous session being used.

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_tailscale.xml"
export ROS_DOMAIN_ID=42
```

Use the same `ROS_DOMAIN_ID` on both machines. If you already set a different domain ID in your robot image, use that value instead.

If you previously used Zenoh, remove its settings from the current shell:

```bash
unset ZENOH_ROUTER_CHECK
unset ZENOH_CONFIG_OVERRIDE
unset ROS_LOCALHOST_ONLY
```

Verify the middleware:

```bash
ros2 doctor --report | grep -i middleware
```

Expected output:

```text
middleware name: rmw_fastrtps_cpp
```

## 4. Test discovery

On one machine:

```bash
ros2 run demo_nodes_cpp talker
```

On the other:

```bash
ros2 run demo_nodes_py listener
```

The listener should print the messages from the talker.

If nothing is received, try the test in the other direction. Discovery can be asymmetric if one side is still using a stale profile or domain ID.

## 5. Check the robot from the container

With the robot bringup running:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

You should see `/scan`, `/odom`, `/imu`, `/camera/image_raw`, and `/cmd_vel`. On Jazzy, `/cmd_vel` is a `geometry_msgs/msg/TwistStamped`:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}}}"
```

## Troubleshooting

**`ros2 doctor` shows the wrong middleware**

Check that `RMW_IMPLEMENTATION` is `rmw_fastrtps_cpp` and that no old Zenoh settings are still exported. Start a fresh terminal after editing your shell configuration.

**The profile is ignored**

Check the path:

```bash
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
ls -l "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

The file must be valid XML and must contain the replaced address, not `PEER_TAILSCALE_IP`.

**No discovery between the container and the robot**

- Confirm both devices are online: `tailscale status`.
- Confirm both use the same `ROS_DOMAIN_ID`.
- Confirm both have the other device's Tailscale IP in their profile.
- Check that UDP ports `7400-7420` are allowed between the two devices.
- Stop stale ROS 2 daemons and try again:

  ```bash
  ros2 daemon stop
  pkill -9 -f ros
  ```

**It works on a normal network but not on SOTON-IoT**

SOTON-IoT drops multicast. Both sides must use the supplied profile so that discovery is unicast only.

## SSH into shared Tailscale devices

By default, if you share your robot from your tailnet to other people, they cannot SSH into it, even if SSH is enabled. To fix this, you must add an ACL rule to allow it.

The following needs to be done on the account of the user who **owns** the robot, i.e. it is not shared with them.

First go to **Access Controls → Tags** and click **Create Tag**. Name it something like `shared-robot` and set the owner field to your account. Then in the **Machines** tab, select the robot and click **Assign ACL Tags**, then add the tag you just created.

Then go to **Access Controls → Tailscale SSH** and click **Add Rule**. In the source field, add the Tailscale usernames of everyone you want to be able to SSH into your robot. Then in the destination field, select the tag you created in the previous step. For destination user, you can put the username you use when you SSH into your robot, or you can put `autogroup:nonroot, root` to allow access to any user. Leave check mode to **Off** and save the rule.

You may also need to add a rule to allow the tagged robot to reach your other devices. Go to **Access Controls → General Access Rules**. Add a rule with the source as your tag, and the destination as `autogroup:members`. Allow all ports and protocols and click save.

SSH into the shared machine should now work. If not, check that you entered the usernames correctly.
