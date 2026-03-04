# Setting up Zenoh with Tailscale for ROS 2 TurtleBot3

## Prerequisite

You should have a working Turtlebot3 Waffle with the correct image. If you do not have this, read the previous section. You will also need a virtual machine to run the remote environment.

You will need a VM to run on the remote PC; you can download VMWare Workstation Pro for Windows [here](https://drive.google.com/file/d/1fImMbF3tlhrv6NMppyDaYYmm7c6O-o8C/view?usp=drive_link).

Information on setting up the VM can be found [here](../sim/README.md)

## Setup Tailscale on VM and TurtleBot3

1. Create a Tailscale account using your Gmail account or GitHub
2. Install Tailscale on the VM:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

3. Start Tailscale to add the VM as a device:

```bash
sudo tailscale up
```

4. Install Tailscale on the TurtleBot3 and add it to the same Tailscale network.
5. Connect the TurtleBot3 to Tailscale and enable SSH:

```bash
sudo tailscale up --ssh
```

6. Ensure you connect using the correct username:

```bash
<username>@computername:~$
```

## Setup Zenoh on VM and TurtleBot3

1. Install Zenoh (RMW implementation) on both devices:

```bash
sudo apt update && sudo apt install ros-humble-rmw-zenoh-cpp
```

2. Confirm the installation:

```bash
ros2 doctor --report | grep rmw
```

You should see rmw_zenoh_cpp listed.

3. Initialise the ROS 2 environment and set the Zenoh middleware:

```bash
source /opt/ros/humble/setup.sh
source ~/.bashrc
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

> [!IMPORTANT]  
> If you are setting this on the AICE2011 VM, place it after the line that sources the `auto_comp.sh` file.


4. Verify middleware configuration:

```bash
ros2 doctor --report | grep middleware
```

Expected output:

```bash
middleware name: rmw_zenoh_cpp
```

A warning may appear. This is normal.

## Starting the Zenoh Middleware

One device must run as the router, the other as a client. We recommend running the router on the VM and having the robot as a client.

### On the router machine

1. Stop any existing ROS services:

```bash
pkill -9 -f ros
ros2 daemon stop
```

2. Start the Zenoh router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

3. Open a new terminal and get the router's Tailscale IP:

```bash
tailscale ip
```

### On the client machine

1. Set the Zenoh client configuration, replacing <router_tailscale_ip> with the router's Tailscale IP:

```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<router_tailscale_ip>:7447"]'
```

2. Confirm connectivity:

```bash
ros2 doctor --report | grep middleware
```

Output should show only:

```bash
middleware name: rmw_zenoh_cpp
```

## Testing with Talker–Listener

1. Verify the ROS domain ID on both devices:

```bash
echo $ROS_DOMAIN_ID
```

2. If blank or different, set them to the same value:

```bash
export ROS_DOMAIN_ID=<domain_id_integer>
```

3. On the router machine:

```bash
source /opt/ros/humble/setup.sh
source ~/.bashrc
ros2 run demo_nodes_cpp talker
```

4. On the client machine:

```bash
ros2 run demo_nodes_cpp listener
```

Messages should be received.

## SSH Into Shared Tailscale Devices

By default, if you share you robot from your Tailnet to other people, they cannot SSH into it, even if SSH is enabled. To fix this, you must add an ACL rule to allow it. 

The following needs to be done on the account of the user who 'owns the robot', i.e. it's not shared with them.

First go to Access Controls->Tags and click "create Tag". Name it something like "shared-robot" and set the owner field to your account.


Then go to Access Controls->Tailscale SSH and click "Add Rule". In the source field, add the Tailscale usernames of everyone you want to be able to SSH into your robot. Then in the Destination field, select the tag you created in the step before. For destination user, you can put the username you use when you SSH into your robot, or you can put "autogroup:nonroot, root" to allow access to any user. Leave check mode to "Off" and add save the rule. 

SSH into the shared machine should now work. If not, check you entered the usernames correctly.
