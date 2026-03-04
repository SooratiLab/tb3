# Flashing the TurtleBot3 Pi

## Outline

This tool utilises Packer and Podman to automate the creation of Raspberry Pi 4 images for TurtleBot3 robots. The tool can be found [here](https://github.com/tgodfrey0/turtlebot3_custom_image). _Disclaimer:_ this system was designed on Linux, YMMV on Windows or MacOS but most instructions are transferable.

A pre-made image for the TB3 Burger (LDS-03, PiCamera (`libcamera` stack)) can be found [here](https://drive.google.com/drive/folders/1bvYBT5LfiV7HOOFlRQBDTyHcVLiAmQ3g?usp=sharing). Make sure to follow the post install instructions once you have flashed the MicroSD card. For this pre-built image, the username is `tb` and the password is `password`; you should change the password to something else. The image is built with the University of Southampton's IoT network as a placeholder; after booting, edit the file `/etc/netplan/50-cloud-init.yaml` and input your network details, then run `sudo netplan try`. You can change the `ROS_DOMAIN_ID` in `/etc/profile.d/90-turtlebot-ros-profile.sh`, then reboot.

## Prerequisites
- *Python 3.11+*
- *Podman* (Containerised Packer execution)
- *Dependencies*: `pip install -r requirements.txt`

## Usage Pipeline

### Configuration
Customise the build via TOML files in `configs/`. Multiple WiFi networks can be defined for robust connectivity.

```toml
[model]
type = "waffle" # or "burger"

[ros]
domain_id = 42

[[network]]
ssid = "Swarm_Lab_5G"
password = "secure_password"
```

See the `configs/example.toml` to view a complete example.

> [!CAUTION]
> It is not recommended to change the ISO that the config points to and the target image size (unless adding new packages/files).

### Build Process
Run the build script to generate the .img (or compressed .img.xz) file.

```python
python build.py --config configs/my_robot.toml --verbose
```

The build may take over 1 hour to build and customise the image.

### Deployment<deploy>
Flash the image to a MicroSD card. Ensure you identify the correct device node using `fdisk -l`.

```bash
xz -dc <IMAGE>.img.xz | sudo dd of=/dev/<DEVICE> status=progress
```

After flashing, expand the main partition to fill the MicroSD card. I'd recommend a tool like GParted to expand it to fill the entire MicroSD card.

## Post-Installation<post>

- Log in with the credentials used in your config file
- First Boot: The system automatically configures the hostname (based on MAC address), OpenCR firmware, and Pi Camera. A reboot is required after the initialisation. *Do not reboot immediately*---wait a few minutes for each process to finish.
- Use the system as normal.

## Key Automated Features

- ROS2 Humble Hawksbill & TurtleBot3 stack installation.
- OpenCR firmware configuration via boot-time service.
- MAC-based Hostname: Automatically sets turtlebot_XX_XX_XX.
- `libcamera` setup for Pi Camera stack
- Netplan configuration for multiple pre-defined SSIDs.

### Automatic Bringup

This image has a service to automatically run `ros2 launch turtlebot3_bringup robot.launch.py`. To enable this, run the following.

```bash
systemctl enable bringup.service # Run on boot
systemctl start bringup.service # Run now
systemctl status bringup.service # Check it launched with no errors
```
