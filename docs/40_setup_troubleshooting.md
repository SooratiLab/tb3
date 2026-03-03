# Setup Troubleshooting

## Troubleshooting Common Issues

The following items can cause issues during use.

1. Old ROS processes:

```bash
pkill -9 -f ros
ros2 daemon stop
```

2. Environment not sourced.
3. Zenoh environment variables not exported.
4. Incorrect Tailscale IP:

```bash
tailscale ip
```

5. Middleware errors:

```bash
ros2 doctor --report | grep middleware
```

6. Talker–Listener issues.
7. Error treating timestamp for received data.

First check clock status on both:

```bash
timedatectl status
```

Look for:
System clock synchronised: yes
NTP service: active

If not synchronised, enable it:

```bash
sudo timedatectl set-ntp true
```

Then verify again:

```bash
timedatectl status
```

8. `Request cancelled` when using the camera

Make sure you have the `TURTLEBOT3_MODEL` variable set.
