# Scripts

These scripts are for physical TurtleBots. The simulation itself runs in the AICE2011 container and needs no host-side setup scripts.

- `tb3_poweroff.py` — shut down one or more TurtleBots over Tailscale SSH. It supports dry runs, username auto-detection, and passwordless key-based SSH.
- `terminal.py` — PTY/subprocess helper used by `tb3_poweroff.py`.

Run the shutdown tool with Python 3:

```bash
python3 sim/scripts/tb3_poweroff.py -d      # list targets without shutting down
python3 sim/scripts/tb3_poweroff.py 1 2 3   # shut down tb1, tb2 and tb3
```

Tailscale must be running on the machine executing the script, and the robots must be online and reachable. See [Networking](../../docs/20_networking.md) for the Fast DDS over Tailscale configuration and [the simulation README](../README.md) for the overall environment.
