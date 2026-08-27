# Getting started

Install the client:

```bash
pip install zero-franky
```

Install the server extra on the control machine:

```bash
pip install 'zero-franky[server]'
zero-franky server --host 0.0.0.0 --port 18812
```

Connect from the application machine:

```python
from zero_franky import Robot, setup_zero_franky

setup_zero_franky("control-machine", 18812)
robot = Robot("192.168.100.1")
```

The server uses three transports:

| Port | Direction | Purpose |
|---:|---|---|
| `18812` | request/reply | Commands and queries. |
| `18813` | publish/subscribe | Robot state broadcasts. |
| `18814` | push/pull | High-rate tracker references; stale updates may be dropped. |

The state and tracker ports are the RPC port plus one and two. `0.0.0.0` listens on all interfaces; use `--host` and firewall rules to restrict access.
