# Archive

Code kept for reference only. Nothing here is installed (`setup.py` doesn't
package this folder), launched or imported, and it is not maintained against
the current software.

| File | What it was | Why it's archived |
|---|---|---|
| `tools/august_bridge.py` | Ruth's August/October nav bridge: VN-100 attitude + DVL velocity dead-reckoned into one ODOMETRY stream at 20 Hz. It was used for pool tests while GUIDED worked (Oct 2026). | It talks to MAVProxy on its own UDP port (`udpout:127.0.0.1:14551`). Since Nov 10 that port belongs to `telemetry_gateway`, so the two would fight. The live bridge is `api/navigation/fused_nav_bridge.py`. |

To bring something back, port it to the gateway first: a ROS `Link(..., node=node, kinds=(...))`
with a component that `api/pixhawk/gateway_core.py` admits for what it sends.
