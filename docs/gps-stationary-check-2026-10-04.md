# Stationary GPS and DVL comparison — October 4, 2026

## Setup and safety

User confirmed Graey disarmed. At capture start the controller API reported MANUAL,
fresh heartbeat, ESC off and no mission. Capture ran 179.7 seconds and recorded 308
samples from `/api/navigation`, with the controller checked on every sample. All
308 samples remained disarmed. No commands, EKF source changes or parameter writes
were sent. Output: `output/stationary-gps-20261004/telemetry-dvl-lock.jsonl`.

## Results

- GPS: 308/308 samples valid, fix type 4, about 22–24 satellites, reported
  horizontal accuracy about 1.28–1.35 m. Coordinate range was 1.34 m north by
  0.90 m east. This is temporal scatter, not absolute accuracy.
- Cube global: 308/308 valid; coordinate range 0.12 m north by 0.27 m east.
- Raw GPS to Cube global separation: mean 7.08 m; range 6.48–7.63 m.
- Cube local position: 308/308 fresh. EKF flags stayed 831.
- DVL: 307/308 valid; mean bottom clearance about 1.10 m; speed near zero.
- VectorNav heading stream: 308 fresh samples, observed range 112.87–114.50 deg.
- Supervisor stayed Fault / Observation: “Navigation bridge restarted; external
  frame must be revalidated”. Source configuration was not verified and no source
  switching occurred.

Recorded parameter snapshot still shows EK3 source set 1 POSXY/VELXY/YAW=6
(ExternalNav) and source set 2 POSXY/VELXY/YAW=0 (None). This has not been
independently read back from the live Cube in this test. Thus this capture does
not establish that GPS was being fused by the active EKF. Stable Cube global/local
coordinates and a steady ~7 m offset from raw GPS suggest a frame or reference
mismatch is possible, but an independent surveyed location and fresh Cube
parameter readback are needed before attributing the cause.

ROS log inspection showed repeated VN-100 serial read failures/reopens and more
than one running DVL/bridge process. Streams were fresh during this capture, so
those process/log observations are not proof of the sensor being invalid in this
window. Investigate duplicate process ownership and serial access before dynamic
fusion testing. No `.BIN` dataflash file was found in the Jetson/container file
search; request/download the relevant Pixhawk log from QGC or MAVLink FTP to
inspect XKF innovations and source-selection messages.

At end, Graey remained disarmed but SA_KILL inhibit was active. Do not clear it as
part of GPS evaluation. Next evidence steps: live readback EK3 source parameters,
resolve bridge frame fault with a reviewed procedure, confirm one owner per sensor
node and stable VN/DVL sampling, acquire Pixhawk DataFlash log, then compare GPS
against an independently surveyed fixed point. Do not tune EK3_POSNE_M_NSE or
innovation gates from this stationary scatter alone.
