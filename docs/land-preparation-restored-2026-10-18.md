# Land north preparation restored — October 18, 2026

Latest user instruction restores the separate land VN/north-capture workflow from
5e6b9be. Restored bridge service, GUI jobs/page and tests, plus land_north module.
Offline Miramar image cache/default layer and HTTP routes remain unchanged.
Surface CHECK_SURFACE hold and descent protections remain unchanged.

Sequence: calibrate VN if needed first, then capture an independently known true
bearing on stationary level land under kill/disarm/ESC-off. Keep VN powered and
continuous while transferring to water. Select Use captured land north and Prepare
at the floating surface for pressure baseline and navigation commissioning.
Land capture does not calibrate water depth, require DVL/GPS, or enable propulsion.
Restart, calibration or VN continuity loss invalidates the session reference.

Validation:254Python tests and both calibration/waypoint JS suites passed. Existing
surface hold/mission implementation unchanged; no redundant SITL rerun.

Wi-Fi attempt: graey.local did not resolve; previous address192.168.8.105 timed out.
User asked for Wi-Fi if possible. No vehicle changes or deployment occurred in this
restoration session. Last verified installed vehicle revision remains2effc53.