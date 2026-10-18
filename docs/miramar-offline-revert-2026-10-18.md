# Restore previous preparation and provide saved Miramar imagery

User requested reverting land north capture and restoring the previous combined
Prepare/north-alignment workflow. Restored calibration page, CalibrationJobs,
FusedNavBridge and their regression tests from 244f264; removed land_north module
and its tests. User can choose GPS/DVL surface movement or a known stationary
true bearing during floating-surface Prepare, as before. Prepare still requires
the correct pressure baseline and safety/health checks. No land reference survives
the navigation service restart.

Retained CHECK_SURFACE armed horizontal hold, upward-only surface velocity,
0.08 m pressure abort, single continuous ExternalNav architecture and DVL recovery.
No live parameter changes, compass enabling, arming or mission execution requested.

User reported global tiles not working. Packaged 58 public Esri World Imagery
JPEGs as static local images: 3x3 level16 lake-overview tiles and 7x7 level19 tiles
around 32.9137076,-117.1001865 (last live GPS near the dock; not surveyed location).
The default Miramar layer loads these images directly from Jetson, retaining
their original Web Mercator tile georeferencing. This avoids browser internet,
screenshots with hidden geographic offsets, and invented calibration points.
It uses the same local-image delivery principle as the pool reference but retains
provider coordinates. Offline does not imply geographically exact shoreline.

Use Miramar overview to browse, then Fit map / vehicle for local waypoint selection.
The dock detail region is roughly 450 m across. Outside saved coverage, imagery
is explicitly unavailable; no online download is silently required. Online current
and archived pool layers remain optional. Waypoint distance limit remains 20 m.
Local image cache is attributed to Esri, Vantor, Earthstar Geographics and GIS
User Community; manifest contains source URLs, capture date and hashes. Acquisition
date of the aerial photographs is unknown. The user's perspective photograph is
not used as a measured navigation reference.

Validation: calibration and waypoint JS suites pass, including offline detail and
overview URLs and clicking a geographic point; Python suite checks every saved
tile hash, explicit HTTP allowlisting and JPEG MIME type alongside existing
mission/bridge regressions. Final test and deployment results recorded below.

Connection: graey.local did not resolve and old Wi-Fi192.168.8.105 timed out.
User then explicitly authorized tether. Fresh read-only tether confirmed killed,
disarmed and ESC-off, tracked release5e6b9be and failed land-capture job, no DVL lock.
Deployment requires another immediate safety check and preserves untracked files.

Local validation complete: 248 Python tests passed; both GUI suites passed. Visually inspected central saved dock tile and confirmed dock/shore imagery rather than a blank or service-error image. Mission controller unchanged from previously validated CHECK_SURFACE release; no redundant SITL rerun for this presentation/workflow revert.
