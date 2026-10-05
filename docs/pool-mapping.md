# Pool map setup — October 4, 2026

The Planner page (/planner) uses the bundled pool image with local Cube NED tracking.
The lab image has an approximate 5 m scale bar from image pixel (75,96) to (695,96),
so its default scale is 5/620 m/pixel. This is not surveyed geographic registration.
The neighbor image still requires its own measured scale. Calibration, start,
heading and marker are saved per image in the current browser; live zero is never
restored across sessions or carried between pools.

Select the correct pool, verify its scale with a measured distance, click the
physical start point and facing direction, then connect tracking and zero only
with fresh position while Graey is at that point. Do not infer an accurate pool
GPS anchor from a screenshot scale alone. A GPS anchor and image north alignment
remain unverified. Planner mission buttons run the existing prequalification
mission; they are not generic GPS waypoint navigation.

The position feed is /pos on port 8081 and requires pos_server. Freshness now
requires separate recent position and attitude samples. This is not an EKF health
or mission permission check. A stale feed disables zeroing and pauses the marker.
SITL must use its own feed, never the physical vehicle feed or mixed QGC stream.
