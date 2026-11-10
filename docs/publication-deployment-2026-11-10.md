# Publication and deployment preparation — November 10, 2026

User authorized pushing all pending local changes online and to GRAEY. Includes the compact Crusader-based GUI, Cube/QGC pose display, Sensors preparation controls, pending heading-reset orientation-cache fix, existing continuous QGC broadcast helper, and local navigation/SITL documentation and tools. GUI GUIDED routes and native QGC AUTO remain separate. Shared OCS changes belong to rx26_ocs, not a GRAEY-only fork.

Validation before publication: 466 GRAEY unittest cases (5 platform skips), 611 shared OCS cases (1 skip), compact GUI synthetic browser workflow passes with no JavaScript errors. Added regression checks that a heading reset rotates the orientation hold/heading exactly once and QGC replies do not redirect subnet broadcast telemetry. SITL demonstration scripts were syntax checked; no new native SITL mission or physical vehicle mission was run for this publication.

Live pre-deployment read: controller fresh, disarmed MANUAL, SA active, ESC command off, no mission claimed. Existing live parameters/model files will be preserved; release activation requires the same fresh inhibited state. This is a preparation record, not a claim that activation has completed. Final activation evidence is recorded separately.
