"""Software current sensitivity model; not a hydrodynamic or measured lake model."""
import json
from pathlib import Path
from test.test_guided_geographic import simulate
from robotx_graey_2026.api.navigation.geographic_frame import rotation
import math
results=[]
for speed in (.05,.10,.20):
    run,e,p,stages=simulate(offset=40.,current=(0.,speed))
    row=dict(current_local_xy_mps=[0.,speed],stage=run.stage,reason=run.reason,
             heading_offset_deg=e.map.angle,alignment_m=run.alignment_distance,
             physical_target_error_m=math.dist(rotation(p,40.),(10.,3.)),stages=stages)
    results.append(row);print(json.dumps(row),flush=True)
Path('docs/current-scenarios-2026-10-24.json').write_text(json.dumps(dict(
    model='Assumed cross current; first-order horizontal drag, pilot thrust/1000 m/s, 1 s time constant; synthetic GPS/DVL/VN; no wave or heave model',
    measured_miramar_conditions=False,results=results),indent=2)+'\n')
