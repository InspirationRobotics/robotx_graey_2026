"""Candidate continuous fusion configuration; never writes vehicle parameters."""
PROFILE = {'GPS_TYPE': 1, 'GPS_TYPE2': 14, 'GPS_AUTO_SWITCH': 0,
           'GPS_PRIMARY': 1, 'EK3_SRC_OPTIONS': 0}
# Identical slots prevent an old selection command changing the fusion policy.
for slot in (1, 2, 3):
    for axis, value in dict(POSXY=3, VELXY=6, POSZ=1, VELZ=0, YAW=6).items():
        PROFILE[f'EK3_SRC{slot}_{axis}'] = value
