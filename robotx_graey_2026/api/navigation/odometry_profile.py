"""External-only navigation profile. Raw GPS is telemetry, never an EKF source."""
PROFILE = {'AHRS_EKF_TYPE': 3, 'EK3_ENABLE': 1, 'GPS_TYPE': 1, 'GPS_TYPE2': 0,
           'GPS_AUTO_SWITCH': 0, 'GPS_PRIMARY': 0, 'EK3_SRC_OPTIONS': 0, 'VISO_TYPE': 1,
           'EK3_OGN_HGT_MASK': 0}
for slot in (1, 2, 3):
    for axis, value in dict(POSXY=6, VELXY=6, POSZ=6, VELZ=0, YAW=6).items():
        PROFILE[f'EK3_SRC{slot}_{axis}'] = value
