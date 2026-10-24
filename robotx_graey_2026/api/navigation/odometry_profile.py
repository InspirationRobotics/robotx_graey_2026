"""External-only navigation profile. Raw GPS is telemetry, never an EKF source."""
import os

FIRMWARE_PROFILE=os.environ.get('GRAEY_FIRMWARE_PROFILE','4.5')
if FIRMWARE_PROFILE not in ('4.5','4.7'):
    raise ValueError('Unsupported navigation firmware profile')
PROFILE = {'AHRS_EKF_TYPE': 3, 'AHRS_OPTIONS': 0, 'EK3_ENABLE': 1, 'GPS_TYPE': 1, 'GPS_TYPE2': 0,
           'GPS_AUTO_SWITCH': 0, 'GPS_PRIMARY': 0, 'EK3_SRC_OPTIONS': 0, 'VISO_TYPE': 1,
           'EK3_OGN_HGT_MASK': 0}
for slot in (1, 2, 3):
    for axis, value in dict(POSXY=6, VELXY=6, POSZ=6, VELZ=0, YAW=6).items():
        PROFILE[f'EK3_SRC{slot}_{axis}'] = value

SHALLOW_LIMITS=dict(WPNAV_SPEED_DN=10.,WPNAV_SPEED_UP=15.,WPNAV_ACCEL_Z=50.)
if FIRMWARE_PROFILE=='4.7':
    PROFILE['GPS1_TYPE']=PROFILE.pop('GPS_TYPE')
    PROFILE['GPS2_TYPE']=PROFILE.pop('GPS_TYPE2')
    SHALLOW_LIMITS=dict(WP_SPD_DN=.10,WP_SPD_UP=.15,WP_ACC_Z=.50)
