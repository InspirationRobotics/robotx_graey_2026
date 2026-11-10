"""Serializable competition route: buoys annotate, paired endpoints execute."""
import math
from .waypoint_logic import geo_to_ne


def validate_route(value):
    if not isinstance(value, dict) or not isinstance(value.get('points'), list):
        raise ValueError('An ordered list of route points is required')
    if not 1<=len(value['points'])<=256:raise ValueError('Use 1–256 route points')
    recording_mode=value.get('recording_mode','automatic')
    if recording_mode not in ('automatic','manual'):raise ValueError('Unknown recording mode')
    points = []
    pending = None
    for index, item in enumerate(value['points']):
        if not isinstance(item, dict):
            raise ValueError('Invalid route point')
        kind = item.get('kind')
        if kind not in ('waypoint', 'pipeline_start', 'pipeline_end', 'pipeline_buoy'):
            raise ValueError('Unknown route point type')
        lat, lon = item.get('lat'), item.get('lon')
        geo_to_ne(lat, lon, (0., 0.))
        point = dict(kind=kind, lat=float(lat), lon=float(lon), display_order=index+1,
                     name=str(item.get('name', f'Point {index+1}'))[:80])
        if kind != 'pipeline_buoy':
            if pending is not None:
                if kind != 'pipeline_end':
                    raise ValueError('Pipeline start must be followed by pipeline end (buoys may be between them)')
                if math.hypot(*geo_to_ne(lat, lon, (pending['lat'], pending['lon']))) < .01:
                    raise ValueError('Pipeline start and end must be different positions')
                pending = None
            elif kind == 'pipeline_end':
                raise ValueError('Pipeline end needs a preceding pipeline start')
            elif kind == 'pipeline_start':
                pending = point
        points.append(point)
    if pending is not None:
        raise ValueError('Pipeline start is missing its end')
    executable = [p for p in points if p['kind'] != 'pipeline_buoy']
    if not executable:
        raise ValueError('Add at least one waypoint or pipeline segment')
    has_pipeline = any(p['kind'] == 'pipeline_start' for p in points)
    # User-approved behavior: surface at the end; recording failures are nonfatal.
    end = value.get('pipeline_end_action', 'surface')
    failure = 'finish_segment'
    if has_pipeline and end != 'surface':
        raise ValueError('Pipeline ends surface before the next waypoint')
    result = dict(mode='surface', map='geographic', depth_m=0.,
                lat=executable[0]['lat'], lon=executable[0]['lon'],
                points=points, pipeline_depth_m=4., pipeline_end_action=end,
                recording_failure_action=failure)
    if 'recording_mode' in value:result['recording_mode']=recording_mode
    return result
