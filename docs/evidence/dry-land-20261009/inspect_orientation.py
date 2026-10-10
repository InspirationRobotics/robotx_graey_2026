import subprocess
code = r'''
import inspect,json,hashlib,time,math
import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from robotx_graey_2026.api.navigation import vn100_node,nav_ekf_bridge
for mod,cls,methods in [(vn100_node,'VN100Node',['parse']),
                        (nav_ekf_bridge,'NavEKFBridge',['on_imu','send_odom'])]:
    path=inspect.getfile(mod)
    print('INSTALLED_MODULE',path,hashlib.sha256(open(path,'rb').read()).hexdigest())
    for name in methods: print(inspect.getsource(getattr(getattr(mod,cls),name)))
    if hasattr(mod,'yaw_offset_quat'): print(inspect.getsource(mod.yaw_offset_quat))
rclpy.init(); node=rclpy.create_node('graey_readonly_orientation_check')
samples={}
def imu(m):
    q=m.orientation
    yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))%360
    samples['imu']=dict(yaw=yaw,quaternion=[q.w,q.x,q.y,q.z],frame=m.header.frame_id)
node.create_subscription(Imu,'/graey/vn100/imu',imu,10)
node.create_subscription(String,'/graey/navigation/bridge_status',lambda m:samples.update(bridge=json.loads(m.data)),10)
for target,names in [('vn100_node',['flip_180','yaw_offset_deg','port','baud']),
                     ('nav_ekf_bridge',['yaw_offset_deg','allow_alignment'])]:
    client=node.create_client(GetParameters,'/'+target+'/get_parameters')
    if not client.wait_for_service(timeout_sec=3):
        print('PARAM_SERVICE_UNAVAILABLE',target); continue
    request=GetParameters.Request(); request.names=names
    f=client.call_async(request); rclpy.spin_until_future_complete(node,f,timeout_sec=4)
    if f.done() and f.result():
        print('ROS_PARAMETERS',target,json.dumps({n:dict(type=v.type,bool=v.bool_value,integer=v.integer_value,double=v.double_value,string=v.string_value) for n,v in zip(names,f.result().values)}))
    else: print('PARAM_TIMEOUT',target)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
print('ROS_SAMPLES',json.dumps(samples));node.destroy_node();rclpy.shutdown()
'''
r=subprocess.run(['docker','exec','-i','graey','bash','-lc',
    'source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && python3 -'],
    input=code,text=True,capture_output=True,timeout=35)
print(r.stdout,r.stderr,flush=True)
raise SystemExit(r.returncode)
