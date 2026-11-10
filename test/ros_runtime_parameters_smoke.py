"""Actual ROS parameter callbacks; synthetic telemetry and mocked hardware owners."""
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
os.environ.update(ROS_DOMAIN_ID='125',ROS_LOCALHOST_ONLY='1',GRAEY_RUNTIME_SCOPE='runtime-parameters-smoke')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from robotx_graey_2026.api.navigation import dvl_node,fused_nav_bridge
    from robotx_graey_2026.api.pixhawk import kill_switch
    rclpy.init();nodes=[];checks=[]
    try:
        with patch.object(dvl_node.DVLNode,'spin_once',lambda self:None):
            dvl=dvl_node.DVLNode();nodes.append(dvl)
        p=lambda name,value:Parameter(name,value=value)
        assert not dvl.set_parameters([p('scale',1.1)])[0].successful
        dvl.runtime_status(String(data=json.dumps(dict(published_monotonic=time.monotonic(),vehicle_fresh=True,armed=False,mission_claimed=False))))
        assert dvl.set_parameters([p('scale',1.1)])[0].successful
        dvl.pub_vel=Mock();dvl.handle_report(json.dumps(dict(vx=1.,vy=2.,vz=0.,velocity_valid=True)).encode())
        assert abs(dvl.pub_vel.publish.call_args.args[0].twist.twist.linear.x-1.1)<1e-9
        assert not dvl.set_parameters([p('scale',-1.)])[0].successful and dvl.scale==1.1
        dvl.runtime_control['armed']=True
        assert not dvl.set_parameters([p('scale',1.2)])[0].successful
        checks.append('DVL runtime scale changes actual published velocity; invalid/armed/stale rejected')
        with patch.object(fused_nav_bridge,'Link'):
            nav=fused_nav_bridge.FusedNavBridge();nodes.append(nav)
        nav.control_state=dict(vehicle_fresh=True,armed=False,mission_claimed=False);nav.control_received=time.monotonic()
        nav.output_permitted=False;nav.est.fault='maintenance retained'
        assert nav.set_parameters([p('water_density',1001.)])[0].successful
        assert nav.settings['water_density']==1001. and not nav.output_permitted and nav.est.fault=='maintenance retained'
        nav.output_permitted=True
        assert not nav.set_parameters([p('water_density',1002.)])[0].successful
        checks.append('Navigation density applies while paused; output/fault stay unchanged; running updates rejected')
        def probe(*args):return SimpleNamespace(interval=args[2],timeout=args[3],close=lambda:None,snapshot=lambda:dict(alive=False,age=None))
        with patch.object(kill_switch,'Link'),patch.object(kill_switch,'TetherProbe',probe):
            controller=kill_switch.KillSwitch();nodes.append(controller)
        controller.vehicle_t=time.monotonic();controller.armed=False;controller.link.reset_mock()
        inhibit,revision=controller.inhibit,controller.revision
        assert controller.set_parameters([p('tether_ping_interval_s',1.5)])[0].successful
        assert controller.probe.interval==1.5 and not controller.esc_commanded_on
        assert controller.inhibit==inhibit and controller.revision==revision and not controller.link.method_calls
        controller.armed=True
        assert not controller.set_parameters([p('tether_timeout_s',4.)])[0].successful
        assert not controller.set_parameters([p('relay_servo',10)])[0].successful
        checks.append('Watchdog timing updates actual probe; inhibit/recovery/ESC unchanged; armed/wiring rejected')
        print(json.dumps(dict(result='PASS',ros_domain=125,checks=checks,hardware_opened=False),indent=2))
    finally:
        for node in reversed(nodes):node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
