"""The FCU gateway decodes once and consumers receive only declared kinds."""
import json
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robotx_graey_2026.api.pixhawk.ros_link import (
    FCU_RX_KINDS, RosTransport, SemanticMessage, mav_topic)


class SemanticTransportTests(unittest.TestCase):
    def fake_ros(self):
        string_type=type('String',(),{'__init__':lambda self,data='':setattr(self,'data',data)})
        module=Mock(String=string_type)
        return string_type,patch.dict(sys.modules,{'std_msgs':Mock(msg=module),'std_msgs.msg':module})

    def test_topic_names_are_bounded_and_no_raw_receive_topic_exists(self):
        self.assertEqual(mav_topic('LOCAL_POSITION_NED'),'/graey/fcu/mav/local_position_ned')
        with self.assertRaises(ValueError):mav_topic('UNUSED_MESSAGE')
        self.assertNotIn('/graey/fcu/rx',{mav_topic(kind) for kind in FCU_RX_KINDS})

    def test_consumer_subscribes_only_to_declared_kinds_plus_heartbeat(self):
        _,context=self.fake_ros()
        with context:
            node=Mock()
            RosTransport(node,195,255,rx_depth=200,kinds=('ATTITUDE',))
        topics={call.args[1] for call in node.create_subscription.call_args_list}
        self.assertEqual(topics,{mav_topic('ATTITUDE'),mav_topic('HEARTBEAT')})
        depths={call.args[1]:call.args[3] for call in node.create_subscription.call_args_list}
        self.assertEqual(depths[mav_topic('ATTITUDE')],1)
        self.assertEqual(depths[mav_topic('HEARTBEAT')],1)

    def test_undeclared_broad_receive_is_rejected(self):
        _,context=self.fake_ros()
        with context:
            with self.assertRaisesRegex(ValueError,'declare'):
                RosTransport(Mock(),195,255)

    def test_semantic_message_preserves_fields_source_sequence_and_flightmode(self):
        String,context=self.fake_ros()
        with context:
            node=Mock()
            transport=RosTransport(node,195,255,kinds=('ATTITUDE',))
            payload=dict(received_monotonic=time.monotonic(),kind='HEARTBEAT',
                source_system=3,source_component=1,sequence=77,flightmode='GUIDED',
                fields=dict(mavpackettype='HEARTBEAT',base_mode=129,custom_mode=4))
            transport.receive(String(data=json.dumps(payload)))
        seen=[];transport.drain(lambda kind,message:seen.append((kind,message)))
        kind,message=seen[0]
        self.assertEqual((kind,message.base_mode,message.get_srcSystem(),
                          message.get_srcComponent(),message.get_seq()),
                         ('HEARTBEAT',129,3,1,77))
        self.assertEqual(transport.mav.flightmode,'GUIDED')

    def test_replaceable_stream_keeps_newest_but_tunnel_keeps_order(self):
        String,context=self.fake_ros()
        with context:
            transport=RosTransport(Mock(),195,255,kinds=('ATTITUDE','TUNNEL'))
            def payload(kind,sequence):
                return String(data=json.dumps(dict(received_monotonic=time.monotonic(),kind=kind,
                    source_system=3,source_component=1,sequence=sequence,fields={})))
            transport.receive_kind('ATTITUDE',payload('ATTITUDE',1))
            transport.receive_kind('ATTITUDE',payload('ATTITUDE',2))
            transport.receive_kind('TUNNEL',payload('TUNNEL',3))
            transport.receive_kind('TUNNEL',payload('TUNNEL',4))
        seen=[];transport.drain(lambda kind,message:seen.append((kind,message.get_seq())))
        self.assertEqual(seen,[('TUNNEL',3),('TUNNEL',4),('ATTITUDE',2)])

    def test_message_view_supports_tunnel_payload_fields(self):
        message=SemanticMessage('TUNNEL',43,191,4,dict(
            target_system=3,target_component=201,payload_length=3,payload=[1,2,3]))
        self.assertEqual(bytes(message.payload)[:message.payload_length],b'\x01\x02\x03')

    def test_gui_uses_bounded_sensor_snapshots_not_duplicate_full_rate_topics(self):
        root=Path(__file__).parents[1]/'robotx_graey_2026/api'
        gui=(root/'gui/gui_node.py').read_text(encoding='utf-8')
        vn=(root/'navigation/vn100_node.py').read_text(encoding='utf-8')
        dvl=(root/'navigation/dvl_node.py').read_text(encoding='utf-8')
        self.assertIn("'/graey/vn100/gui'",gui)
        self.assertIn("'/graey/dvl/sample'",gui)
        for topic in ('/graey/vn100/imu','/graey/vn100/heading',
                      '/graey/dvl/velocity','/graey/dvl/valid','/graey/dvl/altitude'):
            self.assertNotIn(topic,gui)
        self.assertIn("'/graey/vn100/gui'",vn)
        self.assertIn('received_monotonic=time.monotonic()',vn)
        self.assertIn('altitude=altitude',dvl)
        self.assertIn('received_monotonic=time.monotonic()',dvl)

    def test_gui_uses_crusader_cadence_navigation_projection(self):
        root=Path(__file__).parents[1]/'robotx_graey_2026/api'
        gui=(root/'gui/gui_node.py').read_text(encoding='utf-8')
        supervisor=(root/'navigation/navigation_supervisor.py').read_text(encoding='utf-8')
        self.assertIn("'/graey/navigation/gui_snapshot'",gui)
        self.assertNotIn("create_subscription(String, '/graey/navigation/bridge_status'",gui)
        self.assertIn("create_timer(.5,self.publish_gui)",supervisor)
        self.assertIn("'/graey/navigation/gui_snapshot'",supervisor)


if __name__=='__main__':unittest.main()
