"""CPU or bare-SDK DDS data fixture; isolated topics never address robot commands."""
from pathlib import Path
import argparse
import json
import os
import sys
import time
import traceback
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--run-id',required=True)
    p.add_argument('--side',choices=('windows','linux'),required=True)
    p.add_argument('--sdk',type=Path)
    p.add_argument('--duration',type=float,default=12.)
    p.add_argument('--passive-participants',type=int,default=0)
    p.add_argument('--context-mode',choices=('per_node','shared','g2_default'),default='per_node')
    p.add_argument('--bridge-source',type=Path)
    p.add_argument('--command-suffix',action='store_true')
    p.add_argument('--native-app',action='store_true')
    p.add_argument('--paired-ready-wait',type=float,default=35.)
    p.add_argument('--cpp-publisher',type=Path)
    a=p.parse_args()
    if len(a.run_id)!=32 or not all(c in '0123456789abcdef' for c in a.run_id):
        raise ValueError('unique32hex run required')
    if not 1<=a.duration<=20 or not 0<=a.passive_participants<=40 or not 1<=a.paired_ready_wait<=120:
        raise ValueError('probe bounds invalid')
    app=None
    if a.side=='windows':
        if a.sdk is None:raise ValueError('SDK required for installed ROS bootstrap')
        os.environ['ISAAC_PATH']=str(a.sdk)
        sys.path.insert(0,str(a.bridge_source or Path(__file__).resolve().parents[2]))
        from factory_ops.isaac45_runtime import configure_ros_environment
        configure_ros_environment(int(os.environ['ROS_DOMAIN_ID']))
        prebundles=list((a.sdk/'extscache').glob('omni.kit.pip_archive-*/pip_prebundle'))
        if len(prebundles)!=1:raise ValueError('installed pip bundle ambiguous')
        sys.path.insert(0,str(prebundles[0]))
        sys.path.insert(0,str(a.sdk/'exts/isaacsim.ros2.bridge/humble/rclpy'))
        if a.native_app:
            from isaacsim import SimulationApp
            app=SimulationApp({'headless':True,'width':640,'height':360,'multi_gpu':False})
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension('isaacsim.ros2.bridge')
            for _ in range(10):app.update()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist
    report={'run_id':a.run_id,'side':a.side,'status':'starting','GPU_started':a.native_app,
            'motion_topics_used':False,'rclpy_file':rclpy.__file__,
            'DDS_profile':os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
            'domain':os.environ.get('ROS_DOMAIN_ID'),'passive_participants':a.passive_participants,
            'context_mode':a.context_mode,
            'sent':{str(i):0 for i in range(1,5)},'received':{str(i):0 for i in range(1,5)},
            'last_received_sequence':{},'participants_closed':False}
    path=a.stage/(a.side+'-result.json')
    def save():
        temp=path.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');temp.replace(path)
    save();resources=[];publishers=[];executors=[];bridge_states={};bridge_owned=[];cpp_children=[];g2=None;code=1
    try:
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        shared_context=None
        if a.context_mode=='g2_default':
            if a.side!='windows' or a.bridge_source is None:raise ValueError('frozen G2 source required')
            sys.path.insert(0,str(a.bridge_source))
            from scripts.isaac import g2_scene as g2
            from factory_ops.motion import VelocityWatchdog
            from factory_ops.isaac45_runtime import Runtime45
            shared_context=rclpy.get_default_context();rclpy.init(context=shared_context)
            report['g2_source']=str(Path(g2.__file__))
            import hashlib
            report['g2_source_sha256']=hashlib.sha256(Path(g2.__file__).read_bytes()).hexdigest()
        elif a.context_mode=='shared':
            shared_context=Context();rclpy.init(context=shared_context)
        for n in range(a.passive_participants+4):
            context=shared_context
            if context is None:
                context=Context();rclpy.init(context=context)
            i=n-a.passive_participants+1
            prefix='/probe_'+a.run_id
            if g2 is not None and n>=a.passive_participants:
                watchdog=VelocityWatchdog(max_age_s=.5,max_linear_m_s=1.,max_angular_rad_s=1.)
                node,state=g2._setup_robot_ros(rclpy,{'namespace':prefix+f'/to_windows/carter{i}'},[],None,watchdog,
                                              broadcaster_types=Runtime45.tf_broadcaster_types())
                bridge_states[str(i)]=state
            else:
                node=rclpy.create_node(f'dds_{a.side}_{n}',namespace=prefix,context=context)
            resources.append((context,node))
            if n<a.passive_participants:continue
            send='to_linux' if a.side=='windows' else 'to_windows'
            receive='to_windows' if a.side=='windows' else 'to_linux'
            suffix='/cmd_vel' if a.command_suffix else ''
            send=prefix+f'/{send}/carter{i}'+suffix
            receive=prefix+f'/{receive}/carter{i}'+suffix
            publisher=None if a.side=='linux' and a.cpp_publisher else node.create_publisher(Twist,send,qos)
            def callback(msg,i=i):
                if msg.linear.y!=float(i) or msg.angular.x!=12345.:
                    raise ValueError('probe identity differs')
                report['received'][str(i)]+=1
                report['last_received_sequence'][str(i)]=msg.linear.z
            node.create_subscription(Twist,receive,callback,qos)
            if g2 is None:
                executor=SingleThreadedExecutor(context=context);executor.add_node(node)
                executors.append(executor)
            publishers.append((i,publisher,node,receive))
        if g2 is not None:
            bridge_owned=g2._create_node_executors([node for context,node in resources])
        if a.side=='linux' and a.cpp_publisher:
            if not a.command_suffix:raise ValueError('C++ Nav2 publisher requires cmd_vel suffix')
            for i in range(1,5):
                cpp_children.append(subprocess.Popen([str(a.cpp_publisher),str(i),str(a.stage),'/probe_'+a.run_id,str(a.duration)]))
            report['publisher_source']='Installed nav2_util::TwistPublisher in four owned C++ processes; no Python forward publishers.'
        def spin():
            if g2 is not None:g2._spin_nodes(rclpy,[node for context,node in resources])
            else:
                for ex in executors:ex.spin_once(timeout_sec=0.)
            if app is not None:app.update()
        report.update(status='ready',participant_count=len({id(c) for c,n in resources}),node_count=len(resources));save()
        ready=a.stage/(a.side+'-ready.json');ready.write_text(json.dumps({'run_id':a.run_id})+'\n',encoding='utf-8')
        deadline=time.monotonic()+a.paired_ready_wait
        while not (a.stage/'start.json').exists():
            spin()
            if time.monotonic()>deadline:raise TimeoutError('paired start did not arrive')
            time.sleep(.005)
        assert json.loads((a.stage/'start.json').read_text(encoding='utf-8'))['run_id']==a.run_id
        end=time.monotonic()+a.duration;next_send=0.;next_save=0.
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_send:
                for i,pub,node,receive in publishers:
                    if pub is None:continue
                    report['sent'][str(i)]+=1;msg=Twist();msg.linear.z=float(report['sent'][str(i)])
                    msg.linear.y=float(i);msg.angular.x=12345.;pub.publish(msg)
                next_send=now+.05
            spin()
            if now>=next_save:report['status']='sampling';save();next_save=now+1.
            time.sleep(.002)
        report['graph_publishers']={str(i):node.count_publishers(receive) for i,pub,node,receive in publishers}
        if cpp_children:
            for child in cpp_children:
                if child.wait(timeout=3)!=0:raise RuntimeError('C++ publisher failed')
            report['cpp_publishers']={str(i):json.loads((a.stage/f'cpp-carter{i}.json').read_text(encoding='utf-8')) for i in range(1,5)}
            report['sent']={i:v['sent'] for i,v in report['cpp_publishers'].items()}
        report['status']='passed' if all(v>=10 for v in report['received'].values()) else 'failed_delivery'
        if bridge_states:
            report['g2_received']={i:s['cmd_vel'] for i,s in bridge_states.items()}
            report['g2_rejected']={i:s['cmd_rejected'] for i,s in bridge_states.items()}
            report['g2_scope']='Actual frozen callback/executor with authored0 watchdog clock and zero velocity; no clock/actuator or physical evidence.'
            if not all(v>=10 for v in report['g2_received'].values()):report['status']='failed_g2_delivery'
        code=0 if report['status']=='passed' else 1
    except BaseException as error:
        report.update(status='failed',error=f'{type(error).__name__}: {error}',traceback=traceback.format_exc())
    finally:
        errors=[]
        for child in cpp_children:
            if child.poll() is None:
                try:child.terminate();child.wait(timeout=3)
                except BaseException as e:errors.append(str(e))
        if bridge_owned:
            try:g2._shutdown_node_executors(bridge_owned)
            except BaseException as e:errors.append(str(e))
        for ex in executors:
            try:ex.shutdown()
            except BaseException as e:errors.append(str(e))
        closed_contexts={}
        for context,node in reversed(resources):
            try:
                node.destroy_node()
                closed_contexts[id(context)]=context
            except BaseException as e:errors.append(str(e))
        for context in closed_contexts.values():
            try:rclpy.shutdown(context=context)
            except BaseException as e:errors.append(str(e))
        report.update(participants_closed=not errors,cleanup_errors=errors,intended_exit_code=code)
        save()
        if app is not None:app.close(wait_for_replicator=False)
    return code


if __name__=='__main__':raise SystemExit(main())
