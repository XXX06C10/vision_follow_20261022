from pathlib import Path
root=Path(__file__).resolve().parent
parts=['enable_latch.v','encoder_ab_counter.v','vision_move_core.v','vision_servo_core.v','vision_result_guard.v','vision_follow_control.v','vision_cdc_bridge.v',
       'vision_config_rx.v','vision_center_key.v','vision_shadow_target.v','vision_report_tx.v','vision_monitor.v',
       'vision_value_cdc.v','vision_camera_control.v']
body='\n'.join('// Source: '+name+'\n'+(root/'rtl'/name).read_text(encoding='utf-8') for name in parts)
dest=root/'rtl/vision_monitor_all.v'
if not dest.exists() or dest.read_text(encoding='utf-8')!=body:dest.write_text(body,encoding='utf-8')
print('Composed vision_monitor_all.v')
