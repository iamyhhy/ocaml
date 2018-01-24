void __deserialize_LandStateType(enum LandStateType *ptr , int fd ) ;
void __deserialize___anonstruct_rtl_path_9(struct __anonstruct_rtl_path_9 *ptr , int fd ) ;
void __serialize_ThrowModeStage(enum ThrowModeStage *ptr , int fd ) ;
void __deserialize_scalarInt16(short *ptr , int fd ) ;
void __serialize___anonstruct_throw_state_10(struct __anonstruct_throw_state_10 *ptr ,
                                             int fd ) ;
void __serialize___anonstruct_gndeffect_state_11(struct __anonstruct_gndeffect_state_11 *ptr ,
                                                 int fd ) ;
void __serialize_control_mode_t(enum control_mode_t *ptr , int fd ) ;
void __deserialize_scalarFloat32(float *ptr , int fd ) ;
void __deserialize___anonstruct_sensor_health_7(struct __anonstruct_sensor_health_7 *ptr ,
                                                int fd ) ;
void __serialize___anonunion_ap_2(union __anonunion_ap_2 *ptr , int fd ) ;
void __deserialize___anonstruct_throw_state_10(struct __anonstruct_throw_state_10 *ptr ,
                                               int fd ) ;
void __serialize___anonstruct____missing_field_name_3(struct __anonstruct____missing_field_name_3 *ptr ,
                                                      int fd ) ;
void __deserialize_RTLState(enum RTLState *ptr , int fd ) ;
void __serialize_scalarInt8(unsigned char *ptr , int fd ) ;
void __deserialize_ThrowModeStage(enum ThrowModeStage *ptr , int fd ) ;
void __deserialize___anonstruct_gndeffect_state_11(struct __anonstruct_gndeffect_state_11 *ptr ,
                                                   int fd ) ;
void __serialize_Copter(struct Copter *ptr , int fd ) ;
void __deserialize___anonstruct_failsafe_6(struct __anonstruct_failsafe_6 *ptr , int fd ) ;
void __deserialize_GuidedMode(enum GuidedMode *ptr , int fd ) ;
void __serialize_LandStateType(enum LandStateType *ptr , int fd ) ;
void __serialize___anonstruct_takeoff_state_5(struct __anonstruct_takeoff_state_5 *ptr ,
                                              int fd ) ;
void __serialize_mode_reason_t(enum mode_reason_t *ptr , int fd ) ;
void __serialize___anonstruct_failsafe_6(struct __anonstruct_failsafe_6 *ptr , int fd ) ;
void __deserialize_control_mode_t(enum control_mode_t *ptr , int fd ) ;
void __deserialize_Copter(struct Copter *ptr , int fd ) ;
void __deserialize_scalarInt32(int *ptr , int fd ) ;
void __serialize_scalarFloat32(float *ptr , int fd ) ;
void __deserialize_mode_reason_t(enum mode_reason_t *ptr , int fd ) ;
void __deserialize_AutoMode(enum AutoMode *ptr , int fd ) ;
void __serialize___anonstruct_sensor_health_7(struct __anonstruct_sensor_health_7 *ptr ,
                                              int fd ) ;
void __serialize_scalarInt32(int *ptr , int fd ) ;
void __deserialize_scalarInt8(unsigned char *ptr , int fd ) ;
void __serialize___anonstruct_rtl_path_9(struct __anonstruct_rtl_path_9 *ptr , int fd ) ;
void __serialize_PayloadPlaceStateType(enum PayloadPlaceStateType *ptr , int fd ) ;
void __deserialize___anonstruct____missing_field_name_3(struct __anonstruct____missing_field_name_3 *ptr ,
                                                        int fd ) ;
void __serialize___anonstruct_rangefinder_state_1(struct __anonstruct_rangefinder_state_1 *ptr ,
                                                  int fd ) ;
void __serialize___anonstruct_control_switch_state_4(struct __anonstruct_control_switch_state_4 *ptr ,
                                                     int fd ) ;
void __serialize_scalarInt16(short *ptr , int fd ) ;
void __serialize___anonstruct_nav_payload_place_8(struct __anonstruct_nav_payload_place_8 *ptr ,
                                                  int fd ) ;
void __deserialize___anonstruct_control_switch_state_4(struct __anonstruct_control_switch_state_4 *ptr ,
                                                       int fd ) ;
void __deserialize_PayloadPlaceStateType(enum PayloadPlaceStateType *ptr , int fd ) ;
void __deserialize___anonstruct_nav_payload_place_8(struct __anonstruct_nav_payload_place_8 *ptr ,
                                                    int fd ) ;
void __serialize_GuidedMode(enum GuidedMode *ptr , int fd ) ;
void __serialize_AutoMode(enum AutoMode *ptr , int fd ) ;
void __deserialize___anonunion_ap_2(union __anonunion_ap_2 *ptr , int fd ) ;
void __deserialize___anonstruct_takeoff_state_5(struct __anonstruct_takeoff_state_5 *ptr ,
                                                int fd ) ;
void __serialize_RTLState(enum RTLState *ptr , int fd ) ;
void __deserialize___anonstruct_rangefinder_state_1(struct __anonstruct_rangefinder_state_1 *ptr ,
                                                    int fd ) ;
void __deserialize_LandStateType(enum LandStateType *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize___anonstruct_rtl_path_9(struct __anonstruct_rtl_path_9 *ptr , int fd ) 
{ 


  {

}
}
void __serialize_ThrowModeStage(enum ThrowModeStage *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize_scalarInt16(short *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 2);
}
}
void __serialize___anonstruct_throw_state_10(struct __anonstruct_throw_state_10 *ptr ,
                                             int fd ) 
{ 


  {

}
}
void __serialize___anonstruct_gndeffect_state_11(struct __anonstruct_gndeffect_state_11 *ptr ,
                                                 int fd ) 
{ 


  {

}
}
void __serialize_control_mode_t(enum control_mode_t *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize_scalarFloat32(float *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize___anonstruct_sensor_health_7(struct __anonstruct_sensor_health_7 *ptr ,
                                                int fd ) 
{ 


  {

}
}
void __serialize___anonunion_ap_2(union __anonunion_ap_2 *ptr , int fd ) 
{ 


  {

}
}
void __deserialize___anonstruct_throw_state_10(struct __anonstruct_throw_state_10 *ptr ,
                                               int fd ) 
{ 


  {

}
}
void __serialize___anonstruct____missing_field_name_3(struct __anonstruct____missing_field_name_3 *ptr ,
                                                      int fd ) 
{ 


  {

}
}
void __deserialize_RTLState(enum RTLState *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __serialize_scalarInt8(unsigned char *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 1);
}
}
void __deserialize_ThrowModeStage(enum ThrowModeStage *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize___anonstruct_gndeffect_state_11(struct __anonstruct_gndeffect_state_11 *ptr ,
                                                   int fd ) 
{ 


  {

}
}
void __serialize_Copter(struct Copter *ptr , int fd ) 
{ 


  {
  __serialize_scalarInt8(& ptr->command_ack_counter, fd);
  __serialize_scalarInt32(& ptr->in_log_download, fd);
  __serialize___anonstruct_rangefinder_state_1(& ptr->rangefinder_state, fd);
  __serialize_scalarFloat32(& ptr->ekfGndSpdLimit, fd);
  __serialize_scalarFloat32(& ptr->ekfNavVelGainScaler, fd);
  __serialize_scalarInt32(& ptr->ekfYawReset_ms, fd);
  __serialize_scalarInt8(& ptr->ekf_primary_core, fd);
  __serialize___anonunion_ap_2(& ptr->ap, fd);
  __serialize_control_mode_t(& ptr->control_mode, fd);
  __serialize_mode_reason_t(& ptr->control_mode_reason, fd);
  __serialize_control_mode_t(& ptr->prev_control_mode, fd);
  __serialize_mode_reason_t(& ptr->prev_control_mode_reason, fd);
  __serialize___anonstruct_control_switch_state_4(& ptr->control_switch_state, fd);
  __serialize___anonstruct_takeoff_state_5(& ptr->takeoff_state, fd);
  __serialize_scalarFloat32(& ptr->auto_takeoff_no_nav_alt_cm, fd);
  __serialize_scalarInt8(& ptr->receiver_rssi, fd);
  __serialize___anonstruct_failsafe_6(& ptr->failsafe, fd);
  __serialize___anonstruct_sensor_health_7(& ptr->sensor_health, fd);
  __serialize_scalarFloat32(& ptr->scaleLongDown, fd);
  __serialize_scalarInt32(& ptr->wp_bearing, fd);
  __serialize_scalarInt32(& ptr->home_bearing, fd);
  __serialize_scalarInt32(& ptr->home_distance, fd);
  __serialize_scalarInt32(& ptr->wp_distance, fd);
  __serialize_LandStateType(& ptr->land_state, fd);
  __serialize___anonstruct_nav_payload_place_8(& ptr->nav_payload_place, fd);
  __serialize_AutoMode(& ptr->auto_mode, fd);
  __serialize_GuidedMode(& ptr->guided_mode, fd);
  __serialize_RTLState(& ptr->rtl_state, fd);
  __serialize_scalarInt8(& ptr->rtl_state_complete, fd);
  __serialize___anonstruct_rtl_path_9(& ptr->rtl_path, fd);
  __serialize_scalarInt8(& ptr->circle_pilot_yaw_override, fd);
  __serialize_scalarFloat32(& ptr->simple_cos_yaw, fd);
  __serialize_scalarFloat32(& ptr->simple_sin_yaw, fd);
  __serialize_scalarInt32(& ptr->super_simple_last_bearing, fd);
  __serialize_scalarFloat32(& ptr->super_simple_cos_yaw, fd);
  __serialize_scalarFloat32(& ptr->super_simple_sin_yaw, fd);
  __serialize_scalarInt32(& ptr->initial_armed_bearing, fd);
  __serialize_scalarInt16(& ptr->loiter_time_max, fd);
  __serialize_scalarInt32(& ptr->loiter_time, fd);
  __serialize_scalarInt32(& ptr->brake_timeout_start, fd);
  __serialize_scalarInt32(& ptr->brake_timeout_ms, fd);
  __serialize_scalarInt32(& ptr->nav_delay_time_max, fd);
  __serialize_scalarInt32(& ptr->nav_delay_time_start, fd);
  __serialize___anonstruct_throw_state_10(& ptr->throw_state, fd);
  __serialize_scalarInt32(& ptr->control_sensors_present, fd);
  __serialize_scalarInt32(& ptr->control_sensors_enabled, fd);
  __serialize_scalarInt32(& ptr->control_sensors_health, fd);
  __serialize_scalarInt16(& ptr->climb_rate, fd);
  __serialize_scalarFloat32(& ptr->target_rangefinder_alt, fd);
  __serialize_scalarInt32(& ptr->baro_alt, fd);
  __serialize_scalarFloat32(& ptr->baro_climbrate, fd);
  __serialize_scalarInt8(& ptr->auto_yaw_mode, fd);
  __serialize_scalarFloat32(& ptr->yaw_look_at_WP_bearing, fd);
  __serialize_scalarInt32(& ptr->yaw_look_at_heading, fd);
  __serialize_scalarInt16(& ptr->yaw_look_at_heading_slew, fd);
  __serialize_scalarFloat32(& ptr->yaw_look_ahead_bearing, fd);
  __serialize_scalarInt32(& ptr->condition_value, fd);
  __serialize_scalarInt32(& ptr->condition_start, fd);
  __serialize_scalarFloat32(& ptr->G_Dt, fd);
  __serialize_scalarInt16(& ptr->pmTest1, fd);
  __serialize_scalarInt32(& ptr->fast_loopTimer, fd);
  __serialize_scalarInt16(& ptr->mainLoop_count, fd);
  __serialize_scalarInt32(& ptr->rtl_loiter_start_time, fd);
  __serialize_scalarInt32(& ptr->arm_time_ms, fd);
  __serialize_scalarInt8(& ptr->auto_trim_counter, fd);
  __serialize_scalarInt8(& ptr->in_mavlink_delay, fd);
  __serialize_scalarInt8(& ptr->gcs_out_of_time, fd);
  __serialize_scalarInt32(& ptr->last_radio_update_ms, fd);
  __serialize___anonstruct_gndeffect_state_11(& ptr->gndeffect_state, fd);
  __serialize_scalarInt8(& ptr->upgrading_frame_params, fd);
}
}
void __deserialize___anonstruct_failsafe_6(struct __anonstruct_failsafe_6 *ptr , int fd ) 
{ 


  {

}
}
void __deserialize_GuidedMode(enum GuidedMode *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __serialize_LandStateType(enum LandStateType *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __serialize___anonstruct_takeoff_state_5(struct __anonstruct_takeoff_state_5 *ptr ,
                                              int fd ) 
{ 


  {

}
}
void __serialize_mode_reason_t(enum mode_reason_t *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __serialize___anonstruct_failsafe_6(struct __anonstruct_failsafe_6 *ptr , int fd ) 
{ 


  {

}
}
void __deserialize_control_mode_t(enum control_mode_t *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize_Copter(struct Copter *ptr , int fd ) 
{ 


  {
  __deserialize_scalarInt8(& ptr->command_ack_counter, fd);
  __deserialize_scalarInt32(& ptr->in_log_download, fd);
  __deserialize___anonstruct_rangefinder_state_1(& ptr->rangefinder_state, fd);
  __deserialize_scalarFloat32(& ptr->ekfGndSpdLimit, fd);
  __deserialize_scalarFloat32(& ptr->ekfNavVelGainScaler, fd);
  __deserialize_scalarInt32(& ptr->ekfYawReset_ms, fd);
  __deserialize_scalarInt8(& ptr->ekf_primary_core, fd);
  __deserialize___anonunion_ap_2(& ptr->ap, fd);
  __deserialize_control_mode_t(& ptr->control_mode, fd);
  __deserialize_mode_reason_t(& ptr->control_mode_reason, fd);
  __deserialize_control_mode_t(& ptr->prev_control_mode, fd);
  __deserialize_mode_reason_t(& ptr->prev_control_mode_reason, fd);
  __deserialize___anonstruct_control_switch_state_4(& ptr->control_switch_state, fd);
  __deserialize___anonstruct_takeoff_state_5(& ptr->takeoff_state, fd);
  __deserialize_scalarFloat32(& ptr->auto_takeoff_no_nav_alt_cm, fd);
  __deserialize_scalarInt8(& ptr->receiver_rssi, fd);
  __deserialize___anonstruct_failsafe_6(& ptr->failsafe, fd);
  __deserialize___anonstruct_sensor_health_7(& ptr->sensor_health, fd);
  __deserialize_scalarFloat32(& ptr->scaleLongDown, fd);
  __deserialize_scalarInt32(& ptr->wp_bearing, fd);
  __deserialize_scalarInt32(& ptr->home_bearing, fd);
  __deserialize_scalarInt32(& ptr->home_distance, fd);
  __deserialize_scalarInt32(& ptr->wp_distance, fd);
  __deserialize_LandStateType(& ptr->land_state, fd);
  __deserialize___anonstruct_nav_payload_place_8(& ptr->nav_payload_place, fd);
  __deserialize_AutoMode(& ptr->auto_mode, fd);
  __deserialize_GuidedMode(& ptr->guided_mode, fd);
  __deserialize_RTLState(& ptr->rtl_state, fd);
  __deserialize_scalarInt8(& ptr->rtl_state_complete, fd);
  __deserialize___anonstruct_rtl_path_9(& ptr->rtl_path, fd);
  __deserialize_scalarInt8(& ptr->circle_pilot_yaw_override, fd);
  __deserialize_scalarFloat32(& ptr->simple_cos_yaw, fd);
  __deserialize_scalarFloat32(& ptr->simple_sin_yaw, fd);
  __deserialize_scalarInt32(& ptr->super_simple_last_bearing, fd);
  __deserialize_scalarFloat32(& ptr->super_simple_cos_yaw, fd);
  __deserialize_scalarFloat32(& ptr->super_simple_sin_yaw, fd);
  __deserialize_scalarInt32(& ptr->initial_armed_bearing, fd);
  __deserialize_scalarInt16(& ptr->loiter_time_max, fd);
  __deserialize_scalarInt32(& ptr->loiter_time, fd);
  __deserialize_scalarInt32(& ptr->brake_timeout_start, fd);
  __deserialize_scalarInt32(& ptr->brake_timeout_ms, fd);
  __deserialize_scalarInt32(& ptr->nav_delay_time_max, fd);
  __deserialize_scalarInt32(& ptr->nav_delay_time_start, fd);
  __deserialize___anonstruct_throw_state_10(& ptr->throw_state, fd);
  __deserialize_scalarInt32(& ptr->control_sensors_present, fd);
  __deserialize_scalarInt32(& ptr->control_sensors_enabled, fd);
  __deserialize_scalarInt32(& ptr->control_sensors_health, fd);
  __deserialize_scalarInt16(& ptr->climb_rate, fd);
  __deserialize_scalarFloat32(& ptr->target_rangefinder_alt, fd);
  __deserialize_scalarInt32(& ptr->baro_alt, fd);
  __deserialize_scalarFloat32(& ptr->baro_climbrate, fd);
  __deserialize_scalarInt8(& ptr->auto_yaw_mode, fd);
  __deserialize_scalarFloat32(& ptr->yaw_look_at_WP_bearing, fd);
  __deserialize_scalarInt32(& ptr->yaw_look_at_heading, fd);
  __deserialize_scalarInt16(& ptr->yaw_look_at_heading_slew, fd);
  __deserialize_scalarFloat32(& ptr->yaw_look_ahead_bearing, fd);
  __deserialize_scalarInt32(& ptr->condition_value, fd);
  __deserialize_scalarInt32(& ptr->condition_start, fd);
  __deserialize_scalarFloat32(& ptr->G_Dt, fd);
  __deserialize_scalarInt16(& ptr->pmTest1, fd);
  __deserialize_scalarInt32(& ptr->fast_loopTimer, fd);
  __deserialize_scalarInt16(& ptr->mainLoop_count, fd);
  __deserialize_scalarInt32(& ptr->rtl_loiter_start_time, fd);
  __deserialize_scalarInt32(& ptr->arm_time_ms, fd);
  __deserialize_scalarInt8(& ptr->auto_trim_counter, fd);
  __deserialize_scalarInt8(& ptr->in_mavlink_delay, fd);
  __deserialize_scalarInt8(& ptr->gcs_out_of_time, fd);
  __deserialize_scalarInt32(& ptr->last_radio_update_ms, fd);
  __deserialize___anonstruct_gndeffect_state_11(& ptr->gndeffect_state, fd);
  __deserialize_scalarInt8(& ptr->upgrading_frame_params, fd);
}
}
void __deserialize_scalarInt32(int *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __serialize_scalarFloat32(float *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize_mode_reason_t(enum mode_reason_t *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize_AutoMode(enum AutoMode *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __serialize___anonstruct_sensor_health_7(struct __anonstruct_sensor_health_7 *ptr ,
                                              int fd ) 
{ 


  {

}
}
void __serialize_scalarInt32(int *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize_scalarInt8(unsigned char *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 1);
}
}
void __serialize___anonstruct_rtl_path_9(struct __anonstruct_rtl_path_9 *ptr , int fd ) 
{ 


  {

}
}
void __serialize_PayloadPlaceStateType(enum PayloadPlaceStateType *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize___anonstruct____missing_field_name_3(struct __anonstruct____missing_field_name_3 *ptr ,
                                                        int fd ) 
{ 


  {

}
}
void __serialize___anonstruct_rangefinder_state_1(struct __anonstruct_rangefinder_state_1 *ptr ,
                                                  int fd ) 
{ 


  {

}
}
void __serialize___anonstruct_control_switch_state_4(struct __anonstruct_control_switch_state_4 *ptr ,
                                                     int fd ) 
{ 


  {

}
}
void __serialize_scalarInt16(short *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 2);
}
}
void __serialize___anonstruct_nav_payload_place_8(struct __anonstruct_nav_payload_place_8 *ptr ,
                                                  int fd ) 
{ 


  {

}
}
void __deserialize___anonstruct_control_switch_state_4(struct __anonstruct_control_switch_state_4 *ptr ,
                                                       int fd ) 
{ 


  {

}
}
void __deserialize_PayloadPlaceStateType(enum PayloadPlaceStateType *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}
void __deserialize___anonstruct_nav_payload_place_8(struct __anonstruct_nav_payload_place_8 *ptr ,
                                                    int fd ) 
{ 


  {

}
}
void __serialize_GuidedMode(enum GuidedMode *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __serialize_AutoMode(enum AutoMode *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize___anonunion_ap_2(union __anonunion_ap_2 *ptr , int fd ) 
{ 


  {

}
}
void __deserialize___anonstruct_takeoff_state_5(struct __anonstruct_takeoff_state_5 *ptr ,
                                                int fd ) 
{ 


  {

}
}
void __serialize_RTLState(enum RTLState *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
void __deserialize___anonstruct_rangefinder_state_1(struct __anonstruct_rangefinder_state_1 *ptr ,
                                                    int fd ) 
{ 


  {

}
}
#line 606 "wes.i"
int main(void) 
{ 
  struct Copter myCopter ;
  int fd2 ;
  int fd3 ;

  {
  {
  SERIALIZE: ;
  {
  fd2 = open("serialized.data", 514, 504);
  _memoizeMax = 0;
  __serialize_Copter(& myCopter, fd2);
  close(fd2);
  }
  }
  {
  DESERIALIZE: ;
  {
  fd3 = open("serialized.data", 514, 504);
  _memoizeMax = 0;
  __deserialize_Copter(& myCopter, fd3);
  close(fd3);
  }
  }
#line 615
  return (0);
}
}
