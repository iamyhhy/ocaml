typedef char bool;
typedef unsigned char __uint8_t;
typedef short __int16_t;
typedef unsigned short __uint16_t;
typedef int __int32_t;
typedef unsigned int __uint32_t;
typedef long long __int64_t;
typedef unsigned long long __uint64_t;
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long long int64_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
 typedef struct {
        uint8_t dynamic_flight : 1;
        uint8_t init_targets_on_arming : 1;
        uint8_t inverted_flight : 1;
    } heli_flags_t;
    heli_flags_t heli_flags;


typedef struct {
    bool running;
    float max_speed;
    float alt_delta;
    uint32_t start_ms;
} takeoff_state_t;
takeoff_state_t takeoff_state;

enum HomeState {
    HOME_UNSET,
    HOME_SET_NOT_LOCKED,
    HOME_SET_AND_LOCKED
};
typedef enum HomeState HomeState; 

enum control_mode_t {
    STABILIZE = 0,
    ACRO = 1,
    ALT_HOLD = 2,
    AUTO = 3,
    GUIDED = 4,
    LOITER = 5,
    RTL = 6,
    CIRCLE = 7,
    LAND = 9,
    DRIFT = 11,
    SPORT = 13,
    FLIP = 14,
    AUTOTUNE = 15,
    POSHOLD = 16,
    BRAKE = 17,
    THROW = 18,
    AVOID_ADSB = 19,
    GUIDED_NOGPS = 20,
    SMART_RTL = 21,
};
typedef enum control_mode_t control_mode_t;

enum mode_reason_t {
    MODE_REASON_UNKNOWN=0,
    MODE_REASON_TX_COMMAND,
    MODE_REASON_GCS_COMMAND,
    MODE_REASON_RADIO_FAILSAFE,
    MODE_REASON_BATTERY_FAILSAFE,
    MODE_REASON_GCS_FAILSAFE,
    MODE_REASON_EKF_FAILSAFE,
    MODE_REASON_GPS_GLITCH,
    MODE_REASON_MISSION_END,
    MODE_REASON_THROTTLE_LAND_ESCAPE,
    MODE_REASON_FENCE_BREACH,
    MODE_REASON_TERRAIN_FAILSAFE,
    MODE_REASON_BRAKE_TIMEOUT,
    MODE_REASON_FLIP_COMPLETE,
    MODE_REASON_AVOIDANCE,
    MODE_REASON_AVOIDANCE_RECOVERY,
    MODE_REASON_THROW_COMPLETE,
    MODE_REASON_TERMINATE,
};
typedef enum mode_reason_t mode_reason_t;
/*
class Copter : public AP_HAL::HAL::Callbacks {

public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class AP_Rally_Copter;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Avoidance_Copter;



    friend class AP_Arming_Copter;

    Copter(void);


    void setup() override;
    void loop() override;

private:
    static const AP_FWVersion fwver;


    AP_Vehicle::MultiCopter aparm;
*/
struct Copter {

   
   // Parameters g;
   // ParametersG2 g2;


    // AP_Scheduler scheduler;


    //AP_Notify notify;


    uint8_t command_ack_counter;


   /* RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;


    DataFlash_Class DataFlash;

    AP_GPS gps;


    AP_Int8 *flight_modes;

    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
*/
  //  RangeFinder rangefinder{serial_manager, ROTATION_PITCH_270};
    
    struct {
        bool enabled:1;
        bool alt_healthy:1;
        int16_t alt_cm;
        uint32_t last_healthy_ms;
        //LowPassFilterFloat alt_cm_filt;
        int8_t glitch_count;
    } rangefinder_state;

    //AP_RPM rpm_sensor;

/*
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs{ins, barometer, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};


    SITL::SITL sitl;

*/

    AP_Mission mission{ahrs,
            Functor<bool, const AP_Mission::Mission_Command &>::bind<std::remove_reference<decltype(*this)>::type, &Copter::start_command>(this),
            Functor<bool, const AP_Mission::Mission_Command &>::bind<std::remove_reference<decltype(*this)>::type, &Copter::verify_command_callback>(this),
            Functor<void>::bind<std::remove_reference<decltype(*this)>::type, &Copter::exit_mission>(this)};
/*
    bool start_command(const AP_Mission::Mission_Command& cmd) {
        return mode_auto.start_command(cmd);
    }
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) {
        return mode_auto.verify_command_callback(cmd);
    }
    void exit_mission() {
        mode_auto.exit_mission();
    }


    AP_Arming_Copter arming{ahrs, barometer, compass, battery, inertial_nav, ins};



    OpticalFlow optflow{ahrs};

*/

    float ekfGndSpdLimit;


    float ekfNavVelGainScaler;


  //  uint32_t ekfYawReset_ms = 0;
    int8_t ekf_primary_core;

  //  AP_SerialManager serial_manager;


   // GCS_Copter _gcs;
   // GCS_Copter &gcs() { return _gcs; }







     union{
        struct {
            uint8_t unused1 : 1;
            uint8_t simple_mode : 2;
            uint8_t pre_arm_rc_check : 1;
            uint8_t pre_arm_check : 1;
            uint8_t auto_armed : 1;
            uint8_t logging_started : 1;
            uint8_t land_complete : 1;
            uint8_t new_radio_frame : 1;
            uint8_t usb_connected : 1;
            uint8_t rc_receiver_present : 1;
            uint8_t compass_mot : 1;
            uint8_t motor_test : 1;
            uint8_t initialised : 1;
            uint8_t land_complete_maybe : 1;
            uint8_t throttle_zero : 1;
            uint8_t system_time_set : 1;
            uint8_t gps_glitching : 1;
            enum HomeState home_state : 2;
            uint8_t using_interlock : 1;
            uint8_t motor_emergency_stop : 1;
            uint8_t land_repo_active : 1;
            uint8_t motor_interlock_switch : 1;
            uint8_t in_arming_delay : 1;
            uint8_t initialised_params : 1;
            uint8_t compass_init_location : 1;
        };
        uint32_t value;
    } ap_t;

   // ap_t ap;



    control_mode_t control_mode;
    //mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;
    mode_reason_t control_mode_reason;
	//control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    //mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;
    mode_reason_t prev_control_mode_reason;

    struct {
        int8_t debounced_switch_position;
        int8_t last_switch_position;
        uint32_t last_edge_time_ms;
    } control_switch_state;




    float auto_takeoff_no_nav_alt_cm;

    //RCMapper rcmap;


    //AP_BoardConfig BoardConfig;







    uint8_t receiver_rssi;


    struct {
        uint8_t rc_override_active : 1;
        uint8_t radio : 1;
        uint8_t battery : 1;
        uint8_t gcs : 1;
        uint8_t ekf : 1;
        uint8_t terrain : 1;
        uint8_t adsb : 1;

        int8_t radio_counter;

        uint32_t last_heartbeat_ms;
        uint32_t terrain_first_failure_ms;
        uint32_t terrain_last_failure_ms;
    } failsafe;


    struct {
        uint8_t baro : 1;
        uint8_t compass : 1;
        uint8_t primary_gps;
    } sensor_health;
//# 367 "../../ArduCopter/Copter.h"
    //AP_MotorsHeli *motors;
    //const struct AP_Param::GroupInfo *motors_var_info;



    float scaleLongDown;

    int32_t _home_bearing;
    uint32_t _home_distance;




    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;


    int32_t initial_armed_bearing;


    //AP_BattMonitor battery;



    //AP_Frsky_Telem frsky_telemetry{ahrs, battery, rangefinder};



    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;



    int16_t climb_rate;
    float target_rangefinder_alt;
    int32_t baro_alt;
    float baro_climbrate;
    //LowPassFilterVector3f land_accel_ef_filter;


   // LowPassFilterFloat rc_throttle_control_in_filter;


    //AP::PerfInfo perf_info;



    //Location_Class current_loc;



    uint8_t auto_yaw_mode;


    //Vector3f roi_WP;


    float yaw_look_at_WP_bearing;


    int32_t yaw_look_at_heading;


    int16_t yaw_look_at_heading_slew;


    float yaw_look_ahead_bearing;


    float auto_yaw_rate_cds;




    float G_Dt;

/*
    AP_InertialNav_NavEKF inertial_nav;
# 457 "../../ArduCopter/Copter.h"
    AC_AttitudeControl_Heli *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Circle *circle_nav;
*/

    int16_t pmTest1;




    uint32_t fast_loopTimer;

    uint16_t mainLoop_count;

    uint32_t arm_time_ms;


    uint8_t auto_trim_counter;


 //   AP_Relay relay;


   // AP_ServoRelayEvents ServoRelayEvents{relay};



   // AP_Camera camera{&relay, (1<<15), current_loc, ahrs};





    //AP_Mount camera_mount{ahrs, current_loc};




    //AC_Fence fence{ahrs};



    //AC_Avoid avoid{ahrs, fence, g2.proximity, &g2.beacon};




    //AP_Rally_Copter rally{ahrs};



    //AP_RSSI rssi;



    //AC_Sprayer sprayer{&inertial_nav};




   // AP_Parachute parachute{relay};



    //AP_LandingGear landinggear;



   // AP_Terrain terrain{ahrs, mission, rally};




    //AC_PrecLand precland{ahrs, inertial_nav};





    //AC_InputManager_Heli input_manager;


   // AP_ADSB adsb{ahrs};


   // AP_Avoidance_Copter avoidance_adsb{ahrs, adsb};


    bool in_mavlink_delay;


    uint32_t last_radio_update_ms;


    uint32_t esc_calibration_notify_update_ms;



    uint32_t visual_odom_last_update_ms;




    //AP_Param param_loader;





   // ModeFilterInt16_Size5 rotor_speed_deglitch_filter {4};


        int16_t hover_roll_trim_scalar_slew;



    struct {
        bool takeoff_expected;
        bool touchdown_expected;
        uint32_t takeoff_time_ms;
        float takeoff_alt_cm;
    } gndeffect_state;


    bool upgrading_frame_params;

   // static const AP_Scheduler::Task scheduler_tasks[];
   // static const AP_Param::Info var_info[];
   // static const struct LogStructure log_structure[];


/*
    void set_home_state(enum HomeState new_home_state);
    bool home_is_set();
    void set_auto_armed(bool b);
    void set_simple_mode(uint8_t b);
    void set_failsafe_radio(bool b);
    void set_failsafe_battery(bool b);
    void set_failsafe_gcs(bool b);
    void update_using_interlock();
    void set_motor_emergency_stop(bool b);


    void perf_update(void);
    void stats_update();
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_mount();
    void update_trigger(void);
    void update_batt_compass(void);
    void fourhundred_hz_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void dataflash_periodic(void);
    void ins_periodic();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void smart_rtl_save_position();
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();


    float get_smoothing_gain();
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    float get_roi_yaw();
    float get_look_ahead_yaw();
    void update_throttle_hover();
    void set_throttle_takeoff();
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f);
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle();
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
    float get_avoidance_adjusted_climbrate(float target_rate);
    void set_accel_throttle_I_from_pilot_throttle();
    void rotate_body_frame_to_NE(float &x, float &y);
    uint16_t get_pilot_speed_dn();


    void avoidance_adsb_update(void);


    void update_ground_effect_detector(void);


    void init_capabilities(void);


    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock);
    bool set_home(const Location& loc, bool lock);
    void set_ekf_origin(const Location& loc);
    bool far_from_EKF_origin(const Location& loc);
    void set_system_time_from_GPS();


    MAV_RESULT mavlink_compassmot(mavlink_channel_t chan);


    void delay(uint32_t ms);


    void crash_check();
    void parachute_check();
    void parachute_release();
    void parachute_manual_release();


    void ekf_check();
    bool ekf_check_position_problem();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void check_ekf_reset();


    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    void esc_calibration_notify();


    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void failsafe_battery_event(void);
    void failsafe_gcs_check();
    void failsafe_gcs_off_event(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void gpsglitch_check();
    void set_mode_RTL_or_land_with_pause(mode_reason_t reason);
    void set_mode_SmartRTL_or_RTL(mode_reason_t reason);
    void set_mode_SmartRTL_or_land_with_pause(mode_reason_t reason);
    bool should_disarm_on_failsafe();
    void update_events();


    void failsafe_enable();
    void failsafe_disable();





    void fence_check();
    void fence_send_mavlink_status(mavlink_channel_t chan);


    void gcs_send_heartbeat(void);
    void gcs_send_deferred(void);
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_fence_status(mavlink_channel_t chan);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_rpm(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void gcs_data_stream_send(void);
    void gcs_check_input(void);


    void heli_init();
    void check_dynamic_flight(void);
    void update_heli_control_dynamics(void);
    void heli_update_landing_swash();
    void heli_update_rotor_speed_targets();


    void read_inertia();


    void update_land_and_crash_detectors();
    void update_land_detector();
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    void update_throttle_thr_mix();


    void landinggear_update();


    void update_notify();


    void Log_Write_Current();
    void Log_Write_Optflow();
    void Log_Write_Nav_Tuning();
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_EKF_POS();
    void Log_Write_MotBatt();
    void Log_Write_Event(uint8_t id);
    void Log_Write_Data(uint8_t id, int32_t value);
    void Log_Write_Data(uint8_t id, uint32_t value);
    void Log_Write_Data(uint8_t id, int16_t value);
    void Log_Write_Data(uint8_t id, uint16_t value);
    void Log_Write_Data(uint8_t id, float value);
    void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
    void Log_Write_Baro(void);
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high);
    void Log_Write_Home_And_Origin();
    void Log_Sensor_Health();

    void Log_Write_Heli(void);

    void Log_Write_Precland();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool position_ok);
    void Log_Write_Proximity();
    void Log_Write_Beacon();
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);


    bool set_mode(control_mode_t mode, mode_reason_t reason);
    void update_flight_mode();
    void notify_flight_mode();


    uint8_t get_default_auto_yaw_mode(bool rtl);
    void set_auto_yaw_mode(uint8_t yaw_mode);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle);
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_rate(float turn_rate_cds);
    float get_auto_heading(void);
    float get_auto_yaw_rate_cds();


    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horizontal_control();
    void set_mode_land_with_pause(mode_reason_t reason);
    bool landing_with_GPS();


    void motor_test_output();
    bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc);
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec, uint8_t motor_count);
    void motor_test_stop();


    void arm_motors_check();
    void auto_disarm_check();
    bool init_arm_motors(bool arming_from_gcs);
    void init_disarm_motors();
    void motors_output();
    void lost_vehicle_check();


    void run_nav_updates(void);
    int32_t home_bearing();
    uint32_t home_distance();


    void load_parameters(void);
    void convert_pid_parameters(void);


    Vector3f pv_location_to_vector(const Location& loc);
    float pv_alt_above_origin(float alt_above_home_cm);
    float pv_alt_above_home(float alt_above_origin_cm);
    float pv_distance_to_home_cm(const Vector3f &destination);


    void init_precland();
    void update_precland();


    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors();


    void init_barometer(bool full_calibration);
    void read_barometer(void);
    void barometer_accumulate(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok();
    void rpm_update();
    void init_compass();
    void compass_accumulate(void);
    void init_optflow();
    void update_optical_flow(void);
    void read_battery(void);
    void read_receiver_rssi(void);
    void compass_cal_update(void);
    void accel_cal_update(void);
    void gripper_update();
    void button_update();
    void init_proximity();
    void update_proximity();
    void update_sensor_status_flags(void);
    void init_beacon();
    void update_beacon();
    void init_visual_odom();
    void update_visual_odom();
    void winch_init();
    void winch_update();


    void report_compass();
    void print_blanks(int16_t num);
    void print_divider(void);
    void print_enabled(bool b);
    void report_version();


    void read_control_switch();
    bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
    bool check_duplicate_auxsw(void);
    void reset_control_switch();
    uint8_t read_3pos_switch(uint8_t chan);
    void read_aux_switches();
    void init_aux_switches();
    void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
    void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
    void save_trim();
    void auto_trim();


    void init_ardupilot();
    void startup_INS_ground();
    bool calibrate_gyros();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    void update_auto_armed();
    void check_usb_mux(void);
    bool should_log(uint32_t mask);
    void set_default_frame_class();
    uint8_t get_frame_mav_type();
    const char* get_frame_string();
    void allocate_motors(void);


    bool current_mode_has_user_takeoff(bool must_navigate);
    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    void takeoff_timer_start(float alt_cm);
    void takeoff_stop();
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate);
    void auto_takeoff_set_start_alt(void);
    void auto_takeoff_attitude_run(float target_yaw_rate);


    void terrain_update();
    void terrain_logging();
    bool terrain_use();


    void tuning();


    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();
*/
//# 1 "../../ArduCopter/mode.h" 1
       


/*
class Mode {
    friend class Copter;
    friend class AP_Arming_Copter;

public:

    Mode(Copter &copter) :
        _copter(copter),
        g(copter.g),
        g2(copter.g2),
        wp_nav(_copter.wp_nav),
        pos_control(_copter.pos_control),
        inertial_nav(_copter.inertial_nav),
        ahrs(_copter.ahrs),
        attitude_control(_copter.attitude_control),
        motors(_copter.motors),
        channel_roll(_copter.channel_roll),
        channel_pitch(_copter.channel_pitch),
        channel_throttle(_copter.channel_throttle),
        channel_yaw(_copter.channel_yaw),
        G_Dt(_copter.G_Dt),
        ap(_copter.ap),
        takeoff_state(_copter.takeoff_state),
        ekfGndSpdLimit(_copter.ekfGndSpdLimit),
        ekfNavVelGainScaler(_copter.ekfNavVelGainScaler),

        heli_flags(_copter.heli_flags),

        auto_yaw_mode(_copter.auto_yaw_mode)
        { };

protected:

    virtual bool init(bool ignore_checks) = 0;
    virtual void run() = 0;

    virtual bool is_autopilot() const { return false; }
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;

    virtual bool landing_gear_should_be_deployed() const { return false; }

    virtual const char *name() const = 0;


    virtual const char *name4() const = 0;


    void update_navigation();
    virtual void run_autopilot() {}
    virtual uint32_t wp_distance() const { return 0; }
    virtual int32_t wp_bearing() const { return 0; }

    Copter &_copter;


    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl_Heli *&attitude_control;
    AP_MotorsHeli *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;
    ap_t &ap;
    takeoff_state_t &takeoff_state;


    bool takeoff_triggered(float target_climb_rate) const;


    float &ekfGndSpdLimit;


    float &ekfNavVelGainScaler;



    uint8_t &auto_yaw_mode;


    heli_flags_t &heli_flags;





    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max) {
        _copter.get_pilot_desired_lean_angles(roll_in, pitch_in, roll_out, pitch_out, angle_max);
    }
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt) {
        return _copter.get_surface_tracking_climb_rate(target_rate, current_alt_target, dt);
    }
    float get_pilot_desired_yaw_rate(int16_t stick_angle) {
        return _copter.get_pilot_desired_yaw_rate(stick_angle);
    }
    float get_pilot_desired_climb_rate(float throttle_control) {
        return _copter.get_pilot_desired_climb_rate(throttle_control);
    }
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f) {
        return _copter.get_pilot_desired_throttle(throttle_control, thr_mid);
    }
    float get_non_takeoff_throttle() {
        return _copter.get_non_takeoff_throttle();
    }
    void update_simple_mode(void) {
        _copter.update_simple_mode();
    }
    float get_smoothing_gain() {
        return _copter.get_smoothing_gain();
    }
    bool set_mode(control_mode_t mode, mode_reason_t reason) {
        return _copter.set_mode(mode, reason);
    }
    void set_land_complete(bool b) {
        return _copter.set_land_complete(b);
    }
    GCS_Copter &gcs() {
        return _copter.gcs();
    }
    void Log_Write_Event(uint8_t id) {
        return _copter.Log_Write_Event(id);
    }
    void set_throttle_takeoff() {
        return _copter.set_throttle_takeoff();
    }
    void set_auto_yaw_mode(uint8_t yaw_mode) {
        return _copter.set_auto_yaw_mode(yaw_mode);
    }
    void set_auto_yaw_rate(float turn_rate_cds) {
        return _copter.set_auto_yaw_rate(turn_rate_cds);
    }
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle) {
        return _copter.set_auto_yaw_look_at_heading(angle_deg, turn_rate_dps, direction, relative_angle);
    }
    void takeoff_timer_start(float alt_cm) {
        return _copter.takeoff_timer_start(alt_cm);
    }
    void takeoff_stop() {
        return _copter.takeoff_stop();
    }
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate) {
        return _copter.takeoff_get_climb_rates(pilot_climb_rate, takeoff_climb_rate);
    }
    float get_auto_heading() {
        return _copter.get_auto_heading();
    }
    float get_auto_yaw_rate_cds() {
        return _copter.get_auto_yaw_rate_cds();
    }
    float get_avoidance_adjusted_climbrate(float target_rate) {
        return _copter.get_avoidance_adjusted_climbrate(target_rate);
    }
    uint16_t get_pilot_speed_dn() {
        return _copter.get_pilot_speed_dn();
    }



    void zero_throttle_and_relax_ac();
};


class ModeAcro : public Mode {

public:

    ModeAcro(Copter &copter) :
        Copter::Mode(copter)
        { }
    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool is_autopilot() const override { return false; }
    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

private:

};


class ModeAcro_Heli : public ModeAcro {

public:

    ModeAcro_Heli(Copter &copter) :
        Copter::ModeAcro(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

protected:
private:
};



class ModeAltHold : public Mode {

public:

    ModeAltHold(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};


class ModeAuto : public Mode {

public:

    ModeAuto(Copter &copter, AP_Mission &_mission, AC_Circle *& _circle_nav) :
        Copter::Mode(copter),
        mission(_mission),
        circle_nav(_circle_nav)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool is_autopilot() const override { return true; }
    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };


    AutoMode mode() const { return _mode; }

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    void wp_start(const Vector3f& destination);
    void wp_start(const Location_Class& dest_loc);
    void land_start();
    void land_start(const Vector3f& destination);
    void circle_movetoedge_start(const Location_Class &circle_center, float radius_m);
    void circle_start();
    void spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);
    void spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);
    void nav_guided_start();

    bool landing_gear_should_be_deployed() const override;

    void payload_place_start();


    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void exit_mission();


    bool do_guided(const AP_Mission::Mission_Command& cmd);

protected:

    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }
    void run_autopilot() override { mission.update(); }

private:

    bool verify_command(const AP_Mission::Mission_Command& cmd);

    void takeoff_run();
    void wp_run();
    void spline_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();

    void payload_place_start(const Vector3f& destination);
    void payload_place_run();
    bool payload_place_run_should_run();
    void payload_place_run_loiter();
    void payload_place_run_descend();
    void payload_place_run_release();

    AutoMode _mode = Auto_TakeOff;
*/
    AP_Mission &mission;

    /*
    AC_Circle *&circle_nav;

    Location_Class terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);

    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);

    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);


    void do_parachute(const AP_Mission::Mission_Command& cmd);


    void do_gripper(const AP_Mission::Mission_Command& cmd);

    void do_winch(const AP_Mission::Mission_Command& cmd);
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time();
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);

    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);

    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    void auto_spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);


    uint16_t loiter_time_max;
    uint32_t loiter_time;


    int32_t nav_delay_time_max;
    uint32_t nav_delay_time_start;


    int32_t condition_value;
    uint32_t condition_start;

    LandStateType land_state = LandStateType_FlyToLocation;

    struct {
        PayloadPlaceStateType state = PayloadPlaceStateType_Calibrating_Hover_Start;
        uint32_t hover_start_timestamp;
        float hover_throttle_level;
        uint32_t descend_start_timestamp;
        uint32_t place_start_timestamp;
        float descend_throttle_level;
        float descend_start_altitude;
        float descend_max;
    } nav_payload_place;

};
# 563 "../../ArduCopter/mode.h"
class ModeBrake : public Mode {

public:

    ModeBrake(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};


class ModeCircle : public Mode {

public:

    ModeCircle(Copter &copter, AC_Circle *& _circle_nav) :
        Copter::Mode(copter),
        circle_nav(_circle_nav)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_loiter_distance_to_target();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_loiter_bearing_to_target();
    }

private:


    bool pilot_yaw_override = false;
    AC_Circle *&circle_nav;

};


class ModeDrift : public Mode {

public:

    ModeDrift(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:

    ModeFlip(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:


    Vector3f flip_orig_attitude;

};


class ModeGuided : public Mode {

public:

    ModeGuided(Copter &copter) :
        Copter::Mode(copter) { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return from_gcs; }
    bool is_autopilot() const override { return true; }

    void set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool set_destination(const Location_Class& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool takeoff_start(float final_alt_above_home);

    GuidedMode mode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

private:

    void pos_control_start();
    void vel_control_start();
    void posvel_control_start();
    void takeoff_run();
    void pos_control_run();
    void vel_control_run();
    void posvel_control_run();
    void set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);


    GuidedMode guided_mode = Guided_TakeOff;

};


class ModeGuidedNoGPS : public ModeGuided {

public:

    ModeGuidedNoGPS(Copter &copter) :
        Copter::ModeGuided(copter) { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return from_gcs; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:

    ModeLand(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }
    bool landing_gear_should_be_deployed() const override { return true; };

    float get_land_descent_speed();
    bool landing_with_GPS();
    void do_not_use_GPS();

    int32_t get_alt_above_ground(void);

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();
};


class ModeLoiter : public Mode {

public:

    ModeLoiter(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }


    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }


protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_loiter_distance_to_target();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_loiter_bearing_to_target();
    }


    bool do_precision_loiter();
    void precision_loiter_xy();


private:


    bool _precision_loiter_enabled;


};


class ModePosHold : public Mode {

public:

    ModePosHold(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
    void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
    void poshold_update_wind_comp_estimate();
    void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
    void poshold_roll_controller_to_pilot_override();
    void poshold_pitch_controller_to_pilot_override();

};


class ModeRTL : public Mode {

public:

    ModeRTL(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return true; }

    RTLState state() { return _state; }


    bool state_complete() { return _state_complete; }

    bool landing_gear_should_be_deployed() const override;

    void restart_without_terrain();

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }

    void descent_start();
    void descent_run();
    void land_start();
    void land_run(bool disarm_on_land);

    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void build_path(bool terrain_following_allowed);
    void compute_return_target(bool terrain_following_allowed);


    RTLState _state = RTL_InitialClimb;
    bool _state_complete = false;

    struct {

        Location_Class origin_point;
        Location_Class climb_target;
        Location_Class return_target;
        Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;


    uint32_t _loiter_start_time = 0;
};


class ModeSmartRTL : public ModeRTL {

public:

    ModeSmartRTL(Copter &copter) :
        ModeSmartRTL::ModeRTL(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    void save_position();
    void exit();

protected:

    const char *name() const override { return "SMARTRTL"; }
    const char *name4() const override { return "SRTL"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SmartRTLState smart_rtl_state = SmartRTL_PathFollow;

};


class ModeSport : public Mode {

public:

    ModeSport(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};


class ModeStabilize : public Mode {

public:

    ModeStabilize(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};


class ModeStabilize_Heli : public ModeStabilize {

public:

    ModeStabilize_Heli(Copter &copter) :
        Copter::ModeStabilize(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

protected:

private:

};



class ModeThrow : public Mode {

public:

    ModeThrow(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }


protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    bool throw_detected();
    bool throw_position_good();
    bool throw_height_good();
    bool throw_attitude_good();

    ThrowModeStage stage = Throw_Disarmed;
    ThrowModeStage prev_stage = Throw_Disarmed;
    uint32_t last_log_ms;
    bool nextmode_attempted;
    uint32_t free_fall_start_ms;
    float free_fall_start_velz;

};


class ModeAvoidADSB : public ModeGuided {

public:

    ModeAvoidADSB(Copter &copter) :
        Copter::ModeGuided(copter) { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

private:

};

*/
//# 940 "../../ArduCopter/Copter.h" 2

  /*
  Mode *flightmode;

    ModeAcro_Heli mode_acro{*this};



    ModeAltHold mode_althold{*this};
    ModeAuto mode_auto{*this, mission, circle_nav};



    ModeBrake mode_brake{*this};
    ModeCircle mode_circle{*this, circle_nav};
    ModeDrift mode_drift{*this};
    ModeFlip mode_flip{*this};
    ModeGuided mode_guided{*this};
    ModeLand mode_land{*this};
    ModeLoiter mode_loiter{*this};
    ModePosHold mode_poshold{*this};
    ModeRTL mode_rtl{*this};

    ModeStabilize_Heli mode_stabilize{*this};



    ModeSport mode_sport{*this};
    ModeAvoidADSB mode_avoid_adsb{*this};
    ModeThrow mode_throw{*this};
    ModeGuidedNoGPS mode_guided_nogps{*this};
    ModeSmartRTL mode_smartrtl{*this};


    Mode *mode_from_mode_num(const uint8_t mode);
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

public:
    void mavlink_delay_cb();
    void failsafe_check();

    */
};
int main() {
    struct Copter myCopter;
    {
    SERIALIZE:
    }
    {
    DESERIALIZE:
    }

    return 0;
}
