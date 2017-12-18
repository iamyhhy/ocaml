
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long long int64_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;



typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    FIRMWARE_VERSION_TYPE fw_type;
    const char *fw_string;
    const char *fw_hash_str;
    const char *middleware_hash_str;
    const char *os_hash_str;
} AP_FWVersion;

//class Copter : public AP_HAL::HAL::Callbacks {
struct Copter {
/*
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
*/
    enum AUTOTUNE_LEVEL_ISSUE {
        AUTOTUNE_LEVEL_ISSUE_NONE,
        AUTOTUNE_LEVEL_ISSUE_ANGLE_ROLL,
        AUTOTUNE_LEVEL_ISSUE_ANGLE_PITCH,
        AUTOTUNE_LEVEL_ISSUE_ANGLE_YAW,
        AUTOTUNE_LEVEL_ISSUE_RATE_ROLL,
        AUTOTUNE_LEVEL_ISSUE_RATE_PITCH,
        AUTOTUNE_LEVEL_ISSUE_RATE_YAW,
    };

//private:
    static const AP_FWVersion fwver;


    //AP_Vehicle::MultiCopter aparm;



   // Parameters g;
   // ParametersG2 g2;


   // AP_Scheduler scheduler = AP_Scheduler::create();


    //AP_Notify notify = AP_Notify::create();


    uint8_t command_ack_counter;


    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;


    DataFlash_Class DataFlash;

    AP_GPS gps = AP_GPS::create();


    AP_Int8 *flight_modes;

    AP_Baro barometer = AP_Baro::create();
    Compass compass = Compass::create();
    AP_InertialSensor ins = AP_InertialSensor::create();

    RangeFinder rangefinder = RangeFinder::create(serial_manager, ROTATION_PITCH_270);
    struct {
        bool enabled:1;
        bool alt_healthy:1;
        int16_t alt_cm;
        uint32_t last_healthy_ms;
        LowPassFilterFloat alt_cm_filt;
        int8_t glitch_count;
    } rangefinder_state = { false, false, 0, 0 };

    AP_RPM rpm_sensor = AP_RPM::create();


    NavEKF2 EKF2 = NavEKF2::create(&ahrs, barometer, rangefinder);
    NavEKF3 EKF3 = NavEKF3::create(&ahrs, barometer, rangefinder);
    AP_AHRS_NavEKF ahrs = AP_AHRS_NavEKF::create(ins, barometer, gps, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF);


    SITL::SITL sitl;



    AP_Mission mission = AP_Mission::create(ahrs,
            Functor<bool, const AP_Mission::Mission_Command &>::bind<std::remove_reference<decltype(*this)>::type, &Copter::start_command>(this),
            Functor<bool, const AP_Mission::Mission_Command &>::bind<std::remove_reference<decltype(*this)>::type, &Copter::verify_command_callback>(this),
            Functor<void>::bind<std::remove_reference<decltype(*this)>::type, &Copter::exit_mission>(this));


    AP_Arming_Copter arming = AP_Arming_Copter::create(ahrs, barometer, compass, battery, inertial_nav, ins);



    OpticalFlow optflow = OpticalFlow::create(ahrs);



    float ekfGndSpdLimit;


    float ekfNavVelGainScaler;


    uint32_t ekfYawReset_ms = 0;
    int8_t ekf_primary_core;

    AP_SerialManager serial_manager = AP_SerialManager::create();


    GCS_Copter _gcs;
    GCS_Copter &gcs() { return _gcs; }







    union {
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
    } ap;



    control_mode_t control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;


    struct {
        int8_t debounced_switch_position;
        int8_t last_switch_position;
        uint32_t last_edge_time_ms;
    } control_switch_state;

    struct {
        bool running;
        float max_speed;
        float alt_delta;
        uint32_t start_ms;
    } takeoff_state;


    float auto_takeoff_no_nav_alt_cm;

    RCMapper rcmap = RCMapper::create();


    AP_BoardConfig BoardConfig = AP_BoardConfig::create();







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
# 364 "ArduCopterC.cpp"
    AP_MotorsMulticopter *motors;
    const struct AP_Param::GroupInfo *motors_var_info;



    float scaleLongDown;


    int32_t wp_bearing;

    int32_t home_bearing;

    int32_t home_distance;

    uint32_t wp_distance;
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


    AutoMode auto_mode;


    GuidedMode guided_mode;


    RTLState rtl_state;
    bool rtl_state_complete;

    struct {

        Location_Class origin_point;
        Location_Class climb_target;
        Location_Class return_target;
        Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;


    SmartRTLState smart_rtl_state;


    bool circle_pilot_yaw_override;




    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;


    int32_t initial_armed_bearing;


    uint16_t loiter_time_max;
    uint32_t loiter_time;


    uint32_t brake_timeout_start;
    uint32_t brake_timeout_ms;


    int32_t nav_delay_time_max;
    uint32_t nav_delay_time_start;


    Vector3f flip_orig_attitude;


    struct {
        ThrowModeStage stage;
        ThrowModeStage prev_stage;
        uint32_t last_log_ms;
        bool nextmode_attempted;
        uint32_t free_fall_start_ms;
        float free_fall_start_velz;
    } throw_state = {Throw_Disarmed, Throw_Disarmed, 0, false, 0, 0.0f};


    AP_BattMonitor battery = AP_BattMonitor::create();



    AP_Frsky_Telem frsky_telemetry = AP_Frsky_Telem::create(ahrs, battery, rangefinder);



    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;



    int16_t climb_rate;
    float target_rangefinder_alt;
    int32_t baro_alt;
    float baro_climbrate;
    LowPassFilterVector3f land_accel_ef_filter;


    LowPassFilterFloat rc_throttle_control_in_filter;


    AP::PerfInfo perf_info = AP::PerfInfo::create();



    Location_Class current_loc;



    uint8_t auto_yaw_mode;


    Vector3f roi_WP;


    float yaw_look_at_WP_bearing;


    int32_t yaw_look_at_heading;


    int16_t yaw_look_at_heading_slew;


    float yaw_look_ahead_bearing;


    float auto_yaw_rate_cds;


    int32_t condition_value;
    uint32_t condition_start;




    float G_Dt;


    AP_InertialNav_NavEKF inertial_nav;



    AC_AttitudeControl *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Circle *circle_nav;


    int16_t pmTest1;




    uint32_t fast_loopTimer;

    uint16_t mainLoop_count;

    uint32_t rtl_loiter_start_time;

    uint32_t arm_time_ms;


    uint8_t auto_trim_counter;


    AP_Relay relay = AP_Relay::create();


    AP_ServoRelayEvents ServoRelayEvents = AP_ServoRelayEvents::create(relay);



    AP_Camera camera = AP_Camera::create(&relay, (1<<15), current_loc, ahrs);





    AP_Mount camera_mount = AP_Mount::create(ahrs, current_loc);




    AC_Fence fence = AC_Fence::create(ahrs, inertial_nav);



    AC_Avoid avoid = AC_Avoid::create(ahrs, inertial_nav, fence, g2.proximity, &g2.beacon);




    AP_Rally_Copter rally = AP_Rally_Copter::create(ahrs);



    AP_RSSI rssi = AP_RSSI::create();



    AC_Sprayer sprayer = AC_Sprayer::create(&inertial_nav);




    AP_Parachute parachute = AP_Parachute::create(relay);



    AP_LandingGear landinggear = AP_LandingGear::create();



    AP_Terrain terrain = AP_Terrain::create(ahrs, mission, rally);




    AC_PrecLand precland = AC_PrecLand::create(ahrs, inertial_nav);
# 607 "ArduCopterC.cpp"
    AP_ADSB adsb = AP_ADSB::create(ahrs);


    AP_Avoidance_Copter avoidance_adsb = AP_Avoidance_Copter::create(ahrs, adsb);


    bool in_mavlink_delay;


    uint32_t last_radio_update_ms;


    uint32_t esc_calibration_notify_update_ms;



    uint32_t visual_odom_last_update_ms;




    AP_Param param_loader;
# 647 "ArduCopterC.cpp"
    struct {
        bool takeoff_expected;
        bool touchdown_expected;
        uint32_t takeoff_time_ms;
        float takeoff_alt_cm;
    } gndeffect_state;


    bool upgrading_frame_params;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];
# 1166 "ArduCopterC.cpp"
};

