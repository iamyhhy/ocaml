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

enum LandStateType {
    LandStateType_FlyToLocation = 0,
    LandStateType_Descending = 1
};
typedef enum LandStateType LandStateType;

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

enum PayloadPlaceStateType {
    PayloadPlaceStateType_FlyToLocation,
    PayloadPlaceStateType_Calibrating_Hover_Start,
    PayloadPlaceStateType_Calibrating_Hover,
    PayloadPlaceStateType_Descending_Start,
    PayloadPlaceStateType_Descending,
    PayloadPlaceStateType_Releasing_Start,
    PayloadPlaceStateType_Releasing,
    PayloadPlaceStateType_Released,
    PayloadPlaceStateType_Ascending_Start,
    PayloadPlaceStateType_Ascending,
    PayloadPlaceStateType_Done,
};
typedef enum PayloadPlaceStateType PayloadPlaceStateType ;

enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_NavPayloadPlace,
};
typedef enum AutoMode AutoMode ;


enum GuidedMode {
    Guided_TakeOff,
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};
typedef enum GuidedMode GuidedMode ;


enum RTLState {
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};
typedef enum RTLState RTLState ;


enum SmartRTLState {
    SmartRTL_WaitForPathCleanup,
    SmartRTL_PathFollow,
    SmartRTL_PreLandPosition,
    SmartRTL_Descend,
    SmartRTL_Land
};


enum AltHoldModeState {
    AltHold_MotorStopped,
    AltHold_Takeoff,
    AltHold_Flying,
    AltHold_Landed
};


enum LoiterModeState {
    Loiter_MotorStopped,
    Loiter_Takeoff,
    Loiter_Flying,
    Loiter_Landed
};


enum SportModeState {
    Sport_MotorStopped,
    Sport_Takeoff,
    Sport_Flying,
    Sport_Landed
};


enum FlipState {
    Flip_Start,
    Flip_Roll,
    Flip_Pitch_A,
    Flip_Pitch_B,
    Flip_Recover,
    Flip_Abandon
};


enum ThrowModeStage {
    Throw_Disarmed,
    Throw_Detecting,
    Throw_Uprighting,
    Throw_HgtStabilise,
    Throw_PosHold
};

typedef enum ThrowModeStage ThrowModeStage ;


enum ThrowModeType {
    ThrowType_Upward = 0,
    ThrowType_Drop = 1
};

struct Copter {
    uint8_t command_ack_counter;




    int in_log_download;
    // struct DataFlash_Class DataFlash;


    // struct AP_GPS gps;


    // AP_Int8 *flight_modes;


    // struct AP_Baro barometer;

    // struct Compass compass;

    // struct AP_InertialSensor ins;


    // struct RangeFinder rangefinder;

    struct {

        int enabled:1;

        int alt_healthy:1;
        int16_t alt_cm;
        uint32_t last_healthy_ms;

        //pstruct LowPassFilterFloat alt_cm_filt;
        int8_t glitch_count;
    } rangefinder_state; 


    // come back and do this one later
    // struct AP_RPM rpm_sensor;


    // NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    // NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    // AP_AHRS_NavEKF ahrs{ins, barometer, gps, rangefinder, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};



    // struct SITL sitl;




    // struct AP_Mission mission;


    // AP_Arming_Copter arming {ahrs, barometer, compass, battery, inertial_nav, ins};



    // OpticalFlow optflow{ahrs};



    float ekfGndSpdLimit;


    float ekfNavVelGainScaler;


    uint32_t ekfYawReset_ms;
    int8_t ekf_primary_core;


    // AP_SerialManager serial_manager;
    // static const uint8_t num_gcs;

    // GCS_MAVLINK_Copter gcs_chan[5];
    // GCS _gcs;
    // GCS &gcs() { return _gcs; }







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
            uint8_t gps_base_pos_set : 1;
            // enum HomeState home_state : 2;
            uint8_t using_interlock : 1;
            uint8_t motor_emergency_stop : 1;
            uint8_t land_repo_active : 1;
            uint8_t motor_interlock_switch : 1;
            uint8_t in_arming_delay : 1;
        };
        uint32_t value;
    } ap;



    control_mode_t control_mode;
    mode_reason_t control_mode_reason ;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason ;


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

    // RCMapper rcmap;


    // AP_BoardConfig BoardConfig;


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
    // AP_MotorsMulticopter *motors;



    float scaleLongDown;


    int32_t wp_bearing;

    int32_t home_bearing;

    int32_t home_distance;

    uint32_t wp_distance;
    LandStateType land_state ;

    struct {
        PayloadPlaceStateType state ;
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

        // Location_Class origin_point;
        // Location_Class climb_target;
        // Location_Class return_target;
        // Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;


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


    // Vector3f flip_orig_attitude;


    struct {
        ThrowModeStage stage;
        ThrowModeStage prev_stage;
        uint32_t last_log_ms;
        bool nextmode_attempted;
        uint32_t free_fall_start_ms;
        float free_fall_start_velz;
    } throw_state; 


    // AP_BattMonitor battery;



    // AP_Frsky_Telem frsky_telemetry;



    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;



    int16_t climb_rate;
    float target_rangefinder_alt;
    int32_t baro_alt;
    float baro_climbrate;
    // LowPassFilterVector3f land_accel_ef_filter;


    // LowPassFilterFloat rc_throttle_control_in_filter;



    // Location_Class current_loc;



    uint8_t auto_yaw_mode;


    // Vector3f roi_WP;


    float yaw_look_at_WP_bearing;


    int32_t yaw_look_at_heading;


    int16_t yaw_look_at_heading_slew;


    float yaw_look_ahead_bearing;


    int32_t condition_value;
    uint32_t condition_start;




    float G_Dt;


    // AP_InertialNav_NavEKF inertial_nav;



    // AC_AttitudeControl *attitude_control;
    // AC_PosControl *pos_control;
    // AC_WPNav *wp_nav;
    // AC_Circle *circle_nav;


    int16_t pmTest1;




    uint32_t fast_loopTimer;

    uint16_t mainLoop_count;

    uint32_t rtl_loiter_start_time;

    uint32_t arm_time_ms;


    uint8_t auto_trim_counter;


    // AP_Relay relay;


    // AP_ServoRelayEvents ServoRelayEvents;



    // AP_Camera camera;





    // AP_Mount camera_mount;




    // AC_Fence fence;


    // AC_Avoid avoid;



    // AP_Rally_Copter rally;



    // AP_RSSI rssi;



    // AC_Sprayer sprayer;




    // AP_Parachute parachute;



    // AP_LandingGear landinggear;



    // AP_Terrain terrain;




    // AC_PrecLand precland;
    // AP_ADSB adsb {ahrs};


    // AP_Avoidance_Copter avoidance_adsb{ahrs, adsb};


    bool in_mavlink_delay;


    bool gcs_out_of_time;


    uint32_t last_radio_update_ms;



    // AP_Param param_loader;
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
};
