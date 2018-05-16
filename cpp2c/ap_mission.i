//#include <stdio.h>
//#include <stdlib.h>
//#include <sys/types.h>
//#include <unistd.h>
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
 
// the struct of AP_AHRS
struct AP_AHRS
{
/*
public:
    friend class AP_AHRS_View;


    AP_AHRS(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps) :
        roll(0.0f),
        pitch(0.0f),
        yaw(0.0f),
        roll_sensor(0),
        pitch_sensor(0),
        yaw_sensor(0),
        _vehicle_class(AHRS_VEHICLE_UNKNOWN),
        _compass(nullptr),
        _optflow(nullptr),
        _airspeed(nullptr),
        _beacon(nullptr),
        _compass_last_update(0),
        _ins(ins),
        _baro(baro),
        _gps(gps),
        _cos_roll(1.0f),
        _cos_pitch(1.0f),
        _cos_yaw(1.0f),
        _sin_roll(0.0f),
        _sin_pitch(0.0f),
        _sin_yaw(0.0f),
        _active_accel_instance(0)
    {

        AP_Param::setup_object_defaults(this, var_info);



        _gyro_drift_limit = ins.get_gyro_drift_rate();


        _flags.correct_centrifugal = true;


        _home.options = 0;
        _home.alt = 0;
        _home.lng = 0;
        _home.lat = 0;

        _last_trim = _trim.get();
        _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, 0.0f);
        _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
    }

*/


/*
    virtual ~AP_AHRS() {}


    virtual void init() {
        set_orientation();
    };


    void set_fly_forward(bool b) {
        _flags.fly_forward = b;
    }

    bool get_fly_forward(void) const {
        return _flags.fly_forward;
    }






    void set_likely_flying(bool b) {
        if (b && !_flags.likely_flying) {
            _last_flying_ms = AP_HAL::millis();
        }
        _flags.likely_flying = b;
    }






    bool get_likely_flying(void) const {
        return _flags.likely_flying;
    }





    uint32_t get_time_flying_ms(void) const {
        if (!_flags.likely_flying) {
            return 0;
        }
        return AP_HAL::millis() - _last_flying_ms;
    }

    AHRS_VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    void set_vehicle_class(AHRS_VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    void set_wind_estimation(bool b) {
        _flags.wind_estimation = b;
    }

    void set_compass(Compass *compass) {
        _compass = compass;
        set_orientation();
    }

    const Compass* get_compass() const {
        return _compass;
    }

    void set_optflow(const OpticalFlow *optflow) {
        _optflow = optflow;
    }

    const OpticalFlow* get_optflow() const {
        return _optflow;
    }



    void set_orientation() {
        _ins.set_board_orientation((enum Rotation)_board_orientation.get());
        if (_compass != nullptr) {
            _compass->set_board_orientation((enum Rotation)_board_orientation.get());
        }
    }

    void set_airspeed(AP_Airspeed *airspeed) {
        _airspeed = airspeed;
    }

    void set_beacon(AP_Beacon *beacon) {
        _beacon = beacon;
    }

    const AP_Airspeed *get_airspeed(void) const {
        return _airspeed;
    }

    const AP_Beacon *get_beacon(void) const {
        return _beacon;
    }

    const AP_GPS &get_gps() const {
        return _gps;
    }

    const AP_InertialSensor &get_ins() const {
        return _ins;
    }

    const AP_Baro &get_baro() const {
        return _baro;
    }


    virtual uint8_t get_primary_accel_index(void) const {
        return _ins.get_primary_accel();
    }


    virtual uint8_t get_primary_gyro_index(void) const {
        return _ins.get_primary_gyro();
    }


    virtual const Vector3f &get_accel_ef(uint8_t i) const {
        return _accel_ef[i];
    }
    virtual const Vector3f &get_accel_ef(void) const {
        return get_accel_ef(_ins.get_primary_accel());
    }


    virtual const Vector3f &get_accel_ef_blended(void) const {
        return _accel_ef_blended;
    }


    float get_yaw_rate_earth(void) const {
        return get_gyro() * get_rotation_body_to_ned().c;
    }


    virtual void update(bool skip_ins_update=false) = 0;


    virtual const char *prearm_failure_reason(void) const {
        return nullptr;
    }


    virtual bool have_ekf_logging(void) const {
        return false;
    }

*/
    float roll;
    float pitch;
    float yaw;


    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

/*
    virtual const Vector3f &get_gyro(void) const = 0;


    Vector3f get_gyro_latest(void) const;


    virtual const Vector3f &get_gyro_drift(void) const = 0;



    virtual void reset_gyro_drift(void) = 0;


    virtual void reset(bool recover_eulers=false) = 0;


    virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;



    virtual float get_error_rp(void) const = 0;



    virtual float get_error_yaw(void) const = 0;



    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;
    const Matrix3f& get_rotation_autopilot_body_to_vehicle_body(void) const { return _rotation_autopilot_body_to_vehicle_body; }
    const Matrix3f& get_rotation_vehicle_body_to_autopilot_body(void) const { return _rotation_vehicle_body_to_autopilot_body; }



    virtual bool get_position(struct Location &loc) const = 0;


    virtual Vector3f wind_estimate(void) = 0;



    virtual bool airspeed_estimate(float *airspeed_ret) const;



    bool airspeed_estimate_true(float *airspeed_ret) const {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        *airspeed_ret *= get_EAS2TAS();
        return true;
    }


    float get_EAS2TAS(void) const {
        if (_airspeed) {
            return _airspeed->get_EAS2TAS();
        }
        return 1.0f;
    }



    bool airspeed_sensor_enabled(void) const {
        return _airspeed != nullptr && _airspeed->use() && _airspeed->healthy();
    }


    virtual Vector2f groundspeed_vector(void);




    virtual bool get_velocity_NED(Vector3f &vec) const {
        return false;
    }


    virtual bool get_expected_mag_field_NED(Vector3f &ret) const {
        return false;
    }


    virtual bool get_mag_field_correction(Vector3f &ret) const {
        return false;
    }




    virtual bool get_relative_position_NED_home(Vector3f &vec) const {
        return false;
    }




    virtual bool get_relative_position_NED_origin(Vector3f &vec) const {
        return false;
    }


    virtual bool get_relative_position_NE_home(Vector2f &vecNE) const {
        return false;
    }



    virtual bool get_relative_position_NE_origin(Vector2f &vecNE) const {
        return false;
    }



    virtual void get_relative_position_D_home(float &posD) const = 0;



    virtual bool get_relative_position_D_origin(float &posD) const {
        return false;
    }


    float groundspeed(void) {
        return groundspeed_vector().length();
    }


    virtual bool use_compass(void) {
        return _compass && _compass->use_for_yaw();
    }


    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
    }



    void set_correct_centrifugal(bool setting) {
        _flags.correct_centrifugal = setting;
    }


    bool get_correct_centrifugal(void) const {
        return _flags.correct_centrifugal;
    }


    const Vector3f &get_trim() const {
        return _trim.get();
    }


    virtual void set_trim(Vector3f new_trim);


    virtual void add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);


    float cos_roll() const {
        return _cos_roll;
    }
    float cos_pitch() const {
        return _cos_pitch;
    }
    float cos_yaw() const {
        return _cos_yaw;
    }
    float sin_roll() const {
        return _sin_roll;
    }
    float sin_pitch() const {
        return _sin_pitch;
    }
    float sin_yaw() const {
        return _sin_yaw;
    }


    static const struct AP_Param::GroupInfo var_info[];


    virtual bool get_secondary_attitude(Vector3f &eulers) {
        return false;
    }


    virtual bool get_secondary_quaternion(Quaternion &quat) {
        return false;
    }


    virtual bool get_secondary_position(struct Location &loc) {
        return false;
    }



    const struct Location &get_home(void) const {
        return _home;
    }




    virtual void set_home(const Location &loc) = 0;




    virtual bool set_origin(const Location &loc) { return false; }


    virtual bool get_origin(Location &ret) const { return false; }



    virtual bool have_inertial_nav(void) const {
        return false;
    }


    uint8_t get_active_accel_instance(void) const {
        return _active_accel_instance;
    }


    virtual bool healthy(void) const = 0;


    virtual bool initialised(void) const {
        return true;
    };



    virtual uint32_t getLastYawResetAngle(float &yawAng) const {
        return 0;
    };



    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) const {
        return 0;
    };



    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const {
        return 0;
    };



    virtual uint32_t getLastPosDownReset(float &posDelta) const {
        return 0;
    };






    virtual bool resetHeightDatum(void) {
        return false;
    }





    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const {
        return false;
    }


    virtual uint32_t uptime_ms(void) const = 0;


    int8_t get_ekf_type(void) const {
        return _ekf_type;
    }


    virtual void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const { ret.zero(); _ins.get_delta_velocity(ret); dt = _ins.get_delta_velocity_dt(); }


    AP_AHRS_View *create_view(enum Rotation rotation);


    float getAOA(void);


    float getSSA(void);

    virtual void update_AOA_SSA(void);

protected:
    AHRS_VehicleClass _vehicle_class;

*/

    //AP_Float _kp_yaw;
    float _kp_yaw;
   // AP_Float _kp;
   float _kp;
    //AP_Float gps_gain;
    float gps_gain;

    //AP_Float beta;
    float beta;
    //AP_Int8 _gps_use;
    int8_t _gps_use;
    //AP_Int8 _wind_max;
    int8_t _wind_max;
    //AP_Int8 _board_orientation;
    int8_t _board_orientation;
    //AP_Int8 _gps_minsats;
    int8_t _gps_minsats;
    //AP_Int8 _gps_delay;
    int8_t _gps_delay;
    //AP_Int8 _ekf_type;
    int8_t _ekf_type;


    
    struct ahrs_flags {
        uint8_t have_initial_yaw : 1;
        uint8_t fly_forward : 1;
        uint8_t correct_centrifugal : 1;
        uint8_t wind_estimation : 1;
        uint8_t likely_flying : 1;
    } _flags;


    uint32_t _last_flying_ms;

/*
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;



    void update_trig(void);


    void update_cd_values(void);

*/

   // Compass * _compass;


   // const OpticalFlow *_optflow;


    //AP_Airspeed * _airspeed;


    //AP_Beacon * _beacon;


    uint32_t _compass_last_update;



   // AP_InertialSensor &_ins;
   // AP_Baro &_baro;
   // const AP_GPS &_gps;


   // AP_Vector3f _trim;


   // Vector3f _last_trim;
   // Matrix3f _rotation_autopilot_body_to_vehicle_body;
   // Matrix3f _rotation_vehicle_body_to_autopilot_body;



    float _gyro_drift_limit;


   // Vector3f _accel_ef[3];
   // Vector3f _accel_ef_blended;



    //Vector2f _lp;
    //Vector2f _hp;
    //Vector2f _lastGndVelADS;


   // struct Location _home;


    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;


    uint8_t _active_accel_instance;


    //AP_AHRS_View *_view;


    float _AOA, _SSA;
    uint32_t _last_AOA_update_ms;
};

typedef struct AP_AHRS AP_AHRS;

    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_COMPLETE=2
    };

    typedef enum mission_state mission_state;



// AP_Mission

//class AP_Mission {
struct AP_Mission{
//public:
/*
    struct __attribute__((__packed__)) Jump_Command {
        uint16_t target;
        int16_t num_times;
    };


    struct __attribute__((__packed__)) Conditional_Delay_Command {
        float seconds;
    };


    struct __attribute__((__packed__)) Conditional_Distance_Command {
        float meters;
    };


    struct __attribute__((__packed__)) Yaw_Command {
        float angle_deg;
        float turn_rate_dps;
        int8_t direction;
        uint8_t relative_angle;
    };


    struct __attribute__((__packed__)) Change_Speed_Command {
        uint8_t speed_type;
        float target_ms;
        float throttle_pct;
    };


    struct __attribute__((__packed__)) Set_Relay_Command {
        uint8_t num;
        uint8_t state;
    };


    struct __attribute__((__packed__)) Repeat_Relay_Command {
        uint8_t num;
        int16_t repeat_count;
        float cycle_time;
    };


    struct __attribute__((__packed__)) Set_Servo_Command {
        uint8_t channel;
        uint16_t pwm;
    };


    struct __attribute__((__packed__)) Repeat_Servo_Command {
        uint8_t channel;
        uint16_t pwm;
        int16_t repeat_count;
        float cycle_time;
    };


    struct __attribute__((__packed__)) Mount_Control {
        float pitch;
        float roll;
        float yaw;
    };


    struct __attribute__((__packed__)) Digicam_Configure {
        uint8_t shooting_mode;
        uint16_t shutter_speed;
        uint8_t aperture;
        uint16_t ISO;
        uint8_t exposure_type;
        uint8_t cmd_id;
        float engine_cutoff_time;
    };


    struct __attribute__((__packed__)) Digicam_Control {
        uint8_t session;
        uint8_t zoom_pos;
        int8_t zoom_step;
        uint8_t focus_lock;
        uint8_t shooting_cmd;
        uint8_t cmd_id;
    };


    struct __attribute__((__packed__)) Cam_Trigg_Distance {
        float meters;
    };


    struct __attribute__((__packed__)) Gripper_Command {
        uint8_t num;
        uint8_t action;
    };


    struct __attribute__((__packed__)) Altitude_Wait {
        float altitude;
        float descent_rate;
        uint8_t wiggle_time;
    };


    struct __attribute__((__packed__)) Guided_Limits_Command {

        float alt_min;
        float alt_max;
        float horiz_max;
    };


    struct __attribute__((__packed__)) Do_VTOL_Transition {
        uint8_t target_state;
    };


    struct __attribute__((__packed__)) Navigation_Delay_Command {
        float seconds;
        int8_t hour_utc;
        int8_t min_utc;
        int8_t sec_utc;
    };


    struct __attribute__((__packed__)) Do_Engine_Control {
        bool start_control;
        bool cold_start;
        uint16_t height_delay_cm;
    };


    struct __attribute__((__packed__)) Set_Yaw_Speed {
        float angle_deg;
        float speed;
        uint8_t relative_angle;
    };


    struct __attribute__((__packed__)) Winch_Command {
        uint8_t num;
        uint8_t action;
        float release_length;
        float release_rate;
    };
*/
/*
    union __attribute__((__packed__)) Content {

        Jump_Command jump;


        Conditional_Delay_Command delay;


        Conditional_Distance_Command distance;


        Yaw_Command yaw;


        Change_Speed_Command speed;


        Set_Relay_Command relay;


        Repeat_Relay_Command repeat_relay;


        Set_Servo_Command servo;


        Repeat_Servo_Command repeat_servo;


        Mount_Control mount_control;


        Digicam_Configure digicam_configure;


        Digicam_Control digicam_control;


        Cam_Trigg_Distance cam_trigg_dist;


        Gripper_Command gripper;


        Guided_Limits_Command guided_limits;


        Altitude_Wait altitude_wait;


        Do_VTOL_Transition do_vtol_transition;


        Do_Engine_Control do_engine_control;


        Navigation_Delay_Command nav_delay;


        Set_Yaw_Speed set_yaw_speed;


        Winch_Command winch;


        Location location;



        uint8_t bytes[12];
    };
*/

    struct Mission_Command {
        uint16_t index;
        uint16_t id;
        uint16_t p1;
 //       Content content;


        //const char *type() const;
        
    };



    //typedef Functor<bool, const Mission_Command&> mission_cmd_fn_t;
    //typedef Functor<void> mission_complete_fn_t;




   /*
   static AP_Mission create(AP_AHRS &ahrs,
                             mission_cmd_fn_t cmd_start_fn,
                             mission_cmd_fn_t cmd_verify_fn,
                             mission_complete_fn_t mission_complete_fn) {
        return AP_Mission(ahrs, cmd_start_fn, cmd_verify_fn, mission_complete_fn);
    }

    constexpr AP_Mission(AP_Mission &&other) = default;


    AP_Mission(const AP_Mission &other) = delete;
    AP_Mission &operator=(const AP_Mission&) = delete;






    void init();


    mission_state state() const { return _flags.state; }



    uint16_t num_commands() const { return _cmd_total; }


    uint16_t num_commands_max() const;



    void start();


    void stop();



    void resume();


    void start_or_resume();


    bool starts_with_takeoff_cmd();


    void reset();



    bool clear();


    void truncate(uint16_t index);



    void update();
# 358 "/Users/yu/repos/ardupilot/libraries/AP_Mission/AP_Mission.h"
    bool add_cmd(Mission_Command& cmd);




    bool replace_cmd(uint16_t index, Mission_Command& cmd);


    static bool is_nav_cmd(const Mission_Command& cmd);


    const Mission_Command& get_current_nav_cmd() const { return _nav_cmd; }




    uint16_t get_current_nav_index() const {
        return _nav_cmd.index==65535?0:_nav_cmd.index; }




    uint16_t get_prev_nav_cmd_id() const { return _prev_nav_cmd_id; }




    uint16_t get_prev_nav_cmd_index() const { return _prev_nav_cmd_index; }




    uint16_t get_prev_nav_cmd_with_wp_index() const { return _prev_nav_cmd_wp_index; }




    bool get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd);




    int32_t get_next_ground_course_cd(int32_t default_angle);


    const Mission_Command& get_current_do_cmd() const { return _do_cmd; }


    bool set_current_cmd(uint16_t index);



    bool read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const;




    bool write_cmd_to_storage(uint16_t index, Mission_Command& cmd);



    void write_home_to_storage();



    static MAV_MISSION_RESULT mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd);
    static MAV_MISSION_RESULT mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd);



    static MAV_MISSION_RESULT mavlink_cmd_long_to_mission_cmd(const mavlink_command_long_t& packet, AP_Mission::Mission_Command& cmd);



    static bool mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet);
    static bool mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet);


    uint32_t last_change_time_ms(void) const { return _last_change_time_ms; }




    uint16_t get_landing_sequence_start();




    bool jump_to_landing_sequence(void);


    static const struct AP_Param::GroupInfo var_info[];
*/
//private:
//    static StorageAccess _storage;

    struct Mission_Flags {
        mission_state state;
        uint8_t nav_cmd_loaded : 1;
        uint8_t do_cmd_loaded : 1;
        uint8_t do_cmd_all_done : 1;
    } _flags;

/* 
    // constructor:
    AP_Mission(AP_AHRS &ahrs, mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
        _ahrs(ahrs),
        _cmd_start_fn(cmd_start_fn),
        _cmd_verify_fn(cmd_verify_fn),
        _mission_complete_fn(mission_complete_fn),
        _prev_nav_cmd_id(0),
        _prev_nav_cmd_index(65535),
        _prev_nav_cmd_wp_index(65535),
        _last_change_time_ms(0)
    {

        AP_Param::setup_object_defaults(this, var_info);


        _nav_cmd.index = 65535;
        _do_cmd.index = 65535;


        _flags.state = MISSION_STOPPED;
        _flags.nav_cmd_loaded = false;
        _flags.do_cmd_loaded = false;
    }

*/



/*

    void complete();





    bool advance_current_nav_cmd();




    void advance_current_do_cmd();





    bool get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found);





    bool get_next_do_cmd(uint16_t start_index, Mission_Command& cmd);





    void init_jump_tracking();



    int16_t get_jump_times_run(const Mission_Command& cmd);


    void increment_jump_times_run(Mission_Command& cmd);



    void check_eeprom_version();
*/


// TODO!!!
 // const AP_AHRS& _ahrs;
 // const AP_AHRS * _ahrs;

// AP_ParamT object
    //AP_Int16 _cmd_total;
    int16_t _cmd_total;
   // AP_Int8 _restart;
   int8_t _restart;
    //AP_Int16 _options;
    int16_t _options;


    //mission_cmd_fn_t _cmd_start_fn;
    //mission_cmd_fn_t _cmd_verify_fn;
    //mission_complete_fn_t _mission_complete_fn;


    struct Mission_Command _nav_cmd;
    struct Mission_Command _do_cmd;
    uint16_t _prev_nav_cmd_id;
    uint16_t _prev_nav_cmd_index;
    uint16_t _prev_nav_cmd_wp_index;

/*
    struct jump_tracking_struct {
        uint16_t index;
        int16_t num_times_run;
    } _jump_tracking[15];

*/
    uint32_t _last_change_time_ms;
};


int main(){

    struct AP_Mission myApMission;

    {
        SERIALIZE:
    }
    {
        DESERIALIZE:
    }

    return 0;
}



