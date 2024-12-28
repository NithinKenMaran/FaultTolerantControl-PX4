#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/actuator_armed.h>
#include <parameters/param.h>
#include <cmath>

bool auto_start{true};
class MotorFailureDetector : public ModuleBase<MotorFailureDetector>
{
public:
    MotorFailureDetector() = default;
    virtual ~MotorFailureDetector() = default;

    int print_status() override;
    static int print_usage();
    static int custom_command(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);
    static int task_main_trampoline(int argc, char *argv[]);
    int sim_gz_ec_max2 = 0;
    int alt_noted =0 ;

private:
    void task_main();
    bool _should_exit{false};
    int _sensor_combined_sub{-1};
    int _vehicle_local_position_sub{-1};

    float _previous_gyro[3] = {0.0f, 0.0f, 0.0f};
    uint64_t _previous_time = 0;
    float _current_altitude = 0.0f;
    int _step_size = 1;
    int _step_counter = 0;
    float _threshold = 12.5f;
    int _failure_window = 1;
    int _failure_count = 0;
    bool _did_motor_fail = false;
    int _failed_motor_number = -1;

    // Motor failure detection methods
    orb_advert_t actuator_armed_pub = nullptr;  // Publisher for actuator_armed message

    int check_motor_failure(float gyro[3]);
    void handle_motor_failure(int motor_index);  // Add the declaration here
    void disarm_vehicle();
};


int MotorFailureDetector::print_status()
{
    PX4_INFO("Motor failure detector module running.");
    return 0;
}


int MotorFailureDetector::task_main_trampoline(int argc, char *argv[])
{
    MotorFailureDetector *instance = new MotorFailureDetector();
    instance->task_main();
    delete instance;
    return 0;
}

void MotorFailureDetector::task_main()
{
    // Subscribe to uORB topics
    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position)); // Subscribe to vehicle_global_position
    int actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));  // Subscribe to actuator_armed topic
    _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    sensor_combined_s sensor_combined;
    vehicle_local_position_s vehicle_local_position;
    vehicle_global_position_s global_pos_data;
    actuator_armed_s actuator_armed_data;


    while (!_should_exit) {
        // Poll for sensor data
        px4_pollfd_struct_t fds[] = {
            {_sensor_combined_sub, POLLIN},
            {_vehicle_local_position_sub, POLLIN}
        };
        orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos_data);
        orb_copy(ORB_ID(actuator_armed), actuator_armed_sub, &actuator_armed_data);

        if(global_pos_data.alt > 0.5f)
        {
            int num = 0;
            param_set(param_find("SIM_GZ_EC_MIN1"), &num);
            param_set(param_find("SIM_GZ_EC_MIN2"), &num);
            param_set(param_find("SIM_GZ_EC_MIN3"), &num);
            param_set(param_find("SIM_GZ_EC_MIN4"), &num);
        };

        int poll_result = px4_poll(fds, 2, 100);  // 100 ms timeout

        if (poll_result > 0) {
            // Copy sensor data
            if (orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_combined) == PX4_OK &&
                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &vehicle_local_position) == PX4_OK)
            {
                // Process every 'step_size' messages
                _step_counter++;
                if (_step_counter >= _step_size) {
                    _step_counter = 0;  // Reset step counter

                    // Calculate angular acceleration (dω/dt)
                    uint64_t current_time = sensor_combined.timestamp;
                    float current_gyro[3] = {sensor_combined.gyro_rad[0], sensor_combined.gyro_rad[1], sensor_combined.gyro_rad[2]};

                    if (_previous_time != 0) {
                        float dt = (current_time - _previous_time) / 1e6f;  // Convert from microseconds to seconds
                        if (dt > 0.0f) {
                            float dwdt[3] = {
                                (current_gyro[0] - _previous_gyro[0]) / dt,
                                (current_gyro[1] - _previous_gyro[1]) / dt,
                                (current_gyro[2] - _previous_gyro[2]) / dt
                            };

                            // Log angular acceleration and altitude
                            if (!_did_motor_fail) {
                                //PX4_INFO("dω/dt: x=%lf, y=%lf, z=%lf rad/s², Altitude: %lf meters", (double)dwdt[0], (double)dwdt[1], (double)dwdt[2], (double)_current_altitude);

                            }

                            // Check for angular acceleration breaches
                            if (fabs((double)dwdt[0]) > (double)_threshold || fabs((double)dwdt[1]) > (double)_threshold  || fabs((double)dwdt[2]) > (double)_threshold ) {
                                _failure_count++;
                            } else {
                                _failure_count = 0;
                            }

                            // Confirm motor failure if enough breaches
                            if (_failure_count >= _failure_window && !_did_motor_fail) {
                                _did_motor_fail = true;
                                _failed_motor_number = check_motor_failure(dwdt);
                                PX4_WARN("Motor %d has failed", _failed_motor_number);
                                PX4_WARN("Motor %d has failed", _failed_motor_number);
                                PX4_WARN("Motor %d has failed", _failed_motor_number);
                                PX4_WARN("Motor %d has failed", _failed_motor_number);
                            }
                        } else {
                            PX4_WARN("Zero or negative time difference detected.");
                        }

                        if(_did_motor_fail && alt_noted == 0 ){
                            double x = (double)global_pos_data.alt;
                            if(x>=1 && x<=10)
                                {sim_gz_ec_max2 = -0.1058*(x*x*x*x*x) +2.909*(x*x*x*x) -29.57*(x*x*x) + 136.4*(x*x) - 281.4*x + 222.3 ;}

                            else if(x>10 && x<=18)
                                {sim_gz_ec_max2 = 0.4667*(x*x*x*x*x) -33.15*(x*x*x*x) +931.6*(x*x*x) -12940*(x*x) +88760*x -240500 ;}

                            else if(x>18 && x<=27)
                                {sim_gz_ec_max2 = -0.1122*(x*x*x*x*x) +12.86*(x*x*x*x) -586.3*(x*x*x) + 13280*(x*x) - 149500*x + 668500 ;}

                            else if(x>27 && x<36)
                                {sim_gz_ec_max2 = -0.02083*(x*x*x*x*x) +3.227*(x*x*x*x) -200.5*(x*x*x) + 6252*(x*x) - 97850*x + 61500 ;}

                            else if(x>36 && x<=48)
                                {sim_gz_ec_max2 = -0.01585*(x*x*x*x*x) +3.384*(x*x*x*x) -288.5*(x*x*x) + 12270*(x*x) - 260200*x + 2204000 ;}
                                alt_noted = 1;
                     }

                    }
                    // Update previous values for next cycle
                    _previous_gyro[0] = current_gyro[0];
                    _previous_gyro[1] = current_gyro[1];
                    _previous_gyro[2] = current_gyro[2];
                    _previous_time = current_time;
                }
            }
        }
        int disarmed = 0;
        // Publish failure status if detected
        if (_did_motor_fail && _failed_motor_number != -1 && disarmed ==0) {
            PX4_WARN("Motor %d has failed", _failed_motor_number);
            if (global_pos_data.alt < 0.1f) {
                                PX4_WARN("Altitude below 0.1 meter, disarming the vehicle!");
                                disarm_vehicle();
                                disarmed = 1;
                                }
                                PX4_WARN("Altitude: %.1f", (double)global_pos_data.alt);
        }

        px4_usleep(100000);  // Sleep for 100 ms
    }

    PX4_INFO("Motor Failure Detector exiting.");
}

void MotorFailureDetector::disarm_vehicle() {
    actuator_armed_s cmd;
    cmd.armed = false;  // Disarm the vehicle

    if (actuator_armed_pub == nullptr) {
        actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &cmd);
    }

    if (orb_publish(ORB_ID(actuator_armed), actuator_armed_pub, &cmd) < 0) {
        PX4_ERR("Failed to publish actuator_armed message");
    } else {
        PX4_WARN("Vehicle disarmed successfully");
    }
}

void MotorFailureDetector::handle_motor_failure(int motor_index) {
    // Log the failed motor
    PX4_WARN("Motor %d failure detected. Updating control allocation.", motor_index);

    int yaw = 0;
    param_set(param_find("MC_YAWRATE_K"), &yaw);
    param_set(param_find("MC_YAWRATE_P"), &yaw);
    param_set(param_find("MC_YAWRATE_I"), &yaw);
    param_set(param_find("MC_YAWRATE_D"), &yaw);
     //orb_copy(ORB_ID(actuator_armed), actuator_armed_sub, &actuator_armed_data);



    // Example: Adjust the parameter for the corresponding motor
    // Based on the failure logic, decrease the parameter for the other motor
    if (motor_index == 1) {
        param_set(param_find("SIM_GZ_EC_MAX2"), &sim_gz_ec_max2);  // Decrease motor 2's parameter
    } else if (motor_index == 2) {
        // Motor 2 failed, decrease the parameter for motor 1

        param_set(param_find("SIM_GZ_EC_MAX1"), &sim_gz_ec_max2);  // Decrease motor 1's parameter
    } else if (motor_index == 3) {
        // Motor 3 failed, decrease the parameter for motor 4

        param_set(param_find("SIM_GZ_EC_MAX4"), &sim_gz_ec_max2);  // Decrease motor 4's parameter
    } else if (motor_index == 4) {
        // Motor 4 failed, decrease the parameter for motor 3

        param_set(param_find("SIM_GZ_EC_MAX3"), &sim_gz_ec_max2); // Decrease motor 3's parameter
    }
}

int MotorFailureDetector::check_motor_failure(float gyro[3])
{
    float w_x = gyro[0], w_y = gyro[1];
     float thres = 2.0f;  // Threshold for motor check
    if (w_x > thres && w_y <  -thres) {
        return 1;
    } else if (w_x > thres && w_y > thres) {
        return 4;
    } else if (w_x < -thres && w_y < -thres) {
        return 3;
    } else if (w_x < -thres && w_y > thres) {
        return 2;
    }
    return -1;  // No failure detected
}

int MotorFailureDetector::print_usage()
{
    PX4_INFO("Usage: motor_failure_detector [options]");
    PX4_INFO("Options:");
    PX4_INFO("  -h        Show this help message");
    PX4_INFO("  -v        Enable verbose output");
    return 0;
}

int MotorFailureDetector::custom_command(int argc, char *argv[])
{
    // Example custom command handling
    if (argc > 1) {
        if (strcmp(argv[1], "help") == 0) {
            print_usage();
            return 0;
        }
    }

    // Default action if no valid command is found
    PX4_ERR("Unknown command");
    return -1;
}

int MotorFailureDetector::task_spawn(int argc, char *argv[])
{
    // spawn the task to monitor motor failure
    _task_id = px4_task_spawn_cmd("motor_failure_task", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, (px4_main_t)&MotorFailureDetector::task_main_trampoline, nullptr);

    if (_task_id < 0) {
        PX4_ERR("Task spawn failed");
        return -1;
    }

    PX4_INFO("Motor failure detection task spawned");
    return 0;
}

extern "C" __EXPORT int motor_failure_detection_main(int argc, char *argv[])
{
    return MotorFailureDetector::main(argc, argv);
}
