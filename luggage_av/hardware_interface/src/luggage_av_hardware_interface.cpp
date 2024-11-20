#include "luggage_av_hardware_interface/luggage_av_hardware_interface.hpp"

#include "hardware_interface/lexical_casts.hpp"

#include "defaults.hpp"
#include "wheel_commands.pb.h"
#include "wheel_states.pb.h"
#include "cobs.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cmath>
#include <termios.h>


namespace luggage_av {

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
        if(hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS) 
            return hardware_interface::CallbackReturn::ERROR;

        // Perform URDF checks
        //// Check if this system has exactly 2 joints
        if(get_hardware_info().joints.size() != 2) {
            RCLCPP_FATAL(
                get_logger(), "System '%s' has %zu joints. 2 expected.",
                get_hardware_info().name.c_str(), get_hardware_info().joints.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        for(hardware_interface::ComponentInfo& joint : info_.joints) {
            //// Check that only a single command interface exist for the joint
            if(joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the only existing command interface is a velocity interface
            if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that there are exactly 2 state interfaces
            if(joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
            
                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the first state interface is a position interface
            if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the second state interface is a velocity interface
            if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Initialize class memebers
        auto it = get_hardware_info().hardware_parameters.find("device");
        if(it != get_hardware_info().hardware_parameters.end()) {
            dev_ = const_cast<char*>(it->second.c_str());
        }
        else {
            dev_ = luggage_av_default_parameters.dev;
        }
        
        it = get_hardware_info().hardware_parameters.find("linear_velocity_min");
        if(it != get_hardware_info().hardware_parameters.end()) {
            lin_vel_min_ = hardware_interface::stod(it->second);
        }
        else {
            lin_vel_min_ = luggage_av_default_parameters.lin_vel_min;
        }

        it = get_hardware_info().hardware_parameters.find("linear_velocity_max");
        if(it != get_hardware_info().hardware_parameters.end()) {
            lin_vel_max_ = hardware_interface::stod(it->second);
        }
        else {
            lin_vel_max_ = luggage_av_default_parameters.lin_vel_max;
        }

        it = get_hardware_info().hardware_parameters.find("hardware_command_min");
        if(it != get_hardware_info().hardware_parameters.end()) {
            hw_cmd_min_ = hardware_interface::stod(it->second);
        }
        else {
            hw_cmd_min_ = luggage_av_default_parameters.hw_cmd_min;
        }

        it = get_hardware_info().hardware_parameters.find("hardware_command_max");
        if(it != get_hardware_info().hardware_parameters.end()) {
            hw_cmd_max_ = hardware_interface::stod(it->second);
        }
        else {
            hw_cmd_max_ = luggage_av_default_parameters.hw_cmd_max;
        }

        it = get_hardware_info().hardware_parameters.find("encoder_counts_per_revolution");
        if(it != get_hardware_info().hardware_parameters.end()) {
            enc_cpr_ = hardware_interface::stod(it->second);
        }
        else {
            enc_cpr_ = luggage_av_default_parameters.enc_cpr;
        }

        wheel_L_.velocity_command_interface_name = info_.joints[0].name + "/velocity";
        wheel_L_.position_state_interface_name = info_.joints[0].name + "/position";
        wheel_L_.velocity_state_interface_name = info_.joints[0].name + "/velocity";
        wheel_R_.velocity_command_interface_name = info_.joints[1].name + "/velocity";
        wheel_R_.position_state_interface_name = info_.joints[1].name + "/position";
        wheel_R_.velocity_state_interface_name = info_.joints[1].name + "/velocity";
        

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Setup communication with I/O processor

        poll_fd_.fd = open(dev_, O_RDWR | O_NOCTTY);

        if(poll_fd_.fd < 0) {
            RCLCPP_ERROR(get_logger(), "Unable to open %s", dev_);
            
            return hardware_interface::CallbackReturn::ERROR;
        }

        // TODO: TERMIOS configuration
        // tcflush(poll_fd_.fd, TCIOFLUSH);

        poll_fd_.events = POLLIN;

        RCLCPP_INFO(get_logger(), "Successfully opened %s", dev_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Unsetup communication with I/O processor
        if(poll_fd_.fd <= 0) return hardware_interface::CallbackReturn::SUCCESS; // The file descriptor was never opened

        if(close(poll_fd_.fd) < 0) {
            RCLCPP_ERROR(get_logger(), "Unable to close %s", dev_);

            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Successfully closed %s", dev_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Enable the hardware

        return hardware_interface::CallbackReturn::SUCCESS;
        
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Disable the hardware

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Shutdown hardware gracefully

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Handle errors

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    uint8_t in_buf[1024];
    uint8_t ws_msg_buf[256];
    carry_my_luggage::WheelStates ws_msg;

    hardware_interface::return_type LuggageAVHardawreInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // TODO: Read from I/O processor and update state interfaces

        // 1. Poll fd to see if data can be read
        int fd_ready = poll(&poll_fd_, 1, -1); // TODO: decide on a timeout

        // 2. if yes, read up to buffer length
        if(fd_ready < 0) {
            // error
            // TODO: BETTER LOGGING

            RCLCPP_ERROR(get_logger(), "Error when polling fd");

            return hardware_interface::return_type::ERROR;
        }
        if(fd_ready == 0) {
            // timeout
            // TODO: BETTER LOGGING

            printf("polling fd timed out");

            return hardware_interface::return_type::OK;
        }
        ssize_t bytes_read = ::read(poll_fd_.fd, in_buf, 256);

        // printf("Read %ld bytes\n", bytes_read);
        // for(ssize_t i = 0; i < bytes_read; i++) {
        //     printf("%d, ", in_buf[i]);
        // }
        // printf("\n");

        // 3. assume multiple messages in buffer, skip over until you get the last message
        ssize_t last2_zeros[2]{-2,-1};
        for(ssize_t i = 0; i < bytes_read; i++) {
            if(in_buf[i] == '\0') {
                last2_zeros[0] = last2_zeros[1];
                last2_zeros[1] = i;
            }
        }

        // printf("Last 2 zeros at %ld and %ld\n", last2_zeros[0], last2_zeros[1]);
        
        if(last2_zeros[0] < 0 || last2_zeros[1] < 0) {
            // We have to detect full delimiter 0s to make sure we get a complete message
            // TODO: BETTER LOGGING
            printf("Couldn't get a full complete message, skipping...\n");

            return hardware_interface::return_type::OK; 
        }

        // for(ssize_t i = last2_zeros[0]+1; i < last2_zeros[1]; i++) {
        //     printf("%d, ", in_buf[i]);
        // }
        // printf("\n");

        // 3. cobs decode
        cobs_decode_result decode_result = cobs_decode(ws_msg_buf, 256, in_buf+last2_zeros[0]+1, last2_zeros[1]-last2_zeros[0]-1);
        
        if(decode_result.status != cobs_decode_status::COBS_DECODE_OK) {
            // TODO: Better logging

            switch(decode_result.status) {
            case cobs_decode_status::COBS_DECODE_NULL_POINTER:
                RCLCPP_ERROR(get_logger(), "cobs_decode_status::COBS_DECODE_NULL_POINTER");
                break;
            case cobs_decode_status::COBS_DECODE_OUT_BUFFER_OVERFLOW:
                RCLCPP_ERROR(get_logger(), "cobs_decode_status::COBS_DECODE_OUT_BUFFER_OVERFLOW");
                break;
            case cobs_decode_status::COBS_DECODE_ZERO_BYTE_IN_INPUT:
                RCLCPP_ERROR(get_logger(), "cobs_decode_status::COBS_DECODE_ZERO_BYTE_IN_INPUT");
                break;
            case cobs_decode_status::COBS_DECODE_INPUT_TOO_SHORT:
                RCLCPP_ERROR(get_logger(), "cobs_decode_status::COBS_DECODE_INPUT_TOO_SHORT");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown error during cobs decode");
        }

            return hardware_interface::return_type::ERROR;
        }
        // 4. TODO: CRC
        // 5. protobuf decode
        
        if(!(ws_msg.ParseFromString(std::string((const char *)ws_msg_buf, decode_result.out_len)))) {
            // TODO: BETTER LOGGING
            RCLCPP_ERROR(get_logger(), "protobuf parsing resulted in an error");

            return hardware_interface::return_type::ERROR;
        }

        // 6. set states
        
        set_state(wheel_L_.position_state_interface_name, 2 * M_PI * ws_msg.position_left() / enc_cpr_);
        set_state(wheel_R_.position_state_interface_name, 2 * M_PI * ws_msg.position_right() / enc_cpr_);
        set_state(wheel_L_.velocity_state_interface_name, 2 * M_PI * ws_msg.velocity_left() / enc_cpr_);
        set_state(wheel_R_.velocity_state_interface_name, 2 * M_PI * ws_msg.velocity_right() / enc_cpr_);


        return hardware_interface::return_type::OK;
    }

    carry_my_luggage::WheelCommands wc_msg;
    std::string wc_msg_str;
    uint8_t out_buf[256];

    hardware_interface::return_type LuggageAVHardawreInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

        // 1. Read the commands from WheelInterfaces.velocity_commands with get_command()
        // 2. Update msg object
        
        wc_msg.set_velocity_left((hw_cmd_max_-hw_cmd_min_)*(get_command(wheel_L_.velocity_command_interface_name)-lin_vel_min_)/(lin_vel_max_-lin_vel_min_)+hw_cmd_min_);
        wc_msg.set_velocity_right((hw_cmd_max_-hw_cmd_min_)*(get_command(wheel_R_.velocity_command_interface_name)-lin_vel_min_)/(lin_vel_max_-lin_vel_min_)+hw_cmd_min_);
        // wc_msg.set_velocity_left(get_command(wheel_L_.velocity_command_interface_name)*1000);
        // wc_msg.set_velocity_right(get_command(wheel_R_.velocity_command_interface_name)*1000);
        
        // // // 3. encode protobuf message
        wc_msg.SerializeToString(&wc_msg_str);
    
        // 4. TODO: CRC
        // 5. Encode cobs
                                                                // TODO: Find out why out_buf.size() is not available
        cobs_encode_result encode_result = cobs_encode(out_buf, 256, wc_msg_str.c_str(), wc_msg_str.size());
        
        if(encode_result.status != cobs_encode_status::COBS_ENCODE_OK) {
            if(encode_result.status & cobs_encode_status::COBS_ENCODE_NULL_POINTER) {
                RCLCPP_ERROR(get_logger(), "A null pointer was passed to cobs_encode function");
            }
            if(encode_result.status & cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW) {
                RCLCPP_ERROR(get_logger(), "Buffer overflow detected when encoding COBS");
            }
            if(!(encode_result.status & (cobs_encode_status::COBS_ENCODE_NULL_POINTER | cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW))) {
                RCLCPP_ERROR(get_logger(), "Unknown error when encoding COBS");
            }

            return hardware_interface::return_type::ERROR;
        }

        // 6. Add a final \0
        if(encode_result.out_len == 256) {
            RCLCPP_ERROR(get_logger(), "Buffer overflow detected when appending final \\0");

            return hardware_interface::return_type::ERROR;
        }

        out_buf[encode_result.out_len] = 0;
        
        // 7. Write to poll_fd_.fd
        // TODO: Check number of bytes written
        ::write(poll_fd_.fd, out_buf, encode_result.out_len+1);

        return hardware_interface::return_type::OK;
    }
    
}  // namespace luggage_av

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(luggage_av::LuggageAVHardawreInterface, hardware_interface::SystemInterface);
 