#include <cstdint>

namespace luggage_av {

    struct {
        char* dev = const_cast<char*>("/dev/ttyACM0");
        double lin_vel_min = -1.0;
        double lin_vel_max = 1.0;
        int32_t hw_cmd_min = -8312;
        int32_t hw_cmd_max = 8312;
        uint32_t enc_cpr = 109809;
    } luggage_av_default_parameters;

}  // namespace luggage_av
