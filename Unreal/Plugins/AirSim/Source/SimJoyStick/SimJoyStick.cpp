#include "SimJoyStick.h"

#if defined _WIN32 || defined _WIN64

#include "DirectInputJoystick.h"

struct SimJoyStick::impl
{
public:
    void getJoyStickState(int index, SimJoyStick::State& state, const AxisMaps& maps)
    {
        if (index >= kMaxControllers) {
            state.is_initialized = false;
            return;
        }

        if (controllers_[index] == nullptr) {
            controllers_[index].reset(new DirectInputJoyStick());
            if (!controllers_[index]->initialize(index)) {
                state.is_initialized = false;
                state.message = controllers_[index]->getState(false).message;
                return;
            }
            state.is_initialized = true;
        }

        const DirectInputJoyStick::JoystickState& di_state = controllers_[index]->getState();
        const DirectInputJoyStick::JoystickInfo& joystick_info = controllers_[index]->getJoystickInfo();

        state.is_valid = di_state.is_valid;

        state.left_x = getAxisValue(AxisMap::AxisType::LeftX, maps.left_x, di_state, joystick_info.pid_vid);
        state.left_y = getAxisValue(AxisMap::AxisType::LeftY, maps.left_y, di_state, joystick_info.pid_vid);
        state.right_x = getAxisValue(AxisMap::AxisType::RightX, maps.right_x, di_state, joystick_info.pid_vid);
        state.right_y = getAxisValue(AxisMap::AxisType::RightY, maps.right_y, di_state, joystick_info.pid_vid);
        state.right_z = getAxisValue(AxisMap::AxisType::RightZ, maps.right_z, di_state, joystick_info.pid_vid);
        state.left_z = getAxisValue(AxisMap::AxisType::LeftZ, maps.left_z, di_state, joystick_info.pid_vid);

        state.slider0 = static_cast<float>(di_state.slider0);
        state.slider1 = static_cast<float>(di_state.slider1);

        state.pov0 = static_cast<float>(di_state.pov0);
        state.pov1 = static_cast<float>(di_state.pov1);
        state.pov2 = static_cast<float>(di_state.pov2);
        state.pov3 = static_cast<float>(di_state.pov3);

        state.pid_vid = joystick_info.pid_vid;

        state.buttons = 0;
        for (int i = 0; i < sizeof(int) * 8; ++i) {
            state.buttons |= ((di_state.buttons[i] & 0x80) == 0 ? 0 : 1) << i;
        }
    }

    void setAutoCenter(int index, double strength)
    {
        if (index >= 0)
            controllers_[index]->setAutoCenter(strength);
    }

    void setWheelRumble(int index, double strength)
    {
        if (index >= 0)
            controllers_[index]->setWheelRumble(strength);
    }

private:
    float getMappedValue(AxisMap::AxisType axis_type, const AxisMap& map, const DirectInputJoyStick::JoystickState& di_state, const std::string& device_pid_vid)
    {
        AxisMap::AxisType rc_axis;
        if (map.rc_axis == AxisMap::AxisType::Auto) {
            if (device_pid_vid == "" || device_pid_vid == "VID_0483&PID_5710") { //RCs like FrSky Taranis
                switch (axis_type) {
                case AxisMap::AxisType::LeftX:
                    rc_axis = AxisMap::AxisType::RightX;
                    break;
                case AxisMap::AxisType::LeftY:
                    rc_axis = AxisMap::AxisType::LeftX;
                    break;
                case AxisMap::AxisType::LeftZ:
                    rc_axis = AxisMap::AxisType::RightY;
                    break;
                case AxisMap::AxisType::RightX:
                    rc_axis = AxisMap::AxisType::LeftY;
                    break;
                case AxisMap::AxisType::RightY:
                    rc_axis = AxisMap::AxisType::LeftZ;
                    break;
                case AxisMap::AxisType::RightZ:
                    rc_axis = AxisMap::AxisType::RightZ;
                    break;
                default:
                    throw std::invalid_argument("Unsupported axis_type in getMappedValue");
                }
            }
            else if (device_pid_vid == "VID_0401&PID_0401") { //Flysky FS-SM100 RC USB adapter
                switch (axis_type) {
                case AxisMap::AxisType::LeftX:
                    rc_axis = AxisMap::AxisType::RightY;
                    break;
                case AxisMap::AxisType::LeftY:
                    rc_axis = AxisMap::AxisType::LeftX;
                    break;
                case AxisMap::AxisType::LeftZ:
                    rc_axis = AxisMap::AxisType::RightX;
                    break;
                case AxisMap::AxisType::RightX:
                    rc_axis = AxisMap::AxisType::LeftY;
                    break;
                case AxisMap::AxisType::RightY:
                    rc_axis = AxisMap::AxisType::LeftZ;
                    break;
                case AxisMap::AxisType::RightZ:
                    rc_axis = AxisMap::AxisType::RightZ;
                    break;
                default:
                    throw std::invalid_argument("Unsupported axis_type in getMappedValue");
                }
            }
            else { //Xbox controllers
                rc_axis = axis_type;
            }
        }
        else
            rc_axis = map.rc_axis;

        long result;
        switch (rc_axis) {
        case AxisMap::AxisType::LeftX:
            result = di_state.x;
            break;
        case AxisMap::AxisType::LeftY:
            result = di_state.y;
            break;
        case AxisMap::AxisType::LeftZ:
            result = di_state.z;
            break;
        case AxisMap::AxisType::RightX:
            result = di_state.rx;
            break;
        case AxisMap::AxisType::RightY:
            result = di_state.ry;
            break;
        case AxisMap::AxisType::RightZ:
            result = di_state.rz;
            break;
        default:
            throw std::invalid_argument("Unsupported rc_axis in getMappedValue");
        }

        return static_cast<float>(result);
    }

    float getAxisValue(AxisMap::AxisType axis_type, const AxisMap& map, const DirectInputJoyStick::JoystickState& di_state, const std::string& device_pid_vid)
    {
        float val = getMappedValue(axis_type, map, di_state, device_pid_vid);

        //normalize min to max --> 0 to 1
        val = (val - map.min_val) / (map.max_val - map.min_val);

        switch (map.direction) {
        case AxisMap::AxisDirection::Auto:
            if (
                ((device_pid_vid == "" || device_pid_vid == "VID_0483&PID_5710") &&
                 (axis_type == AxisMap::AxisType::LeftZ || axis_type == AxisMap::AxisType::RightY)) ||
                ((device_pid_vid != "" && device_pid_vid != "VID_0483&PID_5710" && device_pid_vid != "VID_0401&PID_0401") &&
                 (axis_type == AxisMap::AxisType::LeftY)))
                val = 1 - val;
            break;
        case AxisMap::AxisDirection::Normal:
            break;
        case AxisMap::AxisDirection::Reverse:
            val = 1 - val;
            break;
        default:
            throw std::invalid_argument("Unsupported map.direction in getAxisValue");
        }

        //normalize 0 to 1 --> -1 to 1
        val = 2 * val - 1;

        return val;
    }

private:
    static constexpr unsigned int kMaxControllers = 4;
    std::unique_ptr<DirectInputJoyStick> controllers_[kMaxControllers];
};

#elif defined __linux__
// implementation for modern linux systems via /dev/input
// based on userspace api docs: https://www.kernel.org/doc/html/latest/input/input_uapi.html

#include <string>
#include <cstring>
#include <map>

extern "C" {
    #include <linux/version.h>
    #include <linux/input.h>
    #include <sys/stat.h>
    #include <dirent.h>
    #include <unistd.h>
    #include <fcntl.h>
}

#define DEV_INPUT "/dev/input"
#define SYS_CLASS_INPUT "/sys/class/input"

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)	((array[LONG(bit)] >> OFF(bit)) & 1)

struct SimJoyStick::impl
{
    impl() {
    }

    ~impl() {
        if (fd_ >= 0)
            close(fd_);
    }

    /**
     * scans /sys/class/input to get the event device number that coressponds with a given
     * joystick device number in effect this lets us figure out which event device file
     * corresponds to a specific joystick device file
     *
     * returns -1 if the specified js device number is not valid
     */
    static int getEventDeviceNumber(int jsDeviceNumber) {
        // unfortunately std::filesystem is a c++17 thing
        auto device_folder = SYS_CLASS_INPUT "/js" + std::to_string(jsDeviceNumber) + "/device";

        DIR *dir = opendir(device_folder.c_str());
        if (dir == NULL)
            return -1;

        int event_dev_num = -1;
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            auto name = std::string(entry->d_name);

            if(name.rfind("event", 0) == 0) {
                // bingo
                event_dev_num = std::stoi(name.substr(5, name.length()));
                break;
            }
        }
        closedir(dir);
        return event_dev_num;
    }

    static inline float normalize(int value, input_absinfo &info, bool reversed=false) {
        float val = (value - info.minimum) / ((float) (info.maximum - info.minimum));
        val = 2 * val - 1;

        if(reversed)
           val *= -1;

        return val;
    }

    void getJoyStickState(int index, SimJoyStick::State &state, const AxisMaps &maps) {
        unused(maps);

        if (index != last_index_) {
            // close previous one
            if (fd_ >= 0) {
                close(fd_);
                state.is_valid = false;
                state.is_initialized = false;
            }

            int devNo = getEventDeviceNumber(index);
            if(devNo < 0) {
                // joystick index specified either doesn't have an event interface
                // or is not a valid joystick
                last_index_ = index;
                goto getJoyStickState_device_missing_failure;
            }

            // open the device file and run ioctls to grab info about the device real quick
            auto devicePath = DEV_INPUT "/event" + std::to_string(devNo);
            fd_ = open(devicePath.c_str(), O_RDONLY|O_NONBLOCK);
            if(fd_ < 0) {
                goto getJoyStickState_device_missing_failure;
            }
            state.is_valid = true;

            // initialize the device and to the controller's default values
            // cache device ids and name
            if(ioctl(fd_, EVIOCGID, id)) {
                goto getJoyStickState_device_read_failure;
            }
            char buffer[128];
            if(ioctl(fd_, EVIOCGNAME(sizeof(name)), buffer) < 0) {
                goto getJoyStickState_device_read_failure;
            }
            name = buffer;

            // get list of valid EV_ABS codes and their related input_absinfo
            // and cache for later use when normalizing values, these seem to be the same but
            // it's possible for them to be different so we're going to cache it for later use
            unsigned long bits[NBITS(KEY_MAX)];
            ioctl(fd_, EVIOCGBIT(EV_ABS, KEY_MAX), bits);
            for(unsigned int code = 0; code < KEY_MAX; code++) {
                if(test_bit(code, bits)) {
                    input_absinfo abs;
                    if(ioctl(fd_, EVIOCGABS(code), &abs) == 0) { // errs when code is invalid
                        absinfo_map[code] = abs;
                    }
                }
            }

            intializeStateToDefaults(state);
            state.is_initialized = true;

            last_index_ = index;
        }

        // read all remaining events in buffer and process them
        if(fd_ > 0) {
            auto bytes_read = read(fd_, ev_, sizeof ev_);
            if(bytes_read != -1) {
                int count = bytes_read / sizeof(struct input_event);
                processInputEvents(state, count);

                // do this here so processInputEvents can set default values for state if necessary
                state.is_initialized = true;
            } else if (!(errno == EWOULDBLOCK || errno == EAGAIN)){
                // something went wrong
                goto getJoyStickState_device_read_failure;
            }
        } else {
            goto getJoyStickState_device_missing_failure;
        }

        return; // everything ran as correcty

getJoyStickState_device_read_failure:
        close(fd_);
getJoyStickState_device_missing_failure:
        fd_ = -1;
        name = "";
        memset(&id, 0, sizeof(id));
        absinfo_map.clear();
        return;
    }

    void setAutoCenter(unsigned int index, double strength)
    {
        //TODO: implement this
        unused(index);
        unused(strength);
    }

    void setWheelRumble(unsigned int index, double strength)
    {
        //TODO: implement this via FF_RUMBLE
        // see: https://www.kernel.org/doc/html/latest/input/ff.html
        unused(index);
        unused(strength);
    }

private:

    void intializeStateToDefaults(SimJoyStick::State &state) {
        if((id[ID_VENDOR] == 0x45e) && (id[ID_PRODUCT] == 0x2fd)) {
            // "Xbox One S Controller [Bluetooth]" gamepad
            // when RT and LT are not pressed, they should be treated as -1 for the this controller
            //
            // NOTE: it might make sense to set left_z and right_z to scale from 0 to 1 instead but
            // these axis are unused afaik, so it's left as is for now
            state.left_z = -1.0f;
            state.right_z = -1.0f;
        }
    }

    // update state by processing the first len input_event entries in ev_
    void processInputEvents(SimJoyStick::State &state, int len) {
        // TODO: add support to handle EV_SYN in the unlikely event that it occurs
        // see: https://www.kernel.org/doc/html/latest/input/event-codes.html#ev-syn

        /* to add support for a new game pad add to the if block below and match based on id[ID_VENDOR] and
         * id[ID_PRODUCT] and/or name; see the default implementation at the end for an example
         *
         * NOTE: if you are trying to figure out which events get triggered because your controller does
         * something non standard, the `evtest` is very helpful (available via apt)
         *
         * NOTE: we could loop through the events buffer backwards to optimize performance a little bit but the buffer
         * shouldn't really get filled up unless the device file isn't read often enough and there's nothing
         * to suggest that this is necessary
         */
        if((id[ID_VENDOR] == 0x45e) && (id[ID_PRODUCT] == 0x2fd)) {
            // "Xbox One S Controller [Bluetooth]" gamepad

            for(int i=0; i < len; i++) {
                auto &ev = ev_[i];
                switch(ev.type) {
                    case EV_ABS: {
                        switch(ev.code) {
                            case ABS_X: {state.left_x = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_Y: {state.left_y = normalize(ev.value, absinfo_map[ev.code], true);} break;
                            case ABS_BRAKE: {state.left_z = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_Z: {state.right_x = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_RZ: {state.right_y = normalize(ev.value, absinfo_map[ev.code], true);} break;
                            case ABS_GAS: {state.right_z = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_HAT0X: {
                                switch(ev.value) {
                                    case 1: {
                                        state.buttons |= (1 << 6); // DPAD_LEFT pressed
                                        state.buttons &= ~(1 << 7); // DPAD_RIGHT released
                                    } break;
                                    case 0: {
                                        state.buttons &= ~((1 << 6) & (1 << 7)); // DPAD_X released
                                    } break;
                                    case -1: {
                                        state.buttons &= ~(1 <<6); // DPAD_LEFT released
                                        state.buttons |= (1 <<7); // DPAD_RIGHT pressed
                                    } break;
                                }
                            } break;
                            case ABS_HAT0Y: {
                                switch(ev.value) {
                                    case 1: {
                                        state.buttons |= (1 << 5); // DPAD_DOWN pressed
                                        state.buttons &= ~(1 << 4); // DPAD_UP released
                                    } break;
                                    case 0: {
                                        state.buttons &= ~((1 << 4) & (1 << 5)); // DPAD_Y released
                                    } break;
                                    case -1: {
                                        state.buttons &= ~(1 << 5); // DPAD_DOWN released
                                        state.buttons |= (1 << 4); // DPAD_UP pressed
                                    } break;
                                }
                            } break;
                        }
                    } break;
                    case EV_KEY: {
                         uint16_t button_change = 1;
                         switch(ev.code) {
                            case BTN_A: {button_change <<= 0;} break;
                            case BTN_B: {button_change <<= 1;} break;
                            case BTN_X: {button_change <<= 2;} break;
                            case BTN_Y: {button_change <<= 3;} break;
                            case BTN_TL: {button_change <<= 8;} break;
                            case BTN_TR: {button_change <<= 9;} break;
                            case KEY_BACK: {button_change <<= 12;} break;
                            case BTN_START: {button_change <<= 13;} break;
                            case BTN_THUMBL: {button_change <<= 14;} break;
                            case BTN_THUMBR: {button_change <<= 15;} break;
                        }

                        if(ev.value == 0) {
                            state.buttons &= ~(button_change);
                        } else {
                            state.buttons |= button_change;
                        }
                    } break;
                }
            }
        } else {
            /* this is the default mapping, it essentially trusts that the linux driver mapped everything
             * correctly and should be a good starting point to copy
             *
             * based on the linux uapi docs: https://www.kernel.org/doc/html/latest/input/gamepad.html
             */

            for(int i=0; i < len; i++) {
                auto &ev = ev_[i];
                switch(ev.type) {
                    case EV_ABS: {
                        switch(ev.code) {
                            case ABS_X: {state.left_x = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_Y: {state.left_y = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_RX: {state.right_x = normalize(ev.value, absinfo_map[ev.code]);} break;
                            case ABS_RY: {state.right_y = normalize(ev.value, absinfo_map[ev.code]);} break;
                        }
                    } break;
                    case EV_KEY: {
                         uint16_t button_change = 1;
                         switch(ev.code) {
                            // NOTE: the position in state.buttons that each BTN is mapped to was an arbitrary choice
                            // but it's probably a good idea to stay consistent across controllers
                            case BTN_A: {button_change <<= 0;} break;
                            case BTN_B: {button_change <<= 1;} break;
                            case BTN_X: {button_change <<= 2;} break;
                            case BTN_Y: {button_change <<= 3;} break;

                            // The BTN_DPAD entires are valid in Ubuntu 22.04 LTS but they are not shipped as part of the
                            // HostLinux headers that come with UnrealEngine 5.4; the following defines are copied over
                            // from mainline linux/input-event-codes.h

                            #define BTN_DPAD_UP         0x220
                            #define BTN_DPAD_DOWN       0x221
                            #define BTN_DPAD_LEFT       0x222
                            #define BTN_DPAD_RIGHT      0x223

                            case BTN_DPAD_UP: {button_change <<= 4;} break;
                            case BTN_DPAD_DOWN: {button_change <<= 5;} break;
                            case BTN_DPAD_LEFT: {button_change <<= 6;} break;
                            case BTN_DPAD_RIGHT: {button_change <<= 7;} break;
                            case BTN_TL: {button_change <<= 8;} break;
                            case BTN_TR: {button_change <<= 9;} break;
                            case BTN_TL2: {button_change <<= 10;} break; // could map this to state.left_z instead
                            case BTN_TR2: {button_change <<= 11;} break; // could map this to state.right_z instead
                            case BTN_SELECT: {button_change <<= 12;} break;
                            case BTN_START: {button_change <<= 13;} break;
                            case BTN_THUMBL: {button_change <<= 14;} break;
                            case BTN_THUMBR: {button_change <<= 15;} break;
                       }

                        if(ev.value == 0) {
                            state.buttons &= ~(button_change);
                        } else {
                            state.buttons |= button_change;
                        }
                    } break;
                }
            }
        }
    }

    int last_index_ = -1;
    int fd_ = -1;
    struct input_event ev_[64]; // event cache; 64 is likely overkill but ram is cheap

    std::string name; // EVIOCGNAME response
    unsigned short id[4]; // EVIOCGID icotl response
    std::map<uint16_t, input_absinfo> absinfo_map;
};

#endif

SimJoyStick::SimJoyStick()
{
    pimpl_.reset(new impl());
}
SimJoyStick::~SimJoyStick()
{
    //required for pimpl
}

void SimJoyStick::getJoyStickState(int index, SimJoyStick::State& state) const
{
    if (index < 0) {
        state.is_initialized = false;
        state.is_valid = false;
        return;
    }

    //TODO: anyway to workaround const_cast?
    const_cast<SimJoyStick*>(this)->pimpl_->getJoyStickState(index, state, axis_maps);
}

void SimJoyStick::setAutoCenter(int index, double strength)
{
    pimpl_->setAutoCenter(index, strength);
}

void SimJoyStick::setWheelRumble(int index, double strength)
{
    pimpl_->setWheelRumble(index, strength);
}
