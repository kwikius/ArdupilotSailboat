

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   Windvane example sketch
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <AP_WindVane/AP_WindVane.h>

#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   static AP_SerialManager serial_manager;
   AP_Int32 logger_bitmask;
   static AP_Logger logger{logger_bitmask};

   class DummyVehicle : public AP_Vehicle {
   public:
       AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
       bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; };
       uint8_t get_mode() const override { return 1; };
       void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) override {};
       void init_ardupilot() override {};
       void load_parameters() override {};
       void init() {
           BoardConfig.init();
           ins.init(100);
           ahrs.init();
       }
   };

   DummyVehicle vehicle;

   // choose which AHRS system to use
   // AP_AHRS_DCM ahrs = AP_AHRS_DCM::create(barometer, gps);
   auto &ahrs = vehicle.ahrs;

   // create airspeed object
   AP_WindVane windVane;

   // try to set the object value but provide diagnostic if it failed
   void set_object_value(const void *object_pointer,
                         const struct AP_Param::GroupInfo *group_info,
                         const char *name, float value)
   {
       if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
           hal.console->printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
                               group_info->name, name);
       }
   }

  static constexpr int pinAUX1 = 50;

  static constexpr int DirectionTypeAnalog = 3;

  static constexpr int directionType = DirectionTypeAnalog;

  //for explicit specialisation
  template <int Type> struct windVaneDirectionSetup;

  template <> struct windVaneDirectionSetup<DirectionTypeAnalog>{

    static constexpr int pin = 2;
    static constexpr float min_V = 0.001f;
    static constexpr float max_V = 3.299f;
    static constexpr float ofs_deg = 0.0f;
    static constexpr float lpFilt_Hz = 0.2f;
    static constexpr float deadZone_deg = 3.0f;

    static void apply()
    {
       set_object_value(&windVane, windVane.var_info, "TYPE", DirectionTypeAnalog);
       set_object_value(&windVane, windVane.var_info, "DIR_PIN", pin);
       set_object_value(&windVane, windVane.var_info, "DIR_V_MIN", min_V);
       set_object_value(&windVane, windVane.var_info, "DIR_V_MAX", max_V);
       set_object_value(&windVane, windVane.var_info, "DIR_OFS", ofs_deg);
       set_object_value(&windVane, windVane.var_info, "DIR_DZ", deadZone_deg);
    }
  };
}

// to be called only once on boot for initializing objects
void setup()
{
    vehicle.init();
    serial_manager.init();
    AP::compass().init();
    if (!AP::compass().read()) {
        hal.console->printf("No compass detected\n");
    }
    AP::gps().init(serial_manager);
    hal.console->printf("ArduPilot Wind Vane library test\n");

    windVaneDirectionSetup<directionType>::apply();

    windVane.init(serial_manager);




}

namespace {
    uint16_t counter = 0U;
    uint32_t last_t = 0U;
   // uint32_t last_print = 0U;
    uint32_t last_compass = 0U;
}
// loop
void loop(void)
{
    uint32_t const now = AP_HAL::micros();

   // float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    if (now - last_compass > 100 * 1000UL &&
        AP::compass().read()) {
       // auto const heading = AP::compass().calculate_heading(ahrs.get_rotation_body_to_ned());
        // read compass at 10Hz
        last_compass = now;
    }

    ahrs.update();
    counter++;

//    if (now - last_print >= 100000 /* 100ms : 10hz */) {
//        Vector3f drift  = ahrs.get_gyro_drift();
//        hal.console->printf(
//                "r:%4.1f  p:%4.1f y:%4.1f "
//                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n",
//                (double)ToDeg(ahrs.roll),
//                (double)ToDeg(ahrs.pitch),
//                (double)ToDeg(ahrs.yaw),
//                (double)ToDeg(drift.x),
//                (double)ToDeg(drift.y),
//                (double)ToDeg(drift.z),
//                (double)(AP::compass().use_for_yaw() ? ToDeg(heading) : 0.0f),
//                (double)((1.0e6f * counter) / (now-last_print)));
//        last_print = now;
//        counter = 0;
//    }

//-------------
    static uint32_t timer;

    // run read() and get_temperature() in 10Hz
    if ((AP_HAL::millis() - timer) > 100) {

        // current system time in milliseconds
        timer = AP_HAL::millis();
        windVane.update();

        hal.console->printf("apparent wind angle = %5.2f deg\n",
         static_cast<double>(wrap_360(degrees(windVane.get_apparent_wind_direction_rad()))));
    }
    hal.scheduler->delay(1);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
