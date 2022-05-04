#include <functional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <k2o/dispatcher.hpp>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <order/controller.h>
#include <order/dxl.h>
#include <order/hub.h>
#include <order/motion.h>
#include <rpc/controller.hpp>
#include <rpc/def.hpp>
#include <rpc/master.hpp>

namespace py = pybind11;

static k2o::dispatcher dispatcher{rpc::controller::keyring};

enum HubMode : Shared_Hub_Mode { BASE = SHARED_HUB_MODE_BASE, TRACKER = SHARED_HUB_MODE_TRACKER };

static struct Measure {
  Measure() = default;
  Measure(Shared_Timestamp time_us, Shared_Tick left_encoder_ticks, Shared_Tick right_encoder_ticks)
      : time_us{time_us}, left_encoder_ticks{left_encoder_ticks}, right_encoder_ticks{right_encoder_ticks} {}

  Shared_Timestamp time_us;
  Shared_Tick left_encoder_ticks;
  Shared_Tick right_encoder_ticks;
} measure;

//
// pybind11 definitions
//
static py::object execute(const std::function<upd::byte_t()> &serial_input,
                          const std::function<void(upd::byte_t)> &serial_output) {
  using namespace std::string_literals;

  auto index = dispatcher(serial_input, serial_output);
  if (index != 0)
    throw std::out_of_range{"Received index"s + std::to_string(index) + " does not match any order"s};

  return py::cast(measure);
}

template <uint16_t I, typename R, typename... Args, auto... Options>
auto make_command(k2o::key<I, R(Args...), Options...> key) {
  using key_t = decltype(key);
  return [](Args &... args, const std::function<void(upd::byte_t)> &serial_output) {
    key_t{}(std::forward<Args &&>(args)...) >> serial_output;
  };
}

PYBIND11_MODULE(controller_rpc, m) {
  m.doc() = R"(
    Order sending and receiving with K2O
  )";
  m.def("execute", execute, R"(
    Execute a received order
  )");
  m.def("translate", [](Shared_Tick setpoint, const std::function<void(upd::byte_t)> &serial_output) {
    if (setpoint > 0) {
      rpc::master::keyring.get<Motion_Set_Forward_Translation_Setpoint>()(setpoint) >> serial_output;
    } else {
      rpc::master::keyring.get<Motion_Set_Backward_Translation_Setpoint>()(-setpoint) >> serial_output;
    }
  });
  m.def("rotate", [](Shared_Tick setpoint, const std::function<void(upd::byte_t)> &serial_output) {
    if (setpoint > 0) {
      rpc::master::keyring.get<Motion_Set_Clockwise_Rotation_Setpoint>()(setpoint) >> serial_output;
    } else {
      rpc::master::keyring.get<Motion_Set_Counterclockwise_Rotation_Setpoint>()(-setpoint) >> serial_output;
    }
  });
  m.def("set_translation_pid",
        [](double kp, double ki, double kd, const std::function<void(upd::byte_t)> &serial_output) {
          rpc::master::keyring.get<Motion_Set_Translation_PID>()(
              TO_SHARED_PID_K_FIXED_POINT(kp), TO_SHARED_PID_K_FIXED_POINT(ki), TO_SHARED_PID_K_FIXED_POINT(kd)) >>
              serial_output;
        });
  m.def("set_rotation_pid", [](double kp, double ki, double kd, const std::function<void(upd::byte_t)> &serial_output) {
    rpc::master::keyring.get<Motion_Set_Rotation_PID>()(
        TO_SHARED_PID_K_FIXED_POINT(kp), TO_SHARED_PID_K_FIXED_POINT(ki), TO_SHARED_PID_K_FIXED_POINT(kd)) >>
        serial_output;
  });
  m.def("set_left_pid", [](double kp, double ki, double kd, const std::function<void(upd::byte_t)> &serial_output) {
    rpc::master::keyring.get<Motion_Set_Left_PID>()(TO_SHARED_PID_K_FIXED_POINT(kp), TO_SHARED_PID_K_FIXED_POINT(ki),
                                                    TO_SHARED_PID_K_FIXED_POINT(kd)) >>
        serial_output;
  });
  m.def("set_right_pid", [](double kp, double ki, double kd, const std::function<void(upd::byte_t)> &serial_output) {
    rpc::master::keyring.get<Motion_Set_Right_PID>()(TO_SHARED_PID_K_FIXED_POINT(kp), TO_SHARED_PID_K_FIXED_POINT(ki),
                                                     TO_SHARED_PID_K_FIXED_POINT(kd)) >>
        serial_output;
  });
  m.def("start_joystick", make_command(rpc::master::keyring.get<Motion_Start_Joystick>()));
  m.def("set_joystick",
        [](Shared_Tick distance, Shared_Tick angle, const std::function<void(upd::byte_t)> &serial_output) {
          rpc::master::keyring.get<Motion_Set_Joystick>()(distance, angle) >> serial_output;
        });
  m.def("set_mode", make_command(rpc::master::keyring.get<Hub_Set_Mode>()));
  m.def("release_motor", make_command(rpc::master::keyring.get<Motion_Release>()));
  m.def("set_free_movement", [](Shared_PWM pwm, const std::function<void(upd::byte_t)> &serial_output) {
    rpc::master::keyring.get<Motion_Set_Free_Movement>()(pwm) >> serial_output;
  });
  m.def("dxl_position", [](uint8_t id, uint32_t position, const std::function<void(upd::byte_t)> &serial_output) {
    rpc::master::keyring.get<DXL_Position>()(id, position) >> serial_output;
  });
  m.def("dxl_position_angle",
        [](uint8_t id, uint32_t position_angle, const std::function<void(upd::byte_t)> &serial_output) {
          rpc::master::keyring.get<DXL_Position_Angle>()(id, position_angle) >> serial_output;
        });
  m.def("set_pump", make_command(rpc::master::keyring.get<Misc_Set_Pump>()));
  m.def("set_valve", make_command(rpc::master::keyring.get<Misc_Set_Valve>()));
  m.def("set_servo", make_command(rpc::master::keyring.get<Misc_Set_Servo>()));
  m.def("give_Res", make_command(rpc::master::keyring.get<giveRes>()));
  m.attr("HEADER") = std::vector<uint8_t>{0xff, 0xff, 0xff};
  py::enum_<rpc::Frame_Type>(m, "FrameType", py::arithmetic())
      .value("REQUEST", rpc::Frame_Type::REQUEST)
      .value("RESPONSE", rpc::Frame_Type::RESPONSE);
  py::enum_<HubMode>(m, "HubMode").value("BASE", HubMode::BASE).value("TRACKER", HubMode::TRACKER);
  py::class_<Measure>(m, "Measure")
      .def_property_readonly("time_us", [](const Measure &measure) { return measure.time_us; })
      .def_property_readonly("left_encoder_ticks", [](const Measure &measure) { return measure.left_encoder_ticks; })
      .def_property_readonly("right_encoder_ticks", [](const Measure &measure) { return measure.right_encoder_ticks; })
      .def(py::pickle(
          [](const Measure &measure) {
            return py::make_tuple(measure.time_us, measure.left_encoder_ticks, measure.right_encoder_ticks);
          },
          [](py::tuple py_tuple) {
            if (py_tuple.size() != 3)
              throw std::runtime_error("Couldn't unpickle a measure");
            return Measure{py_tuple[0].cast<Shared_Timestamp>(), py_tuple[1].cast<Shared_Tick>(),
                           py_tuple[2].cast<Shared_Tick>()};
          }))
      .doc() = R"(
      Holds the values measured by remote at a given moment
    )";
}

//
// K2O orders implementation
//

// Save the latest measure received
void Controller_Report_Measure(Shared_Timestamp time_us, Shared_Tick left_encoder_ticks,
                               Shared_Tick right_encoder_ticks) {
  measure.time_us = time_us;
  measure.left_encoder_ticks = left_encoder_ticks;
  measure.right_encoder_ticks = right_encoder_ticks;
}
