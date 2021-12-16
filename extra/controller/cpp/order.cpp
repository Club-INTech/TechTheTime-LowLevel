#include <functional>
#include <stdexcept>
#include <string>
#include <tuple>

#include <k2o/dispatcher.hpp>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include <order/controller.h>
#include <rpc/controller.hpp>

namespace py = pybind11;

static k2o::dispatcher dispatcher{rpc::controller::keyring};

static uint16_t last_time_us = 0;
static uint16_t last_tick = 0;

//
// pybind11 definitions
//

static py::object execute(const std::function<upd::byte_t()> &serial_input,
                          const std::function<void(upd::byte_t)> &serial_output) {
  using namespace std::string_literals;

  auto index = dispatcher(serial_input, serial_output);
  if (index != 0)
    throw std::out_of_range{"Received index"s + std::to_string(index) + " does not match any order"s};

  return py::cast(std::tuple{last_time_us, last_tick});
}

PYBIND11_MODULE(controller_order, m) {
  m.doc() = R"(
    Order sending and receiving with K2O
  )";
  m.def("execute", &execute, R"(
    Execute a received order
  )");
}

//
// K2O orders implementation
//

// Update the latest measures received
void controller_report_measures(uint16_t time_us, uint16_t tick) {
  last_time_us = time_us;
  last_tick = tick;
}
