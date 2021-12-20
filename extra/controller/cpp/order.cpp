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

static struct Measure {
  Measure() = default;
  Measure(uint32_t time_us, uint32_t left_encoder_ticks, uint32_t right_encoder_ticks)
      : time_us{time_us}, left_encoder_ticks{left_encoder_ticks}, right_encoder_ticks{right_encoder_ticks} {}

  uint32_t time_us;
  uint32_t left_encoder_ticks;
  uint32_t right_encoder_ticks;
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

PYBIND11_MODULE(controller_order, m) {
  m.doc() = R"(
    Order sending and receiving with K2O
  )";
  m.def("execute", &execute, R"(
    Execute a received order
  )");
  py::class_<Measure>(m, "Measure")
      .def("time_us", [](const Measure &measure) { return measure.time_us; })
      .def("left_encoder_ticks", [](const Measure &measure) { return measure.left_encoder_ticks; })
      .def("right_encoder_ticks", [](const Measure &measure) { return measure.right_encoder_ticks; })
      .def(py::pickle(
          [](const Measure &measure) {
            return py::make_tuple(measure.time_us, measure.left_encoder_ticks, measure.right_encoder_ticks);
          },
          [](py::tuple py_tuple) {
            if (py_tuple.size() != 3)
              throw std::runtime_error("Couldn't unpickle a measure");
            return Measure{py_tuple[0].cast<uint32_t>(), py_tuple[1].cast<uint32_t>(), py_tuple[2].cast<uint32_t>()};
          }))
      .doc() = R"(
      Holds the values measured by remote at a given moment
    )";
}

//
// K2O orders implementation
//

// Save the latest measure received
void controller_report_measure(uint32_t time_us, uint32_t left_encoder_ticks, uint32_t right_encoder_ticks) {
  measure.time_us = time_us;
  measure.left_encoder_ticks = left_encoder_ticks;
  measure.right_encoder_ticks = right_encoder_ticks;
}
