#include "crossover.h"
#include "crossover_docs.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_crossover, m)
{
    m.def("selective_route_exchange",
          &pyvrp::crossover::selectiveRouteExchange,
          py::arg("parents"),
          py::arg("data"),
          py::arg("cost_evaluator"),
          py::arg("start_indices"),
          py::arg("num_moved_routes"),
          DOC(pyvrp, crossover, selectiveRouteExchange));
    m.def("heterogeneous_selective_route_exchange",
          &pyvrp::crossover::heterogeneousSelectiveRouteExchange,
          py::arg("parents"),
          py::arg("data"),
          py::arg("cost_evaluator"),
          py::arg("start_indices_per_vehicle_type"),
          py::arg("num_moved_routes_per_vehicle_type"),
          DOC(pyvrp, crossover, heterogeneousSelectiveRouteExchange));
    m.def("route_exchange",
          &pyvrp::crossover::routeExchange,
          py::arg("parents"),
          py::arg("data"),
          py::arg("cost_evaluator"),
          py::arg("exchanges"),
          DOC(pyvrp, crossover, routeExchange));
}
