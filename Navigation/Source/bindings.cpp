//
// Created by jraf2 on 8/09/2025.
//
// bindings.cpp
#include "pybind11/include/pybind11/pybind11.h"
#include "pybind11/include/pybind11/stl.h"
#include "main.h"


namespace py = pybind11;

PYBIND11_MODULE(navigation, m) {  // "mylib" will be the Python module name
    // py::class_<std::pair<int, int>>(m, "intPair")
    //     .def(py::init<>())
    //     .def_readwrite("first", &std::pair<int, int>::first)
    //     .def_readwrite("second", &std::pair<int, int>::second);

    py::class_<GoDog::EntityMover>(m, "EntityMover", "This class is responsible for providing movements to the entity to navigate it to the end position")
        .def(py::init<int, int, int, double>(),
            py::arg("end_relative_x"),
            py::arg("end_relative_y"),
            py::arg("obstacle_radius"),
            py::arg("max_straight_length") = -1,
            "Parameters:\n"
            "   end_relative_x (int): the x position of the goal relative to the entity's start position\n"
            "   end_relative_y (int): the y position of the goal relative to the entity's start position\n"
            "   obstacle_radius (int): the radius around an added point that is considered an obstacle\n"
            "   max_straight_length (int, optional): the magnitude of a provided movement; default -1")
        .def("update_map", &GoDog::EntityMover::updateMap,
            py::arg("entity_x"),
            py::arg("entity_y"),
            py::arg("added_obstacles"),
            py::arg("removed_obstacles"),
            "Updates the entity position and obstacles on the map\n\n"
            "Parameters\n"
            "   entity_x (int): the current x position of the entity\n"
            "   entity_y (int): the current y position of the entity\n"
            "   added_obstacles (list[tuple[int, int]]): a list of the positions of newly added obstacles\n"
            "   removed_obstacles (list[tuple[int, int]]): a list of the positions of newly removed obstacles")
        .def("get_entity_position", &GoDog::EntityMover::getEntityPosition,
            "Returns the current position of the entity")
        .def("get_next_movement", &GoDog::EntityMover::getNextMovement,
            py::arg("entity_x"),
            py::arg("entity_y"),
            py::arg("angle_adjustment") = 0.0,
            "Gets the next movement for the entity\n\n"
            "Parameters\n"
            "   entity_x (int): the current x position of the entity\n"
            "   entity_y (int): the current y position of the entity\n"
            "   angle_adjustment (double, optional): the angle_adjustment required for the entity to be facing upwards on the map; default 0.0\n"
            "Returns\n"
            "   EntityMovement: the movement containing angle and magnitude");

    py::class_<GoDog::MovementVector>(m, "EntityMovement", "This class holds the angle and magnitude of a movement")
        .def(py::init<double, double>(),
            py::arg("angle"),
            py::arg("magnitude"),
            "Parameters:\n"
            "   angle (double): the angle of the movement"
            "   magnitude (double): the magnitude of the movement")
        .def(py::init<std::pair<int, int>, std::pair<int, int>, double>(),
            py::arg("start_position"),
            py::arg("end_position"),
            py::arg("angle_adjustment") = 0.0,
            "Parameters:\n"
            "   start_position (int): the start position of the movement"
            "   end_position (int): the end position of the movement"
            "   angle_adjustment (double, optional): the angle_adjustment required for the entity to be facing upwards on the map; default 0.0\n")
        .def("get_angle", &GoDog::MovementVector::getAngle,
            "Returns\n"
            "   double: the angle of the movement")
        .def("get_magnitude", &GoDog::MovementVector::getMagnitude,
            "Returns\n"
            "   double: the magnitude of the movement");
}