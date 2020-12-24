#include <pybind11/pybind11.h>

using namespace pybind11::literals;

int add(int i, int j) { return i + j; }

PYBIND11_MODULE(pybind01_Basic, m) {
    m.doc() = "pybind01, Basic example";
    m.def("add", &add, "A function which adds two numbers", "i"_a = 1, "j"_a = 2);
    // attributes
    m.attr("answer") = 42;
    m.attr("what") = pybind11::cast("World");
}