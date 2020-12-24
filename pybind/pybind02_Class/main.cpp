#include <pybind11/pybind11.h>
#include <string>

using namespace std;
namespace py = pybind11;
using namespace pybind11::literals;

class Pet {
  public:
    Pet(const std::string& name) : name_(name) {}

    void setName(const std::string& name) { name_ = name; }

    const std::string& getName() const { return name_; }

  private:
    string name_;
};

class Dog : public Pet {
  public:
    Dog(const string& name) : Pet(name) {}

    string bark() const { return "woof!"; }
};

PYBIND11_MODULE(pybind02_Class, m) {
    m.doc() = "pybind02, Use class";

    py::class_<Pet>(m, "Pet")
        // dynamic attributes must has a default constructor
        // py::class_<Pet>(m, "Pet", py::dynamic_attr())
        //     .def(py::init<>())
        .def(py::init<const string&>())
        .def("setName", &Pet::setName)
        .def("getName", &Pet::getName)
        // repr and lambda functions
        .def("__repr__", [](const Pet& p) { return "<pybind02.Pet named \"" + p.getName() + "\">"; })
        // define internal variable
        // .def_readwrite("name", &Pet::name_)
        // define property
        .def_property("name", &Pet::getName, &Pet::setName);

    // define inherit class
    // another method see in
    // https://pybind11.readthedocs.io/en/stable/classes.html#inheritance-and-automatic-downcasting
    py::class_<Dog, Pet>(m, "Dog").def(py::init<const string&>()).def("bark", &Dog::bark);
}