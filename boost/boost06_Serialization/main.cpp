#include <fmt/format.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include "common/common.hpp"

using namespace std;
using namespace boost::archive;
using namespace fmt;
using namespace common;

// Animal class
class Animal {
  public:
    Animal() = default;
    Animal(const string& name) : name_(name) {}
    virtual ~Animal() = default;

  public:
    const string& name() const { return name_; }

  private:
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
        if (version > 0) {
            ar& BOOST_SERIALIZATION_NVP(name_);
        }
    }

  protected:
    string name_;
};

BOOST_CLASS_VERSION(Animal, 1)

class Dog : public Animal {
  public:
    Dog() = default;
    Dog(const string& name, int legs) : Animal(name), legs_(legs) {}

  public:
    int legs() const { return legs_; }

  private:
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& boost::serialization::base_object<Animal>(*this);
        ar& BOOST_SERIALIZATION_NVP(legs_);
    }

  protected:
    int legs_ = 0;
};

BOOST_CLASS_VERSION(Dog, 1)
BOOST_CLASS_EXPORT_GUID(Dog, "Dog")

// save data use text archive
void saveLoadInt() {
    cout << Section("Serialization Int");
    // save
    ofstream outFile("../../data/archive.txt");
    text_oarchive oa(outFile);
    int i{1234};
    oa << i;
    outFile.close();

    // load
    ifstream inFile("../../data/archive.txt");
    text_iarchive ia(inFile);
    int x{0};
    ia >> x;
    cout << format("x = {}", x) << endl;
}

void saveLoadClass() {
    cout << Section("Serialization Class");

    cout << SubSection("Basic");
    // save
    stringstream ss1;
    text_oarchive oa1(ss1);
    Animal a1("animal");
    oa1 << a1;
    cout << format("Saved str: {}", ss1.str()) << endl;
    // load
    text_iarchive ia1(ss1);
    Animal a2;
    ia1 >> a2;
    cout << format("Load: a2.name = {}", a2.name()) << endl;

    cout << SubSection("Advance");
    // save
    stringstream ss2;
    text_oarchive oa2(ss2);
    std::shared_ptr<Animal> b1 = std::make_shared<Dog>("dog", 4);
    oa2 << b1;
    cout << format("Saved str: {}", ss2.str()) << endl;
    // load
    text_iarchive ia2(ss2);
    std::shared_ptr<Animal> b2;
    ia2 >> b2;
    cout << format("Load: b2.name = {}, b2.leg = {}", b2->name(), dynamic_pointer_cast<Dog>(b2)->legs()) << endl;
}

int main(int argc, char* argv[]) {
    saveLoadInt();
    saveLoadClass();

    return 0;
}
