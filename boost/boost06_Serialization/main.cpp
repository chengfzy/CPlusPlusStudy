#include <fmt/format.h>
#include <fmt/ranges.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>
#include <common/common.hpp>

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
        ar& boost::serialization::make_nvp("Name", name_);
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
        ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Animal);
        ar& BOOST_SERIALIZATION_NVP(legs_);
    }

  private:
    int legs_ = 0;
};
BOOST_CLASS_VERSION(Dog, 1)
BOOST_CLASS_EXPORT(Dog)

class Cat : public Animal {
  public:
    Cat(const string& name, int legs) : Animal(name), legs_(legs) {}

  public:
    int legs() const { return legs_; }

  private:
    friend class boost::serialization::access;

    // because load_construct_data() will call serialize() function, so split serialization to load and save, the load
    // will do nothing, because the variable already read in load_construct_data(), cannot read more than one time
    template <class Archive>
    void save(Archive& ar, const unsigned int version) const {
        ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Animal);
        ar& boost::serialization::make_nvp("Legs", legs_);
    }
    template <class Archive>
    void load(Archive& ar, const unsigned int version) {
        // ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Animal);
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

  private:
    int legs_ = 0;
};
BOOST_CLASS_VERSION(Cat, 1)
BOOST_CLASS_EXPORT(Cat)

class People {
  public:
    People(const string& name, int legs) : name_(name), legs_(legs) {}

  public:
    const string& name() const { return name_; }
    int legs() const { return legs_; }

  private:
    friend class boost::serialization::access;

    // template <typename Archive>
    // void serialize(Archive& ar, const unsigned int version) {
    //     ar& boost::serialization::make_nvp("Name", name_);
    //     ar& boost::serialization::make_nvp("Legs", legs_);
    // }

    // because load_construct_data() will call serialize() function, so split serialization to load and save, the load
    // will do nothing, because the variable already read in load_construct_data(), cannot read more than one time
    template <class Archive>
    void save(Archive& ar, const unsigned int version) const {
        ar& boost::serialization::make_nvp("Name", name_);
        ar& boost::serialization::make_nvp("Legs", legs_);
    }
    template <class Archive>
    void load(Archive& ar, const unsigned int version) {}

    BOOST_SERIALIZATION_SPLIT_MEMBER()

  private:
    string name_;
    int legs_ = 0;
};
BOOST_CLASS_VERSION(People, 1)

namespace boost {
namespace serialization {

// don't need save_construct_data() because it's the same as save()
// template <typename Archive>
// inline void save_construct_data(Archive& ar, const People* people, const unsigned int /*version*/) {
//     ar << boost::serialization::make_nvp("Name", people->name());
//     int legs = people->legs();
//     ar << boost::serialization::make_nvp("Legs", legs);
// }

template <typename Archive>
inline void load_construct_data(Archive& ar, People* people, const unsigned int /*version*/) {
    string name;
    int legs;
    ar >> boost::serialization::make_nvp("Name", name);
    ar >> boost::serialization::make_nvp("Legs", legs);
    ::new (people) People(name, legs);
}

// don't need save_construct_data() because it's the same as save()
// template <typename Archive>
// inline void save_construct_data(Archive& ar, const Cat* cat, const unsigned int /*version*/) {
// }

// to support base class, should create a base object for retrieve member
template <typename Archive>
inline void load_construct_data(Archive& ar, Cat* cat, const unsigned int /*version*/) {
    int legs;
    Animal animal;
    ar >> boost::serialization::make_nvp("Animal", animal);
    ar >> boost::serialization::make_nvp("Legs", legs);
    ::new (cat) Cat(animal.name(), legs);
}

}  // namespace serialization
}  // namespace boost

// serialize basic type to text
void serializeBasicText() {
    cout << Section("Serialization Basic with Text");

    // save
    stringstream strStream;
    text_oarchive oa(strStream);
    int a1{1234};
    double a2{1.23456789};
    string a3{"Hello World!"};
    vector<string> a4{"Jeffery", "boost", "C++"};
    oa << a1;
    oa << a2;
    oa << a3;
    oa << a4;
    cout << format("Serialized string: \"{}\"", strStream.str()) << endl;

    // load
    text_iarchive ia(strStream);
    int b1{0};
    double b2{0};
    string b3;
    vector<string> b4;
    ia >> b1;
    ia >> b2;
    ia >> b3;
    ia >> b4;
    cout << format("b1 = {}, b2 = {}, b3 = {}, b4 = {}", b1, b2, b3, b4) << endl;
}

// serialize basic type to xml
void serializeBasicXml() {
    cout << Section("Serialization Basic with XML");

    // save
    stringstream strStream;
    int a1{1234};
    double a2{1.23456789};
    string a3{"Hello World!"};
    vector<string> a4{"Jeffery", "boost", "C++"};
    // add brace to ensure xml end with "boost_serialization" tag
    {
        xml_oarchive oa(strStream);
        oa << BOOST_SERIALIZATION_NVP(a1);
        oa << BOOST_SERIALIZATION_NVP(a2);
        oa << BOOST_SERIALIZATION_NVP(a3);
        oa << BOOST_SERIALIZATION_NVP(a4);
    }
    cout << format("Serialized string:\n{}", strStream.str()) << endl;

    // load
    int b1{0};
    double b2{0};
    string b3;
    vector<string> b4;
    xml_iarchive ia(strStream);
    ia >> BOOST_SERIALIZATION_NVP(b1);
    ia >> BOOST_SERIALIZATION_NVP(b2);
    ia >> BOOST_SERIALIZATION_NVP(b3);
    ia >> BOOST_SERIALIZATION_NVP(b4);
    cout << format("b1 = {}, b2 = {}, b3 = {}, b4 = {}", b1, b2, b3, b4) << endl;
}

// serialize basic type to binary
void serializeBasicBin() {
    cout << Section("Serialization Basic with Binary");

    // save
    stringstream strStream;
    int a1{1234};
    double a2{1.23456789};
    string a3{"Hello World!"};
    vector<string> a4{"Jeffery", "boost", "C++"};
    binary_oarchive oa(strStream);
    oa << BOOST_SERIALIZATION_NVP(a1);
    oa << BOOST_SERIALIZATION_NVP(a2);
    oa << BOOST_SERIALIZATION_NVP(a3);
    oa << BOOST_SERIALIZATION_NVP(a4);
    cout << format("Serialized string: \"{}\"", strStream.str()) << endl;

    // load
    int b1{0};
    double b2{0};
    string b3;
    vector<string> b4;
    binary_iarchive ia(strStream);
    ia >> BOOST_SERIALIZATION_NVP(b1);
    ia >> BOOST_SERIALIZATION_NVP(b2);
    ia >> BOOST_SERIALIZATION_NVP(b3);
    ia >> BOOST_SERIALIZATION_NVP(b4);
    cout << format("b1 = {}, b2 = {}, b3 = {}, b4 = {}", b1, b2, b3, b4) << endl;
}

// serialize class basic usage
void serializeClassBasic() {
    cout << Section("Serialization Class Basic");

    // save
    stringstream strStream;
    Dog a1("DaHuang", 4);
    // add brace to ensure xml end with "boost_serialization" tag
    {
        xml_oarchive oa(strStream);
        oa << BOOST_SERIALIZATION_NVP(a1);
    }
    cout << format("Serialized string:\n{}", strStream.str()) << endl;

    // load
    Dog b1;
    xml_iarchive ia(strStream);
    ia >> BOOST_SERIALIZATION_NVP(b1);
    cout << format("b1.name = {}, b1.leg = {}", b1.name(), b1.legs()) << endl;

    // cout << SubSection("Advance");
    // // save
    // stringstream ss2;
    // text_oarchive oa2(ss2);
    // std::shared_ptr<Animal> b1 = std::make_shared<Dog>("dog", 4);
    // oa2 << b1;
    // cout << format("Saved str: {}", ss2.str()) << endl;
    // // load
    // text_iarchive ia2(ss2);
    // std::shared_ptr<Animal> b2;
    // ia2 >> b2;
    // cout << format("Load: b2.name = {}, b2.leg = {}", b2->name(), dynamic_pointer_cast<Dog>(b2)->legs()) << endl;
}

// serialize class with pointer
void serializeClassPointer() {
    cout << Section("Serialization Class Pointer");

    // save
    stringstream strStream;
    shared_ptr<Animal> a1 = std::make_shared<Dog>("DaHuang", 4);
    // add brace to ensure xml end with "boost_serialization" tag
    {
        xml_oarchive oa(strStream);
        oa << BOOST_SERIALIZATION_NVP(a1);
    }
    cout << format("Serialized string:\n{}", strStream.str()) << endl;

    // load
    std::shared_ptr<Animal> b1;
    xml_iarchive ia(strStream);
    ia >> BOOST_SERIALIZATION_NVP(b1);

    cout << format("b1.name = {}, b1.leg = {}", b1->name(), dynamic_pointer_cast<Dog>(b1)->legs()) << endl;
}

// serialize class without default-constructor
void serializeClassWithoutDefault() {
    cout << Section("Serialization Class without Default Constructor");

    // save
    stringstream strStream;
    People a1("XiaoMing", 2);
    // add brace to ensure xml end with "boost_serialization" tag
    {
        xml_oarchive oa(strStream);
        oa << boost::serialization::make_nvp("People", a1);
    }
    cout << format("Serialized string:\n{}", strStream.str()) << endl;

    // load for shared pointer, cannot be null pointer and should new a class to store the raw pointer, seems so weired.
    std::shared_ptr<People> b1 = make_shared<People>("Hi", 10);
    xml_iarchive ia1(strStream);
    People* tempB = b1.get();
    ia1 >> boost::serialization::make_nvp("People", tempB);
    cout << format("b1.name = {}, b1.leg = {}", b1->name(), b1->legs()) << endl;

    // load for raw pointer
    stringstream strStream2;  // save again
    {
        xml_oarchive oa(strStream2);
        oa << boost::serialization::make_nvp("People", a1);
    }
    People* b2;
    xml_iarchive ia(strStream2);
    ia >> boost::serialization::make_nvp("People", b2);
    cout << format("b2.name = {}, b2.leg = {}", b2->name(), b2->legs()) << endl;
}

// serialize derived class without default-constructor
void serializeDeriveClassWithoutDefault() {
    cout << Section("Serialization Derived Class without Default Constructor");

    // save, cannot save with shared pointer, its result isn't the same
    stringstream strStream;
    // shared_ptr<Animal> a1 = make_shared<Cat>("MiaoMiao", 4);
    Cat a1("MiaoMiao", 4);
    // add brace to ensure xml end with "boost_serialization" tag
    {
        xml_oarchive oa(strStream);
        oa << boost::serialization::make_nvp("Cat", a1);
    }
    cout << format("Serialized string:\n{}", strStream.str()) << endl;

    // load for shared pointer, cannot be null pointer and should new a class to store the raw pointer, seems so
    // weired.
    std::shared_ptr<Cat> b1 = make_shared<Cat>("Hi", 10);
    xml_iarchive ia1(strStream);
    Cat* tempB = b1.get();
    ia1 >> boost::serialization::make_nvp("Cat", tempB);
    cout << format("b1.name = {}, b1.leg = {}", b1->name(), b1->legs()) << endl;

    // load for raw pointer
    stringstream strStream2;  // save again
    {
        xml_oarchive oa(strStream2);
        oa << boost::serialization::make_nvp("Cat", a1);
    }
    Cat* b2;
    xml_iarchive ia(strStream2);
    ia >> boost::serialization::make_nvp("Cat", b2);
    cout << format("b2.name = {}, b2.leg = {}", b2->name(), b2->legs()) << endl;
}

int main(int argc, char* argv[]) {
    serializeBasicText();
    serializeBasicXml();
    serializeBasicBin();
    serializeClassBasic();
    serializeClassPointer();
    serializeClassWithoutDefault();
    serializeDeriveClassWithoutDefault();

    return 0;
}
