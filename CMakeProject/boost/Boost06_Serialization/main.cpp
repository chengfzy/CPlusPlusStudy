#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include "boost/archive/text_iarchive.hpp"
#include "boost/archive/text_oarchive.hpp"

using namespace std;
using namespace boost::archive;

// Animal class
class Animal {
   public:
    Animal() = default;
    Animal(int legs, const string& name) : legs_(legs), name_(name) {}

   public:
    int legs() const { return legs_; }
    const string& name() const { return name_; }

   private:
    friend class boost::serialization::access;

    template <typename Archive>
    friend void serialize(Archive& ar, Animal& a, const unsigned int version) {
        ar& a.legs_;
        if (version > 0) ar& a.name_;
    }

   private:
    int legs_;
    string name_;
};

BOOST_CLASS_VERSION(Animal, 1)

// save data use text archive
void saveLoadInt() {
    // save
    ofstream outFile("./archive.txt");
    text_oarchive oa(outFile);
    int i{1234};
    oa << i;
    outFile.close();

    // load
    ifstream inFile("./archive.txt");
    text_iarchive ia(inFile);
    int x{0};
    ia >> x;
    cout << x << endl;
}

void saveLoadClass() {
    // save
    stringstream ss;
    text_oarchive oa(ss);
    Animal a(4, "cat");
    oa << a;

    // output
    cout << "stringstream: " << ss.str() << endl;

    // load
    text_iarchive ia(ss);
    Animal b;
    ia >> b;
    cout << b.legs() << ", " << b.name() << endl;
}

int main(int argc, char* argv[]) {
    saveLoadInt();
    saveLoadClass();

    return 0;
}
