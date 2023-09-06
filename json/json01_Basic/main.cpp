#include <fmt/format.h>
#include <array>
#include <deque>
#include <forward_list>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
using json = nlohmann::json;

namespace cc {

struct Person {
    string name = "";
    string address = "";
    int age = 0;
};

void to_json(json& j, const Person& p) { j = json{{"name", p.name}, {"address", p.address}, {"age", p.age}}; }

void from_json(const json& j, Person& p) {
    j.at("name").get_to(p.name);
    j.at("address").get_to(p.address);
    j.at("age").get_to(p.age);
}

}  // namespace cc

void createJson() {
    cout << Section("Create Json", false) << endl;

    // create json step by step
    json j1;
    j1["pi"] = 3.141;
    j1["happy"] = true;
    j1["name"] = "Niels";
    j1["nothing"] = nullptr;
    j1["answer"]["everything"] = 42;
    j1["list"] = {0, 1, 2};
    j1["object"] = {{"currency", "USD"}, {"value", 42.99}};
    cout << "j1: " << j1 << endl;

    // write json string directly
    json j2 = {{"pi", 3.141},
               {"happy", true},
               {"name", "Niels"},
               {"nothing", nullptr},
               {"answer", {{"everything", 42}}},
               {"list", {1, 0, 2}},
               {"object", {{"currency", "USD"}, {"value", 42.99}}}};
    cout << "j2: " << j2 << endl;
}

void serialization() {
    cout << Section("Serialization") << endl;
    // from string
    auto j1 = R"({"happy": true, "pi":3.141})"_json;
    cout << "j1: " << j1 << endl;

    // parse explicitly
    auto j2 = json::parse(R"({"happy": true, "pi":3.141})");
    cout << "j2: " << j2 << endl;

    // to string
    string s = j2.dump();
    cout << "j2 string: " << j2.dump(2) << endl;

    // difference between serialization and assignment
    json jStr = "this is a string";
    auto cppStr1 = jStr.get<string>();
    string cppStr2;
    jStr.get_to(cppStr2);
    string serializedStr = jStr.dump();
    cout << "cppStr1: " << cppStr1 << endl;
    cout << "cppStr2: " << cppStr2 << endl;
    cout << "serializedStr: " << serializedStr << endl;
}

void stlLike() {
    cout << Section("STL-Like Access") << endl;
    // create array using push_back
    json j1;
    j1.push_back("foo");
    j1.push_back(1);
    j1.push_back(true);
    j1.emplace_back(1.78);

    // iterate the array
    cout << Paragraph("iterate the array");
    for (auto it = j1.begin(); it != j1.end(); ++it) {
        cout << *it << ", ";
    }
    cout << endl;

    // range-based for
    cout << Paragraph("range-based for") << endl;
    for (auto& v : j1) {
        cout << v << ", ";
    }
    cout << endl;

    // getter/setter
    cout << Paragraph("getter/setter") << endl;
    const auto tmp = j1[0].get<string>();
    j1[1] = 42;
    bool foo = j1.at(2);

    // comparison
    cout << Paragraph("comparison") << endl;
    cout << "comparison: " << (j1 == "[\"foo\", 42, true, 1.78]"_json) << endl;

    // other stuff
    cout << Paragraph("other stuff") << endl;
    cout << " j1.size(): " << j1.size() << endl;
    cout << " j1.empty(): " << j1.empty() << endl;
    j1.type();   // type
    j1.clear();  // clear

    // convenience type checkers
    cout << Paragraph("convenience type checkers") << endl;
    cout << "j1.is_null(): " << j1.is_null() << endl;
    cout << "j1.is_boolean(): " << j1.is_boolean() << endl;
    cout << "j1.is_array(): " << j1.is_array() << endl;
    cout << "j1.is_number(): " << j1.is_number() << endl;
    cout << "j1.is_string(): " << j1.is_string() << endl;
    cout << "j1.is_object(): " << j1.is_object() << endl;

    // create object
    cout << Paragraph("create object") << endl;
    json j2;
    j2["foo"] = 23;
    j2["bar"] = false;
    j2["baz"] = 3.141;
    j2.emplace("weather", "sunny");
    for (auto& v : j2.items()) {
        cout << v.key() << ": " << v.value() << endl;
    }
    cout << Paragraph("structured bindings range for (C++17)") << endl;
    // for (auto& [key, value] : j2.items()) {
    //     cout << key << ": " << value << endl;
    // }

    // find an entry
    cout << Paragraph("find") << endl;
    cout << "j2.find(\"foo\")" << (j2.find("foo") != j2.end()) << endl;
    cout << "foo count: " << j2.count("foo") << endl;
    cout << "fob count: " << j2.count("fob") << endl;

    // delete an entry
    cout << Paragraph("delete an entry") << endl;
    j2.erase("foo");
    for (auto& v : j2.items()) {
        cout << v.key() << ": " << v.value() << endl;
    }
}

void conversionFromStlContainers() {
    cout << Section("Conversion from STL Containers") << endl;

    // conversion from vector
    cout << SubSection("vector") << endl;
    vector<int> vectorContainers{1, 2, 3, 4};
    json jsonVec(vectorContainers);
    cout << setw(2) << jsonVec << endl;

    cout << SubSection("deque") << endl;
    deque<double> dequeContainers{1.2, 2.3, 3.4, 4.5, 5.6};
    json jsonDeq(dequeContainers);
    cout << setw(2) << jsonDeq << endl;

    cout << SubSection("list") << endl;
    list<bool> listContainers{true, true, false, true};
    json jsonList(listContainers);
    cout << setw(2) << jsonList << endl;

    cout << SubSection("forward list") << endl;
    forward_list<int64_t> forwardListContainers{12345678909876, 2345677890098776, 345678900987654, 456789009876543};
    json jsonForwardList(forwardListContainers);
    cout << setw(2) << jsonForwardList << endl;

    cout << SubSection("array") << endl;
    array<unsigned long, 5> arrayContainers{1, 2, 3, 4, 5};
    json jsonArray(arrayContainers);
    cout << setw(2) << jsonArray << endl;

    cout << SubSection("set") << endl;
    set<string> setContainers{"one", "two", "three", "four", "one"};
    json jsonSet(setContainers);
    cout << setw(2) << jsonSet << endl;

    cout << SubSection("unordered_set") << endl;
    unordered_set<string> unorderedSetContainers{"one", "two", "three", "four", "one"};
    json jsonUnorderedSet(unorderedSetContainers);
    cout << setw(2) << jsonUnorderedSet << endl;

    cout << SubSection("multiset") << endl;
    multiset<string> multisetContainers{"one", "two", "three", "four", "one"};
    json jsonMultiSet(multisetContainers);
    cout << setw(2) << jsonMultiSet << endl;

    cout << SubSection("unordered_multiset") << endl;
    unordered_multiset<string> unorderedMultiSetContainers{"one", "two", "three", "four", "one"};
    json jsonUnorderedMultiSet(unorderedMultiSetContainers);
    cout << setw(2) << jsonUnorderedMultiSet << endl;

    cout << SubSection("map") << endl;
    map<string, int> mapContainters{{"one", 1}, {"two", 2}, {"three", 3}};
    json jsonMap(mapContainters);
    cout << setw(2) << jsonMap << endl;

    cout << SubSection("unordered_map") << endl;
    unordered_map<string, double> unorderedMapContainers{{"one", 1.2}, {"two", 2.3}, {"three", 3.4}};
    json jsonUnorderedMap(unorderedMapContainers);
    cout << setw(2) << jsonUnorderedMap << endl;

    cout << SubSection("multimap") << endl;
    multimap<string, bool> multimapContainers{{"one", true}, {"two", true}, {"three", false}, {"three", true}};
    json jsonMultimap(multimapContainers);
    cout << setw(2) << jsonMultimap << endl;

    cout << SubSection("multimap") << endl;
    unordered_multimap<string, bool> unorderedMultimapContainers{
        {"one", true}, {"two", true}, {"three", false}, {"three", true}};
    json jsonUnorderedMultimap(multimapContainers);
    cout << setw(2) << jsonUnorderedMultimap << endl;
}

void arbitraryTypeConversion() {
    cout << Section("Arbitrary Types Conversions") << endl;
    cc::Person p1{"Jeffery", "Chengdu, SiChuan", 25};

    // person => json
    json j = p1;
    cout << setw(2) << j << endl;

    // json => person
    auto p2 = j.get<cc::Person>();
    // assert(p1 == p2);
}

namespace nlohmann {

template <typename T>
struct adl_serializer<std::optional<T>> {
    static void to_json(json& j, const std::optional<T>& opt) {
        if (opt == nullopt) {
            j = nullptr;
        } else {
            j = *opt;
        }
    }

    static void from_json(const json& j, std::optional<T>& opt) {
        if (j.is_null()) {
            opt = nullopt;
        } else {
            opt = j.get<T>();
        }
    }
};

}  // namespace nlohmann

void optionalType() {
    cout << Section("Types Conversions for std::optional") << endl;
    optional<double> x1;
    json j1 = x1;
    cout << setw(2) << j1 << endl;

    optional<double> x2 = 3;
    json j2 = x2;
    cout << setw(2) << j2 << endl;
    optional<double> x2b = j2;
    cout << format("x2b is valid = {}, value = {}", x2b.has_value(), *x2b) << endl;
}

int main(int argc, char* argv[]) {
    // createJson();
    // serialization();
    // stlLike();
    // conversionFromStlContainers();
    // arbitraryTypeConversion();
    optionalType();

    return 0;
}