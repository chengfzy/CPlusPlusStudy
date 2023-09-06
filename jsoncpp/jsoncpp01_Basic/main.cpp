#include <fmt/format.h>
#include <fmt/ranges.h>
#include <json/json.h>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

void createJson() {
    cout << Section("Create Json", false) << endl;

    Json::Value root;
    Json::Value data;
    root["action"] = "run";
    data["number"] = 1;
    root["data"] = data;
    // add array
    Json::Value array;
    for (int i = 0; i < 4; ++i) {
        array[i] = 2 * i;
    }
    root["array"] = array;

    cout << format("root: {}", root.toStyledString()) << endl;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    string jsonFile = Json::writeString(builder, root);
    cout << "json file:" << endl << jsonFile << endl;
}

void readJson() {
    cout << Section("Read Json", false) << endl;
    string rawJson = R"({"City": "Chengdu", "data": [{"Name": "colin", "Age": 20}, {"Name": "Tom", "Age": 23}]})";
    cout << format("raw string: {}", rawJson) << endl;
    Json::CharReaderBuilder builder;
    unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    JSONCPP_STRING err;
    if (!reader->parse(rawJson.c_str(), rawJson.c_str() + rawJson.length(), &root, &err)) {
        cout << "error: " << endl;
        return;
    }

    // define struct
    struct Person {
        string name;
        int age;
    };

    // parse
    string city = root["City"].asString();
    auto dataObj = root["data"];
    vector<Person> data;
    for (size_t i = 0; i < dataObj.size(); ++i) {
        string name = dataObj[static_cast<int>(i)]["Name"].asString();
        int age = dataObj[static_cast<int>(i)]["Age"].asInt();
        Person p{name, age};
        data.emplace_back(p);
    }
    cout << format("city = {}", city) << endl;
    for (size_t i = 0; i < data.size(); ++i) {
        cout << format("[{}] name = {}, age = {}", i, data[i].name, data[i].age) << endl;
    }
}

int main(int argc, char* argv[]) {
    createJson();
    readJson();

    return 0;
}