#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace std;

class Point {
public:
    float x, y, z;

    // write to yaml
    void write(YAML::Node &node) {
        node.push_back(x);
        node.push_back(y);
        node.push_back(z);
    }

    //read from yaml
    bool read(const YAML::Node &node) {
        if (!node.IsSequence() || node.size() < 3)
            return false;

        x = node[0].as<float>();
        y = node[1].as<float>();
        z = node[2].as<float>();
        return true;
    }
};


// write yaml
void writeYaml(const string &file) {
    cout << "write to yaml..." << endl;
    YAML::Node node;

    YAML::Node node1;
    node1["name"] = "jeffery";
    node1["age"] = 20;

    YAML::Node node2;
    node2["name"] = "Jane";
    node2["age"] = 25;

    YAML::Node node3;
    node3["name"] = "John";
    node3["age"] = 5;

    YAML::Node node4;
    node4["name"] = "Jenny";
    node4["age"] = 3;

    node1["spouse"] = node2;
    node1["children"].push_back(node3);
    node1["children"].push_back(node4);

    Point pos;
    pos.x = 100.1;
    pos.y = -200.2;
    pos.z = 3.1415926;
    YAML::Node node5;
    pos.write(node5);

    node["family"] = node1;
    node["pos"] = node5;

    fstream fs(file);
    fs << node;
}

// yaml emitter
void useEmitter() {
    cout << "use emitter.." << endl;
    YAML::Emitter out;
    out << YAML::BeginSeq;
    out << "eggs";
    out << "break";
    out << "milk";
    out << YAML::EndSeq;

    out << YAML::Flow;
    out << YAML::BeginSeq << 2 << 3 << 4 << 7 << YAML::EndSeq;

    vector<int> vec{1, 4, 9, 16};
    out << YAML::BeginSeq;
    out << YAML::Flow << vec << YAML::EndSeq;

    cout << out.c_str() << endl;
}

// read yaml
void readYaml(const string &file) {
    cout << "read from yaml..." << endl;
    YAML::Node node = YAML::LoadFile(file);
    node = node["family"];

    string name;
    unsigned int age;
    name = node["name"].as<string>();
    age = node["age"].as<unsigned int>();

    cout << "name = " << name << ", age = " << age << endl;
}




int main(int argc, char *argv[]) {
    string file = "../../YamlStudy/config.yaml";
    writeYaml(file);
    useEmitter();
    readYaml(file);

    return 0;
}