#include <fstream>
#include <iostream>
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

    // read from yaml
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

    // user-defined class
    Point pos;
    pos.x = 100.1f;
    pos.y = -200.2f;
    pos.z = 3.1415926f;
    YAML::Node node5;
    pos.write(node5);

    // write sequences
    vector<int> vec{1, 2, 3, 4, 5};
    YAML::Node node6;
    // node6["vec"] = vec;

    // write all to file
    node["family"] = node1;
    node["pos"] = node5;
    node["vec"] = vec;

    fstream fs(file, ios::out);
    fs << node;
    fs.close();
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
    YAML::Node node1 = node["family"];

    string name;
    unsigned int age;
    name = node1["name"].as<string>();
    age = node1["age"].as<unsigned int>();
    cout << "name = " << name << ", age = " << age << endl;

    // read pos
    YAML::Node posNode = node["pos"];
    Point pos;
    pos.read(posNode);
    cout << "Pos = [" << pos.x << ", " << pos.y << ", " << pos.z << "]" << endl;

    // read vector
    vector<int> vec = node["vec"].as<vector<int>>();
    cout << "vec = [";
    for (auto it = vec.begin(); it != vec.end(); ++it) {
        if (it != vec.begin()) {
            cout << ", ";
        }
        cout << *it;
    }
    cout << "]" << endl;
}

int main(int argc, char *argv[]) {
    string file = "../../YamlStudy/config.yaml";
    writeYaml(file);
    useEmitter();
    readYaml(file);

    return 0;
}
