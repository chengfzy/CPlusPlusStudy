#include <iostream>
#include <fstream>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

struct person{
	string name;
	string address;
	int age;
};

void to_json(json& j, const person& p){
	j = json{ {"name", p.name}, {"address", p.address}, {"age", p.age} };
}

void from_json(const json& j, person& p) {
	p.name = j.at("name").get<string>();
	p.address = j.at("address").get<string>();
	p.age = j.at("age").get<int>();
}

int main(){
	json j;
	j["pi"] = 3.141;
	j["happy"] = true;
	j["name"] = "jeffery";
	j["answer"]["everything"] = 42;
	j["list"] = { 1,0,2 };
	j["object"] = { {"currency", "USD"},{"value",42.99} };

	json j2 = {
		{"pi", 3.141},
		{"happy", true}
	};	

	cout << j << endl;
	cout << endl;
	cout << j2 << endl;
	cout << j.dump(4) << endl;

	ofstream out("pretty.json");
	out << std::setw(4) << j << endl;

	cout << endl << "\tUser Defined Struct:" << endl;




	person p{ "Jeffery", "Chengdu", 28 };
	json j3 = p;
	cout << setw(2) << j3 << endl;


	return 0;
}


