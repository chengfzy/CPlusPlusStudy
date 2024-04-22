#include <fmt/format.h>
#include <common/common.hpp>
#include <fstream>
#include <iostream>
#include "cc/AddressBook.pb.h"

using namespace std;
using namespace fmt;
using namespace common;

void printAddressBook(const cc::AddressBook& addressBook) {
    for (size_t i = 0; i < addressBook.people_size(); ++i) {
        auto& person = addressBook.people(i);
        cout << format("ID = {}, name = {}, email = {}", person.id(), person.name(), person.email()) << endl;
        for (size_t j = 0; j < person.phones_size(); ++j) {
            cout << format("\t[{}] type = {}, number = {}", j, cc::Person_PhoneType_Name(person.phones(j).type()),
                           person.phones(j).number())
                 << endl;
        }
    }
}

void write() {
    LOG(INFO) << Section("Write");
    cc::AddressBook addressBook;
    auto people = addressBook.add_people();
    people->set_id(123);
    people->set_name("John");
    people->set_email("test@google.com");
    auto phone1 = people->add_phones();
    phone1->set_number("123456789");
    phone1->set_type(cc::Person::PhoneType::Person_PhoneType_HOME);
    auto phone2 = people->add_phones();
    phone2->set_number("23456789");
    phone2->set_type(cc::Person::PhoneType::Person_PhoneType_WORK);

    LOG(INFO) << format("serialize to string: {}", addressBook.SerializeAsString());
    LOG(INFO) << format("debug string: {}", addressBook.DebugString());
    LOG(INFO) << format("short debug string: {}", addressBook.ShortDebugString());
    LOG(INFO) << format("utf-8 debug string: {}", addressBook.Utf8DebugString());
    LOG(INFO) << "print debug string:";
    addressBook.PrintDebugString();
    LOG(INFO) << "print";
    printAddressBook(addressBook);

    // save to file
    LOG(INFO) << format("save to file");
    fstream fs("./data/address.bin", ios::out | ios::app | ios::binary);
    CHECK(fs.is_open()) << format("cannot open file to write pb");
    CHECK(addressBook.SerializeToOstream(&fs)) << "failed to save to file";
    fs.close();
}

void read() {
    LOG(INFO) << Section("Read");

    cc::AddressBook addressBook;

    // load from file
    LOG(INFO) << format("load from file");
    fstream fs("./data/address.bin", ios::in | ios::binary);
    CHECK(fs.is_open()) << format("cannot open file to load pb");
    CHECK(addressBook.ParseFromIstream(&fs)) << "failed to load to file";
    fs.close();

    printAddressBook(addressBook);
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    write();
    write();
    read();

    google::protobuf::ShutdownProtobufLibrary();
    closeLog();
    return 0;
};