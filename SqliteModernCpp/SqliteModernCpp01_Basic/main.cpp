#include <fmt/format.h>
#include <fmt/ranges.h>
#include <sqlite_modern_cpp.h>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
using namespace sqlite;

void basic(const string& dbFile) {
    LOG(INFO) << Section("Basic", false);

    try {
        // create database if note exists
        database db(dbFile);

        // create table
        db << "create table if not exists User("
              "ID integer primary key autoincrement not null,"
              "Age int,"
              "Name text,"
              "Weight real"
              ");";

        // insert new record, binds the fields to '?'
        db << "insert into User (Age,Name,Weight) values (?,?,?);" << 20 << u"Blob" << 832.25;

        int age{21};
        float weight{68.5};
        string name{"Jack"};
        db << u"insert into User (Age,Name,Weight) values (?,?,?);" << age << name << weight;  // utf16 query string
        LOG(INFO) << format("the new record got assigned ID: {}", db.last_insert_rowid());

        // select from User table on a condition
        db << "select Age,Name,Weight from User where Age>?;" << 18 >>
            [&](int age, const string& name, const double& weight) {
                LOG(INFO) << format("Age: {}, Name: {}, Weight: {}", age, name, weight);
            };

        // select the count(*) from User table
        int count{0};
        db << "select count(*) from User" >> count;
        LOG(INFO) << format("count: {}", count);

        // extract multiple column rows
        db << "select Age,Name from User where ID=1;" >> tie(age, name);
        LOG(INFO) << format("ID=1, Name: {}, Age: {}", name, age);

        // could return value with string
        string countStr;
        db << "select count(*) from User" >> countStr;
        LOG(INFO) << format("count str: {}", countStr);
    } catch (const exception& e) {
        LOG(ERROR) << e.what();
    }
}

void advance(const string& dbFile) {
    LOG(INFO) << Section("Advance", false);

    try {
        database db(dbFile);

        // prepared statements
        LOG(INFO) << Paragraph("Prepared Statements");
        auto ps = db << "insert into User (Age,Name,Weight) values (?,?,?);";
        for (size_t i = 0; i < 10; ++i) {
            ps << 10 * i << format("Name{:03d}", i) << 5 * i;
            ps++;  // equal to ps.execute(); ps.reset();
        }

        // blob, using vector<T>
        LOG(INFO) << Paragraph("Blob");
        db << "create table if not exists Person(name text, data blob);";
        db << "insert into Person values (?,?);"
           << "Blob" << vector<int>{1, 2, 3, 4, 5};
        db << "insert into Person Values (?,?);"
           << "Sara" << vector<double>{1.1, 2.2, 3.3, 4.4, 5.6};

        vector<int> data;
        db << "select data from Person where name=?;"
           << "Blob" >>
            data;
        LOG(INFO) << format("Blob data: {}", data);
        db << "select data from Person where name=?;"
           << "Sara" >>
            [&](const vector<double>& raw) { LOG(INFO) << format("Sara data: {}", raw); };
    } catch (const exception& e) {
        LOG(ERROR) << e.what();
    }
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    string dbFile{"./data/SqliteModernCpp.db"};

    // basic(dbFile);
    advance(dbFile);

    closeLog();
    return 0;
}