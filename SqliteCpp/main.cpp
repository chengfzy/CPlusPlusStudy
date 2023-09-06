#include <SQLiteCpp/SQLiteCpp.h>
#include <fmt/format.h>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

// basic operation
void basicOperation() {
    cout << Section("Basic Operation");
    cout << format("sqlite3 version: {} ({})", SQLite::VERSION, SQLite::getLibVersion()) << endl;
    cout << format("SQLiteCPP version: {}", SQLITECPP_VERSION) << endl;

    try {
        // open a database file
        SQLite::Database db("../../data/test.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
        cout << format("SQLite database file: {}", db.getFilename()) << endl;

        // create a new table with an explicit "ID" column aliasing the underlying rowid
        db.exec("DROP TABLE IF EXISTS Student");
        db.exec("CREATE TABLE Student(ID INTEGER PRIMARY KEY, Name TEXT NOT NULL, Age INT NOT NULL)");

        // insert data
        int ret = db.exec("INSERT INTO Student VALUES(NULL, \"David\", 20)");
        cout << format("INSERT INTO Student VALUES(NULL, \"David\", 20), returned: {}", ret) << endl;
        ret = db.exec("INSERT INTO Student VALUES(NULL, \"Jeffery\", 17)");
        cout << format("INSERT INTO Student VALUES(NULL, \"Jeffery\", 17), returned: {}", ret) << endl;
        ret = db.exec("INSERT INTO Student VALUES(NULL, \"Mike\", 18)");
        cout << format("INSERT INTO Student VALUES(NULL, \"Mike\", 18), returned: {}", ret) << endl;
        ret = db.exec("INSERT INTO Student VALUES(NULL, \"Jack\", 26)");
        cout << format("INSERT INTO Student VALUES(NULL, \"Jack\", 26), returned: {}", ret) << endl;

        // update the second row
        ret = db.exec("UPDATE Student SET Age=27 WHERE Name=\"Jeffery\"");
        cout << format("UPDATE Student SET Age=27 WHERE Name=\"Jeffery\", returned: {}", ret) << endl;

        // check the results
        SQLite::Statement query(db, "SELECT * FROM Student");
        while (query.executeStep()) {
            cout << format("ID = {}, Name = {}, Age = {}", query.getColumn(0), query.getColumn(1).getString(),
                           query.getColumn(2).getInt())
                 << endl;
        }

    } catch (const std::exception& e) {
        cout << format("SQLite exception: {}", e.what()) << endl;
    }
}

// query operation
void queryOperation() {
    try {
        cout << Section("Query Operation");
        SQLite::Database db("../../data/test.db", SQLite::OPEN_READONLY);

        // compile a SQL query, containing one parameter(index 1)
        SQLite::Statement query(db, "SELECT * FROM Student WHERE Age >= ?");

        // bind the integer value 20 to the first parameter of the SQL query
        query.bind(1, 20);

        // loop to execute the query step by step, to get rows of result
        while (query.executeStep()) {
            // demonstrate how to get some typed column value
            int id = query.getColumn(0);
            string name = query.getColumn(1);
            int age = query.getColumn(2);
            cout << format("ID = {}, Name = {}, Age = {}", id, name, age) << endl;
        }

    } catch (const std::exception& e) {
        cout << format("SQLite exception: {}", e.what()) << endl;
    }
}

// transaction operation
void transactionOperation() {
    try {
        cout << Section("Transaction Operation");
        SQLite::Database db("../../data/test.db", SQLite::OPEN_READWRITE);

        // begin transaction
        SQLite::Transaction transaction(db);
        int ret = db.exec("INSERT INTO Student VALUES(NULL, \"Tom\", 30)");
        cout << format("INSERT INTO Student VALUES(NULL, \"Tom\", 30), returned: {}", ret) << endl;

        // commit transaction
        transaction.commit();

        // check the results
        SQLite::Statement query(db, "SELECT * FROM Student");
        while (query.executeStep()) {
            cout << format("ID = {}, Name = {}, Age = {}", query.getColumn(0), query.getColumn(1).getString(),
                           query.getColumn(2).getInt())
                 << endl;
        }

    } catch (const std::exception& e) {
        cout << format("SQLite exception: {}", e.what()) << endl;
    }
}

int main(int argc, char* argv[]) {
    basicOperation();
    queryOperation();
    transactionOperation();

    return 0;
}