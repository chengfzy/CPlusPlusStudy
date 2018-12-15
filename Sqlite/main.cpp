#include <iostream>
#include "SQLiteCpp/SQLiteCpp.h"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "SQLite3 version: " << SQLite::VERSION << " (" << SQLite::getLibVersion() << ")" << endl;
    cout << "SQLiteCpp version: " << SQLITECPP_VERSION << endl;

    try {
        // open a database in create/write mode
        SQLite::Database db("test.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
        cout << "Sqlite database file: " << db.getFilename() << endl;

        // create a new table
        db.exec("DROP TABLE IF EXISTS test");
        db.exec("CREATE TABLE test(ID INTEGER PRIMARY KEY, Value TEXT)");

        // insert data
        int ret = db.exec("INSERT INTO test VALUES(NULL, \"first\")");
        cout << "INSERT INTO test VALUES(NULL, \"first\"), return: " << ret << endl;
        ret = db.exec("INSERT INTO test VALUES(NULL, \"second\")");
        cout << "INSERT INTO test VALUES(NULL, \"second\"), return: " << ret << endl;

        // check the results
        SQLite::Statement query(db, "SELECT * FROM test");
        cout << "SELECT * FROM test: " << endl;
        while (query.executeStep()) {
            cout << "Row (" << query.getColumn(0) << ", " << query.getColumn(1) << ")" << endl;
        }

        // transaction
        SQLite::Transaction transaction(db);
        db.exec("CREATE TABLE IF NOT EXISTS student(ID INTEGER PRIMARY KEY, Name TEXT)");
        ret = db.exec("INSERT INTO Student VALUES (NULL, \"Jeffery\")");
        cout << "INSERT INTO Student VALUES (NULL, \"Jeffery\"), return: " << ret << endl;
        transaction.commit();

    } catch (std::exception& e) {
        cout << "SQLite exception: " << e.what() << endl;
        return -1;
    }

    return 0;
}