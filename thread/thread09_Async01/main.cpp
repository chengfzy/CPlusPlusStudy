#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <common/common.hpp>

using namespace std;
using namespace common;

// fetch data from database
string fetchDataFromDb(const string& data) {
    // should taken 5 seconds to fetch data
    this_thread::sleep_for(chrono::seconds(5));

    return "DB_" + data;
}

// fetch data from file
string fetchDataFromFile(const string& data) {
    // should taken 5 seconds to fetch data
    this_thread::sleep_for(chrono::seconds(5));

    return "File_" + data;
}

// struct with operator()
struct DdbFetcher {
    string operator()(const string& data) {
        // should taken 5 seconds to fetch data
        cout << "thread id = " << this_thread::get_id() << endl;
        this_thread::sleep_for(chrono::seconds(5));

        return "DB_" + data;
    }
};

// sequence execute task
void sequence() {
    cout << Section("Sequence Execution") << endl;
    // start time
    auto t0 = chrono::system_clock::now();

    // fetch data from database and file
    string dbData = fetchDataFromDb("Data");
    string fileData = fetchDataFromFile("Data");

    // end time
    auto t1 = chrono::system_clock::now();
    cout << "Total time take = " << chrono::duration_cast<chrono::milliseconds>(t1 - t0).count() << " ms" << endl;

    // package data
    string data = dbData + "::" + fileData;
    cout << "Data = " << data << endl;
}

// async parallel task
void asyncTask() {
    cout << Section("Async Task") << endl;
    // start time
    auto t0 = chrono::system_clock::now();

    // fetch data from database and file
    future<string> resultFromDb = async(launch::async, fetchDataFromDb, "Data");
    string fileData = fetchDataFromFile("Data");
    string dbData = resultFromDb.get();  // wait until get data

    // end time
    auto t1 = chrono::system_clock::now();
    cout << "Total time take = " << chrono::duration_cast<chrono::milliseconds>(t1 - t0).count() << " ms" << endl;

    // package data
    string data = dbData + "::" + fileData;
    cout << "Data = " << data << endl;
}

// async parallel task, using operator() and lambda
void asyncTaskUsingLambda() {
    cout << Section("Async Task with lambda Function") << endl;
    cout << "thread id = " << this_thread::get_id() << endl;

    // start time
    auto t0 = chrono::system_clock::now();

    // fetch data from database and file
    future<string> resultFromDb = async(launch::async, DdbFetcher(), "Data");
    future<string> resultFromFile = async(
        launch::async,
        [](const string& data) {
            // should taken 5 seconds to fetch data
            cout << "thread id = " << this_thread::get_id() << endl;
            this_thread::sleep_for(chrono::seconds(5));
            return "File_" + data;
        },
        "Data");
    string dbData = resultFromDb.get();
    string fileData = resultFromFile.get();

    // end time
    auto t1 = chrono::system_clock::now();
    cout << "Total time take = " << chrono::duration_cast<chrono::milliseconds>(t1 - t0).count() << " ms" << endl;

    // package data
    string data = dbData + "::" + fileData;
    cout << "Data = " << data << endl;
}

int main(int argc, char* argv[]) {
    sequence();  // sequence execute, normal one

    asyncTask();  // parallel task, async

    asyncTaskUsingLambda();  // parallel task, another async constructor

    return 0;
}
