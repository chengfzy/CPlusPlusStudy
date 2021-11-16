#include <fmt/format.h>
#include <fmt/ranges.h>
#include <httplib.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace fmt;
using namespace httplib;

int main(int argc, const char* argv[]) {
    // create server thread
    thread serverThread([]() {
        Server server;
        server.Get("/hi",
                   [](const Request&, Response& response) { response.set_content("Hello World!", "text/plain"); });
        server.listen("0.0.0.0", 8080);
    });

    // create client
    thread clientThread([]() {
        Client client("0.0.0.0", 8080);
        while (true) {
            this_thread::sleep_for(1s);
            auto res = client.Get("/hi");
            cout << "receive data:" << endl;
            cout << format("version: {}", res->version) << endl;
            cout << format("status: {}", res->status) << endl;
            cout << format("headers: {}", res->headers) << endl;
            cout << format("location: {}", res->location) << endl;
            cout << format("body: {}", res->body) << endl << endl;
        }
    });

    // main thread
    while (true) {
        this_thread::sleep_for(1s);
    }

    return 0;
}