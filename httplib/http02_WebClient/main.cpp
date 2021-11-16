#include <fmt/format.h>
#include <fmt/ranges.h>
#include <glog/logging.h>
#include <httplib.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace fmt;
using namespace httplib;

// using http client
void httpClient(const string& url, bool followLocation = true) {
    Client client(url);
    client.set_follow_location(followLocation);
    auto res = client.Get("/");
    cout << format("get data from \"{}", url) << endl;
    cout << format("\tstatus: {}", res->status) << endl;
    cout << format("\tversion: {}", res->version) << endl;
    cout << format("\theaders: {}", res->headers) << endl;
    cout << format("\tlocation: {}", res->location) << endl;
    cout << format("\tbody: {}", res->body) << endl;
}

// using https client
void httpsClient() {
    SSLClient client("raw.githubusercontent.com");
    // client.set_ca_cert_path("./httplib/ca-bundle.crt");
    client.enable_server_certificate_verification(true);
    client.set_follow_location(true);
    auto res = client.Get("/chengfzy/CPlusPlusStudy/master/common/include/common/Heading.hpp");
    if (res) {
        cout << format("\tstatus: {}", res->status) << endl;
        cout << format("\tversion: {}", res->version) << endl;
        cout << format("\theaders: {}", res->headers) << endl;
        cout << format("\tlocation: {}", res->location) << endl;
        cout << format("\tbody: {}", res->body) << endl;
    } else {
        cout << format("error: {}", res.error()) << endl;
        auto ret = client.get_openssl_verify_result();
        if (ret) {
            cout << format("verify error: {}", X509_verify_cert_error_string(ret)) << endl;
        }
    }
}

int main(int argc, const char* argv[]) {
    string baidu = "www.baidu.com";  // or http://www.baidu.com
    string yahoo = "www.yahoo.com";
    string github = "raw.githubusercontent.com";

    // http Client
    httpClient(yahoo, false);  // fail
    httpClient(yahoo, true);   // OK
    httpClient(baidu, true);   // OK
    httpClient(baidu, false);  // OK

    // https client
    httpsClient();

    return 0;
}