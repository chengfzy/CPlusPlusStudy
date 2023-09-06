#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <istream>
#include <string>
#include <thread>
#include <common/common.hpp>

using namespace std;
using namespace common;
using namespace boost::asio;

class Client {
  public:
    Client(io_service& io_service, ssl::context& context, const string& server, const string& path)
        : resolver_(io_service), socket_(io_service, context) {
        // Form the request. We specify the "Connection: close" header so that the server will close the socket after
        // transmitting the response. This will allow us to treat all data up until the EOF as the content.
        ostream request_stream(&request_);
        request_stream << "GET " << path << " HTTP/1.0\r\n";
        request_stream << "Host: " << server << "\r\n";
        request_stream << "Accept: */*\r\n";
        request_stream << "Connection: close\r\n\r\n";

        // Start an asynchronous resolve to translate the server and service names into a list of endpoints.
        ip::tcp::resolver::query query(server, "https");
        resolver_.async_resolve(query, boost::bind(&Client::handle_resolve, this, boost::asio::placeholders::error,
                                                   boost::asio::placeholders::iterator));
    }

  private:
    void handle_resolve(const boost::system::error_code& err, ip::tcp::resolver::iterator endpoint_iterator) {
        if (!err) {
            cout << "Resolve OK" << endl;
            socket_.set_verify_mode(ssl::verify_peer);
            socket_.set_verify_callback(boost::bind(&Client::verify_certificate, this, _1, _2));

            async_connect(socket_.lowest_layer(), endpoint_iterator,
                          boost::bind(&Client::handle_connect, this, boost::asio::placeholders::error));
        } else {
            cout << "Error resolve: " << err.message() << "\n";
        }
    }

    bool verify_certificate(bool preverified, ssl::verify_context& ctx) {
        // The verify callback can be used to check whether the certificate that is
        // being presented is valid for the peer. For example, RFC 2818 describes
        // the steps involved in doing this for HTTPS. Consult the OpenSSL
        // documentation for more details. Note that the callback is called once
        // for each certificate in the certificate chain, starting from the root
        // certificate authority.

        // In this example we will simply print the certificate's subject name.
        char subject_name[256];
        X509* cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());
        X509_NAME_oneline(X509_get_subject_name(cert), subject_name, 256);
        cout << "Verifying " << subject_name << "\n";

        return preverified;
    }

    void handle_connect(const boost::system::error_code& err) {
        if (!err) {
            cout << "Connect OK " << endl;
            socket_.async_handshake(ssl::stream_base::client,
                                    boost::bind(&Client::handle_handshake, this, boost::asio::placeholders::error));
        } else {
            cout << "Connect failed: " << err.message() << "\n";
        }
    }

    void handle_handshake(const boost::system::error_code& error) {
        if (!error) {
            cout << "Handshake OK " << endl;
            cout << "Request: " << endl;
            const char* header = buffer_cast<const char*>(request_.data());
            cout << header << endl;

            // The handshake was successful. Send the request.
            async_write(socket_, request_,
                        boost::bind(&Client::handle_write_request, this, boost::asio::placeholders::error));
        } else {
            cout << "Handshake failed: " << error.message() << "\n";
        }
    }

    void handle_write_request(const boost::system::error_code& err) {
        if (!err) {
            // Read the response status line. The response_ streambuf will
            // automatically grow to accommodate the entire line. The growth may be
            // limited by passing a maximum size to the streambuf constructor.
            async_read_until(socket_, response_, "\r\n",
                             boost::bind(&Client::handle_read_status_line, this, boost::asio::placeholders::error));
        } else {
            cout << "Error write req: " << err.message() << "\n";
        }
    }

    void handle_read_status_line(const boost::system::error_code& err) {
        if (!err) {
            // Check that response is OK.
            std::istream response_stream(&response_);
            string http_version;
            response_stream >> http_version;
            unsigned int status_code;
            response_stream >> status_code;
            string status_message;
            getline(response_stream, status_message);
            if (!response_stream || http_version.substr(0, 5) != "HTTP/") {
                cout << "Invalid response\n";
                return;
            }
            if (status_code != 200) {
                cout << "Response returned with status code ";
                cout << status_code << "\n";
                return;
            }
            cout << "Status code: " << status_code << "\n";

            // Read the response headers, which are terminated by a blank line.
            async_read_until(socket_, response_, "\r\n\r\n",
                             boost::bind(&Client::handle_read_headers, this, boost::asio::placeholders::error));
        } else {
            cout << "Error: " << err.message() << "\n";
        }
    }

    void handle_read_headers(const boost::system::error_code& err) {
        if (!err) {
            // Process the response headers.
            istream response_stream(&response_);
            string header;
            while (getline(response_stream, header) && header != "\r") cout << header << "\n";
            cout << "\n";

            // Write whatever content we already have to output.
            if (response_.size() > 0) cout << &response_;

            // Start reading remaining data until EOF.
            async_read(socket_, response_, transfer_at_least(1),
                       boost::bind(&Client::handle_read_content, this, boost::asio::placeholders::error));
        } else {
            cout << "Error: " << err << "\n";
        }
    }

    void handle_read_content(const boost::system::error_code& err) {
        if (!err) {
            // Write all of the data that has been read so far.
            cout << &response_;

            // Continue reading remaining data until EOF.
            async_read(socket_, response_, transfer_at_least(1),
                       boost::bind(&Client::handle_read_content, this, boost::asio::placeholders::error));
        } else if (err != error::eof) {
            cout << "Error: " << err << "\n";
        }
    }

    ip::tcp::resolver resolver_;
    ssl::stream<ip::tcp::socket> socket_;
    boost::asio::streambuf request_;
    boost::asio::streambuf response_;
};

/**
 * @brief Get content using http
 *
 */
void getHttp(const string& server, const string& path) {
    cout << Section("Get Content using HTTP");
    ip::tcp::iostream stream;
    stream.connect(server, "http");
    stream << "GET " << path << " HTTP/1.0\r\n";
    stream << "Host: " << server << "\r\n";
    stream << "User-Agent:Mozilla/5.0 (Windows; U; Windows NT 6.1; en-US; rv:1.9.1.6) Gecko/20091201 Firefox/3.5.6";
    stream << "Accept: */*\r\n";
    stream << "\r\n";
    stream.flush();
    cout << stream.rdbuf() << endl;
}

/**
 * @brief Get content using https
 *
 */
void getHttps(const string& server, const string& path) {
    cout << Section("Get Content using HTTPS");

    // create a context that uses the default paths for finding CA certificates
    ssl::context ctx(ssl::context::sslv23);
    ctx.set_default_verify_paths();

    // create io service
    io_service service;
    ip::tcp::resolver resolver(service);
    ip::tcp::resolver::query query(server, "https");
    auto itEndpoint = resolver.resolve(query);

    // try each endpoint until we successfully establish a connection
    ssl::stream<ip::tcp::socket> socket(service, ctx);
    connect(socket.lowest_layer(), itEndpoint);
    socket.lowest_layer().set_option(ip::tcp::no_delay(true));

    // perform SSL handshake and verify the remote host's certificate
    socket.set_verify_mode(ssl::verify_peer);
    // TODO
    // socket.set_verify_callback(ssl::rfc2818_verification("host.name"));
    // char subjectName[256];
    // X509* cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());
    // X509_NAME_oneline(X509_get_subject_name(cert), subjectName, 256);
    // cout << "Verifying " << subjectName << "\n";
    socket.handshake(ssl::stream<ip::tcp::socket>::client);

    // form the request. we specify the "Connection:close" header vso that the server will close the socket after
    // transmitting the response, this will allow us to treat all data until the EOF as the contil
    boost::asio::streambuf request;
    std::ostream stream(&request);
    stream << "GET " << path << " HTTP/1.0\r\n";
    stream << "Host: " << server << "\r\n";
    // stream << "User-Agent:Mozilla/5.0 (Windows; U; Windows NT 6.1; en-US; rv:1.9.1.6) Gecko/20091201
    // Firefox/3.5.6";
    stream << "Accept: */*\r\n\r\n";
    stream << "Connection: close\r\n\r\n";
    // stream.flush();
    // cout << stream.rdbuf() << endl;

    // send the request
    write(socket, request);

    // read the response status line, the response streambuf will automatically grow to accommodate the entire line,
    // the growth may be limited by passing a maximum size to the streambuf constructor
    boost::asio::streambuf response;
    read_until(socket, response, "\r\n");

    // check that the response is OK
    istream responseStream(&response);
    string httpsVersion;
    responseStream >> httpsVersion;
    unsigned int statusCode;
    responseStream >> statusCode;
    string statusMessage;
    getline(responseStream, statusMessage);
    if (!responseStream || httpsVersion.substr(0, 5) != "HTTP/") {
        cout << "Invalid Response" << endl;
        return;
    }
    cout << "Status code " << statusCode << endl;
    if (statusCode != 200) {
        cout << "Response returned with the status code " << statusCode << endl;
        cout << statusMessage << endl;
    }
    // read the response headers , which are terminated by a blank line
    read_until(socket, response, "\r\n\r\n");

    // process the response headers
    string header;
    while (getline(responseStream, header) && header != "\r") {
        cout << header << endl;
    }
    if (response.size() > 0) {
        cout << &response << endl;
    }

    boost::system::error_code error;
    while (read(socket, response, transfer_at_least(1), error)) {
        std::cout << &response;
    }
    if (error != error::eof) {
        throw boost::system::system_error(error);
    }

    return;
}

/**
 * @brief Get content using https with async mode, official demo
 *
 */
void getHttpsAsync(const string& server, const string& path) {
    cout << Section("Get Content using HTTPS with Async Mode");
    try {
        ssl::context ctx(ssl::context::sslv23);
        ctx.set_default_verify_paths();

        io_service io_service;
        Client c(io_service, ctx, server, path);
        io_service.run();
    } catch (exception& e) {
        cout << "Exception: " << e.what() << "\n";
    }
}

int main(int argc, char* argv[]) {
    getHttp("www.boost.org", "/LICENSE_1.0.txt");
    this_thread::sleep_for(1s);
    getHttps("raw.githubusercontent.com", "/chengfzy/CPlusPlusStudy/master/common/include/common/Heading.hpp");
    this_thread::sleep_for(1s);
    getHttpsAsync("raw.githubusercontent.com", "/chengfzy/CPlusPlusStudy/master/common/include/common/Heading.hpp");

    return 0;
}