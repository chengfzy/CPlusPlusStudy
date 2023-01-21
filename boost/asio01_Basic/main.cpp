#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

// using a time synchronously, ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tuttimer1.html
void syncTimer() {
    LOG(INFO) << Section("Using a Timer Synchronously", false);
    io_context io;
    steady_timer t(io, std::chrono::seconds(5));
    t.wait();
    LOG(INFO) << "called wait()";
    LOG(INFO) << "hello, world!";
}

// using a time asynchronously, ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tuttimer2.html
void asyncTimer() {
    LOG(INFO) << Section("Using a Timer Asynchronously", false);
    io_context io;
    steady_timer t(io, std::chrono::seconds(5));
    t.async_wait([](const boost::system::error_code& e) { LOG(INFO) << "hello, world!"; });
    LOG(INFO) << "called async_wait()";
    io.run();
}

void printFunc(const boost::system::error_code& e, steady_timer& t, int& count) {
    if (count < 5) {
        LOG(INFO) << format("count = {}", count);
        ++count;

        t.expires_from_now(std::chrono::seconds(1));
        t.async_wait(std::bind(printFunc, std::placeholders::_1, std::ref(t), std::ref(count)));
    }
}

// binding arguments to a completion handler, ref:
// https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tuttimer3.html
void bindArgumentToHandler() {
    LOG(INFO) << Section("Binding Arguments to a Completion Handler", false);

    io_context io;
    int count{0};
    steady_timer t(io, std::chrono::seconds(1));
    // t.async_wait(boost::bind(print, placeholders::error, t, count));
    t.async_wait(std::bind(printFunc, std::placeholders::_1, std::ref(t), std::ref(count)));
    LOG(INFO) << "called async_wait()";
    io.run();
    LOG(INFO) << format("final count = {}", count);
}

class Printer {
  public:
    Printer(io_context& io) : timer_(io, std::chrono::seconds(1)), count_(0) {
        timer_.async_wait(std::bind(&Printer::print, this));
    }

    ~Printer() { LOG(INFO) << format("final count = {}", count_); }

  public:
    void print() {
        if (count_ < 5) {
            LOG(INFO) << format("count = {}", count_);
            ++count_;

            timer_.expires_from_now(std::chrono::seconds(1));
            timer_.async_wait(std::bind(&Printer::print, this));
        }
    }

  private:
    steady_timer timer_;
    int count_;
};

// using a member function as a completion handler, ref:
// https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tuttimer4.html
void memberFunctionHandler() {
    LOG(INFO) << Section("Using a Member Function as A Completion Handler", false);
    io_context io;
    Printer p(io);
    io.run();
}

class MultiThreadPrinter {
  public:
    MultiThreadPrinter(io_context& io)
        : strand_(make_strand(io)),
          timer1_(io, std::chrono::seconds(1)),
          timer2_(io, std::chrono::seconds(1)),
          count_(0) {
        timer1_.async_wait(bind_executor(strand_, std::bind(&MultiThreadPrinter::print1, this)));
        timer2_.async_wait(bind_executor(strand_, std::bind(&MultiThreadPrinter::print2, this)));
    }

    ~MultiThreadPrinter() { LOG(INFO) << format("final count = {}", count_); }

  public:
    void print1() {
        if (count_ < 10) {
            LOG(INFO) << format("timer1, count = {}", count_);
            ++count_;

            timer1_.expires_from_now(std::chrono::seconds(1));
            timer1_.async_wait(bind_executor(strand_, std::bind(&MultiThreadPrinter::print1, this)));
        }
    }

    void print2() {
        if (count_ < 10) {
            LOG(INFO) << format("timer2, count = {}", count_);
            ++count_;

            timer2_.expires_from_now(std::chrono::seconds(1));
            timer2_.async_wait(bind_executor(strand_, std::bind(&MultiThreadPrinter::print2, this)));
        }
    }

  private:
    strand<io_context::executor_type> strand_;
    steady_timer timer1_;
    steady_timer timer2_;
    int count_;
};

// synchronizing completion handlers in multithreaded programs
void multiThreadHandler() {
    LOG(INFO) << Section("Synchronizing Completion Handlers in Multithreaded Programs", false);
    io_context io;
    MultiThreadPrinter p(io);
    std::thread t([&]() { io.run(); });
    t.join();
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    syncTimer();
    asyncTimer();
    bindArgumentToHandler();
    memberFunctionHandler();
    multiThreadHandler();

    closeLog();
    return 0;
}