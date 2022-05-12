#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <fstream>
#include <iostream>
#include "common/common.hpp"

using namespace std;
using namespace fmt;
using namespace common;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    fstream outFs("./data/out.gz", ios::out | ios::binary);
    boost::iostreams::filtering_ostream outStream;
    // outStream.push(boost::iostreams::zlib_compressor{});
    outStream.push(boost::iostreams::gzip_compressor{});
    outStream.push(outFs);

    outStream << "data" << endl;

    outStream.reset();
    outFs.close();

    closeLog();
    return 0;
}
