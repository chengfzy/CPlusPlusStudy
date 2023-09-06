#include <fmt/ranges.h>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    {
        // compress
        fstream fs("./data/out.gz", ios::out | ios::binary);
        boost::iostreams::filtering_ostream stream;
        // stream.push(boost::iostreams::zlib_compressor{});
        stream.push(boost::iostreams::gzip_compressor{});
        stream.push(fs);

        // stream << "data" << endl;
        vector<char> data = {1, 2, 3, 4, 5, 6, 78, 89};
        stream.write(data.data(), data.size());

        stream.reset();
        fs.close();
    }

    {
        // decompress
        fstream fs("./data/out.gz", ios::in | ios::binary);
        boost::iostreams::filtering_istream stream;
        stream.push(boost::iostreams::gzip_decompressor{});
        stream.push(fs);

        // stringstream ss;
        // boost::iostreams::copy(stream, ss);
        // LOG(INFO) << ss.str();
        vector<unsigned char> raw;
        raw.assign(istreambuf_iterator<char>{stream}, {});
        LOG(INFO) << format("raw: {}", raw);

        stream.reset();
        fs.close();
    }
    closeLog();
    return 0;
}
