#include <archive.h>
#include <archive_entry.h>
#include <fmt/format.h>
#include <boost/filesystem.hpp>
#include <common/common.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
namespace fs = boost::filesystem;

vector<string> files{"./CMakeLists.txt", "./common/include/common/Heading.hpp"};
vector<string> pathName{"CMakeLists.txt", "common/include/common/Heading.hpp"};
string saveFile{"./data/compress.7z"};
fs::path decompressFolder{"./data/"};

// compress
void compress() {
    cout << Title("Compress") << endl;

    // allocate buffer
    constexpr size_t kMaxBuffer{1024 * 1024 * 5};  // 5 MB
    vector<char> compressData(kMaxBuffer, 0);
    size_t actualBufferSize{0};

    // create archive
    archive* a = archive_write_new();
    archive_write_set_format_7zip(a);  // 7z
    // archive_write_set_passphrase(a, "password");  // set passphrase, failed for 7z
    string options = format("7zip:compression=lzma2,compression-level={}", 5);
    archive_write_set_options(a, options.c_str());
    archive_write_open_memory(a, compressData.data(), kMaxBuffer, &actualBufferSize);

    // add first data to archive
    ifstream fs(files[0]);
    vector<uint8_t> data(istreambuf_iterator<char>(fs), {});
    fs.close();
    archive_entry* entry = archive_entry_new();
    archive_entry_set_pathname(entry, pathName[0].c_str());
    archive_entry_set_size(entry, data.size());
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);
    archive_write_header(a, entry);
    archive_write_data(a, data.data(), data.size());

    // add second data to archive
    fs.open(files[1]);
    data = vector<uint8_t>(istreambuf_iterator<char>(fs), {});
    fs.close();
    archive_entry_clear(entry);
    archive_entry_set_pathname(entry, pathName[1].c_str());
    archive_entry_set_size(entry, data.size());
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);
    archive_write_header(a, entry);
    archive_write_data(a, data.data(), data.size());

    // close archive
    archive_entry_free(entry);
    archive_write_close(a);
    archive_write_free(a);

    cout << format("compress data size = {} KB", actualBufferSize / 1024.) << endl;

    // save to file
    ofstream saveFs(saveFile, ios::out | ios::binary);
    saveFs.write(compressData.data(), actualBufferSize);
    saveFs.close();
}

void decompress() {
    cout << Title("Decompress") << endl;

    // open and read file
    ifstream fs(saveFile);
    vector<uint8_t> data(istreambuf_iterator<char>(fs), {});
    fs.close();

    // open archive
    archive* a = archive_read_new();
    int ret{0};
    archive_read_support_format_all(a);
    archive_read_support_filter_all(a);
    archive_read_support_format_raw(a);
    // ret = archive_read_open_file(a, saveFile.c_str(), 10240);
    ret = archive_read_open_memory(a, data.data(), data.size());
    if (ret != ARCHIVE_OK) {
        cerr << format("cannot open compressed data, error: {}, {}", ret, archive_error_string(a)) << endl;
        return;
    }

    // uncompress each entry
    archive_entry* entry;
    while (true) {
        ret = archive_read_next_header(a, &entry);
        if (ret == ARCHIVE_EOF) {
            break;
        }
        if (ret != ARCHIVE_OK) {
            cerr << format("read header error: {}, {}", ret, archive_error_string(a)) << endl;
        }

        // get file name
        string fileName = string(archive_entry_pathname(entry));
        cout << format("uncompress {}", fileName) << endl;

        // read data
        vector<char> entryBuffer;
        if (archive_entry_size(entry) > 0) {
            const void* tempBuf{nullptr};
            size_t size{0};
            la_int64_t offset{0};
            while (true) {
                ret = archive_read_data_block(a, &tempBuf, &size, &offset);
                if (ret == ARCHIVE_EOF) {
                    break;
                } else if (ret != ARCHIVE_OK) {
                    cerr << format("read data block error: {}, {}", ret, archive_error_string(a)) << endl;
                }
                entryBuffer.insert(entryBuffer.end(), static_cast<const char*>(tempBuf),
                                   static_cast<const char*>(tempBuf) + size);
            }

            // save entry for file
            fs::path savePath = decompressFolder / fileName;
            fs::create_directories(savePath.parent_path());
            ofstream saveFs(savePath.string(), ios::out | ios::binary);
            saveFs.write(entryBuffer.data(), entryBuffer.size());
            saveFs.close();
        }
    }

    // close
    archive_read_close(a);
    archive_read_free(a);
}

int main(int argc, const char* argv[]) {
    compress();
    decompress();

    return 0;
}