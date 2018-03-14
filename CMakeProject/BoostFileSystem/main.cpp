#include <iostream>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem;

int main(int argc, char* argv[]) {
    // open and list files
    {
        cout << "========================= List Folder and File =========================" << endl;
        string folder{"/"};
        path folderPath(folder);

        // list folder
        cout << "Folders: " << endl;
        directory_iterator itEnd;
        for (directory_iterator it(folderPath); it != itEnd; ++it) {
            if (is_directory(*it)) {
                cout << "\t" << it->path().filename() << endl;
            }
        }

        // list file
        cout << "Files: " << endl;
        for (directory_iterator it(folderPath); it != itEnd; ++it) {
            if (!is_directory(*it)) {
                cout << "\t" << it->path().filename() << endl;
            }
        }
    }

    // create folder and files
    {
        cout << "========================= Create Folder =========================" << endl;
        string fileName{"/home/jeffery/Downloads/new/new1.gps"};
        path filePath(fileName);
        string folderName = filePath.parent_path().string();
        cout << "folderName = " << folderName << endl;

        if (!exists(filePath)) {
            if (!create_directories(filePath)) {
                cout << "cannot create file \"" << filePath.string() << "\"" << endl;
            }
        }
    }

    return 0;
}
