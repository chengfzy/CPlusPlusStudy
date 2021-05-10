/**
 * @brief Print the data size and give a reference when to use const& and value parameters in function.
 * If size(type) > size(void*) then use const &, otherwise use value directly
 */
#include <iostream>
#include <memory>

using namespace std;

#define printSize(type) cout << #type << " = " << sizeof(type) << endl

int main(int argc, char* argv[]) {
    // printSize(void);
    printSize(void*);
    printSize(nullptr);

    printSize(int);
    printSize(int16_t);
    printSize(int32_t);
    printSize(int64_t);
    printSize(short);
    printSize(bool);
    printSize(unsigned int);
    printSize(size_t);
    printSize(float);
    printSize(double);

    printSize(shared_ptr<double>);

    return 0;
}