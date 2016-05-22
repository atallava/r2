#include <string>
#include <serial_api/Test.h>

using namespace serial_api;

int main() {
    bool res;
    Test t = Test();

    res = t.testBuild();
    printf("Test build: %d.\n",res);
}
