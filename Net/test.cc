#include <iostream>
#include <string>

using namespace std;

int main() {
    char buf[100];
    string s = "hello";
    s.copy(buf, s.length());
    cout << buf << endl;
    cout << "size:" << sizeof(buf) << endl;
    
    return 0;
}