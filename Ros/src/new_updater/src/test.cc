#include <bitset>
#include <iostream>
#include <string>

using namespace std;

int main() {
    int n = 59;
    bitset<4> b(n);
    cout << b << endl;
    cout << b.to_string() << endl;
    cout << b.to_string().length() << endl;
    return 0;
}