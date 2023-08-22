#include <iostream>
using namespace std;

//void Add(int a) {
//    a += 1;
//}

void Add(int* a) {
    cout << "address of pointer a in Add: " << &a << endl;
    *a += 1;
}

void Add(int& a) {
    cout << "address of ref a in Add: " << &a << endl;
    a += 1;
}

int main() {
    int a = 1;
    int* p = &a;
    cout << "addredd of p: " << &p << endl;;
    int& ref = a;
    cout << "addredd of ref: " << &p << endl;;
    Add(ref);
    cout << a << endl;

}