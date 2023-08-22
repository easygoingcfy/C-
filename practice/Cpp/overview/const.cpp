#include <iostream>

using namespace std;

void test() {
    const int a = 10;
    int * p = (int*)&a;
    *p = 100;
    cout << a << endl;
    cout << *p << endl;
    cout << "address of a: " << &a << endl;
    cout << "address of p point: " << p << endl;

}

int main() {
  test();
  return 0;
}