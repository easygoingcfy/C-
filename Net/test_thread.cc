#include <thread>
#include <iostream>

using namespace std;

void foo() {
    cout << "hello" << endl;
}

int main() {
  thread my_thread = thread(foo);
  my_thread.detach();
  cout << "main" << endl;
  return 0;
}