#include <string.h>

#include <iostream>

using namespace std;

class Printer {
 private:
  static Printer* pPrinter;
  int mcount;

  Printer() { mcount = 0; }

  Printer(const Printer& printer) {}

  ~Printer() {
    if (pPrinter != NULL) {
      delete pPrinter;
      pPrinter = NULL;
    }
  }

 public:
  static Printer* getPrinter() { return pPrinter; }

  void printPrinter(string name) {
    cout << name << ": 打印" << endl;
    mcount++;
  }

  int getCount() {
    return mcount;
  }

};

Printer* Printer::pPrinter = new Printer;

void test(string name) {
  Printer* p1 = Printer::getPrinter();
  p1->printPrinter(name);
  cout << "目前打印次数：" << p1->getCount() << endl;
}

int main() {
  test("销售部");
  test("技术部");
  test("商业部");
  return 0;
}