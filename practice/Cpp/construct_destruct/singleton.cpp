#include <iostream>

using namespace std;

class Maker {
 public:
  static Maker* GetInstance() {
    return pMaker;
  }

 private:
  static Maker* pMaker;
  Maker() {}
  Maker(const Maker& maker) {}
  ~Maker() {
    if (pMaker != NULL) {
      delete (pMaker);
      pMaker = NULL;
    }
  }
}

Maker* Maker::pMaker = new Maker;

int main() {}