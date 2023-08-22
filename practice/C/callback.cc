#include <iostream>
#include <string>

using namespace std;


void Answer(void* he, string (*answer) (void*)) {
    cout << "the answer is: " << answer(he) << endl;
}


string HopeAnswer(void* cfy) {
    return "Yes";
}

class Cfy {
    private:
      static int age_;
      static double money_;
      bool future_ = false;
    public:
      Cfy(bool future): future_(future){
        cout << "copy constructor call" << endl;
      };
      Cfy(){
        cout << "default constructor call" << endl;
      };
      int GetAge() {
        return age_;
      }
      
      double GetMoney() {
        return money_;
      }

      bool GetFuture() {
        return future_;
      }
};

int Cfy::age_ = 26;
double Cfy::money_ = 50000.0;

int main () {
  Cfy* me = new Cfy;
  cout << "age: " << me->GetAge() << endl;
  cout << "money: " << me->GetMoney() << endl;
  cout << "future: " << me->GetFuture() << endl;
  Answer(me, HopeAnswer);
  return 0;
}