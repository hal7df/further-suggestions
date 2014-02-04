#include <iostream>
#include <string>
#include <sstream>
using namespace std;

struct userinfo{
  string firstnm;
  string lastnm;
  string streetno;
  string streetnm;
  string city;
  string state;
  string zipcode;
  string phoneno;
  string bbg;
  string fulladdress;

} user;

void printuser (userinfo user);

int main ()
{
int x;


  cout << "Enter first name: ";
  getline (cin,user.firstnm);
  cout << "Enter last name: ";
  getline (cin,user.lastnm);
  cout << "Enter street number: ";
  getline (cin,user.streetno);
  cout << "Enter street name: ";
  getline (cin,user.streetnm);
  cout << "Enter city name: ";
  getline (cin,user.city);
  cout << "Enter state: ";
  getline (cin,user.state);
  cout << "Enter zipcode: ";
  getline (cin,user.zipcode);
  cout << "Enter phone number: ";
  getline (cin,user.phoneno);
  cout << "Enter favorite bubble gum flavor: ";
  getline (cin,user.bbg);

  cout <<"What would you like to know about";

  cout <<user.firstnm,user.lastnm;

    cout<<"please type the number for the info you want";

        cout<<"type 1 for full address";
        cout<<"type 2 for phone number";
        cout<<"type 3 for favorite flavor of bubble gum";
        int selection;
            cin>>x;


switch(x) {
case 1:
    cout<<user.streetno;
    cout<<user.streetnm;
    cout<<user.city;
    cout<<user.state;
  break;
case 2:
    cout<<user.phoneno;

  break;
default:
    cout<<user.bbg;

  break;
}
return 0;
}
