#include <iostream>
#include <string>
#include <sstream>
using namespace std;

struct info {
    string namef, namel;
    int street;
    string streetn, city;
    int zip, phone;
    string gum;

} user;

int main ()
{

    cout << "Enter your first name: ";
    cin >> user.namef;
    cout << "Enter your last name: ";
    cin >> user.namel;
    cout << "Enter your street number: ";
    cin >> user.street;
    cout << "Enter your street name: ";
    cin >> user.streetn;
    cout << "Enter your city: ";
    cin >> user.city;
    cout << "Enter your zip code: ";
    cin >> user.zip;
    cout << "Enter your phone number: ";
    cin >> user.phone;
    cout << "Enter your favorite flavor gum: ";
    cin >> user.gum;

    int a;
    cout << "What would you like to know? " << user.namef << ' ' << user.namel << endl;
    cout << "1. Full Address" << endl;
    cout << "2. Phone Number" << endl;
    cout << "3. Favorite bubble gum flavor" << endl;
    cin >> a;

    switch (a) {
        case 1:
            cout << "Your address is: " << user.street << " " << user.streetn << " " << user.city << " " << user.zip << endl;
            break;
        case 2:
            cout << "Your phone number is: " << user.phone;
            break;
        case 3:
            cout << "Your favorite flavor of gum is: " << user.gum;
            break;
        default:
            cout << "You have entered an invalid number.";
        }
    return 0;
}
