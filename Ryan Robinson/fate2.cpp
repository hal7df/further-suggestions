#include <iostream>
#include <string>
#include <sstream>
using namespace std;

int main ()
{
    string choice;
    int door;

    cout << "You have your own chose to pick your own death which will it be, the yellow or green potion" <<endl<< "Choice now (green/yellow/decline): ";
    cin >> choice;
    cout <<"Please pick door 1 or 2";
    cin >> choice;

    if (choice == "yellow")
        {
        switch (door)
        {
        case 1:
        cout << "you have found a pony";
        break;
        case 2:
        cout << "I am very sorry but you have fallen into a bowl of kittens *cough failure";
        break;
        case 3:
            cout << "You have run into a door... words cannot describe how stupid you are";
            break;
        }

if (choice == "green")
{
    switch (door)
    {
case 1:
cout << "You have been eaten by rainbow zombies, what are rainbow zombies do they exist?";
break;
case 2:
    cout << "You have gone through the one good door and have gone on a plane to Florida to retire";
    break;
case 3:
cout << "You have run into a door, just stop now i dont get it how i mean how???";
break;
    }
}
else
  cout << "You have died have a great life";

return 0;

}
}


