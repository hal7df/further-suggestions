#include <iostream>
using namespace std;
int main()
{
    int door;
    int b;

    string potions;
        cout <<"There are two potions in front of you, yellow or green. Choose one." << endl;
        cin >> potions;

        if (potions == "yellow")
            b = 1;
        else if (potions == "green")
            b = 2;
        else {
            cout <<"You have died" << endl;
            return 0;
        }
        cout <<"Choose a door, 1 or 2." << endl;
        cin >> door;

    if (door == 1)
    {
        switch (b) {
        case 1:
            cout <<"You found a pony!";
            break;
        case 2:
            cout <<"You were eaten by rainbow zombies!";
            break;
        }
    }

    if (door == 2)
    {
        switch (b) {
        case 1:
            cout <<"You have fallen into a bowl of kittens!";
            break;
        case 2:
            cout <<"You find a plane and fly to Florida.";
            break;
        }
    }
    if (door != 1 && door != 2)
        cout <<"You have failed to enter a door, and instead decided to run into a door.";
}
