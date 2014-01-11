#include <iostream>

#include <string>

using namespace std;

int main()
{
    int a;

    string potions;
        cout <<"There are two potions in front of you, yellow or green. Choose one." << endl;
        cin >> potions;

        cout <<"Choose a door, 1 or 2." << endl;
        cin >> a;

    if (a == 1)
    {
        if (potions == "yellow")
            cout <<"You found a pony.";
        else if (potions == "green")
            cout <<"Rainbow zombies approach you, greet you, then continue to eat your flesh. Get rekt m80";
        else
            cout <<"You went through the first door, and fell over dead.";
    }

    if (a == 2)
    {
        if (potions == "yellow")
            cout <<"You have fallen into a bowl of kittens, lul.";
        else if (potions == "green")
            cout <<"A plane stands in front of you, you hop in and fly to Florida to retire.";
        else
            cout <<"You have entered the second door, and fell over dead.";
    }
    if (a != 1 && a != 2)
        cout <<"You have failed to enter a door, and instead decided to run into a door.";
    return 0;
}
