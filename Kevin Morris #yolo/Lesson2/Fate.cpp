#include <iostream>

#include <string>

using namespace std;

int main()
{
    string potions;
        cout <<"There are two potions in front of you, yellow or green. Choose one."
        cin >> potions

    string a
        cout <<"Choose a door, 1 or 2."
        cin >> a

    if (a == 1)
    {
        if (potions == yellow)
            cout <<"You found a pony."
        else if (potions == green)
            cout <<"Rainbow zombies approach you, greet you, then continue to eat your flesh. Get rekt m80"
        else
            cout <<"You went through the first door, and fell over dead."
    }

    else if (a == 2)
    {
        if (potions == green)
            cout <<"A plane stands in front of you, you hop in and fly to Florida to retire."
        else if (potions == yellow)
            cout <<"You have fallen into a bowl of kittens, lul."
        else
            cout <<"You have entered the second door, and fell over dead."
    }

    else
        cout <<"You have failed to enter a door, and instead decided to run into a door."
    return 0;
}
