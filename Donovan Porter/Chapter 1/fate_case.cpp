#include <iostream>
using namespace std;

int main ()
{
    string potion;
    int d;

    //Intro text and choices
    cout << "There are two potions, yellow and a green";
    cout << endl;
    cout << "Which do you choose?" << endl;
    cin >> potion;
    cout << "There are two doors, door 1 and 2." << endl;
    cout << "Which do you choose?" << endl;
    cin >> d;

    //Cases
    if (potion=="yellow")
        {
        switch(d) {
        case 1:
            cout << "You find a pony!";
            break;
        case 2:
            cout << "You fall into a bowl of kittens!";
            break;
        default:
            cout << "You run INTO A DOOR!";
                  }
        }
    else if (potion=="green")
        {
        switch(d) {
        case 1:
            cout << "You get eaten by rainbow zombies!";
            break;
        case 2:
            cout << "You fly to Florida and retire!";
            break;
        default:
            cout << "You run INTO A DOOR!";
                  }
        }
    else {
        cout << "You die!";
         }
    return 0;
}
