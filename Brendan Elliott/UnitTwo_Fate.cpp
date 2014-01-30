#include <iostream>
#include <string>
#include <sstream>
using namespace std;

int main()
{
    float a;
    int b;
    int d;

    string choice;

    cout << "Would you like to drink the yellow or green potion?:\n";
    getline (cin,choice);
    stringstream(choice) >> a;
    cout << "***you have chosen*** " << choice << endl << endl;

    if (choice == "green"){
        cout << "Please choose door one or door two (number):\n";
        getline (cin,choice);
        stringstream(choice) >> d;
        if (d == 1){
            cout << "you have been eaten by rainbow zombies.";
            }else if (d == 2)
                cout << "You fly to Florida and retire!";
                    else
                        cout << "You run into a door!";
                        return 0;
    }

    if (choice == "yellow"){
        cout << "Please choose door one or door two (number):\n";
        getline (cin,choice);
        stringstream(choice) >> d;
        if (d == 1){
            cout << "you have found a pony!";
            }else if (d == 2)
                cout << "You fall into a pit of kittens!";
                else
                    cout << "you run into a door...FOOL!";
                    return 0;

    }
     else
        cout << "you die";
}









