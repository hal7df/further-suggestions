// Fate
#include <iostream>
using namespace std;
int y;
string potion;
int main()
{


    cout<<"type either yellow or green potion\n";
    cin>>potion;
    if (potion == "yellow"){
        cin >> y;
        switch(y) {
            case 1:
            cout <<  "user finds a pony\n";
            break;
            case 2:
            cout<< "User falls into a bowl full of kittens" <<endl;
            break;
        default:
            cout<<"User runs into door";}
    }
    else if (potion == "green"){
        cin >> y;
        switch(y) {
            case 1:
            cout <<  "user gets eaten by rainbow zombies\n";
            break;
        case 2:
            cout <<  "user flies to Florida and retires\n";
            break;
        default:
            cout << "User runs into door";
        }
    }
    else
        cout << "User dies\n";

}
