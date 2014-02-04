// Fate
#include <iostream>
using namespace std;

int main()
{
    string potion;

    cout<<"type either yellow or green potion";
    cin>>potion;
    int y;
    cout<<"Enter either door 1 or door 2."<<endl;
    cin>>y;
if (y==NULL)
{
    cout<<"user RAN INTO A DOOR"<<endl;
}
if (potion=="No potion")
{
    cout<<"user dies"<<endl;
}
    if ( y == 1)
    {
        if (potion =="yellow")
            cout<< "user finds a pony" <<endl;
        else
            cout<< "User is eaten by rainbow zombies" <<endl;
    }
    else
    {
        if (potion == "yellow")
            cout<< "User falls into a bowl full of kittens" <<endl;
        else
            cout<< "User flies to Florida and retires" <<endl;

    }

}
