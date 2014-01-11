#include <iostream>

using namespace std;

int main()
{
    char door1, door2;
    int x;

    cout <<"Pick either door 1 or door 2 by entering a 1 or a 2";
    cin >> x;

    if (x == 1)
        cout <<"You picked door 1, you have been eaten by zombies." << endl;
    else if (x == 2)
        cout <<"You picked door 2, you have been pummeled by trolls." << endl;
    else
        cout <<"You picked an invalid door." << endl;
    return 0;
}
