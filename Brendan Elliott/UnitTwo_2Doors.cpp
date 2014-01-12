#include <iostream>
#include <string>
using namespace std;

int main()
{
    int x;
    float y;

    cout << "Please select door one or door two (number):\n";
    cin >> x;

    if (x == 1)
         cout << "You have been ripped limb by limb by a zombie!";
    else if(x == 2)
        cout << "cute little trolls jump on you for hours until you explode!";
    else
        cout << "How dare you defy my instructions";


    return 0;
}


