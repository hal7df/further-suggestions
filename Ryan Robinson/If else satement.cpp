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
         cout << "You have been eaten by a zombie good job!!!";
    else if(x == 2)
        cout << "Trolls jump on you for being stupid!";
    else
        cout << "What don't listen to me you deserve to be buried alive by zombies";

    return 0;
}


