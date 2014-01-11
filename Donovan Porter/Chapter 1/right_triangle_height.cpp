#include <iostream>
using namespace std;

int main ()
{
    int r;

    // User input
    while(true)
    {
        cout << "Input Number: ";
        cin >> r;
        if(r>0)
        {
            break;
        } else
        {
            cout << "Retry.\n";
        }
    }

    // Create Triangle
    for(int i=1; i <= r; i++)
    {
        for(int j=1; j <= i; j++)
        {
            cout << "x";
        }
        cout << "\n";
    }
    return 0;
}
