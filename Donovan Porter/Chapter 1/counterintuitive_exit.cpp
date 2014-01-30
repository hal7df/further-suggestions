#include <iostream>
using namespace std;

int main ()
{
    string str;

    do
    {
        cout << "Would you like to quit the program, yes or no? ";
        cin >> str;
        if (str!="yes" && str != "no") {
            cout << "Improper answer, please reenter ";
            cin >> str;
        }
    } while (str!="no");
    return 0;
}
