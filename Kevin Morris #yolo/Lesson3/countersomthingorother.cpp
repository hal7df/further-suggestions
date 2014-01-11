#include <iostream>
using namespace std;
int main()
{
    string quit;
    do {
        cout << "Would you like to quit the program?" << endl;
        cin >> quit;
        if (quit != "yes" && quit != "no") {
            cout <<"Improper answer." << endl;
        }
    } while(quit != "no");
}
