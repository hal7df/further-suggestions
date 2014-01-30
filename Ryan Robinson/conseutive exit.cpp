#include <iostream>
#include <string>
using namespace std;

int main()
{
  string str;

  do {
    cout << "would you like to leave the application?]";
    cin >> str;

    if (str != "Yes" & str != "No"){
    cout << "That is not a valid answer, please try again";
    cin >> str;
       }
    }while (str != "No");

    return 0;

}
