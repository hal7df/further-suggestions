#include <iostream>

using namespace std;

int main()
{
    int a, b,c;

    cout <<"Enter three numbers ";
    cin >> a;
    cin >> b;
    cin >> c;

    int result1 = a+c;
    int result2 = result1*b;

    cout <<"The sum of the first and third number is: " << result1 << endl;
    cout <<"That number multiplied by the second number is: " << result2 << endl;
    return 0;
}
