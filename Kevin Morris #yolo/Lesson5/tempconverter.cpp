#include <iostream>
#include <iomanip>
using namespace std;

    float celsius (float fahrenheit)
    {
        float centigrade;
        centigrade = (5.0/9.0 * (fahrenheit - 32));
        return centigrade;
    }

int main()
{
    float a;
    float f;

    f = -40;

    for(a = 1; a <= 26; a++)
    {
       cout <<f+1<<"F"
    };

}
