//temperatureconverterbyvarun
#include <iostream>
#include <iomanip>
using namespace std;

    float celsius (float Fahrenheit)
        {
            float centigrd;
                centigrd = (5.0/9.0 * (Fahrenheit - 32));
            return centigrd;
        }

int main()
{
    float c;
    float f;

    f = -50;

    for(c = 0; c <= 26; c++)
        {
            f = f+10;
            cout<<f<<"F"<<" becomes "<<celsius(f)<<"C"<<endl;
        };
    return 0;
}
