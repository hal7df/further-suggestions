#include <iostream>
using namespace std;

float celsius(int f)
{
    float c;
    c=((5.0/9.0)*(f-32));

    return c;
}

int main ()
{
    int f;

    for (f = -40; f<=220; f=f+10)
        cout << f <<'=' << celsius(f) << endl;
//parameters!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    return 0;
}

