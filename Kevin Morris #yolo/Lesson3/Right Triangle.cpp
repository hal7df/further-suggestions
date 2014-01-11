#include <iostream>
#include <cstdio>
using namespace std;
int main()
{
    int n;
    int a;
    for (n = 1; n <= 10; n++)
    {
        for(a = 1; a <= n; a++)
        {
           cout <<'+';
        }
    }
    return 0;
}
