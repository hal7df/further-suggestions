#include<iostream>
using namespace std;

int createrect (int w, int l)
{
    int x, y;
    for (y=1; y<=l; y++)
    {

            for (x=1; x<=w; x++)
            {
                if (x==1 || x==w || y==1 || y==l)
                    cout<< 'x';
                else
                    cout<< ' ';
            }
       cout << endl;
    }
}

int main ()
{
    int w, l;
    cin>> w;
    cin>> l;
    createrect(w,l);
    return 0;
}
