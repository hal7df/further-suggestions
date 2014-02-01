#include <iostream>
using namespace std;

int createrect(int x, int y)
{
    int a;
    int b;
    for (b = 1; b <= y; b++)
    {
        for(a = 1; a <= x; a++)
        {
            if (a == 1 or a == x or b == 1 or b == y)
                cout <<'x';
            else
                cout << ' ';
        }
    cout << endl;
    }
}


int main()
{
    int x;
    int y;

    cout << "Insert the height of the rectangle" <<endl;
    cin >> y;
    cout << "Insert the width of the rectangle" <<endl;
    cin >> x;

    if(x > 1 && y > 1)
    {
        createrect(x,y);
    }
    else
    {
        cout << "The rectangle could not be created with such small dimensions.";
    }

}

