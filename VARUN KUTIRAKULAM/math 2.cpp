// first program
#include <iostream>
using namespace std;

int main()
{
    int x;
    int y;
    int z;
    int result;

    cout<<"Input three numbers."<<endl;
    cin>>x>>y>>z;
    if ( y == 0)
    { cout<< "undefined" <<endl;
    cout<<"Input y."<<endl;
    cin>>y;
    //return 0;
    }
    else
    {
      cout<< "Valid numbers" <<endl;
    }
    result = (x + z) / y;
    cout<<"The answer is " << result <<endl;
}
