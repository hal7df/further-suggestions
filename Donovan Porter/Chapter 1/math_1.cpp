#include<iostream>
using namespace std;

int main ()
{
    int a;
    int b;
    int c;
    int answer;

    cout<< "Please input three numbers."<<endl;
    cin>>a;
    cin>>b;
    cin>>c;

    cout<< "I will then add the first and third numbers, then multiply the sum by the third." <<endl;

    answer=(a+c)*b;

    cout<< "The answer is " <<answer<< endl;
return 0;
}
