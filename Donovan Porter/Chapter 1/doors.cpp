#include<iostream>
using namespace std;

int main ()
{
    int a;
    cout<<"There are two doors. Which one do you go through, one or two?" <<endl;
    cin>> a;
    if (a==1)
        cout<<"You are eaten by Zombies"<<endl;
    else if(a==2)
        cout<<"You are pummeled by Trolls"<<endl;
    else
        cout<<"You died" <<endl;

    return 0;

}
