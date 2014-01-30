#include <iostream>
using namespace std;

int main ()
{
  int choice;
  
  cout<<"You see two doors. Which one do you choose (use a number)?"<<endl;
  cin>>choice;
  
  if (choice == 1)
    cout<<"You are eaten by zombies."<<endl;
  else
    cout<<"You are pummeled by trolls."<<endl;
  
  return 0;
}
