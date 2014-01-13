#include <iostream>
using namespace std;

int main()
{
  string choice;
  int door;
  
  cout<<"You have a glass in your hand. The bartender gives you the option of a green potion and a yellow potion (you could decline). After that, you could walk through one of two doors."<<endl<<"Potion choice (yellow/green/decline): ";
  cin>>choice;
  cout<<"Door choice (1/2): ";
  cin>>door;
  
  if (choice == "yellow")
  {
    switch (door)
    {
      case 1:
	cout<<"You found a pony!!!"<<endl;
	break;
      case 2:
	cout<<"You fell into a bowl of kittens."<<endl;
	break;
      default:
	cout<<"YOU RAN INTO A DOOR!!!"<<endl;
	break;
    }
  }
  else if (choice == "green")
  {
    switch (door)
    {
      case 1:
	cout<<"You are eaten by RAINBOW zombies!!1"<<endl;
	break;
      case 2:
	cout<<"You flew to Florida and retired."<<endl;
	break;
      default:
	cout<<"YOU RAN INTO A DOOR!!!"<<endl;
    }
  }
  else
    cout<<"You died."<<endl;
  
  return 0;
}