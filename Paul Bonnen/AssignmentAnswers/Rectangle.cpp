#include <iostream>
#include <iomanip>
using namespace std;

void createsquare (int x, int y)
{
  int a, b;
  
  for (b = 0; b < y; b++)
  {
    
    for (a = 0; a < x; a++)
    {
      //If on the border
      if (a == 0 || a == (x-1) || b == 0 || b == (y-1))
	cout<<'x';
    
      //If in the middle somewhere
      else
	cout<<' ';
    }
    
    cout<<endl;
  }
}


int main ()
{
  int x, y;
  
  cout<<"Give me the dimensions of the square"<<endl;
  cin>>x>>y;
  
  createsquare(x,y);
  
  return 0;
}