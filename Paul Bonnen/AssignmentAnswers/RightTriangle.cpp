#include <iostream>
#include <iomanip>
using namespace std;

int main ()
{
   int x;
   int y;
  
    for (x = 1; x <= 10; x++)
    {
      
      for (y = 1; y <= x; y++)
	cout<<'+';
      
      cout<<endl;
    };
    
  return 0;
}