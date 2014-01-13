#include <iostream>
using namespace std;

int main ()
{
  int x, y, z;
  int result;
  
  cout<<"Give me three integers."<<endl;
  cin>>x>>y>>z;
  
  result = (x+z)*y;
  
  cout<<result<<endl;
  
  return 0;
}