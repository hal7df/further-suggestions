#include <iostream>
using namespace std;

int main ()
{
  //DECLARATION
  int x;
  int y, z;
  int result;
  
  //Running the code...
  cout<<"Please input three numbers"<<endl;
  cin >> x >> y >> z;
  
  //Sum the numbers
  result = x + y + z;
  
  if (result > 0)
  {
    cout<<"You have a positive result."<<endl;
  }
  else
    cout<<"You have a negative result."<<endl;
  
  cout<<"The sum is "<<result<<'.'<<endl;
  
  return 0;
}