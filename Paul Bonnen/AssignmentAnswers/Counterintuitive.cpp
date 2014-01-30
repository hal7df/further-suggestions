#include <iostream>
using namespace std;

int main()
{
  string response;
  
  do
  {
    cout<<"Would you like to quit?"<<endl;
    cin>>response;
    
    if (response != "no" && response != "yes")
      cout<<"You did not enter a valid option. Please try again."<<endl;
    
  }while(response != "no");
  
  return 0;
}