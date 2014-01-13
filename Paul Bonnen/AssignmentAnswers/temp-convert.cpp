#include <iostream>
#include <iomanip>
using namespace std;

double celsius (double farenheit)
{
  double centigrade;  
    
  centigrade = (5.0/9.0 * (farenheit - 32));
  
  return centigrade;
}

int main ()
{
  double degF;
  int x;
  
  degF = -40;
  
  for (x=0; x <=26; x++)
  {
    //The setw() function provides a set width for an output. It is used here to align the numbers. You do not need it.
    cout<<setw(4)<<degF<<"°F "<<setw(8)<<celsius(degF)<<"°C"<<endl;
    degF = degF + 10;
  };
  
  return 0;
}