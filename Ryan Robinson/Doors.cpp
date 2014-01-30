#include <iostream>
#include <cstdlib>
#include <string>
using namespace std;

int main()
{
//******************Number Generator******************
int num1,num2;
int lseed;

lseed = 879377;
srand(lseed);
num1 = rand() % 50 + 1;
num2 = rand() % 9 + 1;
//***********************END**************************
string name;
int add, sub, multi, div, a1, a2, a3, a4; //integers for answers and input
int a,s,m,d; //Choosing addition, subtraction, multiplication, division
//***********************
add = num1 + num2;
sub = num1 - num2;
multi = num1 * num2;
div = num1 / num2;;
//***********************
int input;
cout << "Please enter your name >> ";
cin >> name;
cout << "Glad to be working with you today " << name << ".  What type of problem do you want. ENTER + - / * : " << endl;
cin >> input;
//************************************************ADD
if (input == 'a')
   cout << "What is " << num1 << "+" << num2 << "? >> ";
   cin >> a1;
   if (a1 = add)
      cout << " You are correct!" << endl;
   else
       cout << " Better luck next time.- the correct answer is " << add << endl;
//************************************************SUBTRACT

else if (input == 'm')
   cout << "What is " << num1 << "*" << num2 << "? >> ";
      if (a3 = multi)
      cout << " You are correct!" << endl;
   else
       cout << " Better luck next time.- the correct answer is " << multi << endl;
//************************************************DIVIDE
else if (input == 'd')
   cout << "What is " << num1 << "/" << num2 << "? >> ";
   cin >> a4;
      if (a4 = div)
      cout << " You are correct!" << endl;
   else
       cout << " Better luck next time.- the correct answer is " << div << endl;


system("pause");
return(0);
}



