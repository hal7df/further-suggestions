#include <iostream>
using namespace std;

int pan [10] = { };
int x;

int main ()
{
    for (int x = 1; x <= 10; x++)
    {
        cout <<"Please insert the amount of pancakes that people 1-10 have eaten" <<endl;
        cin >> pan[x];
    }

    cout<<endl;

    void bubbleSort(int pan,int length);
    {
    int i,j;
    for(i=0;i<10;i++)
        {
        for(j=0;j<i;j++)
            {
            if(pan[i]>pan[j])
                {
                int temp=pan[i];
                pan[i]=pan[j];
                pan[j]=temp;
                }

            }

        }

    }
    void printNum(int pan,int length);
    {
    int i=0;
    for(i=0;i<10;i++)
        cout<<pan[i]<<endl;

    }
    return 0;
}
