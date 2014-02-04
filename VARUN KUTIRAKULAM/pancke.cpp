//pancakegluton
#include <iostream>
using namespace std;
int z;
int pancake [10];
int x;
int person[10] = {1,2,3,4,5,6,7,8,9,10};
int setminmax;
int main() {
    cout<<"please enter numbers for ten people";
    for ( x = 0 ; x <= 9 ; x++)
        cin >> pancake [x];
    for(int x=0; x<10; x++)

	{//bubble sorting

		for(int y=0; y<9; y++)

		{

			if(pancake[y]>pancake[y+1])

			{

				int temp = pancake[y+1];

				pancake[y+1] = pancake[y];

				pancake[y] = temp;
                temp = person[y+1];

				person[y+1] = person [y];

				person [y] = temp;
			}

		}

	}//ending of bubble sorting

	for (x = 0 ; x <= 9 ; x ++){
        cout << pancake [x] << " pancakes were eaten by   " <<person[x]<< " Person" << endl;

    }
	return 0;
}
