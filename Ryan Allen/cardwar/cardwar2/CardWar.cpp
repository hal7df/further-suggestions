#include "../../cards.h"
int main()
{
    srand(time(NULL));
    int initMoney;
    int wager;
    int clears;
    initMoney = 100;

    Player player (initMoney);

    do
    {
        system("cls");
        cout<<"Current money: $"<<player.get_money();
        Card cards[3];

        cout<<endl<<"You have a "<<cards[0].get_rank()<<" of "<<cards[0].get_suit()<<" and a "<<cards[2].get_rank()<<" of "<<cards[2].get_suit()<<'.'<<endl<<"What is your wager? ";
        cin>>wager;
        while (wager > player.get_money())
        {
            cout<<"You don't have that much money to bet! Please enter a new wager: ";
            cin>>wager;
        };
        cout<<"The next card is: "<<cards[1].get_rank()<<" of "<<cards[1].get_suit()<<'.'<<endl;


        if (cards[0]<cards[1] && cards[1]<cards[2])
        {
            cout<<"You won $"<<wager<<"!"<<endl;
            player.change_money(wager);
        }
        else
        {
            cout<<"You lost $"<<wager<<"!"<<endl;
            player.change_money(-wager);
        }

        if (player.get_money() < 1)
        cout<<"You lost"<<endl;
        else
        cout<<"You won"<<endl;
        cout<<"\n press any number to continue!\n";
        cin>> clears;
    }while(player.get_money() >= 1 || player.get_money() <= 1000);


}
