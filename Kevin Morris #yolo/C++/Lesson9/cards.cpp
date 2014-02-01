#include <iostream>
using namespace std;

Cards::Cards();

int rand_num = 1 + rand() % 4;  //Random # between 1 and 4.
    switch(rand_num) {
        case 1: suit = "Hearts"; break;
        case 2: suit = "Diamonds"; break;
        case 3: suit = "Clubs"; break;
        case 4: suit = "Spades"; break;
    }

int rand_num = 1 + rand() % 13; //Random # between 1 and 13
    switch(rand_num) {
        case 1: rank = "Ace"; break;
        case 2: rank = "Two"; break;
        case 3: rank = "Three"; break;
        case 4: rank = "Four"; break;
        case 5: rank = "Five"; break;
        case 6: rank = "Six"; break;
        case 7: rank = "Seven"; break;
        case 8: rank = "Eight"; break;
        case 9: rank = "Nine"; break;
        case 10: rank = "Ten"; break;
        case 11: rank = "Jack"; break;
        case 12: rank = "Queen"; break;
        case 13: rank = "King"; break;
    }
