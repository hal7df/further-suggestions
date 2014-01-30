/** NOTE: This source only works on Linux
 * POSIX-compliant systems!!!!
 */

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <array>
#include <unistd.h>
#include <term.h>
#include <termios.h>

//Define tile types
#define PLAYER 'G'
#define ENEMY 'E'
#define TRAP 'T'
#define TREASURE 'X'

//Define direction bindings
#define UP 'w'
#define LEFT 'a'
#define DOWN 's'
#define RIGHT 'd'

using namespace std;

struct location {
  int x;
  int y;
};

enum state_t {
    good,
    loss,
    win
};

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

void cls()
{
  if (!cur_term)
  {
    int result;
    setupterm(NULL,STDOUT_FILENO,&result);
    if (result <= 0)
      return;
  }

  putp(tigetstr("clear"));
}

array<array<char,10>,10> resetMap ()
{
    int x, y;

    int enemies, traps;
    bool treasure;
    bool player;
    bool backward;

    array<array<char,10>,10> dmap;

    enemies = 0;
    traps = 0;
    treasure = false;
    player = false;
    backward = false;

    //Initialize a blank map
    for (y=0; y<10; y++)
    {
        for (x=0; x<10; x++)
            dmap.at(y).at(x)='.';
    }

    x = 0;
    y = 0;

    //Fill the map with characters
    do
    {
        if (dmap.at(y).at(x) == '.')
        {
            //Place the player
            if ((rand() % 100) == 0 && !player)
            {
                dmap.at(y).at(x)=PLAYER;
                player = true;
            }

            //Place the treasure
            if ((rand() % 100) == 0 && !treasure)
            {
                dmap.at(y).at(x)=TREASURE;
                treasure = true;
            }

            //Place the enemies
            if ((rand() % 50) == 0 && enemies < 2)
            {
                dmap.at(y).at(x)=ENEMY;
                enemies++;
            }

            //Place the traps
            if ((rand() % 20) == 0 && traps < 5)
            {
                dmap.at(y).at(x)=TRAP;
                traps++;
            }

        }

        //Move to the next tile
        //If iterating backwards through the map
        if (backward)
        {
            if (x == 0 && y == 0)
            {
                backward = false;
                x++;
            }
            else if (x == 0)
            {
                x = 9;
                y--;
            }
            else
                x--;
        }

        //If iterating forwards through the map
        else
        {
            if (x == 9 && y == 9)
            {
                backward = true;
                x--;
            }
            else if (x == 9)
            {
                x = 0;
                y++;
            }
            else
                x++;
        }

    }while(enemies < 2 || traps < 5 || !player || !treasure);

    return dmap;
}

void drawMap (array<array<char,10>,10> dmap)
{
  int x, y;

  cls();

  for (y = 0; y < 10; y++)
    {
      for (x = 0; x < 10; x++)
        cout<<dmap.at(y).at(x)<<' ';
      cout<<endl;
    }
}

location findTile (char tType, array<array<char,10>,10> dmap, int instance = 0)
{
    location tLoc;
    int x, y;
    int occur;

    tLoc.x = -1;
    tLoc.y = -1;
    x = 0;
    y = 0;
    occur = 0;

    do
    {
        if (dmap.at(y).at(x) == tType && occur == instance)
        {
            tLoc.x = x;
            tLoc.y = y;
        }
        else if (dmap.at(y).at(x) == tType)
        {
            occur++;

            if (x == 9)
            {
                y++;
                x = 0;
            }
            else
                x++;
        }
        else
        {
            if (x == 9)
            {
                y++;
                x = 0;
            }
            else
                x++;
        }
    }while(tLoc.x == -1 && y < 10);

    return tLoc;
}

state_t movePlayer (array<array<char,10>,10>& dmap)
{
    int x, y;

    location player, treasure;
    location enemy [2];
    location trap [5];
    int livingEnemies;
    bool enemyDies;

    livingEnemies = 2;

    //Find all of the relevant tiles
    player = findTile('G',dmap,0);
    treasure = findTile('X',dmap,0);

    for (x = 0; x < 2; x++)
        enemy[x] = findTile(ENEMY,dmap,x);

    for (x = 0; x < 5; x++)
        trap[x] = findTile(TRAP,dmap,x);

    //Check to see how many enemies are living
    for (x = 0; x < 2; x++)
    {
        if (enemy[x].x == -1)
            livingEnemies--;
    }

    //Ensure all of the necessary tiles are present
    if (player.x == -1)
    {
        cerr<<"Error: Could not find player location. Resetting map..."<<endl;
        usleep(100000);
        dmap = resetMap();
        return state_t::good;
    }
    if (treasure.x == -1)
    {
        cerr<<"Error: Could not find treasure location. Resetting map..."<<endl;
        usleep(100000);
        dmap = resetMap();
        return state_t::good;
    }

    //Move the player
    switch(getch())
    {
        //Move the player up
        case UP:
            if (player.y == 0) //If they are trying to move beyond the edge of the map
                return state_t::good;
            else
            {
                dmap.at(player.y).at(player.x) = '.';

                //Perform loss checks
                for (x = 0; x < livingEnemies; x++)
                {
                    if (player.y-1 == enemy[x].y && player.x == enemy[x].x)
                        return state_t::loss;
                }
                for (x = 0; x < 5; x++)
                {
                    if (player.y-1 == trap[x].y && player.x == trap[x].x)
                        return state_t::loss;
                }

                //Perform win check
                if (player.y-1 == treasure.y && player.x == treasure.x)
                    return state_t::win;

                dmap.at(player.y-1).at(player.x) = PLAYER;
                player.y--;
            }
            break;

        //Move the player left
        case LEFT:
            if (player.x == 0)
                return state_t::good; //If they are trying to move beyond the edge of the map
            else
            {
                dmap.at(player.y).at(player.x) = '.';

                //Perform loss checks
                for (x = 0; x < livingEnemies; x++)
                {
                    if (player.y == enemy[x].y && player.x-1 == enemy[x].x)
                        return state_t::loss;
                }
                for (x = 0; x < 5; x++)
                {
                    if (player.y == trap[x].y && player.x-1 == trap[x].x)
                        return state_t::loss;
                }

                //Perform win check
                if (player.y == treasure.y && player.x-1 == treasure.x)
                    return state_t::win;

                dmap.at(player.y).at(player.x-1) = PLAYER;
                player.x--;
            }
            break;

        //Move the player down
        case DOWN:
            if (player.y == 9)
                return state_t::good; //If they are trying to move beyond the edge of the map
            else
            {
                dmap.at(player.y).at(player.x) = '.';

                //Perform loss checks
                for (x = 0; x < livingEnemies; x++)
                {
                    if (player.y+1 == enemy[x].y && player.x == enemy[x].x)
                        return state_t::loss;
                }
                for (x = 0; x < 5; x++)
                {
                    if (player.y+1 == trap[x].y && player.x == trap[x].x)
                        return state_t::loss;
                }

                //Perform win check
                if (player.y+1 == treasure.y && player.x == treasure.x)
                    return state_t::win;

                dmap.at(player.y+1).at(player.x) = PLAYER;
                player.y++;
            }
            break;

        //Move the player right
        case RIGHT:
            if (player.x == 9)
                return state_t::good; //If they are trying to move beyond the edge of the map
            else
            {
                dmap.at(player.y).at(player.x) = '.';

                for (x = 0; x < livingEnemies; x++)
                {
                    if (player.y == enemy[x].y && player.x+1 == enemy[x].x)
                        return state_t::loss;
                }
                for (x = 0; x < 5; x++)
                {
                    if (player.y == trap[x].y && player.x+1 == trap[x].x)
                        return state_t::loss;
                }

                //Perform win check
                if (player.y == treasure.y && player.x+1 == treasure.x)
                    return state_t::win;

                dmap.at(player.y).at(player.x+1) = PLAYER;
                player.x++;
            }
            break;

        //The player did not hit a valid key
        default:
            return state_t::good;
    }

    //Move the enemies
    for (x=0; x<livingEnemies; x++)
    {
        enemyDies = false;
        switch (rand() % 4)
        {
            //Move the enemy up
            case 0:
                if ((enemy[x].y-1 != treasure.y && enemy[x].x != treasure.x) && (enemy[x].y != 0))
                {
                    dmap.at(enemy[x].y).at(enemy[x].x) = '.';

                    for (y = 0; y < 5; y++)
                    {
                        if (enemy[x].y-1 == trap[y].y && enemy[x].x == trap[y].x)
                            enemyDies = true;
                    }

                    if (enemy[x].y-1 == player.y && enemy[x].x == player.x)
                    {
                        dmap.at(enemy[x].y-1).at(enemy[x].x) = ENEMY;
                        return state_t::loss;
                    }

                    if (!enemyDies)
                        dmap.at(enemy[x].y-1).at(enemy[x].x) = ENEMY;
                }
                break;

            //Move the enemy left
            case 1:
                if ((enemy[x].y != treasure.y && enemy[x].x-1 != treasure.x) && (enemy[x].x != 0))
                {
                    dmap.at(enemy[x].y).at(enemy[x].x) = '.';

                    for (y = 0; y < 5; y++)
                    {
                        if (enemy[x].y == trap[y].y && enemy[x].x-1 == trap[y].x)
                            enemyDies = true;
                    }

                    if (enemy[x].y == player.y && enemy[x].x-1 == player.x)
                    {
                        dmap.at(enemy[x].y).at(enemy[x].x-1) = ENEMY;
                        return state_t::loss;
                    }

                    if (!enemyDies)
                        dmap.at(enemy[x].y).at(enemy[x].x-1) = ENEMY;
                }
                break;

            //Move the enemy down
            case 2:
                if ((enemy[x].y+1 != treasure.y && enemy[x].x != treasure.x) && (enemy[x].y != 9))
                {
                    dmap.at(enemy[x].y).at(enemy[x].x) = '.';

                    for (y = 0; y < 5; y++)
                    {
                        if (enemy[x].y+1 == trap[y].y && enemy[x].x == trap[y].x)
                            enemyDies = true;
                    }

                    if (enemy[x].y+1 == player.y && enemy[x].x == player.x)
                    {
                        dmap.at(enemy[x].y+1).at(enemy[x].x) = ENEMY;
                        return state_t::loss;
                    }

                    if (!enemyDies)
                        dmap.at(enemy[x].y+1).at(enemy[x].x) = ENEMY;
                }
                break;

            //Move the enemy right
            case 3:
                if ((enemy[x].y != treasure.y && enemy[x].x+1 != treasure.x) && (enemy[x].x != 9))
                {
                    dmap.at(enemy[x].y).at(enemy[x].x) = '.';

                    for (y = 0; y < 5; y++)
                    {
                        if (enemy[x].y == trap[y].y && enemy[x].x+1 == trap[y].x)
                            enemyDies = true;
                    }

                    if (enemy[x].y == player.y && enemy[x].x+1 == player.x)
                    {
                        dmap.at(enemy[x].y).at(enemy[x].x+1) = ENEMY;
                        return state_t::loss;
                    }

                    if (!enemyDies)
                        dmap.at(enemy[x].y).at(enemy[x].x+1) = ENEMY;
                }
                break;
        }
    }

    return state_t::good;
}

int main()
{
    srand (time(NULL));

    int x, y;
    array<array<char,10>,10> dmap;
    state_t game_state;

    do
    {
        game_state = good;
        dmap = resetMap();
        drawMap(dmap);

        do
        {
            game_state = movePlayer(dmap);
            drawMap(dmap);
        }while(game_state == good);

        //Display a message
        if (game_state == win)
            cout<<"You won!"<<endl;
        else
            cout<<"You lost!"<<endl;

        //Ask the user if they want to play again
        cout<<"Play again?"<<endl<<"Yes: y"<<endl<<"No: n"<<endl;
    }while (getch() == 'y');

    return 0;
}
