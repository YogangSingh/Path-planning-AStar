#include "astar.h"
#include "stdlib.h"
#include "stdio.h"

#include <algorithm>

#include <limits.h>

extern int mymap [1000*1000];
extern myRect *rect;

AStar::AStar(int Width , int Height, int Start, int End)
{
    width = Width;
    height = Height;
    
    start = Start;
    end = End; 
    int size = width*height;
    
    rect = new myRect[size];
    for (int i = 0; i < size; i++) {
        rect[i].x = i% width;
        rect[i].y = i /width;
    }

    rect[start].g_value = 0;
    rect[start].h_value = calc_h_value(start);
    rect[start].pre = NULL;
    
    open_list.push(start);
    
}
AStar::~AStar()
{
    if(rect != NULL)
        delete[] rect;
}
double AStar::calc_g_value(int pos)
{
    return (rect[pos].pre->g_value + 10);
}
double AStar::calc_h_value(int pos)
{
    return 10*(abs(end/width - pos/width)+abs(end%width-pos%width));
}
void AStar::getResultPath()
{
    myRect *temp = &rect[end];
    while(temp != NULL)
    {
        result.push_back(*temp);
        temp = temp->pre;
    }
    return ;
}
bool AStar::isReachable(int pos)
{
    if((pos/width < height)&& (pos /width >=0)&&
       (pos%width >=0))
        return true;
    else
        return false;
}

bool AStar::testRoad(int pos,int cur)
{
    if(isReachable(pos))
    {
        if (pos == end) {
            rect[pos].pre = &rect[cur];
            return true;
        }
        if (mymap[pos] != 1) {
            if(close_list.end() == find(close_list.begin(),close_list.end(),pos))
            {
                MyQueue<int>::const_iterator iter =open_list.find(pos);
                
                if(iter == open_list.myend())
                {
                    rect[pos].pre = &rect[cur];
                    rect[pos].h_value = calc_h_value(pos);
                    rect[pos].g_value = calc_g_value(pos);
                    open_list.push(pos);
                }
                else
                {
                    if((rect[cur].g_value + 10) < rect[pos].g_value)
                    {
                        rect[pos].pre = &rect[cur];
                        rect[pos].g_value = calc_g_value(pos);
                    }
                }
            }
        }
    }
    return false;
}

bool AStar::FindPath()
{
    if(open_list.empty())
        return false;
    int cur = open_list.top();
    
    close_list.push_back(cur);
    open_list.pop();
    
    int up    = cur - width;
    int down  = cur + width;
    int left  = cur - 1;
    int right = cur + 1;
    if (true == testRoad(up, cur))
        return true;
    if (true == testRoad(down, cur))
        return true;
    if (true == testRoad(left, cur))
        return true;
    if (true == testRoad(right, cur))
        return true;

    return FindPath();
}