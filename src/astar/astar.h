#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <list>
#include <map>
#include <vector>
#include <queue>

template<class T,class Container=std::vector<T>,
         class Compare = std::less<typename Container::value_type> >
class MyQueue: public std::priority_queue<T,Container,Compare>
{
public:
    typedef typename
    std::priority_queue<T,Container,Compare>::container_type::const_iterator const_iterator;
    const_iterator find(const T& val) const
        {
            auto first = this->c.cbegin();
            auto last = this->c.cend();
            while(first!=last){
                if(*first==val) return first;
                ++first;
            }
            return last;
        }
    const_iterator myend() const
        {
            return this->c.cend();
        }
};


typedef struct _Rect
{
    int x,y;
    
    double h_value;
    double g_value;
    struct _Rect *pre;
}myRect;
extern myRect *rect;

struct comp{
    bool operator ()(int a,int b)
        {
            return rect[a].g_value+rect[a].h_value > rect[b].g_value+rect[b].h_value;
        }
};

class AStar
{
public:
    AStar(int Width,int height,int Start,int End);
    ~AStar();

    bool FindPath();
    void getResultPath();

    double calc_g_value(int pos);
    double calc_h_value(int pos);
    bool isReachable(int pos);
    bool testRoad(int pos,int cur);

    std::list<myRect> result;
        
private:
    int width,height,start,end;
    
    MyQueue<int,std::vector<int>,comp> open_list;
    
    std::list<int> close_list;
    
};

#endif // __ASTAR__
