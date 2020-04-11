//
// Created by WYNI on 4/10/2020.
//

#ifndef PROJECT2_SOLVER_H
#define PROJECT2_SOLVER_H

#include <vector>
#include "State.h"
#include "Neighbors.h"
#include "Node.h"

inline bool isInArray(const State &state, const std::vector<std::shared_ptr<Node> > &values)
{
   unsigned int i = 0;
   for (; i < values.size(); ++i)
   {
      if (state == values[i]->GetState())
      {
         return true;
      }
   }
   return false;
}

inline int GetHammingCost(const State &st)
{
   int cost = 0;
   const IntArray &state = st.GetArray();
   for (unsigned int i = 0; i < state.size(); ++i)
   {
      if (state[i] == 0)
      { continue; }
      if (state[i] != i + 1)
      { cost += 1; }
   }
   return cost;
}

inline int GetManhattanCost(const State &st)
{
   int cost = 0;
   const IntArray &state = st.GetArray();
   unsigned int rows_or_cols = st.GetNumRowsOrCols();
   for (unsigned int i = 0; i < state.size(); ++i)
   {
      int v = state[i];
      if (v == 0)
      { continue; }

      // actual index of v should be v-1
      v = v - 1;
      int gx = v % rows_or_cols;
      int gy = v / rows_or_cols;

      int x = i % rows_or_cols;
      int y = i / rows_or_cols;

      int mancost = abs(x - gx) + abs(y - gy);
      cost += mancost;

      int z = 0;
   }
   return cost;
}


class CompareFunctorForGreedyBestFirst
{
public:
    bool operator()(
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = GetManhattanCost(state1) + GetHammingCost(state1);
       const State &state2 = n2->GetState();
       int cost2 = GetManhattanCost(state2) + GetHammingCost(state2);

       return cost1 < cost2;
    }
};

class CompareFunctorForAStar
{
public:
    bool operator()(
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = GetManhattanCost(state1) + GetHammingCost(state1) + n1->GetDepth();
       const State &state2 = n2->GetState();
       int cost2 = GetManhattanCost(state2) + GetHammingCost(state2) + n2->GetDepth();

       return cost1 < cost2;
    }
};

class Solver
{
public:
    enum Type
    {
        DEPTH_FIRST = 0,
        BREADTH_FIRST,
        GREEDY_BEST_FIRST,
        ASTAR,
    };

    Solver(const State &start, const State &goal, Type type = Type::ASTAR)
            : _goal(goal), _solved(false), _type(type)
    {
       NodePtr root(new Node(start, 0, 0));
       _openlist.push_back(root);
    }

    virtual ~Solver()
    {
    }

    inline bool isSolved() const
    {
       return _solved;
    }

    inline bool isSolvable() const
    {
       ///TODO
       return true;
    }

    ///Returns next node in the search.
    //template<class Compare> osg::ref_ptr<Node> GetNextNode(Compare cmp)
    NodePtr GetNextNode()
    {
       if (_openlist.empty())
       { return 0; }
       NodePtr current;

       switch (_type)
       {
          case ASTAR:
          {
             NodeList::iterator current_itr(std::min_element(
                     _openlist.begin(),
                     _openlist.end(),
                     CompareFunctorForAStar()));

             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);

             break;
          }
          case GREEDY_BEST_FIRST:
          {
             NodeList::iterator current_itr(std::min_element(
                     _openlist.begin(),
                     _openlist.end(),
                     CompareFunctorForGreedyBestFirst()));

             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);

             break;
          }
          case BREADTH_FIRST:
          {
             current = _openlist[0];
             _openlist.erase(_openlist.begin());
             _closedlist.push_back(current);

             break;
          }
          case DEPTH_FIRST:
             //current = _openlist[0];
             NodeList::iterator current_itr(_openlist.begin());
             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);

             break;
       }
       return current;
    }

    // expand the graph by looking into the neighbours for the given node.
    void ExpandNode(NodePtr current, const Neighbors &graph)
    {
       if (current->GetState() == _goal)
       {
          _solved = true;
          return;
       }

       int zero = current->GetState().FindEmptyTileIndex();
       const IntArray &neighbours = graph.GetNeighbors(zero);

       for (int next : neighbours)
       {
          State state = current->GetState();
          state.SwapWithEmpty(zero, next);

          if (!isInArray(state, _closedlist))
          {
             NodePtr n(new Node(state, current, current->GetDepth() + 1));
             _openlist.push_back(n);
             static int s_lineNum = 1;
             n->print(std::cout, s_lineNum++);
             //_closedlist.push_back(n);
          }
       }
    }

private:
    typedef std::vector<NodePtr> NodeList;
    NodeList _openlist;
    NodeList _closedlist;
    const State &_goal;
    bool _solved;
    Type _type;
};


#endif //PROJECT2_SOLVER_H
