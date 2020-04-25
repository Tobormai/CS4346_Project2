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

   }
   return cost;
}

// ERIK ADDED HERE BELOW

// Function to Calculate S(n) to be implemented into the A* Algorithm
// Rules are to Check around non-central squares - alloting 2 for each tile not followed by its successor
// alotting 0 for every other tile
// Piece in the center scores one
inline int calculateSofN(const State &st) // Non-admissable heuristic function
{
   int SofN = 0;                          // creating variable to hold S(n)
   const IntArray &state = st.GetArray(); // Creating reference to the array
    for (unsigned int i = 0; i < state.size(); ++i)
      {
              // Multiple Rules, and not saved in order due to goal state -Using if/else
              // Check successors in a circle to follow function
              if (i == 0 ){      // Pos 0
              if (state[0] + 1 != state[1]){ // Checks if pos 0 is 1 less then pos 1
              SofN = SofN + 2;}  // Adds 2
    
              } else if(i == 1){ // Pos 1
              if (state[1] + 1 != state[2]){ // Checks if pos 0 is 1 less then pos 1
              SofN = SofN + 2; }
                  
              } else if(i == 2){ // Pos 2 // Checking Down as circular function
              if (state[2] + 1 != state[5]){ // Checks if pos 2 is 1 less then pos 5
              SofN = SofN + 2;}
                  
              } else if(i == 3){ // Pos 3
              if (state[3] + 1 != state[0]){ // Checks if pos 0 is 1 less then pos 3
              SofN = SofN + 2;}
                  
              } else if (i == 4){ // MIDDLE SQUARE GETS 1 if not 0/blank
              if (state[i] != 0){ // Middle is not the blank
              SofN = SofN +1; }// Add one for middle
                  
              } else if(i == 5){ // Pos 5
              if (state[5] + 1 != state[8]){ // Checks if pos 5 is 1 more then pos 8
              SofN = SofN + 2;}
                  
              } else if(i == 6){ // Pos 6
              if (state[6] + 1 != state[3]){ // Checks if pos 6 is 1 less then pos 3
             SofN = SofN + 2;}
                  
              } else if(i == 7){ // Pos 7
              if (state[7] + 1 != state[6]){ // Checks if pos 7 is 1 more then pos 8
              SofN = SofN + 2;}
                  
              } else if(i == 8){ // Pos 8
              if (state[8] + 1 != state[7]){ // Checks if pos 7 is 1 more then 8
              SofN = SofN + 2;}
          }
      }
return SofN;
}


// Function to calculate admissable heuristic based off a subproblem. This function uses the pattern database method
// to create heuristic. Similar to manhattan distance - but can be more effective only using subproblem calculations
// Set pattern of a subproblem of correct values, then calculate distances of to get to that pattern
// For this pattern we will use subproblem of 3,4,5,6 in correct respective positions
inline int calculatePatternDatabase(const State &st) // Subproblem Admissable heuristic function
{
   int cost = 0;                          // creating variable to hold cost
   const IntArray &state = st.GetArray(); // Creating reference to the array
   unsigned int rows_or_cols = st.GetNumRowsOrCols();
    
    for (unsigned int i = 0; i < state.size(); ++i){
    
    // Locate 3,4,5,6
    // Calculate distance from correct positions
    // Save that item as cost
        if(state[i] == 3){
        if(i != 2) // If correct position - does not add to cost
        {
        int v = state[i];   // Calculation of  distance for this only pattern*
        int gx = v % rows_or_cols;
        int gy = v / rows_or_cols;
        int x = i % rows_or_cols;
        int y = i / rows_or_cols;
        int mancost = abs(x - gx) + abs(y - gy);
        cost += mancost;  // Creating Cost
        }
            
        } else if (state[i] == 4){
        if(i != 5) // If correct position - does not add to cost
        {
        int v = state[i];  // Calculation of distance for this only pattern*
        int gx = v % rows_or_cols;
        int gy = v / rows_or_cols;
        int x = i % rows_or_cols;
        int y = i / rows_or_cols;
        int mancost = abs(x - gx) + abs(y - gy);
        cost += mancost;  // Creating Cost
        }
            
        } else if (state[i] == 5){
        if(i != 8) // If correct position - does not add to cost
        {
        int v = state[i]; // Calculation of distance for this only pattern*
        int gx = v % rows_or_cols;
        int gy = v / rows_or_cols;
        int x = i % rows_or_cols;
        int y = i / rows_or_cols;
        int mancost = abs(x - gx) + abs(y - gy);
        cost += mancost;  // Creating Cost
        }
            
        } else if (state[i] == 6){ // Checking value 6 in correct position
        if(i != 7) // If correct position - does not add to cost
        {
        int v = state[i]; // Calculation of distance for this only pattern*
        int gx = v % rows_or_cols;
        int gy = v / rows_or_cols;
        int x = i % rows_or_cols;
        int y = i / rows_or_cols;
        int mancost = abs(x - gx) + abs(y - gy);
        cost += mancost;  // Creating Cost
        }
        } else {
            continue;
        }
      }
return cost;
}

// ERIK ADDED HERE ABOVE

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

// ERIK ADDED BELOW
class CompareFunctorForAStarSN
{
public:
    bool operator()(
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &SN) const
    {
       // Function for h(n) using S(n)
       // H(n) = h1(n)+ 3*S(n)
       const State &state1 = n1->GetState();
       int cost1 = GetHammingCost(state1) + 3 * calculateSofN(state1) + n1->GetDepth();
        
       const State &state2 = SN->GetState();
       int cost2 = GetHammingCost(state2) + 3 * calculateSofN(state2) + SN->GetDepth();
        
       return cost1 < cost2;
    }
};

class ComparePatternDatabase
{
public:
    bool operator()(
             // Using only a portion of manhattan distance - for subproblem
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &n2) const
    {
     
       const State &state1 = n1->GetState();
       int cost1 = calculatePatternDatabase(state1) + n1->GetDepth(); // Cost is based off distance of pattern values
       const State &state2 = n2->GetState();
       int cost2 = calculatePatternDatabase(state2) + n2->GetDepth(); // Cost is based off distance of pattern values
       return cost1 < cost2;
    }
};



// ERIK ADDED ABOVE

class Solver
{
public:
    enum Type
    {
        DEPTH_FIRST = 0,
        BREADTH_FIRST,
        GREEDY_BEST_FIRST,
        ASTAR,
        SMA,
        IDA,
        AStarSN, // ERIK ADDED
        patternDatabase // ERIK ADDED
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
               
               // **** ERIK Added here
            case patternDatabase:
                 {
                     NodeList::iterator current_itr(std::min_element(
                         _openlist.begin(),
                         _openlist.end(),
                         ComparePatternDatabase()));

                         if (current_itr == _openlist.end())
                         { return 0; }

                        //copy the value first to a shared pointer and then erase from the open list.
                        current = *current_itr;

                        // now erase from the open list.
                        _openlist.erase(current_itr);
                        _closedlist.push_back(current);
                        break;
                        }
         
            case AStarSN:
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
               // **** ERIK Added here
               
               
          case ASTAR:
          {
             NodeList::iterator current_itr(std::min_element(
                     _openlist.begin(),
                     _openlist.end(),
                     CompareFunctorForAStarSN()));

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
          case IDA:
          {
             //Why: Want low space complexity but completeness and optimality
             //Key Idea: re-compute elements of the frontier rather than saving them
             //Use DFS to look for solutions at depth 1, then 2, then 3..etc.
             //for depth D, ignore any paths with longer length
             //depth bound is measured in terms of the f value
             //If you don’t find a solution at a given depth –
             // Increase the depth bound: to the minimum of the f-values that exceeded the previous bound

             //Manhattan Distance get approximation of ‘how far’ are we from reaching the final state.
             // IDDFS to search if target is reachable from v.


             //Steps:

             //
             //Perform depth-bounded search at depth = 0.
             // Initialize Frontier = {A}, depth = 0. Remove A from Frontier. Check if A is goal. A at maximum depth bound of depth 0 so don't expand A.
             // Reiterate. Frontier initialized to A. Remove A from Frontier. A is not a goal. A is not at maximum depth bound so expand A.
             // Children of A are B and C. Frontier = {B, C}. B removed from Frontier. B is not a goal. B is at maximum depth bound so don't expand.
             // Children of A are B and C. Frontier = {C}. C removed from Frontier. C is not a goal. C is at maximum depth bound so don't expand.
             //Reiterate. Start all over from A.

             //Note: Order of Expansion or order nodes removed from Frontier was maintained.
             //Note: Depth bound is incremented on each iteration starting at depth = 0
             //Note: Repetitive but has linear space complexity. allows for fast cache memory rather than having to
             // access slower memory. Speedup due to limited amount of memory have to store simultaneously for this algorithm.
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
