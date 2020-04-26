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

//(h1) : number of misplaced tiles for _state
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

//(h2): sum of the distances of each tile from desired goal state
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

inline int GetThreshold(const std::shared_ptr<Node> &n1)
{
   {
      const State &state1 = n1->GetState();
      int cost1 = GetManhattanCost(state1) + n1->GetDepth();
/*       std::cout << " h1(n):" << GetHammingCost(state1) << " g(n):" << n1->GetDepth() << " f(n): "
                 << GetHammingCost(state1) + n1->GetDepth() << endl;*/
      return cost1;
   }
}

inline int Search(std::shared_ptr<Node> node, int gScore, int threshold)
{
   {
      std::shared_ptr<Node> node;
      const State &state1 = node->GetState();
      int gScore = node->GetDepth();
      int f = gScore + GetManhattanCost(state1);

      if (f > threshold)
      {
         return f;
      }
      //if(node->GetState() )
      {

      }

   }
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
      if (i == 0)
      {      // Pos 0
         if (state[0] + 1 != state[1])
         { // Checks if pos 0 is 1 less then pos 1
            SofN = SofN + 2;
         }  // Adds 2

      }
      else
      {
         if (i == 1)
         { // Pos 1
            if (state[1] + 1 != state[2])
            { // Checks if pos 1 is 1 less then pos 2
               SofN = SofN + 2;
            }

         }
         else
         {
            if (i == 2)
            { // Pos 2 // Checking Down as circular function
               if (state[2] + 1 != state[5])
               { // Checks if pos 2 is 1 less then pos 5
                  SofN = SofN + 2;
               }

            }
            else
            {
               if (i == 3)
               { // Pos 3
                  if (state[3] + 1 != state[0])
                  { // Checks if pos 0 is 1 less then pos 3
                     SofN = SofN + 2;
                  }

               }
               else
               {
                  if (i == 4)
                  { // MIDDLE SQUARE GETS 1 if not 0/blank
                     if (state[i] != 0)
                     { // Middle is not the blank
                        SofN = SofN + 1;
                     }// Add one for middle

                  }
                  else
                  {
                     if (i == 5)
                     { // Pos 5
                        if (state[5] + 1 != state[8])
                        { // Checks if pos 5 is 1 more then pos 8
                           SofN = SofN + 2;
                        }

                     }
                     else
                     {
                        if (i == 6)
                        { // Pos 6
                           if (state[6] + 1 != state[3])
                           { // Checks if pos 6 is 1 less then pos 3
                              SofN = SofN + 2;
                           }

                        }
                        else
                        {
                           if (i == 7)
                           { // Pos 7
                              if (state[7] + 1 != state[6])
                              { // Checks if pos 7 is 1 more then pos 8
                                 SofN = SofN + 2;
                              }

                           }
                           else
                           {
                              if (i == 8)
                              { // Pos 8
                                 if (state[8] + 1 != state[7])
                                 { // Checks if pos 7 is 1 more then 8
                                    SofN = SofN + 2;
                                 }
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
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

   for (unsigned int i = 0; i < state.size(); ++i)
   {

      // Locate 3,4,5,6
      // Calculate distance from correct positions
      // Save that item as cost
      if (state[i] == 3)
      {
         if (i != 2) // If correct position - does not add to cost
         {
            int v = state[i];   // Calculation of  distance for this only pattern*
            int gx = v % rows_or_cols;
            int gy = v / rows_or_cols;
            int x = i % rows_or_cols;
            int y = i / rows_or_cols;
            int mancost = abs(x - gx) + abs(y - gy);
            cost += mancost;  // Creating Cost
         }

      }
      else
      {
         if (state[i] == 4)
         {
            if (i != 5) // If correct position - does not add to cost
            {
               int v = state[i];  // Calculation of distance for this only pattern*
               int gx = v % rows_or_cols;
               int gy = v / rows_or_cols;
               int x = i % rows_or_cols;
               int y = i / rows_or_cols;
               int mancost = abs(x - gx) + abs(y - gy);
               cost += mancost;  // Creating Cost
            }

         }
         else
         {
            if (state[i] == 5)
            {
               if (i != 8) // If correct position - does not add to cost
               {
                  int v = state[i]; // Calculation of distance for this only pattern*
                  int gx = v % rows_or_cols;
                  int gy = v / rows_or_cols;
                  int x = i % rows_or_cols;
                  int y = i / rows_or_cols;
                  int mancost = abs(x - gx) + abs(y - gy);
                  cost += mancost;  // Creating Cost
               }

            }
            else
            {
               if (state[i] == 6)
               { // Checking value 6 in correct position
                  if (i != 7) // If correct position - does not add to cost
                  {
                     int v = state[i]; // Calculation of distance for this only pattern*
                     int gx = v % rows_or_cols;
                     int gy = v / rows_or_cols;
                     int x = i % rows_or_cols;
                     int y = i / rows_or_cols;
                     int mancost = abs(x - gx) + abs(y - gy);
                     cost += mancost;  // Creating Cost
                  }
               }
               else
               {
                  continue;
               }
            }
         }
      }
   }
   return cost;
}


// A heuristic function for 8 square to calculate if numbers are in the correct rows or not
// This function will be admissable because the cost of not being in the correct row would not be overestimated
// Function will search each value, and then do a check if its in the correct row
// If the value is not in the correct row, we will add a cost of 1 - as it would have to move at least 1 to get to correct position
inline int heuristicOfRows(const State &st)
{
   // Admissable - does not over estimate cost
   int cost = 0;
   const IntArray &state = st.GetArray();

   for (unsigned int i = 0; i < state.size(); ++i) // Going through full state
   {
      // Multiple Rules, and not saved in order due to goal state -Using if/else vs loop
      if (i == 0)
      {      // Pos 0
         if (state[3] && state[4] && state[5] != 0)
         { // Checks if 0 is in the correct ROW
            cost = cost + 1;
         }

      }
      else
      {
         if (i == 1)
         { //  1
            if (state[0] && state[1] && state[2] != 1)
            { // Checks if 1 in in the correct ROW
               cost = cost + 1;
            }

         }
         else
         {
            if (i == 2)
            { //  2
               if (state[0] && state[1] && state[2] != 2)
               { // Checks if 2 in in the correct ROW
                  cost = cost + 1;
               }

            }
            else
            {
               if (i == 3)
               { //  3
                  if (state[0] && state[1] && state[2] != 3)
                  { // Checks if 3 in in the correct ROW
                     cost = cost + 1;
                  }

               }
               else
               {
                  if (i == 4)
                  { //  4
                     if (state[3] && state[4] && state[5] != 4)
                     { // Checks if 4 in in the correct ROW
                        cost = cost + 1;
                     }

                  }
                  else
                  {
                     if (i == 5)
                     { //  5
                        if (state[6] && state[7] && state[8] != 5)
                        { // Checks if 5 in in the correct ROW
                           cost = cost + 1;
                        }

                     }
                     else
                     {
                        if (i == 6)
                        { //  6
                           if (state[6] && state[7] && state[8] != 6)
                           { // Checks if 6 in in the correct ROW
                              cost = cost + 1;
                           }

                        }
                        else
                        {
                           if (i == 7)
                           { //  7
                              if (state[6] && state[7] && state[8] != 7)
                              { // Checks if 7 in in the correct ROW
                                 cost = cost + 1;
                              }

                           }
                           else
                           {
                              if (i == 8)
                              { //  8
                                 if (state[3] && state[4] && state[5] != 8)
                                 { // Checks if 8 in in the correct ROW
                                    cost = cost + 1;
                                 }
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   return cost;
}
//ERIK ADDED HERE ABOVE

// Madison Start
inline int GetDiagonalCost(const State &st)
{
   int cost = 0;
   int temp = 0;
   const IntArray &state = st.GetArray();
   for (unsigned int i = 0; i < state.size(); ++i)
   {
      temp = state[i];

      if (temp == 0)
      { continue; }
      if (temp == 1)
      {
         if (i == 0)
         { continue; }
         if (i == 1 || i == 4 || i == 3)
         {
            cost = cost + 1;
         }
         else
         {
            cost = cost + 2;
         }


      }
      if (temp == 2)
      {
         if (i == 1)
         { continue; }
         if (i == 6 || i == 7 || i == 8)
         {
            cost = cost + 2;
         }
         else
         {
            cost = cost + 1;
         }

      }
      if (temp == 3)
      {
         if (i == 2)
         { continue; }
         if (i == 1 || i == 4 || i == 5)
         {
            cost = cost + 1;
         }
         else
         {
            cost = cost + 2;
         }

      }
      if (temp == 4)
      {
         if (i == 3)
         { continue; }
         if (i == 2 || i == 5 || i == 8)
         {
            cost = cost + 2;
         }
         else
         {
            cost = cost + 1;
         }

      }
      if (temp == 5)
      {
         if (i == 4)
         { continue; }
         else
         {
            cost = cost + 1;
         }

      }
      if (temp == 6)
      {
         if (i == 5)
         { continue; }
         if (i == 0 || i == 3 || i == 6)
         {
            cost = cost + 2;
         }
         else
         {
            cost = cost + 1;
         }

      }
      if (temp == 7)
      {
         if (i == 6)
         { continue; }
         if (i == 7 || i == 4 || i == 3)
         {
            cost = cost + 1;
         }
         else
         {
            cost = cost + 2;
         }

      }
      if (temp == 8)
      {
         if (i == 7)
         { continue; }
         if (i == 2 || i == 1 || i == 0)
         {
            cost = cost + 2;
         }
         else
         {
            cost = cost + 1;
         }

      }
   }

   return cost;

}
// Madison End


//find minimum value of the openlist elements based on GREEDYBESTFIRST algorithm
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

class CompareFunctorForAStar_H1
{
public:
    //return bool for compare nodes f(n) value
    bool operator()(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = GetHammingCost(state1) + n1->GetDepth();
/*       std::cout << " h1(n):" << GetHammingCost(state1) << " g(n):" << n1->GetDepth() << " f(n): "
                 << GetHammingCost(state1) + n1->GetDepth() << endl;*/

       const State &state2 = n2->GetState();
       int cost2 = GetHammingCost(state2) + n2->GetDepth();
/*       std::cout << " h1(n):" << GetHammingCost(state2) << " g(n):" << n2->GetDepth() << " f(n): "
                 << GetHammingCost(state1) + n2->GetDepth() << endl;*/
       return cost1 < cost2;
    }
};

//find minimum value of the openlist elements based on ASTAR algorithm
//ASTAR selects node with the least f-score
class CompareFunctorForAStar_H2
{
public:
    bool operator()(
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = GetManhattanCost(state1) + n1->GetDepth();
       /*  std::cout << "h2(n):" << GetManhattanCost(state1) << " g(n):" << n1->GetDepth() << " f(n): "
                   << GetManhattanCost(state1) + n1->GetDepth()
                   << endl;*/

       const State &state2 = n2->GetState();
       int cost2 = GetManhattanCost(state2) + n2->GetDepth();
       /*     std::cout << "h2(n):" << GetManhattanCost(state2) << " g(n):" << n2->GetDepth() << " f(n): "
                      << GetManhattanCost(state2) + n2->GetDepth()
                      << endl;*/
       return cost1 < cost2;
    }
};

class CompareFunctorForAStar_IDA
{
public:

    bool operator()(std::shared_ptr<Node> &n1, std::shared_ptr<Node> &n2)
    {
       const State &state1 = n1->GetState();
       int cost1 = GetManhattanCost(state1) + n1->GetDepth();

       const State &state2 = n2->GetState();
       int cost2 = GetManhattanCost(state2) + n2->GetDepth();
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

class CompareheuristicOfRows
{
public:
    bool operator()(
            // Using only a portion of manhattan distance - for subproblem
            const std::shared_ptr<Node> &n1,
            const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = heuristicOfRows(state1) + n1->GetDepth(); // Cost is based off distance of pattern values
       const State &state2 = n2->GetState();
       int cost2 = heuristicOfRows(state2) + n2->GetDepth(); // Cost is based off distance of pattern values
       return cost1 < cost2;
    }
};

// ERIK ADDED ABOVE

// Madison add
class DiagonalCompare
{
public:
    bool operator()
            (const std::shared_ptr<Node> &n1,
             const std::shared_ptr<Node> &n2) const
    {
       const State &state1 = n1->GetState();
       int cost1 = GetDiagonalCost(state1) +
                   n1->GetDepth(); // Cost is based off distance from home and allows for diagonals
       const State &state2 = n2->GetState();
       int cost2 = GetDiagonalCost(state2) +
                   n2->GetDepth(); // Cost is based off distance from home and allows for diagonals

       return cost1 < cost2;

    }

};
//Madison add end

class Solver
{
public:
    enum Type
    {
        DEPTH_FIRST = 0,
        BREADTH_FIRST,
        GREEDY_BEST_FIRST,
        ASTAR_H1,
        ASTAR_H2,
        SMA,
        IDA,
        AStarSN, // ERIK ADDED
        patternDatabase, // ERIK ADDED
        rowsHeuristic, // ERIK ADDED
        Diagonal //Madison added
    };

    Solver(const State &start, const State &goal, Type type = Type::ASTAR_H2)
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

    inline int Search(std::shared_ptr<Node> node, int gScore, int threshold)
    {
       {
          std::shared_ptr<Node> node;
          const State &state1 = node->GetState();
          int gScore = node->GetDepth();
          int f = gScore + GetManhattanCost(state1);
          int FOUND = 0;
          int MAX_INT = 100;

          if (f > threshold)
          {
             return f;
          }
          if (node->GetState() == _goal)
          {
             return FOUND;
          }
          int min = MAX_INT;
          NodePtr current;
          current = _openlist[0];
          NodeList::iterator current_itr(_openlist.begin());
          if (current_itr == _openlist.end())
          {
             return 0;
          }
          else
          {
             int temp = Search(GetNextNode(), gScore, threshold);
             if (temp == FOUND)
             {
                return FOUND;
             }
             if (temp < min)
             {
                min = temp;
             }
          }
       }

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
          //ASTAR = F(n) = g(n) + h(n)
          // where g(n) is total cost to reach n along path or total cost from ground node to current node.
          // where h1(n) is the number of misplaced tiles
          //where h2(n) is sum of distances of tiles from their goal positions

          case ASTAR_H1:
          {
             NodeList::iterator current_itr(
                     std::min_element(_openlist.begin(), _openlist.end(), CompareFunctorForAStar_H1()));


             if (current_itr == _openlist.end())
             { return 0; }


             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);

             break;
          }
          case ASTAR_H2:
          {
             //min_element function returns an iterator to element with the smallest value in range [_openlist.begin(), openlist.end()) so does not include the element pointed by openlist.end()
             //comp() binary function that accepts two elements in the range as arguments, returns a value convertible to bool. value returned indicates whether the element passed as first argument is less than second argument
             NodeList::iterator current_itr(
                     std::min_element(_openlist.begin(), _openlist.end(), CompareFunctorForAStar_H2()));

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
             Neighbors g;
             std::shared_ptr<Node> node;
             const State &state1 = node->GetState();
             int gScore = node->GetDepth();
             int FOUND = 0;
             current = _openlist[0];
             int threshold = GetThreshold(current);

             std::cout << "Threshold: " << threshold << endl;

             NodeList::iterator current_itr(_openlist.begin());
             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             while (1)
             {
                int temp = Search(node, gScore, threshold);
                if (temp == FOUND)
                {
                   return 0; //return temp
                }
                if (temp == INFINITY)
                {
                   return 0; //or set time limit exceeded
                }
                threshold = temp;

                //node = GetNextNode();
                //ExpandNode(node, g);
             }

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(
                     current); //doesn't keep track of visited node and therefore explores already explored nodes again.

             break;
          }
          case DEPTH_FIRST:
          {
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

             // **** ERIK Added here

          case rowsHeuristic:
          {
             NodeList::iterator current_itr(std::min_element(
                     _openlist.begin(),
                     _openlist.end(),
                     CompareheuristicOfRows())); // Calling function for Heuristic Of Rows

             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);
             break;
          }

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
             // Madison Start
          case Diagonal:
          {
             NodeList::iterator current_itr(
                     std::min_element(_openlist.begin(), _openlist.end(), DiagonalCompare()));

             if (current_itr == _openlist.end())
             { return 0; }

             //copy the value first to a shared pointer and then erase from the open list.
             current = *current_itr;

             // now erase from the open list.
             _openlist.erase(current_itr);
             _closedlist.push_back(current);

             break;
          }
             // Madison End
       }
       _typePrint = _type;
       return current;
    }

    // expand the graph by looking into the neighbors for the given node.
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
             countNG++;
             static int s_lineNum = 1;
             n->print(std::cout, s_lineNum++);
             depth = current->GetDepth() + 1;
             //_closedlist.push_back(n);
          }
       }
       countNE = _closedlist.size();
    }

private:
    typedef std::vector<NodePtr> NodeList;
    NodeList _openlist;
    NodeList _closedlist;
    const State &_goal;
    bool _solved;
    Type _type;
public:
    int countNE = 0;
    int countNG = 0;
    int depth = 0;
    Type _typePrint;
};


#endif //PROJECT2_SOLVER_H
