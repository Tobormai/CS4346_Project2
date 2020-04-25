#include <iostream>
#include <vector>
#include "State.h"
#include "Neighbors.h"
#include "Node.h"
#include "Solver.h"


int main(int argc, char *argv[])
{

   int timer = (clock() * 1000) / CLOCKS_PER_SEC;
   int runtime = 0;


   Neighbors g;

//   State goal(3, std::vector<int>{1, 2, 3, 8, 0, 4, 7, 6, 5});
//   State start(3, std::vector<int>{2, 1, 6, 4, 0, 8, 7, 5, 3});

   State goal(3, std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 0});
   State start(3, std::vector<int>{1, 2, 3, 0, 4, 6, 7, 5, 8});

//********CHANGE ME TO RUN DIFFERENT SOLVER ALGORITHM*******//
   std::shared_ptr<Node> node;
   Solver solver(start, goal, Solver::IDA);
   if (!solver.isSolvable())
   {
      std::cout << "Puzzle state is unsolvable..!\n";
      return 0;
   }
   int count = 0;
   while (!solver.isSolved())
   {
      runtime = ((clock() * 1000) / CLOCKS_PER_SEC) - timer;
      node = solver.GetNextNode();
      solver.ExpandNode(node, g);
      count++;
   }

   // accumulate the nodes for the solution.
   std::vector<NodePtr> solution;
   NodePtr s = node;
   do
   {
      solution.push_back(s);
      s = s->GetParent();
   } while (s != NULL);

   // print the solution.
   std::cout << "The puzzle can be solved in " << solution.size() - 1 << " steps. Solution below\n";
   std::cout << "The puzzle can be solved in " << runtime << " milliseconds\n";
   for (int i = (int) solution.size() - 1; i >= 0; i--)
   {
      solution[i]->GetState().print(std::cout, false);
   }
   std::cout << "\n";

   return 0;
}
