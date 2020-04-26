#include <iostream>
#include <vector>
#include <ctime>
#include <fstream>
#include "State.h"
#include "Neighbors.h"
#include "Node.h"
#include "Solver.h"
#include <chrono>

using namespace std::chrono;

void memory_process(double &vm)
{
   vm = 0.0;

   unsigned long vsize;
   {
      std::string ignore;
      std::ifstream ifs("/proc/self/stat", std::ios_base::in);
      ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
          >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
          >> ignore >> ignore >> vsize;
   }
   vm = vsize / 1024.0;
}

int main(int argc, char *argv[])
{
   double vm;
   Neighbors g;
   auto startClock = high_resolution_clock::now();

   State goal(3, std::vector<int>{1, 2, 3, 8, 0, 4, 7, 6, 5});
//  State start(3, std::vector<int>{2, 8, 3 ,1, 6, 4, 0, 7, 5});
   State start(3, std::vector<int>{2, 1, 6, 4, 0, 8, 7, 5, 3});

//   State goal(3, std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 0});
//   State start(3, std::vector<int>{1, 2, 3, 0, 4, 6, 7, 5, 8});

//********CHANGE ME TO RUN DIFFERENT SOLVER ALGORITHM*******//
   std::shared_ptr<Node> node;
   Solver solver(start, goal, Solver::rowsHeuristic);
   if (!solver.isSolvable())
   {
      std::cout << "Puzzle state is unsolvable..!\n";
      return 0;
   }
   while (!solver.isSolved())
   {
         node = solver.GetNextNode();
         solver.ExpandNode(node, g);
   }

   // accumulate the nodes for the solution.
   std::vector<NodePtr> solution;
   NodePtr s = node;
   do
   {
      solution.push_back(s);
      s = s->GetParent();
   } while (s != NULL);

   // Get duration. Substart timepoints to get duration. To cast it to proper unit
   // use duration cast method
   auto stopClock = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stopClock - startClock);
   // print the solution.
   std::cout << "The puzzle can be solved in " << solution.size() - 1 << " steps. Solution below\n";

   std::cout << "PERFORMANCE DATA: " << endl;

   std::cout << "Execution Time (ET): " << duration.count() << " microseconds\n";
   std::cout << "Number of Nodes Generated (NG):  " << solver.countNG << endl;
   std::cout << "Number of Nodes Expanded (NE):  " << solver.countNE << endl;
   std::cout << "Depth of Tree (d):  " << solver.depth << endl;
   std::cout << "Effective Branching Factor b* (b*):  " << solver.countNG / solver.depth << endl;
   memory_process(vm);
   std::cout << "Memory used " << vm << " \n";

   for (int i = (int) solution.size() - 1; i >= 0; i--)
   {
      solution[i]->GetState().print(std::cout, false);
   }
   std::cout << "\n";

   for (int i = (int) solution.size() - 1; i >= 0; i--)
   {
      solution[i]->GetParent()->print(std::cout, false);
   }
   std::cout << "\n";

   return 0;
}
