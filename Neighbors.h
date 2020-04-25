//
// Created by WYNI on 4/6/2020.
//

#ifndef PROJECT2_NEIGHBORS_H
#define PROJECT2_NEIGHBORS_H
using namespace std;

#include <vector>
#include <random>
#include <map>
#include <iostream>
#include <cassert>
#include <algorithm>

// define neighbors based on where empty tile can move
class Neighbors
{
public:
    typedef map<int, vector<int> > IndexNeighborMap;
    IndexNeighborMap _edges;

    //constructor
    Neighbors()
    {
       CreateGraphFor8Puzzle();
    }

    //returns vector of index of neighboring tiles where empty tile can move; input parameter is index to empty tile
    const vector<int> &GetNeighbors(int id) const
    {
       IndexNeighborMap::const_iterator itr(_edges.find(id));
       if (itr != _edges.end())
       {
          return itr->second;
       }
       static vector<int> s;
       return s;
    }

private:
    //neighbors of empty tile stored in map (stores collection of pairs (key,value) also known as a dictionary whose keys are unique.
    //recall two vertices are adjacent if there is an edge between them so insert edge(s) for neighboring tiles
    //make_pair() It constructs a pair object with its first element set to x and its second element set to y which is vector of neighboring tiles here.
    void CreateGraphFor8Puzzle()
    {
       _edges.insert(make_pair(0, vector < int > {1, 3}));
       _edges.insert(make_pair(1, vector < int > {0, 2, 4}));
       _edges.insert(make_pair(2, vector < int > {1, 5}));
       _edges.insert(make_pair(3, vector < int > {4, 0, 6}));
       _edges.insert(make_pair(4, vector < int > {3, 5, 1, 7}));
       _edges.insert(make_pair(5, vector < int > {4, 2, 8}));
       _edges.insert(make_pair(6, vector < int > {7, 3}));
       _edges.insert(make_pair(7, vector < int > {6, 8, 4}));
       _edges.insert(make_pair(8, vector < int > {7, 5}));
    }
};

#endif //PROJECT2_NEIGHBORS_H
