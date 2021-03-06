//
// Created by Shervone Mayes on 4/6/2020.
//

#ifndef PROJECT2_STATE_H
#define PROJECT2_STATE_H

#include <vector>
#include <random>
#include <map>
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace std;

//type definitions
typedef std::vector<int> IntArray;

//State class represents a unique combination of tiles
class State
{
private:
    IntArray _state; //represent tiles on puzzle; unique combination of tiles
    unsigned int _rows_or_cols; //defines puzzle dimensions (e.g. want 3x3 so value is 3)

public:
    //constructor: default accepts value to define puzzle dimension
    explicit State(unsigned int rows_or_cols) : _rows_or_cols(_rows_or_cols)
    {
       _state.resize(_rows_or_cols * rows_or_cols);
       for (unsigned int i = 0; i < _state.size(); i++)
       {
          _state[i] = i;
       }
    }

    //constructor: that takes in the num_rows_or_cols and an initial state of the array
    State(unsigned int rows_or_cols, const IntArray &arr) : _rows_or_cols(rows_or_cols)
    {
       assert(arr.size() == _rows_or_cols * rows_or_cols);
       _state = arr;
    }

    //constructor: copy
    State(const State &other)
    {
       _rows_or_cols = other._rows_or_cols;
       _state = other._state;
    }

    //assignment operator
    State &operator=(const State &other)
    {
       if (this != &other)
       {
          _rows_or_cols = other._rows_or_cols;
          _state = other._state;
       }
       return *this;
    }

    //equal to operator
    friend bool operator==(const State &a, const State &b)
    {
       return (a._state == b._state);
    }

    //not equal to operator
    friend bool operator!=(const State &a, const State &b)
    {
       return (a._state != b._state);
    }

    //determine the index of blank tile. iterate until determine index of empty tile
    inline int FindEmptyTileIndex() const
    {
       for (int i = 0; i < _state.size(); i++)
       {
          if (_state[i] == 0)
          { return i; }
       }
       return (int) _state.size();
    }

    //slide empty tile
    inline void SwapWithEmpty(int i0, int i1)
    {
       int tmp = _state[i1];
       _state[i1] = _state[i0];
       _state[i0] = tmp;
    }

    //randomize state of puzzle
    inline void Randomize()
    {
       random_shuffle(_state.begin(), _state.end());
    }

    //get array state
    inline const IntArray &GetArray() const
    {
       return _state;
    }

    //set array state
    void SetArray(const IntArray &arr)
    {
       _state = arr;;
    }


    inline unsigned int GetNumRowsOrCols() const
    {
       return _rows_or_cols;
    }

    //print state
    void print(ostream &str, bool flat = false) const
    {
       for (unsigned int i = 0; i < _rows_or_cols; ++i)
       {
          for (unsigned int j = 0; j < _rows_or_cols; ++j)
          {
             unsigned int index = i * _rows_or_cols + j;
             if (flat)
             {
                str << _state[index];
             }
             else
             {
                str << _state[index] << " ";
             }
          }
          if (!flat)
          {
             str << "\n";
          }
       }
       str << "\n";
    }

};


#endif //PROJECT2_STATE_H
