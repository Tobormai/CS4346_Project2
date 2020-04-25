//
// Created by WYNI on 4/10/2020.
//

#ifndef PROJECT2_NODE_H
#define PROJECT2_NODE_H


#include <memory>
#include "State.h"

//represent tree elements

class Node
{
private:
    State _state;
    std::shared_ptr<Node> _parent;
    int _depth;

public:
    Node(const State &state, std::shared_ptr<Node> parent, int depth = 0)
            : _state(state), _depth(depth)
    {
       _parent = parent;
    }

    void SetParent(Node *node)
    {
       _parent.reset(node);
    }

    void SetParent(std::shared_ptr<Node> node)
    {
       _parent = node;
    }

    std::shared_ptr<Node> GetParent()
    {
       return _parent;
    }

    const std::shared_ptr<Node> GetParent() const
    {
       return _parent;
    }

    const State &GetState() const
    {
       return _state;
    }

    int GetDepth() const
    {
       return _depth;
    }

    void print(std::ostream &out, int lineNum) const
    {
       out << lineNum << " - Node { ";
       for (unsigned int i = 0; i < _state.GetArray().size(); ++i)
       {
          out << _state.GetArray()[i];
       }
       out << " | Depth: " << _depth;
       out << " }" << "\n";
    }
};

typedef std::shared_ptr<Node> NodePtr;


#endif //PROJECT2_NODE_H
