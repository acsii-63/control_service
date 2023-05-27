#include <vector>
#include <string>

class Node
{
public:
    int index;           // Index of Node in Branching linked list
    std::string command; // Command in node
    Node *next;          // Next node if the command executed successfully
    Node *jump;          // Jump to this node if the command executed fails

    Node(int _index, const std::string &_command);
};

class BranchingLinkedList
{
public:
    BranchingLinkedList(const std::string &_command);

    // Add next node after current node.
    void addNextNode(Node *_currNode, Node *_nextNode);

    // Add jump node with given current node.
    void addJumpNode(Node *_currNode, Node *_jumpNode);

    // Get the node with given index.
    Node *getNode(int _index);

    // Add jump node to current node with given index of jump node.
    void addJumpNodeWithIndex(Node *_currNode, int _index);

private:
    Node *head;
};

/****************************************************/

Node::Node(int _index, const std::string &_command) : index(_index),
                                                      command(_command)
{
    next = nullptr;
    jump = nullptr;
}

/****************************************************/

BranchingLinkedList::BranchingLinkedList(const std::string &_command)
{
    head = new Node(0, _command);
}

void BranchingLinkedList::addNextNode(Node *_currNode, Node *_nextNode)
{
    _currNode->next = _nextNode;
}

void BranchingLinkedList::addJumpNode(Node *_currNode, Node *_jumpNode)
{
    _currNode->jump = _jumpNode;
}

Node *BranchingLinkedList::getNode(int _index)
{
    Node *currNode = head;
    while (currNode != nullptr)
    {
        if (currNode->index = _index)
        {
            return currNode;
        }
        currNode = currNode->next;
    }
    return nullptr;
}

void BranchingLinkedList::addJumpNodeWithIndex(Node *_currNode, int _index)
{
    Node *jumpNode = getNode(_index);
    if (jumpNode == nullptr)
        return;
    else
        addJumpNode(_currNode, jumpNode);
}
