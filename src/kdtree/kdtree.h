#ifndef KDTREE_H_
#define KDTREE_H_
#include <vector>
#include <iostream>
#include <cmath>

namespace kdTree {

    // Structure to represent node of kd tree
    struct Node
    {
        std::vector<float> point;
        int id;
        Node* left;
        Node* right;

        Node(std::vector<float> arr, int setId)
                :	point(arr), id(setId), left(NULL), right(NULL)
        {}
    };

    struct tree
    {
        Node* root;

        tree()
                : root(NULL)
        {}



        void insert(std::vector<float> point, int id)
        {
            // the function should create a new node and place correctly with in the root
            Node* n = new Node(point,id);
            if(root == NULL) {
                root = n;
            }
            else {
                insertNode(root, n, 0);
            }

        }



        void insertNode(Node* &node, Node* &newNode, unsigned int depth) {
            if(node == nullptr) {
                node = newNode;
            }
            else {
                unsigned int i = depth%2; // 0 for x values, 1 for y values
                if(newNode->point[i] < node->point[i]) {
                    insertNode(node->left, newNode,++i);
                }
                else {
                    insertNode(node->right, newNode,++i);
                }
            }
        }



        // return a list of point ids in the tree that are within distance of target
        std::vector<int> search(std::vector<float> target, float distanceTol)
        {
            std::vector<int> ids;

            searchPoints(target, distanceTol, root, 0, ids);

            return ids;
        }




        void searchPoints(std::vector<float> target, float distanceTol, Node *node, unsigned int depth,
                          std::vector<int> &ids)
        {
            if(node == nullptr) return;

            // check if this point is within the boundary
            if(node->point[0] <= node->point[0]+distanceTol && node->point[0] >= node->point[0]-distanceTol) {
                if(node->point[1] <= node->point[1]+distanceTol && node->point[1] >= node->point[1]-distanceTol) {
                    if(sqrt( (node->point[0]-target[0])*(node->point[0]-target[0]) +  (node->point[1]-target[1])*(node->point[1]-target[1]) ) < distanceTol) {
                        ids.push_back(node->id);
                    }
                }
            }

            // Check the childs only of their x/y component is within the distance tolerance
            if(target[depth%2] - distanceTol <= node->point[depth%2]) {
                searchPoints(target, distanceTol, node->left, depth + 1, ids);
            }
            if(target[depth%2] + distanceTol >= node->point[depth%2]) {
                searchPoints(target, distanceTol, node->right, depth + 1, ids);
            }


        }

    };
}

#endif /* KDTREE_H_ */