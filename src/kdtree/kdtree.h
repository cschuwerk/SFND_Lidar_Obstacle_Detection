#ifndef PLAYBACK_KDTREE_H
#define PLAYBACK_KDTREE_H
#include <utility>
#include <vector>
#include <iostream>
#include <cmath>

namespace kdTree {

    // Structure to represent node of kd tree
    struct node
    {
        std::vector<float> point;
        int id;
        node* left;
        node* right;

        node(std::vector<float> arr, int setId)
                :	point(std::move(arr)), id(setId), left(nullptr), right(nullptr)
        {}
    };

    struct tree
    {
        node* root;

        tree()
                : root(nullptr)
        {}



        void insert(std::vector<float> point, int id)
        {
            // the function should create a new node and place correctly with in the root
            struct node* n = new node(std::move(point),id);
            if(root == nullptr) {
                root = n;
            }
            else {
                insertNode(root, n, 0);
            }

        }



        static void insertNode(node* &currNode, node* &newNode, unsigned int depth) {
            if(currNode == nullptr) {
                currNode = newNode;
            }
            else {
                unsigned int i = depth%3; // 0 for x values, 1 for y values, 2 for z values
                if(newNode->point[i] < currNode->point[i]) {
                    insertNode(currNode->left, newNode,++i);
                }
                else {
                    insertNode(currNode->right, newNode,++i);
                }
            }
        }



        // return a list of point ids in the tree that are within distance of target
        std::vector<int> search(std::vector<float> target, float distanceTol)
        {
            std::vector<int> ids;

            searchPoints(std::move(target), distanceTol, root, 0, ids);

            return ids;
        }


        static void searchPoints(std::vector<float> target, float distanceTol, node *node, unsigned int depth,
                          std::vector<int> &ids)
        {
            if(node == nullptr) return;

            // check if this point is within the boundary
            if(node->point[0] <= node->point[0]+distanceTol && node->point[0] >= node->point[0]-distanceTol) {
                if(node->point[1] <= node->point[1]+distanceTol && node->point[1] >= node->point[1]-distanceTol) {
                    if(node->point[2] <= node->point[2]+distanceTol && node->point[2] >= node->point[2]-distanceTol) {
                        if (sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                 (node->point[1] - target[1]) * (node->point[1] - target[1]) +
                                 (node->point[2] - target[2]) * (node->point[2] - target[2])) < distanceTol) {
                            ids.push_back(node->id);
                        }
                    }
                }
            }

            // Check the childs only of their x/y/z component is within the distance tolerance
            if(target[depth%3] - distanceTol <= node->point[depth%3]) {
                searchPoints(target, distanceTol, node->left, depth + 1, ids);
            }
            if(target[depth%3] + distanceTol >= node->point[depth%3]) {
                searchPoints(target, distanceTol, node->right, depth + 1, ids);
            }


        }

    };
}

#endif /*PLAYBACK_KDTREE_H*/