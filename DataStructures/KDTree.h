/**
 * File: KDTree.h
 * Author: Yunlong Nick Liu
 * ------------------------
 * An interface representing a kd-tree in some number of dimensions. The tree
 * can be constructed from a set of data and then queried for membership and
 * nearest neighbors.
 */

#ifndef KDTREE_INCLUDED
#define KDTREE_INCLUDED

#include "Point.h"
#include "BoundedPQueue.h"
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <map>
#include <utility>

template <size_t N, typename ElemType>
class KDTree {
public:
    // Constructor: KDTree();
    // Usage: KDTree<3, int> myTree;
    // ----------------------------------------------------
    // Constructs an empty KDTree.
    KDTree();

	// Constructor: range based constructor
	//KDTree(std::pair<Point<N>, ElemType> *begin, std::pair<Point<N>, ElemType> *end);
    
    // Destructor: ~KDTree()
    // Usage: (implicit)
    // ----------------------------------------------------
    // Cleans up all resources used by the KDTree.
    ~KDTree();
    
    // KDTree(const KDTree& rhs);
    // KDTree& operator=(const KDTree& rhs);
    // Usage: KDTree<3, int> one = two;
    // Usage: one = two;
    // -----------------------------------------------------
    // Deep-copies the contents of another KDTree into this one.
    KDTree(const KDTree& rhs);
    KDTree& operator=(const KDTree& rhs);
    
    // size_t dimension() const;
    // Usage: size_t dim = kd.dimension();
    // ----------------------------------------------------
    // Returns the dimension of the points stored in this KDTree.
    size_t dimension() const;
    
    // size_t size() const;
    // bool empty() const;
    // Usage: if (kd.empty())
    // ----------------------------------------------------
    // Returns the number of elements in the kd-tree and whether the tree is
    // empty.
    size_t size() const;
    bool empty() const;
    
    // bool contains(const Point<N>& pt) const;
    // Usage: if (kd.contains(pt))
    // ----------------------------------------------------
    // Returns whether the specified point is contained in the KDTree.
    bool contains(const Point<N>& pt) const;
    
    // void insert(const Point<N>& pt, const ElemType& value);
    // Usage: kd.insert(v, "This value is associated with v.");
    // ----------------------------------------------------
    // Inserts the point pt into the KDTree, associating it with the specified
    // value. If the element already existed in the tree, the new value will
    // overwrite the existing one.
    void insert(const Point<N>& pt, const ElemType& value);
    
    // ElemType& operator[](const Point<N>& pt);
    // Usage: kd[v] = "Some Value";
    // ----------------------------------------------------
    // Returns a reference to the value associated with point pt in the KDTree.
    // If the point does not exist, then it is added to the KDTree using the
    // default value of ElemType as its key.
    ElemType& operator[](const Point<N>& pt);
    
    // ElemType& at(const Point<N>& pt);
    // const ElemType& at(const Point<N>& pt) const;
    // Usage: cout << kd.at(v) << endl;
    // ----------------------------------------------------
    // Returns a reference to the key associated with the point pt. If the point
    // is not in the tree, this function throws an out_of_range exception.
    ElemType& at(const Point<N>& pt);
    const ElemType& at(const Point<N>& pt) const;
    
    // ElemType kNNValue(const Point<N>& key, size_t k) const
    // Usage: cout << kd.kNNValue(v, 3) << endl;
    // ----------------------------------------------------
    // Given a point v and an integer k, finds the k points in the KDTree
    // nearest to v and returns the most common value associated with those
    // points. In the event of a tie, one of the most frequent value will be
    // chosen.
    ElemType kNNValue(const Point<N>& key, size_t k) const;
    
private:
    
    struct TreeNode {
        TreeNode *left;
        TreeNode *right;
        Point<N> key;
        ElemType object;
        
        TreeNode(const Point<N>& k = Point<N>(), const ElemType& obj = ElemType())
            : left(nullptr), right(nullptr), key(k), object(obj) {}
        
        ~TreeNode() {
            delete left;
            delete right;
        }
    };
    
    TreeNode *root;
    size_t treeSize;
    
    void kNNValueHelper(TreeNode *n, int level, const Point<N>& pt, BoundedPQueue<ElemType> &bpq) const;
    
    void treeCopy(TreeNode *thisNode, TreeNode *otherNode);
    
    TreeNode *findNode(const Point<N>& pt) const;
};

/** KDTree class implementation details */

template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree() : root(nullptr), treeSize(0) {}

template <size_t N, typename ElemType>
KDTree<N, ElemType>::~KDTree() {
    delete root;
    root = nullptr;
}

template <size_t N, typename ElemType>
size_t KDTree<N, ElemType>::dimension() const {
    return N;
}

/*
template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(std::pair<Point<N>, ElemType> *begin, std::pair<Point<N>, ElemType> *end) {
	node* root = new node(nullptr, nullptr, elems[length / 2]);
	std::stack<std::tuple<node*, int, int, bool>> iter;
	iter.emplace(root, 0, length, false);

	node *current;
	int start, end;
	bool isR = false;
	while (!iter.empty()) {
		std::tie(current, start, end, isR) = iter.top();
		iter.pop();
		switch (end - start) {
		case 0:
			continue;
		case 1:
			if (elems[start] < current->key) {
				node *left = new node(nullptr, nullptr, elems[start]);
				current->left = isR ? makeLeftRed(left) : left;
			}
			else {
				current->right = new node(nullptr, nullptr, elems[start]);
			}
			break;
		default:
			node *left = new node(nullptr, nullptr, elems[(end - start) / 4 + start]);
			current->left = isR ? makeLeftRed(left) : left;
			current->right = new node(nullptr, nullptr, elems[(end - start) * 3 / 4 + start]);
			iter.emplace(current->left, start, (end - start) / 2 + start, !isR);
			iter.emplace(current->right, (end - start) / 2 + start + 1, end, !isR);
		}
	}
	return root;
}
*/

template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(const KDTree& rhs) {
    treeSize = rhs.treeSize;
    root = new TreeNode();
    treeCopy(root, rhs.root);
}

template <size_t N, typename ElemType>
KDTree<N, ElemType>& KDTree<N, ElemType>::operator=(const KDTree& rhs) {
    if (this != &rhs) {
        treeSize = rhs.treeSize;
        root = new TreeNode();
        treeCopy(root, rhs.root);
    }
    return *this;
}

template <size_t N, typename ElemType>
size_t KDTree<N, ElemType>::size() const {
    return treeSize;
}

template <size_t N, typename ElemType>
bool KDTree<N, ElemType>::empty() const {
    return treeSize == 0;
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::insert(const Point<N>& pt, const ElemType& value) {
    TreeNode **ndPtr = &root;
    int level = N-1;
    while (*ndPtr && ++level) {
        if ((*ndPtr)->key == pt) {
            (*ndPtr)->object = value;
            return;
        }
        ndPtr = pt[level%N] < (*ndPtr)->key[level%N]
                ? &(*ndPtr)->left : &(*ndPtr)->right;
    }
    *ndPtr = new TreeNode(pt, value);
    treeSize++;
}

template <size_t N, typename ElemType>
bool KDTree<N, ElemType>::contains(const Point<N>& pt) const {
    return findNode(pt) != nullptr;
}

template <size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::operator[] (const Point<N>& pt) {
    TreeNode **ndPtr = &root;
    int level = N-1;
    while (*ndPtr && ++level) {
        if ((*ndPtr)->key == pt) {
            return (*ndPtr)->object;
        }
        ndPtr = pt[level%N] < (*ndPtr)->key[level%N]
        ? &(*ndPtr)->left : &(*ndPtr)->right;
    }
    *ndPtr = new TreeNode(pt, ElemType());
    treeSize++;
    return (*ndPtr)->object;
}

template <size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) {
    return const_cast<ElemType&>(static_cast<const KDTree&>(*this).at(pt));
}

template <size_t N, typename ElemType>
const ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) const {
    TreeNode *n = findNode(pt);
    if (!n) {
        throw out_of_range("The point is out of range");
    }
    return n->object;
}


template <size_t N, typename ElemType>
typename KDTree<N, ElemType>::TreeNode*
KDTree<N, ElemType>::findNode(const Point<N>& pt) const {
    int level = N-1;
    TreeNode *n = root;
    while (n && n->key != pt && ++level) {
        n = pt[level%N] < n->key[level%N] ? n->left : n->right;
    }
    return n;
}

template <size_t N, typename ElemType>
ElemType KDTree<N, ElemType>::kNNValue(const Point<N>& pt, size_t k) const {
    BoundedPQueue<ElemType> bpq(k);
    kNNValueHelper(root, 0, pt, bpq);
    std::map<ElemType, int> freqMap;
    while (!bpq.empty()) {
        freqMap[bpq.dequeueMin()]++;
    }
    int max = 0;
    ElemType frequent = ElemType();
    for (const std::pair<ElemType, int> &p : freqMap) {
        if (p.second > max) {
            max = p.second;
            frequent = p.first;
        }
    }
    return frequent;
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::kNNValueHelper(TreeNode *cur, int level, const Point<N>& pt,
                                         BoundedPQueue<ElemType> &bpq) const {
    if (!cur)
        return;
    bpq.enqueue(cur->object, Distance(cur->key, pt));
    TreeNode *next = pt[level%N] < cur->key[level%N] ? cur->left : cur->right;
    kNNValueHelper(next, level + 1, pt, bpq);
    TreeNode *other = pt[level%N] < cur->key[level%N] ? cur->right : cur->left;
    if (bpq.size() < bpq.maxSize() || std::fabs(pt[level%N] - cur->key[level%N]) < bpq.worst())
        kNNValueHelper(other, level + 1, pt, bpq);
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::treeCopy(TreeNode *thisNode, TreeNode *otherNode) {
    
    thisNode->object = otherNode->object;
    thisNode->key = otherNode->key;
    if (otherNode->left) {
        if (!thisNode->left)
            thisNode->left = new TreeNode();
        treeCopy(thisNode->left, otherNode->left);
    } else {
        if (thisNode->left) {
            delete thisNode->left;
            thisNode->left = nullptr;
        }
    }
    if (otherNode->right) {
        if (!thisNode->right)
            thisNode->right = new TreeNode();
        treeCopy(thisNode->right, otherNode->right);
    } else {
        if (thisNode->right) {
            delete thisNode->right;
            thisNode->left = nullptr;
        }
    }
}

#endif // KDTREE_INCLUDED
