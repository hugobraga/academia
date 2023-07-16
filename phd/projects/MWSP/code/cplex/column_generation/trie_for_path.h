#ifndef TRIE_FOR_PATH_H
#define TRIE_FOR_PATH_H

#include <map>
#include <list>
#include "spanner.h"
#include "trie_for_path.h"

using namespace std;

struct TrieNode;

/* struct cmpByOurEdgeLabel { */
/*     bool operator()(const OurEdge& a, const OurEdge& b) const {       */
/*         return a.label < b.label; */
/*     } */
/* }; */

typedef std::map<OurEdge*, TrieNode*/*, cmpByOurEdgeLabel*/> EdgeTrieMap;

// trie node
struct TrieNode
{
  //vector<struct TrieNode*> children;
  EdgeTrieMap children;
  //list<OurEdge*> children;
  
  
  //struct TrieNode *children[ALPHABET_SIZE];
 
    // isLeaf is true if the node represents
    // end of a word
    bool isLeaf;
  TrieNode() : isLeaf(false) {
    cout << "inicializou" << endl;
    children = EdgeTrieMap();
    // children.reserve(input.nEdges);

    // for (i = 0; i < input.nEdges; i++)
    //   //for (i = 0; i < ALPHABET_SIZE; i++)
    //   children[i] = NULL;
    
  }
};

TrieNode *getTrieNode(void);
void insertTrie(struct TrieNode *root, SpanPath* p);
bool searchTrie(TrieNode *root, SpanPath* p);

#endif
