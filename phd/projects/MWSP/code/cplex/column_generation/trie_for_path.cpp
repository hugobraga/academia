// C implementation of search and insert operations
// on Trie
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//#include "spanner.h"
#include "trie_for_path.h" 
 
// Returns new trie node (initialized to NULLs)
TrieNode *getTrieNode(void)
{
    TrieNode* pNode = NULL;
    pNode = new TrieNode;
    pNode->children = EdgeTrieMap();
    pNode->isLeaf = false;
    return pNode;
}
 
// If not present, inserts key into trie
// If the key is prefix of trie node, just marks leaf node
void insertTrie(struct TrieNode *root, SpanPath* p)
{
  cout << "dentro de insertTrie, root: " << root << endl;
    TrieNode *pCrawl = root;

    FOR_EACH_EDGE_e_IN_PATH_p
      EdgeTrieMap::iterator it = pCrawl->children.find(e);
      cout << "procurando pela aresta " << e->u << "-" << e->v << endl;
      if (it == pCrawl->children.end()) {
	cout << "nao encontrou aresta de path na trie" << endl;
	pCrawl->children[e] = getTrieNode();	
	pCrawl = pCrawl->children[e];
      } else {
	cout << "achou a aresta: " << it->first->u << "-" << it->first->v << endl;
	pCrawl = it->second;
      }      
   }
  
    // mark last node as leaf
    pCrawl->isLeaf = true;
}
 
// Returns true if path presents in trie, else false
bool searchTrie(TrieNode *root, SpanPath* p)
{
  cout << "dentro de searchTrie, root: " << root << endl;
    TrieNode *pCrawl = root;

    FOR_EACH_EDGE_e_IN_PATH_p
      cout << "e: " << e->u << "-" << e->v << endl;
      EdgeTrieMap::iterator it = pCrawl->children.find(e);
      cout << "iterador armazena referencia da aresta" << endl;
      if (it == pCrawl->children.end()) {
	cout << "nao achou o caminho" << endl;
	return false;
      } else {
	cout << "ate agora o caminho está batendo" << endl;
      }
      pCrawl = it->second;;
    }

if (pCrawl == NULL)
  cout << "pCrawl == NULL" << endl;
 else
   cout << "pCrawl != NULL" << endl;
if (pCrawl->isLeaf)
  cout << "pCrawl eh folha" << endl;
 else
   cout << "pCrawl não eh folha" << endl;
    return (pCrawl != NULL && pCrawl->isLeaf);
}
 
// // Driver
// int main()
// {
//     // Input keys (use only 'a' through 'z' and lower case)
//     char keys[][8] = {"the", "a", "there", "answer", "any",
//                      "by", "bye", "their"};
 
//     char output[][32] = {"Not present in trie", "Present in trie"};
 
 
//     struct TrieNode *root = getNode();
 
//     // Construct trie
//     int i;
//     for (i = 0; i < ARRAY_SIZE(keys); i++)
//         insert(root, keys[i]);
 
//     // Search for different keys
//     printf("%s --- %s\n", "the", output[search(root, "the")] );
//     printf("%s --- %s\n", "these", output[search(root, "these")] );
//     printf("%s --- %s\n", "their", output[search(root, "their")] );
//     printf("%s --- %s\n", "thaw", output[search(root, "thaw")] );
 
//     return 0;
// }
