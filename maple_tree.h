#include <pthread.h>
#include"spinlock.h"

// Takes RCU read lock:
//  * mtree_load()
//  * mt_find()
//  * mt_for_each()
//  * mt_next()
//  * mt_prev()

struct maple_tree {};
void *mtree_load(struct maple_tree *mt, unsigned long index);
void *mt_find(struct maple_tree *mt, unsigned long *index, unsigned long max);
#define mt_for_each(__tree, __entry, __index, __max) \
	for (__entry = mt_find(__tree, &(__index), __max); \
		__entry; __entry = mt_find_after(__tree, &(__index), __max))
void *mt_find_after(struct maple_tree *mt, unsigned long *index, unsigned long max);
void *mt_next(struct maple_tree *mt, unsigned long index, unsigned long max);
void *mt_prev(struct maple_tree *mt, unsigned long index, unsigned long min);	

// Takes ma_lock internally:
//  * mtree_store()
//  * mtree_store_range()
//  * mtree_insert()
//  * mtree_insert_range()
//  * mtree_erase()
//  * mtree_destroy()
//  * mt_set_in_rcu()
//  * mt_clear_in_rcu()

int mtree_store_range(struct maple_tree *mt, unsigned long first, unsigned long last, void *entry);
int mtree_store(struct maple_tree *mt, unsigned long index,	void *entry);
int mtree_insert(struct maple_tree *mt, unsigned long index, void *entry);
int mtree_insert_range(struct maple_tree *mt, unsigned long first, unsigned long last, void *entry);
void *mtree_erase(struct maple_tree *mt, unsigned long index);
void mtree_destroy(struct maple_tree *mt);
static inline void mt_set_in_rcu(struct maple_tree *mt){}
static inline void mt_clear_in_rcu(struct maple_tree *mt){}
