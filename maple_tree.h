#include <pthread.h>
#include"spinlock.h"
#include <urcu.h>

// macro
#define MA_STATE(name, mt, first, end)					\
	struct ma_state name = {					\
		.tree = mt,						\
		.index = first,						\
		.last = end,						\
		.node = MAS_START,					\
		.min = 0,						\
		.max = ULONG_MAX,					\
		.alloc = NULL,						\
		.mas_flags = 0,						\
	}

//data structures 
struct maple_tree {
	union {
		// locks
		spinlock ma_lock;
	};
	void       *ma_root;
	unsigned int	ma_flags;

};

struct ma_state {
	struct maple_tree *tree;	/* The tree we're operating in */
	unsigned long index;		/* The index we're operating on - range start */
	unsigned long last;		/* The last index we're operating on - range end */
	struct maple_enode *node;	/* The node containing this entry */
	unsigned long min;		/* The minimum index of this node - implied pivot min */
	unsigned long max;		/* The maximum index of this node - implied pivot max */
	struct maple_alloc *alloc;	/* Allocated nodes for this operation */
	unsigned char depth;		/* depth of tree descent during write */
	unsigned char offset;
	unsigned char mas_flags;
};

/*
 * The Maple Tree squeezes various bits in at various points which aren't
 * necessarily obvious.  Usually, this is done by observing that pointers are
 * N-byte aligned and thus the bottom log_2(N) bits are available for use.  We
 * don't use the high bits of pointers to store additional information because
 * we don't know what bits are unused on any given architecture.
 *
 * Nodes are 256 bytes in size and are also aligned to 256 bytes, giving us 8
 * low bits for our own purposes.  Nodes are currently of 4 types:
 * 1. Single pointer (Range is 0-0)
 * 2. Non-leaf Allocation Range nodes
 * 3. Non-leaf Range nodes
 * 4. Leaf Range nodes All nodes consist of a number of node slots,
 *    pivots, and a parent pointer.
 */

struct maple_node {
	union {
		struct {
			struct maple_pnode *parent;
			void __rcu *slot[MAPLE_NODE_SLOTS];
		};
		struct {
			void *pad;
			struct rcu_head rcu;
			struct maple_enode *piv_parent;
			unsigned char parent_slot;
			enum maple_type type;
			unsigned char slot_len;
			unsigned int ma_flags;
		};
		struct maple_range_64 mr64;
		struct maple_arange_64 ma64;
		struct maple_alloc alloc;
	};
};


/*
Start by initialising a maple tree
DEFINE_MTREE() for statically allocated maple trees 
mt_init() for dynamically allocated ones. 
A freshly-initialised maple tree contains a ``NULL`` pointer for the range ``0`` - ``ULONG_MAX``.  

two types of maple trees supported: the allocation tree and the regular tree.
The regular tree has a higher branching factor for internal nodes.  
The allocation tree has a lower branching factor 
but allows the user to search for a gap of a given size or larger from either ``0`` upwards or ``ULONG_MAX`` down.
An allocation tree can be used by passing in the ``MT_FLAGS_ALLOC_RANGE`` flag when initialising the tree.

mtree_store() 
mtree_store_range() set entries
mtree_store() will overwrite any entry with the new entry and return 0 on success or an error code otherwise.  
mtree_store_range() works in the same way but takes a range.  
mtree_load() is used to retrieve the entry stored at a given index. 
mtree_erase() erase an entire range by only knowing one value within that range
mtree_store() call with an entry of NULL may be used to partially erase a range or many ranges at once.

mtree_insert_range() store a new entry to a range (or index) if that range is currently ``NULL``
mtree_insert() return -EEXIST if the range is not empty.

mt_find() search for an entry from an index upwards

mt_for_each() walk each entry within a range  
You must provide a temporary variable to store a cursor.  
If you want to walk each element of the tree then ``0`` and ``ULONG_MAX`` may be used as the range.  
If the caller is going to hold the lock for the duration of the walk 
then it is worth looking at the mas_for_each() API in the :ref:`maple-tree-advanced-api` section.

Sometimes it is necessary to ensure the next call to store to a maple tree does
not allocate memory, please see :ref:`maple-tree-advanced-api` for this use case.

mtree_destroy() remove all entries from a maple tree by calling
If the maple tree entries are pointers, free the entries first.
*/

// Normal API
#define DEFINE_MTREE(name)						\
	struct maple_tree name = MTREE_INIT(name, 0)

#define MTREE_INIT(name, __flags) {					\
	.ma_lock = SPINLOCK_INITIALIZER,		\
	.ma_flags = __flags,						\
	.ma_root = NULL,						\
}
static inline void mt_init_flags(struct maple_tree *mt, unsigned int flags);
static inline void mt_init(struct maple_tree *mt);




// Takes RCU read lock:
//  * mtree_load()
//  * mt_find()
//  * mt_for_each()
//  * mt_next()
//  * mt_prev()

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
