#include <pthread.h>
#include "spinlock.h"
#include <urcu.h>
#include <limits.h>
#include <errno.h>
#include <string.h>

// macro

/**
 * DOC: Maple tree flags
 *
 * * MT_FLAGS_ALLOC_RANGE	- Track gaps in this tree
 * * MT_FLAGS_USE_RCU		- Operate in RCU mode
 * * MT_FLAGS_HEIGHT_OFFSET	- The position of the tree height in the flags
 * * MT_FLAGS_HEIGHT_MASK	- The mask for the maple tree height value
 * * MT_FLAGS_LOCK_MASK		- How the mt_lock is used
 * * MT_FLAGS_LOCK_IRQ		- Acquired irq-safe
 * * MT_FLAGS_LOCK_BH		- Acquired bh-safe
 * * MT_FLAGS_LOCK_EXTERN	- mt_lock is not used
 *
 * MAPLE_HEIGHT_MAX	The largest height that can be stored
 */
#define MT_FLAGS_ALLOC_RANGE	0x01
#define MT_FLAGS_USE_RCU	0x02
#define MT_FLAGS_HEIGHT_OFFSET	0x02
#define MT_FLAGS_HEIGHT_MASK	0x7C
#define MT_FLAGS_LOCK_MASK	0x300
#define MT_FLAGS_LOCK_IRQ	0x100
#define MT_FLAGS_LOCK_BH	0x200
#define MT_FLAGS_LOCK_EXTERN	0x300

#define MAPLE_HEIGHT_MAX	31


#define MAPLE_NODE_TYPE_MASK	0x0F
#define MAPLE_NODE_TYPE_SHIFT	0x03

#define MAPLE_RESERVED_RANGE	4096

#define MAPLE_NODE_MASK		255UL


#ifdef CONFIG_LOCKDEP
typedef struct lockdep_map *lockdep_map_p;
#define mt_lock_is_held(mt)	lock_is_held(mt->ma_external_lock)
#define mt_set_external_lock(mt, lock)					\
	(mt)->ma_external_lock = &(lock)->dep_map
#else
typedef struct { /* nothing */ } lockdep_map_p;
#define mt_lock_is_held(mt)	1
#define mt_set_external_lock(mt, lock)	do { } while (0)
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#define MAS_START ((struct maple_enode *)1UL) // we have not searched the tree
#define MAS_ROOT             \
	((struct maple_enode \
		  *)5UL) // we have searched the tree and the entry we found lives in the root of the tree
#define MAS_NONE             \
	((struct maple_enode \
		  *)9UL) // we have searched the tree and there is no node in the tree for this entry
#define MAS_PAUSE ((struct maple_enode *)17UL) //
#define MAS_OVERFLOW ((struct maple_enode *)33UL)
#define MAS_UNDERFLOW ((struct maple_enode *)65UL)
// MA_ERROR represents an errno.
// After dropping the lock and attempting to resolve the error,
// the walk would have to be restarted from the top of the tree as the tree may have been modified.
#define MA_ERROR(err) ((struct maple_enode *)(((unsigned long)err << 2) | 2UL))
#define mtree_lock(mt)		spin_lock((&(mt)->ma_lock))
#define mtree_unlock(mt)	spin_unlock((&(mt)->ma_lock))

#define MA_STATE(name, mt, first, end) \
	struct ma_state name = {       \
		.tree = mt,            \
		.index = first,        \
		.last = end,           \
		.node = MAS_START,     \
		.min = 0,              \
		.max = ULONG_MAX,      \
		.alloc = NULL,         \
		.mas_flags = 0,        \
	}

#define MA_WR_STATE(name, ma_state, wr_entry) \
	struct ma_wr_state name = {           \
		.mas = ma_state,              \
		.content = NULL,              \
		.entry = wr_entry,            \
	}

/*
 * Allocated nodes are mutable until they have been inserted into the tree,
 * at which time they cannot change their type until they have been removed
 * from the tree and an RCU grace period has passed.
 *
 * Removed nodes have their ->parent set to point to themselves.  RCU readers
 * check ->parent before relying on the value that they loaded from the
 * slots array.  This lets us reuse the slots array for the RCU head.
 *
 * Nodes in the tree point to their parent unless bit 0 is set.
 */

/* 64bit sizes */
#define MAPLE_NODE_SLOTS 31 /* 256 bytes including ->parent */
#define MAPLE_RANGE64_SLOTS 16 /* 256 bytes */
#define MAPLE_ARANGE64_SLOTS 10 /* 240 bytes */
#define MAPLE_ARANGE64_META_MAX 15 /* Out of range for metadata */
#define MAPLE_ALLOC_SLOTS (MAPLE_NODE_SLOTS - 1)

//data structures
struct maple_tree {
	union {
		// locks
		spinlock ma_lock;
	};
	void *ma_root;
	unsigned int ma_flags;
};

struct ma_state {
	struct maple_tree *tree; /* The tree we're operating in */
	unsigned long index; /* The index we're operating on - range start */
	unsigned long last; /* The last index we're operating on - range end */
	struct maple_enode *node; /* The node containing this entry */
	unsigned long
		min; /* The minimum index of this node - implied pivot min */
	unsigned long
		max; /* The maximum index of this node - implied pivot max */
	struct maple_alloc *alloc; /* Allocated nodes for this operation */
	unsigned char depth; /* depth of tree descent during write */
	unsigned char offset;
	unsigned char mas_flags;
};

/*
 * This metadata is used to optimize the gap updating code and in reverse
 * searching for gaps or any other code that needs to find the end of the data.
 */
struct maple_metadata {
	unsigned char end;
	unsigned char gap;
};

/*
 * Leaf nodes do not store pointers to nodes, they store user data.  Users may
 * store almost any bit pattern.  As noted above, the optimisation of storing an
 * entry at 0 in the root pointer cannot be done for data which have the bottom
 * two bits set to '10'.  We also reserve values with the bottom two bits set to
 * '10' which are below 4096 (ie 2, 6, 10 .. 4094) for internal use.  Some APIs
 * return errnos as a negative errno shifted right by two bits and the bottom
 * two bits set to '10', and while choosing to store these values in the array
 * is not an error, it may lead to confusion if you're testing for an error with
 * mas_is_err().
 *
 * Non-leaf nodes store the type of the node pointed to (enum maple_type in bits
 * 3-6), bit 2 is reserved.  That leaves bits 0-1 unused for now.
 *
 * In regular B-Tree terms, pivots are called keys.  The term pivot is used to
 * indicate that the tree is specifying ranges,  Pivots may appear in the
 * subtree with an entry attached to the value whereas keys are unique to a
 * specific position of a B-tree.  Pivot values are inclusive of the slot with
 * the same index.
 */

struct maple_range_64 {
	struct maple_pnode *parent;
	unsigned long pivot[MAPLE_RANGE64_SLOTS - 1];
	union {
		void __rcu *slot[MAPLE_RANGE64_SLOTS];
		struct {
			void __rcu *pad[MAPLE_RANGE64_SLOTS - 1];
			struct maple_metadata meta;
		};
	};
};

/*
 * At tree creation time, the user can specify that they're willing to trade off
 * storing fewer entries in a tree in return for storing more information in
 * each node.
 *
 * The maple tree supports recording the largest range of NULL entries available
 * in this node, also called gaps.  This optimises the tree for allocating a
 * range.
 */
struct maple_arange_64 {
	struct maple_pnode *parent;
	unsigned long pivot[MAPLE_ARANGE64_SLOTS - 1];
	void __rcu *slot[MAPLE_ARANGE64_SLOTS];
	unsigned long gap[MAPLE_ARANGE64_SLOTS];
	struct maple_metadata meta;
};

struct maple_alloc {
	unsigned long total;
	unsigned char node_count;
	unsigned int request_count;
	struct maple_alloc *slot[MAPLE_ALLOC_SLOTS];
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
enum maple_type {
	maple_dense,
	maple_leaf_64,
	maple_range_64,
	maple_arange_64,
};
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
struct ma_wr_state {
	struct ma_state *mas;
	struct maple_node *node; /* Decoded mas->node */
	unsigned long r_min; /* range min */
	unsigned long r_max; /* range max */
	enum maple_type type; /* mas->node type */
	unsigned char offset_end; /* The offset where the write ends */
	unsigned char node_end; /* mas->node end */
	unsigned long *pivots; /* mas->node->pivots pointer */
	unsigned long end_piv; /* The pivot at the offset end */
	void __rcu **slots; /* mas->node->slots pointer */
	void *entry; /* The entry to write */
	void *content; /* The existing entry that is being overwritten */
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
#define DEFINE_MTREE(name) struct maple_tree name = MTREE_INIT(name, 0)

#define MTREE_INIT(name, __flags)                                     \
	{                                                             \
		.ma_lock = SPINLOCK_INITIALIZER, .ma_flags = __flags, \
		.ma_root = NULL,                                      \
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
#define mt_for_each(__tree, __entry, __index, __max)                \
	for (__entry = mt_find(__tree, &(__index), __max); __entry; \
	     __entry = mt_find_after(__tree, &(__index), __max))
void *mt_find_after(struct maple_tree *mt, unsigned long *index,
		    unsigned long max);
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

int mtree_store_range(struct maple_tree *mt, unsigned long first,
		      unsigned long last, void *entry);
int mtree_store(struct maple_tree *mt, unsigned long index, void *entry);
int mtree_insert(struct maple_tree *mt, unsigned long index, void *entry);
int mtree_insert_range(struct maple_tree *mt, unsigned long first,
		       unsigned long last, void *entry);
void *mtree_erase(struct maple_tree *mt, unsigned long index);
void mtree_destroy(struct maple_tree *mt);
static inline void mt_set_in_rcu(struct maple_tree *mt);
static inline void mt_clear_in_rcu(struct maple_tree *mt);
static inline void *mas_wr_store_entry(struct ma_wr_state *wr_mas);
static inline struct maple_enode *mas_start(struct ma_state *mas);
static inline bool mas_is_start(const struct ma_state *mas);
static inline void *mas_root(struct ma_state *mas);
static inline bool mt_locked(const struct maple_tree *mt);
static inline bool mas_is_none(const struct ma_state *mas);
static inline bool mas_is_ptr(const struct ma_state *mas);
static inline void mas_store_root(struct ma_state *mas, void *entry);
static inline int mas_root_expand(struct ma_state *mas, void *entry);
static inline void *mas_root_locked(struct ma_state *mas);
static inline void *mt_root_locked(struct maple_tree *mt);
static void mas_node_count(struct ma_state *mas, int count);
static inline unsigned long mas_allocated(const struct ma_state *mas);
static inline void mas_set_alloc_req(struct ma_state *mas, unsigned long count);
static inline void mas_alloc_nodes(struct ma_state *mas);
static inline bool is_node(const void *entry);
static inline bool mte_dead_node(const struct maple_enode *enode);
static inline struct maple_node *mte_parent(const struct maple_enode *enode);
static inline void mas_push_node(struct ma_state *mas, struct maple_node *used);
static inline struct maple_node *mas_pop_node(struct ma_state *mas);
static inline unsigned int mas_alloc_req(const struct ma_state *mas);
static inline void mas_set_alloc_req(struct ma_state *mas, unsigned long count);
static inline struct maple_node *mt_alloc_one();
static inline unsigned long *ma_pivots(struct maple_node *node, enum maple_type type);
static inline void __rcu **ma_slots(struct maple_node *mn, enum maple_type mt);
static void mas_set_height(struct ma_state *mas);
static inline void ma_set_meta(struct maple_node *mn, enum maple_type mt, unsigned char offset, unsigned char end);
bool mas_nomem(struct ma_state *mas);
bool mas_is_err(struct ma_state *mas);
