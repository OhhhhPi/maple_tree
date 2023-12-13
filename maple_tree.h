#include <pthread.h>
#include "spinlock.h"
#include <urcu.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>

/*
 * The Parent Pointer
 * Excluding root, the parent pointer is 256B aligned like all other tree nodes.
 * When storing a 32 or 64 bit values, the offset can fit into 5 bits.  The 16
 * bit values need an extra bit to store the offset.  This extra bit comes from
 * a reuse of the last bit in the node type.  This is possible by using bit 1 to
 * indicate if bit 2 is part of the type or the slot.
 *
 * Note types:
 *  0x??1 = Root
 *  0x?00 = 16 bit nodes
 *  0x010 = 32 bit nodes
 *  0x110 = 64 bit nodes
 *
 * Slot size and alignment
 *  0b??1 : Root
 *  0b?00 : 16 bit values, type in 0-1, slot in 2-7
 *  0b010 : 32 bit values, type in 0-2, slot in 3-7
 *  0b110 : 64 bit values, type in 0-2, slot in 3-7
 */

#define MAPLE_PARENT_ROOT		0x01

#define MAPLE_PARENT_SLOT_SHIFT		0x03
#define MAPLE_PARENT_SLOT_MASK		0xF8

#define MAPLE_PARENT_16B_SLOT_SHIFT	0x02
#define MAPLE_PARENT_16B_SLOT_MASK	0xFC

#define MAPLE_PARENT_RANGE64		0x06
#define MAPLE_PARENT_RANGE32		0x04
#define MAPLE_PARENT_NOT_RANGE16	0x02


#define likely(x)       __builtin_expect(!!(x),1)
#define unlikely(x)     __builtin_expect(!!(x),0)

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

#define MA_TOPIARY(name, tree)						\
	struct ma_topiary name = {					\
		.head = NULL,						\
		.tail = NULL,						\
		.mtree = tree,						\
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


/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 * WARNING: any const qualifier of @ptr is lost.
 */
#define container_of(ptr, type, member) ({          \
    const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
    (type *)( (char *)__mptr - offsetof(type,member) );})

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))



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



enum maple_type {
	maple_dense,
	maple_leaf_64,
	maple_range_64,
	maple_arange_64,
};









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

 /*
 * At tree creation time, the user can specify that they're willing to trade off
 * storing fewer entries in a tree in return for storing more information in
 * each node.
 *
 * The maple tree supports recording the largest range of NULL entries available
 * in this node, also called gaps.  This optimises the tree for allocating a
 * range.
 */

struct maple_tree {
	union {
		// locks
		spinlock ma_lock;
	};
	void *ma_root;
	unsigned int ma_flags;
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

struct maple_topiary {
	struct maple_pnode *parent;
	struct maple_enode *next; /* Overlaps the pivot */
};

/*
 * More complicated stores can cause two nodes to become one or three and
 * potentially alter the height of the tree.  Either half of the tree may need
 * to be rebalanced against the other.  The ma_topiary struct is used to track
 * which nodes have been 'cut' from the tree so that the change can be done
 * safely at a later date.  This is done to support RCU.
 */
struct ma_topiary {
	struct maple_enode *head;
	struct maple_enode *tail;
	struct maple_tree *mtree;
};

void *mtree_load(struct maple_tree *mt, unsigned long index);

int mtree_insert(struct maple_tree *mt, unsigned long index,
		void *entry);
int mtree_insert_range(struct maple_tree *mt, unsigned long first,
		unsigned long last, void *entry);
int mtree_alloc_range(struct maple_tree *mt, unsigned long *startp,
		void *entry, unsigned long size, unsigned long min,
		unsigned long max);
int mtree_alloc_rrange(struct maple_tree *mt, unsigned long *startp,
		void *entry, unsigned long size, unsigned long min,
		unsigned long max);

int mtree_store_range(struct maple_tree *mt, unsigned long first,
		      unsigned long last, void *entry);
int mtree_store(struct maple_tree *mt, unsigned long index,
		void *entry);
void *mtree_erase(struct maple_tree *mt, unsigned long index);

void mtree_destroy(struct maple_tree *mt);
void __mt_destroy(struct maple_tree *mt);

/**
 * mtree_empty() - Determine if a tree has any present entries.
 * @mt: Maple Tree.
 *
 * Context: Any context.
 * Return: %true if the tree contains only NULL pointers.
 */
static inline bool mtree_empty(const struct maple_tree *mt)
{
	return mt->ma_root == NULL;
}

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

void *mas_walk(struct ma_state *mas);
void *mas_store(struct ma_state *mas, void *entry);
void *mas_erase(struct ma_state *mas);
int mas_store_gfp(struct ma_state *mas, void *entry);
void mas_store_prealloc(struct ma_state *mas, void *entry);
void *mas_find(struct ma_state *mas, unsigned long max);
void *mas_find_range(struct ma_state *mas, unsigned long max);
void *mas_find_rev(struct ma_state *mas, unsigned long min);
void *mas_find_range_rev(struct ma_state *mas, unsigned long max);
int mas_preallocate(struct ma_state *mas);
bool mas_is_err(struct ma_state *mas);

bool mas_nomem(struct ma_state *mas);
void mas_pause(struct ma_state *mas);
void maple_tree_init(void);
void mas_destroy(struct ma_state *mas);
int mas_expected_entries(struct ma_state *mas, unsigned long nr_entries);

void *mas_prev(struct ma_state *mas, unsigned long min);
void *mas_prev_range(struct ma_state *mas, unsigned long max);
void *mas_next(struct ma_state *mas, unsigned long max);
void *mas_next_range(struct ma_state *mas, unsigned long max);

int mas_empty_area(struct ma_state *mas, unsigned long min, unsigned long max,
		   unsigned long size);
/*
 * This finds an empty area from the highest address to the lowest.
 * AKA "Topdown" version,
 */
int mas_empty_area_rev(struct ma_state *mas, unsigned long min,
		       unsigned long max, unsigned long size);

static inline void mas_init(struct ma_state *mas, struct maple_tree *tree,
			    unsigned long addr)
{
	memset(mas, 0, sizeof(struct ma_state));
	mas->tree = tree;
	mas->index = mas->last = addr;
	mas->max = ULONG_MAX;
	mas->node = MAS_START;
}

/* Checks if a mas has not found anything */
static inline bool mas_is_none(const struct ma_state *mas)
{
	return mas->node == MAS_NONE;
}

/* Checks if a mas has been paused */
static inline bool mas_is_paused(const struct ma_state *mas)
{
	return mas->node == MAS_PAUSE;
}

/* Check if the mas is pointing to a node or not */
static inline bool mas_is_active(struct ma_state *mas)
{
	if ((unsigned long)mas->node >= MAPLE_RESERVED_RANGE)
		return true;

	return false;
}

/**
 * mas_reset() - Reset a Maple Tree operation state.
 * @mas: Maple Tree operation state.
 *
 * Resets the error or walk state of the @mas so future walks of the
 * array will start from the root.  Use this if you have dropped the
 * lock and want to reuse the ma_state.
 *
 * Context: Any context.
 */
static inline void mas_reset(struct ma_state *mas)
{
	mas->node = MAS_START;
}

/**
 * mas_for_each() - Iterate over a range of the maple tree.
 * @__mas: Maple Tree operation state (maple_state)
 * @__entry: Entry retrieved from the tree
 * @__max: maximum index to retrieve from the tree
 *
 * When returned, mas->index and mas->last will hold the entire range for the
 * entry.
 *
 * Note: may return the zero entry.
 */
#define mas_for_each(__mas, __entry, __max) \
	while (((__entry) = mas_find((__mas), (__max))) != NULL)

/**
 * mas_set_range() - Set up Maple Tree operation state for a different index.
 * @mas: Maple Tree operation state.
 * @start: New start of range in the Maple Tree.
 * @last: New end of range in the Maple Tree.
 *
 * Move the operation state to refer to a different range.  This will
 * have the effect of starting a walk from the top; see mas_next()
 * to move to an adjacent index.
 */
static inline
void mas_set_range(struct ma_state *mas, unsigned long start, unsigned long last)
{
	       mas->index = start;
	       mas->last = last;
	       mas->node = MAS_START;
}

/**
 * mas_set() - Set up Maple Tree operation state for a different index.
 * @mas: Maple Tree operation state.
 * @index: New index into the Maple Tree.
 *
 * Move the operation state to refer to a different index.  This will
 * have the effect of starting a walk from the top; see mas_next()
 * to move to an adjacent index.
 */
static inline void mas_set(struct ma_state *mas, unsigned long index)
{

	mas_set_range(mas, index, index);
}

static inline bool mt_external_lock(const struct maple_tree *mt)
{
	return (mt->ma_flags & MT_FLAGS_LOCK_MASK) == MT_FLAGS_LOCK_EXTERN;
}

/**
 * mt_init_flags() - Initialise an empty maple tree with flags.
 * @mt: Maple Tree
 * @flags: maple tree flags.
 *
 * If you need to initialise a Maple Tree with special flags (eg, an
 * allocation tree), use this function.
 *
 * Context: Any context.
 */
static inline void mt_init_flags(struct maple_tree *mt, unsigned int flags)
{
	mt->ma_flags = flags;
	if (!mt_external_lock(mt))
		mt->ma_lock = (spinlock)SPINLOCK_INITIALIZER;
	rcu_assign_pointer(mt->ma_root, NULL);
}

/**
 * mt_init() - Initialise an empty maple tree.
 * @mt: Maple Tree
 *
 * An empty Maple Tree.
 *
 * Context: Any context.
 */
static inline void mt_init(struct maple_tree *mt)
{
	mt_init_flags(mt, 0);
}

static inline bool mt_in_rcu(struct maple_tree *mt)
{
#ifdef CONFIG_MAPLE_RCU_DISABLED
	return false;
#endif
	return mt->ma_flags & MT_FLAGS_USE_RCU;
}

/**
 * mt_clear_in_rcu() - Switch the tree to non-RCU mode.
 * @mt: The Maple Tree
 */
static inline void mt_clear_in_rcu(struct maple_tree *mt)
{
	if (!mt_in_rcu(mt))
		return;

	if (mt_external_lock(mt)) {
		// WARN_ON(!mt_lock_is_held(mt));
		if (!mt_lock_is_held(mt)) {
			fprintf(stdout, "!mt_lock_is_held(mt)");
		}
		mt->ma_flags &= ~MT_FLAGS_USE_RCU;
	} else {
		mtree_lock(mt);
		mt->ma_flags &= ~MT_FLAGS_USE_RCU;
		mtree_unlock(mt);
	}
}

/**
 * mt_set_in_rcu() - Switch the tree to RCU safe mode.
 * @mt: The Maple Tree
 */
static inline void mt_set_in_rcu(struct maple_tree *mt)
{
	if (mt_in_rcu(mt))
		return;

	if (mt_external_lock(mt)) {
		// WARN_ON(!mt_lock_is_held(mt));
		if (!mt_lock_is_held(mt)) {
			fprintf(stdout, "!mt_lock_is_held(mt)");
		}
		mt->ma_flags |= MT_FLAGS_USE_RCU;
	} else {
		mtree_lock(mt);
		mt->ma_flags |= MT_FLAGS_USE_RCU;
		mtree_unlock(mt);
	}
}

static inline unsigned int mt_height(const struct maple_tree *mt)
{
	return (mt->ma_flags & MT_FLAGS_HEIGHT_MASK) >> MT_FLAGS_HEIGHT_OFFSET;
}

void *mt_find(struct maple_tree *mt, unsigned long *index, unsigned long max);
void *mt_find_after(struct maple_tree *mt, unsigned long *index,
		    unsigned long max);
void *mt_prev(struct maple_tree *mt, unsigned long index,  unsigned long min);
void *mt_next(struct maple_tree *mt, unsigned long index, unsigned long max);

/**
 * mt_for_each - Iterate over each entry starting at index until max.
 * @__tree: The Maple Tree
 * @__entry: The current entry
 * @__index: The index to update to track the location in the tree
 * @__max: The maximum limit for @index
 *
 * Note: Will not return the zero entry.
 */
#define mt_for_each(__tree, __entry, __index, __max) \
	for (__entry = mt_find(__tree, &(__index), __max); \
		__entry; __entry = mt_find_after(__tree, &(__index), __max))


