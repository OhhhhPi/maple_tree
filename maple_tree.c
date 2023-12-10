#include "maple_tree.h"
#include <stdio.h>
#include <stdatomic.h>

#define MA_ROOT_PARENT 1


/*
 * Maple state flags
 * * MA_STATE_BULK		- Bulk insert mode
 * * MA_STATE_REBALANCE		- Indicate a rebalance during bulk insert
 * * MA_STATE_PREALLOC		- Preallocated nodes, WARN_ON allocation
 */
#define MA_STATE_BULK		1
#define MA_STATE_REBALANCE	2
#define MA_STATE_PREALLOC	4

#define ma_parent_ptr(x) ((struct maple_pnode *)(x))
#define ma_mnode_ptr(x) ((struct maple_node *)(x))
#define ma_enode_ptr(x) ((struct maple_enode *)(x))
static struct kmem_cache *maple_node_cache;

#ifdef CONFIG_DEBUG_MAPLE_TREE
static const unsigned long mt_max[] = {
	[maple_dense]		= MAPLE_NODE_SLOTS,
	[maple_leaf_64]		= ULONG_MAX,
	[maple_range_64]	= ULONG_MAX,
	[maple_arange_64]	= ULONG_MAX,
};
#define mt_node_max(x) mt_max[mte_node_type(x)]
#endif

static const unsigned char mt_slots[] = {
	[maple_dense]		= MAPLE_NODE_SLOTS,
	[maple_leaf_64]		= MAPLE_RANGE64_SLOTS,
	[maple_range_64]	= MAPLE_RANGE64_SLOTS,
	[maple_arange_64]	= MAPLE_ARANGE64_SLOTS,
};
#define mt_slot_count(x) mt_slots[mte_node_type(x)]

static const unsigned char mt_pivots[] = {
	[maple_dense]		= 0,
	[maple_leaf_64]		= MAPLE_RANGE64_SLOTS - 1,
	[maple_range_64]	= MAPLE_RANGE64_SLOTS - 1,
	[maple_arange_64]	= MAPLE_ARANGE64_SLOTS - 1,
};
#define mt_pivot_count(x) mt_pivots[mte_node_type(x)]

static const unsigned char mt_min_slots[] = {
	[maple_dense]		= MAPLE_NODE_SLOTS / 2,
	[maple_leaf_64]		= (MAPLE_RANGE64_SLOTS / 2) - 2,
	[maple_range_64]	= (MAPLE_RANGE64_SLOTS / 2) - 2,
	[maple_arange_64]	= (MAPLE_ARANGE64_SLOTS / 2) - 1,
};
#define mt_min_slot_count(x) mt_min_slots[mte_node_type(x)]

#define MAPLE_BIG_NODE_SLOTS	(MAPLE_RANGE64_SLOTS * 2 + 2)
#define MAPLE_BIG_NODE_GAPS	(MAPLE_ARANGE64_SLOTS * 2 + 1)

struct maple_big_node {
	struct maple_pnode *parent;
	unsigned long pivot[MAPLE_BIG_NODE_SLOTS - 1];
	union {
		struct maple_enode *slot[MAPLE_BIG_NODE_SLOTS];
		struct {
			unsigned long padding[MAPLE_BIG_NODE_GAPS];
			unsigned long gap[MAPLE_BIG_NODE_GAPS];
		};
	};
	unsigned char b_end;
	enum maple_type type;
};

/*
 * The maple_subtree_state is used to build a tree to replace a segment of an
 * existing tree in a more atomic way.  Any walkers of the older tree will hit a
 * dead node and restart on updates.
 */
struct maple_subtree_state {
	struct ma_state *orig_l;	/* Original left side of subtree */
	struct ma_state *orig_r;	/* Original right side of subtree */
	struct ma_state *l;		/* New left side of subtree */
	struct ma_state *m;		/* New middle of subtree (rare) */
	struct ma_state *r;		/* New right side of subtree */
	struct ma_topiary *free;	/* nodes to be freed */
	struct ma_topiary *destroy;	/* Nodes to be destroyed (walked and freed) */
	struct maple_big_node *bn;
};

#ifdef CONFIG_KASAN_STACK
/* Prevent mas_wr_bnode() from exceeding the stack frame limit */
#define noinline_for_kasan noinline_for_stack
#else
#define noinline_for_kasan inline
#endif


/* Bit 1 indicates the root is a node */
#define MAPLE_ROOT_NODE			0x02
/* maple_type stored bit 3-6 */
#define MAPLE_ENODE_TYPE_SHIFT		0x03
/* Bit 2 means a NULL somewhere below */
#define MAPLE_ENODE_NULL		0x04

/*
 * Maple state flags
 * * MA_STATE_BULK		- Bulk insert mode
 * * MA_STATE_REBALANCE		- Indicate a rebalance during bulk insert
 * * MA_STATE_PREALLOC		- Preallocated nodes, WARN_ON allocation
 */
#define MA_STATE_BULK		1
#define MA_STATE_REBALANCE	2
#define MA_STATE_PREALLOC	4

#define ma_parent_ptr(x) ((struct maple_pnode *)(x))
#define ma_mnode_ptr(x) ((struct maple_node *)(x))
#define ma_enode_ptr(x) ((struct maple_enode *)(x))
static struct kmem_cache *maple_node_cache;


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
static bool mas_wr_walk(struct ma_wr_state *wr_mas);
static inline void mas_wr_walk_descend(struct ma_wr_state *wr_mas);
static inline void mas_wr_walk_traverse(struct ma_wr_state *wr_mas);
static inline enum maple_type mte_node_type(const struct maple_enode *entry);
static inline void mas_wr_node_walk(struct ma_wr_state *wr_mas);
static inline bool ma_is_dense(const enum maple_type type);
static inline struct maple_node *mas_mn(const struct ma_state *mas);
static inline unsigned char ma_data_end(struct maple_node *node, enum maple_type type, unsigned long *pivots, unsigned long max);
static inline unsigned char ma_meta_end(struct maple_node *mn, enum maple_type mt);
static inline struct maple_metadata *ma_meta(struct maple_node *mn, enum maple_type mt);
static inline unsigned long mas_safe_min(struct ma_state *mas, unsigned long *pivots, unsigned char offset);
static bool mas_is_span_wr(struct ma_wr_state *wr_mas);
static inline bool ma_is_leaf(const enum maple_type type);

/* Functions */
static inline struct maple_node *mt_alloc_one()
{
	return (struct maple_node *)malloc(sizeof(struct maple_node));
	// return kmem_cache_alloc(maple_node_cache, gfp);
}

static inline int mt_alloc_bulk(size_t size, void **nodes)
{
	for (size_t i = 0; i < size; ++i) {
		nodes[i] = malloc(sizeof(struct maple_node));
		if (!nodes[i]) {
			/* Handle allocation failure if needed */
			fprintf(stderr, "allocation failure");
			return -1;
		}
	}
	return 0;
	// return kmem_cache_alloc_bulk(maple_node_cache, gfp, size, nodes);
}

static inline void mt_free_bulk(size_t size, void __rcu **nodes)
{
	for (size_t i = 0; i < size; i++) {
        free(nodes[i]);
    }
	// kmem_cache_free_bulk(maple_node_cache, size, (void **)nodes);
}

static void mt_free_rcu(struct rcu_head *head)
{
	struct maple_node *node = container_of(head, struct maple_node, rcu);
	free(node);
	// kmem_cache_free(maple_node_cache, node);
}

static inline struct maple_node *mte_to_node(const struct maple_enode *entry)
{
	return (struct maple_node *)((unsigned long)entry & ~MAPLE_NODE_MASK);
}
static inline struct maple_enode *mt_mk_node(const struct maple_node *node,
					     enum maple_type type)
{
	return (void *)((unsigned long)node |
			(type << MAPLE_ENODE_TYPE_SHIFT) | MAPLE_ENODE_NULL);
}

static inline void *mte_mk_root(const struct maple_enode *node)
{
	return (void *)((unsigned long)node | MAPLE_ROOT_NODE);
}

static inline void *mte_safe_root(const struct maple_enode *node)
{
	return (void *)((unsigned long)node & ~MAPLE_ROOT_NODE);
}

static inline void *mte_set_full(const struct maple_enode *node)
{
	return (void *)((unsigned long)node & ~MAPLE_ENODE_NULL);
}

static inline void *mte_clear_full(const struct maple_enode *node)
{
	return (void *)((unsigned long)node | MAPLE_ENODE_NULL);
}

static inline bool mte_has_null(const struct maple_enode *node)
{
	return (unsigned long)node & MAPLE_ENODE_NULL;
}

static inline bool ma_is_root(struct maple_node *node)
{
	return ((unsigned long)node->parent & MA_ROOT_PARENT);
}

static inline bool mte_is_root(const struct maple_enode *node)
{
	return ma_is_root(mte_to_node(node));
}

static inline bool mas_is_root_limits(const struct ma_state *mas)
{
	return !mas->min && mas->max == ULONG_MAX;
}

static inline bool mt_is_alloc(struct maple_tree *mt)
{
	return (mt->ma_flags & MT_FLAGS_ALLOC_RANGE);
}


static inline void mt_init(struct maple_tree *mt){
	mt_init_flags(mt, 0);
}
static inline void mt_init_flags(struct maple_tree *mt, unsigned int flags)
{
	mt->ma_flags = flags;
	// if (!mt_external_lock(mt))
	// 	spin_lock_init(&mt->ma_lock);
	mt->ma_lock = SPINLOCK_INITIALIZER;
	rcu_assign_pointer(mt->ma_root, NULL);
}

int mtree_store(struct maple_tree *mt, unsigned long index, void *entry)
{
	return mtree_store_range(mt, index, index, entry);
}

int mtree_store_range(struct maple_tree *mt, unsigned long index, unsigned long last, void *entry){
	// initialize ma_state
	MA_STATE(mas,mt,index,last);
	// initialize ma_wr_state
	MA_WR_STATE(wr_mas, &mas, entry);

	// trace_ma_write(__func__, &mas, 0, entry);
	// if (WARN_ON_ONCE(xa_is_advanced(entry)))
	// 	return -EINVAL;

	if (index > last)
		return -EINVAL;

	spin_lock(&mt->ma_lock);
	// mtree_lock(mt);
retry:
	mas_wr_store_entry(&wr_mas);
	if (mas_nomem(&mas))
		goto retry;

	mtree_unlock(mt);
	// if (mas_is_err(&mas))
	// 	return xa_err(mas.node);

	return 0;
}

/* Checks if a mas has not found anything */
static inline bool mas_is_none(const struct ma_state *mas)
{
	return mas->node == MAS_NONE;
}

static inline bool mas_is_ptr(const struct ma_state *mas)
{
	return mas->node == MAS_ROOT;
}

static inline bool ma_is_dense(const enum maple_type type)
{
	return type < maple_leaf_64;
}

static inline bool ma_is_leaf(const enum maple_type type)
{
	return type < maple_range_64;
}

/*
 * mas_safe_min() - Return the minimum for a given offset.
 * @mas: The maple state
 * @pivots: The pointer to the maple node pivots
 * @offset: The offset into the pivot array
 *
 * Return: The minimum range value that is contained in @offset.
 */
static inline unsigned long
mas_safe_min(struct ma_state *mas, unsigned long *pivots, unsigned char offset)
{
	if (likely(offset))
		return pivots[offset - 1] + 1;

	return mas->min;
}


/*
 * mas_mn() - Get the maple state node.
 * @mas: The maple state
 *
 * Return: the maple node (not encoded - bare pointer).
 */
static inline struct maple_node *mas_mn(const struct ma_state *mas)
{
	return mte_to_node(mas->node);
}

static inline struct maple_metadata *ma_meta(struct maple_node *mn,
					     enum maple_type mt)
{
	switch (mt) {
	case maple_arange_64:
		return &mn->ma64.meta;
	default:
		return &mn->mr64.meta;
	}
}

/*
 * ma_meta_end() - Get the data end of a node from the metadata
 * @mn: The maple node
 * @mt: The maple node type
 */
static inline unsigned char ma_meta_end(struct maple_node *mn,
					enum maple_type mt)
{
	struct maple_metadata *meta = ma_meta(mn, mt);

	return meta->end;
}

/*
 * ma_data_end() - Find the end of the data in a node.
 * @node: The maple node
 * @type: The maple node type
 * @pivots: The array of pivots in the node
 * @max: The maximum value in the node
 *
 * Uses metadata to find the end of the data when possible.
 * Return: The zero indexed last slot with data (may be null).
 */
static inline unsigned char ma_data_end(struct maple_node *node,
					enum maple_type type,
					unsigned long *pivots,
					unsigned long max)
{
	unsigned char offset;

	if (!pivots)
		return 0;

	if (type == maple_arange_64)
		return ma_meta_end(node, type);

	offset = mt_pivots[type] - 1;
	if (likely(!pivots[offset]))
		return ma_meta_end(node, type);

	if (likely(pivots[offset] == max))
		return offset;

	return mt_pivots[type];
}

/*
 * mas_wr_node_walk() - Find the correct offset for the index in the @mas.
 * @wr_mas: The maple write state
 *
 * Uses mas_slot_locked() and does not need to worry about dead nodes.
 */
static inline void mas_wr_node_walk(struct ma_wr_state *wr_mas)
{
	struct ma_state *mas = wr_mas->mas;
	unsigned char count, offset;

	if (unlikely(ma_is_dense(wr_mas->type))) {
		wr_mas->r_max = wr_mas->r_min = mas->index;
		mas->offset = mas->index = mas->min;
		return;
	}

	wr_mas->node = mas_mn(wr_mas->mas);
	wr_mas->pivots = ma_pivots(wr_mas->node, wr_mas->type);
	count = wr_mas->node_end = ma_data_end(wr_mas->node, wr_mas->type,
					       wr_mas->pivots, mas->max);
	offset = mas->offset;

	while (offset < count && mas->index > wr_mas->pivots[offset])
		offset++;

	wr_mas->r_max = offset < count ? wr_mas->pivots[offset] : mas->max;
	wr_mas->r_min = mas_safe_min(mas, wr_mas->pivots, offset);
	wr_mas->offset_end = mas->offset = offset;
}


static inline enum maple_type mte_node_type(const struct maple_enode *entry)
{
	return ((unsigned long)entry >> MAPLE_NODE_TYPE_SHIFT) &
		MAPLE_NODE_TYPE_MASK;
}

static inline void mas_wr_walk_descend(struct ma_wr_state *wr_mas)
{
	wr_mas->type = mte_node_type(wr_mas->mas->node);
	mas_wr_node_walk(wr_mas);
	wr_mas->slots = ma_slots(wr_mas->node, wr_mas->type);
}

static inline void mas_wr_walk_traverse(struct ma_wr_state *wr_mas)
{
	wr_mas->mas->max = wr_mas->r_max;
	wr_mas->mas->min = wr_mas->r_min;
	wr_mas->mas->node = wr_mas->content;
	wr_mas->mas->offset = 0;
	wr_mas->mas->depth++;
}

/*
 * mas_is_span_wr() - Check if the write needs to be treated as a write that
 * spans the node.
 * @mas: The maple state
 * @piv: The pivot value being written
 * @type: The maple node type
 * @entry: The data to write
 *
 * Spanning writes are writes that start in one node and end in another OR if
 * the write of a %NULL will cause the node to end with a %NULL.
 *
 * Return: True if this is a spanning write, false otherwise.
 */
static bool mas_is_span_wr(struct ma_wr_state *wr_mas)
{
	unsigned long max = wr_mas->r_max;
	unsigned long last = wr_mas->mas->last;
	enum maple_type type = wr_mas->type;
	void *entry = wr_mas->entry;

	/* Contained in this pivot, fast path */
	if (last < max)
		return false;

	if (ma_is_leaf(type)) {
		max = wr_mas->mas->max;
		if (last < max)
			return false;
	}

	if (last == max) {
		/*
		 * The last entry of leaf node cannot be NULL unless it is the
		 * rightmost node (writing ULONG_MAX), otherwise it spans slots.
		 */
		if (entry || last == ULONG_MAX)
			return false;
	}

	// trace_ma_write(__func__, wr_mas->mas, wr_mas->r_max, entry);
	return true;
}

/*
 * mas_slot_locked() - Get the slot value when holding the maple tree lock.
 * @mas: The maple state
 * @slots: The pointer to the slots
 * @offset: The offset into the slots array to fetch
 *
 * Return: The entry stored in @slots at the @offset.
 */
static inline void *mas_slot_locked(struct ma_state *mas, void __rcu **slots,
				       unsigned char offset)
{
	if(!mt_locked(mas->tree)){
		fprintf(stderr, "!mt_locked(mas->tree)");
	}
	return rcu_dereference(slots[offset]);
	// return mt_slot_locked(mas->tree, slots, offset);
}

/*
 * mas_wr_walk() - Walk the tree for a write.
 * @wr_mas: The maple write state
 *
 * Uses mas_slot_locked() and does not need to worry about dead nodes.
 *
 * Return: True if it's contained in a node, false on spanning write.
 */
static bool mas_wr_walk(struct ma_wr_state *wr_mas)
{
	struct ma_state *mas = wr_mas->mas;

	while (true) {
		mas_wr_walk_descend(wr_mas);
		if (unlikely(mas_is_span_wr(wr_mas)))
			return false;

		wr_mas->content = mas_slot_locked(mas, wr_mas->slots,
						  mas->offset);
		if (ma_is_leaf(wr_mas->type))
			return true;

		mas_wr_walk_traverse(wr_mas);
	}

	return true;
}

static inline bool mte_is_leaf(const struct maple_enode *entry)
{
	return ma_is_leaf(mte_node_type(entry));
}

static inline void *mt_slot_locked(struct maple_tree *mt, void __rcu **slots,
				   unsigned char offset)
{
	if(!mt_locked(mt)){
		fprintf(stderr, "!mt_locked(mt)");
	}
	return rcu_dereference(slots[offset]);
	// return rcu_dereference_protected(slots[offset], mt_locked(mt));
}

/*
 * mte_set_node_dead() - Set a maple encoded node as dead.
 * @mn: The maple encoded node.
 */
static inline void mte_set_node_dead(struct maple_enode *mn)
{
	mte_to_node(mn)->parent = ma_parent_ptr(mte_to_node(mn));
    atomic_thread_fence(memory_order_release); /* Needed for RCU */
	// smp_wmb(); /* Needed for RCU */
}

static inline void __rcu **mte_destroy_descend(struct maple_enode **enode,
	struct maple_tree *mt, struct maple_enode *prev, unsigned char offset)
{
	struct maple_node *node;
	struct maple_enode *next = *enode;
	void __rcu **slots = NULL;
	enum maple_type type;
	unsigned char next_offset = 0;

	do {
		*enode = next;
		node = mte_to_node(*enode);
		type = mte_node_type(*enode);
		slots = ma_slots(node, type);
		next = mt_slot_locked(mt, slots, next_offset);
		if ((mte_dead_node(next)))
			next = mt_slot_locked(mt, slots, ++next_offset);

		mte_set_node_dead(*enode);
		node->type = type;
		node->piv_parent = prev;
		node->parent_slot = offset;
		offset = next_offset;
		next_offset = 0;
		prev = *enode;
	} while (!mte_is_leaf(next));

	return slots;
}

static inline void *mt_slot(const struct maple_tree *mt,
		void __rcu **slots, unsigned char offset)
{
	if(!mt_locked(mt)){
		fprintf(stderr, "!mt_locked(mt)");
	}
	return rcu_dereference(slots[offset]);
	// return rcu_dereference_check(slots[offset], mt_locked(mt));
}

/*
 * mte_dead_leaves() - Mark all leaves of a node as dead.
 * @mas: The maple state
 * @slots: Pointer to the slot array
 * @type: The maple node type
 *
 * Must hold the write lock.
 *
 * Return: The number of leaves marked as dead.
 */
static inline
unsigned char mte_dead_leaves(struct maple_enode *enode, struct maple_tree *mt,
			      void __rcu **slots)
{
	struct maple_node *node;
	enum maple_type type;
	void *entry;
	int offset;

	for (offset = 0; offset < mt_slot_count(enode); offset++) {
		entry = mt_slot(mt, slots, offset);
		type = mte_node_type(entry);
		node = mte_to_node(entry);
		/* Use both node and type to catch LE & BE metadata */
		if (!node || !type)
			break;

		mte_set_node_dead(entry);
		node->type = type;
		rcu_assign_pointer(slots[offset], node);
	}

	return offset;
}


/**
 * mte_dead_walk() - Walk down a dead tree to just before the leaves
 * @enode: The maple encoded node
 * @offset: The starting offset
 *
 * Note: This can only be used from the RCU callback context.
 */
static void __rcu **mte_dead_walk(struct maple_enode **enode, unsigned char offset)
{
	struct maple_node *node, *next;
	void __rcu **slots = NULL;

	next = mte_to_node(*enode);
	do {
		*enode = ma_enode_ptr(next);
		node = mte_to_node(*enode);
		slots = ma_slots(node, node->type);
		// if(!lock_is_held(&rcu_callback_map)){
		// }
		next = rcu_dereference(slots[offset]);
		offset = 0;
	} while (!ma_is_leaf(next->type));

	return slots;
}

/**
 * mt_free_walk() - Walk & free a tree in the RCU callback context
 * @head: The RCU head that's within the node.
 *
 * Note: This can only be used from the RCU callback context.
 */
static void mt_free_walk(struct rcu_head *head)
{
	void __rcu **slots;
	struct maple_node *node, *start;
	struct maple_enode *enode;
	unsigned char offset;
	enum maple_type type;

	node = container_of(head, struct maple_node, rcu);

	if (ma_is_leaf(node->type))
		goto free_leaf;

	start = node;
	enode = mt_mk_node(node, node->type);
	slots = mte_dead_walk(&enode, 0);
	node = mte_to_node(enode);
	do {
		mt_free_bulk(node->slot_len, slots);
		offset = node->parent_slot + 1;
		enode = node->piv_parent;
		if (mte_to_node(enode) == node)
			goto free_leaf;

		type = mte_node_type(enode);
		slots = ma_slots(mte_to_node(enode), type);
		if ((offset < mt_slots[type]) &&
		    rcu_dereference(slots[offset]))
					      // lock_is_held(&rcu_callback_map)))
			slots = mte_dead_walk(&enode, offset);
		node = mte_to_node(enode);
	} while ((node != start) || (node->slot_len < offset));

	slots = ma_slots(node, node->type);
	mt_free_bulk(node->slot_len, slots);

free_leaf:
	mt_free_rcu(&node->rcu);
}

/*
 * mt_clear_meta() - clear the metadata information of a node, if it exists
 * @mt: The maple tree
 * @mn: The maple node
 * @type: The maple node type
 * @offset: The offset of the highest sub-gap in this node.
 * @end: The end of the data in this node.
 */
static inline void mt_clear_meta(struct maple_tree *mt, struct maple_node *mn,
				  enum maple_type type)
{
	struct maple_metadata *meta;
	unsigned long *pivots;
	void __rcu **slots;
	void *next;

	switch (type) {
	case maple_range_64:
		pivots = mn->mr64.pivot;
		if (unlikely(pivots[MAPLE_RANGE64_SLOTS - 2])) {
			slots = mn->mr64.slot;
			next = mt_slot_locked(mt, slots,
					      MAPLE_RANGE64_SLOTS - 1);
			if (unlikely((mte_to_node(next) &&
				      mte_node_type(next))))
				return; /* no metadata, could be node */
		}
		// fallthrough;
	case maple_arange_64:
		meta = ma_meta(mn, type);
		break;
	default:
		return;
	}

	meta->gap = 0;
	meta->end = 0;
}

static void mt_destroy_walk(struct maple_enode *enode, struct maple_tree *mt,
			    bool free)
{
	void __rcu **slots;
	struct maple_node *node = mte_to_node(enode);
	struct maple_enode *start;

	if (mte_is_leaf(enode)) {
		node->type = mte_node_type(enode);
		goto free_leaf;
	}

	start = enode;
	slots = mte_destroy_descend(&enode, mt, start, 0);
	node = mte_to_node(enode); // Updated in the above call.
	do {
		enum maple_type type;
		unsigned char offset;
		struct maple_enode *parent, *tmp;

		node->slot_len = mte_dead_leaves(enode, mt, slots);
		if (free)
			mt_free_bulk(node->slot_len, slots);
		offset = node->parent_slot + 1;
		enode = node->piv_parent;
		if (mte_to_node(enode) == node)
			goto free_leaf;

		type = mte_node_type(enode);
		slots = ma_slots(mte_to_node(enode), type);
		if (offset >= mt_slots[type])
			goto next;

		tmp = mt_slot_locked(mt, slots, offset);
		if (mte_node_type(tmp) && mte_to_node(tmp)) {
			parent = enode;
			enode = tmp;
			slots = mte_destroy_descend(&enode, mt, parent, offset);
		}
next:
		node = mte_to_node(enode);
	} while (start != enode);

	node = mte_to_node(enode);
	node->slot_len = mte_dead_leaves(enode, mt, slots);
	if (free)
		mt_free_bulk(node->slot_len, slots);

free_leaf:
	if (free)
		mt_free_rcu(&node->rcu);
	else
		mt_clear_meta(mt, node, node->type);
}


static inline bool mt_in_rcu(struct maple_tree *mt)
{
// #ifdef CONFIG_MAPLE_RCU_DISABLED
// 	return false;
// #endif
	return mt->ma_flags & MT_FLAGS_USE_RCU;
}

/*
 * mte_destroy_walk() - Free a tree or sub-tree.
 * @enode: the encoded maple node (maple_enode) to start
 * @mt: the tree to free - needed for node types.
 *
 * Must hold the write lock.
 */
static inline void mte_destroy_walk(struct maple_enode *enode,
				    struct maple_tree *mt)
{
	struct maple_node *node = mte_to_node(enode);

	if (mt_in_rcu(mt)) {
		mt_destroy_walk(enode, mt, false);
		call_rcu(&node->rcu, mt_free_walk);
	} else {
		mt_destroy_walk(enode, mt, true);
	}
}


/*
 * mas_new_root() - Create a new root node that only contains the entry passed
 * in.
 * @mas: The maple state
 * @entry: The entry to store.
 *
 * Only valid when the index == 0 and the last == ULONG_MAX
 *
 * Return 0 on error, 1 on success.
 */
static inline int mas_new_root(struct ma_state *mas, void *entry)
{
	struct maple_enode *root = mas_root_locked(mas);
	enum maple_type type = maple_leaf_64;
	struct maple_node *node;
	void __rcu **slots;
	unsigned long *pivots;

	if (!entry && !mas->index && mas->last == ULONG_MAX) {
		mas->depth = 0;
		mas_set_height(mas);
		rcu_assign_pointer(mas->tree->ma_root, entry);
		mas->node = MAS_START;
		goto done;
	}

	mas_node_count(mas, 1);
	if (mas_is_err(mas))
		return 0;

	node = mas_pop_node(mas);
	pivots = ma_pivots(node, type);
	slots = ma_slots(node, type);
	node->parent = ma_parent_ptr(
		      ((unsigned long)mas->tree | MA_ROOT_PARENT));
	mas->node = mt_mk_node(node, type);
	rcu_assign_pointer(slots[0], entry);
	pivots[0] = mas->last;
	mas->depth = 1;
	mas_set_height(mas);
	rcu_assign_pointer(mas->tree->ma_root, mte_mk_root(mas->node));

done:
	// if (xa_is_node(root))
	// 	mte_destroy_walk(root, mas->tree);
	mte_destroy_walk(root, mas->tree);
	return 1;
}

static unsigned int mas_mt_height(struct ma_state *mas)
{
	return mt_height(mas->tree);
}

static bool mas_wr_walk_index(struct ma_wr_state *wr_mas)
{
	struct ma_state *mas = wr_mas->mas;

	while (true) {
		mas_wr_walk_descend(wr_mas);
		wr_mas->content = mas_slot_locked(mas, wr_mas->slots,
						  mas->offset);
		if (ma_is_leaf(wr_mas->type))
			return true;
		mas_wr_walk_traverse(wr_mas);

	}
	return true;
}

/*
 * mas_safe_pivot() - get the pivot at @piv or mas->max.
 * @mas: The maple state
 * @pivots: The pointer to the maple node pivots
 * @piv: The pivot to fetch
 * @type: The maple node type
 *
 * Return: The pivot at @piv within the limit of the @pivots array, @mas->max
 * otherwise.
 */
static inline unsigned long
mas_safe_pivot(const struct ma_state *mas, unsigned long *pivots,
	       unsigned char piv, enum maple_type type)
{
	if (piv >= mt_pivots[type])
		return mas->max;

	return pivots[piv];
}

/*
 * mas_extend_spanning_null() - Extend a store of a %NULL to include surrounding %NULLs.
 * @l_wr_mas: The left maple write state
 * @r_wr_mas: The right maple write state
 */
static inline void mas_extend_spanning_null(struct ma_wr_state *l_wr_mas,
					    struct ma_wr_state *r_wr_mas)
{
	struct ma_state *r_mas = r_wr_mas->mas;
	struct ma_state *l_mas = l_wr_mas->mas;
	unsigned char l_slot;

	l_slot = l_mas->offset;
	if (!l_wr_mas->content)
		l_mas->index = l_wr_mas->r_min;

	if ((l_mas->index == l_wr_mas->r_min) &&
		 (l_slot &&
		  !mas_slot_locked(l_mas, l_wr_mas->slots, l_slot - 1))) {
		if (l_slot > 1)
			l_mas->index = l_wr_mas->pivots[l_slot - 2] + 1;
		else
			l_mas->index = l_mas->min;

		l_mas->offset = l_slot - 1;
	}

	if (!r_wr_mas->content) {
		if (r_mas->last < r_wr_mas->r_max)
			r_mas->last = r_wr_mas->r_max;
		r_mas->offset++;
	} else if ((r_mas->last == r_wr_mas->r_max) &&
	    (r_mas->last < r_mas->max) &&
	    !mas_slot_locked(r_mas, r_wr_mas->slots, r_mas->offset + 1)) {
		r_mas->last = mas_safe_pivot(r_mas, r_wr_mas->pivots,
					     r_wr_mas->type, r_mas->offset + 1);
		r_mas->offset++;
	}
}

/*
 * ma_gaps() - Get a pointer to the maple node gaps.
 * @node - the maple node
 * @type - the node type
 *
 * Return: A pointer to the maple node gaps
 */
static inline unsigned long *ma_gaps(struct maple_node *node,
				     enum maple_type type)
{
	switch (type) {
	case maple_arange_64:
		return node->ma64.gap;
	case maple_range_64:
	case maple_leaf_64:
	case maple_dense:
		return NULL;
	}
	return NULL;
}

/*
 * mas_mab_cp() - Copy data from a maple state inclusively to a maple_big_node
 * and set @b_node->b_end to the next free slot.
 * @mas: The maple state
 * @mas_start: The starting slot to copy
 * @mas_end: The end slot to copy (inclusively)
 * @b_node: The maple_big_node to place the data
 * @mab_start: The starting location in maple_big_node to store the data.
 */
static inline void mas_mab_cp(struct ma_state *mas, unsigned char mas_start,
			unsigned char mas_end, struct maple_big_node *b_node,
			unsigned char mab_start)
{
	enum maple_type mt;
	struct maple_node *node;
	void __rcu **slots;
	unsigned long *pivots, *gaps;
	int i = mas_start, j = mab_start;
	unsigned char piv_end;

	node = mas_mn(mas);
	mt = mte_node_type(mas->node);
	pivots = ma_pivots(node, mt);
	if (!i) {
		b_node->pivot[j] = pivots[i++];
		if (unlikely(i > mas_end))
			goto complete;
		j++;
	}

	piv_end = min(mas_end, mt_pivots[mt]);
	for (; i < piv_end; i++, j++) {
		b_node->pivot[j] = pivots[i];
		if (unlikely(!b_node->pivot[j]))
			break;

		if (unlikely(mas->max == b_node->pivot[j]))
			goto complete;
	}

	if (likely(i <= mas_end))
		b_node->pivot[j] = mas_safe_pivot(mas, pivots, i, mt);

complete:
	b_node->b_end = ++j;
	j -= mab_start;
	slots = ma_slots(node, mt);
	memcpy(b_node->slot + mab_start, slots + mas_start, sizeof(void *) * j);
	if (!ma_is_leaf(mt) && mt_is_alloc(mas->tree)) {
		gaps = ma_gaps(node, mt);
		memcpy(b_node->gap + mab_start, gaps + mas_start,
		       sizeof(unsigned long) * j);
	}
}

/*
 * mas_logical_pivot() - Get the logical pivot of a given offset.
 * @mas: The maple state
 * @pivots: The pointer to the maple node pivots
 * @offset: The offset into the pivot array
 * @type: The maple node type
 *
 * When there is no value at a pivot (beyond the end of the data), then the
 * pivot is actually @mas->max.
 *
 * Return: the logical pivot of a given @offset.
 */
static inline unsigned long
mas_logical_pivot(struct ma_state *mas, unsigned long *pivots,
		  unsigned char offset, enum maple_type type)
{
	unsigned long lpiv = mas_safe_pivot(mas, pivots, offset, type);

	if (likely(lpiv))
		return lpiv;

	if (likely(offset))
		return mas->max;

	return lpiv;
}

/*
 * mas_bulk_rebalance() - Rebalance the end of a tree after a bulk insert.
 * @mas: The maple state
 * @end: The maple node end
 * @mt: The maple node type
 */
static inline void mas_bulk_rebalance(struct ma_state *mas, unsigned char end,
				      enum maple_type mt)
{
	if (!(mas->mas_flags & MA_STATE_BULK))
		return;

	if (mte_is_root(mas->node))
		return;

	if (end > mt_min_slots[mt]) {
		mas->mas_flags &= ~MA_STATE_REBALANCE;
		return;
	}
}

/*
 * mas_store_b_node() - Store an @entry into the b_node while also copying the
 * data from a maple encoded node.
 * @wr_mas: the maple write state
 * @b_node: the maple_big_node to fill with data
 * @offset_end: the offset to end copying
 *
 * Return: The actual end of the data stored in @b_node
 */
static noinline_for_kasan void mas_store_b_node(struct ma_wr_state *wr_mas,
		struct maple_big_node *b_node, unsigned char offset_end)
{
	unsigned char slot;
	unsigned char b_end;
	/* Possible underflow of piv will wrap back to 0 before use. */
	unsigned long piv;
	struct ma_state *mas = wr_mas->mas;

	b_node->type = wr_mas->type;
	b_end = 0;
	slot = mas->offset;
	if (slot) {
		/* Copy start data up to insert. */
		mas_mab_cp(mas, 0, slot - 1, b_node, 0);
		b_end = b_node->b_end;
		piv = b_node->pivot[b_end - 1];
	} else
		piv = mas->min - 1;

	if (piv + 1 < mas->index) {
		/* Handle range starting after old range */
		b_node->slot[b_end] = wr_mas->content;
		if (!wr_mas->content)
			b_node->gap[b_end] = mas->index - 1 - piv;
		b_node->pivot[b_end++] = mas->index - 1;
	}

	/* Store the new entry. */
	mas->offset = b_end;
	b_node->slot[b_end] = wr_mas->entry;
	b_node->pivot[b_end] = mas->last;

	/* Appended. */
	if (mas->last >= mas->max)
		goto b_end;

	/* Handle new range ending before old range ends */
	piv = mas_logical_pivot(mas, wr_mas->pivots, offset_end, wr_mas->type);
	if (piv > mas->last) {
		if (piv == ULONG_MAX)
			mas_bulk_rebalance(mas, b_node->b_end, wr_mas->type);

		if (offset_end != slot)
			wr_mas->content = mas_slot_locked(mas, wr_mas->slots,
							  offset_end);

		b_node->slot[++b_end] = wr_mas->content;
		if (!wr_mas->content)
			b_node->gap[b_end] = piv - mas->last + 1;
		b_node->pivot[b_end] = piv;
	}

	slot = offset_end + 1;
	if (slot > wr_mas->node_end)
		goto b_end;

	/* Copy end data to the end of the node. */
	mas_mab_cp(mas, slot, wr_mas->node_end + 1, b_node, ++b_end);
	b_node->b_end--;
	return;

b_end:
	b_node->b_end = b_end;
}

/*
 * mte_parent_slot_mask() - Get the slot mask for the parent.
 * @parent: The parent pointer cast as an unsigned long.
 * Return: The slot mask for that parent.
 */
static inline unsigned long mte_parent_slot_mask(unsigned long parent)
{
	/* Note bit 1 == 0 means 16B */
	if (likely(parent & MAPLE_PARENT_NOT_RANGE16))
		return MAPLE_PARENT_SLOT_MASK;

	return MAPLE_PARENT_16B_SLOT_MASK;
}

/*
 * mas_parent_type() - Return the maple_type of the parent from the stored
 * parent type.
 * @mas: The maple state
 * @enode: The maple_enode to extract the parent's enum
 * Return: The node->parent maple_type
 */
static inline
enum maple_type mas_parent_type(struct ma_state *mas, struct maple_enode *enode)
{
	unsigned long p_type;

	p_type = (unsigned long)mte_to_node(enode)->parent;
	if(p_type & MAPLE_PARENT_ROOT){
		fprintf(stdout, "parent is MAPLE_PARENT_ROOT");
	}
	
	// if (WARN_ON(p_type & MAPLE_PARENT_ROOT))
	// 	return 0;

	p_type &= MAPLE_NODE_MASK;
	p_type &= ~mte_parent_slot_mask(p_type);
	switch (p_type) {
	case MAPLE_PARENT_RANGE64: /* or MAPLE_PARENT_ARANGE64 */ 
		if (mt_is_alloc(mas->tree))
			return maple_arange_64;
		return maple_range_64;
	}

	return 0;
}

/*
 * mte_parent_shift() - Get the parent shift for the slot storage.
 * @parent: The parent pointer cast as an unsigned long
 * Return: The shift into that pointer to the star to of the slot
 */
static inline unsigned long mte_parent_shift(unsigned long parent)
{
	/* Note bit 1 == 0 means 16B */
	if (likely(parent & MAPLE_PARENT_NOT_RANGE16))
		return MAPLE_PARENT_SLOT_SHIFT;

	return MAPLE_PARENT_16B_SLOT_SHIFT;
}

/*
 * mte_parent_slot() - get the parent slot of @enode.
 * @enode: The encoded maple node.
 *
 * Return: The slot in the parent node where @enode resides.
 */
static inline unsigned int mte_parent_slot(const struct maple_enode *enode)
{
	unsigned long val = (unsigned long)mte_to_node(enode)->parent;

	if (val & MA_ROOT_PARENT)
		return 0;

	/*
	 * Okay to use MAPLE_PARENT_16B_SLOT_MASK as the last bit will be lost
	 * by shift if the parent shift is MAPLE_PARENT_SLOT_SHIFT
	 */
	return (val & MAPLE_PARENT_16B_SLOT_MASK) >> mte_parent_shift(val);
}

/*
 * ma_dead_node() - check if the @enode is dead.
 * @enode: The encoded maple node
 *
 * Return: true if dead, false otherwise.
 */
static inline bool ma_dead_node(const struct maple_node *node)
{
	struct maple_node *parent;

	/* Do not reorder reads from the node prior to the parent check */
	// smp_rmb();
	atomic_thread_fence(memory_order_release);
	parent = (void *)((unsigned long) node->parent & ~MAPLE_NODE_MASK);
	return (parent == node);
}

/*
 * mas_ascend() - Walk up a level of the tree.
 * @mas: The maple state
 *
 * Sets the @mas->max and @mas->min to the correct values when walking up.  This
 * may cause several levels of walking up to find the correct min and max.
 * May find a dead node which will cause a premature return.
 * Return: 1 on dead node, 0 otherwise
 */
static int mas_ascend(struct ma_state *mas)
{
	struct maple_enode *p_enode; /* parent enode. */
	struct maple_enode *a_enode; /* ancestor enode. */
	struct maple_node *a_node; /* ancestor node. */
	struct maple_node *p_node; /* parent node. */
	unsigned char a_slot;
	enum maple_type a_type;
	unsigned long min, max;
	unsigned long *pivots;
	bool set_max = false, set_min = false;

	a_node = mas_mn(mas);
	if (ma_is_root(a_node)) {
		mas->offset = 0;
		return 0;
	}

	p_node = mte_parent(mas->node);
	if (unlikely(a_node == p_node))
		return 1;

	a_type = mas_parent_type(mas, mas->node);
	mas->offset = mte_parent_slot(mas->node);
	a_enode = mt_mk_node(p_node, a_type);

	/* Check to make sure all parent information is still accurate */
	if (p_node != mte_parent(mas->node))
		return 1;

	mas->node = a_enode;

	if (mte_is_root(a_enode)) {
		mas->max = ULONG_MAX;
		mas->min = 0;
		return 0;
	}

	if (!mas->min)
		set_min = true;

	if (mas->max == ULONG_MAX)
		set_max = true;

	min = 0;
	max = ULONG_MAX;
	do {
		p_enode = a_enode;
		a_type = mas_parent_type(mas, p_enode);
		a_node = mte_parent(p_enode);
		a_slot = mte_parent_slot(p_enode);
		a_enode = mt_mk_node(a_node, a_type);
		pivots = ma_pivots(a_node, a_type);

		if (unlikely(ma_dead_node(a_node)))
			return 1;

		if (!set_min && a_slot) {
			set_min = true;
			min = pivots[a_slot - 1] + 1;
		}

		if (!set_max && a_slot < mt_pivots[a_type]) {
			set_max = true;
			max = pivots[a_slot];
		}

		if (unlikely(ma_dead_node(a_node)))
			return 1;

		if (unlikely(ma_is_root(a_node)))
			break;

	} while (!set_min || !set_max);

	mas->max = max;
	mas->min = min;
	return 0;
}

/*
 * mas_data_end() - Find the end of the data (slot).
 * @mas: the maple state
 *
 * This method is optimized to check the metadata of a node if the node type
 * supports data end metadata.
 *
 * Return: The zero indexed last slot with data (may be null).
 */
static inline unsigned char mas_data_end(struct ma_state *mas)
{
	enum maple_type type;
	struct maple_node *node;
	unsigned char offset;
	unsigned long *pivots;

	type = mte_node_type(mas->node);
	node = mas_mn(mas);
	if (type == maple_arange_64)
		return ma_meta_end(node, type);

	pivots = ma_pivots(node, type);
	if (unlikely(ma_dead_node(node)))
		return 0;

	offset = mt_pivots[type] - 1;
	if (likely(!pivots[offset]))
		return ma_meta_end(node, type);

	if (likely(pivots[offset] == mas->max))
		return offset;

	return mt_pivots[type];
}

/*
 * mas_slot() - Get the slot value when not holding the maple tree lock.
 * @mas: The maple state
 * @slots: The pointer to the slots
 * @offset: The offset into the slots array to fetch
 *
 * Return: The entry stored in @slots at the @offset
 */
static inline void *mas_slot(struct ma_state *mas, void __rcu **slots,
			     unsigned char offset)
{
	return mt_slot(mas->tree, slots, offset);
}

/*
 * mas_descend() - Descend into the slot stored in the ma_state.
 * @mas - the maple state.
 *
 * Note: Not RCU safe, only use in write side or debug code.
 */
static inline void mas_descend(struct ma_state *mas)
{
	enum maple_type type;
	unsigned long *pivots;
	struct maple_node *node;
	void __rcu **slots;

	node = mas_mn(mas);
	type = mte_node_type(mas->node);
	pivots = ma_pivots(node, type);
	slots = ma_slots(node, type);

	if (mas->offset)
		mas->min = pivots[mas->offset - 1] + 1;
	mas->max = mas_safe_pivot(mas, pivots, mas->offset, type);
	mas->node = mas_slot(mas, slots, mas->offset);
}

/*
 * mast_rebalance_next() - Rebalance against the next node
 * @mast: The maple subtree state
 * @old_r: The encoded maple node to the right (next node).
 */
static inline void mast_rebalance_next(struct maple_subtree_state *mast)
{
	unsigned char b_end = mast->bn->b_end;

	mas_mab_cp(mast->orig_r, 0, mt_slot_count(mast->orig_r->node),
		   mast->bn, b_end);
	mast->orig_r->last = mast->orig_r->max;
}

/*
 * mte_to_mat() - Convert a maple encoded node to a maple topiary node.
 * @entry: The maple encoded node
 *
 * Return: a maple topiary pointer
 */
static inline struct maple_topiary *mte_to_mat(const struct maple_enode *entry)
{
	return (struct maple_topiary *)
		((unsigned long)entry & ~MAPLE_NODE_MASK);
}

/*
 * mat_add() - Add a @dead_enode to the ma_topiary of a list of dead nodes.
 * @mat - the ma_topiary, a linked list of dead nodes.
 * @dead_enode - the node to be marked as dead and added to the tail of the list
 *
 * Add the @dead_enode to the linked list in @mat.
 */
static inline void mat_add(struct ma_topiary *mat,
			   struct maple_enode *dead_enode)
{
	mte_set_node_dead(dead_enode);
	mte_to_mat(dead_enode)->next = NULL;
	if (!mat->tail) {
		mat->tail = mat->head = dead_enode;
		return;
	}

	mte_to_mat(mat->tail)->next = dead_enode;
	mat->tail = dead_enode;
}

/*
 * mas_topiary_range() - Add a range of slots to the topiary.
 * @mas: The maple state
 * @destroy: The topiary to add the slots (usually destroy)
 * @start: The starting slot inclusively
 * @end: The end slot inclusively
 */
static inline void mas_topiary_range(struct ma_state *mas,
	struct ma_topiary *destroy, unsigned char start, unsigned char end)
{
	void __rcu **slots;
	unsigned char offset;

	if(mte_is_leaf(mas->node)){
		fprintf(stderr, "mte_is_leaf(mas->node)");
	}

	// MAS_BUG_ON(mas, mte_is_leaf(mas->node));

	slots = ma_slots(mas_mn(mas), mte_node_type(mas->node));
	for (offset = start; offset <= end; offset++) {
		struct maple_enode *enode = mas_slot_locked(mas, slots, offset);

		if (mte_dead_node(enode))
			continue;

		mat_add(destroy, enode);
	}
}

/*
 * mab_shift_right() - Shift the data in mab right. Note, does not clean out the
 * old data or set b_node->b_end.
 * @b_node: the maple_big_node
 * @shift: the shift count
 */
static inline void mab_shift_right(struct maple_big_node *b_node,
				 unsigned char shift)
{
	unsigned long size = b_node->b_end * sizeof(unsigned long);

	memmove(b_node->pivot + shift, b_node->pivot, size);
	memmove(b_node->slot + shift, b_node->slot, size);
	if (b_node->type == maple_arange_64)
		memmove(b_node->gap + shift, b_node->gap, size);
}

/*
 * mast_rebalance_prev() - Rebalance against the previous node
 * @mast: The maple subtree state
 * @old_l: The encoded maple node to the left (previous node)
 */
static inline void mast_rebalance_prev(struct maple_subtree_state *mast)
{
	unsigned char end = mas_data_end(mast->orig_l) + 1;
	unsigned char b_end = mast->bn->b_end;

	mab_shift_right(mast->bn, end);
	mas_mab_cp(mast->orig_l, 0, end - 1, mast->bn, 0);
	mast->l->min = mast->orig_l->min;
	mast->orig_l->index = mast->orig_l->min;
	mast->bn->b_end = end + b_end;
	mast->l->offset += end;
}

/*
 * mast_spanning_rebalance() - Rebalance nodes with nearest neighbour favouring
 * the node to the right.  Checking the nodes to the right then the left at each
 * level upwards until root is reached.  Free and destroy as needed.
 * Data is copied into the @mast->bn.
 * @mast: The maple_subtree_state.
 */
static inline
bool mast_spanning_rebalance(struct maple_subtree_state *mast)
{
	struct ma_state r_tmp = *mast->orig_r;
	struct ma_state l_tmp = *mast->orig_l;
	struct maple_enode *ancestor = NULL;
	unsigned char start, end;
	unsigned char depth = 0;

	r_tmp = *mast->orig_r;
	l_tmp = *mast->orig_l;
	do {
		mas_ascend(mast->orig_r);
		mas_ascend(mast->orig_l);
		depth++;
		if (!ancestor &&
		    (mast->orig_r->node == mast->orig_l->node)) {
			ancestor = mast->orig_r->node;
			end = mast->orig_r->offset - 1;
			start = mast->orig_l->offset + 1;
		}

		if (mast->orig_r->offset < mas_data_end(mast->orig_r)) {
			if (!ancestor) {
				ancestor = mast->orig_r->node;
				start = 0;
			}

			mast->orig_r->offset++;
			do {
				mas_descend(mast->orig_r);
				mast->orig_r->offset = 0;
				depth--;
			} while (depth);

			mast_rebalance_next(mast);
			do {
				unsigned char l_off = 0;
				struct maple_enode *child = r_tmp.node;

				mas_ascend(&r_tmp);
				if (ancestor == r_tmp.node)
					l_off = start;

				if (r_tmp.offset)
					r_tmp.offset--;

				if (l_off < r_tmp.offset)
					mas_topiary_range(&r_tmp, mast->destroy,
							  l_off, r_tmp.offset);

				if (l_tmp.node != child)
					mat_add(mast->free, child);

			} while (r_tmp.node != ancestor);

			*mast->orig_l = l_tmp;
			return true;

		} else if (mast->orig_l->offset != 0) {
			if (!ancestor) {
				ancestor = mast->orig_l->node;
				end = mas_data_end(mast->orig_l);
			}

			mast->orig_l->offset--;
			do {
				mas_descend(mast->orig_l);
				mast->orig_l->offset =
					mas_data_end(mast->orig_l);
				depth--;
			} while (depth);

			mast_rebalance_prev(mast);
			do {
				unsigned char r_off;
				struct maple_enode *child = l_tmp.node;

				mas_ascend(&l_tmp);
				if (ancestor == l_tmp.node)
					r_off = end;
				else
					r_off = mas_data_end(&l_tmp);

				if (l_tmp.offset < r_off)
					l_tmp.offset++;

				if (l_tmp.offset < r_off)
					mas_topiary_range(&l_tmp, mast->destroy,
							  l_tmp.offset, r_off);

				if (r_tmp.node != child)
					mat_add(mast->free, child);

			} while (l_tmp.node != ancestor);

			*mast->orig_r = r_tmp;
			return true;
		}
	} while (!mte_is_root(mast->orig_r->node));

	*mast->orig_r = r_tmp;
	*mast->orig_l = l_tmp;
	return false;
}

/*
 * mas_new_ma_node() - Create and return a new maple node.  Helper function.
 * @mas: the maple state with the allocations.
 * @b_node: the maple_big_node with the type encoding.
 *
 * Use the node type from the maple_big_node to allocate a new node from the
 * ma_state.  This function exists mainly for code readability.
 *
 * Return: A new maple encoded node
 */
static inline struct maple_enode
*mas_new_ma_node(struct ma_state *mas, struct maple_big_node *b_node)
{
	return mt_mk_node(ma_mnode_ptr(mas_pop_node(mas)), b_node->type);
}

/*
 * mab_middle_node() - Check if a middle node is needed (unlikely)
 * @b_node: the maple_big_node that contains the data.
 * @size: the amount of data in the b_node
 * @split: the potential split location
 * @slot_count: the size that can be stored in a single node being considered.
 *
 * Return: true if a middle node is required.
 */
static inline bool mab_middle_node(struct maple_big_node *b_node, int split,
				   unsigned char slot_count)
{
	unsigned char size = b_node->b_end;

	if (size >= 2 * slot_count)
		return true;

	if (!b_node->slot[split] && (size >= 2 * slot_count - 1))
		return true;

	return false;
}

/*
 * mab_no_null_split() - ensure the split doesn't fall on a NULL
 * @b_node: the maple_big_node with the data
 * @split: the suggested split location
 * @slot_count: the number of slots in the node being considered.
 *
 * Return: the split location.
 */
static inline int mab_no_null_split(struct maple_big_node *b_node,
				    unsigned char split, unsigned char slot_count)
{
	if (!b_node->slot[split]) {
		/*
		 * If the split is less than the max slot && the right side will
		 * still be sufficient, then increment the split on NULL.
		 */
		if ((split < slot_count - 1) &&
		    (b_node->b_end - split) > (mt_min_slots[b_node->type]))
			split++;
		else
			split--;
	}
	return split;
}

/*
 * mab_calc_split() - Calculate the split location and if there needs to be two
 * splits.
 * @bn: The maple_big_node with the data
 * @mid_split: The second split, if required.  0 otherwise.
 *
 * Return: The first split location.  The middle split is set in @mid_split.
 */
static inline int mab_calc_split(struct ma_state *mas,
	 struct maple_big_node *bn, unsigned char *mid_split, unsigned long min)
{
	unsigned char b_end = bn->b_end;
	int split = b_end / 2; /* Assume equal split. */
	unsigned char slot_min, slot_count = mt_slots[bn->type];

	/*
	 * To support gap tracking, all NULL entries are kept together and a node cannot
	 * end on a NULL entry, with the exception of the left-most leaf.  The
	 * limitation means that the split of a node must be checked for this condition
	 * and be able to put more data in one direction or the other.
	 */
	if (unlikely((mas->mas_flags & MA_STATE_BULK))) {
		*mid_split = 0;
		split = b_end - mt_min_slots[bn->type];

		if (!ma_is_leaf(bn->type))
			return split;

		mas->mas_flags |= MA_STATE_REBALANCE;
		if (!bn->slot[split])
			split--;
		return split;
	}

	/*
	 * Although extremely rare, it is possible to enter what is known as the 3-way
	 * split scenario.  The 3-way split comes about by means of a store of a range
	 * that overwrites the end and beginning of two full nodes.  The result is a set
	 * of entries that cannot be stored in 2 nodes.  Sometimes, these two nodes can
	 * also be located in different parent nodes which are also full.  This can
	 * carry upwards all the way to the root in the worst case.
	 */
	if (unlikely(mab_middle_node(bn, split, slot_count))) {
		split = b_end / 3;
		*mid_split = split * 2;
	} else {
		slot_min = mt_min_slots[bn->type];

		*mid_split = 0;
		/*
		 * Avoid having a range less than the slot count unless it
		 * causes one node to be deficient.
		 * NOTE: mt_min_slots is 1 based, b_end and split are zero.
		 */
		while ((split < slot_count - 1) &&
		       ((bn->pivot[split] - min) < slot_count - 1) &&
		       (b_end - split > slot_min))
			split++;
	}

	/* Avoid ending a node on a NULL entry */
	split = mab_no_null_split(bn, split, slot_count);

	if (unlikely(*mid_split))
		*mid_split = mab_no_null_split(bn, *mid_split, slot_count);

	return split;
}

/*
 * mas_mab_to_node() - Set up right and middle nodes
 *
 * @mas: the maple state that contains the allocations.
 * @b_node: the node which contains the data.
 * @left: The pointer which will have the left node
 * @right: The pointer which may have the right node
 * @middle: the pointer which may have the middle node (rare)
 * @mid_split: the split location for the middle node
 *
 * Return: the split of left.
 */
static inline unsigned char mas_mab_to_node(struct ma_state *mas,
	struct maple_big_node *b_node, struct maple_enode **left,
	struct maple_enode **right, struct maple_enode **middle,
	unsigned char *mid_split, unsigned long min)
{
	unsigned char split = 0;
	unsigned char slot_count = mt_slots[b_node->type];

	*left = mas_new_ma_node(mas, b_node);
	*right = NULL;
	*middle = NULL;
	*mid_split = 0;

	if (b_node->b_end < slot_count) {
		split = b_node->b_end;
	} else {
		split = mab_calc_split(mas, b_node, mid_split, min);
		*right = mas_new_ma_node(mas, b_node);
	}

	if (*mid_split)
		*middle = mas_new_ma_node(mas, b_node);

	return split;

}

/*
 * mas_set_parent() - Set the parent node and encode the slot
 * @enode: The encoded maple node.
 * @parent: The encoded maple node that is the parent of @enode.
 * @slot: The slot that @enode resides in @parent.
 *
 * Slot number is encoded in the enode->parent bit 3-6 or 2-6, depending on the
 * parent type.
 */
static inline
void mas_set_parent(struct ma_state *mas, struct maple_enode *enode,
		    const struct maple_enode *parent, unsigned char slot)
{
	unsigned long val = (unsigned long)parent;
	unsigned long shift;
	unsigned long type;
	enum maple_type p_type = mte_node_type(parent);

	if(p_type == maple_dense || p_type == maple_leaf_64){
		fprintf(stderr, "p_type = %d", p_type);
	}

	// MAS_BUG_ON(mas, p_type == maple_dense);
	// MAS_BUG_ON(mas, p_type == maple_leaf_64);

	switch (p_type) {
	case maple_range_64:
	case maple_arange_64:
		shift = MAPLE_PARENT_SLOT_SHIFT;
		type = MAPLE_PARENT_RANGE64;
		break;
	default:
	case maple_dense:
	case maple_leaf_64:
		shift = type = 0;
		break;
	}

	val &= ~MAPLE_NODE_MASK; /* Clear all node metadata in parent */
	val |= (slot << shift) | type;
	mte_to_node(enode)->parent = ma_parent_ptr(val);
}

/*
 * mas_set_split_parent() - combine_then_separate helper function.  Sets the parent
 * of @mas->node to either @left or @right, depending on @slot and @split
 *
 * @mas - the maple state with the node that needs a parent
 * @left - possible parent 1
 * @right - possible parent 2
 * @slot - the slot the mas->node was placed
 * @split - the split location between @left and @right
 */
static inline void mas_set_split_parent(struct ma_state *mas,
					struct maple_enode *left,
					struct maple_enode *right,
					unsigned char *slot, unsigned char split)
{
	if (mas_is_none(mas))
		return;

	if ((*slot) <= split)
		mas_set_parent(mas, mas->node, left, *slot);
	else if (right)
		mas_set_parent(mas, mas->node, right, (*slot) - split - 1);

	(*slot)++;
}

/*
 * mte_mid_split_check() - Check if the next node passes the mid-split
 * @**l: Pointer to left encoded maple node.
 * @**m: Pointer to middle encoded maple node.
 * @**r: Pointer to right encoded maple node.
 * @slot: The offset
 * @*split: The split location.
 * @mid_split: The middle split.
 */
static inline void mte_mid_split_check(struct maple_enode **l,
				       struct maple_enode **r,
				       struct maple_enode *right,
				       unsigned char slot,
				       unsigned char *split,
				       unsigned char mid_split)
{
	if (*r == right)
		return;

	if (slot < mid_split)
		return;

	*l = *r;
	*r = right;
	*split = mid_split;
}

/*
 * mast_set_split_parents() - Helper function to set three nodes parents.  Slot
 * is taken from @mast->l.
 * @mast - the maple subtree state
 * @left - the left node
 * @right - the right node
 * @split - the split location.
 */
static inline void mast_set_split_parents(struct maple_subtree_state *mast,
					  struct maple_enode *left,
					  struct maple_enode *middle,
					  struct maple_enode *right,
					  unsigned char split,
					  unsigned char mid_split)
{
	unsigned char slot;
	struct maple_enode *l = left;
	struct maple_enode *r = right;

	if (mas_is_none(mast->l))
		return;

	if (middle)
		r = middle;

	slot = mast->l->offset;

	mte_mid_split_check(&l, &r, right, slot, &split, mid_split);
	mas_set_split_parent(mast->l, l, r, &slot, split);

	mte_mid_split_check(&l, &r, right, slot, &split, mid_split);
	mas_set_split_parent(mast->m, l, r, &slot, split);

	mte_mid_split_check(&l, &r, right, slot, &split, mid_split);
	mas_set_split_parent(mast->r, l, r, &slot, split);
}

/*
 * mte_node_or_node() - Return the encoded node or MAS_NONE.
 * @enode: The encoded maple node.
 *
 * Shorthand to avoid setting %NULLs in the tree or maple_subtree_state.
 *
 * Return: @enode or MAS_NONE
 */
static inline struct maple_enode *mte_node_or_none(struct maple_enode *enode)
{
	if (enode)
		return enode;

	return ma_enode_ptr(MAS_NONE);
}

/*
 * mas_leaf_set_meta() - Set the metadata of a leaf if possible.
 * @mas: The maple state
 * @node: The maple node
 * @pivots: pointer to the maple node pivots
 * @mt: The maple type
 * @end: The assumed end
 *
 * Note, end may be incremented within this function but not modified at the
 * source.  This is fine since the metadata is the last thing to be stored in a
 * node during a write.
 */
static inline void mas_leaf_set_meta(struct ma_state *mas,
		struct maple_node *node, unsigned long *pivots,
		enum maple_type mt, unsigned char end)
{
	/* There is no room for metadata already */
	if (mt_pivots[mt] <= end)
		return;

	if (pivots[end] && pivots[end] < mas->max)
		end++;

	if (end < mt_slots[mt] - 1)
		ma_set_meta(node, mt, 0, end);
}

/*
 * mab_mas_cp() - Copy data from maple_big_node to a maple encoded node.
 * @b_node: the maple_big_node that has the data
 * @mab_start: the start location in @b_node.
 * @mab_end: The end location in @b_node (inclusively)
 * @mas: The maple state with the maple encoded node.
 */
static inline void mab_mas_cp(struct maple_big_node *b_node,
			      unsigned char mab_start, unsigned char mab_end,
			      struct ma_state *mas, bool new_max)
{
	int i, j = 0;
	enum maple_type mt = mte_node_type(mas->node);
	struct maple_node *node = mte_to_node(mas->node);
	void __rcu **slots = ma_slots(node, mt);
	unsigned long *pivots = ma_pivots(node, mt);
	unsigned long *gaps = NULL;
	unsigned char end;

	if (mab_end - mab_start > mt_pivots[mt])
		mab_end--;

	if (!pivots[mt_pivots[mt] - 1])
		slots[mt_pivots[mt]] = NULL;

	i = mab_start;
	do {
		pivots[j++] = b_node->pivot[i++];
	} while (i <= mab_end && likely(b_node->pivot[i]));

	memcpy(slots, b_node->slot + mab_start,
	       sizeof(void *) * (i - mab_start));

	if (new_max)
		mas->max = b_node->pivot[i - 1];

	end = j - 1;
	if (likely(!ma_is_leaf(mt) && mt_is_alloc(mas->tree))) {
		unsigned long max_gap = 0;
		unsigned char offset = 15;

		gaps = ma_gaps(node, mt);
		do {
			gaps[--j] = b_node->gap[--i];
			if (gaps[j] > max_gap) {
				offset = j;
				max_gap = gaps[j];
			}
		} while (j);

		ma_set_meta(node, mt, offset, end);
	} else {
		mas_leaf_set_meta(mas, node, pivots, mt, end);
	}
}

/*
 * mast_cp_to_nodes() - Copy data out to nodes.
 * @mast: The maple subtree state
 * @left: The left encoded maple node
 * @middle: The middle encoded maple node
 * @right: The right encoded maple node
 * @split: The location to split between left and (middle ? middle : right)
 * @mid_split: The location to split between middle and right.
 */
static inline void mast_cp_to_nodes(struct maple_subtree_state *mast,
	struct maple_enode *left, struct maple_enode *middle,
	struct maple_enode *right, unsigned char split, unsigned char mid_split)
{
	bool new_lmax = true;

	mast->l->node = mte_node_or_none(left);
	mast->m->node = mte_node_or_none(middle);
	mast->r->node = mte_node_or_none(right);

	mast->l->min = mast->orig_l->min;
	if (split == mast->bn->b_end) {
		mast->l->max = mast->orig_r->max;
		new_lmax = false;
	}

	mab_mas_cp(mast->bn, 0, split, mast->l, new_lmax);

	if (middle) {
		mab_mas_cp(mast->bn, 1 + split, mid_split, mast->m, true);
		mast->m->min = mast->bn->pivot[split] + 1;
		split = mid_split;
	}

	mast->r->max = mast->orig_r->max;
	if (right) {
		mab_mas_cp(mast->bn, 1 + split, mast->bn->b_end, mast->r, false);
		mast->r->min = mast->bn->pivot[split] + 1;
	}
}

/*
 * mast_ascend_free() - Add current original maple state nodes to the free list
 * and ascend.
 * @mast: the maple subtree state.
 *
 * Ascend the original left and right sides and add the previous nodes to the
 * free list.  Set the slots to point to the correct location in the new nodes.
 */
static inline void
mast_ascend_free(struct maple_subtree_state *mast)
{
	MA_WR_STATE(wr_mas, mast->orig_r,  NULL);
	struct maple_enode *left = mast->orig_l->node;
	struct maple_enode *right = mast->orig_r->node;

	mas_ascend(mast->orig_l);
	mas_ascend(mast->orig_r);
	mat_add(mast->free, left);

	if (left != right)
		mat_add(mast->free, right);

	mast->orig_r->offset = 0;
	mast->orig_r->index = mast->r->max;
	/* last should be larger than or equal to index */
	if (mast->orig_r->last < mast->orig_r->index)
		mast->orig_r->last = mast->orig_r->index;
	/*
	 * The node may not contain the value so set slot to ensure all
	 * of the nodes contents are freed or destroyed.
	 */
	wr_mas.type = mte_node_type(mast->orig_r->node);
	mas_wr_node_walk(&wr_mas);
	/* Set up the left side of things */
	mast->orig_l->offset = 0;
	mast->orig_l->index = mast->l->min;
	wr_mas.mas = mast->orig_l;
	wr_mas.type = mte_node_type(mast->orig_l->node);
	mas_wr_node_walk(&wr_mas);

	mast->bn->type = wr_mas.type;
}

/*
 * mast_combine_cp_left - Copy in the original left side of the tree into the
 * combined data set in the maple subtree state big node.
 * @mast: The maple subtree state
 */
static inline void mast_combine_cp_left(struct maple_subtree_state *mast)
{
	unsigned char l_slot = mast->orig_l->offset;

	if (!l_slot)
		return;

	mas_mab_cp(mast->orig_l, 0, l_slot - 1, mast->bn, 0);
}

/*
 * mas_leaf_max_gap() - Returns the largest gap in a leaf node
 * @mas - the maple state
 *
 * Return: The maximum gap in the leaf.
 */
static unsigned long mas_leaf_max_gap(struct ma_state *mas)
{
	enum maple_type mt;
	unsigned long pstart, gap, max_gap;
	struct maple_node *mn;
	unsigned long *pivots;
	void __rcu **slots;
	unsigned char i;
	unsigned char max_piv;

	mt = mte_node_type(mas->node);
	mn = mas_mn(mas);
	slots = ma_slots(mn, mt);
	max_gap = 0;
	if (unlikely(ma_is_dense(mt))) {
		gap = 0;
		for (i = 0; i < mt_slots[mt]; i++) {
			if (slots[i]) {
				if (gap > max_gap)
					max_gap = gap;
				gap = 0;
			} else {
				gap++;
			}
		}
		if (gap > max_gap)
			max_gap = gap;
		return max_gap;
	}

	/*
	 * Check the first implied pivot optimizes the loop below and slot 1 may
	 * be skipped if there is a gap in slot 0.
	 */
	pivots = ma_pivots(mn, mt);
	if (likely(!slots[0])) {
		max_gap = pivots[0] - mas->min + 1;
		i = 2;
	} else {
		i = 1;
	}

	/* reduce max_piv as the special case is checked before the loop */
	max_piv = ma_data_end(mn, mt, pivots, mas->max) - 1;
	/*
	 * Check end implied pivot which can only be a gap on the right most
	 * node.
	 */
	if (unlikely(mas->max == ULONG_MAX) && !slots[max_piv + 1]) {
		gap = ULONG_MAX - pivots[max_piv];
		if (gap > max_gap)
			max_gap = gap;
	}

	for (; i <= max_piv; i++) {
		/* data == no gap. */
		if (slots[i])
			continue;

		pstart = pivots[i - 1];
		gap = pivots[i] - pstart;
		if (gap > max_gap)
			max_gap = gap;

		/* There cannot be two gaps in a row. */
		i++;
	}
	return max_gap;
}

/*
 * ma_meta_gap() - Get the largest gap location of a node from the metadata
 * @mn: The maple node
 * @mt: The maple node type
 */
static inline unsigned char ma_meta_gap(struct maple_node *mn,
					enum maple_type mt)
{
	return mn->ma64.meta.gap;
}

/*
 * mas_max_gap() - find the largest gap in a non-leaf node and set the slot.
 * @mas: The maple state.
 *
 * If the metadata gap is set to MAPLE_ARANGE64_META_MAX, there is no gap.
 *
 * Return: The gap value.
 */
static inline unsigned long mas_max_gap(struct ma_state *mas)
{
	unsigned long *gaps;
	unsigned char offset;
	enum maple_type mt;
	struct maple_node *node;

	mt = mte_node_type(mas->node);
	if (ma_is_leaf(mt))
		return mas_leaf_max_gap(mas);

	node = mas_mn(mas);
	if(mt != maple_arange_64){
		fprintf(stderr, "mt != maple_arange_64");
	}
	
	// MAS_BUG_ON(mas, mt != maple_arange_64);
	offset = ma_meta_gap(node, mt);
	if (offset == MAPLE_ARANGE64_META_MAX)
		return 0;

	gaps = ma_gaps(node, mt);
	return gaps[offset];
}

/*
 * mab_set_b_end() - Add entry to b_node at b_node->b_end and increment the end
 * pointer.
 * @b_node - the big node to add the entry
 * @mas - the maple state to get the pivot (mas->max)
 * @entry - the entry to add, if NULL nothing happens.
 */
static inline void mab_set_b_end(struct maple_big_node *b_node,
				 struct ma_state *mas,
				 void *entry)
{
	if (!entry)
		return;

	b_node->slot[b_node->b_end] = entry;
	if (mt_is_alloc(mas->tree))
		b_node->gap[b_node->b_end] = mas_max_gap(mas);
	b_node->pivot[b_node->b_end++] = mas->max;
}

/*
 * mast_combine_cp_right: Copy in the original right side of the tree into the
 * combined data set in the maple subtree state big node.
 * @mast: The maple subtree state
 */
static inline void mast_combine_cp_right(struct maple_subtree_state *mast)
{
	if (mast->bn->pivot[mast->bn->b_end - 1] >= mast->orig_r->max)
		return;

	mas_mab_cp(mast->orig_r, mast->orig_r->offset + 1,
		   mt_slot_count(mast->orig_r->node), mast->bn,
		   mast->bn->b_end);
	mast->orig_r->last = mast->orig_r->max;
}

/*
 * mast_topiary() - Add the portions of the tree to the removal list; either to
 * be freed or discarded (destroy walk).
 * @mast: The maple_subtree_state.
 */
static inline void mast_topiary(struct maple_subtree_state *mast)
{
	MA_WR_STATE(wr_mas, mast->orig_l, NULL);
	unsigned char r_start, r_end;
	unsigned char l_start, l_end;
	void __rcu **l_slots, **r_slots;

	wr_mas.type = mte_node_type(mast->orig_l->node);
	mast->orig_l->index = mast->orig_l->last;
	mas_wr_node_walk(&wr_mas);
	l_start = mast->orig_l->offset + 1;
	l_end = mas_data_end(mast->orig_l);
	r_start = 0;
	r_end = mast->orig_r->offset;

	if (r_end)
		r_end--;

	l_slots = ma_slots(mas_mn(mast->orig_l),
			   mte_node_type(mast->orig_l->node));

	r_slots = ma_slots(mas_mn(mast->orig_r),
			   mte_node_type(mast->orig_r->node));

	if ((l_start < l_end) &&
	    mte_dead_node(mas_slot_locked(mast->orig_l, l_slots, l_start))) {
		l_start++;
	}

	if (mte_dead_node(mas_slot_locked(mast->orig_r, r_slots, r_end))) {
		if (r_end)
			r_end--;
	}

	if ((l_start > r_end) && (mast->orig_l->node == mast->orig_r->node))
		return;

	/* At the node where left and right sides meet, add the parts between */
	if (mast->orig_l->node == mast->orig_r->node) {
		return mas_topiary_range(mast->orig_l, mast->destroy,
					     l_start, r_end);
	}

	/* mast->orig_r is different and consumed. */
	if (mte_is_leaf(mast->orig_r->node))
		return;

	if (mte_dead_node(mas_slot_locked(mast->orig_l, l_slots, l_end)))
		l_end--;


	if (l_start <= l_end)
		mas_topiary_range(mast->orig_l, mast->destroy, l_start, l_end);

	if (mte_dead_node(mas_slot_locked(mast->orig_r, r_slots, r_start)))
		r_start++;

	if (r_start <= r_end)
		mas_topiary_range(mast->orig_r, mast->destroy, 0, r_end);
}

/*
 * mast_sufficient: Check if the maple subtree state has enough data in the big
 * node to create at least one sufficient node
 * @mast: the maple subtree state
 */
static inline bool mast_sufficient(struct maple_subtree_state *mast)
{
	if (mast->bn->b_end > mt_min_slot_count(mast->orig_l->node))
		return true;

	return false;
}

/*
 * mast_overflow: Check if there is too much data in the subtree state for a
 * single node.
 * @mast: The maple subtree state
 */
static inline bool mast_overflow(struct maple_subtree_state *mast)
{
	if (mast->bn->b_end >= mt_slot_count(mast->orig_l->node))
		return true;

	return false;
}

/*
 * mast_new_root() - Set a new tree root during subtree creation
 * @mast: The maple subtree state
 * @mas: The maple state
 */
static inline void mast_new_root(struct maple_subtree_state *mast,
				 struct ma_state *mas)
{
	mas_mn(mast->l)->parent =
		ma_parent_ptr(((unsigned long)mas->tree | MA_ROOT_PARENT));
	if (!mte_dead_node(mast->orig_l->node) &&
	    !mte_is_root(mast->orig_l->node)) {
		do {
			mast_ascend_free(mast);
			mast_topiary(mast);
		} while (!mte_is_root(mast->orig_l->node));
	}
	if ((mast->orig_l->node != mas->node) &&
		   (mast->l->depth > mas_mt_height(mas))) {
		mat_add(mast->free, mas->node);
	}
}

/*
 * mas_adopt_children() - Set the parent pointer of all nodes in @parent to
 * @parent with the slot encoded.
 * @mas - the maple state (for the tree)
 * @parent - the maple encoded node containing the children.
 */
static inline void mas_adopt_children(struct ma_state *mas,
		struct maple_enode *parent)
{
	enum maple_type type = mte_node_type(parent);
	struct maple_node *node = mas_mn(mas);
	void __rcu **slots = ma_slots(node, type);
	unsigned long *pivots = ma_pivots(node, type);
	struct maple_enode *child;
	unsigned char offset;

	offset = ma_data_end(node, type, pivots, mas->max);
	do {
		child = mas_slot_locked(mas, slots, offset);
		mas_set_parent(mas, child, parent, offset);
	} while (offset--);
}

/*
 * mas_free() - Free an encoded maple node
 * @mas: The maple state
 * @used: The encoded maple node to free.
 *
 * Uses rcu free if necessary, pushes @used back on the maple state allocations
 * otherwise.
 */
static inline void mas_free(struct ma_state *mas, struct maple_enode *used)
{
	struct maple_node *tmp = mte_to_node(used);

	if (mt_in_rcu(mas->tree))
		ma_free_rcu(tmp);
	else
		mas_push_node(mas, tmp);
}


/*
 * mas_replace() - Replace a maple node in the tree with mas->node.  Uses the
 * parent encoding to locate the maple node in the tree.
 * @mas - the ma_state to use for operations.
 * @advanced - boolean to adopt the child nodes and free the old node (false) or
 * leave the node (true) and handle the adoption and free elsewhere.
 */
static inline void mas_replace(struct ma_state *mas, bool advanced)
	// __must_hold(mas->tree->ma_lock)
{
	struct maple_node *mn = mas_mn(mas);
	struct maple_enode *old_enode;
	unsigned char offset = 0;
	void __rcu **slots = NULL;

	if (ma_is_root(mn)) {
		old_enode = mas_root_locked(mas);
	} else {
		offset = mte_parent_slot(mas->node);
		slots = ma_slots(mte_parent(mas->node),
				 mas_parent_type(mas, mas->node));
		old_enode = mas_slot_locked(mas, slots, offset);
	}

	if (!advanced && !mte_is_leaf(mas->node))
		mas_adopt_children(mas, mas->node);

	if (mte_is_root(mas->node)) {
		mn->parent = ma_parent_ptr(
			      ((unsigned long)mas->tree | MA_ROOT_PARENT));
		rcu_assign_pointer(mas->tree->ma_root, mte_mk_root(mas->node));
		mas_set_height(mas);
	} else {
		rcu_assign_pointer(slots[offset], mas->node);
	}

	if (!advanced) {
		mte_set_node_dead(old_enode);
		mas_free(mas, old_enode);
	}
}

/*
 * mas_wmb_replace() - Write memory barrier and replace
 * @mas: The maple state
 * @free: the maple topiary list of nodes to free
 * @destroy: The maple topiary list of nodes to destroy (walk and free)
 *
 * Updates gap as necessary.
 */
static inline void mas_wmb_replace(struct ma_state *mas,
				   struct ma_topiary *free,
				   struct ma_topiary *destroy)
{
	/* All nodes must see old data as dead prior to replacing that data */
	atomic_thread_fence(memory_order_release);
	// smp_wmb(); /* Needed for RCU */

	/* Insert the new data in the tree */
	mas_replace(mas, true);

	if (!mte_is_leaf(mas->node))
		mas_descend_adopt(mas);

	mas_mat_free(mas, free);

	if (destroy)
		mas_mat_destroy(mas, destroy);

	if (mte_is_leaf(mas->node))
		return;

	mas_update_gap(mas);
}

/*
 * mas_spanning_rebalance() - Rebalance across two nodes which may not be peers.
 * @mas: The starting maple state
 * @mast: The maple_subtree_state, keeps track of 4 maple states.
 * @count: The estimated count of iterations needed.
 *
 * Follow the tree upwards from @l_mas and @r_mas for @count, or until the root
 * is hit.  First @b_node is split into two entries which are inserted into the
 * next iteration of the loop.  @b_node is returned populated with the final
 * iteration. @mas is used to obtain allocations.  orig_l_mas keeps track of the
 * nodes that will remain active by using orig_l_mas->index and orig_l_mas->last
 * to account of what has been copied into the new sub-tree.  The update of
 * orig_l_mas->last is used in mas_consume to find the slots that will need to
 * be either freed or destroyed.  orig_l_mas->depth keeps track of the height of
 * the new sub-tree in case the sub-tree becomes the full tree.
 *
 * Return: the number of elements in b_node during the last loop.
 */
static int mas_spanning_rebalance(struct ma_state *mas,
		struct maple_subtree_state *mast, unsigned char count)
{
	unsigned char split, mid_split;
	unsigned char slot = 0;
	struct maple_enode *left = NULL, *middle = NULL, *right = NULL;

	MA_STATE(l_mas, mas->tree, mas->index, mas->index);
	MA_STATE(r_mas, mas->tree, mas->index, mas->last);
	MA_STATE(m_mas, mas->tree, mas->index, mas->index);
	MA_TOPIARY(free, mas->tree);
	MA_TOPIARY(destroy, mas->tree);

	/*
	 * The tree needs to be rebalanced and leaves need to be kept at the same level.
	 * Rebalancing is done by use of the ``struct maple_topiary``.
	 */
	mast->l = &l_mas;
	mast->m = &m_mas;
	mast->r = &r_mas;
	mast->free = &free;
	mast->destroy = &destroy;
	l_mas.node = r_mas.node = m_mas.node = MAS_NONE;

	/* Check if this is not root and has sufficient data.  */
	if (((mast->orig_l->min != 0) || (mast->orig_r->max != ULONG_MAX)) &&
	    unlikely(mast->bn->b_end <= mt_min_slots[mast->bn->type]))
		mast_spanning_rebalance(mast);

	mast->orig_l->depth = 0;

	/*
	 * Each level of the tree is examined and balanced, pushing data to the left or
	 * right, or rebalancing against left or right nodes is employed to avoid
	 * rippling up the tree to limit the amount of churn.  Once a new sub-section of
	 * the tree is created, there may be a mix of new and old nodes.  The old nodes
	 * will have the incorrect parent pointers and currently be in two trees: the
	 * original tree and the partially new tree.  To remedy the parent pointers in
	 * the old tree, the new data is swapped into the active tree and a walk down
	 * the tree is performed and the parent pointers are updated.
	 * See mas_descend_adopt() for more information..
	 */
	while (count--) {
		mast->bn->b_end--;
		mast->bn->type = mte_node_type(mast->orig_l->node);
		split = mas_mab_to_node(mas, mast->bn, &left, &right, &middle,
					&mid_split, mast->orig_l->min);
		mast_set_split_parents(mast, left, middle, right, split,
				       mid_split);
		mast_cp_to_nodes(mast, left, middle, right, split, mid_split);

		/*
		 * Copy data from next level in the tree to mast->bn from next
		 * iteration
		 */
		memset(mast->bn, 0, sizeof(struct maple_big_node));
		mast->bn->type = mte_node_type(left);
		mast->orig_l->depth++;

		/* Root already stored in l->node. */
		if (mas_is_root_limits(mast->l))
			goto new_root;

		mast_ascend_free(mast);
		mast_combine_cp_left(mast);
		l_mas.offset = mast->bn->b_end;
		mab_set_b_end(mast->bn, &l_mas, left);
		mab_set_b_end(mast->bn, &m_mas, middle);
		mab_set_b_end(mast->bn, &r_mas, right);

		/* Copy anything necessary out of the right node. */
		mast_combine_cp_right(mast);
		mast_topiary(mast);
		mast->orig_l->last = mast->orig_l->max;

		if (mast_sufficient(mast))
			continue;

		if (mast_overflow(mast))
			continue;

		/* May be a new root stored in mast->bn */
		if (mas_is_root_limits(mast->orig_l))
			break;

		mast_spanning_rebalance(mast);

		/* rebalancing from other nodes may require another loop. */
		if (!count)
			count++;
	}

	l_mas.node = mt_mk_node(ma_mnode_ptr(mas_pop_node(mas)),
				mte_node_type(mast->orig_l->node));
	mast->orig_l->depth++;
	mab_mas_cp(mast->bn, 0, mt_slots[mast->bn->type] - 1, &l_mas, true);
	mas_set_parent(mas, left, l_mas.node, slot);
	if (middle)
		mas_set_parent(mas, middle, l_mas.node, ++slot);

	if (right)
		mas_set_parent(mas, right, l_mas.node, ++slot);

	if (mas_is_root_limits(mast->l)) {
new_root:
		mast_new_root(mast, mas);
	} else {
		mas_mn(&l_mas)->parent = mas_mn(mast->orig_l)->parent;
	}

	if (!mte_dead_node(mast->orig_l->node))
		mat_add(&free, mast->orig_l->node);

	mas->depth = mast->orig_l->depth;
	*mast->orig_l = l_mas;
	mte_set_node_dead(mas->node);

	/* Set up mas for insertion. */
	mast->orig_l->depth = mas->depth;
	mast->orig_l->alloc = mas->alloc;
	*mas = *mast->orig_l;
	mas_wmb_replace(mas, &free, &destroy);
	mtree_range_walk(mas);
	return mast->bn->b_end;
}

/*
 * mas_wr_spanning_store() - Create a subtree with the store operation completed
 * and new nodes where necessary, then place the sub-tree in the actual tree.
 * Note that mas is expected to point to the node which caused the store to
 * span.
 * @wr_mas: The maple write state
 *
 * Return: 0 on error, positive on success.
 */
static inline int mas_wr_spanning_store(struct ma_wr_state *wr_mas)
{
	struct maple_subtree_state mast;
	struct maple_big_node b_node;
	struct ma_state *mas;
	unsigned char height;

	/* Left and Right side of spanning store */
	MA_STATE(l_mas, NULL, 0, 0);
	MA_STATE(r_mas, NULL, 0, 0);

	MA_WR_STATE(r_wr_mas, &r_mas, wr_mas->entry);
	MA_WR_STATE(l_wr_mas, &l_mas, wr_mas->entry);

	/*
	 * A store operation that spans multiple nodes is called a spanning
	 * store and is handled early in the store call stack by the function
	 * mas_is_span_wr().  When a spanning store is identified, the maple
	 * state is duplicated.  The first maple state walks the left tree path
	 * to ``index``, the duplicate walks the right tree path to ``last``.
	 * The data in the two nodes are combined into a single node, two nodes,
	 * or possibly three nodes (see the 3-way split above).  A ``NULL``
	 * written to the last entry of a node is considered a spanning store as
	 * a rebalance is required for the operation to complete and an overflow
	 * of data may happen.
	 */
	mas = wr_mas->mas;
	// trace_ma_op(__func__, mas);

	if (unlikely(!mas->index && mas->last == ULONG_MAX))
		return mas_new_root(mas, wr_mas->entry);
	/*
	 * Node rebalancing may occur due to this store, so there may be three new
	 * entries per level plus a new root.
	 */
	height = mas_mt_height(mas);
	mas_node_count(mas, 1 + height * 3);
	if (mas_is_err(mas))
		return 0;

	/*
	 * Set up right side.  Need to get to the next offset after the spanning
	 * store to ensure it's not NULL and to combine both the next node and
	 * the node with the start together.
	 */
	r_mas = *mas;
	/* Avoid overflow, walk to next slot in the tree. */
	if (r_mas.last + 1)
		r_mas.last++;

	r_mas.index = r_mas.last;
	mas_wr_walk_index(&r_wr_mas);
	r_mas.last = r_mas.index = mas->last;

	/* Set up left side. */
	l_mas = *mas;
	mas_wr_walk_index(&l_wr_mas);

	if (!wr_mas->entry) {
		mas_extend_spanning_null(&l_wr_mas, &r_wr_mas);
		mas->offset = l_mas.offset;
		mas->index = l_mas.index;
		mas->last = l_mas.last = r_mas.last;
	}

	/* expanding NULLs may make this cover the entire range */
	if (!l_mas.index && r_mas.last == ULONG_MAX) {
		mas_set_range(mas, 0, ULONG_MAX);
		return mas_new_root(mas, wr_mas->entry);
	}

	memset(&b_node, 0, sizeof(struct maple_big_node));
	/* Copy l_mas and store the value in b_node. */
	mas_store_b_node(&l_wr_mas, &b_node, l_wr_mas.node_end);
	/* Copy r_mas into b_node. */
	if (r_mas.offset <= r_wr_mas.node_end)
		mas_mab_cp(&r_mas, r_mas.offset, r_wr_mas.node_end,
			   &b_node, b_node.b_end + 1);
	else
		b_node.b_end++;

	/* Stop spanning searches by searching for just index. */
	l_mas.index = l_mas.last = mas->index;

	mast.bn = &b_node;
	mast.orig_l = &l_mas;
	mast.orig_r = &r_mas;
	/* Combine l_mas and r_mas and split them up evenly again. */
	return mas_spanning_rebalance(mas, &mast, height + 1);
}


/*
 * mas_wr_store_entry() - Internal call to store a value
 * @mas: The maple state
 * @entry: The entry to store.
 *
 * Return: The contents that was stored at the index.
 */
static inline void *mas_wr_store_entry(struct ma_wr_state *wr_mas)
{
	struct ma_state *mas = wr_mas->mas;

	wr_mas->content = mas_start(mas);
	if (mas_is_none(mas) || mas_is_ptr(mas)) {
		mas_store_root(mas, wr_mas->entry);
		return wr_mas->content;
	}

	if (unlikely(!mas_wr_walk(wr_mas))) {
		mas_wr_spanning_store(wr_mas);
		return wr_mas->content;
	}

	/* At this point, we are at the leaf node that needs to be altered. */
	mas_wr_end_piv(wr_mas);

	if (!wr_mas->entry)
		mas_wr_extend_null(wr_mas);

	/* New root for a single pointer */
	if (unlikely(!mas->index && mas->last == ULONG_MAX)) {
		mas_new_root(mas, wr_mas->entry);
		return wr_mas->content;
	}

	mas_wr_modify(wr_mas);
	return wr_mas->content;
}

/*
 * mas_start() - Sets up maple state for operations.
 * @mas: The maple state.
 *
 * If mas->node == MAS_START, then set the min, max and depth to
 * defaults.
 *
 * Return:
 * - If mas->node is an error or not MAS_START, return NULL.
 * - If it's an empty tree:     NULL & mas->node == MAS_NONE
 * - If it's a single entry:    The entry & mas->node == MAS_ROOT
 * - If it's a tree:            NULL & mas->node == safe root node.
 */
static inline struct maple_enode *mas_start(struct ma_state *mas)
{
	if (likely(mas_is_start(mas))) {
		struct maple_enode *root;

		mas->min = 0;
		mas->max = ULONG_MAX;

retry:
		mas->depth = 0;
		root = mas_root(mas);
		/* Tree with nodes */
		// if (likely(is_node(root))) {
		if (root){
			mas->depth = 1;
			mas->node = mte_safe_root(root);
			mas->offset = 0;
			if (mte_dead_node(mas->node))
				goto retry;

			return NULL;
		}

		/* empty tree */
		if (unlikely(!root)) {
			mas->node = MAS_NONE;
			mas->offset = MAPLE_NODE_SLOTS;
			return NULL;
		}

		/* Single entry tree */
		mas->node = MAS_ROOT;
		mas->offset = MAPLE_NODE_SLOTS;

		/* Single entry tree. */
		if (mas->index > 0)
			return NULL;

		return root;
	}

	return NULL;
}

static inline bool mas_is_start(const struct ma_state *mas)
{
	return mas->node == MAS_START;
}

/*
 * mte_dead_node() - check if the @enode is dead.
 * @enode: The encoded maple node
 *
 * Return: true if dead, false otherwise.
 */
static inline bool mte_dead_node(const struct maple_enode *enode)
{
	struct maple_node *parent, *node;

	node = mte_to_node(enode);
	/* Do not reorder reads from the node prior to the parent check */
	//smp_rmb();
	asm volatile("mfence" ::: "memory");
	parent = mte_parent(enode);
	return (parent == node);
}

/*
 * mte_parent() - Get the parent of @node.
 * @node: The encoded maple node.
 *
 * Return: The parent maple node.
 */
static inline struct maple_node *mte_parent(const struct maple_enode *enode)
{
	return (void *)((unsigned long)
			(mte_to_node(enode)->parent) & ~MAPLE_NODE_MASK);
}

static inline void mas_store_root(struct ma_state *mas, void *entry)
{
	if (likely((mas->last != 0) || (mas->index != 0)))
		mas_root_expand(mas, entry);
	else if (((unsigned long) (entry) & 3) == 2)
		mas_root_expand(mas, entry);
	else {
		rcu_assign_pointer(mas->tree->ma_root, entry);
		mas->node = MAS_START;
	}
}

/*
 * mas_root_expand() - Expand a root to a node
 * @mas: The maple state
 * @entry: The entry to store into the tree
 */
static inline int mas_root_expand(struct ma_state *mas, void *entry)
{
	void *contents = mas_root_locked(mas);
	enum maple_type type = maple_leaf_64;
	struct maple_node *node;
	void __rcu **slots;
	unsigned long *pivots;
	int slot = 0;

	mas_node_count(mas, 1);
	if (unlikely(mas_is_err(mas)))
		return 0;

	node = mas_pop_node(mas);
	pivots = ma_pivots(node, type);
	slots = ma_slots(node, type);
	node->parent = ma_parent_ptr(
		      ((unsigned long)mas->tree | MA_ROOT_PARENT));
	mas->node = mt_mk_node(node, type);

	if (mas->index) {
		if (contents) {
			rcu_assign_pointer(slots[slot], contents);
			if (likely(mas->index > 1))
				slot++;
		}
		pivots[slot++] = mas->index - 1;
	}

	rcu_assign_pointer(slots[slot], entry);
	mas->offset = slot;
	pivots[slot] = mas->last;
	if (mas->last != ULONG_MAX)
		pivots[++slot] = ULONG_MAX;

	mas->depth = 1;
	mas_set_height(mas);
	ma_set_meta(node, maple_leaf_64, 0, slot);
	/* swap the new root into the tree */
	rcu_assign_pointer(mas->tree->ma_root, mte_mk_root(mas->node));
	return slot;
}

static inline struct maple_metadata *ma_meta(struct maple_node *mn,
					     enum maple_type mt)
{
	switch (mt) {
	case maple_arange_64:
		return &mn->ma64.meta;
	default:
		return &mn->mr64.meta;
	}
}

/*
 * ma_set_meta() - Set the metadata information of a node.
 * @mn: The maple node
 * @mt: The maple node type
 * @offset: The offset of the highest sub-gap in this node.
 * @end: The end of the data in this node.
 */
static inline void ma_set_meta(struct maple_node *mn, enum maple_type mt,
			       unsigned char offset, unsigned char end)
{
	struct maple_metadata *meta = ma_meta(mn, mt);

	meta->gap = offset;
	meta->end = end;
}

static void mas_set_height(struct ma_state *mas)
{
	unsigned int new_flags = mas->tree->ma_flags;

	new_flags &= ~MT_FLAGS_HEIGHT_MASK;
	if(mas->depth > MAPLE_HEIGHT_MAX){
		fprintf(stderr, "depth > MAPLE_HEIGHT_MAX");
	}
	new_flags |= mas->depth << MT_FLAGS_HEIGHT_OFFSET;
	mas->tree->ma_flags = new_flags;
}

/*
 * ma_slots() - Get a pointer to the maple node slots.
 * @mn: The maple node
 * @mt: The maple node type
 *
 * Return: A pointer to the maple node slots
 */
static inline void __rcu **ma_slots(struct maple_node *mn, enum maple_type mt)
{
	switch (mt) {
	default:
	case maple_arange_64:
		return mn->ma64.slot;
	case maple_range_64:
	case maple_leaf_64:
		return mn->mr64.slot;
	case maple_dense:
		return mn->slot;
	}
}

/*
 * ma_pivots() - Get a pointer to the maple node pivots.
 * @node - the maple node
 * @type - the node type
 *
 * In the event of a dead node, this array may be %NULL
 *
 * Return: A pointer to the maple node pivots
 */
static inline unsigned long *ma_pivots(struct maple_node *node,
					   enum maple_type type)
{
	switch (type) {
	case maple_arange_64:
		return node->ma64.pivot;
	case maple_range_64:
	case maple_leaf_64:
		return node->mr64.pivot;
	case maple_dense:
		return NULL;
	}
	return NULL;
}


/*
 * mas_alloc_req() - get the requested number of allocations.
 * @mas: The maple state
 *
 * The alloc count is either stored directly in @mas, or in
 * @mas->alloc->request_count if there is at least one node allocated.  Decode
 * the request count if it's stored directly in @mas->alloc.
 *
 * Return: The allocation request count.
 */
static inline unsigned int mas_alloc_req(const struct ma_state *mas)
{
	if ((unsigned long)mas->alloc & 0x1)
		return (unsigned long)(mas->alloc) >> 1;
	else if (mas->alloc)
		return mas->alloc->request_count;
	return 0;
}

/*
 * mas_set_alloc_req() - Set the requested number of allocations.
 * @mas: the maple state
 * @count: the number of allocations.
 *
 * The requested number of allocations is either in the first allocated node,
 * located in @mas->alloc->request_count, or directly in @mas->alloc if there is
 * no allocated node.  Set the request either in the node or do the necessary
 * encoding to store in @mas->alloc directly.
 */
static inline void mas_set_alloc_req(struct ma_state *mas, unsigned long count)
{
	if (!mas->alloc || ((unsigned long)mas->alloc & 0x1)) {
		if (!count)
			mas->alloc = NULL;
		else
			mas->alloc = (struct maple_alloc *)(((count) << 1U) | 1U);
		return;
	}

	mas->alloc->request_count = count;
}

/*
 * mas_pop_node() - Get a previously allocated maple node from the maple state.
 * @mas: The maple state
 *
 * Return: A pointer to a maple node.
 */
static inline struct maple_node *mas_pop_node(struct ma_state *mas)
{
	struct maple_alloc *ret, *node = mas->alloc;
	unsigned long total = mas_allocated(mas);
	unsigned int req = mas_alloc_req(mas);

	/* nothing or a request pending. */
	if (!total){
		fprintf(stderr, "nothing or a request pending");
		return NULL;
	}
		

	if (total == 1) {
		/* single allocation in this ma_state */
		mas->alloc = NULL;
		ret = node;
		goto single_node;
	}

	if (node->node_count == 1) {
		/* Single allocation in this node. */
		mas->alloc = node->slot[0];
		mas->alloc->total = node->total - 1;
		ret = node;
		goto new_head;
	}
	node->total--;
	ret = node->slot[--node->node_count];
	node->slot[node->node_count] = NULL;

single_node:
new_head:
	if (req) {
		req++;
		mas_set_alloc_req(mas, req);
	}

	memset(ret, 0, sizeof(*ret));
	return (struct maple_node *)ret;
}

/*
 * mas_allocated() - Get the number of nodes allocated in a maple state.
 * @mas: The maple state
 *
 * The ma_state alloc member is overloaded to hold a pointer to the first
 * allocated node or to the number of requested nodes to allocate.  If bit 0 is
 * set, then the alloc contains the number of requested nodes.  If there is an
 * allocated node, then the total allocated nodes is in that node.
 *
 * Return: The total number of nodes allocated
 */
static inline unsigned long mas_allocated(const struct ma_state *mas)
{
	if (!mas->alloc || ((unsigned long)mas->alloc & 0x1))
		return 0;

	return mas->alloc->total;
}

/*
 * mas_alloc_nodes() - Allocate nodes into a maple state
 * @mas: The maple state
 * @gfp: The GFP Flags
 */
static inline void mas_alloc_nodes(struct ma_state *mas)
{
	struct maple_alloc *node;
	unsigned long allocated = mas_allocated(mas);
	unsigned int requested = mas_alloc_req(mas);
	unsigned int count;
	void **slots = NULL;
	unsigned int max_req = 0;

	if (!requested)
		return;

	mas_set_alloc_req(mas, 0);
	if (mas->mas_flags & MA_STATE_PREALLOC) {
		if (allocated){
			return;
		} else {
			fprintf(stderr, "mas not allocated");
		}
	}

	if (!allocated || mas->alloc->node_count == MAPLE_ALLOC_SLOTS) {
		node = (struct maple_alloc *)mt_alloc_one();
		if (!node)
			goto nomem_one;

		if (allocated) {
			node->slot[0] = mas->alloc;
			node->node_count = 1;
		} else {
			node->node_count = 0;
		}

		mas->alloc = node;
		node->total = ++allocated;
		requested--;
	}

	node = mas->alloc;
	node->request_count = 0;
	while (requested) {
		max_req = MAPLE_ALLOC_SLOTS - node->node_count;
		slots = (void **)&node->slot[node->node_count];
		max_req = min(requested, max_req);
		count = mt_alloc_bulk(max_req, slots);
		if (!count)
			goto nomem_bulk;

		if (node->node_count == 0) {
			node->slot[0]->node_count = 0;
			node->slot[0]->request_count = 0;
		}

		node->node_count += count;
		allocated += count;
		node = node->slot[0];
		requested -= count;
	}
	mas->alloc->total = allocated;
	return;

nomem_bulk:
	/* Clean up potential freed allocations on bulk failure */
	memset(slots, 0, max_req * sizeof(unsigned long));
nomem_one:
	mas_set_alloc_req(mas, requested);
	if (mas->alloc && !(((unsigned long)mas->alloc & 0x1)))
		mas->alloc->total = allocated;
	mas_set_err(mas, -ENOMEM);
}

bool mas_is_err(struct ma_state *mas)
{
	// TODO: replace xrray
	return false;
}

/**
 * mas_nomem() - Check if there was an error allocating and do the allocation
 * if necessary If there are allocations, then free them.
 * @mas: The maple state
 * @gfp: The GFP_FLAGS to use for allocations
 * Return: true on allocation, false otherwise.
 */
bool mas_nomem(struct ma_state *mas)
	__must_hold(mas->tree->ma_lock)
{
	if (likely(mas->node != MA_ERROR(-ENOMEM))) {
		mas_destroy(mas);
		return false;
	}

	mtree_unlock(mas->tree);
	mas_alloc_nodes(mas);
	mtree_lock(mas->tree);
	
	if (!mas_allocated(mas))
		return false;

	mas->node = MAS_START;
	return true;
}

/*
 * mas_root() - Get the maple tree root.
 * @mas: The maple state.
 *
 * Return: The pointer to the root of the tree
 */
static inline void *mas_root(struct ma_state *mas)
{
	return rcu_dereference(mas->tree->ma_root);
	// return rcu_dereference_check(mas->tree->ma_root, mt_locked(mas->tree));
}
// static inline bool mt_locked(const struct maple_tree *mt)
// {
// 	return lockdep_is_held(&mt->ma_lock);
// }

/* Checks if a mas has not found anything */
static inline bool mas_is_none(const struct ma_state *mas)
{
	return mas->node == MAS_NONE;
}

static inline bool mas_is_ptr(const struct ma_state *mas)
{
	return mas->node == MAS_ROOT;
}





/*
 * mas_root_locked() - Get the maple tree root when holding the maple tree lock.
 * @mas: The maple state.
 *
 * Return: The pointer to the root of the tree
 */
static inline void *mas_root_locked(struct ma_state *mas)
{
	return mt_root_locked(mas->tree);
}

/*
 * mas_root() - Get the maple tree root.
 * @mas: The maple state.
 *
 * Return: The pointer to the root of the tree
 */
static inline void *mas_root(struct ma_state *mas)
{
	if (mt_locked(mas->tree)){
		return rcu_dereference(mas->tree->ma_root);
	} else {
		fprintf(stderr, "mas->tree not locked");
	}
	// return rcu_dereference_check(mas->tree->ma_root, mt_locked(mas->tree));
}

static inline void *mt_root_locked(struct maple_tree *mt)
{
	if (!mt_locked(mt)) {
		fprintf(stderr, "mt_lock is not held\n");
	}
	return rcu_dereference(mt->ma_root);
}

static inline bool mt_locked(const struct maple_tree *mt)
{
	// TODO: how to impl lockdep_is_held()?
	return 1;
}

/*
 * mas_node_count() - Check if enough nodes are allocated and request more if
 * there is not enough nodes.
 * @mas: The maple state
 * @count: The number of nodes needed
 *
 * Note: Uses GFP_NOWAIT | __GFP_NOWARN for gfp flags.
 */
static void mas_node_count(struct ma_state *mas, int count)
{
	unsigned long allocated = mas_allocated(mas);

	if (allocated < count) {
		mas_set_alloc_req(mas, count - allocated);
		mas_alloc_nodes(mas, gfp);
	}
}



/*
 * mas_set_alloc_req() - Set the requested number of allocations.
 * @mas: the maple state
 * @count: the number of allocations.
 *
 * The requested number of allocations is either in the first allocated node,
 * located in @mas->alloc->request_count, or directly in @mas->alloc if there is
 * no allocated node.  Set the request either in the node or do the necessary
 * encoding to store in @mas->alloc directly.
 */
static inline void mas_set_alloc_req(struct ma_state *mas, unsigned long count)
{
	if (!mas->alloc || ((unsigned long)mas->alloc & 0x1)) {
		if (!count)
			mas->alloc = NULL;
		else
			mas->alloc = (struct maple_alloc *)(((count) << 1U) | 1U);
		return;
	}

	mas->alloc->request_count = count;
}

/*
 * mas_alloc_nodes() - Allocate nodes into a maple state
 * @mas: The maple state
 * @gfp: The GFP Flags
 */
static inline void mas_alloc_nodes(struct ma_state *mas)
{
	struct maple_alloc *node;
	unsigned long allocated = mas_allocated(mas);
	unsigned int requested = mas_alloc_req(mas);
	unsigned int count;
	void **slots = NULL;
	unsigned int max_req = 0;

	if (!requested)
		return;

	mas_set_alloc_req(mas, 0);
	if (mas->mas_flags & MA_STATE_PREALLOC) {
		if (allocated)
			return;
		WARN_ON(!allocated);
	}

	if (!allocated || mas->alloc->node_count == MAPLE_ALLOC_SLOTS) {
		node = (struct maple_alloc *)mt_alloc_one(gfp);
		if (!node)
			goto nomem_one;

		if (allocated) {
			node->slot[0] = mas->alloc;
			node->node_count = 1;
		} else {
			node->node_count = 0;
		}

		mas->alloc = node;
		node->total = ++allocated;
		requested--;
	}

	node = mas->alloc;
	node->request_count = 0;
	while (requested) {
		max_req = MAPLE_ALLOC_SLOTS - node->node_count;
		slots = (void **)&node->slot[node->node_count];
		max_req = min(requested, max_req);
		count = mt_alloc_bulk(gfp, max_req, slots);
		if (!count)
			goto nomem_bulk;

		if (node->node_count == 0) {
			node->slot[0]->node_count = 0;
			node->slot[0]->request_count = 0;
		}

		node->node_count += count;
		allocated += count;
		node = node->slot[0];
		requested -= count;
	}
	mas->alloc->total = allocated;
	return;

nomem_bulk:
	/* Clean up potential freed allocations on bulk failure */
	memset(slots, 0, max_req * sizeof(unsigned long));
nomem_one:
	mas_set_alloc_req(mas, requested);
	if (mas->alloc && !(((unsigned long)mas->alloc & 0x1)))
		mas->alloc->total = allocated;
	mas_set_err(mas, -ENOMEM);
}

static inline bool is_node(const void *entry)
{
	return ((unsigned long)entry & 3) == 2 && (unsigned long)entry > 4096;
}



/*
 * mas_destroy() - destroy a maple state.
 * @mas: The maple state
 *
 * Upon completion, check the left-most node and rebalance against the node to
 * the right if necessary.  Frees any allocated nodes associated with this maple
 * state.
 */
void mas_destroy(struct ma_state *mas)
{
	struct maple_alloc *node;
	unsigned long total;

	/*
	 * When using mas_for_each() to insert an expected number of elements,
	 * it is possible that the number inserted is less than the expected
	 * number.  To fix an invalid final node, a check is performed here to
	 * rebalance the previous node with the final node.
	 */
	if (mas->mas_flags & MA_STATE_REBALANCE) {
		unsigned char end;

		mas_start(mas);
		mtree_range_walk(mas);
		end = mas_data_end(mas) + 1;
		if (end < mt_min_slot_count(mas->node) - 1)
			mas_destroy_rebalance(mas, end);

		mas->mas_flags &= ~MA_STATE_REBALANCE;
	}
	mas->mas_flags &= ~(MA_STATE_BULK|MA_STATE_PREALLOC);

	total = mas_allocated(mas);
	while (total) {
		node = mas->alloc;
		mas->alloc = node->slot[0];
		if (node->node_count > 1) {
			size_t count = node->node_count - 1;

			mt_free_bulk(count, (void __rcu **)&node->slot[1]);
			total -= count;
		}
		kmem_cache_free(maple_node_cache, node);
		total--;
	}

	mas->alloc = NULL;
}