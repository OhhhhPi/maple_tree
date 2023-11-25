#include "maple_tree.h"
#include <stdio.h>

#define MA_ROOT_PARENT 1


/* Bit 1 indicates the root is a node */
#define MAPLE_ROOT_NODE			0x02
/* maple_type stored bit 3-6 */
#define MAPLE_ENODE_TYPE_SHIFT		0x03
/* Bit 2 means a NULL somewhere below */
#define MAPLE_ENODE_NULL		0x04


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