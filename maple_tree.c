#include "maple_tree.h"

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
	if (mas_nomem(&mas, gfp))
		goto retry;

	mtree_unlock(mt);
	if (mas_is_err(&mas))
		return xa_err(mas.node);

	return 0;
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
		if (likely(xa_is_node(root))) {
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