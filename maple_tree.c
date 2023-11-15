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
int mtree_store_range(struct maple_tree *mt, unsigned long first, unsigned long last, void *entry){
	// MA_STATE(mas, mt, index, last);
	MA_STATE(mas,mt,index,last);
	MA_WR_STATE(wr_mas, &mas, entry);

	trace_ma_write(__func__, &mas, 0, entry);
	if (WARN_ON_ONCE(xa_is_advanced(entry)))
		return -EINVAL;

	if (index > last)
		return -EINVAL;

	mtree_lock(mt);
retry:
	mas_wr_store_entry(&wr_mas);
	if (mas_nomem(&mas, gfp))
		goto retry;

	mtree_unlock(mt);
	if (mas_is_err(&mas))
		return xa_err(mas.node);

	return 0;
}