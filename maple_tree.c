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
	// rcu_assign_pointer(mt->ma_root, NULL);
}