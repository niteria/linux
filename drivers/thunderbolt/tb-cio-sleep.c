// SPDX-License-Identifier: GPL-2.0-only
/* Included by tb.c. Native CIO PM keeps tunnel allocations and host ownership. */
static const struct tb_cm_ops tb_cm_ops;
static void tb_restore_dp_resources(struct tb_switch *sw);

static int tb_cio_check_topology(struct tb_switch *sw)
{
	struct tb_port *port;
	int ret;

	if (sw->is_unplugged || !sw->uid)
		return -ENODEV;
	tb_switch_for_each_port(sw, port) {
		if (port->xdomain)
			return -EOPNOTSUPP;
		if (tb_port_has_remote(port)) {
			ret = tb_cio_check_topology(port->remote->sw);
			if (ret)
				return ret;
		}
	}
	return 0;
}

int tb_cio_sleep_prepare(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	int ret;

	if (tb->cm_ops != &tb_cm_ops || !tb->nhi->ops->router_sleep ||
	    !tb_switch_is_usb4(tb->root_switch))
		return -EOPNOTSUPP;
	mutex_lock(&tb->lock);
	if (tcm->cio_prepared || tcm->stopping || !tcm->hotplug_active) {
		ret = -EBUSY;
		goto out;
	}
	ret = tb_cio_check_topology(tb->root_switch);
	if (ret)
		goto out;
	tcm->hotplug_active = false;
	tcm->cio_prepared = true;
	/*
	 * Displays do not survive the host router's power cycle: release them
	 * now, while their owners still run, and pair them again afterwards as
	 * after a plug. Only the PCIe and USB tunnels are kept and replayed.
	 */
	tb_disconnect_and_release_dp(tb);
	tcm->cio_dp_released = true;
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		ret = tb_tunnel_sleep_prepare(tunnel);
		if (ret)
			break;
	}
	mutex_unlock(&tb->lock);
	/* A DPRX callback already outside the lock must finish before device PM. */
	flush_workqueue(tb->wq);
	/* Bandwidth release runs on a system queue, outside the CM workqueue. */
	for (unsigned int i = 0; i < ARRAY_SIZE(tcm->groups); i++) {
		if (cancel_delayed_work_sync(&tcm->groups[i].release_work)) {
			guard(mutex)(&tb->lock);
			tcm->cio_bw_pending |= BIT(i);
		}
	}
	if (ret)
		tb_cio_sleep_complete(tb);
	return ret;
out:
	mutex_unlock(&tb->lock);
	return ret;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep_prepare, "thunderbolt_apple");

/* The restarted host router trains its links again once it has the cable word. */
static int tb_cio_wait_links(struct tb *tb)
{
	struct tb_port *port;
	int ret = 0;

	tb_switch_for_each_port(tb->root_switch, port) {
		unsigned int tries;

		if (!tb_port_has_remote(port))
			continue;
		for (tries = 0; tries < 30; tries++) {
			ret = tb_wait_for_port(port, true);
			if (ret)
				break;
			msleep(100);
		}
		tb_info(tb, "CIO port %u link after wake: %d (%u retries)\n", port->port, ret,
			tries);
		if (ret <= 0)
			return ret ?: -ENOLINK;
	}
	return 0;
}

static int tb_cio_wake_locked(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	const char *step;
	bool restarted;
	u64 uid = 0;
	int ret;

	lockdep_assert_held(&tb->lock);
	if (!tcm->cio_changed)
		return 0;
	if (tcm->cio_ctl_stopped) {
		tb_ctl_start(tb->ctl);
		tcm->cio_ctl_stopped = false;
	}
	/* Saved router state means the controller went through its power cycle. */
	restarted = tcm->cio_router.valid;
	if (restarted) {
		/* A restarted router answers for its adapters only once enumerated. */
		step = "router enumerate";
		ret = tb_switch_configure(tb->root_switch);
		if (ret)
			goto fail;
	}
	if (tcm->cio_router.valid) {
		step = "router restore";
		ret = apple_router_restore_sleep(tb->root_switch, &tcm->cio_router);
		if (ret)
			goto fail;
	}
	step = "host router UID";
	ret = usb4_switch_read_uid(tb->root_switch, &uid);
	if (ret || uid != tcm->cio_host_uid) {
		ret = ret ?: -ENODEV;
		goto fail;
	}
	if (restarted && tb->nhi->ops->router_wake) {
		step = "router wake";
		ret = tb->nhi->ops->router_wake(tb->nhi);
		if (ret)
			goto fail;
		step = "link";
		ret = tb_cio_wait_links(tb);
		if (ret)
			goto fail;
	}
	step = "router resume";
	ret = tb_switch_resume(tb->root_switch, false);
	if (ret)
		goto fail;
	step = "topology";
	ret = tb_cio_check_topology(tb->root_switch);
	if (ret)
		goto fail;
	tb_restore_children(tb->root_switch);
	step = "tunnel replay";
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		ret = tb_tunnel_wake(tunnel);
		if (ret)
			goto fail;
	}
	tb_switch_enter_redrive(tb->root_switch);
	tcm->cio_changed = false;
	/* Do not run DP callbacks or reopen hotplug until all drivers resume. */
	return 0;
fail:
	tb_err(tb, "CIO wake failed at %s: %d (UID %#llx)\n", step, ret, uid);
	return ret;
}

int tb_cio_sleep(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	int ret, recover;

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared || tcm->cio_changed)
		return -EINVAL;
	/* A host router without a UID of its own reads zero here; wake compares like with like. */
	ret = usb4_switch_read_uid(tb->root_switch, &tcm->cio_host_uid);
	if (ret)
		return ret;
	tcm->cio_changed = true;
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		ret = tb_tunnel_sleep(tunnel);
		if (ret)
			goto unwind;
	}
	tb_switch_exit_redrive(tb->root_switch);
	ret = tb_switch_suspend_cio(tb->root_switch) ?:
	      tb->nhi->ops->router_sleep(tb->nhi) ?:
	      apple_router_save_sleep(tb->root_switch, &tcm->cio_router);
	if (ret)
		goto unwind;
	tb_ctl_stop(tb->ctl);
	tcm->cio_ctl_stopped = true;
	return 0;
unwind:
	recover = tb_cio_wake_locked(tb);
	if (recover)
		panic("Apple CIO: router suspend %d, recovery %d", ret, recover);
	return ret;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep, "thunderbolt_apple");

int tb_cio_wake(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared)
		return -EINVAL;
	return tb_cio_wake_locked(tb);
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_wake, "thunderbolt_apple");

void tb_cio_sleep_complete(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared)
		return;
	if (tcm->cio_changed || tcm->cio_ctl_stopped || tcm->cio_router.valid)
		panic("Apple CIO: completing an unrestored router transaction");
	list_for_each_entry(tunnel, &tcm->tunnel_list, list)
		tb_tunnel_sleep_complete(tunnel);
	tcm->cio_prepared = false;
	tcm->hotplug_active = true;
	if (tcm->cio_dp_released) {
		tcm->cio_dp_released = false;
		tb_restore_dp_resources(tb->root_switch);
		tcm->dp_tunnel_pending = false;
		tb_tunnel_dp(tb);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(tcm->groups); i++) {
		if (tcm->cio_bw_pending & BIT(i))
			mod_delayed_work(system_percpu_wq, &tcm->groups[i].release_work, 0);
	}
	tcm->cio_bw_pending = 0;
	/* The ordinary complete hook rescans events suppressed during sleep. */
	tcm->resume_cleanup_pending = true;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep_complete, "thunderbolt_apple");
