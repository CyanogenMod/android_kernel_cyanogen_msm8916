/*
 * Yulong color enhancement implementation
 */
#include <linux/kernel.h>
#include <linux/color_enhancement.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include "mdss_dsi.h"

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds);

struct mdss_color_enhancement_t {
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct work_struct set_ce_work;
	struct mutex set_ce_lock;
	u32 settings;
	bool first_set_ce;
};

static struct mdss_color_enhancement_t mdss_ce_data;

static int r69429_ce_set_level_impl(struct mdss_color_enhancement_t *impl_data, int level)
{
	impl_data->settings &= (~CE_MASK);

	switch (level) {
	case 0:
		if (impl_data->ctrl->ce_off_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_off_cmds);
		impl_data->settings &= (~EN);
		break;
	case 1:
		if (impl_data->ctrl->ce_level1_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level2_cmds);
		impl_data->settings |= (EN | LEVEL1);
		break;

	case 2:
		if (impl_data->ctrl->ce_level2_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level3_cmds);
		impl_data->settings |= (EN | LEVEL2);
		break;

	case 3:
		if (impl_data->ctrl->ce_level3_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level1_cmds);
		impl_data->settings |= (EN | DEFAULT);
		break;
	}

	return 0;
}

static int r69429_ce_set_level(struct color_enhancement_t *ce, int level)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	return r69429_ce_set_level_impl(impl_data, level);
}

static int r69429_ce_restore_default(struct color_enhancement_t *ce)
{
	/* Do nothing, previous implementation blanked/unblanked the
	 * framebuffer, which is horrible.
	 */
	return 0;
}

static int r69429_aco(struct color_enhancement_t *ce, int on)
{
	struct mdss_color_enhancement_t *impl_data;

	pr_debug("%s : %s", __func__, on ? "on" : "off");

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	if (on)
		impl_data->settings |= ACO_ON;
	else
		impl_data->settings &= (~ACO_ON);

	pr_debug("%s: setting = %x", __func__, impl_data->settings);

	return 0;
}

static int r69429_osc(struct color_enhancement_t *ce, int weak)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	if (weak)
		impl_data->settings |= OSC_WEAK;
	else
		impl_data->settings &= ~OSC_WEAK;

	return 0;
}

static int r69429_cabc_off(struct color_enhancement_t *ce)
{
	struct mdss_color_enhancement_t *impl_data;

	pr_debug("%s", __func__);

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	if (impl_data->ctrl->cabc_off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(impl_data->ctrl,
				&impl_data->ctrl->cabc_off_cmds);
	impl_data->settings &= ~CABC_MOVING;

	pr_debug("%s: setting = %x", __func__, impl_data->settings);

	return 0;
}

static int r69429_moving_mode(struct color_enhancement_t *ce)
{
	struct mdss_color_enhancement_t *impl_data;

	pr_debug("%s", __func__);

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	if (impl_data->ctrl->cabc_moving_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(impl_data->ctrl,
				&impl_data->ctrl->cabc_moving_cmds);
	impl_data->settings |= CABC_MOVING;

	pr_debug("%s: setting = %x", __func__, impl_data->settings);

	return 0;
}

static int r69429_init_setting(struct color_enhancement_t *ce, int setting)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	impl_data->settings = (setting & 0x111F);

	pr_debug("--mdss--ce_init--%s: ce->setting = %x--\n",
			__func__, impl_data->settings);

	schedule_work(&impl_data->set_ce_work);

	return 0;
}

static int r69429_ce_enable(struct color_enhancement_t *ce)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	if (impl_data->ctrl->ce_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(impl_data->ctrl,
				&impl_data->ctrl->ce_cmds);

	impl_data->settings |= EN;

	pr_debug("--mdss%s: setting = %x--\n", __func__, impl_data->settings);

	return 0;
}

static int r69429_ce_disable(struct color_enhancement_t *ce)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	r69429_ce_set_level_impl(impl_data, 0);

	pr_debug("--mdss--%s: setting = %x--\n",
			__func__, impl_data->settings);

	return 0;
}

static void set_ce_fn(struct work_struct *work)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = container_of(work, struct mdss_color_enhancement_t,
			set_ce_work);

	pr_debug("--mdss--enter--%s: ivan setting = %x--\n",
			__func__, impl_data->settings);

	mutex_lock(&impl_data->set_ce_lock);

	if (impl_data->settings == DEFAULT_SETTING)
		goto set_ce_fn_end;

	if (!impl_data->first_set_ce) {
		impl_data->first_set_ce = true;
		goto set_ce_fn_end;
	}

	/* Need delay in resume period */
	msleep(100);

	switch (impl_data->settings) {
		case PICTURE:
			/* 0x1105 */
			r69429_ce_set_level_impl(impl_data, 1);
			break;

		case MOVIE:
			/* 0x0019 */
			r69429_ce_set_level_impl(impl_data, 2);
			break;

		case STANDARD:
			/* 0x1013 */
			r69429_ce_set_level_impl(impl_data, 3);
			break;

		default:
			/* Disable */
			r69429_ce_set_level_impl(impl_data, 0);
			break;
	}

	/* sleep a frame time, every dcs cmd will send frame by frame */
	msleep(20);

set_ce_fn_end:
	mutex_unlock(&impl_data->set_ce_lock);

	pr_debug("--mdss--exit--%s--\n", __func__);
}

static int r69429_ce_init(struct color_enhancement_t *ce)
{
	INIT_WORK(&mdss_ce_data.set_ce_work, set_ce_fn);
	mutex_init(&mdss_ce_data.set_ce_lock);
	mdss_ce_data.settings = DEFAULT_SETTING;

	ce->impl_data = (void *) &mdss_ce_data;

	return 0;
}

struct ce_impl_ops_t ce_impl_ops = {
	.init = r69429_ce_init,
	.enable = r69429_ce_enable,
	.disable = r69429_ce_disable,
	.set_level = r69429_ce_set_level,
	.restore_default = r69429_ce_restore_default,
	.cabc_off = r69429_cabc_off,
	.ui_mode = NULL,
	.still_mode = NULL,
	.moving_mode = r69429_moving_mode,
	.aco = r69429_aco,
	.osc = r69429_osc,
	.init_setting = r69429_init_setting,
};

void color_enhancement_impl_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_ce_data.ctrl = ctrl_pdata;
}
EXPORT_SYMBOL(color_enhancement_impl_init);
