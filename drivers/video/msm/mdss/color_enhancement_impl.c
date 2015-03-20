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
					&impl_data->ctrl->ce_level1_cmds);
		impl_data->settings |= (EN | LEVEL1);
		break;

	case 2:
		if (impl_data->ctrl->ce_level2_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level2_cmds);
		impl_data->settings |= (EN | LEVEL2);
		break;

	case 3:
		if (impl_data->ctrl->ce_level3_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level3_cmds);
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

static int r69429_get_mode(struct color_enhancement_t *ce)
{
	struct mdss_color_enhancement_t *impl_data;

	impl_data = ce->impl_data;
	if (!impl_data) {
		pr_err("%s: no driver data\n", __func__);
		return -ENODEV;
	}

	return impl_data->settings;
}

static int r69429_ce_init(struct color_enhancement_t *ce)
{
	mutex_init(&mdss_ce_data.set_ce_lock);

	ce->impl_data = (void *) &mdss_ce_data;

	return 0;
}

struct ce_impl_ops_t ce_impl_ops = {
	.init = r69429_ce_init,
	.enable = NULL,
	.disable = NULL,
	.set_level = r69429_ce_set_level,
	.restore_default = NULL,
	.cabc_off = NULL,
	.ui_mode = NULL,
	.still_mode = NULL,
	.moving_mode = NULL,
	.aco = NULL,
	.osc = NULL,
	.init_setting = NULL,
	.get_mode = r69429_get_mode,
};

void color_enhancement_impl_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_ce_data.ctrl = ctrl_pdata;
}
EXPORT_SYMBOL(color_enhancement_impl_init);
