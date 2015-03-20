/*
 * Yulong color enhancement implementation
 */
#include <linux/kernel.h>
#include <linux/color_enhancement.h>
#include <linux/fb.h>
#include "mdss_dsi.h"

#define MDSS_CE_OFF	0
#define MDSS_CE_L1	1
#define MDSS_CE_L2	2
#define MDSS_CE_L3	3

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds);

struct mdss_color_enhancement_t {
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct mutex set_ce_lock;
	int current_level;
};

static struct mdss_color_enhancement_t mdss_ce_data;

static int r69429_ce_set_level_impl(struct mdss_color_enhancement_t *impl_data, int level)
{
	mutex_lock(&impl_data->set_ce_lock);

	switch (level) {
	case MDSS_CE_OFF:
		if (impl_data->ctrl->ce_off_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_off_cmds);
		break;

	case MDSS_CE_L1:
		if (impl_data->ctrl->ce_level1_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level1_cmds);
		break;

	case MDSS_CE_L2:
		if (impl_data->ctrl->ce_level2_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level2_cmds);
		break;

	case MDSS_CE_L3:
		if (impl_data->ctrl->ce_level3_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(impl_data->ctrl,
					&impl_data->ctrl->ce_level3_cmds);
		break;
	}

	impl_data->current_level = level;

	mutex_unlock(&impl_data->set_ce_lock);

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

	return impl_data->current_level;
}

static int r69429_ce_init(struct color_enhancement_t *ce)
{
	mutex_init(&mdss_ce_data.set_ce_lock);
	mdss_ce_data.current_level = MDSS_CE_OFF;

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

void color_enhancement_impl_apply(void)
{
	struct mdss_color_enhancement_t *ce = &mdss_ce_data;

	r69429_ce_set_level_impl(ce, ce->current_level);
}

void color_enhancement_impl_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_ce_data.ctrl = ctrl_pdata;
}
EXPORT_SYMBOL(color_enhancement_impl_init);
