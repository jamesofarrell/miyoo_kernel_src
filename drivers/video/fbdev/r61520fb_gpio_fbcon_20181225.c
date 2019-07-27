/*
 * Copyright (C) 2018 Steward Fu <steward.fu@gmail.com>
 *
 * framebuffer driver for Renesas R61520 SLCD panel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/console.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <linux/clk-provider.h>
#include <video/of_display_timing.h>
#include <linux/gpio.h>
#include <linux/omapfb.h>
#include <linux/compiler.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch-sunxi/cpu.h>
#include <asm/arch-sunxi/gpio.h>
#include <asm/arch-sunxi/lcdc.h>
#include <asm/arch-sunxi/clock.h>
#include <asm/arch-sunxi/common.h>
#include <asm/arch-sunxi/display.h>

#define SUNXI_LCD_GPIO      1
#define PALETTE_SIZE		    256
#define DRIVER_NAME         "r61520_slcd"
#define SLCD_PWM				    ((32 * 4) + 6)    // PE6
#define SLCD_RESET			    ((32 * 4) + 11)   // PE11
 
struct r61520fb_par {
  struct device *dev;
 
  resource_size_t p_palette_base;
  unsigned short *v_palette_base;
 
  dma_addr_t vram_phys;
  unsigned long vram_size;
  void *vram_virt;
 
  dma_addr_t tram_phys;
  void *tram_virt;
 
  int irq;
  u32 pseudo_palette[16];
  struct fb_videomode mode;
  unsigned int bpp;

	uint32_t is_enabled;
  struct mutex lock;
	struct delayed_work refresh_work;
};

static struct fb_var_screeninfo r61520fb_var;
static struct fb_fix_screeninfo r61520fb_fix = {
  .id = "R61520 FB",
  .type = FB_TYPE_PACKED_PIXELS,
  .type_aux = 0,
  .visual = FB_VISUAL_TRUECOLOR,
  .xpanstep = 0,
  .ypanstep = 1,
  .ywrapstep = 0,
  .accel = FB_ACCEL_NONE
};

struct sunxi_iomm {
  struct sunxi_ccm_reg *ccm;
  struct sunxi_lcdc_reg *lcdc;
  struct sunxi_gpio_reg *gpio;
  struct sunxi_de_fe_reg *de_fe;
  struct sunxi_de_be_reg *de_be;
};
static struct sunxi_iomm iomm={0};

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)
static int fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info)
{
  //printk("%s, visual:%d, bits_per_pixel:%d, regno:%d, r:0x%x, g:0x%x, b:0x%x\n", __func__, info->fix.visual, info->var.bits_per_pixel, regno, red, green, blue);
  red = CNVT_TOHW(red, info->var.red.length);
  blue = CNVT_TOHW(blue, info->var.blue.length);
  green = CNVT_TOHW(green, info->var.green.length);
  ((u32*)(info->pseudo_palette))[regno] = (red << info->var.red.offset) | (green << info->var.green.offset) | (blue << info->var.blue.offset);
  return 0;
}
#undef CNVT_TOHW
 
static int fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
  int err = 0;
  int bpp = var->bits_per_pixel >> 3;
  struct r61520fb_par *par = info->par;
  unsigned long line_size = var->xres_virtual * bpp;
 
  //if((var->xres != 320) || (var->yres != 240) || (var->bits_per_pixel != 16)){
    //return -EINVAL;
  //}
 
  printk("%s, xres:%d, yres:%d, bpp:%d\n", __func__, var->xres, var->yres, var->bits_per_pixel);
  var->transp.offset = 0;
  var->transp.length = 0;
  var->red.offset = 11;
  var->red.length = 5;
  var->green.offset = 5;
  var->green.length = 6;
  var->blue.offset = 0;
  var->blue.length = 5;
  var->red.msb_right = 0;
  var->green.msb_right = 0;
  var->blue.msb_right = 0;
  var->transp.msb_right = 0;
  if(line_size * var->yres_virtual > par->vram_size){
    var->yres_virtual = par->vram_size / line_size;
  }
  if(var->yres > var->yres_virtual){
    var->yres = var->yres_virtual;
  }
  if(var->xres > var->xres_virtual){
    var->xres = var->xres_virtual;
  }
  if(var->xres + var->xoffset > var->xres_virtual){
    var->xoffset = var->xres_virtual - var->xres;
  }
  if(var->yres + var->yoffset > var->yres_virtual){
    var->yoffset = var->yres_virtual - var->yres;
  }
  return err;
}
 
static int fb_remove(struct platform_device *dev)
{
  struct fb_info *info = dev_get_drvdata(&dev->dev);
 
  if(info){
    struct r61520fb_par *par = info->par;
    unregister_framebuffer(info);
    fb_dealloc_cmap(&info->cmap);
    dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base, par->p_palette_base);
    dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);
    pm_runtime_put_sync(&dev->dev);
    pm_runtime_disable(&dev->dev);
    framebuffer_release(info);
  }
  return 0;
}
 
static int fb_set_par(struct fb_info *info)
{
  struct r61520fb_par *par = info->par;
   
  fb_var_to_videomode(&par->mode, &info->var);
  printk("%s, xres:%d, yres:%d, bpp:%d, xoffset:%d, yoffset:%d\n", __func__, 
    info->var.xres, info->var.yres, info->var.bits_per_pixel, info->var.xoffset, info->var.yoffset);
  par->bpp = info->var.bits_per_pixel;
  info->fix.visual = FB_VISUAL_TRUECOLOR;
  info->fix.line_length = (par->mode.xres * par->bpp) / 8; 
  return 0;
}
 
static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
  printk("%s(cmd: 0x%x)++\n", __func__, cmd);
  switch(cmd){
  case OMAPFB_QUERY_PLANE:
    printk("OMAPFB_QUERY_PLANE\n");
    break;
  case OMAPFB_QUERY_MEM:
    printk("OMAPFB_QUERY_MEM\n");
    break;
  case OMAPFB_SETUP_PLANE:
    printk("OMAPFB_SETUP_PLANE\n");
    break;
  case OMAPFB_SETUP_MEM:
    printk("OMAPFB_SETUP_MEM\n");
    break;
  }
  printk("%s\n", __func__);
  return 0;
}
 
static int fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
  const unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
  const unsigned long size = vma->vm_end - vma->vm_start;
  
  if(offset + size > info->fix.smem_len){
    return -EINVAL;
  }
  
  if(remap_pfn_range(vma, vma->vm_start, (info->fix.smem_start + offset) >> PAGE_SHIFT, size, vma->vm_page_prot)){
    return -EAGAIN;
  }
  return 0;
}
 
static struct fb_ops r61520fb_ops = {
  .owner = THIS_MODULE,
  .fb_check_var = fb_check_var,
  .fb_set_par   = fb_set_par,
  .fb_setcolreg = fb_setcolreg,
  .fb_ioctl     = fb_ioctl,
  .fb_mmap      = fb_mmap,
 
  .fb_fillrect  = sys_fillrect,
  .fb_copyarea  = sys_copyarea,
  .fb_imageblit = sys_imageblit,
};
 
static const u32 sunxi_vert_coef[32] = {
	0x00004000, 0x000140ff, 0x00033ffe, 0x00043ffd,
	0x00063efc, 0xff083dfc, 0x000a3bfb, 0xff0d39fb,
	0xff0f37fb, 0xff1136fa, 0xfe1433fb, 0xfe1631fb,
	0xfd192ffb, 0xfd1c2cfb, 0xfd1f29fb, 0xfc2127fc,
	0xfc2424fc, 0xfc2721fc, 0xfb291ffd, 0xfb2c1cfd,
	0xfb2f19fd, 0xfb3116fe, 0xfb3314fe, 0xfa3611ff,
	0xfb370fff, 0xfb390dff, 0xfb3b0a00, 0xfc3d08ff,
	0xfc3e0600, 0xfd3f0400, 0xfe3f0300, 0xff400100,
};

static const u32 sunxi_horz_coef[64] = {
	0x40000000, 0x00000000, 0x40fe0000, 0x0000ff03,
	0x3ffd0000, 0x0000ff05, 0x3ffc0000, 0x0000ff06,
	0x3efb0000, 0x0000ff08, 0x3dfb0000, 0x0000ff09,
	0x3bfa0000, 0x0000fe0d, 0x39fa0000, 0x0000fe0f,
	0x38fa0000, 0x0000fe10, 0x36fa0000, 0x0000fe12,
	0x33fa0000, 0x0000fd16, 0x31fa0000, 0x0000fd18,
	0x2ffa0000, 0x0000fd1a, 0x2cfa0000, 0x0000fc1e,
	0x29fa0000, 0x0000fc21, 0x27fb0000, 0x0000fb23,
};

static void sunxi_frontend_init(void)
{
  uint32_t i=0;

  // clocks on
  sunxi_setbits(&iomm.ccm->bus_soft_rst1, CCM_BUS_SOFT_RST1_DEFE);
  sunxi_setbits(&iomm.ccm->bus_clk_gating1, CCM_BUS_CLOCK_GATING1_DEFE);
  sunxi_setbits(&iomm.ccm->dram_gating, CCM_DRAM_GATING_FE);
  clock_set_de_mod_clock(&iomm.ccm->fe_clk, 300000000);
  sunxi_setbits(&iomm.de_fe->enable, SUNXI_DE_FE_ENABLE_EN);

  for (i = 0; i < 32; i++) {
    writel(sunxi_horz_coef[i], &iomm.de_fe->ch0_horzcoef0[i]);
    writel(sunxi_vert_coef[i], &iomm.de_fe->ch0_vertcoef[i]);
    writel(sunxi_horz_coef[i], &iomm.de_fe->ch1_horzcoef0[i]);
    writel(sunxi_vert_coef[i], &iomm.de_fe->ch1_vertcoef[i]);
  }
  sunxi_setbits(&iomm.de_fe->frame_ctrl, SUNXI_DE_FE_FRAME_CTRL_REG_RDY);
}

static void sunxi_backend_init(void)
{
  uint32_t i=0;

	// clocks on
  sunxi_setbits(&iomm.ccm->bus_soft_rst1, CCM_BUS_SOFT_RST1_DEBE);
  sunxi_setbits(&iomm.ccm->bus_clk_gating1, CCM_BUS_CLOCK_GATING1_DEBE);
  sunxi_setbits(&iomm.ccm->dram_gating, CCM_DRAM_GATING_BE);
  clock_set_de_mod_clock(&iomm.ccm->be_clk, 300000000);
 
  // engine bug, clear registers after reset
  for (i = 0x0800; i < 0x1000; i += 4) {
    writel(0, ((uint8_t*)iomm.de_be + i));
  }
  sunxi_setbits(&iomm.de_be->mode, SUNXI_DE_BE_MODE_ENABLE); 
}

static void sunxi_frontend_mode_set(const struct r61520fb_par *par)
{
  sunxi_setbits(&iomm.de_fe->bypass, SUNXI_DE_FE_BYPASS_CSC_BYPASS);
  writel(par->tram_virt, &iomm.de_fe->ch0_addr);
  writel(par->mode.xres * 4, &iomm.de_fe->ch0_stride);
  writel(SUNXI_DE_FE_INPUT_FMT_ARGB8888, &iomm.de_fe->input_fmt);
  writel(SUNXI_DE_FE_OUTPUT_FMT_ARGB8888, &iomm.de_fe->output_fmt);
  writel(SUNXI_DE_FE_HEIGHT(par->mode.yres) | SUNXI_DE_FE_WIDTH(par->mode.xres), &iomm.de_fe->ch0_insize);
  writel(SUNXI_DE_FE_HEIGHT(par->mode.yres) | SUNXI_DE_FE_WIDTH(par->mode.xres), &iomm.de_fe->ch0_outsize);
  writel(SUNXI_DE_FE_FACTOR_INT(1), &iomm.de_fe->ch0_horzfact);
  writel(SUNXI_DE_FE_FACTOR_INT(1), &iomm.de_fe->ch0_vertfact);

  writel(SUNXI_DE_FE_HEIGHT(par->mode.yres) | SUNXI_DE_FE_WIDTH(par->mode.xres), &iomm.de_fe->ch1_insize);
  writel(SUNXI_DE_FE_HEIGHT(par->mode.yres) | SUNXI_DE_FE_WIDTH(par->mode.xres), &iomm.de_fe->ch1_outsize);
  writel(SUNXI_DE_FE_FACTOR_INT(1), &iomm.de_fe->ch1_horzfact);
  writel(SUNXI_DE_FE_FACTOR_INT(1), &iomm.de_fe->ch1_vertfact);

  sunxi_setbits(&iomm.de_fe->frame_ctrl, SUNXI_DE_FE_FRAME_CTRL_REG_RDY);
}

static void sunxi_backend_mode_set(const struct r61520fb_par *par)
{
  sunxi_setbits(&iomm.de_be->mode, SUNXI_DE_BE_MODE_LAYER0_ENABLE);
}

static void r61520_lcd_cmd(uint16_t val)
{
  uint32_t ret;

#if defined(SUNXI_LCD_GPIO)
  ret = (val & 0x00ff) << 1;
  ret|= (val & 0xff00) << 2;
  ret|= 0x100000;
  writel(ret, &iomm.gpio->gpio_bank[3].dat);
  ndelay(1);
  ret|= 0x40000;
  writel(ret, &iomm.gpio->gpio_bank[3].dat);
#else
  ret = readl(&iomm.lcdc->tcon0_cpu_intf);
  ret&= 0xfdffffff;
  writel(ret, &iomm.lcdc->tcon0_cpu_intf);
  ret|= 1;
  writel(ret, &iomm.lcdc->tcon0_cpu_intf);
  while(readl(&iomm.lcdc->tcon0_cpu_intf) & 0xc0000){
    ndelay(1);
  }
  writel(val, &iomm.lcdc->tcon0_cpu_wr_dat);
#endif
}

static void r61520_lcd_data(uint16_t val)
{
  uint32_t ret;

#if defined(SUNXI_LCD_GPIO)
  ret = (val & 0x00ff) << 1;
  ret|= (val & 0xff00) << 2;
  ret|= 0x80000;
  ret|= 0x100000;
  writel(ret, &iomm.gpio->gpio_bank[3].dat);
  ndelay(1);
  ret|= 0x40000;
  writel(ret, &iomm.gpio->gpio_bank[3].dat);
#else
  ret = readl(&iomm.lcdc->tcon0_cpu_intf);
  ret|= 0x2000000;
  writel(ret, &iomm.lcdc->tcon0_cpu_intf);
  ret|= 1;
  writel(ret, &iomm.lcdc->tcon0_cpu_intf);
  while(readl(&iomm.lcdc->tcon0_cpu_intf) & 0xc0000){
    ndelay(1);
  }
  writel(val, &iomm.lcdc->tcon0_cpu_wr_dat);
#endif
}

static void sunxi_lcdc_init(const struct r61520fb_par *par)
{
  uint32_t ret;

#if defined(SUNXI_LCD_GPIO)
  writel(0x11111117, &iomm.gpio->gpio_bank[3].cfg[0]); // 0x22222227
  writel(0x11111171, &iomm.gpio->gpio_bank[3].cfg[1]); // 0x22222272
  writel(0x00111111, &iomm.gpio->gpio_bank[3].cfg[2]); // 0x00222222, CS/RD/RS/WR
#else
  writel(0x22222227, &iomm.gpio->gpio_bank[3].cfg[0]); // 0x22222227
  writel(0x22222272, &iomm.gpio->gpio_bank[3].cfg[1]); // 0x22222272
  writel(0x00222222, &iomm.gpio->gpio_bank[3].cfg[2]); // 0x00222222, CS/RD/RS/WR
#endif

  ret = readl(&iomm.gpio->gpio_bank[3].cfg[0]);
  printk("%s, portd cfg-0: 0x%08x\n", __func__, ret);
  ret = readl(&iomm.gpio->gpio_bank[3].cfg[1]);
  printk("%s, portd cfg-1: 0x%08x\n", __func__, ret);
  ret = readl(&iomm.gpio->gpio_bank[3].cfg[2]);
  printk("%s, portd cfg-2: 0x%08x\n", __func__, ret);
 
  ret = clock_get_video_pll();
  printk("%s, pll video(before): %dHz\n", __func__, ret);
  clock_set_video_pll(12000000);
  ret = clock_get_video_pll();
  printk("%s, pll video(after): %dHz\n", __func__, ret);
  
  sunxi_setbits(&iomm.ccm->bus_soft_rst1, (1 << 4));
  ret = readl(&iomm.ccm->bus_soft_rst1);
  printk("%s, bus_soft_rst1: 0x%08x\n", __func__, ret);
  
  sunxi_setbits(&iomm.ccm->bus_clk_gating1, (1 << 4));
  ret = readl(&iomm.ccm->bus_clk_gating1);
  printk("%s, bus_clk_gating1: 0x%08x\n", __func__, ret);
  
  sunxi_setbits(&iomm.ccm->tcon_clk, (1 << 31));
  ret = readl(&iomm.ccm->tcon_clk);
  printk("%s, tcon_clk: 0x%08x\n", __func__, ret);

  writel(0x00000000, &iomm.lcdc->ctrl);
  writel(0x00000000, &iomm.lcdc->int0);
  writel(0x00000000, &iomm.lcdc->int1);
	writel((1 << 31) | (1 << 24), &iomm.lcdc->tcon0_ctrl);
  writel((1 << 31) | (2 << 0), &iomm.lcdc->tcon0_dclk);
  writel(0x014000f0, &iomm.lcdc->tcon0_timing_active);
  writel(0x01e00003, &iomm.lcdc->tcon0_timing_h);
  writel(0x03840003, &iomm.lcdc->tcon0_timing_v);
  writel(0x00000000, &iomm.lcdc->tcon0_timing_sync);
  writel((1 << 31), &iomm.lcdc->tcon0_cpu_intf);
  writel(0x00000000, &iomm.lcdc->tcon0_io_polarity);
  writel(0x00000000, &iomm.lcdc->tcon0_io_tristate);
  writel(0x00000000, &iomm.lcdc->tcon1_io_tristate);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[0]);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[1]);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[2]);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[3]);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[4]);
	writel(0x11111111, &iomm.lcdc->tcon0_frm_seed[5]);
	writel(0x01010000, &iomm.lcdc->tcon0_frm_table[0]);
	writel(0x15151111, &iomm.lcdc->tcon0_frm_table[1]);
	writel(0x57575555, &iomm.lcdc->tcon0_frm_table[2]);
	writel(0x7f7f7777, &iomm.lcdc->tcon0_frm_table[3]);
	writel((1 << 31) | (5 << 4), &iomm.lcdc->tcon0_frm_ctrl);
  writel((1 << 31), &iomm.lcdc->ctrl);
}

static void sunxi_frontend_enable(void)
{
  sunxi_setbits(&iomm.de_fe->frame_ctrl, SUNXI_DE_FE_FRAME_CTRL_FRM_START);
}

static void sunxi_backend_enable(void)
{
  sunxi_setbits(&iomm.de_be->reg_ctrl, SUNXI_DE_BE_REG_CTRL_LOAD_REGS);
  sunxi_setbits(&iomm.de_be->mode, SUNXI_DE_BE_MODE_START);
}

static void r61520_lcd_init(void)
{
  gpio_request(SLCD_PWM, "slcd_pwm");
  gpio_direction_output(SLCD_PWM, 1);
  gpio_set_value(SLCD_PWM, 1);
  
  gpio_request(SLCD_RESET, "slcd_reset");
  gpio_direction_output(SLCD_RESET, 0);
  gpio_set_value(SLCD_RESET, 0);
  mdelay(150);
  gpio_set_value(SLCD_RESET, 1);
  mdelay(50);

  r61520_lcd_cmd(0xB0);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xB1);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xB3);
  r61520_lcd_data(0x02);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xB4);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xc0);
  r61520_lcd_data(0x07);
  r61520_lcd_data(0x4F);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x01);
  r61520_lcd_data(0x33);

  r61520_lcd_cmd(0xc1);
  r61520_lcd_data(0x01);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x1A);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x08);

  r61520_lcd_cmd(0xc3);
  r61520_lcd_data(0x01);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x1A);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x08);

  r61520_lcd_cmd(0xc4);
  r61520_lcd_data(0x11);
  r61520_lcd_data(0x01);
  r61520_lcd_data(0x43);
  r61520_lcd_data(0x01);

  r61520_lcd_cmd(0xC8);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x8a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x09);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x23);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x60);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xC9);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x8a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x09);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x23);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x09);
  r61520_lcd_data(0x88);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x23);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xCA);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x8a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x09);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x23);
  r61520_lcd_data(0x10);
  r61520_lcd_data(0x05);
  r61520_lcd_data(0x09);
  r61520_lcd_data(0x88);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x08);
  r61520_lcd_data(0x0a);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x23);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0xD0);
  r61520_lcd_data(0x07);
  r61520_lcd_data(0xc6);
  r61520_lcd_data(0xdc);

  r61520_lcd_cmd(0xD1);
  r61520_lcd_data(0x54);
  r61520_lcd_data(0x0D);
  r61520_lcd_data(0x02);

  r61520_lcd_cmd(0xD2);
  r61520_lcd_data(0x63);
  r61520_lcd_data(0x24);

  r61520_lcd_cmd(0xD4);
  r61520_lcd_data(0x63);
  r61520_lcd_data(0x24);

  r61520_lcd_cmd(0xD8);
  r61520_lcd_data(0x07);
  r61520_lcd_data(0x07);

  r61520_lcd_cmd(0xE0);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0x13);

  r61520_lcd_cmd(0x20);

  r61520_lcd_cmd(0x35);
  r61520_lcd_data(0x00);

  r61520_lcd_cmd(0x44);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x30);

  r61520_lcd_cmd(0x36);
  r61520_lcd_data(0xE0);

  r61520_lcd_cmd(0x3A);
  r61520_lcd_data(0x55);

  r61520_lcd_cmd(0x2a);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x01);
  r61520_lcd_data(0x3f);

  r61520_lcd_cmd(0x2b);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0x00);
  r61520_lcd_data(0xef);

  r61520_lcd_cmd(0x11);
  r61520_lcd_cmd(0x29);
  r61520_lcd_cmd(0x2c);
}

void r61520_lcd_fill(uint16_t color)
{
  uint16_t x, y;

  for(y=0; y<240; y++){
    for(x=0; x<320; x++){
      r61520_lcd_data(color);
    }
  }
}

static void lcd_upload_frame_cpu(struct r61520fb_par *par)
{
  uint32_t i;
  const int total = par->mode.xres * par->mode.yres;
  uint16_t *p = par->vram_virt;
   
  r61520_lcd_cmd(0x2c);
  for(i=0; i<total; i++){
    r61520_lcd_data(*p++);
  }
}

static void lcd_refresh_work(struct work_struct *work)
{
  struct r61520fb_par *par = container_of(work, struct r61520fb_par, refresh_work.work);
 
  mutex_lock(&par->lock);
  if(par->is_enabled){
    lcd_upload_frame_cpu(par);
    schedule_delayed_work(&par->refresh_work, HZ / 60);
  }
  mutex_unlock(&par->lock);
}

static int fb_probe(struct platform_device *device)
{
  uint32_t ret, ulcm;
  struct r61520_lcdc_platform_data *fb_pdata = device->dev.platform_data;
  struct fb_videomode *lcdc_info;
  struct fb_info *r61520_fb_info;
  struct r61520fb_par *par;

  printk("%s++\n", __func__);
  if((fb_pdata == NULL) && (!device->dev.of_node)){
    dev_err(&device->dev, "can not get platform data\n");
    return -ENOENT;
  }
 
  lcdc_info = devm_kzalloc(&device->dev, sizeof(struct fb_videomode), GFP_KERNEL);
  if(lcdc_info == NULL){
    return -ENODEV;
  }
  lcdc_info->name = "320x240";
  lcdc_info->xres = 320;
  lcdc_info->yres = 240;
  lcdc_info->vmode = FB_VMODE_NONINTERLACED; 
  pm_runtime_enable(&device->dev);
  pm_runtime_get_sync(&device->dev);
   
  r61520_fb_info = framebuffer_alloc(sizeof(struct r61520fb_par), &device->dev);
  if(!r61520_fb_info){
    dev_dbg(&device->dev, "memory allocation failed for fb_info\n");
    ret = -ENOMEM;
    goto err_pm_runtime_disable;
  }
 
  par = r61520_fb_info->par;
  par->dev = &device->dev;
  par->bpp = 16;
   
  fb_videomode_to_var(&r61520fb_var, lcdc_info);
  printk("%s, xres: %d, yres:%d, bpp:%d\n", __func__, lcdc_info->xres, lcdc_info->yres, par->bpp);
 
  // allocate frame buffer
  par->vram_size = lcdc_info->xres * lcdc_info->yres * par->bpp;
  ulcm = lcm((lcdc_info->xres * par->bpp * 2) / 8, PAGE_SIZE); // double buffer
  par->vram_size = roundup(par->vram_size/8, ulcm);
  par->vram_size = par->vram_size;
  par->vram_virt = dma_alloc_coherent(NULL, par->vram_size, (resource_size_t*) &par->vram_phys, GFP_KERNEL | GFP_DMA);
  if(!par->vram_virt){
    dev_err(&device->dev, "GLCD: kmalloc for frame buffer(vram) failed\n");
    ret = -EINVAL;
    goto err_release_fb;
  }
 
  // swap video ram
  par->tram_virt = dma_alloc_coherent(NULL, par->vram_size, (resource_size_t*)&par->tram_phys, GFP_KERNEL | GFP_DMA);
  if(!par->tram_virt){
    dev_err(&device->dev, "GLCD: kmalloc for frame buffer(tram) failed\n");
    ret = -EINVAL;
    goto err_release_fb;
  }
 
  r61520_fb_info->screen_base = (char __iomem*) par->vram_virt;
  r61520fb_fix.smem_start    = par->vram_phys;
  r61520fb_fix.smem_len      = par->vram_size;
  r61520fb_fix.line_length   = (lcdc_info->xres * par->bpp) / 8;
 
  // allocate palette buffer
  par->v_palette_base = dma_alloc_coherent(NULL, PALETTE_SIZE, (resource_size_t*)&par->p_palette_base, GFP_KERNEL | GFP_DMA);
  if(!par->v_palette_base){
    dev_err(&device->dev, "GLCD: kmalloc for palette buffer failed\n");
    ret = -EINVAL;
    goto err_release_fb_mem;
  }
  memset(par->v_palette_base, 0, PALETTE_SIZE);

  //sunxi_frontend_init();
  //sunxi_backend_init();
  //sunxi_frontend_mode_set(par);
  //sunxi_backend_mode_set(par);
  //sunxi_frontend_enable();
  //sunxi_backend_enable();
  sunxi_lcdc_init(par);
  r61520_lcd_init();
 
  r61520fb_var.grayscale = 0;
  r61520fb_var.bits_per_pixel = par->bpp;
 
  // Initialize fbinfo
  r61520_fb_info->flags = FBINFO_FLAG_DEFAULT;
  r61520_fb_info->fix = r61520fb_fix;
  r61520_fb_info->var = r61520fb_var;
  r61520_fb_info->fbops = &r61520fb_ops;
  r61520_fb_info->pseudo_palette = par->pseudo_palette;
  r61520_fb_info->fix.visual = (r61520_fb_info->var.bits_per_pixel <= 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR; 
  ret = fb_alloc_cmap(&r61520_fb_info->cmap, PALETTE_SIZE, 0);
  if(ret){
    goto err_release_pl_mem;
  }
  r61520_fb_info->cmap.len = 32;
 
  // initialize var_screeninfo
  r61520fb_var.activate = FB_ACTIVATE_FORCE;
  fb_set_var(r61520_fb_info, &r61520fb_var);
  dev_set_drvdata(&device->dev, r61520_fb_info);
  if(register_framebuffer(r61520_fb_info) < 0){
    dev_err(&device->dev, "GLCD: Frame Buffer Registration Failed(/dev/fb0) !\n");
    ret = -EINVAL;
    goto err_dealloc_cmap;
  } 
  fb_prepare_logo(r61520_fb_info, 0);
  fb_show_logo(r61520_fb_info, 0);
	
	INIT_DELAYED_WORK(&par->refresh_work, lcd_refresh_work);
  schedule_delayed_work(&par->refresh_work, 0);
  printk("%s--\n", __func__);
  return 0;
 
err_dealloc_cmap:
  fb_dealloc_cmap(&r61520_fb_info->cmap);
 
err_release_pl_mem:
  dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base, par->p_palette_base);
 
err_release_fb_mem:
  dma_free_coherent(NULL, par->vram_size, par->tram_virt, par->tram_phys);
  dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);
 
err_release_fb:
  framebuffer_release(r61520_fb_info);
 
err_pm_runtime_disable:
  pm_runtime_put_sync(&device->dev);
  pm_runtime_disable(&device->dev); 
  return ret;
}
 
#ifdef CONFIG_PM
static int fb_suspend(struct platform_device *dev, pm_message_t state)
{
  struct fb_info *info = platform_get_drvdata(dev);
 
  console_lock();
  fb_set_suspend(info, 1);
  pm_runtime_put_sync(&dev->dev);
  console_unlock();
  return 0;
}
 
static int fb_resume(struct platform_device *dev)
{
  struct fb_info *info = platform_get_drvdata(dev);

  console_lock();
  pm_runtime_get_sync(&dev->dev);
  fb_set_suspend(info, 0);
  console_unlock();
  return 0;
}
#else
#define fb_suspend NULL
#define fb_resume NULL 
#endif
 
static const struct of_device_id fb_of_match[] = {{.compatible = "suniv-f1c500s,r61520", },{}};
MODULE_DEVICE_TABLE(of, fb_of_match);
 
static struct platform_driver fb_driver = {
  .probe = fb_probe,
  .remove = fb_remove,
  .suspend = fb_suspend,
  .resume = fb_resume,
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(fb_of_match),
  },
};

static void sunxi_ioremap(void)
{
  iomm.ccm = (struct sunxi_ccm_reg*)ioremap(SUNXI_CCM_BASE, 1024);
  iomm.lcdc = (struct sunxi_lcdc_reg*)ioremap(SUNXI_LCDC_BASE, 1024);
  iomm.gpio = (struct sunxi_gpio_reg*)ioremap(SUNXI_GPIO_BASE, 1024);
  iomm.de_fe = (struct sunxi_de_fe_reg*)ioremap(SUNXI_DE_FE_BASE, 1024);
  iomm.de_be = (struct sunxi_de_be_reg*)ioremap(SUNXI_DE_BE_BASE, 1024);
}

static void sunxi_iounmap(void)
{
  iounmap(iomm.ccm);
  iounmap(iomm.lcdc);
  iounmap(iomm.gpio);
  iounmap(iomm.de_fe);
  iounmap(iomm.de_be);
}

static int __init fb_init(void)
{
  printk("%s\n", __func__);
  sunxi_ioremap();
  return platform_driver_register(&fb_driver);
}
 
static void __exit fb_cleanup(void)
{
  printk("%s\n", __func__);
  sunxi_iounmap();
  platform_driver_unregister(&fb_driver);
}
 
module_init(fb_init);
module_exit(fb_cleanup);
 
MODULE_DESCRIPTION("suniv-f1c500s framebuffer driver for Renesas R61520 SLCD panel");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_LICENSE("GPL");

