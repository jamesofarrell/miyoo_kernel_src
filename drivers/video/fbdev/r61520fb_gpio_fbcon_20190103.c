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
#include <linux/cdev.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch-sunxi/dma.h>
#include <asm/arch-sunxi/cpu.h>
#include <asm/arch-sunxi/gpio.h>
#include <asm/arch-sunxi/lcdc.h>
#include <asm/arch-sunxi/clock.h>
#include <asm/arch-sunxi/common.h>

#define PALETTE_SIZE		      256
#define DRIVER_NAME           "r61520_slcd"
#define SLCD_PWM				      ((32 * 4) + 6)    // PE6
#define SLCD_RESET			      ((32 * 4) + 11)   // PE11
#define IOCTL_BASE            0x100
#define IOCTL_TURN_ON_WORK    _IOW (IOCTL_BASE, 1, unsigned long)
#define IOCTL_TURN_OFF_WORK   _IOW (IOCTL_BASE, 2, unsigned long)
#define IOCTL_GET_GPIO_PTR    _IOW (IOCTL_BASE, 3, unsigned long)

struct r61520fb_par {
  struct device *dev;
 
  resource_size_t p_palette_base;
  unsigned short *v_palette_base;
 
  dma_addr_t vram_phys;
  uint32_t vram_size;
  void *vram_virt;
 
  dma_addr_t tram_phys;
  void *tram_virt;
  uint32_t tram_index;
 
  int irq;
  u32 pseudo_palette[16];
  struct fb_videomode mode;
  uint16_t bpp;

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
  uint8_t *dma;
  uint8_t *ccm;
  uint8_t *gpio;
};
static struct sunxi_iomm iomm={0};
static struct r61520fb_par *fbpar;

static int major = -1;
static struct cdev mycdev;
static struct class *myclass = NULL;
static int work_stop=0;

static void clrbits(void __iomem *reg, u32 clr_val)
{
	uint32_t reg_val;

	reg_val = readl(reg);
	reg_val&= ~(clr_val);
	writel(reg_val, reg);
}

static void setbits(void __iomem *reg, u32 set_val)
{
	uint32_t reg_val;

	reg_val = readl(reg);
	reg_val|= set_val;
	writel(reg_val, reg);
}

static void clrsetbits(void __iomem *reg, u32 clr_val, u32 set_val)
{
	uint32_t reg_val;

	reg_val = readl(reg);
	reg_val&= ~(clr_val);
	reg_val|= set_val;
	writel(reg_val, reg);
}

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
 
  if((var->xres != 320) || (var->yres != 240) || (var->bits_per_pixel != 16)){
    return -EINVAL;
  }
 
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
 
static void sunxi_lcdc_output(uint32_t is_data, uint32_t val)
{
  uint32_t ret;
  uint32_t *src = fbpar->tram_virt;

  ret = (val & 0x00ff) << 1;
  ret|= (val & 0xff00) << 2;
  ret|= is_data ? 0x80000 : 0;
  ret|= 0x100000;
  //writel(ret, iomm.gpio + PD_DATA);
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  ret|= 0x40000;
  //writel(ret, iomm.gpio + PD_DATA);
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
  src[fbpar->tram_index++] = ret;
}

static void r61520_lcd_cmd(uint32_t val)
{
  sunxi_lcdc_output(0, val);
}

static void r61520_lcd_dat(uint32_t val)
{
  sunxi_lcdc_output(1, val);
}

static void sunxi_lcdc_gpio_config(uint32_t use_gpio)
{
  writel(0x11111111, iomm.gpio + PD_CFG0); // 0x11111117
  writel(0x11111111, iomm.gpio + PD_CFG1); // 0x11111171
  writel(0x00111111, iomm.gpio + PD_CFG2); // 0x00111111, CS/RD/RS/WR
  writel(0xffffffff, iomm.gpio + PD_DATA);
}


static void sunxi_dma_init(const struct r61520fb_par *par)
{
  uint32_t ret;

  fbpar->tram_index = 0;
  setbits(iomm.ccm + BUS_CLK_GATING_REG0, (1 << 6));
  setbits(iomm.ccm + BUS_SOFT_RST_REG0, (1 << 6));
  writel(0x00000000, iomm.dma + DMA_INT_CTRL_REG);
  writel(0xffffffff, iomm.dma + DMA_INT_STA_REG);
  writel(0x1e310211, iomm.dma + NDMA0_CFG_REG);
  writel(0x00000002, iomm.dma + DMA_INT_CTRL_REG);
  writel(par->tram_phys, iomm.dma + NDMA0_SRC_ADR_REG);
  writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + NDMA0_DES_ADR_REG);

  #if 0
    // testing ram
    uint32_t i;
    uint8_t *s = par->tram_virt;
    uint8_t *d = par->tram_virt + 16;
    printk("src addr: 0x%x\n", s);
    printk("dst addr: 0x%x\n", d);
    for(i=0; i<16; i++){
      s[i] = i;
    }
    writel(par->tram_phys, iomm.dma + NDMA0_SRC_ADR_REG);
    writel(par->tram_phys + 16, iomm.dma + NDMA0_DES_ADR_REG);
    writel(16, iomm.dma + NDMA0_BYTE_CNT_REG);
    writel(2, iomm.dma + DMA_INT_STA_REG);
    setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
    for(i=0; i<16; i++){
      printk("0x%x ", d[i]);
    }
  #endif
 
  #if 0
    // testing gpio
    uint32_t i;
    uint32_t *s = par->tram_virt;
    for(i=0; i<22; i++){
      s[0] = ~(1 << i);
      writel(4, iomm.dma + NDMA0_BYTE_CNT_REG);
      writel(2, iomm.dma + DMA_INT_STA_REG);
      setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
      mdelay(3000);
    }
  #endif
}

static void sunxi_cpu_init(void)
{
  uint32_t ret;

  // 768MHz
  ret = readl(iomm.ccm + PLL_CPU_CTRL_REG);
  writel(ret | 0x00001f00, iomm.ccm + PLL_CPU_CTRL_REG); 
  printk("%s, 0x%08x PLL_CPU_CTRL_REG\n", __func__, readl(iomm.ccm + PLL_CPU_CTRL_REG));
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

  r61520_lcd_cmd(0xb0);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xb1);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xb3);
  r61520_lcd_dat(0x02);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xb4);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xc0);
  r61520_lcd_dat(0x07);
  r61520_lcd_dat(0x4f);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x01);
  r61520_lcd_dat(0x33);

  r61520_lcd_cmd(0xc1);
  r61520_lcd_dat(0x01);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x1a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x08);

  r61520_lcd_cmd(0xc3);
  r61520_lcd_dat(0x01);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x1a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x08);

  r61520_lcd_cmd(0xc4);
  r61520_lcd_dat(0x11);
  r61520_lcd_dat(0x01);
  r61520_lcd_dat(0x43);
  r61520_lcd_dat(0x01);

  r61520_lcd_cmd(0xc8);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x8a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x09);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x23);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x60);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xc9);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x8a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x09);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x23);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x09);
  r61520_lcd_dat(0x88);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x23);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xca);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x8a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x09);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x23);
  r61520_lcd_dat(0x10);
  r61520_lcd_dat(0x05);
  r61520_lcd_dat(0x09);
  r61520_lcd_dat(0x88);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x08);
  r61520_lcd_dat(0x0a);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x23);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0xd0);
  r61520_lcd_dat(0x07);
  r61520_lcd_dat(0xc6);
  r61520_lcd_dat(0xdc);

  r61520_lcd_cmd(0xd1);
  r61520_lcd_dat(0x54);
  r61520_lcd_dat(0x0d);
  r61520_lcd_dat(0x02);

  r61520_lcd_cmd(0xd2);
  r61520_lcd_dat(0x63);
  r61520_lcd_dat(0x24);

  r61520_lcd_cmd(0xd4);
  r61520_lcd_dat(0x63);
  r61520_lcd_dat(0x24);

  r61520_lcd_cmd(0xd8);
  r61520_lcd_dat(0x07);
  r61520_lcd_dat(0x07);

  r61520_lcd_cmd(0xe0);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0x13);

  r61520_lcd_cmd(0x20);

  r61520_lcd_cmd(0x35);
  r61520_lcd_dat(0x00);

  r61520_lcd_cmd(0x44);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x30);

  r61520_lcd_cmd(0x36);
  r61520_lcd_dat(0xe0);

  r61520_lcd_cmd(0x3a);
  r61520_lcd_dat(0x55);

  r61520_lcd_cmd(0x2a);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x01);
  r61520_lcd_dat(0x3f);

  r61520_lcd_cmd(0x2b);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0x00);
  r61520_lcd_dat(0xef);

  r61520_lcd_cmd(0x11);
  r61520_lcd_cmd(0x29);
  r61520_lcd_cmd(0x2c);
  
  writel(fbpar->tram_index * 4, iomm.dma + NDMA0_BYTE_CNT_REG);
  writel(0x00000002, iomm.dma + DMA_INT_CTRL_REG);
  setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
  fbpar->tram_index = 0;
}

static void lcd_upload_frame_cpu(struct r61520fb_par *par)
{
  uint16_t *p = par->vram_virt;
  uint32_t i, total = par->mode.xres * par->mode.yres;

  r61520_lcd_cmd(0x2c);
  for(i=0; i<total; i++){
    r61520_lcd_dat(*p++);
  }
  writel(par->tram_index * 4, iomm.dma + NDMA0_BYTE_CNT_REG);
  writel(0x00000002, iomm.dma + DMA_INT_CTRL_REG);
  setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
  par->tram_index = 0;
}

static void lcd_refresh_work(struct work_struct *work)
{
  struct r61520fb_par *par = container_of(work, struct r61520fb_par, refresh_work.work);

  if(work_stop == 0){
    mutex_lock(&par->lock);
    //lcd_upload_frame_cpu(par);
    schedule_delayed_work(&par->refresh_work, HZ / 60);
    mutex_unlock(&par->lock);
  }
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
  fbpar = par;
   
  fb_videomode_to_var(&r61520fb_var, lcdc_info);
  printk("%s, xres: %d, yres:%d, bpp:%d\n", __func__, lcdc_info->xres, lcdc_info->yres, par->bpp);
 
  // allocate frame buffer
  par->vram_size = lcdc_info->xres * lcdc_info->yres * par->bpp;
  ulcm = lcm((lcdc_info->xres * par->bpp * 2) / 8, PAGE_SIZE);
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

  sunxi_lcdc_gpio_config(1); 
  sunxi_cpu_init();
  sunxi_dma_init(par);
  r61520_lcd_init();
   
  // Initialize fbinfo
  r61520fb_var.grayscale = 0;
  r61520fb_var.bits_per_pixel = par->bpp;
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
  iomm.dma = (uint8_t*)ioremap(SUNXI_DMA_BASE, 4096);
  iomm.ccm = (uint8_t*)ioremap(SUNXI_CCM_BASE, 4096);
  iomm.gpio = (uint8_t*)ioremap(SUNXI_GPIO_BASE, 4096);
}

static void sunxi_iounmap(void)
{
  iounmap(iomm.dma);
  iounmap(iomm.ccm);
  iounmap(iomm.gpio);
}

static int efb_open_close(struct inode *inode, struct file *file)
{
  printk("%s\n", __func__);
  return 0;
}

static long efb_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  unsigned long ptr;

  printk("%s, cmd: 0x%x\n", __func__, cmd);
  switch(cmd){
  case IOCTL_TURN_ON_WORK:
    work_stop = 0;
    break;
  case IOCTL_TURN_OFF_WORK:
    work_stop = 1;
    break;
  }
  return 0;
}

static int efb_write(struct file* filep, const char __user *buff, size_t count, loff_t *offp)
{
  uint32_t cnt;
  uint16_t *p = (uint16_t*)fbpar->tram_virt;
  
  copy_from_user(fbpar->tram_virt, buff, count);

	r61520_lcd_cmd(0x2c);
  count/= 2;
  for(cnt=0; cnt<count; cnt++){
	  r61520_lcd_dat(p[cnt]);
  }
	return count;
}
 
static const struct file_operations fops =
{
  .owner = THIS_MODULE,
  .open = efb_open_close,
  .release = efb_open_close,
  .write = efb_write,
  .unlocked_ioctl = efb_ioctl,
};

static void efb_clean_node(void)
{
  if(myclass)
  {
    device_destroy(myclass, major);
    cdev_del(&mycdev);
    class_destroy(myclass);
  }
  if(major != -1)
  {
    unregister_chrdev_region(major, 1);
  }
}

static int __init efb_create_node(void)
{
  // /proc/devices
  if(alloc_chrdev_region(&major, 0, 1, DRIVER_NAME) < 0){
    goto error;
  }
  // /sys/class
  if((myclass = class_create(THIS_MODULE, DRIVER_NAME)) == NULL){
    goto error;
  }
  // /dev/
  if(device_create(myclass, NULL, major, NULL, DRIVER_NAME) == NULL){
    goto error;
  }
  cdev_init(&mycdev, &fops);
  if(cdev_add(&mycdev, major, 1) == -1){
    goto error;
  }
  printk("%s, create %s successfully\n", __func__, DRIVER_NAME);
  return 0;

error:
  efb_clean_node();
  printk("%s, failed to create device node\n", __func__);
  return -1;
}

static int __init fb_init(void)
{
  printk("%s\n", __func__);
  sunxi_ioremap();
  efb_create_node();
  return platform_driver_register(&fb_driver);
}
 
static void __exit fb_cleanup(void)
{
  printk("%s\n", __func__);
  sunxi_iounmap();
  efb_clean_node();
  platform_driver_unregister(&fb_driver);
}
 
module_init(fb_init);
module_exit(fb_cleanup);
 
MODULE_DESCRIPTION("suniv-f1c500s framebuffer driver for Renesas R61520 SLCD panel");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_LICENSE("GPL");

