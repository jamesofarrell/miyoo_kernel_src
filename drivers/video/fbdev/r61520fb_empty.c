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
#include <asm/arch-sunxi/display.h>

#define PALETTE_SIZE		256
#define DRIVER_NAME     "r61520_slcd"
#define SLCD_PWM				((32 * 4) + 6)    // PE6
#define SLCD_RESET			((32 * 4) + 11)   // PE11
 
struct r61520_fb_par {
  struct device *dev;
 
  resource_size_t p_palette_base;
  unsigned short *v_palette_base;
 
  dma_addr_t vram_phys;
  unsigned long vram_size;
  void *vram_virt;
 
  dma_addr_t tram_phys;
  void *tram_virt;
 
  dma_addr_t lram_phys[2];
  unsigned long lram_size;
  void *lram_virt[2];
 
  int irq;
  u32 pseudo_palette[16];
  struct fb_videomode mode;
  unsigned int bpp;

	uint32_t is_enabled;
  struct mutex lock;
	struct delayed_work refresh_work;
};

static struct fb_var_screeninfo r61520_fb_var;
static struct fb_fix_screeninfo r61520_fb_fix = {
  .id = "R61520 FB",
  .type = FB_TYPE_PACKED_PIXELS,
  .type_aux = 0,
  .visual = FB_VISUAL_TRUECOLOR,
  .xpanstep = 0,
  .ypanstep = 1,
  .ywrapstep = 0,
  .accel = FB_ACCEL_NONE
};

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
  struct r61520_fb_par *par = info->par;
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
    struct r61520_fb_par *par = info->par;
    unregister_framebuffer(info);
    fb_dealloc_cmap(&info->cmap);
    dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base, par->p_palette_base);
    dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);
    dma_free_coherent(NULL, par->lram_size, par->lram_virt[0], par->lram_phys[0]);
    dma_free_coherent(NULL, par->lram_size, par->lram_virt[1], par->lram_phys[1]);
    pm_runtime_put_sync(&dev->dev);
    pm_runtime_disable(&dev->dev);
    framebuffer_release(info);
  }
  return 0;
}
 
static int fb_set_par(struct fb_info *info)
{
  struct r61520_fb_par *par = info->par;
   
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
 
static struct fb_ops r61520_fb_ops = {
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
 
static void lcd_upload_frame_cpu(struct r61520_fb_par *par)
{
  //uint32_t i;
  //const int total = par->mode.xres * par->mode.yres;
  //uint16_t *p = par->vram_virt;
   
}

static void lcd_refresh_work(struct work_struct *work)
{
  struct r61520_fb_par *par = container_of(work, struct r61520_fb_par, refresh_work.work);
 
  mutex_lock(&par->lock);
  if(par->is_enabled){
    lcd_upload_frame_cpu(par);
    schedule_delayed_work(&par->refresh_work, HZ / 60);
  }
  mutex_unlock(&par->lock);
}

static int fb_probe(struct platform_device *device)
{
  int i, ret;
  uint32_t ulcm;
  struct r61520_lcdc_platform_data *fb_pdata = device->dev.platform_data;
  struct fb_videomode *lcdc_info;
  struct fb_info *r61520_fb_info;
  struct r61520_fb_par *par;

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
   
  r61520_fb_info = framebuffer_alloc(sizeof(struct r61520_fb_par), &device->dev);
  if(!r61520_fb_info){
    dev_dbg(&device->dev, "memory allocation failed for fb_info\n");
    ret = -ENOMEM;
    goto err_pm_runtime_disable;
  }
 
  par = r61520_fb_info->par;
  par->dev = &device->dev;
  par->bpp = 16;
   
  fb_videomode_to_var(&r61520_fb_var, lcdc_info);
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
 
  par->lram_size = 320 * 240 * 2; // fixed size for r61520 panel
  for(i=0; i<2; i++){
    par->lram_virt[i] = dma_alloc_coherent(NULL, par->lram_size, (resource_size_t*) &par->lram_phys[i], GFP_KERNEL | GFP_DMA);
    if(!par->lram_virt[i]){
      dev_err(&device->dev, "GLCD: kmalloc for frame buffer[%d](lram) failed\n", i);
      ret = -EINVAL;
      goto err_release_fb;
    }
    memset(par->lram_virt[i], 0, par->lram_size);
  }
  r61520_fb_info->screen_base = (char __iomem*) par->vram_virt;
  r61520_fb_fix.smem_start    = par->vram_phys;
  r61520_fb_fix.smem_len      = par->vram_size;
  r61520_fb_fix.line_length   = (lcdc_info->xres * par->bpp) / 8;
 
  // allocate palette buffer
  par->v_palette_base = dma_alloc_coherent(NULL, PALETTE_SIZE, (resource_size_t*)&par->p_palette_base, GFP_KERNEL | GFP_DMA);
  if(!par->v_palette_base){
    dev_err(&device->dev, "GLCD: kmalloc for palette buffer failed\n");
    ret = -EINVAL;
    goto err_release_fb_mem;
  }
  memset(par->v_palette_base, 0, PALETTE_SIZE);

  // lcd init

  r61520_fb_var.grayscale = 0;
  r61520_fb_var.bits_per_pixel = par->bpp;
 
  // initialize fbinfo
  r61520_fb_info->flags = FBINFO_FLAG_DEFAULT;
  r61520_fb_info->fix = r61520_fb_fix;
  r61520_fb_info->var = r61520_fb_var;
  r61520_fb_info->fbops = &r61520_fb_ops;
  r61520_fb_info->pseudo_palette = par->pseudo_palette;
  r61520_fb_info->fix.visual = (r61520_fb_info->var.bits_per_pixel <= 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR; 
  ret = fb_alloc_cmap(&r61520_fb_info->cmap, PALETTE_SIZE, 0);
  if(ret){
    goto err_release_pl_mem;
  }
  r61520_fb_info->cmap.len = 32;
 
  // initialize var_screeninfo
  r61520_fb_var.activate = FB_ACTIVATE_FORCE;
  fb_set_var(r61520_fb_info, &r61520_fb_var);
  dev_set_drvdata(&device->dev, r61520_fb_info);

  // Register the Frame Buffer
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
 
static int __init fb_init(void)
{
  printk("%s\n", __func__);
  return platform_driver_register(&fb_driver);
}
 
static void __exit fb_cleanup(void)
{
  printk("%s\n", __func__);
  platform_driver_unregister(&fb_driver);
}
 
module_init(fb_init);
module_exit(fb_cleanup);
 
MODULE_DESCRIPTION("suniv-f1c500s framebuffer driver for Renesas R61520 SLCD panel");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_LICENSE("GPL");

