/*
 * Copyright (C) 2018 Steward Fu <steward.fu@gmail.com>
 *
 * Framebuffer driver for Renesas R61520 panel
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
#include <asm/arch-suniv/dma.h>
#include <asm/arch-suniv/cpu.h>
#include <asm/arch-suniv/gpio.h>
#include <asm/arch-suniv/intc.h>
#include <asm/arch-suniv/lcdc.h>
#include <asm/arch-suniv/debe.h>
#include <asm/arch-suniv/clock.h>
#include <asm/arch-suniv/common.h>

#define DMA_NUM       4
#define PALETTE_SIZE  256
#define DRIVER_NAME   "miyoofb"

#define MIFB_BASE                   0x100
#define MIFB_GET_DMA                _IOWR (MIFB_BASE, 1, unsigned long)
#define MIFB_REFILL_MODE            _IOWR (MIFB_BASE, 2, unsigned long)
#define MIFB_FLUSH_DMA              _IOWR (MIFB_BASE, 3, unsigned long)
#define MIFB_REFILL_FULL_AUTO       0
#define MIFB_REFILL_HALF_AUTO       1
#define MIFB_REFILL_MANUAL_UPDATE   2

struct myfb_par {
  struct device *dev;
 
  resource_size_t p_palette_base;
  unsigned short *v_palette_base;
 
  dma_addr_t vram_phys;
  uint32_t vram_size;
  void *vram_virt;
 
  dma_addr_t tram_phys;
  void *tram_virt;
 
  dma_addr_t lram_phys[DMA_NUM];
  uint32_t lram_size;
  void *lram_virt[DMA_NUM];
  uint32_t dma_ch;
  uint32_t refill_mode;

  uint32_t dma_irq;
  uint32_t lcdc_irq;
  u32 pseudo_palette[16];
  struct fb_videomode mode;
  unsigned int bpp;  
};

struct sunxi_iomm {
  uint8_t *dma;
  uint8_t *ccm;
  uint8_t *gpio;
  uint8_t *lcdc;
  uint8_t *debe;
};
static struct sunxi_iomm iomm={0};

static struct fb_var_screeninfo myfb_var;
static struct fb_fix_screeninfo myfb_fix = {
  .id = DRIVER_NAME,
  .type = FB_TYPE_PACKED_PIXELS,
  .type_aux = 0,
  .visual = FB_VISUAL_TRUECOLOR,
  .xpanstep = 0,
  .ypanstep = 1,
  .ywrapstep = 0,
  .accel = FB_ACCEL_NONE
};

static void sunxi_dma_start(struct myfb_par *par)
{
  //printk("%s, ch:%d\n", __func__, par->dma_ch);
  switch(par->dma_ch){
  case 0:
    suniv_setbits(iomm.dma + DDMA0_CFG_REG, (1 << 31));
    break;
  case 1:
    suniv_setbits(iomm.dma + DDMA1_CFG_REG, (1 << 31));
    break;
  case 2:
    suniv_setbits(iomm.dma + DDMA2_CFG_REG, (1 << 31));
    break;
  case 3:
    suniv_setbits(iomm.dma + DDMA3_CFG_REG, (1 << 31));
    break;
  }
}

static void sunxi_dma_fill(struct myfb_par *par)
{
  uint32_t ret, i, val, cnt=0;
  uint32_t ch = par->dma_ch;
  uint16_t *src = par->vram_virt;
  uint32_t *dst = par->lram_virt[ch];
  uint32_t total = par->mode.xres * par->mode.yres;

  //printk("%s, ch:%d, x:%d, y:%d\n", __func__, ch, par->mode.xres, par->mode.yres);
  val = 0x2c;
  ret = (val & 0x00ff) << 1;
  ret|= (val & 0xff00) << 2;
  ret|= 0;
  ret|= 0x100000;
  dst[cnt++] = ret;
  ret|= 0x40000;
  dst[cnt++] = ret;
  for(i=0; i<total; i++){
    val = *src++;
    ret = (val & 0x00ff) << 1;
    ret|= (val & 0xff00) << 2;
    ret|= 0x80000;
    ret|= 0x100000;
    dst[cnt++] = ret;
    ret|= 0x40000;
    dst[cnt++] = ret;
  }
}

static void sunxi_lcdc_gpio_config(uint32_t use_gpio)
{
  uint32_t ret;

  if(use_gpio){
    writel(0x11111111, iomm.gpio + PD_CFG0); // 0x11111117
    writel(0x11111111, iomm.gpio + PD_CFG1); // 0x11111171
    writel(0x00111111, iomm.gpio + PD_CFG2); // 0x00111111, CS/RD/RS/WR
    writel(0xffffffff, iomm.gpio + PD_DATA);  
  }
  else{
    writel(0x22222222, iomm.gpio + PD_CFG0); // 0x22222227
    writel(0x22222222, iomm.gpio + PD_CFG1); // 0x22222272
    writel(0x00222222, iomm.gpio + PD_CFG2); // 0x00222222, CS/RD/RS/WR
    writel(0x00000000, iomm.gpio + PD_PUL0);
    writel(0x00000000, iomm.gpio + PD_PUL1);
    writel(0xffffffff, iomm.gpio + PD_DRV0);
    writel(0xffffffff, iomm.gpio + PD_DRV1);
  }
  ret = readl(iomm.gpio + PE_CFG1); // tear pin with interrupt
  ret&= 0xfffff0ff;
  ret|= 0x00000000; // 0:input, 6:int
  writel(ret, iomm.gpio + PE_CFG1);
  ret = readl(iomm.gpio + PE_PUL0); // pull-up
  ret&= 0xffcfffff;
  ret|= 0x00100000;
  writel(ret, iomm.gpio + PE_PUL0);
  /*ret = readl(iomm.gpio + PE_INT_CFG1); // raising edge
  ret&= 0xfffff0ff;
  writel(ret, iomm.gpio + PE_INT_CFG1);
  suniv_setbits(iomm.gpio + PE_INT_CTRL, (1 << 10)); // enable interrupt
  writel(0xffffffff, iomm.gpio + PE_INT_STA);*/
}

#if 1
static uint32_t lcdc_wait_busy(void)
{
	uint32_t cnt=0;
	
  suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 0));
	ndelay(10);
	while(1){
		if(readl(iomm.lcdc + TCON0_CPU_IF_REG) & 0x00c00000){
			if(cnt > 200){
				return -1;
			}
		  else{
		  	cnt+= 1;
			}
		}
		break;
	}
	return 0;
}

static uint32_t extend_16b_to_24b(uint32_t value)
{
	return ((value & 0xfc00) << 8) | ((value & 0x0300) << 6) | ((value & 0x00e0) << 5) | ((value & 0x001f) << 3);
}

static irqreturn_t lcdc_irq_handler(int irq, void *arg)
{
  printk("%s, 0x%08x\n", __func__, readl(iomm.lcdc + TCON_INT_REG0));
  suniv_clrbits(iomm.lcdc + TCON_INT_REG0, (1 << 15));
  return IRQ_HANDLED;
}

static void lcdc_cmd(uint32_t cmd)
{
	while(lcdc_wait_busy());
	suniv_clrbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 25)); // ca=0
	while(lcdc_wait_busy());
	writel(extend_16b_to_24b(cmd), iomm.lcdc + TCON0_CPU_WR_REG);
}

static void lcdc_dat(uint32_t data)
{
	while(lcdc_wait_busy());
	suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 25)); // ca=1
	while(lcdc_wait_busy());
	writel(extend_16b_to_24b(data), iomm.lcdc + TCON0_CPU_WR_REG);
}

static void sunxi_lcdc_init(struct myfb_par *par)
{
  uint32_t ret, i, bp, total;
	uint32_t pixel_clock_hz = 9000000;
	uint32_t h_front_porch = 40;
	uint32_t h_back_porch = 87;
	uint32_t h_sync_len = 1;
	uint32_t v_front_porch = 13;
	uint32_t v_back_porch = 31;
	uint32_t v_sync_len = 1;
  
  printk("%s++\n", __func__);
  suniv_setbits(iomm.ccm + PLL_VIDEO_CTRL_REG, (1 << 31));
  while(!(readl(iomm.ccm + PLL_VIDEO_CTRL_REG) & (1 << 28)));
  printk("%s, 0x%08x PLL_VIDEO_CTRL_REG\n", __func__, readl(iomm.ccm + PLL_VIDEO_CTRL_REG));

  suniv_setbits(iomm.ccm + PLL_PERIPH_CTRL_REG, (1 << 31));
  while(!(readl(iomm.ccm + PLL_PERIPH_CTRL_REG) & (1 << 28)));
  printk("%s, 0x%08x PLL_PERIPH_CTRL_REG\n", __func__, readl(iomm.ccm + PLL_PERIPH_CTRL_REG));

  suniv_setbits(iomm.ccm + FE_CLK_REG, (1 << 31));
  printk("%s, 0x%08x FE_CLK_REG\n", __func__, readl(iomm.ccm + FE_CLK_REG));
  suniv_setbits(iomm.ccm + BE_CLK_REG, (1 << 31));
  printk("%s, 0x%08x BE_CLK_REG\n", __func__, readl(iomm.ccm + BE_CLK_REG));
  suniv_setbits(iomm.ccm + DI_CLK_REG, (1 << 31));
  printk("%s, 0x%08x DI_CLK_REG\n", __func__, readl(iomm.ccm + DI_CLK_REG));
  suniv_setbits(iomm.ccm + TCON_CLK_REG, (1 << 31));
  printk("%s, 0x%08x TCON_CLK_REG\n", __func__, readl(iomm.ccm + TCON_CLK_REG));
  suniv_setbits(iomm.ccm + BUS_CLK_GATING_REG1, (1 << 14) | (1 << 12) | (1 << 5) | (1 << 4));
  printk("%s, 0x%08x BUS_CLK_GATING_REG1\n", __func__, readl(iomm.ccm + BUS_CLK_GATING_REG1));

	// disable tcon
	writel(0, iomm.lcdc + TCON_CTRL_REG);
	writel(0, iomm.lcdc + TCON_INT_REG0);
	ret = readl(iomm.lcdc + TCON_CLK_CTRL_REG);
	ret&= ~(0xf << 28);
	writel(ret, iomm.lcdc + TCON_CLK_CTRL_REG);
	writel(0xffffffff, iomm.lcdc + TCON0_IO_CTRL_REG1);
	writel(0xffffffff, iomm.lcdc + TCON1_IO_CTRL_REG1);

	// set debe
  for(i=0x0800; i<0x1000; i+=4){
    writel(0, iomm.debe + i);
  }
	ret = readl(iomm.debe + DEBE_MODE_CTRL_REG);
	ret|= (1 << 0);
	writel(ret, iomm.debe + DEBE_MODE_CTRL_REG);
	writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_DISP_SIZE_REG);
	writel((((par->mode.yres) - 1) << 16) | (((par->mode.xres) - 1) << 0), iomm.debe + DEBE_LAY0_SIZE_REG);
	writel(par->mode.xres << 4, iomm.debe + DEBE_LAY0_LINEWIDTH_REG);
	writel((uint32_t)(par->vram_phys) << 3, iomm.debe + DEBE_LAY0_FB_ADDR_REG);
	writel((uint32_t)(par->vram_phys) >> 29, iomm.debe + DEBE_LAY0_FB_HI_ADDR_REG);
	writel(0x05 << 8, iomm.debe + DEBE_LAY0_ATT_CTRL_REG1);
	ret = readl(iomm.debe + DEBE_MODE_CTRL_REG);
	ret|= (1 << 8);
	writel(ret, iomm.debe + DEBE_MODE_CTRL_REG);
	ret = readl(iomm.debe + DEBE_REGBUFF_CTRL_REG);
	ret|= (1 << 0);
	writel(ret, iomm.debe + DEBE_REGBUFF_CTRL_REG);
  ret = readl(iomm.debe + DEBE_MODE_CTRL_REG);
	ret|= (1 << 1);
	writel(ret, iomm.debe + DEBE_MODE_CTRL_REG);
  printk("%s, 0x%08x DEBE_MODE_CTRL_REG\n", __func__, readl(iomm.debe + DEBE_MODE_CTRL_REG));
  printk("%s, 0x%08x DEBE_LAY0_SIZE_REG\n", __func__, readl(iomm.debe + DEBE_LAY0_SIZE_REG));
  printk("%s, 0x%08x DEBE_LAY0_LINEWIDTH_REG\n", __func__, readl(iomm.debe + DEBE_LAY0_LINEWIDTH_REG));
  printk("%s, 0x%08x DEBE_LAY0_FB_ADDR_REG\n", __func__, readl(iomm.debe + DEBE_LAY0_FB_ADDR_REG));
  printk("%s, 0x%08x DEBE_REGBUFF_CTRL_REG\n", __func__, readl(iomm.debe + DEBE_REGBUFF_CTRL_REG));

  // set tcon cpu mode
	ret = readl(iomm.lcdc + TCON_CTRL_REG);
	ret&= ~(0x1 << 0);
	writel(ret, iomm.lcdc + TCON_CTRL_REG);
	ret = (v_front_porch + v_back_porch + v_sync_len);
	writel((1 << 31) | ((ret & 0x1f) << 4) | (0x01 << 24), iomm.lcdc + TCON0_CTRL_REG);
	ret = 279000000 / pixel_clock_hz;
	writel((0xf << 28) | (ret << 0), iomm.lcdc + TCON_CLK_CTRL_REG);
	writel(((par->mode.xres - 1) << 16) | ((par->mode.yres - 1) << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG0);
	bp = h_sync_len + h_back_porch;
	total = par->mode.xres * 3 + h_front_porch + bp;
	writel(((total - 1) << 16) | ((bp - 1) << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG1);
	bp = v_sync_len + v_back_porch;
	total = par->mode.yres + v_front_porch + bp;
	writel(((total * 2) << 16) | ((bp - 1) << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG2);
	writel(((h_sync_len - 1) << 16) | ((v_sync_len - 1) << 0), iomm.lcdc + TCON0_BASIC_TIMING_REG3);
	writel(0, iomm.lcdc + TCON0_HV_TIMING_REG);
	writel((0x4 << 29) | (1 << 26), iomm.lcdc + TCON0_CPU_IF_REG);
  /*writel(0x11111111, iomm.lcdc + TCON_FRM_SEED0_R_REG);
  writel(0x11111111, iomm.lcdc + TCON_FRM_SEED0_G_REG);
  writel(0x11111111, iomm.lcdc + TCON_FRM_SEED0_B_REG);
  writel(0x11111111, iomm.lcdc + TCON_FRM_SEED1_R_REG);
  writel(0x11111111, iomm.lcdc + TCON_FRM_SEED1_G_REG);
  writel(0x11111111, iomm.lcdc + TCON_FRM_SEED1_B_REG);
  writel(0x01010000, iomm.lcdc + TCON_FRM_TBL_REG0);
  writel(0x15151111, iomm.lcdc + TCON_FRM_TBL_REG1);
  writel(0x57575555, iomm.lcdc + TCON_FRM_TBL_REG2);
  writel(0x7f7f7777, iomm.lcdc + TCON_FRM_TBL_REG3);
  writel((1 << 31) | (5 << 4), iomm.lcdc + TCON_FRM_CTRL_REG);*/
	writel(1 << 28, iomm.lcdc + TCON0_IO_CTRL_REG0);
	writel(0, iomm.lcdc + TCON0_IO_CTRL_REG1);	
	par->lcdc_irq = 29;
	ret = request_irq(par->lcdc_irq, lcdc_irq_handler, IRQF_SHARED, "miyoo_lcdc_irq", par);
	if(ret){
		printk("%s, failed to register LCDC interrupt\n", __func__);
	}
	
	// enable tcon
	suniv_setbits(iomm.lcdc + TCON_CTRL_REG, (1 << 31));
	suniv_setbits(iomm.lcdc + TCON_INT_REG0, (1 << 31));
	//suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 28));
	suniv_setbits(iomm.lcdc + TCON0_CPU_IF_REG, (1 << 27));

  printk("%s, 0x%08x TCON_CTRL_REG\n", __func__, readl(iomm.lcdc + TCON_CTRL_REG));
  printk("%s, 0x%08x TCON0_CTRL_REG\n", __func__, readl(iomm.lcdc + TCON0_CTRL_REG));
  printk("%s, 0x%08x TCON_CLK_CTRL_REG\n", __func__, readl(iomm.lcdc + TCON_CLK_CTRL_REG));
  printk("%s, 0x%08x TCON0_BASIC_TIMING_REG0\n", __func__, readl(iomm.lcdc + TCON0_BASIC_TIMING_REG0));
  printk("%s, 0x%08x TCON0_BASIC_TIMING_REG1\n", __func__, readl(iomm.lcdc + TCON0_BASIC_TIMING_REG1));
  printk("%s, 0x%08x TCON0_BASIC_TIMING_REG2\n", __func__, readl(iomm.lcdc + TCON0_BASIC_TIMING_REG2));
  printk("%s, 0x%08x TCON0_BASIC_TIMING_REG3\n", __func__, readl(iomm.lcdc + TCON0_BASIC_TIMING_REG3));
  printk("%s, 0x%08x TCON0_HV_TIMING_REG\n", __func__, readl(iomm.lcdc + TCON0_HV_TIMING_REG));
  printk("%s, 0x%08x TCON0_CPU_IF_REG\n", __func__, readl(iomm.lcdc + TCON0_CPU_IF_REG));
  printk("%s, 0x%08x TCON0_IO_CTRL_REG0\n", __func__, readl(iomm.lcdc + TCON0_IO_CTRL_REG0));
  printk("%s, 0x%08x TCON0_IO_CTRL_REG1\n", __func__, readl(iomm.lcdc + TCON0_IO_CTRL_REG1));
  printk("%s++\n", __func__);
   
  #if 0
  {
    lcdc_cmd(0x2c);
    for(ret=0; ret<320*240; ret++){
      lcdc_dat(0xf800);
    }
  }
  #endif
}
#endif

static void wait_te_ready(void)
{
  uint32_t ret = readl(iomm.gpio + PE_DATA) & (1 << 10);

  if(ret == 1){
    while(readl(iomm.gpio + PE_DATA) & (1 << 10));
  }
  while(!(readl(iomm.gpio + PE_DATA) & (1 << 10)));
}

static irqreturn_t dma_irq_handler(int irq, void *arg)
{
  uint32_t ret;
  struct myfb_par *par = (struct myfb_par*)arg;

  ret = readl(iomm.dma + DMA_INT_STA_REG);
  writel(0xffffffff, iomm.dma + DMA_INT_STA_REG);
  if(ret & 0x00020000){
    switch(par->refill_mode){
    case MIFB_REFILL_FULL_AUTO:
      sunxi_dma_fill(par);
      sunxi_dma_start(par);
      break;
    case MIFB_REFILL_HALF_AUTO:
      //wait_te_ready();
      sunxi_dma_start(par);
      break;
    default:
      break;
    }
  }
  else if(ret & 0x00000002){
    printk("%s, normal dma0\n", __func__);
  }
  return IRQ_HANDLED;
}

static void sunxi_dma_init(struct myfb_par *par)
{
  uint32_t ret;

  par->dma_ch = 0;
  par->refill_mode = MIFB_REFILL_FULL_AUTO;
  //par->refill_mode = MIFB_REFILL_HALF_AUTO;
  suniv_setbits(iomm.ccm + BUS_CLK_GATING_REG0, (1 << 6));
  suniv_setbits(iomm.ccm + BUS_SOFT_RST_REG0, (1 << 6));
  writel(0x00000000, iomm.dma + DMA_INT_CTRL_REG);
  writel(0xffffffff, iomm.dma + DMA_INT_STA_REG);
  writel(0xffffffff, iomm.dma + DMA_INT_CTRL_REG);
  // dma 0
  writel(0x02290201, iomm.dma + DDMA0_CFG_REG);
  writel(0x00000000, iomm.dma + DDMA0_PAR_REG);
  writel(par->lram_phys[0], iomm.dma + DDMA0_SRC_ADR_REG);
  writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + DDMA0_DES_ADR_REG);
  writel((320 * 240 * 8) + 8, iomm.dma + DDMA0_BYTE_CNT_REG);
  // dma 1
  writel(0x02290201, iomm.dma + DDMA1_CFG_REG);
  writel(0x00000000, iomm.dma + DDMA1_PAR_REG);
  writel(par->lram_phys[1], iomm.dma + DDMA1_SRC_ADR_REG);
  writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + DDMA1_DES_ADR_REG);
  writel((320 * 240 * 8) + 8, iomm.dma + DDMA1_BYTE_CNT_REG);
  // dma 2
  writel(0x02290201, iomm.dma + DDMA2_CFG_REG);
  writel(0x00000000, iomm.dma + DDMA2_PAR_REG);
  writel(par->lram_phys[2], iomm.dma + DDMA2_SRC_ADR_REG);
  writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + DDMA2_DES_ADR_REG);
  writel((320 * 240 * 8) + 8, iomm.dma + DDMA2_BYTE_CNT_REG);
  // dma 3
  writel(0x02290201, iomm.dma + DDMA3_CFG_REG);
  writel(0x00000000, iomm.dma + DDMA3_PAR_REG);
  writel(par->lram_phys[3], iomm.dma + DDMA3_SRC_ADR_REG);
  writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + DDMA3_DES_ADR_REG);
  writel((320 * 240 * 8) + 8, iomm.dma + DDMA3_BYTE_CNT_REG);

	par->dma_irq = 18;
	ret = request_irq(par->dma_irq, dma_irq_handler, IRQF_SHARED, "miyoo_dma_irq", par);
  if(ret){
    printk("%s, failed to register DMA interrupt\n", __func__);
  }

  #if 0
    // ndma
    writel(0x02310211, iomm.dma + NDMA0_CFG_REG);
    writel(par->lram_phys[0], iomm.dma + NDMA0_SRC_ADR_REG);
    writel(SUNXI_GPIO_BASE + PD_DATA, iomm.dma + NDMA0_DES_ADR_REG);
  #endif

  #if 0
    // testing ram
    uint32_t i;
    uint8_t *s = par->lram_virt[0];
    uint8_t *d = par->lram_virt[0] + 16;
    printk("src addr: 0x%x\n", s);
    printk("dst addr: 0x%x\n", d);
    for(i=0; i<16; i++){
      s[i] = i;
    }
    writel(par->lram_phys[0], iomm.dma + NDMA0_SRC_ADR_REG);
    writel(par->lram_phys[0] + 16, iomm.dma + NDMA0_DES_ADR_REG);
    writel(16, iomm.dma + NDMA0_BYTE_CNT_REG);
    writel(2, iomm.dma + DMA_INT_STA_REG);
    suniv_setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
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
      suniv_setbits(iomm.dma + NDMA0_CFG_REG, (1 << 31));
      mdelay(3000);
    }
  #endif
}

static void sunxi_cpu_init(struct myfb_par *par)
{
  uint32_t ret;

  // 768MHz
  ret = readl(iomm.ccm + PLL_CPU_CTRL_REG);
  writel(ret | 0x00001f00, iomm.ccm + PLL_CPU_CTRL_REG); 
  printk("%s, 0x%08x PLL_CPU_CTRL_REG\n", __func__, readl(iomm.ccm + PLL_CPU_CTRL_REG));
}

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)
static int myfb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info)
{
  //printk("%s, visual:%d, bits_per_pixel:%d, regno:%d, r:0x%x, g:0x%x, b:0x%x\n", __func__, info->fix.visual, info->var.bits_per_pixel, regno, red, green, blue);
  red = CNVT_TOHW(red, info->var.red.length);
  blue = CNVT_TOHW(blue, info->var.blue.length);
  green = CNVT_TOHW(green, info->var.green.length);
  ((u32*)(info->pseudo_palette))[regno] = (red << info->var.red.offset) | (green << info->var.green.offset) | (blue << info->var.blue.offset);
  return 0;
}
#undef CNVT_TOHW
 
static int myfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
  int bpp = var->bits_per_pixel >> 3;
  struct myfb_par *par = info->par;
  unsigned long line_size = var->xres_virtual * bpp;
 
  if((var->xres != 320) || (var->yres != 240) || (var->bits_per_pixel != 16)){
    return -EINVAL;
  }
 
  //printk("%s, xres:%d, yres:%d, bpp:%d\n", __func__, var->xres, var->yres, var->bits_per_pixel);
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
  return 0;
}
 
static int myfb_remove(struct platform_device *dev)
{
  struct fb_info *info = dev_get_drvdata(&dev->dev);
 
  if(info){
    uint8_t i;
    struct myfb_par *par = info->par;

    unregister_framebuffer(info);
    fb_dealloc_cmap(&info->cmap);
    dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base, par->p_palette_base);
    dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);
    for(i=0; i<DMA_NUM;i++){
      dma_free_coherent(NULL, par->lram_size, par->lram_virt[i], par->lram_phys[i]);
    }
    pm_runtime_put_sync(&dev->dev);
    pm_runtime_disable(&dev->dev);
		free_irq(par->dma_irq, par);
		free_irq(par->lcdc_irq, par);
    framebuffer_release(info);
  }
  return 0;
}
 
static int myfb_set_par(struct fb_info *info)
{
  struct myfb_par *par = info->par;
   
  fb_var_to_videomode(&par->mode, &info->var);
  //printk("%s, xres:%d, yres:%d, bpp:%d, xoffset:%d, yoffset:%d\n", __func__, 
  //  info->var.xres, info->var.yres, info->var.bits_per_pixel, info->var.xoffset, info->var.yoffset);
  par->bpp = info->var.bits_per_pixel;
  info->fix.visual = FB_VISUAL_TRUECOLOR;
  info->fix.line_length = (par->mode.xres * par->bpp) / 8; 
  return 0;
}
 
static int myfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
  uint32_t ret, index;
  struct myfb_par *par = info->par;

  //printk("%s(cmd: 0x%x)++\n", __func__, cmd);
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
  case MIFB_GET_DMA:
    ret = copy_from_user(&index, (void*)arg, sizeof(unsigned long));
    if(index >= DMA_NUM){
      printk("%s, invalid dma index(%d)\n", __func__, index);
      return -1;
    }
    ret = copy_to_user((void*)arg,  &par->lram_phys[index], sizeof(unsigned long));
    printk("MIFB_GET_DMA 0x%x(index: %d)\n", (uint32_t)&par->lram_phys[ret], index);
    break;
  case MIFB_REFILL_MODE:
    par->refill_mode = arg;
    printk("MIFB_REFILL_MODE %d\n", par->refill_mode);
    break;
  case MIFB_FLUSH_DMA:
    index = arg;
    if(index >= DMA_NUM){
      printk("%s, invalud dma index(%d)\n", __func__, index);
      return -1;
    }
    par->dma_ch = index;
    wait_te_ready();
    sunxi_dma_start(par);
    break;
  }
  //printk("%s\n", __func__);
  return 0;
}
 
static int myfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
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

static int myfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
  uint8_t *addr;
  uint32_t size;
  struct myfb_par *par = info->par;
  struct fb_var_screeninfo new_var;
  struct fb_fix_screeninfo *fix = &info->fix;

  //printk("%s, xres:%d, yres:%d, xoffset:%d, yoffset:%d\n", __func__, var->xres, var->yres, var->xoffset, var->yoffset);
  if((var->xoffset != info->var.xoffset) || (var->yoffset != info->var.yoffset)){
    memcpy(&new_var, &info->var, sizeof(new_var));
    new_var.xoffset = var->xoffset;
    new_var.yoffset = var->yoffset;
    memcpy(&info->var, &new_var, sizeof(new_var));  
    size = par->vram_size;
    addr = par->vram_virt + (new_var.yoffset * fix->line_length) + ((new_var.xoffset * info->var.bits_per_pixel) / 8);
  }
  return 0;
}

static struct fb_ops r61520_fb_ops = {
  .owner          = THIS_MODULE,
  .fb_check_var   = myfb_check_var,
  .fb_set_par     = myfb_set_par,
  .fb_setcolreg   = myfb_setcolreg,
  .fb_pan_display = myfb_pan_display,
  .fb_ioctl       = myfb_ioctl,
  .fb_mmap        = myfb_mmap,
 
  .fb_fillrect  = sys_fillrect,
  .fb_copyarea  = sys_copyarea,
  .fb_imageblit = sys_imageblit,
};

void print_time(void)
{
  struct timeval t;
  struct tm broken;
 
  do_gettimeofday(&t);
  time_to_tm(t.tv_sec, 0, &broken);
  printk("%d:%d:%d:%ld\n", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec);
}

static int myfb_probe(struct platform_device *device)
{
  int i, ret;
  uint32_t ulcm;
  struct r61520_lcdc_platform_data *fb_pdata = device->dev.platform_data;
  struct fb_videomode *lcdc_info;
  struct fb_info *r61520_fb_info;
  struct myfb_par *par;

  //printk("%s++\n", __func__);
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
   
  r61520_fb_info = framebuffer_alloc(sizeof(struct myfb_par), &device->dev);
  if(!r61520_fb_info){
    dev_dbg(&device->dev, "memory allocation failed for fb_info\n");
    ret = -ENOMEM;
    goto err_pm_runtime_disable;
  }
 
  par = r61520_fb_info->par;
  par->dev = &device->dev;
  par->bpp = 16;
   
  fb_videomode_to_var(&myfb_var, lcdc_info);
  //printk("%s, xres: %d, yres:%d, bpp:%d\n", __func__, lcdc_info->xres, lcdc_info->yres, par->bpp);
 
  // allocate frame buffer
  par->vram_size = lcdc_info->xres * lcdc_info->yres * par->bpp;
  ulcm = lcm((lcdc_info->xres * par->bpp * 2) / 8, PAGE_SIZE); // double buffer
  par->vram_size = roundup(par->vram_size/8, ulcm);
  par->vram_size = par->vram_size;
  par->vram_virt = dma_alloc_coherent(NULL, par->vram_size, (resource_size_t*) &par->vram_phys, GFP_KERNEL | GFP_DMA);
  if(!par->vram_virt){
    dev_err(&device->dev, "%s, failed to allocate frame buffer(vram)\n", __func__);
    ret = -EINVAL;
    goto err_release_fb;
  }
 
  // swap video ram
  par->tram_virt = dma_alloc_coherent(NULL, par->vram_size, (resource_size_t*)&par->tram_phys, GFP_KERNEL | GFP_DMA);
  if(!par->tram_virt){
    dev_err(&device->dev, "%s, failed to allocate frame buffer(tram)\n", __func__);
    ret = -EINVAL;
    goto err_release_fb;
  }
  memset(par->tram_virt, 0xff, par->vram_size);
 
  par->lram_size = (320 * 240 * 8) + 8; // fixed size for r61520 panel
  for(i=0; i<DMA_NUM; i++){
    par->lram_virt[i] = dma_alloc_coherent(NULL, par->lram_size, (resource_size_t*) &par->lram_phys[i], GFP_KERNEL | GFP_DMA);
    if(!par->lram_virt[i]){
      dev_err(&device->dev, "%s, failed to allocate frame buffer[%d](lram)\n", __func__, i);
      ret = -EINVAL;
      goto err_release_fb;
    }
    memset(par->lram_virt[i], 0xff, par->lram_size);
  }
  r61520_fb_info->screen_base = (char __iomem*) par->vram_virt;
  myfb_fix.smem_start    = par->vram_phys;
  myfb_fix.smem_len      = par->vram_size;
  myfb_fix.line_length   = (lcdc_info->xres * par->bpp) / 8;
 
  // allocate palette buffer
  par->v_palette_base = dma_alloc_coherent(NULL, PALETTE_SIZE, (resource_size_t*)&par->p_palette_base, GFP_KERNEL | GFP_DMA);
  if(!par->v_palette_base){
    dev_err(&device->dev, "GLCD: kmalloc for palette buffer failed\n");
    ret = -EINVAL;
    goto err_release_fb_mem;
  }
  memset(par->v_palette_base, 0, PALETTE_SIZE);
  myfb_var.grayscale = 0;
  myfb_var.bits_per_pixel = par->bpp;
 
  // initialize fbinfo
  r61520_fb_info->flags = FBINFO_FLAG_DEFAULT;
  r61520_fb_info->fix = myfb_fix;
  r61520_fb_info->var = myfb_var;
  r61520_fb_info->fbops = &r61520_fb_ops;
  r61520_fb_info->pseudo_palette = par->pseudo_palette;
  r61520_fb_info->fix.visual = (r61520_fb_info->var.bits_per_pixel <= 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR; 
  ret = fb_alloc_cmap(&r61520_fb_info->cmap, PALETTE_SIZE, 0);
  if(ret){
    goto err_release_pl_mem;
  }
  r61520_fb_info->cmap.len = 32;
 
  // initialize var_screeninfo
  myfb_var.activate = FB_ACTIVATE_FORCE;
  fb_set_var(r61520_fb_info, &myfb_var);
  dev_set_drvdata(&device->dev, r61520_fb_info);

  // register the frame Buffer
  if(register_framebuffer(r61520_fb_info) < 0){
    dev_err(&device->dev, "failed to register /dev/fb0\n");
    ret = -EINVAL;
    goto err_dealloc_cmap;
  } 
  fb_prepare_logo(r61520_fb_info, 0);
  fb_show_logo(r61520_fb_info, 0);

  // lcd init
  sunxi_cpu_init(par);
  #if 0
    sunxi_lcdc_gpio_config(1);
    sunxi_dma_init(par)
  #else
    sunxi_lcdc_gpio_config(0);
    sunxi_lcdc_init(par);
  #endif
/* 
  {
    uint16_t *s = par->vram_virt;
    uint16_t *d = par->lram_virt[0];
    for(ret=0; ret<320*240; ret++){
      *d++ = *s++;
    }
  }
*/

  //sunxi_dma_fill(par);
  //sunxi_dma_start(par);
  //printk("%s--\n", __func__);
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
static int myfb_suspend(struct platform_device *dev, pm_message_t state)
{
  struct fb_info *info = platform_get_drvdata(dev);
 
  console_lock();
  fb_set_suspend(info, 1);
  pm_runtime_put_sync(&dev->dev);
  console_unlock();
  return 0;
}
 
static int myfb_resume(struct platform_device *dev)
{
  struct fb_info *info = platform_get_drvdata(dev);

  console_lock();
  pm_runtime_get_sync(&dev->dev);
  fb_set_suspend(info, 0);
  console_unlock();
  return 0;
}
#else
#define myfb_suspend NULL
#define myfb_resume NULL 
#endif
 
static const struct of_device_id fb_of_match[] = {
  {
    .compatible = "allwinner,suniv-f1c500s,r61520", 
  },{}
};
MODULE_DEVICE_TABLE(of, fb_of_match);
 
static struct platform_driver fb_driver = {
  .probe    = myfb_probe,
  .remove   = myfb_remove,
  .suspend  = myfb_suspend,
  .resume   = myfb_resume,
  .driver = {
    .name   = DRIVER_NAME,
    .owner  = THIS_MODULE,
    .of_match_table = of_match_ptr(fb_of_match),
  },
};
 
static void sunxi_ioremap(void)
{
  iomm.dma = (uint8_t*)ioremap(SUNXI_DMA_BASE, 4096);
  iomm.ccm = (uint8_t*)ioremap(SUNXI_CCM_BASE, 4096);
  iomm.gpio = (uint8_t*)ioremap(SUNXI_GPIO_BASE, 4096);
  iomm.lcdc = (uint8_t*)ioremap(SUNXI_LCDC_BASE, 4096);
  iomm.debe = (uint8_t*)ioremap(SUNXI_DEBE_BASE, 4096);
}

static void sunxi_iounmap(void)
{
  iounmap(iomm.dma);
  iounmap(iomm.ccm);
  iounmap(iomm.gpio);
  iounmap(iomm.lcdc);
  iounmap(iomm.debe);
}

static int __init fb_init(void)
{
  sunxi_ioremap();
  return platform_driver_register(&fb_driver);
}
 
static void __exit fb_cleanup(void)
{
  sunxi_iounmap();
  platform_driver_unregister(&fb_driver);
}
 
module_init(fb_init);
module_exit(fb_cleanup);
 
MODULE_DESCRIPTION("Allwinner f1c500s framebuffer driver for R61520 panel");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_LICENSE("GPL");

