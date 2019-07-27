/*
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/io.h>
#include <asm/arch-suniv/cpu.h>
#include <asm/arch-suniv/gpio.h>

#define IN_L1   ((32 * 2) + 1)
#define IN_R1   ((32 * 2) + 2)
#define IN_L2   ((32 * 4) + 0)
#define IN_R2   ((32 * 2) + 3)
#define IN_1    ((32 * 4) + 7)
#define IN_2    ((32 * 4) + 8)
#define IN_3    ((32 * 4) + 9)
#define OUT_1   ((32 * 4) + 2)
#define OUT_2   ((32 * 4) + 3)
#define OUT_3   ((32 * 4) + 4)
#define OUT_4   ((32 * 4) + 5)

static struct input_dev *mydev;
static struct timer_list mytimer;
static int myperiod=30;
 
static int do_input_request(uint32_t pin, const char*name)
{
  if(gpio_request(pin, name) < 0){
    printk("failed to request gpio: %s\n", name);
    return -1;
  }
  gpio_direction_input(pin);
  return 0;
}

static int do_output_request(uint32_t pin, const char* name)
{
  if(gpio_request(pin, name) < 0){
    printk("failed to request gpio: %s\n", name);
    return -1;
  }
  gpio_direction_output(pin, 1);
  return 0;
}

/*static void print_key(uint32_t val, uint8_t is_pressed)
{
  uint32_t i;
  uint32_t map_val[] = {0x0200, 0x0800, 0x0400, 0x0080, 0x0004, 0x0010, 0x0002, 0x0008, 0x0100, 0x0020, 0x0040, 0x1000, 0x2000, 0x4000, 0x8000, -1};
  char* map_key[] = {"UP", "DOWN", "LEFT", "RIGHT", "A", "B", "C", "D", "SELECT", "START", "MENU", "L1", "R1", "L2", "R2"};

  for(i=0; map_val[i]!=-1; i++){
    if(map_val[i] == val){
      if(is_pressed){
        printk("%s\n", map_key[i]);
      } 
      break;
    }
  }
}*/

static void report_key(uint32_t btn, uint32_t mask, uint8_t key)
{
  static uint32_t btn_pressed=0;
  static uint32_t btn_released=0xffff;
 
  if(btn & mask){
    btn_released&= ~mask;
    if((btn_pressed & mask) == 0){
      btn_pressed|= mask;
      input_report_key(mydev, key, 1);
      //print_key(btn & mask, 1);
    }
  }
  else{
    btn_pressed&= ~mask;
    if((btn_released & mask) == 0){
      btn_released|= mask;
      input_report_key(mydev, key, 0);
      //print_key(btn & mask, 0);
    }
  }
}
 
static void scan_handler(unsigned long unused)
{
  static uint32_t pre=0;
  uint32_t scan=0, val=0, total=0;

  for(scan=0; scan<4; scan++){
    gpio_set_value(OUT_1, 1);
    gpio_set_value(OUT_2, 1);
    gpio_set_value(OUT_3, 1);
    gpio_set_value(OUT_4, 1);
    gpio_direction_input(OUT_1);
    gpio_direction_input(OUT_2);
    gpio_direction_input(OUT_3);
    gpio_direction_input(OUT_4);
    switch(scan){
    case 0:
      gpio_direction_output(OUT_1, 0);
      break;
    case 1:
      gpio_direction_output(OUT_2, 0);
      break;
    case 2:
      gpio_direction_output(OUT_3, 0);
      break;
    default:
      gpio_direction_output(OUT_4, 0);
      break;
    }
    if (gpio_get_value(IN_1) == 0){
      val|= ((1 << 0) << (scan * 3));
      total+= 1;
    }
    if (gpio_get_value(IN_2) == 0){
      val|= ((1 << 1) << (scan * 3));
      total+= 1;
    }
    if (gpio_get_value(IN_3) == 0){
      val|= ((1 << 2) << (scan * 3));
      total+= 1;
    }
    if(total >= 2){
      break;
    }
  }
  if (gpio_get_value(IN_L1) == 0){
    val|= 0x1000;
  }
  if (gpio_get_value(IN_R1) == 0){
    val|= 0x2000;
  }
  if (gpio_get_value(IN_L2) == 0){
    val|= 0x4000;
  }
  if (gpio_get_value(IN_R2) == 0){
    val|= 0x8000;
  }

  if(pre != val){
    pre = val;
    report_key(pre, 0x0200, KEY_UP);
    report_key(pre, 0x0800, KEY_DOWN);
    report_key(pre, 0x0400, KEY_LEFT);
    report_key(pre, 0x0080, KEY_RIGHT);

    report_key(pre, 0x0004, KEY_LEFTCTRL);
    report_key(pre, 0x0010, KEY_SPACE);
    report_key(pre, 0x0002, KEY_LEFTALT);
    report_key(pre, 0x0008, KEY_LEFTSHIFT);
    
    report_key(pre, 0x0100, KEY_ESC);
    report_key(pre, 0x0020, KEY_ENTER);

    report_key(pre, 0x0040, KEY_RIGHTCTRL);

    report_key(pre, 0x1000, KEY_TAB);
    report_key(pre, 0x2000, KEY_BACKSPACE);
    report_key(pre, 0x4000, KEY_RIGHTALT);
    report_key(pre, 0x8000, KEY_RIGHTSHIFT);
    input_sync(mydev);
  }
  mod_timer(&mytimer, jiffies + msecs_to_jiffies(myperiod));
}
 
static int __init kbd_init(void)
{
  uint32_t ret;

  uint8_t *gpio = ioremap(0x01c20800, 4096);
  ret = readl(gpio + (2 * 0x24 + 0x00));
  ret&= 0xffff0000;
  writel(ret, gpio + (2 * 0x24 + 0x00));

  ret = readl(gpio + (2 * 0x24 + 0x1c));
  ret&= 0xffffff00;
  ret|= 0x00000055;
  writel(ret, gpio + (2 * 0x24 + 0x1c));

  ret = readl(gpio + (4 * 0x24 + 0x00));
  ret&= 0xffffff00;
  writel(ret, gpio + (4 * 0x24 + 0x00));

  ret = readl(gpio + (4 * 0x24 + 0x1c));
  ret&= 0xffffffff0;
  ret|= 0x000000005;
  writel(ret, gpio + (4 * 0x24 + 0x1c));
  iounmap(gpio);

  do_input_request(IN_L1, "gpio_l1");
  do_input_request(IN_R1, "gpio_r1");
  do_input_request(IN_L2, "gpio_l2");
  do_input_request(IN_R2, "gpio_r2");
  do_input_request(IN_1, "gpio_pe7");
  do_input_request(IN_2, "gpio_pe8");
  do_input_request(IN_3, "gpio_pe9");
  do_output_request(OUT_1, "gpio_pe2");
  do_output_request(OUT_2, "gpio_pe3");
  do_output_request(OUT_3, "gpio_pe4");
  do_output_request(OUT_4, "gpio_pe5");
  
  mydev = input_allocate_device();
  set_bit(EV_KEY,mydev-> evbit);
  set_bit(KEY_UP, mydev->keybit);
  set_bit(KEY_DOWN, mydev->keybit);
  set_bit(KEY_LEFT, mydev->keybit);
  set_bit(KEY_RIGHT, mydev->keybit);
  set_bit(KEY_ENTER, mydev->keybit);
  set_bit(KEY_ESC, mydev->keybit);
  set_bit(KEY_LEFTCTRL, mydev->keybit);
  set_bit(KEY_LEFTALT, mydev->keybit);
  set_bit(KEY_SPACE, mydev->keybit);
  set_bit(KEY_LEFTSHIFT, mydev->keybit);
  set_bit(KEY_TAB, mydev->keybit);
  set_bit(KEY_BACKSPACE, mydev->keybit);
  set_bit(KEY_RIGHTCTRL, mydev->keybit);
  set_bit(KEY_RIGHTALT, mydev->keybit);
  set_bit(KEY_RIGHTSHIFT, mydev->keybit);
  mydev->name = "miyoo_kbd";
  mydev->id.bustype = BUS_HOST;
  ret = input_register_device(mydev);
  
  setup_timer(&mytimer, scan_handler, 0);
  mod_timer(&mytimer, jiffies + msecs_to_jiffies(myperiod));
  return 0;
}
  
static void __exit kbd_exit(void)
{
  input_unregister_device(mydev);
  del_timer(&mytimer);
}
  
module_init(kbd_init);
module_exit(kbd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steward Fu <steward.fu@gmail.com>");
MODULE_DESCRIPTION("Keyboard Driver for Miyoo handheld");
 
