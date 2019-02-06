/****************************************************************************
 * Copyright (c) 2009 by Focus Robotics. All rights reserved.
 *
 * This program is an unpublished work fully protected by the United States 
 * copyright laws and is considered a trade secret belonging to the copyright
 * holder. No part of this design may be reproduced stored in a retrieval 
 * system, or transmitted, in any form or by any means, electronic, 
 * mechanical, photocopying, recording, or otherwise, without prior written 
 * permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Sat Jun 27 2009
 * 
 * Brief Description: basic eagle device driver
 * 
 * Functionality:
 * 
 * Issues:
 * - Should alloc memory in probe so that driver refuses to run immediately if nomem
 * - Should alloc 4 1MB or 512KB regions instead of 1 4MB DMA region
 * - Should add ioctls to get DMA addresses instead of reading from memory
 * - Add an Eagle class so that /dev/eagle gets created automatically
 * - Use automatically allocated device major and minor numbers
 * - Add support for up to 4 or 8 devices present
 * - Maybe add support for simple interrupts--maybe ioctl to wait for interrupt
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/
//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
//#include <asm-i386/mtrr.h>      /* mtrr_add */

//#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */

#define EAGLE_MAX_DEVICES                       4
#define EAGLE_MEMORY_SPACE_LENGTH               0x02000000

// My primary eagle device structure where I keep track of everything
struct eagle_dev {
  struct cdev cdev;
  struct pci_dev *pci_dev;
  struct device *ecldev;
  unsigned long eagle_csrs_base;
  unsigned long eagle_memory_base;
  dma_addr_t dma_handle;
  void *vaddr;
  //int mtrr_reg;
  dev_t devno;
};

struct eagle_drv {
  struct eagle_dev *edevs[4];
  struct class *eclass;
  int nxtidx;
  dev_t base_devno;
};

//static struct eagle_dev edev;
static struct eagle_drv edrv;


/******************************************************************************* 
 * File Operations
 */

int eagle_open(struct inode *inode, struct file *filp)
{
  dma_addr_t *bus_addr;
  int idx = iminor(inode);

  // edrv.edevs is a list of struct eagle_dev indexed by minor device number.
  // All info about this eagle device is in that struct and can be accessed just
  // by knowing that minor device number.
  struct eagle_dev *edev = edrv.edevs[idx];

  // Once the struct eagle_dev is found, it is stored in the private_data field
  // for this file so that all other file operations can access it trivially.
  filp->private_data = edev;

  // FIXME: Make a single-open device.
  // Have a flag that gets set on open and is cleared by release. Additional opens
  // should fail once the flag is set. Does it need to be a semaphore for multi-proc
  // machines?

  // HACK: 
  // Put the bus address of the dma region in the first 4 bytes of the dma region
  // so we can easily grab it with the first read.
  // dma_handle is the bus address to be written into eagle's dst csr
  // vaddr is the virtual address to be mapped to user space
  // This value should really be accessed by an ioctl, not by this hack!
  bus_addr = edev->vaddr;
  *bus_addr = edev->dma_handle;

  // HACK: 
  // Check if the PCI device is enabled. It becomes un-enabled when we re-load
  // the xilinx bitstream. If it isn't enabled, then enable it.
  if(edev->pci_dev!=0) {
    //int err;
    u8 val;
    pci_read_config_byte(edev->pci_dev, 0x04, &val);
    if((val & 0x02) != 0x02) {
      pci_write_config_dword(edev->pci_dev, 0x10, edev->eagle_csrs_base);
      pci_write_config_dword(edev->pci_dev, 0x14, edev->eagle_memory_base);
      pci_write_config_byte(edev->pci_dev, 0x04, (val|0x02));
      printk("eagle_open: re-enabled the eagle device\n");
    }
  }

  return 0; // success
}

int eagle_release(struct inode *inode, struct file *filp)
{
  // FIXME: release the single-open flag when single-open is implemented
  // should I bother clearing filp.private_data?
  return 0; // success
}

// Read reads from the dma buffer
// This will get more complicated if the dma buffer is segmented into multiple
// 1MB or smaller chunks.
ssize_t eagle_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  struct eagle_dev *edev = filp->private_data;
  // check bounds. f_pos should be between 0 and 4MB. Count should not bring us beyond 4MB
  // use copy_to_user to copy all required data from edev.vaddr
  //printk("Eagle: reading at position 0x%llx, count is %d\n", *f_pos, count);
  if(copy_to_user(buf, edev->vaddr+*f_pos, count))
    return -EFAULT;

  *f_pos += count;
  return count;
}

// Write writes to the DMA buffer only. Not ever used right now.
ssize_t eagle_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
  struct eagle_dev *edev = filp->private_data;
  // check bounds. f_pos should be between 0 and 4MB. Count should not bring us beyond 4MB from cur pos
  // use copy_from_user to copy all required data to edev.vaddr
  printk("Eagle: writing at position 0x%llx, count is %d\n", *f_pos, count);
  if(copy_from_user(edev->vaddr+*f_pos, buf, count))
    return -EFAULT;

  *f_pos += count;
  return count;
}

// Mmap maps the memory on the eagle circuit card into user space. Reads and
// writes to this mapped memory go directly to the memory on the PCI card but
// read access is quite slow and DMA should normally be used. Write access works
// at a decent speed.
int eagle_mmap(struct file *filp, struct vm_area_struct *vma)
{
  struct eagle_dev *edev = filp->private_data;
  unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
  unsigned long vsize = vma->vm_end - vma->vm_start;
  unsigned long pfn = 0;

  if(offset == 0 && vsize==EAGLE_MEMORY_SPACE_LENGTH) { // mem at offset 0M, size 32M
    pfn = edev->eagle_memory_base >> PAGE_SHIFT;
  } else if(offset == EAGLE_MEMORY_SPACE_LENGTH && vsize==0x00100000) { // csrs at offset 32M, size 1M
    pfn = edev->eagle_csrs_base >> PAGE_SHIFT;
  } else {
    return -EINVAL;
  }

  if(remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot))
    return -EAGAIN;

  return 0;
}

int eagle_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  return 0;
}

loff_t eagle_llseek(struct file *filp, loff_t offset, int mode)
{
  if(mode) return -EINVAL;
  filp->f_pos = offset;
  return offset;
}

static struct file_operations eagle_fops = {
  .owner =    THIS_MODULE,
  .read =     eagle_read,
  .write =    eagle_write,
  .mmap =     eagle_mmap,
  .open =     eagle_open,
  //  .unlocked_ioctl =    eagle_ioctl,
  .llseek =   eagle_llseek,
  .release =  eagle_release,
};

/******************************************************************************* 
 * Probe and Remove and related support functions
 * Probe is called once to init each device so memory should be allocated here
 */
static unsigned char eagle_get_revision(struct pci_dev *dev)
{
  u8 revision;

  pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
  return revision;
}

static int eagle_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
  struct eagle_dev *edev;
  int err;

  // Refuse to probe if nxtidx is already at max
  if(edrv.nxtidx==EAGLE_MAX_DEVICES) {
    return -ENODEV;
  }

  // Create and initialize the struct eagle_dev
  edev = (struct eagle_dev*)kmalloc(sizeof(struct eagle_dev), GFP_KERNEL);
  edev->pci_dev = pdev;
  edev->devno = edrv.base_devno+edrv.nxtidx;

  // Actually register the character driver here
  cdev_init(&edev->cdev, &eagle_fops);
  edev->cdev.owner = THIS_MODULE;
  edev->cdev.ops = &eagle_fops;
  if((err = cdev_add(&edev->cdev, edev->devno, 1))) return err;

  // FIXME: call request_region();

  // What, exactly does pci_enable_device do? Put a reminder here.
  if(pci_enable_device(pdev)) {
    printk(KERN_ERR "Can't enable eagle device.\n");
    return -EIO;
  }
  printk("eagle_probe: Enabled eagle device\n");

  // HACK: I save csrs_base and memory_base so that I can reprogram them if the 
  // Xilinx bitstream gets reloaded in operation and these values get reset
  // FIXME: reserve these regions so that other drivers can't access them!
  edev->eagle_csrs_base = pci_resource_start(pdev, 0);
  edev->eagle_memory_base = pci_resource_start(pdev, 1);
  printk("eagle_probe: CSR base address is %lx and Memory base address is %lx\n",
	 edev->eagle_csrs_base, edev->eagle_memory_base);

  // Set up a write-combining region using the processors mtrr for maximum performance
  // Write combining region disabled because it was causing problems on some motherboards
  //edev.mtrr_reg = mtrr_add(edev.eagle_memory_base, EAGLE_MEMORY_SPACE_LENGTH, MTRR_TYPE_WRCOMB, 0);
  //printk("Added MTRR write-combining region for eagle as region %d\n", edev.mtrr_reg);

  if (eagle_get_revision(pdev) == 0x42) // just an example--not required
    return -ENODEV;

  // Allocate the DMA buffer(s) for this device
  edev->vaddr = dma_alloc_coherent(&pdev->dev, 1024*4096, &edev->dma_handle, GFP_KERNEL);
  if(edev->vaddr!=0) {
    printk("eagle_probe: Created eagle DMA region at address %p, bus address %x\n", edev->vaddr, edev->dma_handle);
  } else {
    printk("eagle_probe: Allocation of DMA region failed!\n");
    // Fail the probe with an appropriate code if we can't allocate the dma region
    // FIXME: cleanup other stuff if this fails!
    return -ENOMEM;
  }

  //edev->ecldev = device_create(edrv.eclass, &pdev->dev, edev->devno, "eagle%d", edrv.nxtidx);
  edev->ecldev = device_create(edrv.eclass, &pdev->dev, edev->devno, edev, "eagle%d", edrv.nxtidx);
  if(IS_ERR(edev->ecldev)) {
    printk("Error %ld encountered when trying to create the device in the eagle class!\n", PTR_ERR(edev->ecldev));
  }

  pci_set_drvdata(pdev, edev);
  edrv.edevs[edrv.nxtidx] = edev;
  printk("eagle_probe: completed probe for device %d\n", edrv.nxtidx);
  edrv.nxtidx++;

  return 0;
}

static void eagle_remove(struct pci_dev *pdev)
{

  struct eagle_dev *edev = pci_get_drvdata(pdev);
  pci_set_drvdata(pdev, 0);

  // Free the DMA buffer(s) for this device
  dma_free_coherent(&pdev->dev, 1024*4096, edev->vaddr, edev->dma_handle);

  /* clean up any allocated resources and stuff here.
   * like call release_region();
   */
  //mtrr_del(edev.mtrr_reg, edev.eagle_memory_base, EAGLE_MEMORY_SPACE_LENGTH);

  // Unregister character driver for this device
  cdev_del(&edev->cdev);

  // Unregister and destroy the class device
  device_destroy(edrv.eclass, edev->devno);

  kfree(edev);

  printk("eagle_remove complete: Disabled eagle device and released resources\n");

}

static struct pci_device_id eagle_ids[] = {
  { PCI_DEVICE(0x105D, 0x850A), },
  { 0, }
};
MODULE_DEVICE_TABLE(pci, eagle_ids);

static struct pci_driver pci_driver = {
  .name = "eagle",
  .id_table = eagle_ids,
  .probe = eagle_probe,
  .remove = eagle_remove,
};

/******************************************************************************* 
 * Module Init and Exit functions 
 */

static int __init eagle_init(void)
{
  int err;

  edrv.eclass = class_create(THIS_MODULE, "eagle");
  if(edrv.eclass == NULL || IS_ERR(edrv.eclass)) {
    printk("Error encountered when trying to create the eagle class!\n");
  }

  // Alloc an arbitrary major device number with 4 minors starting at 0
  if((err = alloc_chrdev_region(&edrv.base_devno, 0, EAGLE_MAX_DEVICES, "eagle"))) return err;

  if((err = pci_register_driver(&pci_driver))) return err; // FIXME: should also unregister the chrdev_region

  return 0;
}

static void __exit eagle_exit(void)
{
  pci_unregister_driver(&pci_driver);
  unregister_chrdev_region(edrv.base_devno, EAGLE_MAX_DEVICES);
  class_destroy(edrv.eclass);
}

MODULE_LICENSE("GPL");

module_init(eagle_init);
module_exit(eagle_exit);
