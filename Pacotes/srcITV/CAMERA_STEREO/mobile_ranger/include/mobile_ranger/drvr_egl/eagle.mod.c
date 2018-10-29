#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x35ec255d, "module_layout" },
	{ 0x84b6f4af, "class_destroy" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0x44a8e76b, "pci_unregister_driver" },
	{ 0xe7472ec, "__pci_register_driver" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0x32b91bd6, "__class_create" },
	{ 0xe59ded99, "pci_bus_write_config_byte" },
	{ 0xe0078b25, "pci_bus_write_config_dword" },
	{ 0x2f287f0d, "copy_to_user" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0xd8ecbfbb, "remap_pfn_range" },
	{ 0x60781a70, "device_create" },
	{ 0x91095cab, "x86_dma_fallback_dev" },
	{ 0x7087c350, "dma_alloc_from_coherent" },
	{ 0x1d400e71, "pci_bus_read_config_byte" },
	{ 0x9a756412, "pci_enable_device" },
	{ 0xebd5feb1, "cdev_add" },
	{ 0xb2c831b6, "cdev_init" },
	{ 0x41ad0272, "kmem_cache_alloc_trace" },
	{ 0xcea0a119, "kmalloc_caches" },
	{ 0x50eedeb8, "printk" },
	{ 0x37a0cba, "kfree" },
	{ 0x6c5f5671, "device_destroy" },
	{ 0x2f35ab80, "cdev_del" },
	{ 0xf1243fad, "dma_release_from_coherent" },
	{ 0x31944a28, "dma_ops" },
	{ 0x93d085b2, "dev_set_drvdata" },
	{ 0xddffa24, "dev_get_drvdata" },
	{ 0xb4390f9a, "mcount" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v0000105Dd0000850Asv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "8E104D5E87FDD5B01858C1F");
