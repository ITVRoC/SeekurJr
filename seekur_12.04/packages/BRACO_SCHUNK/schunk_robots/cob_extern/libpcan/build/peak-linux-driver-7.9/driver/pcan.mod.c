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
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x1fedf0f4, "__request_region" },
	{ 0xcea0a119, "kmalloc_caches" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x69a358a6, "iomem_resource" },
	{ 0x93d085b2, "dev_set_drvdata" },
	{ 0x9ab897f1, "usb_init_urb" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x4b5669c4, "usb_reset_endpoint" },
	{ 0xdeae679e, "pci_disable_device" },
	{ 0x3b79797d, "i2c_transfer" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xf921be04, "usb_kill_urb" },
	{ 0xe1fe0b58, "usb_deregister_dev" },
	{ 0x999872e2, "remove_proc_entry" },
	{ 0x6c5f5671, "device_destroy" },
	{ 0x95aaa356, "usb_reset_configuration" },
	{ 0x4e66fafc, "parport_find_base" },
	{ 0xc8e0b6e7, "__register_chrdev" },
	{ 0xe7ddec76, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xfb0e29f, "init_timer_key" },
	{ 0x6baae653, "cancel_delayed_work_sync" },
	{ 0x6339a8bc, "mutex_unlock" },
	{ 0x9fa48dc5, "pci_bus_write_config_word" },
	{ 0x91715312, "sprintf" },
	{ 0x7d11c268, "jiffies" },
	{ 0x68dfc59f, "__init_waitqueue_head" },
	{ 0x35b6b772, "param_ops_charp" },
	{ 0x2bc95bd4, "memset" },
	{ 0xff7559e4, "ioport_resource" },
	{ 0xf97456ea, "_raw_spin_unlock_irqrestore" },
	{ 0xe7a0f60c, "current_task" },
	{ 0x70d1f8f3, "strncat" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0x985f683d, "usb_deregister" },
	{ 0xc5c74531, "__mutex_init" },
	{ 0x50eedeb8, "printk" },
	{ 0x92ea6ff2, "parport_unregister_device" },
	{ 0x8440ef1e, "usb_set_interface" },
	{ 0xb6ed1e53, "strncpy" },
	{ 0x2f287f0d, "copy_to_user" },
	{ 0xb4390f9a, "mcount" },
	{ 0xa07534a5, "usb_register_dev" },
	{ 0x64d0da33, "usb_control_msg" },
	{ 0x6c2e3320, "strncmp" },
	{ 0xcf510c4a, "mutex_lock" },
	{ 0x60781a70, "device_create" },
	{ 0x597f2d5a, "parport_claim" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0xeac7e6ca, "parport_release" },
	{ 0x83215a3e, "i2c_del_adapter" },
	{ 0xa8a6f639, "__check_region" },
	{ 0xee48b157, "usb_submit_urb" },
	{ 0x42c8de35, "ioremap_nocache" },
	{ 0x7188b4e4, "pci_bus_read_config_word" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xd44c22ec, "usb_reset_device" },
	{ 0x77edf722, "schedule_delayed_work" },
	{ 0x5a215206, "parport_register_device" },
	{ 0x4292364c, "schedule" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x86a4889a, "kmalloc_order_trace" },
	{ 0xdce3b43c, "usb_clear_halt" },
	{ 0xfc635aa2, "create_proc_entry" },
	{ 0x7c61340c, "__release_region" },
	{ 0x44a8e76b, "pci_unregister_driver" },
	{ 0x41ad0272, "kmem_cache_alloc_trace" },
	{ 0x21fb443e, "_raw_spin_lock_irqsave" },
	{ 0xadb5559d, "param_ops_byte" },
	{ 0xe45f60d8, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x4f68e5c9, "do_gettimeofday" },
	{ 0x37a0cba, "kfree" },
	{ 0x2e60bace, "memcpy" },
	{ 0x622fa02a, "prepare_to_wait" },
	{ 0x4845c423, "param_array_ops" },
	{ 0xedc03953, "iounmap" },
	{ 0xe7472ec, "__pci_register_driver" },
	{ 0x4f99f09f, "usb_register_driver" },
	{ 0x84b6f4af, "class_destroy" },
	{ 0x75bb675a, "finish_wait" },
	{ 0x98709188, "i2c_bit_add_bus" },
	{ 0x8235805b, "memmove" },
	{ 0x487d9343, "param_ops_ushort" },
	{ 0x9a756412, "pci_enable_device" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0x32b91bd6, "__class_create" },
	{ 0xddffa24, "dev_get_drvdata" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=parport,i2c-algo-bit";

MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "3CB4471C8250489CA2183E4");
