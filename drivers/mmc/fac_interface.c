#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

extern int release_all_wakelocks(void);
unsigned char release_all_wakelocks_sign = 0;
EXPORT_SYMBOL(release_all_wakelocks_sign);
ssize_t asus_release_all_wakelocks_read(struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[5];
	len = sprintf(kernelbuf,"%d\n",release_all_wakelocks_sign);
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
ssize_t asus_release_all_wakelocks_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	printk(KERN_INFO"[jevian log]asus_release_all_wakelocks_write argue is %c\n",buffer[0]);
	if(buffer[0] == '1') {
		release_all_wakelocks_sign = 1;
		release_all_wakelocks();
	} else if(buffer[0] == '0') {
		release_all_wakelocks_sign = 0;
	}
    return count;
}
void init_asus_release_all_wakelocks(void)
{
	static struct file_operations release_all_wakelocks_fops = {
		.read = asus_release_all_wakelocks_read,
		.write = asus_release_all_wakelocks_write,
	};
	if(proc_create("release_all_wakelocks", 0777,NULL, &release_all_wakelocks_fops)) {
		printk(KERN_ERR"[jevian log]create /proc/release_all_wakelocks fail\n");
	}
}


static int __init fac_interface_init(void)
{
	printk(KERN_ERR"[jevian log]this is fac img\n");
	init_asus_release_all_wakelocks();
	return 0;
}

static void __exit fac_interface_exit(void)
{
	;
}

late_initcall(fac_interface_init);
module_exit(fac_interface_exit);
