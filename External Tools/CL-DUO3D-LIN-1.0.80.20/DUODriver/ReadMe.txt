DUO Driver
----------
There are two flavors of DUO driver:
  -duo-1024.ko - using 1024 bytes transfers (better performance)
  -duo-512.ko - using 512 bytes transfers (better compatibility)
Always try to use duo-1024.ko and see if it works on your machine first.

Loading the driver
------------------
To load the driver in the teminal type:
  sudo insmod duo-1024-$(uname -r).ko
or
  sudo insmod duo-512-$(uname -r).ko

Unloading the driver
--------------------
To unload the driver in the teminal type:
  sudo rmmod -f duo

Installing the driver
---------------------
To install the driver in the teminal type:
  sudo cp duo-1024-$(uname -r).ko /lib/modules/$(uname -r)/kernel/drivers/duo.ko
  sudo echo 'duo' >> /etc/modules
  sudo depmod
or
  sudo cp duo-512-$(uname -r).ko /lib/modules/$(uname -r)/kernel/drivers/duo.ko
  sudo echo 'duo' >> /etc/modules
  sudo depmod

