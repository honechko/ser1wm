# Serial 1-wire bus master driver for Linux

## Build Out-of-Tree (fast way)

* To build the module, you may need to install kernel headers and makefiles if you have not done so already:

    ```$ sudo yum install -y kernel-devel```  
    or  
    ```$ sudo apt-get install -y linux-headers-$(uname -r)```  

* Build module:

    ```$ cd kernel/module```  
    ```$ make```  

* Load module:

    ```$ sudo modprobe wire```  
    ```$ sudo insmod ./ser1wm.ko```  

## Build In-Tree (preferred way)

* Download and unpack [kernel source](http://kernel.org/) ([rpi kernel source](https://github.com/raspberrypi/linux/))

* Optionally apply patches from [patches directory](https://github.com/honechko/ser1wm/tree/main/kernel/patches/):

    ```$ patch -p1 < /path/to/enhancement_w1_touchblock.patch```  
    ```$ patch -p1 < /path/to/enhancement_w1_resetcounter.patch```  
    ```$ patch -p1 < /path/to/fix_w1_slavecount.patch```  
    ```$ patch -p1 < /path/to/fix_w1_loadmodule.patch```  
    ```$ patch -p1 < /path/to/fix_w1_thermpullup.patch```  

* Add driver to kernel source tree:

    ```$ cp /path/to/ser1wm.c drivers/w1/masters```  
    ```$ patch -p1 < /path/to/intree-ser1wm.patch```  

* Configure, make and install modules as usual:

    ```$ make menuconfig```  
    ```$ make modules```  
    ```$ sudo make modules_install```  

## Usage

* Build ```ser1wm_attach```

    ```$ cd user/attach```  
    ```$ make```  

* Attach driver to serial line to which serial 1-wire bus master is connected:

    ```$ sudo ./ser1wm_attach /dev/ttyUSB0```  

    or attach driver to TCP-port on which network 1-wire bus master is listening:

    ```$ sudo ./ser1wm_attach 10.0.0.2:4232```  

* Optionally create udev rules to automatically attach driver when serial devices appear as shown in [this example](https://github.com/honechko/ser1wm/blob/main/user/udev/85-owmasters.rules).

