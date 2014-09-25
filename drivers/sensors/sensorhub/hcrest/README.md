# Linux Kernel driver for Hillcrest Labs SensorHub

# Build instructions

## Integration

 - Add this directory in it's entirety under `drivers/misc/`
 - Source the Sensorhub driver in to your Kconfig by adding the following
 to `drivers/misc/Kconfig`:

   ```
   source "drivers/misc/sensorhub/Kconfig"
   ```
 - Include the Sensorhub driver directory in the Kernel Makefiles by adding the
 following to the end of `drivers/misc/Makefile`:

   ```
   obj-y += sensorhub/
   ```
 - Symlink `drivers/misc/sensorhub/sensorhub.h` under `include/linux`

 ## In-Kernel Build
 - To enable the SensorHub I2C driver set the following Kernel config options:

   ```
   CONFIG_SENSOR_HUB_I2C=y
   # Atmel devices with Firmware Update
   CONFIG_SENSOR_HUB_I2C_ATMEL_DFU=y
   # for debug builds
   CONFIG_DYNAMIC_DEBUG=y
   CONFIG_SENSOR_HUB_DEBUG=y
   ```

 ## Loadable module
Set the following variables (via env or Make CLI):

 - Set the following Kernel config options:
```
   CONFIG_SENSOR_HUB_I2C=m
   # Atmel devices with Firmware Update
   CONFIG_SENSOR_HUB_I2C_ATMEL_DFU=y

   # enable loadable module support
   CONFIG_MODULES=y
   CONFIG_MODULE_UNLOAD=y
   # for debug builds
   CONFIG_DYNAMIC_DEBUG=y
   CONFIG_SENSOR_HUB_DEBUG=y
```
 - You can quickly build only the Sensorhub driver by setting the following env
 vars and running `make` under `drivers/misc/sensorhub`.
    - KERNEL_SRC - Source directory for Kernel you're building against.
      Builds against builtin Kernel headers if not specified
    - KERNEL_OBJ - A Kernel build output directory (if building using O=<dir>)

To cross compile setup the standard `ARCH` and `CROSS_COMPILE` vars before running Make.

# Character Device Interface

Each sensor hub device will have it's own character device. Userpace applications should access the hardware using these character devices.

The chararacter devices are typically created as `/dev/sensorhub#` by udev or Android's ueventd system.

 - **HID Input Reports**

  HID Input reports are received via the `read()` function. A single HID Input message is returned for each `read()` function call.

  The first byte is the hid report id and the length is returned by `read()` as the number of bytes copied in to the user buffer.

  This call will not block if `O_NONBLOCK` is set.

  The call returns `-1` and sets `errno` to `EAGAIN` if no input reports are present **and*( `O_NONBLOCK` is set.

 - **HID Output Reports**

  Hid Output reports are sent via the `write()` function.

  A single HID output message is written with each `write().`

  The first byte is the HID report id.

  The length of the HID message is the length of the user buffer.

  This call is expected to block.  It ignores `O_NONBLOCK`.

 - **HID Feature Reports**

  HID features can be get and set using the `ioctl()` function

  The get and set operations expect a user read-write buffer as the command argument. The first byte is the HID report id.

  The `ioctl` commands are defined via macros in [sensorhub.h] [sensorhub_header_link].  These macros take the length of the user buffer as an argument.

  This call is expected to block. It ignores O_NONBLOCK.

 - **Poll**

  When polling the file these poll revents flags are generated:
    - `POLLERR` - when a unrecovarable error occurs
    - `POLLHUP` - when the hardware resets
    - `POLLIN | POLLRDNORM` - when data is available to read without blocking

# Sysfs Interface

The sensorhub driver provides the following sysfs interface. These files are populated under `/sys/devices/virtual/misc/sensorhub#`

|      File     | Access | Debug |                                    Description                                     |
|---------------|--------|-------|------------------------------------------------------------------------------------|
| `descriptor`  | R      |       | Read HID Report descriptor                                                         |
| `forcehangup` | W      | Y     | Any write here causes a POLLHUP on all clients                                     |
| `reset`       | W      |       | Perform a reset. Writing 'hard' toggles reset line, anything else for 'soft' reset |
| `insert`      | W      | Y     | Inserts data to /dev/sensorhubX file                                               |
| `product`     | R      |       | Sensorhub product id                                                               |
| `state`       | RW     |       | Boot/Update status - see [firmware update][firmware_update]                        |
| `stats`       | R      | Y     | Display various statistics                                                         |
| `suspend`     | W      | Y     | Write 1 to suspend device, anythign else to resume                                 |
| `vendor`      | R      |       | Sensorhub vendor id                                                                |
| `version`     | R      |       | Sensorhub firmware version                                                         |

<a name="dfu"></a>
# Firmware Update

The driver determines the state of the device on startup and sets the value in the state file as listed below.

`state` sysfs file to indicate one of the following:
   - `app` if the main sensorhub application is running
   - `bootloader` if no app image is found and the device is waiting for an application image
   - `download` a firmware download is in progress
   - `unknown` if either the app or bootloader did not respond


## Firmware Update Process

The firmware download relies on the Kernel's firmware class.  For most Linux systems udev sevices handle the firmware request and supply the firmware file to the Kernel. See the [firmware class] [fw_cls_link] for more information if this feature is not available.

Follow these steps:
  - Check if the device requires a firmware update (application version or `factory` state)
  - Write the word `update` to the `state` sysfs file
    The `state` file will change to `download` to indicate that the driver
    has received firmware and a download is in progress.
  - Poll the `state` file and wait for it to change to `app` to indicate completion of dowload

## Firmware Download for Atmel I2C Devices

Firmware download support is available for Atmel devices and driver support must
be enabled via the Kconfig option mentioned above. The platform config must specify the necessary GPIOs as well.

## Android Notes

The Android `udev` daemon supports the request firmware `udev` event. To supply
firmware to the driver, simply copy the firmware file to `/system/vendor/firmware/`
and initiate a firmware download.

## Sensor Hub Firmware Files

  - Atmel SAM D20  - FSP3xx.bin TODO
  - Atmel SAM G51  - FSP3xx.bin TODO

# Platform Config Example

```cpp
// Sample sensorhub configuration for PandaBoard
// This code snippet shows how to:
//      - specify IRQ/GPIO for the host interrupt
//      - specify GPIO for reset
//      - configure GPIO direction and pull direction for each IO
//      - (Optional) specify a GPIO for debug
//      - register sensorhub i2c driver

#include <linux/sensorhub.h>

#define GPIO_SENSORHUB_DATA  138
#define GPIO_SENSORHUB_RESET 139
#define GPIO_SENSORHUB_CHG   136

static struct gpio panda_sensorhub_gpios[] __initdata = {
	{ GPIO_SENSORHUB_IRQ,	GPIOF_IN,  			 "sensorhub-data"  },
	{ GPIO_SENSORHUB_RESET,	GPIOF_OUT_INIT_HIGH, "sensorhub-reset" },
	{ GPIO_SENSORHUB_DEBUG,	GPIOF_OUT_INIT_LOW,  "sensorhub-debug" }
};

static struct sensorhub_platform_data __initdata panda_sensorhub_pdata = {
	.irq            = OMAP_GPIO_IRQ(GPIO_SENSORHUB_IRQ),
	.gpio_interrupt = GPIO_SENSORHUB_IRQ,
	.gpio_reset     = GPIO_SENSORHUB_RESET,
	.gpio_debug     = GPIO_SENSORHUB_DEBUG
};

static struct i2c_board_info __initdata panda_i2c_expansion_bus[] = {
	{
		I2C_BOARD_INFO(SENSORHUB_I2C_ID, SENSORHUB_I2C_ADDR),
		.platform_data = &panda_sensorhub_pdata,
	},
};

static int __init omap4_panda_sh1_init(void) {

	omap_mux_init_gpio(GPIO_SENSORHUB_IRQ,   OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(GPIO_SENSORHUB_RESET, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_SENSORHUB_DEBUG, OMAP_PIN_OUTPUT);

	if (gpio_request_array(panda_sensorhub_gpios, ARRAY_SIZE(panda_sensorhub_gpios))) {
		pr_err("Error requesting sensorhub GPIOs");
		return -EFAULT;
	}

	// Export the GPIOs to user-space for debug
	gpio_export(GPIO_SENSORHUB_IRQ,   0);
	gpio_export(GPIO_SENSORHUB_RESET, 0);
	gpio_export(GPIO_SENSORHUB_DEBUG, 0);

	return omap_register_i2c_bus(4, 400, panda_i2c_expansion_bus, ARRAY_SIZE(panda_i2c_expansion_bus));
}
```


 [fw_cls_link]: https://www.kernel.org/doc/Documentation/firmware_class/README "Linux Firmware Class README"
 [sensorhub_header_link]: sensorhub.h "sensorhub.h"
 [firmware_update]: #dfu "firmware update"
