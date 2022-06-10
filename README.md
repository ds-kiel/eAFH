# eAFH: Informed Exploration for Adaptive Frequency Hopping in Bluetooth Low Energy

To cite the paper, please use:
> V. Poirot, O. Landsiedel,  “eAFH: Informed Exploration for Adaptive Frequency Hopping in Bluetooth Low Energy,” in the 18th IEEE International Conference on Distributed Computing in Sensor Systems (DCOSS), 2022.

eAFH is a channel management for Bluetooth Low Energy specifically tailored for mobile scenarios.


## Abstract

With more than 4 billion devices produced in 2020, Bluetooth and Bluetooth Low Energy (BLE) have become the dominant solutions for short-range wireless communication in IoT. BLE mitigates interference via Adaptive Frequency Hopping (AFH), spreading communication over the entire spectrum.
However, the ever-growing number of BLE devices and WiFi traffic in the already crowded 2.4 GHz band lead to situations where the quality of BLE connections dynamically changes with nearby wireless traffic, location, and time of day. These dynamic environments demand new approaches for channel management in AFH, by both dynamically excluding frequencies suffering from localized interference and adaptively re-including channels, thus providing sufficient channel diversity to survive the rise of new interference. We introduce eAFH, a new channel-management approach in BLE with a strong focus on efficient channel re-inclusion. eAFH introduces informed exploration as a driver for inclusion: using only past measurements, eAFH assesses which frequencies we are most likely to benefit from re-inclusion into the hopping sequence. As a result, eAFH adapts in dynamic scenarios where interference varies over time. We show that eAFH achieves 98-99.5% link-layer reliability in the presence of dynamic WiFi interference with 1\% control overhead and 40\% higher channel diversity than state-of-the-art approaches.

## Setup & Usage

##### Setup


We implemented eAFH for the Zephyr RTOS v2.4rc1.
To install the Zephyr toolchain, please follow the [official documentation guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).
In summary, you need to install west (`pip install west`), install the python requirements (`pip install -r src/zephyr/scripts/requirements.txt`) and should be able to run west within the `src/` folder immediately.
Warning: all `west` commands must be run from within `src/` from this point onward.

In case you need to recreate a west folder, you need to run `mkdir destination_folder`, `west init destination_folder`, `cd destination_folder`, `west update`, and replace the newly downloaded zephyr folder by this repository's implementation: `rm -r destination_folder/zephyr/* && cp src/zephyr/* destination_folder/zephyr`.

##### How to run eAFH 

We provide an example code in `src/apps/sample_eAFH`.
We used the nRF52840 Development Kit platform for our implementation and evaluation. To compile an application using eAFH for the nRF52840dk, use the following west commands:
`cd src && west build --pristine -b nrf52840dk_nrf52840 apps/sample_eAFH`
To flash the device, you can use:
`west flash`
The application would then start once the device is resetted via its RESET button, or e.g., by using `nrfjprog --reset`.
To look at the device output, you can use, e.g., cat or picocom.
`picocom /dev/tty.usbmodem0006836469181 --baud 115200 --databits 8 --parity n --flow n` on MacOS, where the tty.usbmodem is unique to each device.
To quit picocom, you can use `Ctrl + A + X`.
To make it work, you'll need to add your own devices' hardware address, see line 71 and 72.
The hardware address is printed at the beginning of the program. You can simply copy the printed address and paste it in line 71 and 72 of `main.c`.

By default, the central device prints the status of every connection event. You will obtain a log looking like this:
`[00:04:58.758,575] <dbg> bt_afh.conn_chm_remap: e: 14878 c: 18 m: 0x1ffffffeff p: 1`
Where `e` is the connection event number (a counter stricly increasing, with overflow), `c` the channel used for this event, `m` the channel map (in hex format, MSB), and `p` the PDR of the connection event (1 if all packets were successful, else 0).


To activate eAFH in your own application, use the following options in your `prj.conf` file:
`CONFIG_BT_AFH=y`
`CONFIG_BT_AFH_EAFH=y`
Alternatively, you can use
`CONFIG_BT_AFH_PDR_EXCLUSION=y`
If you desire to use the PDR-Exclusion implementation (Improving the Reliability of Bluetooth Low Energy Connections, Michael Spork et al, EWSN 2020)
or
`CONFIG_BT_AFH_NONE=y`
If you desire to activate the eAFH module, but keep all channels active (e.g., for collecting channel statistics)


The complete configuration of eAFH can be found in `zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/eAFH.h`.

##### Platforms

This implementation has been tested on the [nRF52840 Development Kit from Nordic](https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dk).
It has also been tested on the [Adafruit Feather nRF52840](https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather?view=all).
Some implementation of eAFH is specific to the Nordic implementation of Zephyr's BLE stack. To port eAFH to a new platform, you might need to modify some files. Please see the implementation details for more information.


##### Implementation details

The following Zephyr files were modified to connect the eAFH module:
`zephyr/subsys/bluetooth/controller/CMakeLists.txt`
`zephyr/subsys/bluetooth/Kconfig`
Explanation: Adds the eAFH module and its implementations to Zephyr
`zephyr/subsys/bluetooth/controller/ll_sw/ull.c`
Explanation: Initializes the eAFH module by calling ull_afh_init
`zephyr/subsys/bluetooth/controller/ll_sw/ull_master.c`
Explanation: Initializes eAFH for a new connection by calling ull_afh_conn_init
`zephyr/subsys/bluetooth/controller/ll_sw/nordic/lll/lll_conn.c`
Explanation: If CONFIG_BT_AFH is defined, a call to lll_afh_conn_isr adds connection statistics to the eAFH module
`zephyr/subsys/bluetooth/controller/ll_sw/nordic/lll/lll_chan.c`
Explanation: Implements a Round-Robin Channel selection Algorithm (CSA), useful if you need to collect channel statistics. Use it with CONFIG_BT_CTLR_CHAN_SEL_ROUNDROBIN=y in your prj.conf


The following files were added to Zephyr:
- eAFH module:
`zephyr/subsys/bluetooth/controller/ll_sw/ull_afh.h`
`zephyr/subsys/bluetooth/controller/ll_sw/ull_afh.c`

- eAFH channel management implementations:
`zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/PDR_exclusion.h`
`zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/PDR_exclusion.c`
`zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/eAFH.h`
`zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/eAFH.c`
`zephyr/subsys/bluetooth/controller/ll_sw/afh_impl/none.c`

- Application code:
`apps/sample_eAFH/prj.conf`
`apps/sample_eAFH/main.c`

## Disclaimer 
> Although we tested the code extensively, eAFH is a research prototype that likely contain bugs. We take no responsibility for and give no warranties in respect of using the code.

Unless explicitly stated otherwise, all eAFH sources are distributed under the terms of the Apache License 2.0, see [here](https://github.com/zephyrproject-rtos/zephyr/blob/main/LICENSE) for more details.

This repository includes code from the Zephyr RTOS, under Aopache License 2.0. The Zephyr code is available at [github/zephyrproject-rtos/zephyr](https://github.com/zephyrproject-rtos/zephyr).