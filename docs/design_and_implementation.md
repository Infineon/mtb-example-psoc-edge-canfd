[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function and the CM55 core is subsequently put to DeepSleep mode.

In the CM33 non-secure application, the clocks and system resources are initialized by the BSP initialization function. The retarget-io middleware is configured to use the debug UART. The debug UART prints a message (as shown in [Terminal output on program startup](../images/terminal-hello-world.png)) on the terminal emulator, the onboard KitProg3 acts the USB-UART bridge to create the virtual COM port. The User LED1 blinks every 1 second. 

This design consists of a CAN FD configuration as nodes and a user button. On the user button (**USER_BTN1**), press from one node sends the CAN frame to the another and vice versa, and both the CAN FD nodes log the received data over UART serial terminal. Each time a CAN FD frame is received, the user LED toggles.

### CAN FD frame format
- ID - CAN FD identifier
- DLC - Data length code
- Data - Actual data bytes

 ID  | DLC | Data 
------|------------|------
 0x22 | 0x08 | 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
 
<br>

**Figure 1** highlights the CAN FD configuration and parameter setting.

**Figure 1. CAN FD configuration**

![](../images/can_config.png)
