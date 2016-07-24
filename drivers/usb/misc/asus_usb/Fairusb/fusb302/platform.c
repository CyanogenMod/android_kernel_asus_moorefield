#include <linux/kernel.h>                                                                                                        // Implementation details
#include "../core/platform.h"
#include "fusb302_driver.h"
#include "../core/core.h"

#define USB_PLATFORM_INFO(...)	printk("[USB][platform] " __VA_ARGS__);

extern int smb1351_otg(int on);

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Enable/Disable the 5V Source
        fusb_gpio_set_vbus_5v(blnEnable == TRUE ? true : false);
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
    	fusb_gpio_set_vbus_other(blnEnable);
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return fusb_gpio_get_vbus_5v() ? TRUE : FALSE;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.
		return fusb_gpio_get_vbus_other() ? TRUE : FALSE;
    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_i2c_data_get_int_n() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        printk(KERN_ERR "%s - Error: Write data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb302_i2c_write_block((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = TRUE;
    }
    else  // I2C Write failure
    {
        ret = FALSE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;

    if (Data == NULL)
    {
        printk(KERN_ERR "%s - Error: Read data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb302_i2c_read_block(RegisterAddress, DataLength, Data))    // Do block reads if able and necessary
    {
            ret = TRUE;
    }
	else  // I2C Write failure
	{
		ret = FALSE;       // Write data block to the device
		printk(KERN_ERR "%s - I2C read failed! RegisterAddress: 0x%02x\n", __func__, (unsigned int)RegisterAddress);
	}

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{
    if (enable == TRUE)
    {
        fusb_i2c_enable_timers();
    }
    else
    {
        fusb_i2c_disable_timers();
    }
}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_i2c_delay_10us(delayCount);
}

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
    set_usb30_mux(orientation);								// Set MUX orientation for usb3.0
    USB_PLATFORM_INFO("sourceOrSink = %d\n" , sourceOrSink);

    if ((sourceOrSink == Source) && (orientation != NONE))	// Enabled OTG when source
    {
            /* OTG Example Code */
            USB_PLATFORM_INFO("otg(ex:usb3 storage) on\n");
            USB_PLATFORM_INFO("sourceOrSink = %d\n" , sourceOrSink);
            smb1351_otg(1); // Type-C Sink connected

    }
    else
    {
            /* OTG Example Code */
            USB_PLATFORM_INFO("otg(ex:usb3 storage) off\n");
            USB_PLATFORM_INFO("sourceOrSink = %d\n" , sourceOrSink);
            smb1351_otg(0); // Type-C device disconnected

    }

    // Add charger function here that calls 'core_get_advertised_current()'

    /* For Debug: */
    FSC_U16 currentAdvert;
    currentAdvert = core_get_advertised_current();
    USB_PLATFORM_INFO("[%s] Update Current: %dmA\n" , __func__, currentAdvert);
}

/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_contract(FSC_BOOL contract)
{
    // Add charger function here that calls 'core_get_advertised_current()'

    /* For Debug: */
    FSC_U16 currentAdvert;
    currentAdvert = core_get_advertised_current();
    USB_PLATFORM_INFO("[%s] PD Enabled: %d | Update Current: %dmA\n" , __func__, contract, currentAdvert);

}
