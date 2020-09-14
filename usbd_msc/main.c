/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup usbd_msc main.c
 * @{
 * @ingroup usbd_msc
 * @brief usbd msc application main file
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 * Note: Limitation when setting "APP_USBD_CONFIG_EVENT_QUEUE_ENABLE = 0":
 *       when USB cable is unplugged and re-plugged, the application goes into an endless
 *       loop.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdlib.h>

#include "nrf.h"
#include "nrf_block_dev.h"
#include "nrf_block_dev_ram.h"
#include "nrf_block_dev_empty.h"
#include "nrf_block_dev_qspi.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_atomic.h"
#include "nrf_drv_power.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "ff.h"
#include "diskio_blkdev.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_msc.h"
#include "app_error.h"
#include "app_timer.h"

#include "bsp.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@file
 * @defgroup usbd_msc_example main.c
 * @{
 * @ingroup usbd_msc_example
 * @brief USBD MSC example
 *
 */

#define LED_USB_RESUME   (BSP_BOARD_LED_0)
#define LED_USB_START    (BSP_BOARD_LED_1)

#define BTN_RANDOM_FILE  0
#define BTN_LIST_DIR     1
#define BTN_MKFS         2

#define KEY_EV_RANDOM_FILE_MSK (1U << BTN_RANDOM_FILE)
#define KEY_EV_LIST_DIR_MSK    (1U << BTN_LIST_DIR   )
#define KEY_EV_MKFS_MSK        (1U << BTN_MKFS       )

#define USB_TOGGLE_BUTTON   BSP_BUTTON_0
#define RANDOM_FILE_BUTTON  BSP_BUTTON_1
#define LIST_DIR_MSK_BUTTON BSP_BUTTON_2
#define MKFS_BUTTON         BSP_BUTTON_3 /**< Button that will trigger the notification event with the LED Button Service */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 10 /**< Maximum number of events in the scheduler queue. */
#endif


/**
 * @brief SD card enable/disable
 */
#define USE_SD_CARD       0

/**
 * @brief FatFS for QPSI enable/disable
 */
#define USE_FATFS_QSPI    1

/**
 * @brief Mass storage class user event handler
 */
static void msc_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_msc_user_event_t event);


/**
 * @brief Ram block device size
 *
 * @note Windows fails to format volumes smaller than 190KB
 */
#define RAM_BLOCK_DEVICE_SIZE (380 * 512)

/**
 * @brief  RAM block device work buffer
 */
static uint8_t m_block_dev_ram_buff[RAM_BLOCK_DEVICE_SIZE];

/**
 * @brief  RAM block device definition
 */
NRF_BLOCK_DEV_RAM_DEFINE(
        m_block_dev_ram,
        NRF_BLOCK_DEV_RAM_CONFIG(512, m_block_dev_ram_buff, sizeof(m_block_dev_ram_buff)),
        NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "RAM", "1.00")
        );


/**
 * @brief Empty block device definition
 */
NRF_BLOCK_DEV_EMPTY_DEFINE(
        m_block_dev_empty,
        NRF_BLOCK_DEV_EMPTY_CONFIG(512, 1024 * 1024),
        NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "EMPTY", "1.00")
        );


/**
 * @brief  QSPI block device definition
 */
NRF_BLOCK_DEV_QSPI_DEFINE(
        m_block_dev_qspi,
        NRF_BLOCK_DEV_QSPI_CONFIG(
                512,
                NRF_BLOCK_DEV_QSPI_FLAG_CACHE_WRITEBACK,
                NRF_DRV_QSPI_DEFAULT_CONFIG
                ),
        NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "QSPI", "1.00")
        );

#if USE_SD_CARD

#define SDC_SCK_PIN     (27)        ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    (26)        ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    (2)         ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      (32 + 15)   ///< SDC chip select (CS) pin.

/**
 * @brief  SDC block device definition
 */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
                ),
        NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
        );


/**
 * @brief Block devices list passed to @ref APP_USBD_MSC_GLOBAL_DEF
 */
#define BLOCKDEV_LIST() (                                   \
                NRF_BLOCKDEV_BASE_ADDR(m_block_dev_ram, block_dev),     \
                NRF_BLOCKDEV_BASE_ADDR(m_block_dev_empty, block_dev),   \
                NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev),    \
                NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev)      \
                )

#else
#define BLOCKDEV_LIST() (                                       \
                NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev)       \
                )
#endif

/**
 * @brief Endpoint list passed to @ref APP_USBD_MSC_GLOBAL_DEF
 */
#define ENDPOINT_LIST() APP_USBD_MSC_ENDPOINT_LIST(1, 1)

/**
 * @brief Mass storage class work buffer size
 */
#define MSC_WORKBUFFER_SIZE (1024)

/*lint -save -e26 -e64 -e123 -e505 -e651*/
/**
 * @brief Mass storage class instance
 */
APP_USBD_MSC_GLOBAL_DEF(m_app_msc,
                        0,
                        msc_user_ev_handler,
                        ENDPOINT_LIST(),
                        BLOCKDEV_LIST(),
                        MSC_WORKBUFFER_SIZE);

/*lint -restore*/

/**
 * @brief Events from keys
 */
static nrf_atomic_u32_t m_key_events;

/**
 * @brief  USB connection status
 */
static bool m_usb_connected = false;


#if USE_FATFS_QSPI

static FATFS m_filesystem;

static uint32_t record_number = 0; //Record number for stored data
static volatile bool write_file = false;

static bool fatfs_init(void)
{
        FRESULT ff_result;
        DSTATUS disk_state = STA_NOINIT;

        memset(&m_filesystem, 0, sizeof(FATFS));

        // Initialize FATFS disk I/O interface by providing the block device.
        static diskio_blkdev_t drives[] =
        {
                DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev), NULL)
        };

        diskio_blockdev_register(drives, ARRAY_SIZE(drives));

        NRF_LOG_INFO("Initializing disk 0 (QSPI)...");
        disk_state = disk_initialize(0);
        if (disk_state)
        {
                NRF_LOG_ERROR("Disk initialization failed.");
                return false;
        }

        NRF_LOG_INFO("Mounting volume...");
        ff_result = f_mount(&m_filesystem, "", 1);
        if (ff_result != FR_OK)
        {
                if (ff_result == FR_NO_FILESYSTEM)
                {
                        NRF_LOG_ERROR("Mount failed. Filesystem not found. Please format device.");
                }
                else
                {
                        NRF_LOG_ERROR("Mount failed: %u", ff_result);
                }
                return false;
        }

        return true;
}

static void fatfs_mkfs(void)
{
        FRESULT ff_result;

        if (m_usb_connected)
        {
                NRF_LOG_ERROR("Unable to operate on filesystem while USB is connected");
                return;
        }

        NRF_LOG_INFO("\r\nCreating filesystem...");
        static uint8_t buf[512];
        ff_result = f_mkfs("", FM_FAT, 1024, buf, sizeof(buf));
        if (ff_result != FR_OK)
        {
                NRF_LOG_ERROR("Mkfs failed.");
                return;
        }

        NRF_LOG_INFO("Mounting volume...");
        ff_result = f_mount(&m_filesystem, "", 1);
        if (ff_result != FR_OK)
        {
                NRF_LOG_ERROR("Mount failed.");
                return;
        }

        NRF_LOG_INFO("Done");
}


static void test_write(void)
{
        int i = 0;
        char log_record[128] = {0};
        char log_record_r[128] = {0};

        FRESULT ff_result;
        FIL file;
        uint32_t fileSize;
        uint32_t num;

        ff_result = f_open(&file, "log_data.txt", FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
        if (ff_result != FR_OK)
        {
                if(!m_usb_connected)
                        NRF_LOG_INFO("Unable to open or create log_data.txt: %u", ff_result);
                NRF_LOG_FLUSH();
                return;
        }

        ++record_number;

        sprintf(log_record, "1234567890123456789012345678901234567890%d\r\n", record_number+10000000);

        FSIZE_t ptr = f_tell(&file);

        i =  f_write(&file, log_record, strlen(log_record), &num);

        i = f_close(&file);
        if (i != 0)
        {
                NRF_LOG_INFO("f_close != 0, %d, %d", i, record_number);
        }

        NRF_LOG_INFO("Wrote Data Record: %d", record_number);
}

static void fatfs_ls(void)
{
        DIR dir;
        FRESULT ff_result;
        FILINFO fno;

        if (m_usb_connected)
        {
                NRF_LOG_ERROR("Unable to operate on filesystem while USB is connected");
                return;
        }

        NRF_LOG_INFO("\r\nListing directory: /");
        ff_result = f_opendir(&dir, "/");
        if (ff_result != FR_OK)
        {
                NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
                return;
        }

        uint32_t entries_count = 0;
        do
        {
                ff_result = f_readdir(&dir, &fno);
                if (ff_result != FR_OK)
                {
                        NRF_LOG_ERROR("Directory read failed: %u", ff_result);
                        return;
                }

                if (fno.fname[0])
                {
                        if (fno.fattrib & AM_DIR)
                        {
                                NRF_LOG_RAW_INFO("   <DIR>   %s\r\n",(uint32_t)fno.fname);
                        }
                        else
                        {
                                NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
                        }
                }

                ++entries_count;
                NRF_LOG_FLUSH();
        } while (fno.fname[0]);


        NRF_LOG_RAW_INFO("Entries count: %u\r\n", entries_count);
}

static void fatfs_file_create(void)
{
        FRESULT ff_result;
        FIL file;
        char filename[16];

        if (m_usb_connected)
        {
                NRF_LOG_ERROR("Unable to operate on filesystem while USB is connected");
                return;
        }

        (void)snprintf(filename, sizeof(filename), "%08x.txt", rand());

        NRF_LOG_RAW_INFO("Creating random file: %s ...", (uint32_t)filename);
        NRF_LOG_FLUSH();

        ff_result = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (ff_result != FR_OK)
        {
                NRF_LOG_ERROR("\r\nUnable to open or create file: %u", ff_result);
                NRF_LOG_FLUSH();
                return;
        }

        ff_result = f_close(&file);
        if (ff_result != FR_OK)
        {
                NRF_LOG_ERROR("\r\nUnable to close file: %u", ff_result);
                NRF_LOG_FLUSH();
                return;
        }
        NRF_LOG_RAW_INFO("done\r\n");
}

static void fatfs_uninit(void)
{
        NRF_LOG_INFO("Un-initializing disk 0 (QSPI)...");
        UNUSED_RETURN_VALUE(disk_uninitialize(0));
}
#else //USE_FATFS_QSPI
#define fatfs_init()        false
#define fatfs_mkfs()        do { } while (0)
#define fatfs_ls()          do { } while (0)
#define fatfs_file_create() do { } while (0)
#define fatfs_uninit()      do { } while (0)
#endif

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 */
static void msc_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_msc_user_event_t event)
{
        UNUSED_PARAMETER(p_inst);
        UNUSED_PARAMETER(event);
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
        switch (event)
        {
        case APP_USBD_EVT_DRV_SUSPEND:
                bsp_board_led_off(LED_USB_RESUME);
                NRF_LOG_INFO("APP_USBD_EVT_DRV_SUSPEND");
                break;
        case APP_USBD_EVT_DRV_RESUME:
                bsp_board_led_on(LED_USB_RESUME);
                NRF_LOG_INFO("APP_USBD_EVT_DRV_RESUME");
                break;
        case APP_USBD_EVT_STARTED:
                bsp_board_led_on(LED_USB_START);
                NRF_LOG_INFO("APP_USBD_EVT_STARTED");
                break;
        case APP_USBD_EVT_STOPPED:
                UNUSED_RETURN_VALUE(fatfs_init());
                app_usbd_disable();
                bsp_board_leds_off();
                NRF_LOG_INFO("APP_USBD_EVT_STOPPED");
                break;
        case APP_USBD_EVT_POWER_DETECTED:
                NRF_LOG_INFO("USB power detected");
                if (!nrf_drv_usbd_is_enabled())
                {
                        fatfs_uninit();
                        app_usbd_enable();
                }
                break;
        case APP_USBD_EVT_POWER_REMOVED:
                NRF_LOG_INFO("USB power removed");
                app_usbd_stop();
                m_usb_connected = false;
                break;
        case APP_USBD_EVT_POWER_READY:
                NRF_LOG_INFO("USB ready");
                app_usbd_start();
                m_usb_connected = true;
                break;
        default:
                break;
        }
}

static void bsp_event_callback(bsp_event_t ev)
{
        switch (ev)
        {
        /* Just set a flag to be processed in the main loop */
        case CONCAT_2(BSP_EVENT_KEY_, BTN_RANDOM_FILE):
                UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_RANDOM_FILE_MSK));
                break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_LIST_DIR):
                UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_LIST_DIR_MSK));
                break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MKFS):
                UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_MKFS_MSK));
                break;

        default:
                return; // no implementation needed
        }
}
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case USB_TOGGLE_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        if (m_usb_connected == false)
                        {
                                if (!nrf_drv_usbd_is_enabled())
                                {
                                        //fatfs_uninit();
                                        app_usbd_enable();
                                }
                                m_usb_connected = true;
                                NRF_LOG_INFO("Enable the USB");
                        }
                        else
                        {
                                app_usbd_stop();
                                m_usb_connected = false;
                                bsp_board_leds_off();
                                NRF_LOG_INFO("Disable the USB");
                        }
                        NRF_LOG_INFO("Press USB Toggle %d", m_usb_connected);
                }
                break;

        case RANDOM_FILE_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_RANDOM_FILE_MSK));
                        NRF_LOG_INFO("Press RANDOM_FILE_BUTTON");
                }
                break;

        case LIST_DIR_MSK_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_LIST_DIR_MSK));
                        NRF_LOG_INFO("Press RANDOM_FILE_BUTTON");
                }
                break;
        case MKFS_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_MKFS_MSK));
                        NRF_LOG_INFO("Press MKFS_BUTTON");
                }
                break;
        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {USB_TOGGLE_BUTTON, false, BUTTON_PULL, button_event_handler},
                {RANDOM_FILE_BUTTON,  false, BUTTON_PULL, button_event_handler},
                {LIST_DIR_MSK_BUTTON, false, BUTTON_PULL, button_event_handler},
                {MKFS_BUTTON, false, BUTTON_PULL, button_event_handler}
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


int main(void)
{
        ret_code_t ret;

        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);

        static const app_usbd_config_t usbd_config = {
                .ev_state_proc = usbd_user_ev_handler
        };

        ret = NRF_LOG_INIT(app_usbd_sof_timestamp_get);
        APP_ERROR_CHECK(ret);
        NRF_LOG_DEFAULT_BACKENDS_INIT();

        ret = nrf_drv_clock_init();
        APP_ERROR_CHECK(ret);

        scheduler_init();

        /* Fill whole RAM block device buffer */
        for (size_t i = 0; i < sizeof(m_block_dev_ram_buff); ++i)
        {
                m_block_dev_ram_buff[i] = i;
        }

        /* Configure LEDs and buttons */
        nrf_drv_clock_lfclk_request(NULL);
        ret = app_timer_init();
        APP_ERROR_CHECK(ret);

        buttons_init();

        // ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
        // APP_ERROR_CHECK(ret);
        bsp_board_init(BSP_INIT_LEDS);

        if (fatfs_init())
        {
                fatfs_ls();
                fatfs_file_create();
        }

        ret = app_usbd_init(&usbd_config);
        APP_ERROR_CHECK(ret);

        app_usbd_class_inst_t const * class_inst_msc = app_usbd_msc_class_inst_get(&m_app_msc);
        ret = app_usbd_class_append(class_inst_msc);
        APP_ERROR_CHECK(ret);

        NRF_LOG_INFO("USBD MSC example started.");

        if (USBD_POWER_DETECTION)
        {
                ret = app_usbd_power_events_enable();
                APP_ERROR_CHECK(ret);
        }
        else
        {
                NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

                app_usbd_enable();
                app_usbd_start();
                m_usb_connected = true;
        }

        //while (m_usb_connected != true) {};

        while (true)
        {

                UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());

                /* Process BSP key events flags.*/
                uint32_t events = nrf_atomic_u32_fetch_store(&m_key_events, 0);
                if (events & KEY_EV_RANDOM_FILE_MSK)
                {
                        //app_sched_event_put(NULL, 0, fatfs_file_create);
                        app_sched_event_put(NULL, 0, test_write);
                        //fatfs_file_create();
                }

                if (events & KEY_EV_LIST_DIR_MSK)
                {
                        app_sched_event_put(NULL, 0, fatfs_ls);
                        //fatfs_ls();
                }

                if (events & KEY_EV_MKFS_MSK)
                {
                        app_sched_event_put(NULL, 0, fatfs_mkfs);
                        //fatfs_mkfs();
                }

                while (app_usbd_event_queue_process())
                {
                        /* Nothing to do */
                }

                app_sched_execute();
                /* Sleep CPU only if there was no interrupt since last loop processing */
                __WFE();
        }
}

/** @} */
