#include <nrf9160.h>
#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <device.h>
#include <drivers/uart.h>
#include <drivers/adc.h>
#include <drivers/watchdog.h>
#include <sys/printk.h>
#include <stdbool.h>
#include <hal/nrf_saadc.h>

#define CONFIG_BOARD_NRF9160_PCA10090NS
#if defined(CONFIG_BOARD_NRF52_PCA10040) ||   \
  defined(CONFIG_BOARD_NRF52840_PCA10056) ||  \
  defined(CONFIG_BOARD_NRF9160_PCA10090NS) || \
  defined(CONFIG_BOARD_NRF52840_BLIP)

#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2
#endif

/* To use this sample, either the devicetree's /aliases must have a
  'watchdog0' property, or one of the following watchdog compatibles
   must have an enabled node.*/
#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#elif DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_watchdog)
#define WDT_NODE DT_INST(0, nordic_nrf_watchdog)
#endif

/* If the devicetree has a watchdog node, get its label property.*/
#ifdef WDT_NODE
#define WDT_DEV_NAME DT_LABEL(WDT_NODE)
#else
#define WDT_DEV_NAME ""
#error "Unsupported SoC and no watchdog0 alias in zephyr.dts"
#endif

#define WDT_FEED_TRIES 5
#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];
const struct device *adc_dev; /*Runtime device structure (in memory) per driver instance*/ 
static int adc_sample(void);

/* Structure for specifying the configuration of an ADC channel. */
static const struct adc_channel_cfg m_1st_channel_cfg = {
  .gain = ADC_GAIN,
  .reference = ADC_REFERENCE,
  .acquisition_time = ADC_ACQUISITION_TIME,
  .channel_id = ADC_1ST_CHANNEL_ID,
  #if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_1ST_CHANNEL_INPUT,
  #endif
};

static int adc_sample(void) {
  int ret;
  /* Structure defining an ADC sampling sequence. */
  const struct adc_sequence sequence = {
    .channels = BIT(ADC_1ST_CHANNEL_ID),
    .buffer = m_sample_buffer,
    .buffer_size = sizeof(m_sample_buffer),
    .resolution = ADC_RESOLUTION,
  };
  if(!adc_dev) {
    return -1;
  }
  /*Set a read request.*/
  ret = adc_read(adc_dev, &sequence); 

  /* Print the AIN0 values */
  for(int i = 0; i < BUFFER_SIZE; i++) {
    float adc_voltage = 0;
    adc_voltage = (float)(((float)m_sample_buffer[i] / 1023.0f) * 3600.0f);
    printk("ADC raw value: %d\n", m_sample_buffer[i]);
    printf("Measured voltage: %f mV\n", adc_voltage);
  }
  return ret;
}

static void wdt_callback(const struct device *wdt_dev, int channel_id) {
  static bool handled_event;
  if(handled_event) {
    return;
  }
  wdt_feed(wdt_dev, channel_id);
  printk("Handled things..ready to reset\n");
  handled_event = true;
}

void main(void) {
  int err;
  int wdt_channel_id;
  const struct device *wdt;
  struct wdt_timeout_cfg wdt_config;
  wdt = device_get_binding(WDT_DEV_NAME);
  if(!wdt) {
    printk("Cannot get WDT device\n");
    return;
  }
  /* Reset SoC when watchdog timer expires. */
  wdt_config.flags = WDT_FLAG_RESET_SOC;

  /* Expire watchdog after 1000 milliseconds. */
  wdt_config.window.min = 0U;
  wdt_config.window.max = 1000U;

  /* Set up watchdog callback. Jump into it when watchdog expired. */
  wdt_config.callback = wdt_callback;

  wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
  if(wdt_channel_id == -ENOTSUP) {
    /* IWDG driver for STM32 doesn't support callback */
    wdt_config.callback = NULL;
    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
  }

  if(wdt_channel_id < 0) {
    printk("Watchdog install error\n");
    return;
  }
  err = wdt_setup(wdt,0);
  if(err < 0) {
    printk("Watchdog setup error\n");
    return;
  }
  int err1;
  int err2;
  //adc_sample();
  printk("nrf91 saadc sampling AIN0 (P0.13)\n");
  printk("Example requires secure_boot to have ");
  printk("SAADC set to non-secure!\n");
  printk("If not; BusFault/UsageFault will be triggered\n");
  adc_dev = device_get_binding("ADC_0");
  if(!adc_dev) {
    printk("device_get_binding ADC_0 failed\n");
  }
  err1 = adc_channel_setup(adc_dev, &m_1st_channel_cfg); //Configure an ADC channel.
  if(err1) {
    printk("Error in adc setup: %d\n", err1);
  }

  /* Trigger offset calibration
   * As this generates a _DONE and _RESULT event
   * the first result will be incorrect.
   */
  NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;

  while(1) {
    /* Feeding watchdog. */
    printk("Feeding watchdog %d times\n", WDT_FEED_TRIES);

    for(int i = 0; i < WDT_FEED_TRIES; ++i) {
      err1 = adc_sample();
      err2 = wdt_feed(wdt, wdt_channel_id);
      k_sleep(K_MSEC(50));
    }
 
    if(err2) {
      /* Waiting for the SoC reset. */
      printk("Waiting for reset...\n");
      while(1) {
        k_yield();
      }
    }

    if(err1) {
      printk("Error in adc sampling: %d\n", err1);
    }

    k_sleep(K_MSEC(50));
  }
}
