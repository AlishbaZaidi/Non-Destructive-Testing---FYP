#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#define ADC_NODE DT_ALIAS(adc0)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 12
#define NUM_SAMPLES 1000
#define SAMPLE_DELAY_MS 10
#define VREF_MV 1650      //VREF lowered from 3.3v to 1.65v

static int16_t sample_buffer;

#define ACQ_TIME ADC_ACQ_TIME_DEFAULT

static const struct adc_channel_cfg ch_cfg_0 = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ACQ_TIME,
    .channel_id       = 0,
    .differential     = 0,
};

static const struct adc_channel_cfg ch_cfg_1 = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ACQ_TIME,
    .channel_id       = 1,
    .differential     = 0,
};

static uint32_t total_samples = 0;
static uint32_t start_time;

void print_stats(void)
{
    uint32_t elapsed_ms = k_uptime_get_32() - start_time;
    uint32_t sampling_rate = (total_samples * 1000) / elapsed_ms; // Integer math
    
    printk("\n--- Performance Statistics ---\n");
    printk("Total samples: %"PRIu32"\n", total_samples);
    printk("Elapsed time: %"PRIu32" ms\n", elapsed_ms);
    printk("Sampling rate: %"PRIu32" Hz\n", sampling_rate);
    printk("-----------------------------\n");
}

int main(void)
{
    int ret;

    printk("Booting High-Speed ADC test...\n");
    start_time = k_uptime_get_32();

    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready: %s\n", adc_dev->name);
        return 0;
    }

    printk("ADC device: %s\n", adc_dev->name);
    
    /* Setup channels */
    ret = adc_channel_setup(adc_dev, &ch_cfg_0);
    if (ret < 0) {
        printk("Channel 0 setup failed (%d)\n", ret);
        return 0;
    }

    ret = adc_channel_setup(adc_dev, &ch_cfg_1);
    if (ret < 0) {
        printk("Channel 1 setup failed (%d)\n", ret);
        return 0;
    }

    struct adc_sequence seq = {
        .buffer      = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = ADC_RESOLUTION,
    };

    printk("Starting ADC sampling...\n");

    for (int s = 0; s < NUM_SAMPLES; s++) {
        /* --- Channel 0 --- */
        seq.channels = BIT(0);
        ret = adc_read(adc_dev, &seq);
        if (ret < 0) {
            printk("ADC0 read failed (%d)\n", ret);
        } else {
            int32_t val_mv = sample_buffer;
            ret = adc_raw_to_millivolts(VREF_MV,
                                        ADC_GAIN_1,
                                        ADC_RESOLUTION,
                                        &val_mv);
            if (s % 100 == 0) {
                printk("Sample[%d] CH0: raw=%d, %dmV\n", s, sample_buffer, val_mv);
            }
        }

        /* --- Channel 1 --- */
        seq.channels = BIT(1);
        ret = adc_read(adc_dev, &seq);
        if (ret < 0) {
            printk("ADC1 read failed (%d)\n", ret);
        } else {
            int32_t val_mv = sample_buffer;
            ret = adc_raw_to_millivolts(VREF_MV,
                                        ADC_GAIN_1,
                                        ADC_RESOLUTION,
                                        &val_mv);
            if (s % 100 == 0) {
                printk("Sample[%d] CH1: raw=%d, %dmV\n", s, sample_buffer, val_mv);
            }
        }

        total_samples += 2;
        k_msleep(SAMPLE_DELAY_MS);
    }
    
    print_stats();
    printk("Done after %d samples\n", NUM_SAMPLES);
    return 0;
}