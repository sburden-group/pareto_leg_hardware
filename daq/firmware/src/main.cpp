#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/nvic.h> // for otg_fs_isr and other isr's
#include <libopencm3/stm32/timer.h>  // for timer constants.
#include <usbd_setup.h>
#include <analog_input.h>
#include <millis.h>
#include <stdlib.h>


// TODO: abstract all of this USB buffering junk into a small class
// with pointers to writing buffers ping and pong.

// ------ Overview: ------
// Define a Packet structure such that we can send multiple datapoints over
// to the PC with one USB packet transfer (64 bytes).
// Create a Ping-Pong Buffer to aggreggate datapoints in one packet while
// writing out the other one over USB.
// Setup a timer to define our sampling frequency (~ 1 KHz).
// In the main loop,
// -- check if it's time to read new sample data over SPI (as set by the timer).
// -- check if we've filled up a usb packet. If so, switch buffers and
//    dispatch the full packet.


// ------ Constants: ------
static const size_t DEFAULT_SAMPLING_FREQ = 100; // [Hz]
static const size_t DATAPTS_PER_PKT = 30; // number of samples in a USB packet.


// ------ Structure Definitions: ------
// Define structure of the USB Packet that will transfer the analog data.
#pragma pack(push, 1) // Tell compiler we want an exact fit without padding.
struct usb_packet // use pragma to enforce packet size of 64 bytes.
{
    uint16_t analog_time_series[DATAPTS_PER_PKT]; // 60 bytes
    uint32_t packet_index; // 4 bytes
};
#pragma pack(pop)

// Create some system-wide flags to manage control flow.
struct state_flags
{
    bool adc_read_pending; // Flag to indicate when to read new analog data.
    bool usb_output_data_ready; // Flag to indicate when to write usb packet.
};

// ------ Variables and Structures: ------
volatile state_flags system_state{false, false}; // mark volatile since ISRs will modify it.

// Ping-Pong Buffers for aggregating analog data.
// Write to one while transferring the other. Then swap.
usb_packet ping;
usb_packet pong;
// Pointers to demarcate who is being written-to and who is being transferred.
usb_packet* sampling_buffer;
usb_packet* usb_writing_buffer;

// A packet ID that increments once per stuffed packet.
// Note that packets can fly as fast as 1KHz on Full Speed, so this value
// should be sufficiently large to ID packets before rollover.
uint32_t usb_sent_pkts_index = 0;
// Index into the packet to determine where to write next.
uint8_t usb_pkt_datapt_index = 0;


// ------ Functions: ------
// Callback fn for handling usb control transfers.
// Change the sampling freq over USB.
static enum usbd_request_return_codes simple_control_cb(
        usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)buf;
	(void)len;
	(void)complete;
	(void)usbd_dev;

    // Ignore all other requests besides a Vendor Request type.
	if (req->bmRequestType != 0x40) // Vendor Request
		return USBD_REQ_NOTSUPP;

    // TODO: setup callback to change the sample frequency
	if (req->wValue & 1)
		gpio_set(GPIOB, GPIO7);
	else
		gpio_clear(GPIOB, GPIO7);

	return USBD_REQ_HANDLED;
}


static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

    // Setup bulk transfer output.
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    // Callback fn for handling data from a received control transfer.
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				simple_control_cb);
}


// Set sampling frequency at values between 15.26[Hz] and 1[MHz]
// not all frequencies are possible.
// Pause and re-enable TIM7 interrupt by default unless told to ignore.
void set_sampling_freq(uint32_t hz, bool ignore_irq_state = false)
{
    uint32_t ticks = 1e6/hz; // Note: timer is set to tick at 1 [MHz].
    // 1 tick for 1[MHz]. 1000 ticks for 1[KHz]. 4000 ticks for 250[Hz]. etc.
    // eqn is: UPDATE_FREQ = TICK_RATE / PERIOD
    if (!ignore_irq_state)
        nvic_disable_irq(NVIC_TIM7_IRQ);
    timer_set_period(TIM7, ticks);
    if (!ignore_irq_state)
        nvic_enable_irq(NVIC_TIM7_IRQ); // re-enable interrupt.
}


// Setup trigger interval for Timer7 interrupt.
// This ISR dictates the sample rate.
void setup_sampling_timer(void)
{
    rcc_periph_clock_enable(RCC_TIM7);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    rcc_periph_reset_pulse(RST_TIM7); // reset timer 7 to defaults.

// Primary Datasheet for STM32F415xx Sec 2.2, Fig 5 has clock APB mapping.
// RM0090 Manual for Sec 7.2, Fig 21 has the clock tree (prescaling) mapping.
// TIM7 uses APB1.
// For current clock (rcc) settings, we have:
// AHB --> no prescaling
// APB1 --> DIV4
// CLK_INT --> 168000000 / 4 x 2 = 84 [MHz]

    timer_set_mode(TIM7,
                   TIM_CR1_CKD_CK_INT, // No additional clock division ratio.
                   TIM_CR1_CMS_EDGE, // Edge alignment.
                   TIM_CR1_DIR_UP); // Up-counting counter

    timer_set_prescaler(TIM7, 84); // Increment at 1[MHz]
    set_sampling_freq(DEFAULT_SAMPLING_FREQ, true); // don't alter irq state.
    timer_disable_preload(TIM7);
	timer_continuous_mode(TIM7);
    timer_enable_counter(TIM7);

    // When the timer updates, the interrupt should fire.
    timer_enable_irq(TIM7, TIM_DIER_UIE); // Timer 7 update interrupt enable.
}


// Handle usb activity via ISR so we don't have to poll it in the main loop.
// this fn is weakly defined in NVIC.h
void otg_fs_isr(void)
{
   usbd_poll(usbd_dev);
}

// ISR to sample the data and transfer it into ping-pong buffer.
// this fn is weakly defined in NVIC.h
void tim7_isr(void)
{
    timer_clear_flag(TIM7, TIM_SR_UIF);
    //gpio_toggle(GPIOA, GPIO8);
    system_state.adc_read_pending = true;
}


// Read the analog input data
// Flag if the usb data is full and ready for transfer.
void sample_and_store_datapt(void)
{

    sampling_buffer->analog_time_series[usb_pkt_datapt_index] =
        read_adc_naiive(ADC1, 8); // PB0 is ADC1_IN8.

    // Increment to the next datapt.
    ++usb_pkt_datapt_index;
}

// Check if we've filled up the buffer we're writing to.
// Flag if we need to dispatch usb packet.
void handle_usb_buffers(void)
{
    // Check buffer index. Switch buffers if necessary.
    if (usb_pkt_datapt_index > DATAPTS_PER_PKT - 1)
    {
        /// Setup writing at the beginning of the next packet.
        usb_pkt_datapt_index = 0;
        // ID the packet before transmitting so we can sort them on the receiving end.
        sampling_buffer->packet_index = usb_sent_pkts_index;
        // Swap buffers.
        usb_writing_buffer = sampling_buffer;
        sampling_buffer = (sampling_buffer == &ping)? &pong: &ping;
        // Flag that we are ready to transfer USB data in the main loop.
        system_state.usb_output_data_ready = true;
        // Move on to the next packet.
        ++usb_sent_pkts_index;
    }
}


// ------ main thread: ------
int main(void)
{
    setup_system_clock(); // do this first.
    setup_systick(); // for delay_ms fn

    // Enable GPIO for a Blinky.
	rcc_periph_clock_enable(RCC_GPIOB);
    // enable GPIOA for usb-related GPIO.
	rcc_periph_clock_enable(RCC_GPIOA);

    // encoder counting and sampling setup:
    setup_sampling_timer();

    // USB init.
	rcc_periph_clock_enable(RCC_OTGFS);


	// Setup 1bity's status LED for output.
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

    // Relevant Pinouts:
    // PA8 --> USB SOF (start of frame, unused)
    // PA9 --> USB Vbus (otg fs VBus sensing, unused, disabled)
    // PA10 --> USB ID (otg fs id)
    // PA11 --> USB DM (Data minus)
    // PA12 --> USB DP (Data plus)
    // PG6 --> USB GPIO OUT (unused)
    //gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11 | GPIO12);
    //gpio_set_af(GPIOA, GPIO_AF10, GPIO10 | GPIO11 | GPIO12);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    // Setup interrupts for handling usb events.
    nvic_enable_irq(NVIC_OTG_FS_IRQ);

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
                         usb_strings, 3,
                         usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);

    setup_adc(ADC1);
    setup_gpio_for_adc(GPIOB, 0); // PB0 is ADC1_IN8.


    // Main Loop: Check if it's time to write the packet, and write it.
    // tim7_isr flags for any of these behaviors to execute.
    // Note: we assume that a USB packet transfer happens faster than it takes
    //       to fill the usb buffer.
    // Note: We assume that these combined behaviors execute and complete
    //       before it's time to execute them again.
	while (1)
    {
		//usbd_poll(usbd_dev); // not needed since we're using interrupts.
        if (system_state.adc_read_pending)
        {
            system_state.adc_read_pending = false; // Clear the flag.
            sample_and_store_datapt(); // read adc data and store it in packet.
            handle_usb_buffers(); // Switch PingPong buffers. Flag for sending.
        }

        if (system_state.usb_output_data_ready)
        {
            system_state.usb_output_data_ready = false; // Clear the flag.
            // Cast the struct into a serialized sequence of bytes.
            usbd_ep_write_packet(usbd_dev, 0x81, (const void*)usb_writing_buffer,
                                 BULK_BUFFER_SIZE);
            gpio_toggle(GPIOA, GPIO8);
        }
    }

}
