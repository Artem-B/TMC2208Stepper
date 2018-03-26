#include <HAL.h>
#include <Stream.h>
#include "TMC2208Stepper.h"
#include "TMC2208Stepper_REGDEFS.h"
#include <LPC17xx.h>
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "fastio.h"

static void tmc_write(pin_t tx_pin, uint8_t data) {
    digitalWrite(tx_pin, 1); // initial state
    cli();
    sei();
}

static uint8_t tmc_calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		} 
	}
	return crc;
}

#define DMA_SIZE 64
const int tmc_oversampling = 8;
const int tmc_msg_bytes = 8;
const int tmc_gap_bytes = 16;

#define __AHBSRAM0__ __attribute__((section("AHBSRAM0")))
#define __AHBSRAM1__ __attribute__((section("AHBSRAM1")))
uint32_t __AHBSRAM1__ Buf1[DMA_SIZE];
uint32_t __AHBSRAM1__ Buf2[DMA_SIZE];
uint32_t __AHBSRAM1__ Buf3[DMA_SIZE];

#define BITBAND_SRAM_REF 0x20000000
#define BITBAND_SRAM_BASE 0x22000000
#define BITBAND_SRAM(a,b) ((BITBAND_SRAM_BASE + ((a)-BITBAND_SRAM_REF)*32 \
 + ((b)*4))) // Convert SRAM address
#define BITBAND_PERI_REF 0x40000000
#define BITBAND_PERI_BASE 0x42000000
#define BITBAND_PERI(a,b) ((BITBAND_PERI_BASE + ((a)-BITBAND_PERI_REF)*32 \
 + ((b)*4))) // Convert PERI address

#define LPC1768_PIN_PORT(pin) ((uint8_t)((pin >> 5) & 0b111))
#define LPC1768_PIN_PIN(pin) ((uint8_t)(pin & 0b11111))
// LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOSET = LPC_PIN(LPC1768_PIN_PIN(pin));

uint32_t *bit_alias(void *ptr, int bit = 0) {
  uint32_t p = (uint32_t)ptr;
  if (p >= BITBAND_SRAM_REF && p < BITBAND_SRAM_REF + 0x00100000) {
    return (uint32_t *)BITBAND_SRAM(p, bit);
  }
  if (p >= BITBAND_PERI_REF && p < BITBAND_PERI_REF + 0x00100000) {
    return (uint32_t *)BITBAND_PERI(p, bit);
  }
  return nullptr;
}

static void do_dma(){
  static int done = 0;
  if (done)
    return;
  done = 1;

	GPDMA_Channel_CFG_Type GPDMACfg;
	GPDMA_LLI_Type DMA_LLI_Struct[2];

  for (int i = 0; i< DMA_SIZE; ++i) {
    Buf1[i] = 0x1000000 + i;
    Buf2[i] = 0x2000000 + i;
  }

	/* Init GPDMA link list */
	DMA_LLI_Struct[0].SrcAddr = (uint32_t)Buf3;
	DMA_LLI_Struct[0].DstAddr = (uint32_t)Buf1;
	DMA_LLI_Struct[0].NextLLI = (uint32_t)&DMA_LLI_Struct[1];
	DMA_LLI_Struct[0].Control = (DMA_SIZE)
								| GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_HALFWORD) //source width 32 bit
								| GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_HALFWORD) //dest. width 32 bit
								//| GPDMA_DMACCxControl_SI //source increment
								| GPDMA_DMACCxControl_DI
								;
	DMA_LLI_Struct[1].SrcAddr = (uint32_t)Buf2;
	DMA_LLI_Struct[1].DstAddr = (uint32_t)Buf3;
	DMA_LLI_Struct[1].NextLLI = 0;
	DMA_LLI_Struct[1].Control = (DMA_SIZE)
								| GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_BYTE) //source width 32 bit
								| GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_BYTE) //dest. width 32 bit
								//| GPDMA_DMACCxControl_SI //source increment
								| GPDMA_DMACCxControl_DI
								;

	// Setup GPDMA channel --------------------------------
	// channel 0
	GPDMACfg.ChannelNum = 0;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t)bit_alias(Buf1); // Ignored.
	// Destination memory
	GPDMACfg.DstMemAddr = (uint32_t)Buf2;
	// Transfer size
	GPDMACfg.TransferSize = DMA_SIZE;
	// Transfer width
	GPDMACfg.TransferWidth = GPDMA_WIDTH_WORD;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
	// Source connection - unused
	GPDMACfg.SrcConn = GPDMA_CONN_MAT2_0; // DMA is driven by TIM2/TC0
	// Destination connection - unused
	GPDMACfg.DstConn = 0;
	// Linker List Item
	GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct[0];
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);

	LPC_GPDMACH0->DMACCControl
				= GPDMA_DMACCxControl_TransferSize((DMA_SIZE-1)) 
						| GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) 
						| GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) 
						| GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD) 
						| GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD) 
						| GPDMA_DMACCxControl_SI 
						| GPDMA_DMACCxControl_DI 
						| 0 /* GPDMA_DMACCxControl_I */;
  LPC_GPDMACH0->DMACCSrcAddr = (uint32_t)bit_alias(Buf1); //&(SysTick->VAL);
	// Enable GPDMA channel 0
	GPDMA_ChannelCmd(0, ENABLE);
}

#define MULTIPLES_OF(n, m) (((n) + (m) - 1) / (m))

static void TMC_STATUS(unsigned n = 0) {
#if 1
  // Cohesion 3d  
  #define MYSERIAL MYSERIAL1
  digitalWrite(P1_18, n & 1);
  digitalWrite(P1_19, n & 2);
  digitalWrite(P1_20, n & 4);
  digitalWrite(P1_21, n & 8);
#else  
  // LPCXpresso1769
  #define MYSERIAL MYSERIAL0
  digitalWrite(P3_26, !(n & 1));
  digitalWrite(P3_26, !(n & 2));
  //digitalWrite(P1_20, n & 4);
  digitalWrite(P0_22, !(n & 8));
#endif
}

static void TMC_ERROR(unsigned n = 0) {
  TMC_STATUS(n | 0x8);
  while(1);
}
// Add start/stop bits. Returns number of bytes in the buffer.
static int encode_rs232(uint8_t *out, int out_len, uint8_t *in, int in_len) {
  TMC_STATUS(1);
  int out_bytes = 1 + in_len * 12 / 8;
  if (out_len < out_bytes)
    TMC_ERROR(1);
  *out++ = 1; // Always start with logical 1.
  for (int i = 0; i < in_len; ++i) {
    *out++ = 0;           // 0 start bit
    *out++ = *in++;  // 1 
    *out++ = *in++;  // 2
    *out++ = *in++;  // 3
    *out++ = *in++;  // 4
    *out++ = *in++;  // 5
    *out++ = *in++;  // 6
    *out++ = *in++;  // 7
    *out++ = *in++;  // 8 
    *out++ = 1;           // 9 stop bit
    *out++ = 1;           // 10 gap bit
    *out++ = 1;           // 11 gap bit
  }
  return out_bytes;
}

// Strips and checks start/stop bits.
static int decode_rs232(uint8_t *out, int out_len, uint8_t *in, int in_len) {
  TMC_STATUS(2);
  int out_bytes = 0;
  int i = 0;
  out_bytes = 0;
  while(i < in_len) {
    int start_bit; 
    // Find a start bit.
    do {
      start_bit = *in++;
      i++;
    } while(start_bit && i < in_len);
    // Quit if there's not enough input bits for data + stop bit 
    // or if we don't have enough output space.
    if (i+9 >= in_len || out_bytes+8 >= out_len)
      break;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    *out++ = *in++;
    int stop_bit = *in++;
    if (stop_bit != 1)
      break; // Frame error?
    out_bytes += 8;
    i += 9;
  }
  return out_bytes;
}

// Replicate input bytes n_samples times.
static int encode_multisample(int n_samples, uint8_t *out, int out_len, uint8_t *in, int in_len) {
  TMC_STATUS(3);
  int out_bytes = in_len * n_samples;
  if (out_len < out_bytes)
    TMC_ERROR(3);
  for (int i = 0; i < in_len; ++i) {
    uint8_t bit_value = *in ? 0xff : 0;
    memset(out, bit_value, n_samples);
    out += n_samples;
    in++;
  }
  return out_bytes;
}

bool bit(int v) {
  return v != 0;
}

// Contracts input bits by n_factor times, synchronizing on signal edges.
int  decode_multisample(int n_samples, uint8_t *out, int out_len, uint8_t *in, int in_len) {
  TMC_STATUS(4);
  int out_bytes = in_len / n_samples;
  if (out_len < out_bytes)
    TMC_ERROR(4);
  int last_edge = 0;
  int mid_sample = (n_samples+1)/2;
  bool last_bit = bit(in[last_edge + mid_sample]); 
  for (int i = 0; i < out_len; ++i) {
    bool this_bit = bit(in[last_edge + mid_sample]);
    *out++ = this_bit;
    if (this_bit != last_bit) {
      // Bit flipped. Resync last_edge.
      last_edge += mid_sample;
      while(bit(in[last_edge]) == this_bit)
        last_edge--;
      last_edge++;
    }
    // Advance to the (expected) edge of the next bit.
    last_edge += n_samples;
    last_bit = this_bit;
  }
  return out_bytes;
}

int unpack_bytes(uint8_t *out, int out_len, uint8_t *in, int in_len)
{
  TMC_STATUS(5);
  int out_bytes = in_len * 8;
  if (out_len < out_bytes)
    TMC_ERROR(5);
  for (int i = 0; i < in_len; ++i) {
    uint8_t b = *in++;
    *out++ = bit((b >> 0) & 1);
    *out++ = bit((b >> 1) & 1);
    *out++ = bit((b >> 2) & 1);
    *out++ = bit((b >> 3) & 1);
    *out++ = bit((b >> 4) & 1);
    *out++ = bit((b >> 5) & 1);
    *out++ = bit((b >> 6) & 1);
    *out++ = bit((b >> 7) & 1);
  }
  return out_bytes;
}

int pack_bytes(uint8_t *out, int out_len, uint8_t *in, int in_len)
{
  TMC_STATUS(6);
  int out_bytes = in_len / 8;
  if (out_len < out_bytes)
    TMC_ERROR(6);
  for (int i = 0; i < out_bytes; ++i) {
    uint8_t b = 0;
    for (int j = 0; j < 8; ++j){
      b |= bit(*in++) << j;
    }
    *out++ = b;
  }
  return out_bytes;
}

int dma_txrx(pin_t tx_pin, pin_t rx_pin, uint8_t *out_data, int out_bytes,
             uint8_t *in_data, int in_bytes)
{
  TMC_STATUS(7);
  uint32_t rx_pin_dir = 0;
  GPDMA_Channel_CFG_Type GPDMACfg;
  GPDMA_LLI_Type PinDir_LLI, Rx_LLI;

  digitalWrite(tx_pin, 1);

  //LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOSET = LPC_PIN(LPC1768_PIN_PIN(pin))
  auto tx_gpio = LPC_GPIO(LPC1768_PIN_PORT(tx_pin));
  uint8_t *tx_pin_val = (uint8_t *)&(tx_gpio->FIOPIN);
  tx_pin_val += LPC1768_PIN_PIN(tx_pin) / 8;

  // Setup GPDMA channel --------------------------------
  // channel 0
  GPDMACfg.ChannelNum = 0;
  // Source memory
  GPDMACfg.SrcMemAddr = 0; // Ignored during P2M setup.
  // Destination memory
  GPDMACfg.DstMemAddr = (uint32_t)tx_pin_val;
  // Transfer size
  GPDMACfg.TransferSize = 1; // Will set it up lates.
  // Transfer width
  GPDMACfg.TransferWidth = GPDMA_WIDTH_WORD;
  // Transfer type
  GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
  // Source connection - unused
  GPDMACfg.SrcConn = GPDMA_CONN_MAT2_0; // DMA is driven by TIM2/TC0
  // Destination connection - unused
  GPDMACfg.DstConn = 0;
  // Linker List Item
  GPDMACfg.DMALLI = 0;
  // Setup channel with given parameter
  GPDMA_Setup(&GPDMACfg);

  // Transfer one word at a time. Both source and destination are in
  // bit band alias region, so we transfer one bit per timer match.
  LPC_GPDMACH0->DMACCControl =
      GPDMA_DMACCxControl_TransferSize(out_bytes) |
      GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
      GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
      GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_BYTE) |
      GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_BYTE) | GPDMA_DMACCxControl_SI
      //| GPDMA_DMACCxControl_DI
      ;

  LPC_GPDMACH0->DMACCSrcAddr = (uint32_t)out_data;

  if (rx_pin != -1 && in_data)
  {
    auto rx_gpio = LPC_GPIO(LPC1768_PIN_PORT(rx_pin));
    // Sets RX pin direction to input.
    rx_pin_dir = rx_gpio->FIODIR & ~(1u << LPC1768_PIN_PIN(rx_pin));
    PinDir_LLI.SrcAddr = (uint32_t) &rx_pin_dir;
    PinDir_LLI.DstAddr = (uint32_t) &(rx_gpio->FIODIR);
    PinDir_LLI.NextLLI = (uint32_t) &Rx_LLI;
    PinDir_LLI.Control = GPDMA_DMACCxControl_TransferSize(1) |
                         GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                         GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                         GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD) |
                         GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD);

    // Fills in_data with bits sampled from rx_pin.
    uint8_t *rx_pin_val = (uint8_t *)&(rx_gpio->FIOPIN);
    rx_pin_val += LPC1768_PIN_PIN(rx_pin) / 8;
    //MYSERIAL.printf("Set up RX at %08x\n", rx_pin_val);
    Rx_LLI.SrcAddr = (uint32_t) rx_pin_val;
    Rx_LLI.DstAddr = (uint32_t) in_data;
    Rx_LLI.NextLLI = (uint32_t) 0;
    Rx_LLI.Control = GPDMA_DMACCxControl_TransferSize(in_bytes) |
                     GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                     GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                     GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_BYTE) |
                     GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_BYTE) |
                     GPDMA_DMACCxControl_DI;
    if (rx_pin == tx_pin) {
      // Flip pin direction and then start receiving.
      LPC_GPDMACH0->DMACCLLI = (uint32_t)&PinDir_LLI;
    } else {
      // No need to flip direction. Start receiving right away.
      LPC_GPDMACH0->DMACCLLI = (uint32_t)&Rx_LLI;
    }
  }

  cli();
  // Mask out all but tx_pin
  uint32_t old_tx_mask = tx_gpio->FIOMASK;
  uint32_t old_rx_mask = 0;
  // Mask out pins other than tx_pin, so our writes do not affect them.
  tx_gpio->FIOMASK = ~(1u << LPC1768_PIN_PIN(tx_pin));

  // Enable GPDMA channel 0
  GPDMA_ChannelCmd(0, ENABLE);

  // TODO: Wait for DMA to finish.
  while(GPDMA_IntGetStatus(GPDMA_STAT_ENABLED_CH, 0) == SET) {
    // nothing
  }

  // restore pin mask.
  tx_gpio->FIOMASK = old_tx_mask;
  sei();
  
  if (GPDMA_IntGetStatus(GPDMA_STAT_RAWINTERR, 0) == SET)
    TMC_ERROR(7);

  if (rx_pin != -1) {
    // Go through received data and convert received bits to 1/0.
    uint8_t pin_mask = 1u << (LPC1768_PIN_PIN(rx_pin) % 8);
    for (int i = 0; i < in_bytes; ++i) {
      in_data[i] = (in_data[i] & pin_mask) ? 1 : 0;
    }
  }
  return in_bytes;
}

static bool tmc_rxtx(pin_t tx_pin, pin_t rx_pin, uint8_t *data, int tx_length, int rx_length) {
  if (tx_length > tmc_msg_bytes)
    return false;

  static uint8_t __AHBSRAM1__ data_bits[(tmc_msg_bytes+tmc_gap_bytes+2) * 8];
  static uint8_t __AHBSRAM1__ uart_bits[sizeof(data_bits)*12/8];
  static uint8_t __AHBSRAM1__ dma_bits[sizeof(uart_bits)*tmc_oversampling];
  static uint8_t __AHBSRAM1__ dma_bits_in[sizeof(uart_bits)*tmc_oversampling];
  
  // Make sure the data is in AHB RAM as we need it for bit banding.
  int tx_bit_len = unpack_bytes(data_bits, sizeof(data_bits), data, tx_length);
  int tx_rs232_bytes = encode_rs232(uart_bits, sizeof(uart_bits), data_bits, tx_bit_len);
  int tx_dma_bytes = encode_multisample(tmc_oversampling, dma_bits, sizeof(dma_bits),
                                        uart_bits, tx_rs232_bytes);

  int rx_dma_bytes = dma_txrx(tx_pin, rx_pin, dma_bits, tx_dma_bytes, dma_bits_in, sizeof(dma_bits_in));
  //int rx_buf_bytes = dma_txrx(tx_pin, rx_pin, buf_a, tx_buf_bytes, buf_b, sizeof(buf_b));
  if (rx_pin != -1) {
    int rx_rs232_bytes = decode_multisample(tmc_oversampling, uart_bits, sizeof(uart_bits), dma_bits_in, rx_dma_bytes);
    int rx_bit_len = decode_rs232(data_bits, sizeof(data_bits), uart_bits, rx_rs232_bytes);
    if (rx_bit_len < 0) // Framing error.
      return false;
    int rx_data_bytes = pack_bytes(data, rx_length, data_bits, rx_bit_len);
#if 0    
    MYSERIAL.printf("\nRX %d: ", rx_data_bytes);
    for (int i = 0; i < rx_data_bytes; ++i) {
      MYSERIAL.printf("%02x ", data[i]);
    }
    MYSERIAL.printf("\n");
    MYSERIAL.flushTX();
#endif
    if (rx_data_bytes != tmc_msg_bytes) {
      TMC_STATUS(7);
      return false;
      TMC_ERROR(7);
    }
  }
  TMC_STATUS(0);
  return true;
}

static bool tmc_rw(pin_t tx_pin, pin_t rx_pin, uint8_t addr, uint32_t regVal, uint32_t *result = nullptr) {
  uint8_t datagram[8] = {TMC2208_SYNC,
                         TMC2208_SLAVE_ADDR,
                         (uint8_t)(addr | (result ? TMC2208_READ : TMC2208_WRITE)),
                         (uint8_t)(regVal >> 24),
                         (uint8_t)(regVal >> 16),
                         (uint8_t)(regVal >> 8),
                         (uint8_t)(regVal >> 0),
                         0x00};
  //uint8_t datagram1[8] = {0, 0xff, 0xF0, 0x0F, 0xc0, 0x0c, 0x55, 0xaa};
  int len = result ? 3 : 7; // If result != nullptr, it means we must read stuff.
  datagram[len] = tmc_calcCRC(datagram, len);
  tmc_rxtx(tx_pin, result ? rx_pin : -1, datagram, len+1, sizeof(datagram));
  if (result) {
    uint8_t crc = tmc_calcCRC(datagram, 7);
    *result = ((uint32_t)datagram[3] << 24) | ((uint32_t)datagram[4] << 16) |
              ((uint32_t)datagram[5] << 8) | ((uint32_t)datagram[6]);
    return crc == datagram[7];
  }
  return true;
}

// Address-only packet
static bool tmc_read_reg(pin_t tx_pin, pin_t rx_pin, uint8_t addr, uint32_t *regVal) {
  return tmc_rw(tx_pin, rx_pin, addr, 0, regVal);
}
static void tmc_write_reg(pin_t tx_pin, uint8_t addr, uint32_t regVal) {
  tmc_rw(tx_pin, -1, addr, regVal);
}


TMC2208StepperUART::TMC2208StepperUART(pin_t tx_pin, pin_t rx_pin)
    : _rx_pin(rx_pin), _tx_pin(tx_pin) {
  if (_tx_pin != _rx_pin) {
    digitalWrite(_tx_pin, 1);
  }
  pinMode(_rx_pin, INPUT_PULLUP);
  // Set up timer 3 as free-running counter at CCLK
  // Enable power
  CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, ENABLE);
  // init timer. It sets PCLK=CCLK/4
  TIM_TIMERCFG_Type cfg;
  cfg.PrescaleOption = TIM_PRESCALE_TICKVAL;
  cfg.PrescaleValue = 1;
  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &cfg);
  // Set clock div to /1
  CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER2, CLKPWR_PCLKSEL_CCLK_DIV_1);
  TIM_MATCHCFG_Type MC;
  MC.MatchChannel = 0; 
  MC.ResetOnMatch = true;
  MC.IntOnMatch = false;
  MC.StopOnMatch = false;
  MC.ExtMatchOutputType = 0; // No external pin toggling
  MC.MatchValue = 120;  // times oversampling -> < 1Mbaud.
  TIM_ConfigMatch(LPC_TIM2, &MC);
  TIM_Cmd(LPC_TIM2, ENABLE);

    /* Initialize GPDMA controller */
	GPDMA_Init();
}

void TMC2208StepperUART::write_register(uint8_t addr, uint32_t regVal) {
  pinMode(_tx_pin, OUTPUT);
  // regVal = 0x55aac33c;
  tmc_write_reg(_tx_pin, addr, regVal);
  pinMode(_rx_pin, INPUT_PULLUP);
}

bool TMC2208StepperUART::read_register(uint8_t addr, uint32_t *value) {
    pinMode(_tx_pin, OUTPUT);
    return tmc_read_reg(_tx_pin, _rx_pin, addr, value);
}
