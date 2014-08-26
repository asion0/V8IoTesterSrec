/*******************************************************************************
 * Copyright (c) 2014 SkyTraq Technology, Inc. Hsinchu, Taiwan
 * All Rights Reserved.
 * 
 * file name: flashloader.c
 * Initial: 
 * Current: 
 ******************************************************************************/

//---------- head file inclusions -------------------------
#include "flashloader_v8.h"
//#define UART_INIT_BAUDRATE  7 //0: 4800, 1:9600, 2:19200; 3:38400; 4:57600; 5:115200; 6: 230400: 7:460800, 8:921600
typedef volatile unsigned long Register;
//---------- local definitions ----------------------------
#define MAKE_U32(a,b,c,d)   ( (((a)<<24)&0xFF000000) | (((b)<<16)&0xFF0000) | (((c)<<8)&0xFF00) | ((d)&0xFF))
//---------- local definitions ----------------------------
#define BUILD_MONTH  (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? "01" : "06") \
                    : __DATE__ [2] == 'b' ? "02" \
                    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? "03" : "04") \
                    : __DATE__ [2] == 'y' ? "05" \
                    : __DATE__ [2] == 'l' ? "07" \
                    : __DATE__ [2] == 'g' ? "08" \
                    : __DATE__ [2] == 'p' ? "09" \
                    : __DATE__ [2] == 't' ? "10" \
                    : __DATE__ [2] == 'v' ? "11" : "12")

#if defined(V838)
 #define REV_ID "V838_"
 #define IO_TEST
 #define TEST_RTC
#elif defined(V858)
 #define REV_ID "V858_"
 #define IO_TEST
 #define TEST_RTC
 #define GSN_MAG_TEST
#elif defined(V838R)
 #define REV_ID "V838R"
 //#define IO_TEST
 #define TEST_RTC
 //#define GSN_MAG_TEST
#elif defined(V822) 
 #define REV_ID "V822_"
 #define IO_TEST
 #define TEST_RTC
#define GSN_MAG_TEST
#endif


char _LOADER_VER_[30];
//---------- local types ----------------------------------
typedef struct {
  U08 test_type;   // 0=normal,1=just output
  U08 pin0_type;    // 0=gpio, 1=pio
  U08 pin0;
  U08 pin1_type;
  U08 pin1;
  U08 other_pin_type;
  U08 other_pin;
} IO_TEST_T;

typedef enum
{
  V821 = 0, 
  NavSpark = 2
} IcType;
IcType testChip = NavSpark;
  
//---------- static global variables ----------------------
U32 uart_offset = 0;
unsigned char chipmode = 0;
U32 mybaud = 0;  
const int ADC_COUNT = 10;
//---------- static function declarations -----------------
static void GetVersion(void);
static void InitSystem();
static U32 GetInt(const int start, unsigned char* buf);
static void DebugOutput(const char *title, U32 data);
static void ErrorEnd(int error);
static void gps1sputs(const char *s);
static void UartPutLine(const char *s);
inline void Delay (U16 DelayCount);

#if defined(IO_TEST)
static void gnss_gpio_set_mux_mode();
static U08 GPIO_Test();
#endif

#if defined(GSN_MAG_TEST)	
static U08 GSN_MAG_Test();
#endif

#if defined(TEST_RTC)	
static U08 Rtc_Test();
#endif

//---------- functions definition --------------------------------------------//
int main()
{
  InitSystem();
  GetVersion();
  gps1sputs(_LOADER_VER_);
  gps1sputs("END");
  U08 result;
  U08 failReturn = FALSE;
  
#if defined(IO_TEST)
  gnss_gpio_set_mux_mode();
  result = GPIO_Test();
	if (result == FALSE)
	{
		UartPutLine("FALSE\r\n");
		failReturn = TRUE;
	}
#endif

#if defined(GSN_MAG_TEST)	
  result = GSN_MAG_Test();
	if (result == FALSE)
	{
		UartPutLine("FALSE\r\n");
		failReturn = TRUE;
	}
#endif

#if defined(TEST_RTC)
  result = Rtc_Test();
	if (result == FALSE)
	{
		UartPutLine("FALSE\r\n");
		failReturn = TRUE;
	}
#endif

  if(failReturn == TRUE)
  {
    UartPutLine("FAIL\r\n");
  }
  else
  {
    UartPutLine("PASS\r\n");
  }
Reboot:
  UartPutLine("FINISH\r\n"); 
  //volatile int aaa;
  if(((*(volatile unsigned long *)(0x2000f010))&0x80000000)!=0)//leon2
  {
    //*((volatile unsigned short *) (0x0)) = 0xf0;
    //for(aaa=0;aaa<0x200000;aaa++);//need delay here or the END will not be sent out
    while(((*((volatile unsigned long *) (0x2000242c)))&0x8)==0);
    *((volatile unsigned short *) (0x20004c20)) = *((volatile unsigned short *) (0x20004c20))& 0xfffffffe;
    *((volatile unsigned long *) (0x8000004c)) = 0x10;
    //cause need some delay between counter & enable
    *((volatile unsigned long *) (0x800000b8))=*((volatile unsigned long *) (0x8000004c));
    *((volatile unsigned long *) (0x20001014)) = 
    ((*((volatile unsigned long *) (0x20001014))) & 0xfffffffe);
	} //if(((*(volatile unsigned long *)(0x2000f010)) & 0x80000000) != 0)//leon2
  else
  {
    if(mybaud<9)
    {
      while(((iord(UART1_CESR+uart_offset))&0x8)==0)
      {
      };
    }
    else
    {
      gps1sputs("END"); 
      while(( *((volatile unsigned long *) (SPI_S_MAIN))&0x4)==1){};
      //want to disable SPI_SLAVE
      //for(aaa=0;aaa<0x2000000;aaa++);//need delay here or the END will not be sent out
      *((volatile unsigned short *) (0x2000f004)) = *((volatile unsigned short *) (0x2000f004))& 0xfffffffa;
    }
    *((volatile unsigned short *) (0x20004c20)) = *((volatile unsigned short *) (0x20004c20))& 0xfffffffe;
    //below for leon2
      *((volatile unsigned long *) (0x8000004c)) = 0x10;
    //below for leon3
    *((volatile unsigned long *) (0x80000344)) = 0x10;
    *((volatile unsigned long *) (0x80000348)) = 0xd;

    //cause need some delay between counter & enable
    *((volatile unsigned long *) (0x800000b8))=*((volatile unsigned long *) (0x8000004c));
    //below for leon 3
    *((volatile unsigned long *) (0x20001014)) = 
    ((*((volatile unsigned long *) (0x20001014))) & 0xfffffffe);
  } //else | if(((*(volatile unsigned long *)(0x2000f010)) & 0x80000000) != 0) els
  while(1);
Error4:
  //while(1);
  gps1sputs("Error4");
  while(1);
}

void WD_disable()
{
  *((volatile unsigned long *) (0x20001014)) =
  ((*((volatile unsigned long *) (0x20001014))) | 0x00000001);
}

unsigned long iord (unsigned long addr)
{
  volatile unsigned long *ioreg32 = (volatile unsigned long *) REG_BASE_MEMORYIO_ADDR;
  unsigned long   out;

  //disable_irq(7);
  out =  ioreg32[addr/4];
  //enable_irq(7); 
  return (out);
}

void iowr (unsigned long addr, unsigned long data)
{
  volatile unsigned long *ioreg32 = (volatile unsigned long *) REG_BASE_MEMORYIO_ADDR;
  //  disable_irq(7);
  ioreg32[addr/4] = data ;
  //enable_irq(7);
}

static U08 SPI_S_wr_buff_read(U08 *buf, U08 cnt)
{
  U32 value;
  U08 idx=0;
  U08 j;
  U16 i;
  //cnt=cnt/4;
  if(cnt>16)
    return 0;
  for(i=0;i<cnt;i++)
  {
    if(i<idx)
      continue;
    value=*((volatile unsigned long *) (SPI_S_WRBUF_BASE+idx));
    for(j=0;j<4;j++)
    {
      if((idx+j)<cnt)
        buf[idx+j] = (value>>(8*j))&0xff;
    }
    //buf[idx+3]=value>>24;
    //buf[idx+2]=(value>>16)&0xff;
    //buf[idx+1]=(value>>8)&0xff;
    //buf[idx]=value&0xff;
    idx = idx + 4;
  }
  return 1;
}

static void SPI_S_write_reg(U32 reg, U32 value)
{
  (*((volatile unsigned long *) (reg)))=value;
}

static U32 SPI_S_read_reg(U32 reg)
{
  return (*((volatile unsigned long *) (reg)));
}

void gps1sgets(unsigned char *s, U32 mybaud) 
{
  if(mybaud<9)
  {
    unsigned long i,j,k;
    unsigned char a;
    volatile int *ioreg = (volatile int *) REG_BASE_MEMORYIO_ADDR;
    unsigned char slength;
    U32  status;
    do 
    {
    
      status=iord (UART1_RBBC+uart_offset); 
      slength=status&0x1f;
      while (slength==0)
      {
        status=iord (UART1_RBBC+uart_offset); 
        slength=status&0x1f;
         };
         for(j=0;j<(slength);j++)
            {
        i=iord (UART1_RBR+uart_offset);
        *s=(unsigned char)(i&0xff);
        if ((*s == '\0')) 
        {
          return;
        }
        s++;
      }
    } while (1);
  }
  else
  {
    U32 value;
    U32 size;
    U08 inputbuf[16];
    U08 j;
    do
    {
      value=SPI_S_read_reg(SPI_S_MAIN);
      if((value&0x2)==0)
      {
        //s_rx_byte=value;
        size=(value>>8)&0xff;
        SPI_S_wr_buff_read(inputbuf, size);
        value=value|0x2;
        SPI_S_write_reg(SPI_S_MAIN, value);
        for(j=0;j<(size);j++)
        {
          *s=(unsigned char)(inputbuf[j]&0xff);
          if ((*s == '\0')) 
          {
            return;
          }
          s++;
        }
      }
    } while(1);
  }
}

static U08 SPI_S_rd_buff_write(const U08 *buf, U08 cnt)
{
   U16 i;
   
   U32 value;
   U08 idx=0;
   //cnt=cnt/4;
   if(cnt>16)
   return 0;
   for(i=0;i<cnt;i++)
   {
      if(i<idx)
      continue;
      value=buf[idx+3];
      value=value<<8 | buf[idx+2];
      value=value<<8 | buf[idx+1];
      value=value<<8 | buf[idx];
      *((volatile unsigned long *) (SPI_S_RDBUF_BASE+idx)) = value;
      idx=idx+4;
   }
   return 1;
}

static U08 SPI_S_set_rd_buff_bytes(U32 bytes)
{
   if(bytes>16)
   return 0;
   *((volatile unsigned long *) (SPI_S_MAIN)) = ((*((volatile unsigned long *) (SPI_S_MAIN)))&0xff00ffff)
   | (bytes<<16);
   return 1;
}

static void SPI_S_set_rd_buff_ready()
{
   *((volatile unsigned long *) (SPI_S_MAIN)) = *((volatile unsigned long *) (SPI_S_MAIN))
   | 0x4;
}

static void gps1sputs0(const char *s, int s_len, U32 mybaud)
{
  if(mybaud<9)
  {
    unsigned long len = 0;
    if(s_len>0)
    {
      for(len=0;len<s_len;len++)
      {
        //if(((iord(UART1_TBBC)&0x1f) !=0x0))
        while (((iord(UART1_TBBC+uart_offset)&0x1f) ==0x0));
          iowr(UART1_THR+uart_offset, *s++);
      }
      //while (((iord (UART1_TBBC)&0x1f)!=0x10))   ;
      //while((iord(UART1_CESR)&0x8)==0);
    }
    else
    {
        while(*s) 
        {
          if(((iord(UART1_TBBC+uart_offset)&0x1f) !=0x0))
          {
          //while (((iord (UART1_PISR)&0x02)==0))   ;
            iowr(UART1_THR+uart_offset, *s++);
          }
        } while (((iord (UART1_TBBC+uart_offset)&0x1f)==0))   ;
      iowr(UART1_THR+uart_offset, '\0');
    }
  } //if(mybaud<9)
  else
  {
    if(s_len>0)
    {
      SPI_S_rd_buff_write(s, s_len);
      SPI_S_set_rd_buff_bytes(s_len);
      SPI_S_set_rd_buff_ready();
    }
    else
    {
      U08 count=0;
      U08 outbuf[16];
      while(*s) 
      {
        outbuf[count]=*s++;
        count++;
      }
      outbuf[count]='\0';
      count++;
      SPI_S_rd_buff_write(outbuf, count);
      SPI_S_set_rd_buff_bytes(count);
      SPI_S_set_rd_buff_ready();
    }
  }
}

static void UartPutLine(const char *s)
{
	while(*s) 
	{
	  if(((iord(UART1_TBBC+uart_offset) & 0x1f) !=0x0))
	  {
	    iowr(UART1_THR+uart_offset, *s++);
	  }
	} 
	while (((iord (UART1_TBBC+uart_offset) & 0x1f)==0));
}

static void gps1sputs(const char *s)
{
  gps1sputs0(s, 0, 0);
}

int gps1sgetsbin(unsigned char *s, int size, U32 mybaud) 
{
  unsigned long binlength = 0;
  do 
  {
      U32 status = iord(UART1_RBBC + uart_offset); 
      unsigned long slength = status & 0x1f;
    while (slength==0)
    {
      status=iord(UART1_RBBC+uart_offset); 
      slength=status&0x1f;
       }
       
       unsigned long i, j;
      for(j=0; j<slength; j++)
          {
      i=iord (UART1_RBR+uart_offset);
      *s=(unsigned char)(i&0xff);
          ++binlength;
      if (binlength == size) 
      {
      return binlength;
      }
            s++;
    }
  } while (1);
}

VOID UART_init(U08 baudidx)
{
  U32 register_baud;
  U32 baudrate[9]={4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  //U32 register_baud;

  U32 reg_val2;
  U08 uart_boost;
  reg_val2=*(volatile unsigned long *)(0x2000f004);
  if((baudidx==6)||(baudidx==7))
  {
    reg_val2=(reg_val2&0x3fffffff) | 0x40000000;
    uart_boost=2;  
  }
  else
  if((baudidx==8))
  {
    
    reg_val2=(reg_val2&0x3fffffff) | 0x80000000;
    uart_boost=4;  
  }
  else  //set back
  {
    reg_val2=reg_val2&0x3fffffff;
    uart_boost=1;
  }
  *(volatile unsigned long *)(0x2000f004)=reg_val2;


  U32 f08_val;
  *((volatile unsigned long *)(0x2000f018))=0x20000;
  f08_val=*((volatile unsigned long *)(0x2000f018));
  while((f08_val&0x20000)!=0)
  {
     f08_val=*((volatile unsigned long *)(0x2000f018));
   };//the bit 17 case not zero should wait to go further
  if(((f08_val&0x20000)==0)&&((f08_val&0x10000)==0))
  {
   f08_val=61378751*uart_boost;
  }
  else
  if(((f08_val&0x20000)==0)&&((f08_val&0x10000)!=0))
  {
   f08_val=f08_val&0xffff;
   f08_val=f08_val/8;
   f08_val=f08_val*32768;
  }
  else
  {
  //should not come here
  f08_val=61378751*uart_boost;
  }
  
  //register_baud=((f08_val/115200)*reg_val)/16;
  register_baud=((f08_val/baudrate[baudidx]))/16;


  
  iowr (UART1_LCR+uart_offset, 0x83); // set DLAB=1, set transaction bits=8
  iowr (UART1_DLL+uart_offset, register_baud&0xff); // now uart uses 50MHz clk, so, 50M/(px51 * 0x10) = 38400Hz (baud rate) 
  iowr (UART1_DLM+uart_offset, (register_baud>>8)&0xff);
  iowr (UART1_LCR+uart_offset, 0x03); // set DLAB=0
  iowr (UART1_FCR+uart_offset, 0x86); // 1. reset TX/RX FIFO,  2. set RX FIFO trigger level to 8 bytes
  //iowr (UART1_IER, 0x03); // set interrupt enables, 0x03==> TX/RX fifo status

  while((iord(UART1_CESR+uart_offset)&0x6)!=0);
}

static void InitSystem() 
{
  Register tmp;
  volatile int *lreg = (volatile int *) REG_BASE_ADDR;
  
  tmp = lreg[MCFG1/4];
  lreg[MCFG1/4] = (tmp& 0xffff0000) | (FLASH_WIDTH/FLASH_CHIP_WIDTH)<<8 | 0x877;

  if(((*(volatile unsigned long *)(0x2000f010))&0x80000000)!=0)//leon2
  {
    lreg[IMASK_LEON2/4] = 0x0; /* mask all interrupt */
    //cache keep to backward to ML605 leon2 board
    tmp = lreg[CACFG/4]; /* disable and flush cache */
    lreg[CACFG/4] = (tmp | 0x600000) & 0xfffffff0;  
  }
  else
  {
    lreg[IMASK/4] = 0x0; /* mask all interrupt */
  }
  
  tmp = lreg[MCFG1/4]; 
  lreg[MCFG1/4] = (tmp&0xffff)| 0x10080000; // 15 wait state 
  //cpu boost

  if(((*(volatile unsigned long *)(0x2000f000))&0x1)!=0)//uart 0
  {
    uart_offset = 0;
  }
  else
  {
    uart_offset = 0x100;
  }
  
  //check boot from where
  tmp = (*(volatile unsigned long *)(0x2000f010));
  chipmode = (tmp >> 20) & 0x7;
  
	//analogADCClock(10000000);
} //InitSystem()

static U32 GetInt(const int start, unsigned char* buf)
{
  U32 data = 0;
  static U08 i = 0;

  if(-1 == start)
  {
    i = 0;
    return 0;
  }
  
  i += start;
  for(; i < 100; i++)
  {
    if (buf[i] != ' ')
    {
      data = data*10 + buf[i] - 0x30;
    }
    else
    {
      break;
    }
  }
  return data;
}

static void DebugOutput(const char *title, U32 data)
{
  char buf[128];
  int i = 0;
  while(title[i])
  {
    buf[i] = title[i];
    ++i;
  }
  
  unsigned long long m = 0x0F;
  int j = 0;
  for(;j < 8; ++j)
  {
    char c = (data >> (7- j) * 4 ) & m;
    if(c > 9)
    {
      buf[i] = c - 10 + 'A'  ;
    }
    else
    {
      buf[i] = c + '0';
    }
    ++i;
  }
  buf[i] = 0;
  UartPutLine(buf);
}

static void ErrorEnd(int error)
{
  char estring[8] = "Error0";
  if(error < 10)
  {
    estring[5] = '0' + error;
  }
  while(1)
  {
    gps1sputs(estring);
  }
}

static void GetVersion(void)
{
  _LOADER_VER_[0] = '$';
  _LOADER_VER_[1] = 'I';
  _LOADER_VER_[2] = 'O';
  _LOADER_VER_[3] = 'T';
  _LOADER_VER_[4] = 'E';
  _LOADER_VER_[5] = 'S';
  _LOADER_VER_[6] = 'T';
  _LOADER_VER_[7] = ',';
  _LOADER_VER_[8] = REV_ID[0];
  _LOADER_VER_[9] = REV_ID[1];
  _LOADER_VER_[10] = REV_ID[2];
  _LOADER_VER_[11] = REV_ID[3];
  _LOADER_VER_[12] = REV_ID[4];
  _LOADER_VER_[13] = ',';
  _LOADER_VER_[14] = __DATE__[9];
  _LOADER_VER_[15] = __DATE__[10];
  _LOADER_VER_[16] = BUILD_MONTH[0];
  _LOADER_VER_[17] = BUILD_MONTH[1];
  _LOADER_VER_[18] = (__DATE__[4]==' ') ? '0' : __DATE__[4];
  _LOADER_VER_[19] = __DATE__[5];
  _LOADER_VER_[20] = __TIME__[0];
  _LOADER_VER_[21] = __TIME__[1];
  _LOADER_VER_[22] = __TIME__[3];
  _LOADER_VER_[23] = __TIME__[4];
  _LOADER_VER_[24] = __TIME__[6];
  _LOADER_VER_[25] = __TIME__[7];
  _LOADER_VER_[26] = 0x0d;
  _LOADER_VER_[27] = 0x0a;
  _LOADER_VER_[28] = 0;
}
//---------------------------------------------------------
#define GPIOIO 0xA0
#define GPIODIR 0xA4
#define GPIO_OUTPUT_ADDR       (0x20001008UL)
#define GPIO_DIR_ADDR          (0x2000100CUL)
U32 pio_val = 0x0;

#define OUTPUT_MODE   1
#define INPUT_MODE      0
#define GPIO_TYPE   0
#define PIO_TYPE      1
/*
GPIO
A partially bit-wise programmable 32-bit I/O port is provided on-chip. 
Each bit bits of the I/O port can be individually programmed as output 
or input.

Two registers are associated with the operation of the I/O port; the 
combined GPIO input/output register (0x20001008[31:0]), and GPIO 
direction register (0x2000100c[31:0]). The direction register defines 
the direction for each individual port bit (0=output, 1=input).

*/
inline void gpio_set_output( int gpio )
{
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_DIR_ADDR;
  unsigned long int reg = *lreg;
  reg  &= ~(0x1UL << gpio);
  *lreg = reg;
}

inline void gpio_set_input( int gpio )
{
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_DIR_ADDR;
  unsigned long int reg = *lreg;
  reg  |= 0x1UL << gpio;
  *lreg = reg;
}


inline void gpio_high( int gpio )
{
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_OUTPUT_ADDR;
  unsigned long int reg = *lreg;
  reg  |= 0x1UL << gpio;
  *lreg = reg;
}

//---------------------------------------------------------
inline void gpio_low( int gpio )
{
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_OUTPUT_ADDR;
  unsigned long int reg = *lreg;
  reg  &= ~(0x1UL << gpio);
  *lreg = reg;
}

inline S16 gpio_read_input( int gpio )
{
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_OUTPUT_ADDR;
  return ( (*lreg&(0x1UL<<gpio)) != 0 );
}

inline void Delay (U16 DelayCount)
{
  U16 i;
  for(i=0;i<(DelayCount * 8);i++)
  asm("nop;");
}

#if defined(IO_TEST)

static U08 taggle = 0;
U08 GPIO_TestOneGroup(IO_TEST_T *test)
{
  gpio_set_output(test->pin0);
  gpio_set_input(test->pin1);

  gpio_high(test->pin0);
 
  if (gpio_read_input(test->pin1) == 0) 
  {
//  gps1sputs("A2\r\n");
    return FALSE;
  }

//gps1sputs("A3\r\n");
  gpio_low(test->pin0);
	Delay(1000);
  if (gpio_read_input(test->pin1) == 1) 
  {
//  gps1sputs("A4\r\n");
    return FALSE;
  }
  
//gps1sputs("A5\r\n");
  if (test->test_type == 1)   // pin0 is just output
  {
    return TRUE;
  }
  
  gpio_set_output(test->pin1);
  gpio_set_input(test->pin0);
   
  gpio_high(test->pin1);
  Delay(1000);
  if (gpio_read_input(test->pin0) == 0) 
  {
//    gps1sputs("A6\r\n");
    return FALSE;
  }

  gpio_low(test->pin1);
  Delay(1000);
//	gps1sputs("A7\r\n");
  if (gpio_read_input(test->pin0) == 1) 
  {
//  gps1sputs("A8\r\n");
    return FALSE;
  }
//	gps1sputs("A9\r\n");
  return TRUE;
}

static void gnss_gpio_set_mux_mode()
{
  volatile U32 *ioreg = (volatile U32 *)0x20000000;
	//case 0: // buf_gpsclk
	ioreg[0xf014/4] &= (~(0x1UL << 8));

	//case 1: // buf_sgn/urxd2
	//case 2: // buf_mag/utxd2
  // GPIO => buf_rf_sig[2:1] = 0x0 (0x2000_f014[10:9]=0x0)
  //         dis_uart2_out = 0x1 (0x2000_f078[1]=0x1)
  // SPEC => buf_rf_sig[2:1] = 0x0 (0x2000_f014[10:9]=0x0)
  //         dis_uart2_out = 0x0 (0x2000_f078[1]=0x0)
	ioreg[0xf014/4] &= (~(0x3UL << 9));
	ioreg[0xf078/4] |= (0x1UL << 1);

	//case 3: // buf_gloclk/p1pps
  // GPIO => buf_rf_sig[3] = 0x0 (0x2000_f014[11]=0x0)
  //         dis_p1pps_out = 0x1 (0x2000_f078[0]=0x1)
  // SPEC => buf_rf_sig[3] = 0x0 (0x2000_f014[11]=0x0)
  //         dis_p1pps_out = 0x0 (0x2000_f078[0]=0x0)
  ioreg[0xf014/4] &= (~(0x1UL << 11));
  ioreg[0xf078/4] |= (0x1UL << 0);

	//case 4: // buf_sgn2/i2c_scl
  //case 5: // buf_mag2/i2c_sda
  // GPIO => buf_rf_sig[5:4] = 0x0 (0x2000_f014[13:12]=0x0)
  //         dis_i2cms = 0x1 (0x2000_f078[8]=0x1)
  // SPEC => buf_rf_sig[5:4] = 0x0 (0x2000_f014[13:12]=0x0)
  //         dis_i2cms = 0x0 (0x2000_f078[8]=0x0)
  ioreg[0xf014/4] &= (~(0x3UL << 12));
  ioreg[0xf078/4] |= (0x1UL << 8);

	//case 6: // spim_cs2/wtic_in
	//case 17: // spim_cs3/jam_bit[0]
	// GPIO => extra_spim_ss = 0x0 (0x2000_f004[29]=0x0)
	// SPEC => extra_spim_ss = 0x1 (0x2000_f004[29]=0x1)
	ioreg[0xf004/4] &= (~(0x1UL << 29));

	//case 7: // extra_pps
	// GPIO => pps10mout_en = 0x0 (0x2000_f014[6]=0x0)
	// SPEC => pps10mout_en = 0x1 (0x2000_f014[6]=0x1)
	ioreg[0xf014/4] &= (~(0x1UL << 6));

  //case 8: // wtic_meas_ps
  // GPIO => en_meas_ps = 0x0 (0x2000_f00c[16]=0x0)
  // SPEC => en_meas_ps = 0x1 (0x2000_f00c[16]=0x1)
  ioreg[0xf00c/4] &= (~(0x1UL << 16));

  //case 9: // sync_1ms_ps
  // GPIO => syncout_en = 0x0 (0x2000_f014[5]=0x0)
  // SPEC => syncout_en = 0x1 (0x2000_f014[5]=0x1)
  ioreg[0xf014/4] &= (~(0x1UL << 5));

	//case 13: // therm_out1
	//case 15: // therm_out2
	// GPIO => ena_therm = 0x0 (0x2000_f014[31]=0x0)
	// SPEC => ena_therm = 0x1 (0x2000_f014[31]=0x1)
	ioreg[0xf014/4] &= (~(0x1UL << 31));
  
	//case 16: // cheap_xtal_out
	// GPIO => en_xtal_gpsclk = 0x0 (0x2000_f004[28]=0x0)
	// SPEC => N/A
	ioreg[0xf004/4] &= (~(0x1UL << 28));

	//case 20: // pwm0/DIR_IN
	// GPIO => pwm0_en = 0x0 (0x2000_f02c[24]=0x0)
	// SPEC => pwm0_en = 0x1 (0x2000_f02c[24]=0x1)
	ioreg[0xf02c/4] &= (~(0x1UL << 24));

	//case 21: // pwm1
	// GPIO => pwm1_en = 0x0 (0x2000_f034[24]=0x0)
	// SPEC => pwm1_en = 0x1 (0x2000_f034[24]=0x1)
	ioreg[0xf034/4] &= (~(0x1UL << 24));

	//case 22: // pio5/mcs1
	ioreg[0x4c18/4] &= (~(0x1UL << 23)); // spi_ena = 0

	//case 23: // pio6/mcs0
	//case 24: // pio7/msck
	//case 26: // pio9/msout
	ioreg[0xf078/4] &= (~(0x1UL << 3)); // loop_spims = 0
	ioreg[0x4c18/4] &= (~(0x1UL << 23)); // spi_ena = 0

  //case 27: // cpu_clk_out
  // GPIO => en_lclkout = 0x0 (0x2000_f014[14]=0x0)
  // SPEC => en_lclkout = 0x1 (0x2000_f014[14]=0x1)
  ioreg[0xf014/4] &= (~(0x1UL << 14));

  //case 28: // spis_csb/spims_csb
  //case 29: // spis_sck/spims_sck
  //case 30: // spis_sdi/spims_mosi
  //case 31: // spis_sdo/spims_miso
  ioreg[0xf078/4] &= (~(0x1UL << 3));
 	ioreg[0xf078/4] &= (~(0x1UL << 2));
  ioreg[0xf004/4] &= (~(0x1UL << 0));
}

/*
  U08 test_type;   // 0=normal,1=just output
  U08 pin0_type;    // 0=gpio, 1=pio
  U08 pin0;
  U08 pin1_type;
  U08 pin1;
  U08 other_pin_type;
  U08 other_pin;
*/
/*
IO_TEST_T io_test[] = {
  {0, 0,  5, 0, 11, 0, 0},
  {1, 0,  3, 0,  4, 0, 0},
  {0, 0, 14, 0, 15, 0, 0},
  {0, 0, 29, 0, 30, 0, 0},
  {0, 0, 22, 0, 23, 0, 0},
  {0, 0,  2, 0, 12, 0, 0},
  {0, 0,  0, 0,  1, 0, 0},
  {0, 0,  6, 0, 28, 0, 0},
  {0, 0, 25, 0, 30, 0, 0},
 // {1, 0,  0, 0, 24, 9, 0},
}; 
const int TEST_GPIO_COUNT = 9;
*/     
IO_TEST_T io_test[] = {
#if defined(V822)
  {0, 0, 30, 0, 28, 0, 0},
  {0, 0, 22, 0, 28, 0, 0},
//  {0, 0, 30, 0, 28, 0, 0},
  {1, 0,  3, 0, 14, 0, 0},
  {0, 0, 12, 0, 13, 0, 0},
  {0, 0,  1, 0, 31, 0, 0},
  {0, 0, 29, 0, 31, 0, 0},
  {1, 0,  3, 0,  2, 0, 0},
  {0, 0, 16, 0,  7, 0, 0},
  {0, 0, 20, 0,  0, 0, 0},


/*
  {0, 0,  0, 0, 20, 0, 0},
  */
#else
  {0, 0, 25, 0, 24, 0, 0},
  {0, 0,  0, 0,  2, 0, 0},
  {0, 0,  7, 0,  6, 0, 0},
  {0, 0,  5, 0, 23, 0, 0},
  {0, 0,  4, 0, 12, 0, 0},
  {0, 0,  9, 0, 16, 0, 0},
  {0, 0, 13, 0,  8, 0, 0},
  {0, 0, 15, 0, 20, 0, 0},
  {0, 0, 10, 0, 11, 0, 0},
  {0, 0, 21, 0, 27, 0, 0},
#endif
};
const int TEST_GPIO_COUNT = sizeof(io_test) / sizeof(io_test[0]);

static U08 GPIO_Test()
{
  int i;
  int passCount = 0;
  for(i=0;i<TEST_GPIO_COUNT;i++)
  {
    if (GPIO_TestOneGroup(&io_test[i]) == 0) 
    {
		  DebugOutput(" p0 ", io_test[i].pin0);
		  DebugOutput(" p1 ", io_test[i].pin1);
		  UartPutLine(" NG\r\n");
		  //return FALSE;
    }
    else
    {
		  DebugOutput(" p0 ", io_test[i].pin0);
		  DebugOutput(" p1 ", io_test[i].pin1);
		  UartPutLine(" OK\r\n");
		  passCount++;
    }
  }
  if(passCount==TEST_GPIO_COUNT)
  {
    return TRUE;
  }
  return FALSE;
}
#endif

#if defined(GSN_MAG_TEST)	
static U08 GSN_MAG_Test()
{
  volatile U32 *ioreg = (volatile U32 *)0x20000000;

  U32 data;
  int i = 0;
  
  const U32 MAG_MASK = 0x010;
  const U32 SGN_MASK = 0x020;
  int mag3rd_cnt = 0, sgn3rd_cnt = 0;
  
  data = ioreg[0xf07c/4];
  U32 magStatus = data & MAG_MASK;
  U32 sgnStatus = data & SGN_MASK;
  
  for(i=0; i<100; ++i)
  {
    data = ioreg[0xf07c/4];
    if((data & MAG_MASK) != magStatus)
    {
      mag3rd_cnt++;
      magStatus = data & MAG_MASK;
    }
    if((data & SGN_MASK) != sgnStatus)
    {
      sgn3rd_cnt++;
      sgnStatus = data & SGN_MASK;
    }
    if(mag3rd_cnt > 10 && sgn3rd_cnt > 10)
    {
      break;
    }
  }  
  DebugOutput(" i=", i);
  DebugOutput(" sgn3rd_cnt=", sgn3rd_cnt);
  DebugOutput(" mag3rd_cnt=", mag3rd_cnt);
  if(sgn3rd_cnt > 10 && mag3rd_cnt > 10)
  {
    UartPutLine(" GSN_MAG OK\r\n");
    return TRUE;
  }
  
  UartPutLine(" GSN_MAG NG\r\n");
  return FALSE;
}
#endif

#if defined(TEST_RTC)
static U08 Rtc_Test()
{
  int i;
  volatile unsigned long t1, t2;
  *((volatile unsigned long *)(0x20014C3C)) = 0x2;
  
  t1 = *(volatile unsigned long *)(0x20014C34);
  
  // about 3 seconds
  int iter = 0;
  for(i=0; i<3000000; i++)
  {
    t2 = *(volatile unsigned long *)(0x20014C34);
    if (iter==0 && t2==(t1 + 1))
    {
      i = 0;
      iter = 1;
    }
    if (t2 > (t1 + 1))
    {
      break;
    }
  }
  DebugOutput(" i=", i);
  DebugOutput(" t1=", t1);
  DebugOutput(" t2=", t2);
  if (t1 != t2 && t1 != 0 && t2 != 0)
  {
    UartPutLine(" RTC OK\r\n");
    return TRUE;
  }

  UartPutLine(" RTC NG\r\n");
  return FALSE;
}
#endif
