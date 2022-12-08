#ifndef NO_ENCODER
#include <ESP32Encoder.h>
#endif

#ifdef UTF8
#define Wchars  15      // try two more chars
#else
#define Wchars  13      // wdith of oled display in no. of charaters (1st line 14 touches edge)
#endif

#define Nstr  64

#define Tscroll 200    // srcolling speed in ms
#define Nwaitscr 12     // wait N*Tscroll before start srcolling

#ifndef NO_ENCODER
extern ESP32Encoder encUI;
#endif

extern volatile int clickCounterSW1, clickCounterSWL, clickCounterSWR;
extern int cancelJog;

int volatile btnClicked( void );
int volatile btnClickedRlsd( void );

void clearBtn( void );

void eznc_dispatch( void );

void oled_flash();

void select_from_menu( uint8_t N, char (*ptr)[Nstr], int8_t *sp, int8_t *sminp, void (*f2upd)(char *,int) =NULL );

void u8g_print( char *s0, char *s1 =(char *)"", char *s2 =(char *)"", char *s3 =(char *)"",
                int8_t sel = -1, int8_t frame = 1 );

int32_t readEncoder(int);