#ifndef NO_ENCODER
#include <ESP32Encoder.h>
#endif

#ifdef UTF8
#define Wchars  15      // TBD, not sure this lib supports UTF8
#else
#define Wchars  16      // wdith of oled display in no. of charaters
#endif

#define Nstr  64

#define Tscroll 200    // srcolling speed in ms
#define Nwaitscr 12     // wait N*Tscroll before start srcolling

#ifndef NO_ENCODER
extern ESP32Encoder encUI;
#endif

extern volatile int clickCounterSW1, touchedL, touchedR;
extern int cancelJog;

extern volatile bool ez_check_cancel;

int volatile btnClicked( void );
int volatile btnClickedRlsd( void );
int volatile touched( void );
void clearBtn( void );
void clearBtnTouch( void );
void clearTouch( void );

void eznc_dispatch( void );
void oled_flash();

void select_from_menu( uint8_t N, char (*ptr)[Nstr], int8_t *sp, int8_t *sminp, void (*f2upd)(char *,int) =NULL );
void u8g_print( char *s0, char *s1 =(char *)"", char *s2 =(char *)"", char *s3 =(char *)"",
                int8_t sel = -1, int8_t frame = 1 );

int32_t readEncoder(int);

bool confirm( char *msg1, char *msg2 = (char *)"", int yes = 1);
bool yes_or_no( char *msg1, char *msg2, int yes = 1 );

float set_float( float X, char *s, uint8_t dd, float max=+1e5, float min = -1e5, float dx = 0.0 );
float set_float_auto_unit( float Xx, char *s, float max=+1e5, float min= -1e5, float dx=0.0 );
