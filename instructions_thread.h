////////////////////////////////////////////////////////////////////////////////
//	File Name					: LED_thread.h
//	Description				: Header file for LED thread
//	Author						: Harsh Aurora
//	Date							: Oct 28, 2016
////////////////////////////////////////////////////////////////////////////////



//		Exported Functios		//
void start_instructions_thread(void *args);

#define	Col1		GPIO_PIN_10
#define	Col2		GPIO_PIN_11
#define	Col3		GPIO_PIN_12
#define Row1		GPIO_PIN_6
#define Row2		GPIO_PIN_7
#define Row3		GPIO_PIN_8
#define Row4		GPIO_PIN_9

void Keypad_Columninput(void);
void Keypad_Rowinput(void);
int Keypad_Read(void);

extern int leftFlag;
extern int rightFlag;
extern int middleFlag;

void instructions_thread_periph_init(void);

