/* SensorModule.c * Created: 2022-11-01 10:22:22 * Author : denan251 / marjo441 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <time.h>
#include <stdio.h>
#include <util/twi.h>


//PD0(14) Mux/Demux A
//PD1(15) Mux/Demux B
//PD2(16) Mux/Demux C
//PD3(17) Mux/Demux D
//PD4(18) Mux/Demux E

//PA0(40) :: Demux / Reflex sensor / Mux
//PA1(39) :: Wheel sensor 1
//PA2(38) :: Wheel sensor 2
//PA3(37) :: IR-sensor

//PC5(27) :: Test Data Input	(Jtag)
//PC4(26) :: Test Data Output	(Jtag)
//PC3(25) :: Test Mode Select	(Jtag)
//PC2(24) :: Test Clock			(Jtag)

//PC1(23) :: TWI Data
//PC0(22) :: TWI Clock

//Reset(9):: IN
//VCC(10) :: IN
//GND(11) :: IN

//GND(31) :: Demux / Reflexsensor / Mux / Hjulsensor 1 / Hjulsensor 2 / IR-sensor

#define StartConv	0xC0									//11000000
#define BreakWait	0x80									//10000000
#define SelectADC0	0xA0	//0b10100000					//0x21						//00100001
#define SelectADC1	0xA1	//0b10100001					//0x22						//00100010
#define SelectADC2	0xA2	//0b10100010					//0x24						//00100100
#define SelectADC3	0xA3	//0b10100011 1.1v voltage ref	//0x28						//00101000 va t?nkte vi h?r?


#define CPUFrequency 20000000							//20MHz  (F_CPU 1000000UL?)
#define TWIFrequency 50000								//Oklart?

#define TWCRDEF (1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE)

//Variables have underscores in their name, functions does not!
//Twi
uint8_t address = 0x40;
int o =0;
//Reflex sensors
int Above_Tape[11];
int Reflex_Temp;
int Sum_Of_Reflex_Sensors;
int n = 0;												//Used for sensor diff data
double Reflex_Sensor[11];
double Sensor_Diff_Data[11];
double Most_Left_Sensor;
double Most_Right_Sensor;
double Final_Sensor_Diff;
double Reflex_Average;

//Wheel sensors
int Wheel_Sensor[1];
typedef struct wheel
{
	int Pins;
	int Previous_Status;
	double Time;
	double Speed;
	time_t End_Time;
	time_t Start_Time;
}wheel;
wheel Wheel_1, Wheel_2;

//IR sensor
double IR_Sensor;

//Out || Out-temp || Time
double Current_Data[3];
double Out_Data[3];

//Init for TIW
void TwiInit(uint8_t address)
{
	cli();
	//TWSR = 0x00;											//Set pre-scaler  (no pre-scaling)


	//TWBR = ((CPUFrequency / TWIFrequency) - 16) / 2;	//Set bit rate TWBR = 192
	
	TWCR = (1 << TWIE) | (1 << TWEN);
	TWAR = (address << 1) | 0x01;
	TWCR = TWCRDEF;					//Enable TWI and Interrupt
	sei();
}

//  -https://www.engineersgarage.com/how-to-use-i2c-twi-two-wire-interface-in-avr-atmega32-part-36-46/

/* Interrupt handler for I2C interrupts. */
ISR(TWI_vect)
{
	uint8_t status = (TWSR & TW_NO_INFO);	// Get status code of incoming I2C interrupt.
	switch ( status )
	{

		case TW_ST_SLA_ACK:		//Vi har reagerat p bussen och skickar ack till mastern
		TWCR = TWCRDEF;
		TWDR = Out_Data[o];
		break;

		case TW_ST_DATA_ACK:	//vi har skickat data och vntar p ack fr att skicka ny byte?
		TWCR = TWCRDEF;
		o++;
		TWDR = Out_Data[o];
		break;
		
		case TW_ST_DATA_NACK:
		TWCR = TWCRDEF;
		o = 0;
		break;
		
		case TW_SR_SLA_ACK:
		TWCR = TWCRDEF;
		break;
		
		case TW_SR_STOP:
		TWCR = TWCRDEF;
		
		default:
		//Hmmmm ha kvar fr felskning ifall vi missar ngot case!!!
		TWCR = TWCRDEF;
	}
}


//------------------------------------------------------------------------------------------------------------------
void WaitForConversion(int StartCon, int BreakCon)		//Wait for conversion
{
	while(ADCSRA == StartCon)
	{
		if (ADCSRA == BreakCon)
		{
			break;
		}
	}
}

//------------------------------------------------------------------------------------------------------------------
void CheckReflexSensors(int in)							//(EDCBA)
{
	PORTD = (0<<PIND4);
	ADMUX = SelectADC0;									//Selects ADC0 0xA0 with internal ref 1.1 voltage
	PORTD = in;											//Selects pins based on the in variable
	ADCSRA = StartConv;									//Start conversion 0xC0
	while((ADCSRA &(1<<ADSC)));							//Wait for conversion
	Reflex_Sensor[in] = ADCH;							//Save converted value
	PORTD = (1<<PIND4);
}

//------------------------------------------------------------------------------------------------------------------
void WheelSensors()										//PA1 :: ADC1 || PA2 :: ADC2
{
	//Wheel 1
	ADMUX = SelectADC1;									//Select ADC1
	ADCSRA = StartConv;									//Start conversion
	while((ADCSRA &(1<<ADSC))!=0);
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	Wheel_Sensor[0] = ADCH;								//Save converted value :: Wheel sensor 1
	
	//Wheel 2
	ADMUX = SelectADC2;									//Select ADC2
	ADCSRA = StartConv;									//Start conversion
	while((ADCSRA &(1<<ADSC)));
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	Wheel_Sensor[1] = ADCH;								//Save converted value :: Wheel sensor 2
}

//------------------------------------------------------------------------------------------------------------------
void IRsensor()											//PA3 :: ADC3
{
	ADMUX = SelectADC3;									//Select ADC3
	ADCSRA = 0b11000000;//StartConv;									//Start conversion
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	while((ADCSRA &(1<<ADSC)));
	IR_Sensor = ADCH;									//Save converted value :: IRsensor
}

//------------------------------------------------------------------------------------------------------------------
double CalcSpeed(wheel Wheel, int Sensor)
{
	Wheel.End_Time = time(NULL);									//Check current time
	Wheel.Time = difftime(Wheel.End_Time, Wheel.Start_Time);		//Get diff from previous time
	Wheel.Previous_Status = Sensor;//Wheel_Sensor[0];						//Save current sensor data as previous sensor data

	Wheel.Speed = 50 / Wheel.Time;									//10 pins is about 50 mm (with space between pins included).
	Wheel.Speed = Wheel.Speed / 1000;								//Meters per Second	:: Convert to speed
	
	//Output = Wheel.Speed;
	//Current_Data[1] = Wheel.Speed;									//Save speed to output
	Wheel.Start_Time = time(NULL);									//Set start time for use when the state(pin / no pin) is changed next time
	Wheel.Pins = 0;
	
	return Wheel.Speed;

	//Maybe use clock() instead of time() ???	---- gettimeofday() :: timedifference_msec() ???
	//<time.h> -> time_t msec = time(NULL) * 1000;
	//Maybe add if-statement for when x amount of pins has been passed ???
}

//------------------------------------------------------------------------------------------------------------------
int main(void)
{
	//INIT
	DDRA = 0x00;										//Inputs
	DDRD = 0xFF;										//Outputs
	//DDRC = 0x23;										//00100011 (0x10) - TestDataInput/TWIdata/TWIclock(out from ATmega) and TestDataOutput/TestModeSelect/TestClock(in to ATmega).
	ADCSRA = 0x80;										//ADEN = 1		//ADC Enable ADEN.
	ADMUX = 0x20;										//ADLAR = 1		//To only use ADCH.
	
	//Interrupt Setup
	TwiInit(address);

	while (1) //Self running mode active = 1  (If switch is active :: pin PD5 :: If PD5 = 5V then Auto else if PD5 = GND then manual)
	{
		
		//----------------------------------------------------------------------------------------------------------
		
		//Get data from reflex sensors :: Updates Reflex_Sensor[]
		for (int i=0; i<11; i++)
		{
			CheckReflexSensors(i);
		}
		
		//Get reflex sensor average
		Reflex_Temp = 0;
		for (int i=0; i<11; i++)
		{
			Reflex_Temp += Reflex_Sensor[i];
		}
		Reflex_Average = Reflex_Temp / 11;
		
		//Determine which sensor is above tape
		for (int i=0; i<11; i++)
		{
			if (Reflex_Sensor[i] >= Reflex_Average)
			{
				Above_Tape[i] = 1;						//Sensors which are not zero.
				Sum_Of_Reflex_Sensors += 1;
			}
			else
			{
				Above_Tape[i] = 0;						//Sensors which are zero.
			}
		}
		
		//Checks the difference between the most Left(active) sensor and the most Right(active) sensor ------------
		//Reflex_Sensor[i](Type: double) || Above_Tape[i](Type: int) || Sensor_Diff_Data[n](Type: double)
		n = 0;
		for (int i=0; i<11; i++)
		{
			if (Above_Tape[i] == 1)
			{
				Sensor_Diff_Data[n] = Reflex_Sensor[i];
				n++;
			}
		}

		//Diff :: Most_Left/Right_Sensor are of type double :: Final_Sensor_Diff is of type double
		Current_Data[0] = Sensor_Diff_Data[0] - Sensor_Diff_Data[n];
		
		//----------------------------------------------------------------------------------------------------------
		
		//Get data from wheel sensors
		WheelSensors();

		//Wheel 1
		if (Wheel_Sensor[0] != Wheel_1.Previous_Status)
		{
			if (Wheel_1.Pins>= 10)
			{
				Current_Data[1] = CalcSpeed(Wheel_1, Wheel_Sensor[0]);
			}
			else
			{
				Wheel_1.Pins++;
			}
		}

		//Wheel 2
		if (Wheel_Sensor[1] != Wheel_2.Previous_Status)
		{
			if (Wheel_2.Pins>= 10)
			{
				Current_Data[2] = CalcSpeed(Wheel_2, Wheel_Sensor[1]);
			}
			else
			{
				Wheel_2.Pins++;
			}
		}

		//----------------------------------------------------------------------------------------------------------
		
		//Save data from the IRsensor to CurrentData.
		IRsensor();
		
		//If an object is closer than 20 cm to the vehicle we output a 1
		//if (IR_Sensor >= 0.6)
		if (IR_Sensor >= 100)
		{
			Current_Data[3] = 1;
		}
		else
		{
			Current_Data[3] = 0;
		}

		//----------------------------------------------------------------------------------------------------------
		
		//Check if all sensors are above tape.
		if (Sum_Of_Reflex_Sensors >= 11)
		{
			//Only occurs when turning 90 degrees.
			//Update OutData with 1's
			for (int i=0; i<4; i++)
			{
				Out_Data[i] = 1;
			}
		}
		else
		{
			//Update OutData with current data. CurrentData -> OutData.
			for (int i=0; i<4; i++)
			{
				Out_Data[i] = Current_Data[i];
			}
		}
	}
	
	return 0;
}
