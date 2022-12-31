/*/ SensorModule.c * Created: 2022-11-01 10:22:22 * Author : denan251 / marjo441 */

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

#define StartConv	0xC6	//0b11000110					//11000110
#define BreakWait	0x80	//0b10010110					//10000110
#define SelectADC0	0x60	//0b01100000					//0x21
#define SelectADC1	0x61	//0b01100001					//0x22
#define SelectADC2	0x62	//0b01100010					//0x24
#define SelectADC3	0x63	//0b01100011 1.1v voltage ref	//0x28


#define CPUFrequency 20000000							//20MHz  (F_CPU 1000000UL?)
#define TWIFrequency 50000								//Oklart?

#define TWCRDEF (1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE)

//Variables have underscores in their name, functions does not!
//Twi
uint8_t address = 0x40;
uint8_t Outdata_counter = 0;
//Reflex sensors
int Above_Tape[11];
int Reflex_Temp;
int Sum_Of_Reflex_Sensors;
uint8_t n = 0;												//Used for sensor diff data
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
int Current_Data[6];
int Out_Data[6]; // 0 = tejpsensor, 1 = wheel, 2 = wheel, 3 = kollision, 4 = sv�ng variabel, 5 = vinkel diff, 6 = autonom knapp


//Init for TIW
void TwiInit(uint8_t address)
{
cli();
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
	if (Outdata_counter > 3)
	{
		Outdata_counter=0;
	}
	switch ( status )
	{

	case TW_ST_SLA_ACK:		//vi reagerar p� v�r adress efter restart
		TWDR = Out_Data[Outdata_counter];
		TWCR = TWCRDEF;
		break;

	case TW_ST_DATA_ACK:	//vi har skickat data och vntar p ack fr att skicka ny byte?
		Outdata_counter++;
		TWDR = Out_Data[Outdata_counter];
		
		TWCR = TWCRDEF;
		break;

	case TW_ST_DATA_NACK:
		Outdata_counter = 0;
		TWCR = TWCRDEF;
		break;

	case TW_SR_SLA_ACK: // mastern har skickat v�r adress och vi skickar ack tillbaka
		TWCR = TWCRDEF;
		break;

	case TW_SR_STOP:
		TWCR = TWCRDEF;

	default:
	//Hmmmm ha kvar fr felskning ifall vi missar ngot case!!!
		TWCR = TWCRDEF;
		break;
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
void CheckReflexSensors(uint8_t in)							//(EDCBA)
{
	PORTD = in;											//Selects pins based on the in variable
	//ADMUX = SelectADC0;									//Selects ADC0 0xA0 with internal ref 1.1 voltage
	//ADCSRA = StartConv;									//Start conversion 0xC0
	//while((ADCSRA &(1<<ADSC)));							//Wait for conversion

	ADCSRA |= (1<<ADSC);								//Start conversion
	
	while(ADCSRA & (1<<ADSC));							//wait for conversion complete
	Reflex_Sensor[in] = ADCH;							//Save converted value
	if(in == 10)
	{
		PORTD = (1<<PIND4);
	}
}

//------------------------------------------------------------------------------------------------------------------
void WheelSensors()										//PA1 :: ADC1 || PA2 :: ADC2
{
	//Wheel 1
	ADMUX = SelectADC1;									//Select ADC1
	ADCSRA = StartConv;									//Start conversion
	while(ADCSRA & (1<<ADSC));
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	Wheel_Sensor[0] = ADCH;								//Save converted value :: Wheel sensor 1
	
	//Wheel 2
	ADMUX = SelectADC2;									//Select ADC2
	ADCSRA = StartConv;									//Start conversion
	while(ADCSRA & (1<<ADSC));
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	Wheel_Sensor[1] = ADCH;								//Save converted value :: Wheel sensor 2
}


//------------------------------------------------------------------------------------------------------------------
void IRsensor()											//PA3 :: ADC3
{
	ADMUX = SelectADC3;									//Select ADC3
	ADCSRA = StartConv;									//Start conversion
	//WaitForConversion(StartConv, BreakWait);			//Wait for conversion
	while(ADCSRA & (1<<ADSC));
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
double ReflexAverage(double Reflex_Sensor[])
{
	int Reflex_Temp = 0;
	for (int i=0; i<11; i++)
	{
		Reflex_Temp += Reflex_Sensor[i];
	}
	Reflex_Average = Reflex_Temp / 11;
	return Reflex_Average;
}
//------------------------------------------------------------------------------------------------------------------
uint8_t IdentifyTurn(int Above_Tape[])
{
	uint8_t Turn_left = 0;
	uint8_t Turn_right = 0;
	uint8_t tmp = 0;
	if ( (Above_Tape[0] || Above_Tape[1] || Above_Tape[2]) == 1)
	{
		if ( (Above_Tape[4] || Above_Tape[5] || Above_Tape[6]) == 1)
		{
			Turn_left = 0x80;
		}
		else
		{
			Turn_left = 0;
		}
	}
	else if ( (Above_Tape[8] || Above_Tape[9] || Above_Tape[10]) == 1)
	{
		if ( (Above_Tape[4] || Above_Tape[5] || Above_Tape[6]) == 1)
		{
			Turn_right = 0x40;
		}
		else
		{
			Turn_right = 0;
		}
	}
	else
	{
		Turn_right = 0;
		Turn_left = 0;
	}
	
	if (Turn_left == 1 && Turn_right == 1)
	{
		tmp = 1; // 11 means that we are going from a pallet station 
	} 
	else if (Turn_left == 1 && Turn_right == 0)
	{
		tmp = 2; // 3 means that we are on the last pallet station and turning into the lane
	}
	else if (Turn_left == 0 && Turn_right == 1)
	{
		tmp = 3; // 7 means that we are on the lane and a turn into a pallet station is detected
	}
	else
	{
		tmp = 0;
	}
	tmp |= Turn_left;
	tmp |= Turn_right;
	return tmp;
}
//------------------------------------------------------------------------------------------------------------------
int main(void)
{
	//INIT
	DDRA = 0x00;										//Inputs
	DDRD = 0xFF;										//Outputs
	//DDRC = 0x23;										//00100011 (0x10) - TestDataInput/TWIdata/TWIclock(out from ATmega) and TestDataOutput/TestModeSelect/TestClock(in to ATmega).
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR);			//ADLAR = 1	REFS0 = 1 0b01100000	//To only use ADCH.
	//ADCSRA = 0x86;									//ADEN = 1		//ADC Enable ADEN.
	ADMUX = 0x60;
	ADCSRA |= (1<<ADSC);								//Start one conversion because it takes the longest time
	while(ADCSRA & (1<<ADSC));							//Wait for first conversion to complete


	//Interrupt Setup
	TwiInit(address);

	while (PIND7 == 1) //Self running mode active = 1  (If switch is active :: pin PD5 :: If PD5 = 5V then Auto else if PD5 = GND then manual)
	{

		//----------------------------------------------------------------------------------------------------------

		//Get data from reflex sensors :: Updates Reflex_Sensor[]
		//stage left = 0
		//Reflex_Sensor[i] noll �r p� v�nster sida i f�rdriktning
		ADMUX &= 0x60;//0b01100000
		for (uint8_t i = 0; i<11; i++)
		{
			CheckReflexSensors(i);
		}

		//Get reflex sensor average

		Reflex_Average = ReflexAverage(Reflex_Sensor);


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
		//

		Current_Data[0] = Sensor_Diff_Data[0] - Sensor_Diff_Data[n];
		//current_diff = Sensor_Diff_Data[0] - Sensor_Diff_Data[n]
		//current_diff /= 100
		//error = current_diff - previous_diff
		//? = arcsin(error)
		//previous_diff = current_diff
		//current_data[0] = ?
		//Current_Data[5] = ?
		//----------------------------------------------------------------------------------------------------------
		//Identify if there is a turn on the tape, are different cases in function which return 3,7,11 
		//uint8_t Turn_Variable = IdentifyTurn(Above_Tape);
		//Out_Data[4] = Turn_Variable;

		//----------------------------------------------------------------------------------------------------------

		//Get data from wheel sensors
		WheelSensors();
		uint8_t hundred_mm_traveled_left = 0;
		uint8_t hundred_mm_traveled_right = 0;
		//Wheel 1
		if (Wheel_Sensor[0] != Wheel_1.Previous_Status)
		{
			if (Wheel_1.Pins>= 10)
			{
				Current_Data[1] = CalcSpeed(Wheel_1, Wheel_Sensor[0]);
				hundred_mm_traveled_left += 1;
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
				hundred_mm_traveled_right += 1;
			}
			else
			{
				Wheel_2.Pins++;
			}
		}

		if (hundred_mm_traveled_left == 2)
		{
			Current_Data[1] |= 0x80;
			hundred_mm_traveled_left = 0;
		}
		if (hundred_mm_traveled_right == 2)
		{
			Current_Data[2] |= 0x80;
			hundred_mm_traveled_right = 0;
		}


		//----------------------------------------------------------------------------------------------------------

		//Save data from the IRsensor to CurrentData.
		ADMUX &= 0x60;
		ADMUX |= 0x63;
		IRsensor();

		//If an object is closer than 20 cm to the vehicle we output a 1
		//if (IR_Sensor >= 0.6)
		if (IR_Sensor >= 20)
		{
			Current_Data[3] = 1;
		}
		else
		{
			Current_Data[3] = 0;
		}

		//----------------------------------------------------------------------------------------------------------


		/*
		//Check if all sensors are above tape.
		if (Sum_Of_Reflex_Sensors >= 11)
		{
		//Only occurs when turning 90 degrees.
		//Update OutData with 1's
			for (int i=0; i<4; i++)
			{
				Out_Data[i] = 1; // g�ra en till byte f�r detta??
			}
		}
		else
		{*/
			//Update OutData with current data. CurrentData -> OutData.
			//st�nga av inte
		for (int i=0; i<Outdata_counter; i++)
		{
			Out_Data[i] = Current_Data[i];
		}
		//}
	}

	return 0;
}


