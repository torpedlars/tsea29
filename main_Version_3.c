/* SensorModule.c * Created: 2022-11-01 10:22:22 * Author : denan251 / marjo441 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <time.h>

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

#define StartConv 0xC0									//11000000
#define BreakWait 0x80									//10000000
#define SelectADC0 0x21									//00100001
#define SelectADC1 0x22									//00100010
#define SelectADC2 0x24									//00100100
#define SelectADC3 0x28									//00101000

#define CPUFrequency 20000000							//20MHz  (F_CPU 1000000UL?)
#define TWIFrequency 50000								//Oklart?

//Variables have underscores in their name, functions does not!
//Reflex sensors
int Above_Tape[10];
int Reflex_Temp;
int Sum_Of_Reflex_Sensors;
int n = 0;												//Used for sensor diff data
double Reflex_Sensor[10];
double Sensor_Diff_Data[10];
double Most_Left_Sensor;
double Most_Right_Sensor;
double Final_Sensor_Diff;
double Reflex_Average;

//Wheel sensors
int Wheel_Sensor[1];
double Wheel_Time;
double Wheel_1_Traveled;
double Wheel_2_Traveled;
double Wheel_1_Speed;
double Wheel_2_Speed;

//IR sensor
double IR_Sensor;

//Out || Out-temp || Time
double Current_Data[3];
double Out_Data[3];
time_t Previous_t, New_t;

void TWIINIT()
{
	//Set pre-scaler  (no pre-scaling)
	TWSR = 0;
	
	//Set bit rate
	TWBR = ((CPUFrequency / TWIFrequency) - 16) / 2;
	
	//Enable TWI and Interrupt
	TWCR = (1 << TWIE) | (1 << TWEN);
}

//When interrupt is triggered
ISR(TWI_vect)											//Vector 27 (TWI)
{
	for (int i=0; i<4; i++)
	{
		TWDR = Out_Data[i];
		while(TWINT == 0)								//Wait for TWINT to get set to 1
		{}
		TWCR = 0x80;									//Set TWINT = 0 || 0b10000000
	}	
}

int main(void)
{
														//INIT
	DDRA = 0x00;										//Inputs
	DDRD = 0xFF;										//Outputs
	DDRC = 0x23;										//00100011 (0x10) - TestDataInput/TWIdata/TWIclock(out from ATmega) and TestDataOutput/TestModeSelect/TestClock(in to ATmega).
	ADCSRA = 0x80;										//ADEN = 1		//ADC Enable ADEN.
	ADMUX = 0x20;										//ADLAR = 1		//To only use ADCH.
	
	//Interrupt Setup
	TWIINIT();
	sei();

//------------------------------------------------------------------------------------------------------------------
	void WaitForConversion(int StartCon, int BreakCon)	//Wait for conversion
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
	void CheckReflexSensors(int in)						//(EDCBA)
	{
		ADMUX = SelectADC0;								//Selects ADC0
		PORTD = in;										//Selects pins based on the in variable
		ADCSRA = StartConv;								//Start conversion
		WaitForConversion(StartConv, BreakWait);		//Wait for conversion
		Reflex_Sensor[in] = ADCH;						//Save converted value
	}
	
//------------------------------------------------------------------------------------------------------------------	
	void WheelSensors()									//PA1 :: ADC1 || PA2 :: ADC2
	{
		//Wheel 1
		ADMUX = SelectADC1;								//Select ADC1
		ADCSRA = StartConv;								//Start conversion
		WaitForConversion(StartConv, BreakWait);		//Wait for conversion
		Wheel_Sensor[0] = ADCH;							//Save converted value :: Wheel sensor 1
		
		//Wheel 2 
		ADMUX = SelectADC2;								//Select ADC2
		ADCSRA = StartConv;								//Start conversion
		WaitForConversion(StartConv, BreakWait);		//Wait for conversion
		Wheel_Sensor[1] = ADCH;							//Save converted value :: Wheel sensor 2	
	}	
	
//------------------------------------------------------------------------------------------------------------------	
	void IRsensor()										//PA3 :: ADC3
	{
		ADMUX = SelectADC3;								//Select ADC3
		ADCSRA = StartConv;								//Start conversion
		WaitForConversion(StartConv, BreakWait);		//Wait for conversion
		IR_Sensor = ADCH;								//Save converted value :: IRsensor
	}
	
//------------------------------------------------------------------------------------------------------------------
    while (1) //Self running mode active = 1 
    {
		
		//----------------------------------------------------------------------------------------------------------
		
		//Get data from reflex sensors :: Updates Reflex_Sensor[]
		for (int i=0; i<11; i++)
		{
			CheckReflexSensors(i);
		}
		
		//Get reflex sensor average
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
		
		//Checks the difference between the most Left(active) sensor and the most Right(active) sensor --------------------------------
		//Reflex_Sensor[i](Type: double) || Above_Tape[i](Type: int) || Sensor_Diff_Data[n](Type: double)
		for (int i=0; i<11; i++)
		{
			if (Above_Tape[i] == 1)
			{
				Sensor_Diff_Data[n] = Reflex_Sensor[i];
				n++;
			} 
		}
		
		//Most_Left/Right_Sensor are of type double
		Most_Left_Sensor = Sensor_Diff_Data[0];
		Most_Right_Sensor = Sensor_Diff_Data[n];
		
		//Final_Sensor_Diff is of type double
		Final_Sensor_Diff = Most_Left_Sensor - Most_Right_Sensor;
		Current_Data[0] = Final_Sensor_Diff;
		
		//-----------------------------------------------------------------------------------------------------------------------------
		
		//Check if all sensors are above tape.
		/*if (Sum_Of_Reflex_Sensors >= 11)
		{
			//Only occurs when turning 90 degrees.
		} 
		else
		{
		}*/
		
		//----------------------------------------------------------------------------------------------------------
		
		//Get data from wheel sensors
		WheelSensors();
		
		//Convert Wheel_Sensor input to distance in meter
		//------------------------------------Check possibilities with interrupt, amount of pins passed per 10 ms---
		
		//Meters
		Wheel_1_Traveled = Wheel_Sensor[0];
		Wheel_2_Traveled = Wheel_Sensor[1];
		
		//Seconds :: time_t Previous_t, New_t;
		Wheel_Time = difftime(New_t, Previous_t);
		
		//Meters per Second
		Wheel_1_Speed = Wheel_1_Traveled / Wheel_Time;
		Wheel_2_Speed = Wheel_1_Traveled / Wheel_Time;

		//Add to output array
		Current_Data[1] = Wheel_1_Speed;
		Current_Data[2] = Wheel_2_Speed;
		
		//----------------------------------------------------------------------------------------------------------
		
		//Save data from the IRsensor to CurrentData.
		IRsensor();
		
		//If an object is closer than 20 cm to the vehicle we output a 1
		if (IR_Sensor >= 0.6)
		{
			Current_Data[3] = 1;
		}
		else
		{
			Current_Data[3] = 0;
		}
		
		//----------------------------------------------------------------------------------------------------------
		
		//Update OutData with current data. CurrentData -> OutData.
		for (int i=0; i<4; i++)
		{
			Out_Data[i] = Current_Data[i];
		}

		//----------------------------------------------------------------------------------------------------------
		
    }
	
return 0;
}
