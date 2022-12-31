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
int Pins_Wheel_1;
int Pins_Wheel_2;
int Wheel_Sensor[1];
int Wheel_1_Previous_Status;
int Wheel_2_Previous_Status;
double Wheel_1_Time;
double Wheel_2_Time;
double Wheel_1_Pin_Length;
double Wheel_2_Pin_Length;
double Wheel_1_Traveled;
double Wheel_2_Traveled;
double Wheel_1_Speed;
double Wheel_2_Speed;

//IR sensor
double IR_Sensor;

//Out || Out-temp || Time
double Current_Data[3];
double Out_Data[3];
time_t End_Time_Wheel_1, End_Time_Wheel_2, Start_Time_Wheel_1, Start_Time_Wheel_2;

//Init for TWI
void TWIINIT()
{
	TWSR = 0;											//Set pre-scaler  (no pre-scaling)
	TWBR = ((CPUFrequency / TWIFrequency) - 16) / 2;	//Set bit rate
	TWCR = (1 << TWIE) | (1 << TWEN);					//Enable TWI and Interrupt
}


//  -https://www.engineersgarage.com/how-to-use-i2c-twi-two-wire-interface-in-avr-atmega32-part-36-46/

//When interrupt is triggered
ISR(TWI_vect)											//Vector 27 (TWI)
{
	TWCR = 0x44;										//01000100 :: TWEN = 1 || TWEA = 1 :: Send ACK
	for (int i=0; i<4; i++)
	{
		TWDR = Out_Data[i];
		while(TWINT == 0)								//Wait for TWINT to get set to 1 (high)
		{}
		TWCR = 0x85;									//Set TWINT = 1 || TWEN = 1 || TWIE = 1 || 0b10000101
	}	
	reti();
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
    while (1) //Self running mode active = 1  (If switch is active :: pin PD5 :: If PD5 = 5V then Auto else if PD5 = GND then manual)
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

		//Diff :: Most_Left/Right_Sensor are of type double :: Final_Sensor_Diff is of type double
		Current_Data[0] = Sensor_Diff_Data[0] - Sensor_Diff_Data[n];
		
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
			
			
		//Maybe use clock() instead of time() ???	---- gettimeofday() :: timedifference_msec() ???
		//<time.h> -> time_t msec = time(NULL) * 1000;
		//Maybe add if-statement for when x amount of pins has been passed ???
	
		//Wheel 1
		if (Wheel_Sensor[0] != Wheel_1_Previous_Status)
		{
			//Wait 10 pins before re-calculating the speed
			if (Pins_Wheel_1 >= 10)
			{
				End_Time_Wheel_1 = time(NULL);										//Check current time
				Wheel_1_Time = difftime(End_Time_Wheel_1, Start_Time_Wheel_1);		//Get diff from previous time
				Wheel_1_Previous_Status = Wheel_Sensor[0];							//Save current sensor data as previous sensor data
			
				//Meters per Second													/Convert to speed :: meters per second
				//Wheel_1_Speed = Wheel_1_Traveled / Wheel_Time;
				Wheel_1_Traveled = Wheel_1_Pin_Length / (Wheel_1_Time / 10);
			
				Current_Data[1] = Wheel_1_Speed;									//Save speed to output
				Start_Time_Wheel_1 = time(NULL);									//Set start time for use when the state(pin / no pin) is changed next time
				Pins_Wheel_1 = 0;
			}
			else
			{
				Pins_Wheel_1++;
			}
		} 
		else
		{
			Wheel_1_Previous_Status = Wheel_Sensor[0];
		}
		
		//Wheel 2
		if (Wheel_Sensor[1] != Wheel_2_Previous_Status)
		{
			//Wait 10 pins before re-calculating the speed
			if (Pins_Wheel_2 >= 10)
			{
				End_Time_Wheel_2 = time(NULL);
				Wheel_2_Time = difftime(End_Time_Wheel_2, Start_Time_Wheel_2);
				Wheel_2_Previous_Status = Wheel_Sensor[1];
				
				//Meters per Second
				//Wheel_2_Speed = Wheel_1_Traveled / Wheel_Time;
				Wheel_2_Traveled = Wheel_2_Pin_Length / (Wheel_2_Time / 10);
				
				Current_Data[2] = Wheel_2_Speed;
				Start_Time_Wheel_2 = time(NULL);
				Pins_Wheel_2 = 0;
			}
			else
			{
				Pins_Wheel_2++;
			}
		}
		else
		{
			Wheel_2_Previous_Status = Wheel_Sensor[1];
		}

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
