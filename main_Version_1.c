/*
 * SensorModule.c
 *
 * Created: 2022-11-01 10:22:22
 * Author : denan251
 */ 

#include <avr/io.h>

//PD0(14) Mux/Demux A
//PD1(15) Mux/Demux B
//PD2(16) Mux/Demux C
//PD3(17) Mux/Demux D
//PD4(18) Mux/Demux E

//PA0(40) :: Demux / Reflexsensor / Mux
//PA1(39) :: Hjulsensor 1
//PA2(38) :: Hjulsensor 2
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


int main(void)
{
	
	DDRA = 0x00;		//Inputs
	DDRD = 0xFF;		//Outputs
	DDRC = 0x10;		//00010000 (0x10) - TestDataInput(ut fr�n ATmega) och TestDataOutput/TestModeSelect/TestClock(in till ATmega).
	ADCSRA = 0x80;		//ADEN = 1;		//ADC Enable n�r ADEN �r h�g(1).
	ADLAR = 1;		//ADLAR = 1 f�r att bara anv�nda ADHC.					 -------PROBLEM_VARIABEL!!!!-------
	
	int CurrentData[13];
	int OutData[13];
	int Reflex_Sensor[10];
	int Hjul_Sensor[1];
	int IR_Sensor;
	
//------------------------------------------------------------------------------------------------------------------	
	void CheckReflexsensor(int in)
	{
		
		switch (in)		//(EDCBA)
		{
		case 0 :		//(00000)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADCSRA == 0x40)			//V�ntar p� omvandling
			{
				if (ADCSRA == 0x00)
				{
					break;
				}
			}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
			
		case 1 :		//(00001)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
			
		case 2 :		//(00010)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 3 :		//(00011)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 4 :		//(00100)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 5 :		//(00101)
		 	PORTD = in;
		 	ADCSRA = 0x40;					//Startar omvandling
		 	while(ADSC == 0x01)				//V�ntar p� omvandling
		 	{}
		 	Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde	
		break;
		
		case 6 :		//(00110)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 7 :		//(00111)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 8 :		//(01000)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 9 :		//(01001)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		
		case 10 :		//(01010)
			PORTD = in;
			ADCSRA = 0x40;					//Startar omvandling
			while(ADSC == 0x01)				//V�ntar p� omvandling
			{}
			Reflex_Sensor[in] = ADCH;		//Spara omvandlat v�rde
		break;
		}

	}
//------------------------------------------------------------------------------------------------------------------	
	void Hjulsensor()						//PA1 :: ADC1 || PA2 :: ADC2
	{
		ADCSRA = 0x40;						//Startar omvandling
		while(ADSC == 0x01)					//V�ntar p� omvandling
		{
			if (ADSC == 0x00)
			{
				break;
			}
		}
		Hjul_Sensor[0] = ADCL1;				//Spara omvandlat v�rde :: Hjulsensor 1
		Hjul_Sensor[1] = ADCH2;				//Spara omvandlat v�rde :: Hjulsensor 2
	}	
//------------------------------------------------------------------------------------------------------------------	
	void IRsensor()							//PA3 :: ADC3
	{
		PORTA = 0x08;
		ADCSRA = 0x40;						//Startar omvandling
		while(ADSC == 0x01)					//V�ntar p� omvandling
		{
			if (ADSC == 0x00)
			{
				break;
			}
		}
		IR_Sensor = ADCH3;					//Spara omvandlat v�rde :: IRsensor
	}
//------------------------------------------------------------------------------------------------------------------
	
    while (1) //N�r sj�lvk�randel�ge �r aktivt 
    {
		
		//Spara data fr�n Reflexsensor till CurrentData.
		
		for (int i=0; i<11; i++)
		{
			CheckReflexsensor(i);
		}
		CurrentData[0-10] = Reflex_Sensor[0-10];
		
		//Spara data fr�n Hjulsensor 1 och 2 till CurrentData.
		Hjulsensor();
		CurrentData[11-12] = Hjul_Sensor[0-1];
		 	
		//Spara data fr�n IRsensor till CurrentData.
		IRsensor();
		CurrentData[13] = IR_Sensor;
		
		//Uppdatera utg�ende paket med senaste datan. Fr�n CurrentData till OutData.
		for (int i=0; i<14; i++)
		{
			OutData[i] = CurrentData[i];
		}

		//Skicka data p� TWI-bussen
		for (int i=0; i<14; i++)
		{
			//PC1(23)
			OutData[i];
		}

    }
	
return 0;
}

