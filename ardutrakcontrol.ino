#include <Wire.h>
#include <MI0283QT2.h>
#include <Adafruit_ADS1015.h>
#include <ChibiOS_AVR.h>
#include <EEPROMEx.h>
#include <OneWire.h>

// Data wire is plugged into pin 2 on the Arduino
int8_t tempPin = 6;
int8_t tempWarn = 100; //temperatur bei der dann die Farbe auf ROT umgestellt wird!!!
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ds(tempPin);

MI0283QT2 lcd;

int16_t STEERSPEED = 300; //millisec Wie lange soll er die Hydraulik einschalten
int8_t STEERWAITFACTOR = 2; // faktor wie lange hydraulik nach dem lenken warten soll steerspeed x steerwaitfaktor
int8_t STEERACTIONTOLERANCE = 1; // 0 rechnen in welchem Bereich soll er abschalten
int16_t SENSORREADSPEED = 300; // millisec wenn er die sensoren überprüft
int16_t DRAWSPEED = 200; // millisec wie oft soll er das display zeichnen
int16_t LCDRESETTIME = 120; //Sekunden wird die grafik geresetet, wegen artefakte...
int16_t DRAWSTEERINICATOROFFSET = 95; //damit kann ich den lenkungsanzeige nach unten oder oben verschieben

int16_t frontMinAddress; // eeprom adresse für frontmin-wert
int16_t frontMaxAddress; // eeprom adresse für frontmax-wert
int16_t backMinAddress; // eeprom adresse für backmin-wert
int16_t backMaxAddress; // eeprom adresse für backmax-wert

unsigned long BTN_PRESSTIME = 10000; //millisec wie lange btn gedrückt werden muss um in kallibrier-modus zu gelangen
unsigned long btnStartTime = 0;
unsigned long btnEndTime = 0;
// volatile bool btnPressed = false;
volatile bool btnControlPressed = false;
volatile bool btnControlPressedFirst = false;
// volatile bool controlMode = false;
volatile bool autoSteer = false;
// volatile uint8_t btnCount = 0;


uint16_t autoSteerColor = COLOR_WHITE; //Farbe des Textes wenn er automatisch steuert

/*
Drivemodes:
0 = ControlMode = zum kalibrieren
1 = normal
2 = Allrad
3 = Hundsgang
4 = Fehlercode 1 - Problem beim Drivemode-reading, beide Eingänge beim schalter sind auf 1 oder auf 0

*/
volatile int8_t driveMode = 1;

int16_t sensorValFront = 0;
int16_t sensorValBack = 0;
int16_t frontMin = 0;
int16_t frontMax = 0;
int16_t frontRange = 0;
int16_t frontZero = 0;
float frontIndicatorStep = 0.0;
int16_t backMin = 0;
int16_t backMax = 0;
int16_t backZero = 0;
int16_t backRange = 0;
float backIndicatorStep = 0.0;

static WORKING_AREA(waAdsReadThread, 40);
static WORKING_AREA(waBtnCtrlPressedThread, 8);
static WORKING_AREA(waLcdThread, 50);
static WORKING_AREA(waSteerThread, 30);

void setup() {

	DDRC &= ~((1 << 0) | (1 << 1)); //damit setze ich bit 0 in register C auf 0 = INPUT
	DDRD &= ~(1 << 2); //pin D2 auf INPUT Kallibierbutton

	Serial.begin(57600);

	attachInterrupt(1, btnSetAction, FALLING); //interrupt für btn lenkungssteuerung

	EEPROM.setMaxAllowedWrites(30);
	EEPROM.setMemPool(0, EEPROMSizeATmega328);

	frontMinAddress = EEPROM.getAddress(sizeof(int));
	frontMaxAddress = EEPROM.getAddress(sizeof(int));
	backMinAddress = EEPROM.getAddress(sizeof(int));
	backMaxAddress = EEPROM.getAddress(sizeof(int));

	readMinMax(); //liest gespeicherte Werte aus EEPROM

	chBegin(mainThread);
}

// ###########################################################################################################
// ###########################################################################################################
// ###########################################################################################################
// ###########################################################################################################

void mainThread() {
	chThdCreateStatic(waAdsReadThread, sizeof(waAdsReadThread),
		NORMALPRIO + 2, (tfunc_t) AdsReadThread, 0);

	chThdCreateStatic(waBtnCtrlPressedThread, sizeof(waBtnCtrlPressedThread),
		NORMALPRIO, (tfunc_t) btnCtrlPressedThread, 0);

	chThdCreateStatic(waLcdThread, sizeof(waLcdThread),
		NORMALPRIO, (tfunc_t) lcdThread, 0);

	chThdCreateStatic(waSteerThread, sizeof(waSteerThread),
		NORMALPRIO + 1, (tfunc_t) steerThread, 0);

	// while (true)
	//   loop();
}

// ###########################################################################################################
// ###########################################################################################################

void AdsReadThread() {
	int8_t adsArraySize = 3;
	int16_t adsArrayFront[adsArraySize];
	int16_t adsArrayBack[adsArraySize];

	Adafruit_ADS1015 ads1015;
	ads1015.begin();
	float filterVal = 0.7;

	while (1) {
		// Serial.println("1 adsthread");
		checkDriveMode(); //schaut welcher drivemode eingeschalten ist
		int valFront = (ads1015.readADC_Differential_0_1() >> 2);
		int valBack = (ads1015.readADC_Differential_2_3() >> 2);
		// Serial.print(sensorValFront);
		// Serial.print(" | ");
		// Serial.println(sensorValBack);

		//Filter für Sensorvals, umso näher bei 1 umso länger braucht er um auf dem tatsächlichen wert zu kommen
		sensorValFront = (int)(sensorValFront * filterVal + valFront * (1 - filterVal));
		sensorValBack = (int)(sensorValBack * filterVal + valBack * (1 - filterVal));

		// Serial.print(sensorValFront);
		// Serial.print(" | ");
		// Serial.println(sensorValBack);
		chThdSleepMilliseconds(SENSORREADSPEED);
	}
}

// ###########################################################################################################
// ###########################################################################################################

void btnCtrlPressedThread() {
	while (1) {
		// Serial.println("2 btnthread");

		btnControlPressed = (bool)(PIND & (1 << 2));; //D2 Pin als calibrate-pin
		// Serial.println(btnControlPressed);

		if (!btnControlPressed && !btnControlPressedFirst) {
			btnStartTime = millis();
			btnControlPressedFirst = true;
		} else if (btnControlPressed) {
			btnControlPressedFirst = false;
		}

		if (!btnControlPressed && btnControlPressedFirst) {
			// Serial.println(millis() - btnStartTime);

			if ((millis() - btnStartTime) >= BTN_PRESSTIME && driveMode == 1) { //&& controlMode == false 
				// controlMode = true;
				driveMode = 0;
				btnStartTime = millis();
				frontMin = 2000;
				frontMax = 0;
				backMin = 2000;
				backMax = 0;
				// Serial.println("---------- controlMode enabled ---------");
			} else if ((millis() - btnStartTime) >= BTN_PRESSTIME && driveMode == 0) { //&& controlMode == true
				// controlMode = false;
				driveMode = 1;
				btnStartTime = millis();

				if (frontRange > 100 && backRange > 100) { //nur speichern wenn die Range größer 100 ist, weil sollte sich alleine aktivieren, dann steht alles am selben fleck und range ist sehr klein
					saveMinMax();
					readMinMax();
				}

				// drawSetup(); // löscht die Anzeige
				// Serial.println("---------- controlMode disabled ---------");
			}

		}

		if (driveMode == 0) {
			Serial.println("cmode in progress...");
			calibrateMinMax(sensorValFront, sensorValBack); //einstellen der Front und Back MinMax
			// Serial.print(frontMin);
			// Serial.print(",");
			// Serial.print(frontMax);
			// Serial.print("|");
			// Serial.print(backMin);
			// Serial.print(",");
			// Serial.println(backMax);
		}
		chThdSleepMilliseconds(1000);
	}
}

// ###########################################################################################################
// ###########################################################################################################

void lcdThread() {
	// Serial.println("lcd1");
	lcd.init(2); //spi-clk = Fcpu/2

	lcd.led(70); //helligkeit vom lcd
	drawSetup();

	int8_t lcdResetCount = 0;
	int8_t lcdResetCountSec = 0;

	int lastXFront = 0;
	int lastXBack = 0;


	while (1) {
		// Serial.println("3 ldcThread");
		// inidcator damit man sieht ob button gedrückt wurde
		if (!btnControlPressed) { // weil ctrl-button normalerweise auf 1 ist (pullup)
			lcd.fillRect(0, 0, 5, 5, COLOR_WHITE);
		} else {
			lcd.fillRect(0, 0, 5, 5, COLOR_BLACK);
		}

		// reset des LCD, damit keine Artefakte sichtbar
		lcdResetCount++;

		if (lcdResetCount >= (1000 / DRAWSPEED)) {
			lcdResetCountSec++;
			lcdResetCount = 0;
			if (lcdResetCountSec >= LCDRESETTIME) {
				drawSetup();
				lcdResetCountSec = 0;
			}
		}

		//wenn autosteer aktiv, dann ändere die Farbe der DrivemodeAnzeige
		if (autoSteer) {
			autoSteerColor = COLOR_YELLOW;
		} else {
			autoSteerColor = COLOR_WHITE;
		}

		//drivemode anzeige
		switch (driveMode) {
			case 0:
				lcd.drawText(60, 0, "KALIBRIEREN", 2, COLOR_RED, COLOR_BLACK);
				break;
			case 1:
				lcd.drawText(60, 0, "   FRONT   ", 2, autoSteerColor, COLOR_BLACK);
				break;
			case 2:
				lcd.drawText(60, 0, "  ALLRAD   ", 2, autoSteerColor, COLOR_BLACK);
				break;
			case 3:
				lcd.drawText(60, 0, " HUNDSGANG ", 2, autoSteerColor, COLOR_BLACK);
				break;
			case 4:
				lcd.drawText(60, 0, "  Fehler 1 ", 2, COLOR_RED, COLOR_BLACK);
				break;
		}

		if (driveMode == 0) {
			//aktuellen sensorValFront anzeigen
			int clsX = lcd.drawText(140, DRAWSTEERINICATOROFFSET, sensorValFront, 2, COLOR_GREEN, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET, "   ", 2, COLOR_GREEN, COLOR_BLACK);
			// aktuellen frontMin anzeigen
			clsX = lcd.drawText(0, DRAWSTEERINICATOROFFSET, frontMin, 1, COLOR_YELLOW, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET, "   ", 1, COLOR_YELLOW, COLOR_BLACK);
			// aktuellen frontMax anzeigen
			clsX = lcd.drawText(290, DRAWSTEERINICATOROFFSET, frontMax, 1, COLOR_YELLOW, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET, "   ", 1, COLOR_YELLOW, COLOR_BLACK);
			//aktellen sensorValBack anzeigen
			clsX = lcd.drawText(140, DRAWSTEERINICATOROFFSET + 120, sensorValBack, 2, COLOR_GREEN, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET + 120, "   ", 2, COLOR_GREEN, COLOR_BLACK);
			//aktellen backMin anzeigen
			clsX = lcd.drawText(0, DRAWSTEERINICATOROFFSET + 120, backMin, 1, COLOR_YELLOW, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET + 120, "   ", 1, COLOR_YELLOW, COLOR_BLACK);
			//aktellen backMax anzeigen
			clsX = lcd.drawText(290, DRAWSTEERINICATOROFFSET + 120, backMax, 1, COLOR_YELLOW, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET + 120, "   ", 1, COLOR_YELLOW, COLOR_BLACK);
		} else {
			int clsX = lcd.drawText(140, DRAWSTEERINICATOROFFSET, sensorValFront - (frontZero + frontMin), 2, COLOR_WHITE, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET, "   ", 2, COLOR_WHITE, COLOR_BLACK);

			clsX = lcd.drawText(140, DRAWSTEERINICATOROFFSET + 120, sensorValBack - (backZero + backMin), 2, COLOR_WHITE, COLOR_BLACK);
			lcd.drawText(clsX, DRAWSTEERINICATOROFFSET + 120, "   ", 2, COLOR_WHITE, COLOR_BLACK);

			lastXFront = drawSteerIndicatorFront(sensorValFront, lastXFront); //löscht die alte front-indicator-line
			lastXBack = drawSteerIndicatorBack(sensorValBack, lastXBack); // löscht die alte back-indicator-line


		}

		//Temp Anzeige
		int8_t temp = getTemp();
		if (temp >= tempWarn) {
			lcd.drawText(0, 40, temp, 2, COLOR_RED, COLOR_BLACK);
		} else if (temp >= (tempWarn - 10)) { //bei 10°C unter der Warntemp wird auf gelb umgestellt
			lcd.drawText(0, 40, temp, 2, COLOR_YELLOW, COLOR_BLACK);
		} else {
			lcd.drawText(0, 40, temp, 2, COLOR_GREEN, COLOR_BLACK); //alles ok, temp im grünen bereich
		}

		// Serial.print(sensorValFront);
		// Serial.print(" | ");
		// Serial.println(sensorValBack);

		chThdSleepMilliseconds(DRAWSPEED);

	}
}

// ###########################################################################################################
// ###########################################################################################################

void steerThread() {

	DDRC |= (1 << 2); //pin A2 auf Output = bit auf 1
	DDRC |= (1 << 3); //pin A3 auf Output = bit auf 1
	DDRD |= (1 << 4); //pin D4 auf Output = bit auf 1
	DDRD |= (1 << 5); //pin D5 auf Output = bit auf 1

	PORTC &= ~(1 << 2); //pin A2 auf 0V
	PORTC &= ~(1 << 3); //pin A3 auf 0V
	PORTD &= ~(1 << 4); //pin D4 auf 0V
	PORTD &= ~(1 << 5); //pin D5 auf 0V


	while (1) {
		// Serial.println("4 steerthread");

		if (autoSteer) {
			switch (driveMode) {
				case 0: // kalibiermodus
					steerAllStop();
					autoSteer = false;
					break;
				case 1: //normal / front lenkung
					autoSteerFront();
					chThdSleepMilliseconds(STEERSPEED);
					steerAllStop();
					break;
				case 2: //allradlenkung
					autoSteerAllrad();
					chThdSleepMilliseconds(STEERSPEED);
					steerAllStop();
					break;
				case 3: //hundsganglenkung
					autoSteerHundsgang();
					chThdSleepMilliseconds(STEERSPEED);
					steerAllStop();
					break;
			}
		} else {
			steerFrontStop();
			steerBackStop();
			autoSteer = false;
		}
		chThdSleepMilliseconds(STEERSPEED * STEERWAITFACTOR);
	}
}


// ###########################################################################################################
// ###########################################################################################################
// ###########################################################################################################
// ###########################################################################################################


void loop() {
	// Serial.println((bool)(PIND & (1 << 2)));
	// delay(1000);
}

void btnSetAction() {
	// Serial.println("released");
	if (millis() > 10000) {
		autoSteer = true;
	}
}

void readMinMax() {
	frontMin = EEPROM.readInt(frontMinAddress);
	frontMax = EEPROM.readInt(frontMaxAddress);
	backMin = EEPROM.readInt(backMinAddress);
	backMax = EEPROM.readInt(backMaxAddress);

	frontRange = frontMax - frontMin;
	frontZero = frontRange / 2;
	frontIndicatorStep = 320.0 / frontRange;

	// Serial.print(frontRange);
	// Serial.print(" | ");
	// Serial.print(frontZero);
	// Serial.print(" | ");
	// Serial.println(frontIndicatorStep);

	backRange = backMax - backMin;
	backZero = backRange / 2;
	backIndicatorStep = 320.0 / backRange;

	// Serial.print(backRange);
	// Serial.print(" | ");
	// Serial.print(backZero);
	// Serial.print(" | ");
	// Serial.println(backIndicatorStep);
}

// 1 = normal, 2 = Allrad, 3 = Hundsgang, 0 = ControlMode, 4 = Error beim auslesen des Drivemode
// read Register through PINx, write Register through PORTx (x= B, C, D)
void checkDriveMode() {
	if (driveMode != 0) {
		bool allr = (bool)(PINC & (1 << 0)); //damit lese ich status von A0 Pin
		bool hund = (bool)(PINC & (1 << 1)); // damit lese ich status von A1 Pin
		if (!allr && !hund) {
			driveMode = 1;
		} else if (allr && !hund) {
			driveMode = 2;
		} else if (!allr && hund) {
			driveMode = 3;
		} else {
			driveMode = 4;
		}
	}
	// Serial.print("dm: ");
	// Serial.println(driveMode);

}

void saveMinMax() {
	EEPROM.updateInt(frontMinAddress, frontMin);
	EEPROM.updateInt(frontMaxAddress, frontMax);
	EEPROM.updateInt(backMinAddress, backMin);
	EEPROM.updateInt(backMaxAddress, backMax);
}

//setzt MinMax für Front und Back und generiert den Null-Mittelpunkt
void calibrateMinMax(int sensorValFront, int sensorValBack) {
	if (sensorValFront > frontMax) {
		frontMax = sensorValFront;
	} else if (sensorValFront < frontMin) {
		frontMin = sensorValFront;
	}

	if (sensorValBack > backMax) {
		backMax = sensorValBack;
	} else if (sensorValBack < backMin) {
		backMin = sensorValBack;
	}


	frontRange = frontMax - frontMin;
	frontZero = frontRange / 2;

	backRange = backMax - backMin;
	backZero = backRange / 2;
}

int8_t getTemp() {
	//returns the temperature from one DS18S20 in DEG Celsius

	byte data[12];
	byte addr[8];

	if (!ds.search(addr)) {
		//no more sensors on chain, reset search
		ds.reset_search();
		return -1000;
	}

	if (OneWire::crc8(addr, 7) != addr[7]) {
		// Serial.println("CRC is not valid!");
		return -1000;
	}

	if (addr[0] != 0x10 && addr[0] != 0x28) {
		// Serial.print("Device is not recognized");
		return -1000;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1); // start conversion, with parasite power on at the end

	byte present = ds.reset();
	ds.select(addr);
	ds.write(0xBE); // Read Scratchpad


	for (int i = 0; i < 9; i++) { // we need 9 bytes
		data[i] = ds.read();
	}

	ds.reset_search();

	byte MSB = data[1];
	byte LSB = data[0];

	float tempRead = ((MSB << 8) | LSB); //using two's compliment
	float TemperatureSum = tempRead / 16;

	return (int8_t) TemperatureSum;
}

void drawSetup() {
	//clear screen
	lcd.clear(COLOR_BLACK);

	//draws indicatorline for Front
	lcd.drawLine(0, DRAWSTEERINICATOROFFSET + 70, 320, DRAWSTEERINICATOROFFSET + 70, COLOR_WHITE);
	lcd.drawLine(0, DRAWSTEERINICATOROFFSET + 71, 320, DRAWSTEERINICATOROFFSET + 71, COLOR_WHITE);
	for (int i = 0; i < 16; i++) { // 16, weil ich i * 20 rechne = 320 pixel
		if (i == 8) {
			lcd.drawLine(i * 20, DRAWSTEERINICATOROFFSET + 50, i * 20, DRAWSTEERINICATOROFFSET + 70, COLOR_WHITE);
		} else {
			lcd.drawLine(i * 20, DRAWSTEERINICATOROFFSET + 65, i * 20, DRAWSTEERINICATOROFFSET + 70, COLOR_WHITE);
		}
	}

	//draws indicatorline for Back
	lcd.drawLine(0, DRAWSTEERINICATOROFFSET + 75, 320, DRAWSTEERINICATOROFFSET + 75, COLOR_WHITE);
	lcd.drawLine(0, DRAWSTEERINICATOROFFSET + 76, 320, DRAWSTEERINICATOROFFSET + 76, COLOR_WHITE);
	for (int i = 0; i < 16; i++) { //16, weil ich i * 20 rechne = 320 pixel
		if (i == 8) {
			lcd.drawLine(i * 20, DRAWSTEERINICATOROFFSET + 76, i * 20, DRAWSTEERINICATOROFFSET + 95, COLOR_WHITE);
		} else {
			lcd.drawLine(i * 20, DRAWSTEERINICATOROFFSET + 76, i * 20, DRAWSTEERINICATOROFFSET + 80, COLOR_WHITE);
		}
	}

	//öltemp text
	lcd.drawText(55, 40, "C TEMP", 1, COLOR_WHITE, COLOR_BLACK);
}

int16_t drawSteerIndicatorFront(int sensorValFront, int16_t lastXFront) {

	lcd.drawLine(lastXFront - 1, DRAWSTEERINICATOROFFSET + 35, lastXFront - 1, DRAWSTEERINICATOROFFSET + 64, COLOR_BLACK);
	lcd.drawLine(lastXFront, DRAWSTEERINICATOROFFSET + 35, lastXFront, DRAWSTEERINICATOROFFSET + 64, COLOR_BLACK);
	lcd.drawLine(lastXFront + 1, DRAWSTEERINICATOROFFSET + 35, lastXFront + 1, DRAWSTEERINICATOROFFSET + 64, COLOR_BLACK);

	if (lastXFront >= 158 && lastXFront <= 162) {
		lcd.drawLine(160, DRAWSTEERINICATOROFFSET + 50, 160, DRAWSTEERINICATOROFFSET + 70, COLOR_WHITE);
	}

	lastXFront = (int)((sensorValFront - frontMin) * frontIndicatorStep); //-frontmin ist wichtig, sonst wird lastxfront nie 0
	// Serial.print(frontInicatorStep);
	// Serial.print(" | ");
	// Serial.println((sensorValFront - frontMin) * frontIndicatorStep);

	lcd.drawLine(lastXFront - 1, DRAWSTEERINICATOROFFSET + 35, lastXFront - 1, DRAWSTEERINICATOROFFSET + 64, COLOR_WHITE);
	lcd.drawLine(lastXFront, DRAWSTEERINICATOROFFSET + 35, lastXFront, DRAWSTEERINICATOROFFSET + 64, COLOR_WHITE);
	lcd.drawLine(lastXFront + 1, DRAWSTEERINICATOROFFSET + 35, lastXFront + 1, DRAWSTEERINICATOROFFSET + 64, COLOR_WHITE);

	return lastXFront;
}

int16_t drawSteerIndicatorBack(int sensorValBack, int16_t lastXBack) {
	lcd.drawLine(lastXBack - 1, DRAWSTEERINICATOROFFSET + 81, lastXBack - 1, DRAWSTEERINICATOROFFSET + 110, COLOR_BLACK);
	lcd.drawLine(lastXBack, DRAWSTEERINICATOROFFSET + 81, lastXBack, DRAWSTEERINICATOROFFSET + 110, COLOR_BLACK);
	lcd.drawLine(lastXBack + 1, DRAWSTEERINICATOROFFSET + 81, lastXBack + 1, DRAWSTEERINICATOROFFSET + 110, COLOR_BLACK);

	if (lastXBack >= 158 && lastXBack <= 162) {
		lcd.drawLine(160, DRAWSTEERINICATOROFFSET + 76, 160, DRAWSTEERINICATOROFFSET + 95, COLOR_WHITE);
	}

	lastXBack = (int)((sensorValBack - backMin) * backIndicatorStep);

	// Serial.println(lastXBack);
	lcd.drawLine(lastXBack - 1, DRAWSTEERINICATOROFFSET + 81, lastXBack - 1, DRAWSTEERINICATOROFFSET + 110, COLOR_WHITE);
	lcd.drawLine(lastXBack, DRAWSTEERINICATOROFFSET + 81, lastXBack, DRAWSTEERINICATOROFFSET + 110, COLOR_WHITE);
	lcd.drawLine(lastXBack + 1, DRAWSTEERINICATOROFFSET + 81, lastXBack + 1, DRAWSTEERINICATOROFFSET + 110, COLOR_WHITE);

	return lastXBack;
}

void steerBackLeft() {
	PORTC &= ~(1 << 3);
	PORTC |= (1 << 2);
}
void steerBackRight() {
	PORTC &= ~(1 << 2);
	PORTC |= (1 << 3);
}
void steerBackStop() {
	PORTC &= ~(1 << 2);
	PORTC &= ~(1 << 3);
}

void steerFrontLeft() {
	PORTD &= ~(1 << 5);
	PORTD |= (1 << 4);
}
void steerFrontRight() {
	PORTD &= ~(1 << 4);
	PORTD |= (1 << 5);
}
void steerFrontStop() {
	PORTD &= ~(1 << 4);
	PORTD &= ~(1 << 5);
}

void steerAllStop() {
	steerFrontStop();
	steerBackStop();
}

void autoSteerFront() {
	if (sensorValBack > ((backZero + backMin))){ //+ STEERACTIONTOLERANCE)) {
		steerBackLeft();
	} else if (sensorValBack < ((backZero + backMin))){ // - STEERACTIONTOLERANCE)) {
		steerBackRight();
	} else {
		steerBackStop();
		autoSteer = false;
	}
}

void autoSteerAllrad() {
	bool frontIsLeft = sensorValFront <= (frontZero + frontMin) ? true : false;
	bool backIsLeft = sensorValBack <= (backZero + backMin) ? true : false;

	int16_t frontOffset = 0;
	int16_t backOffset = 0;

	if (frontIsLeft) {
		frontOffset = (frontZero + frontMin) - sensorValFront;
		if (backIsLeft) {
			backOffset = (backZero + backMin) - sensorValBack;
			// Serial.println("steer right1");
			if (frontOffset >= STEERACTIONTOLERANCE) {
				steerBackRight();
			} else {
				if (backOffset >= STEERACTIONTOLERANCE) {
					steerBackRight();
				} else {
					steerBackStop();
					autoSteer = false;
				}
			}
		} else {
			backOffset = sensorValBack - (backZero + backMin);
			if (backOffset >= (frontOffset + STEERACTIONTOLERANCE)) {
				// Serial.println("steer left1");
				steerBackLeft();
			} else if (backOffset <= (frontOffset - STEERACTIONTOLERANCE)) {
				// Serial.println("steer right2");
				steerBackRight();
			} else {
				steerBackStop();
				autoSteer = false;
			}
		}
	} else {
		frontOffset = sensorValFront - (frontZero + frontMin);
		if (backIsLeft) {
			backOffset = (backZero + backMin) - sensorValBack;
			if (backOffset >= (frontOffset + STEERACTIONTOLERANCE)) {
				// Serial.println("steer right3");
				steerBackRight();
			} else if (backOffset <= (frontOffset - STEERACTIONTOLERANCE)) {
				// Serial.println("steer left2");
				steerBackLeft();
			} else {
				steerBackStop();
				autoSteer = false;
			}
		} else {
			backOffset = sensorValBack - (backZero + backMin);
			// Serial.println("steer left3");
			if (frontOffset >= STEERACTIONTOLERANCE) {
				steerBackLeft();
			} else {
				if (backOffset >= STEERACTIONTOLERANCE) {
					steerBackLeft();
				} else {
					steerBackStop();
					autoSteer = false;
				}
			}
		}
	}
}

void autoSteerHundsgang() {

	//auf welcher seite befinden sich die räder
	bool frontIsLeft = sensorValFront <= (frontZero + frontMin) ? true : false;
	bool backIsLeft = sensorValBack <= (backZero + backMin) ? true : false;

	int16_t frontOffset = 0;
	int16_t backOffset = 0;

	if (frontIsLeft) {
		frontOffset = (frontZero + frontMin) - sensorValFront;

		if (backIsLeft) {
			backOffset = (backZero + backMin) - sensorValBack;

			if (backOffset >= (frontOffset + STEERACTIONTOLERANCE)) {
				// Serial.println("steer right");
				steerBackRight();
			} else if (backOffset <= (frontOffset - STEERACTIONTOLERANCE)) {
				// Serial.println("steer left");
				steerBackLeft();
			} else {
				// Serial.println("steer nothing 1");
				steerBackStop();
				autoSteer = false;
			}
		} else {
			backOffset = sensorValBack - (backZero + backMin);
			// Serial.println("steer left");
			if (frontOffset >= STEERACTIONTOLERANCE) {
				steerBackLeft();
			} else {

				if (backOffset >= STEERACTIONTOLERANCE) {
					steerBackLeft();
				} else {
					steerBackStop();
					autoSteer = false;
				}

				// Serial.print(frontOffset);
				// Serial.print(" | ");
				// Serial.print(backOffset);
				// Serial.print("|");
				// Serial.println(STEERACTIONTOLERANCE);
			}
		}
	} else {
		frontOffset = sensorValFront - (frontZero + frontMin);
		if (backIsLeft) {
			backOffset = (backZero + backMin) - sensorValBack;
			// Serial.println("steer right");
			if (frontOffset >= STEERACTIONTOLERANCE) {
				steerBackRight();
			} else {
				if (backOffset >= STEERACTIONTOLERANCE) {
					steerBackRight();
				} else {
					steerBackStop();
					autoSteer = false;
				}
			}
		} else {
			backOffset = sensorValBack - (backZero + backMin);
			if (backOffset >= (frontOffset + STEERACTIONTOLERANCE)) {
				// Serial.println("steer left");
				steerBackLeft();
			} else if (backOffset <= (frontOffset - STEERACTIONTOLERANCE)) {
				// Serial.println("steer right");
				steerBackRight();
			} else {
				// Serial.println("steer nothing 2");
				steerBackStop();
				autoSteer = false;
			}
		}
	}
	// Serial.print(frontOffset);
	// Serial.print(" | ");
	// Serial.println(backOffset);
}