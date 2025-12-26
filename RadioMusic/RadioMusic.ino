/*
 RADIO MUSIC
 https://github.com/TomWhitwell/RadioMusic
 
 Audio out: Onboard DAC, teensy3.1 pin A14/DAC
 
 Bank Button: 2
 Bank LEDs 3,4,5,6
 Reset Button: 8  
 Reset LED 11 
 Reset CV input: 9 
 Channel Pot: A9 
 Channel CV: A8 // check 
 Time Pot: A7 
 Time CV: A6 // check 
 SD Card Connections: 
 SCLK 14
 MISO 12
 MOSI 7 
 SS   10 
 
 NB: Compile using modified versions of: 
 SD.cpp (found in the main Arduino package) 
 play_sd_raw.cpp  - In Teensy Audio Library 
 play_sc_raw.h    - In Teensy Audio Library 
 
 from:https://github.com/TomWhitwell/RadioMusic/tree/master/Collateral/Edited%20teensy%20files

 Additions and changes:
 2016 by Jouni Stenroos - jouni.stenroos@iki.fi 
 - New bank change mode
 - Removing 330 file limit
 - Improving reset
 - File sorting
 - Audio crossfade
 - Some refactoring and organization of code.
 
 */
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RadioMusic.h"
#include "AudioSystemHelpers.h"
#include "Settings.h"
#include "LedControl.h"
#include "FileScanner.h"
#include "AudioEngine.h"
#include "Interface.h"
#include "PlayState.h"

#ifdef DEBUG
#define D(x) x
#else
#define D(x)
#endif

// Press reset button to reboot
//#define RESET_TO_REBOOT
//#define ENGINE_TEST

#define EEPROM_BANK_SAVE_ADDRESS 0

#define FLASHTIME 	10  	// How long do LEDs flash for?
#define SHOWFREQ 	250 	// how many millis between serial Debug updates

#define peakFPS 30   //  FRAMERATE FOR PEAK METER

#define SD_CARD_CHECK_DELAY 20

// //////////
// TIMERS
// //////////

elapsedMillis showDisplay;
elapsedMillis resetLedTimer = 0;
elapsedMillis ledFlashTimer = 0;

elapsedMillis meterDisplayDelayTimer; // Counter to hide MeterDisplay after bank change
elapsedMillis peakDisplayTimer; // COUNTER FOR PEAK METER FRAMERATE

uint32_t  fileMillis;
uint32_t  fileMillisSeg;
uint32_t  fileMillisSegOld = UINT32_MAX;

int divideList[11] = {1,2,3,4,6,8,12,16,32,64,128};
int divideIndex;
int divide;

boolean divideReload = false;

// ================================
// PLAYHEAD BASED CLOCK
// ================================
uint32_t lastSegIndex = UINT32_MAX;
elapsedMillis clockPulseTimer;
bool clockHigh = false;

uint32_t clockHighStartMs = 0;    // HIGH開始時刻（ミリ秒）
uint32_t currentClockPulseWidthMs = 4; // 現在のパルス幅（ミリ秒）

uint32_t playheadBaseMs = 0;
bool playheadBaseValid = false;

bool forceFirstClock = false;


int prevBankTimer = 0;
boolean flashLeds = false;
boolean bankChangeMode = false;
File settingsFile;

Settings settings("SETTINGS.TXT");
LedControl ledControl;
FileScanner fileScanner;
AudioEngine audioEngine;
Interface interface;
PlayState playState;

int NO_FILES = 0;
uint8_t noFilesLedIndex = 0;

uint8_t rebootCounter = 0;

bool sampleChangePending = false;
bool bankChangePending = false;       // バンク切替待ちフラグ
int nextBankIndex = -1;               // 切替予定バンク番号


// --- グローバル変数 ---
bool doubleBlinkActive = false;
uint8_t blinkStep = 0;
elapsedMillis blinkTimer;
int prevSelected = -1;

// --- 点滅時間（ms）チューニング用定数 ---
// POT選択一致 → ダブル点滅
const uint16_t BLINK_ON_TIME  = 100; // 点灯時間
const uint16_t BLINK_OFF_TIME = 100; // 消灯時間
const uint16_t BLINK_END_WAIT = 100; // 最終消灯から通常表示に戻るまで

// 再生待機中 → 全LED点滅
const uint16_t WAIT_BLINK_ON_TIME  = 50; // 点灯時間
const uint16_t WAIT_BLINK_OFF_TIME = 50; // 消灯時間

// LED物理接続が 0,1,2,3 に対して、左から右表示にしたい場合のマッピング
const uint8_t ledMap[16] = {
	0b0000, // 0
	0b1000, // 1
	0b0100, // 2
	0b1100, // 3
	0b0010, // 4
	0b1010, // 5
	0b0110, // 6
	0b1110, // 7
	0b0001, // 8
	0b1001, // 9
	0b0101, // 10
	0b1101, // 11
	0b0011, // 12
	0b1011, // 13
	0b0111, // 14
	0b1111  // 15
};
const uint8_t ledBlinkMap[16] = {
	0b1111, // 0
	0b1000, // 1
	0b0100, // 2
	0b1100, // 3
	0b0010, // 4
	0b1010, // 5
	0b0110, // 6
	0b1110, // 7
	0b0001, // 8
	0b1001, // 9
	0b0101, // 10
	0b1101, // 11
	0b0011, // 12
	0b1011, // 13
	0b0111, // 14
	0b1111  // 15
};

bool hiPosForCurrentCh = true;

void setup() {

#ifdef DEBUG_STARTUP
	while( !Serial );
	Serial.println("Starting");
#endif // DEBUG_STARTUP

	ledControl.init();
	ledControl.single(playState.bank);

	// SD CARD SETTINGS FOR AUDIO SHIELD
	SPI.setMOSI(7);
	SPI.setSCK(14);

	boolean hasSD = openSDCard();
	if(!hasSD) {
		Serial.println("Rebooting");
		reBoot(0);
	}

	settings.init(hasSD);

	File root = SD.open("/");
	fileScanner.scan(&root, settings);

	getSavedBankPosition();

	audioEngine.init(settings);

	int numFiles = 0;
	for(int i=0;i<=fileScanner.lastBankIndex;i++) {
		numFiles += fileScanner.numFilesInBank[i];
	}
	D(Serial.print("File Count ");Serial.println(numFiles););

	if(numFiles == 0) {
		NO_FILES = 1;
		D(Serial.println("No files"););
		ledFlashTimer = 0;
	} else if(fileScanner.numFilesInBank[playState.bank] == 0) {
		D(Serial.println("Empty bank"););
		while(fileScanner.numFilesInBank[playState.bank] == 0) {
			playState.bank++;
			if(playState.bank == fileScanner.lastBankIndex) {
				playState.bank = 0;
			}
		}
		D(Serial.print("Set bank to ");Serial.println(playState.bank););
	}

	interface.init(fileScanner.fileInfos[playState.bank][0].size, fileScanner.numFilesInBank[playState.bank], settings, &playState);

	D(Serial.println("--READY--"););
}

void getSavedBankPosition() {
	// CHECK  FOR SAVED BANK POSITION
	int a = 0;
	a = EEPROM.read(EEPROM_BANK_SAVE_ADDRESS);
	if (a >= 0 && a <= fileScanner.lastBankIndex) {
		D(
			Serial.print("Using bank from EEPROM ");
			Serial.print(a);
			Serial.print(" . Active banks ");
			Serial.println(fileScanner.lastBankIndex);

		);
		playState.bank = a;
		playState.channelChanged = true;
	} else {
		EEPROM.write(EEPROM_BANK_SAVE_ADDRESS, 0);
	};
}

boolean openSDCard() {
	if (!(SD.begin(SS))) {

		Serial.println("No SD.");
		while (!(SD.begin(SS))) {
			ledControl.single(15);
			delay(SD_CARD_CHECK_DELAY);
			ledControl.single(rebootCounter % 4);
			delay(SD_CARD_CHECK_DELAY);
			rebootCounter++;
			Serial.print("Crash Countdown ");
			Serial.println(rebootCounter);
			if (rebootCounter > 4) {
				return false;
			}
		}
	}
	return true;
}

void loop() {

	#ifdef CHECK_CPU
	checkCPU();
//	audioEngine.measure();
	#endif

	if(NO_FILES) {
		// TODO : Flash the lights to show there are no files
		if(ledFlashTimer > 100) {
			ledControl.single(noFilesLedIndex);
			noFilesLedIndex++;
			noFilesLedIndex %= 4;
			ledFlashTimer = 0;
			rebootCounter ++;
			// Wait 3 seconds and reboot
			if(rebootCounter == 30) {
				reBoot(0);
			}
		}
		return;
	}

	updateInterfaceAndDisplay();

	audioEngine.update();

	if(audioEngine.error) {
		// Too many read errors, reboot
		Serial.println("Audio Engine errors. Reboot");
		reBoot(0);
	}

	if (playState.channelChanged) {
		D(
		Serial.print("RM: Going to next channel : ");
		if(playState.channelChanged) Serial.print("RM: Channel Changed. ");
		Serial.println("");
		);

		playState.currentChannel = playState.nextChannel;

		AudioFileInfo* currentFileInfo = &fileScanner.fileInfos[playState.bank][playState.nextChannel];

		audioEngine.changeTo(currentFileInfo, interface.start);
		lastSegIndex = UINT32_MAX;
		playState.channelChanged = false;

		resetLedTimer = 0;

		fileMillis = currentFileInfo->getFileLengthMillis();
		divideReload = true;
	}

	// indexマッピング
	divideIndex = map(interface.start, 0, 8192, 0, 10);
	// index範囲確認（予防）
	divideIndex = constrain(divideIndex, 0, 10);

	divide = divideList[divideIndex];
	
	if(audioEngine.isSeeked()){
    // ループ終了タイミング

		ledControl.showReset(true);   // ★ ループ先頭でLED点灯
		resetLedTimer = 0;            // タイマーリセット
		lastSegIndex = UINT32_MAX; // ★ ループ先頭で必ずクロックを出す
		forceFirstClock = true;   // ★ 追加

		if(sampleChangePending) {
			playState.channelChanged = true;
			audioEngine.skipTo(0);  // サンプルリスタート
			lastSegIndex = UINT32_MAX;
			sampleChangePending = false;
			flashLeds = false;
		}

		if(bankChangePending) {
			if(nextBankIndex >= 0 && nextBankIndex <= fileScanner.lastBankIndex) {
				playState.bank = nextBankIndex;
				if (playState.nextChannel >= fileScanner.numFilesInBank[playState.bank])
					playState.nextChannel = fileScanner.numFilesInBank[playState.bank] - 1;

				interface.setChannelCount(fileScanner.numFilesInBank[playState.bank]);
				playState.channelChanged = true;
				// EEPROM.write(EEPROM_BANK_SAVE_ADDRESS, playState.bank);
			}
			bankChangePending = false;
			flashLeds = false;   // LED点滅終了
		}

	}



	// --- Reset LEDの自動消灯処理 ---
	if (resetLedTimer > 100) { // 100ms以上経過したら消灯
		ledControl.showReset(false);
	}

	
	// =====================================
	// UNIFORM CLOCK GENERATOR (FINAL FIXED)
	// =====================================

	if (fileMillis > 0 && divide > 0) {

		uint32_t playheadMs = audioEngine.getPlayheadMillis();

		uint32_t segmentIndex =
			(uint64_t)playheadMs * divide / fileMillis;

		if (segmentIndex >= (uint32_t)divide)
			segmentIndex = divide - 1;

		// ★ ループ直後の強制1発
		if (forceFirstClock) {

			// パルス幅計算: 現在の分割数の1周期の半分
			uint32_t intervalMs = fileMillis / divide;
			currentClockPulseWidthMs = intervalMs / 2;
			digitalWrite(RESET_CV, HIGH);
			clockHighStartMs = millis();
			clockHigh = true;

			lastSegIndex = segmentIndex;
			forceFirstClock = false;

		}
		// 通常処理
		else if (segmentIndex != lastSegIndex) {

			// パルス幅計算: 現在の分割数の1周期の半分
			uint32_t intervalMs = fileMillis / divide;
			currentClockPulseWidthMs = intervalMs / 2;
			digitalWrite(RESET_CV, HIGH);
			clockHighStartMs = millis();
			clockHigh = true;

			lastSegIndex = segmentIndex;
		}
	}


	// ---- パルスOFF ----
	if (clockHigh) {
		// HIGH開始から現在までの経過（ミリ秒）
		uint32_t elapsedMs = millis() - clockHighStartMs;
		if (elapsedMs >= currentClockPulseWidthMs) {
			digitalWrite(RESET_CV, LOW);
			clockHigh = false;
		}
	}


}

void updateInterfaceAndDisplay() {

	uint16_t changes = checkInterface();
	updateDisplay(changes);
}

// --- LED制御関数 ---
void updateDisplay(uint16_t changes) {
    uint8_t selectedIndex = playState.nextChannel;
    uint8_t currentIndex  = playState.currentChannel;
	

    // POTが「再生中インデックス」に一致した瞬間に2回点滅開始
    if (selectedIndex == currentIndex && selectedIndex != prevSelected) {
        doubleBlinkActive = true;
        blinkStep = 0;
        blinkTimer = 0;
    }
    prevSelected = selectedIndex;

	
	if(selectedIndex > currentIndex && !hiPosForCurrentCh){
		hiPosForCurrentCh = true;
		doubleBlinkActive = true;
		blinkStep = 0;
		blinkTimer = 0;
	}

	if(selectedIndex < currentIndex && hiPosForCurrentCh){
		hiPosForCurrentCh = false;
		doubleBlinkActive = true;
		blinkStep = 0;
		blinkTimer = 0;
	}

    // --- 2回点滅処理（選択インデックス = 再生中インデックス） ---
    if (doubleBlinkActive) {
		uint8_t pattern = ledBlinkMap[currentIndex & 0x0F];

        if (blinkStep == 0 && blinkTimer < BLINK_ON_TIME) {
            ledControl.multi(pattern);
			// ledControl.multi(0x0F);
        } else if (blinkStep == 0 && blinkTimer >= BLINK_ON_TIME) {
            ledControl.multi(0);
            blinkStep = 1;
            blinkTimer = 0;
        } else if (blinkStep == 1 && blinkTimer < BLINK_OFF_TIME) {
			ledControl.multi(0);
        } else if (blinkStep == 1 && blinkTimer >= BLINK_OFF_TIME) {
			ledControl.multi(pattern);
            // ledControl.multi(0x0F);
            blinkStep = 2;
            blinkTimer = 0;
        } else if (blinkStep == 2 && blinkTimer >= BLINK_ON_TIME) {
            ledControl.multi(0);
			blinkStep = 3;
			blinkTimer = 0;
        } else if (blinkStep == 3 && blinkTimer >= BLINK_END_WAIT) {
            doubleBlinkActive = false;
        }
        return; // 点滅優先
    }

	// サンプル切替待ち点滅 (全点滅)
	if (sampleChangePending) {
		static bool waitBlinkOn = true;
		static elapsedMillis waitBlinkTimer = 0;

		if (waitBlinkOn && waitBlinkTimer < WAIT_BLINK_ON_TIME) {
			ledControl.multi(0x0F); // 全LED点灯 (4つ分)
		} else if (waitBlinkOn && waitBlinkTimer >= WAIT_BLINK_ON_TIME) {
			ledControl.multi(0);
			waitBlinkOn = false;
			waitBlinkTimer = 0;
		} else if (!waitBlinkOn && waitBlinkTimer < WAIT_BLINK_OFF_TIME) {
			ledControl.multi(0);
		} else if (!waitBlinkOn && waitBlinkTimer >= WAIT_BLINK_OFF_TIME) {
			ledControl.multi(0x0F);
			waitBlinkOn = true;
			waitBlinkTimer = 0;
		}
		return;
	}

	// バンク切替待ち点滅 (チェイス表示)
	if (bankChangePending) {
		static uint8_t pos = 0;
		static int8_t dir = 1;             // 進行方向 (1=右, -1=左)
		static elapsedMillis runBlinkTimer = 0;

		if (runBlinkTimer > 50) {         // 0.05秒ごとに進む
			pos += dir;
			if (pos == 3) dir = -1;        // 右端で折り返し
			else if (pos == 0) dir = 1;    // 左端で折り返し
			runBlinkTimer = 0;
		}

		uint8_t pattern = (1 << pos);      // 1つだけ点灯
		ledControl.multi(pattern);

		return;
	}

    // --- 通常表示 ---
	uint8_t ledPattern = ledMap[selectedIndex & 0x0F];
	ledControl.multi(ledPattern);
}

// INTERFACE //

uint16_t checkInterface() {

	uint16_t changes = interface.update();
	changes |= interface.updateChannelCVTrigger();

	#ifdef RESET_TO_REBOOT
	if (changes & BUTTON_SHORT_PRESS) {
		reBoot(0);
	}
	#endif

	// BANK MODE HANDLING
	if((changes & BUTTON_LONG_PRESS) && !bankChangeMode) {
		D(Serial.println("Enter bank change mode"););
		bankChangeMode = true;
		nextBank();
//		ledFlashTimer = 0;
	} else if((changes & BUTTON_LONG_RELEASE) && bankChangeMode) {
		D(Serial.println("Exit bank change mode"););
		flashLeds = false;
		bankChangeMode = false;
	}

	if(changes & BUTTON_PULSE) {
//		flashLeds = false;
		if(bankChangeMode) {
			D(Serial.println("BUTTON PULSE"););
			nextBank();
		} else {
			D(Serial.println("Button Pulse but not in bank mode"););
		}

	}

	boolean resetTriggered = changes & RESET_TRIGGERED;

	bool skipToStartPoint = false;
	bool speedChange = false;

	if(settings.pitchMode) {

		if(resetTriggered) {
			skipToStartPoint = true;
		}

		if((changes & (ROOT_NOTE_CHANGED | ROOT_POT_CHANGED | ROOT_CV_CHANGED) ) || resetTriggered) {
			speedChange = true;
		}

	} else {

		if((changes & CHANGE_START_NOW) || resetTriggered) {
			skipToStartPoint = true;
		}
	}

	if(resetTriggered) {
		if((changes & CHANNEL_CHANGED) || playState.nextChannel != playState.currentChannel) {
			// playState.channelChanged = true;
			sampleChangePending = true;
		} else {
			resetLedTimer = 0;
		}
	}

	if(speedChange) doSpeedChange();
	if(skipToStartPoint && !playState.channelChanged) {
		if(settings.pitchMode) {
			audioEngine.skipTo(0);
			lastSegIndex = UINT32_MAX;
		} else {
			D(Serial.print("Skip to ");Serial.println(interface.start););
			sampleChangePending = true;
		}
	}

	if (changes & CHANNEL_CV_TRIGGERED) {
		// TODOリセット用関数作る
		audioEngine.skipTo(0);
		// ★ クロック位相をリセットするだけ
    	lastSegIndex = UINT32_MAX;
	}

	return changes;
}

void doSpeedChange() {
	float speed = 1.0;
	speed = interface.rootNote - settings.rootNote;
	D(Serial.print("Root ");Serial.println(interface.rootNote););
	speed = pow(2,speed / 12);

	audioEngine.setPlaybackSpeed(speed);
}

void nextBank() {
    if(fileScanner.lastBankIndex == 0) {
        D(Serial.println("Only 1 bank."););
        return;
    }
    int candidate = playState.bank + 1;
    if (candidate > fileScanner.lastBankIndex) {
        candidate = 0;
    }
    if(fileScanner.numFilesInBank[candidate] == 0) {
        D(Serial.print("No file in bank ");Serial.println(candidate););
        playState.bank = candidate; // 空バンクをスキップ
        nextBank();
        return;
    }

    nextBankIndex = candidate;
    bankChangePending = true;   // ★ 切替待ちフラグをセット

    D(
        Serial.print("RM: Next Bank (pending) ");
        Serial.println(nextBankIndex);
    );

	meterDisplayDelayTimer = 0;
	EEPROM.write(EEPROM_BANK_SAVE_ADDRESS, playState.bank);
}

#ifdef ENGINE_TEST
boolean tested = false;
int testIndex = 0;

void engineTest() {

	if(!tested) {
		audioEngine.test(fileScanner.fileInfos[playState.bank][0],fileScanner.fileInfos[playState.bank][1]);
		tested = true;
	}

	uint8_t changes = interface.update();

	if(changes & BUTTON_SHORT_PRESS) {
		testIndex += 2;
		if(testIndex >= fileScanner.numFilesInBank[playState.bank]) {
			Serial.println("Back to start");
			testIndex = 0;
		}
		audioEngine.test(fileScanner.fileInfos[playState.bank][testIndex],fileScanner.fileInfos[playState.bank][testIndex+1]);
	}

	return;
}
#endif

void peakMeter() {
	if( (peakDisplayTimer < 50) || (meterDisplayDelayTimer < settings.meterHide) ) return;

	float peakReading = audioEngine.getPeak();
	int monoPeak = round(peakReading * 4);
	monoPeak = round(pow(2, monoPeak));
//	D(Serial.print("Peak ");Serial.print(peakReading,4);Serial.print(" ");Serial.println(monoPeak););
	ledControl.multi(monoPeak - 1);
	peakDisplayTimer = 0;
}

