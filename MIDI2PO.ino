// Before uploading to board:
// - Set the CPU clock to 120MHz or 240MHz
// - Set the USB Stack to TinyUSB

// Based on:
// - https://github.com/franklinscudder/Midi2PO
// - https://github.com/ejlabs/arduino-midi-sync
// - https://github.com/kleinpa/operatorer
// - https://learn.adafruit.com/adafruit-feather-rp2040-with-usb-type-a-host
// - https://learn.adafruit.com/adafruit-midi-featherwing
// - https://learn.adafruit.com/usb-midi-host-messenger
// - https://github.com/DisasterAreaDesigns/EZ_USB_MIDI_HOST


// SPDX-FileCopyrightText: 2024 john park for Adafruit Industries
//
// SPDX-License-Identifier: MIT
/**
 * For USB MIDI Host Feather RP2040 with MIDI FeatherWing
 * Modified 12 Jun 2024 - @todbot -- added USB MIDI forwarding
 * Modified by @johnedgarpark -- added UART MIDI forwarding and message filtering
 * originally from: https://github.com/rppicomidi/EZ_USB_MIDI_HOST/blob/main/examples/arduino/EZ_USB_MIDI_HOST_PIO_example/EZ_USB_MIDI_HOST_PIO_example.ino
 */

/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 rppicomidi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/**
 * This demo program is designed to test the USB MIDI Host driver for a single USB
 * MIDI device connected to the USB Host port. It also
 * forwards MIDI received from the USB MIDI device to USB and UART MIDI devices.
 *
 * This program works with a single USB MIDI device connected via a USB hub, but it
 * does not handle multiple USB MIDI devices connected at the same time.
 * 
 *  Libraries (all available via library manager): 
 *  - MIDI -- https://github.com/FortySevenEffects/arduino_midi_library
 */



#include <MIDI.h>

#if defined(USE_TINYUSB_HOST) || !defined(USE_TINYUSB)
#error "Please use the Menu to select Tools->USB Stack: Adafruit TinyUSB"
#endif
#include "pio_usb.h"
#define HOST_PIN_DP 16  // Pin used as D+ for host, D- = D+ + 1
#include "EZ_USB_MIDI_HOST.h"

// USB Host object
Adafruit_USBH_Host USBHost;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

//RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault)
struct mycustomsettings : public MidiHostSettingsDefault
{
    static const unsigned MidiRxBufsize = 512;
};

RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, mycustomsettings)

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDIuart);      // Serial MIDI over MIDI FeatherWing

static uint8_t midiDevAddr = 0;

static bool core0_booting = true;
static bool core1_booting = true;

#define DACPin 0
int clockCount = 0;
bool started = false;
bool tick = false;
long tickTS;

#define BUTTONPin A1
int btnState = 1;
int divs[] = { 6, 12, 24 }; //
int divIndex = 1; // by default sync signals are sent every 12 MIDI clock signals

/* MIDI IN MESSAGE REPORTING */
static void onMidiClock() {
  if(started) {
    if(clockCount == 0) {
      tick = true;
      analogWrite(DACPin, 676); // 1024 / 5 * 3.3v
    }
    else {
      tick = false;
      analogWrite(DACPin, 0);
    }
    clockCount = (++clockCount) % divs[divIndex];
  }
}

static void onMidiStart() {
  started = true;
  clockCount = 0;
}

static void onMidiContinue() {
  started = true;
  clockCount = 0;
}

static void onMidiStop() {
  started = false;
}

static void registerMidiInCallbacks() {
  auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, 0);
  if (intf == nullptr) return;
  intf->setHandleClock(onMidiClock);                      // 0xF8
  intf->setHandleStart(onMidiStart);            // 0xFA
  intf->setHandleContinue(onMidiContinue);      // 0xFB
  intf->setHandleStop(onMidiStop);              // 0xFC

  auto dev = usbhMIDI.getDevFromDevAddr(midiDevAddr);
  if (dev == nullptr) return;
}

/* CONNECTION MANAGEMENT */
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables) {
  //Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
  midiDevAddr = devAddr;
  registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr) {
  ///Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
  midiDevAddr = 0;
}


/* MAIN LOOP FUNCTIONS */

// core1's setup
void setup1() {
  #if ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST
    pinMode(18, OUTPUT);  // Sets pin USB_HOST_5V_POWER to HIGH to enable USB power
    digitalWrite(18, HIGH);
  #endif

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    delay(2000);  // wait for native usb
    while (1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;

  USBHost.configure_pio_usb(1, &pio_cfg);
  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing work done in core1 to free up core0 for other work
  usbhMIDI.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);
  core1_booting = false;
  while (core0_booting);
}

// core1's loop
void loop1() {
  USBHost.task();
}

void setup() {
  TinyUSBDevice.setManufacturerDescriptor("mk135");
  TinyUSBDevice.setProductDescriptor("MIDI2PO");

  MIDIuart.begin(MIDI_CHANNEL_OMNI);          // don't forget OMNI
  MIDIuart.setHandleClock(onMidiClock);       // 0xF8
  MIDIuart.setHandleStart(onMidiStart);       // 0xFA
  MIDIuart.setHandleContinue(onMidiContinue); // 0xFB
  MIDIuart.setHandleStop(onMidiStop);         // 0xFC

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DACPin, OUTPUT);
  analogWrite(DACPin, 0);

  pinMode(BUTTONPin, INPUT_PULLUP);

  core0_booting = false;
  while (core1_booting);
}

void loop() {
  usbhMIDI.readAll();
  usbhMIDI.writeFlushAll();
  MIDIuart.read();
  digitalWrite(LED_BUILTIN, tick ? HIGH : LOW);

	int state = digitalRead(BUTTONPin);
  if(state == 0 && btnState == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
    divIndex = (++divIndex) % 3;
    btnState = 0;
  }
  else if(state == 1 && btnState == 0) {
	  btnState = 1;
  }
}
