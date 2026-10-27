// nano_scope.ino  (v2)
// Slow multi-channel scope for Arduino Nano (ATmega328P), controlled by scope_gui.py.
//
// Inputs:  A0..A5 (0-5V ONLY, common GND, series resistor + clamp per input)
// Serial:  500000 baud, binary protocol (see scope_gui.py)
//
// Features: exact sample timing (Timer1), pre-trigger ring buffer, rising/falling
// trigger with level, auto / normal / single, 1..6 channels sharing one ring buffer.
// No extra libraries needed.

#include <Arduino.h>

const uint32_t BAUD     = 500000;
const uint8_t  NCH      = 6;       // A0..A5
const uint16_t BUFSZ    = 1020;    // divisible by 1,2,3,4,5,6
const uint8_t  CMD_LEN  = 14;
const uint8_t  HYST     = 3;       // trigger hysteresis in 8-bit ADC codes (~60 mV)

// Timer1 prescalers /8 /64 /256 /1024  ->  0.5 / 4 / 16 / 64 us per tick
const uint8_t PS_BITS[4] = {
  _BV(CS11),
  _BV(CS11) | _BV(CS10),
  _BV(CS12),
  _BV(CS12) | _BV(CS10)
};

enum { FLAG_TRIG = 1, FLAG_OVERRUN = 2, FLAG_FORCED = 4 };
enum { MODE_STOP = 0, MODE_AUTO = 1, MODE_NORMAL = 2, MODE_SINGLE = 3 };

uint8_t  ring[BUFSZ];

// current configuration (set by the GUI)
uint8_t  cMask   = 0b000011;
uint8_t  cPs     = 1;
uint16_t cTicks  = 1000;
uint16_t cN      = 128;     // samples per channel
uint16_t cPre    = 64;      // samples before the trigger point
uint8_t  cTrigCh = 0;
uint8_t  cEdge   = 0;       // 0 rising, 1 falling
uint8_t  cLevel  = 128;     // 8-bit code
uint8_t  cMode   = MODE_STOP;

uint16_t vccmV   = 5000;
uint8_t  cmd[CMD_LEN];
uint8_t  cmdLen  = 0;
uint8_t  frameCount = 0;

// ---------- ADC ----------

uint16_t readVcc() {
  // measure the 1.1V bandgap against AVcc to get the real supply voltage
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  for (uint8_t i = 0; i < 2; i++) {
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
  }
  uint16_t r = ADC;
  return r ? (uint16_t)(1125300UL / r) : 5000;
}

static inline uint8_t adc8(uint8_t ch) {
  ADMUX = _BV(REFS0) | _BV(ADLAR) | ch;     // AVcc reference, left adjusted: ADCH = top 8 bits
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  return ADCH;
}

// ---------- commands from the GUI ----------
// [0xA5, mask, ps, ticks_lo, ticks_hi, n_lo, n_hi, pre_lo, pre_hi, trigch, edge, level, mode, xor]

void applyConfig() {
  cMask   = cmd[1] & 0x3F;
  cPs     = cmd[2] & 3;
  cTicks  = (uint16_t)cmd[3] | ((uint16_t)cmd[4] << 8);
  cN      = (uint16_t)cmd[5] | ((uint16_t)cmd[6] << 8);
  cPre    = (uint16_t)cmd[7] | ((uint16_t)cmd[8] << 8);
  cTrigCh = (cmd[9] > 5) ? 0 : cmd[9];
  cEdge   = cmd[10] & 1;
  cLevel  = cmd[11];
  cMode   = cmd[12] & 3;
  if (cTicks < 2) cTicks = 2;
}

bool pollSerial() {
  bool applied = false;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (cmdLen == 0 && b != 0xA5) continue;
    cmd[cmdLen++] = b;
    if (cmdLen == CMD_LEN) {
      cmdLen = 0;
      uint8_t x = 0;
      for (uint8_t i = 1; i < CMD_LEN - 1; i++) x ^= cmd[i];
      if (x == cmd[CMD_LEN - 1]) {
        applyConfig();
        applied = true;
      }
    }
  }
  return applied;
}

// ---------- frame out ----------
// 0xAA 0x55, mask, flags, ps, ticks(2), vcc_mV(2), n(2), trigIdx(2), data[n][nch], xor

void sendFrame(uint8_t nch, uint16_t n, uint16_t start, uint16_t trigIdx, uint8_t flags) {
  uint8_t h[11];
  h[0]  = cMask;
  h[1]  = flags;
  h[2]  = cPs;
  h[3]  = cTicks & 0xFF;  h[4]  = cTicks >> 8;
  h[5]  = vccmV & 0xFF;   h[6]  = vccmV >> 8;
  h[7]  = n & 0xFF;       h[8]  = n >> 8;
  h[9]  = trigIdx & 0xFF; h[10] = trigIdx >> 8;

  uint8_t x = 0;
  for (uint8_t i = 0; i < 11; i++) x ^= h[i];
  uint16_t total = n * nch;
  for (uint16_t i = 0; i < total; i++) x ^= ring[i];

  uint16_t off = start * nch;               // oldest sample, ring is sent oldest first
  Serial.write(0xAA);
  Serial.write(0x55);
  Serial.write(h, 11);
  Serial.write(&ring[off], total - off);
  if (off) Serial.write(ring, off);
  Serial.write(x);
}

// ---------- acquisition ----------
// Samples continuously into a ring buffer on a Timer1 compare tick, watches the trigger
// channel, and after the trigger keeps sampling for (n - pre - 1) more sets.
// Returns true if a frame was sent, false if a new config aborted the capture.

bool acquire() {
  uint8_t list[NCH];
  uint8_t nch = 0;
  for (uint8_t c = 0; c < NCH; c++) if (cMask & (1 << c)) list[nch++] = c;
  if (nch == 0) return false;

  uint16_t nmax = BUFSZ / nch;
  uint16_t n = cN;
  if (n > nmax) n = nmax;
  if (n < 8) n = 8;
  uint16_t pre = cPre;
  if (pre >= n) pre = n - 1;

  uint8_t tk = 0;                             // trigger channel's slot in the sample set
  for (uint8_t k = 0; k < nch; k++) if (list[k] == cTrigCh) tk = k;
  const uint8_t level = cLevel;
  const uint8_t edge  = cEdge;
  const bool    autoMode = (cMode == MODE_AUTO);

  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | PS_BITS[cPs];         // CTC, TOP = OCR1A
  OCR1A  = cTicks - 1;
  TCNT1  = 0;
  TIFR1  = _BV(OCF1A);

  uint16_t wp = 0;
  uint32_t cnt = 0;
  uint16_t post = 0;
  bool trig = false, armed = false;
  uint8_t flags = 0;

  for (;;) {
    while (!(TIFR1 & _BV(OCF1A))) {}          // wait for the next sample tick
    TIFR1 = _BV(OCF1A);

    uint8_t *p = &ring[wp * nch];
    for (uint8_t k = 0; k < nch; k++) p[k] = adc8(list[k]);
    if (TIFR1 & _BV(OCF1A)) flags |= FLAG_OVERRUN;   // reading took longer than one interval
    uint8_t v = p[tk];
    cnt++;
    if (++wp >= n) wp = 0;

    bool done = false;
    if (!trig) {
      if (cnt > pre) {                         // enough pre-trigger history in the ring
        if (!edge) {
          if ((int)v + HYST < (int)level) armed = true;
          else if (armed && v >= level) trig = true;
        } else {
          if ((int)v > (int)level + HYST) armed = true;
          else if (armed && v <= level) trig = true;
        }
      }
      if (trig) {
        flags |= FLAG_TRIG;
        post = n - pre - 1;
        if (post == 0) done = true;
      } else if (autoMode && cnt >= n) {
        flags |= FLAG_FORCED;                  // no trigger: show what we have
        done = true;
      }
    } else {
      if (--post == 0) done = true;
    }

    if (done) {
      sendFrame(nch, n, wp, (flags & FLAG_TRIG) ? pre : 0xFFFF, flags);
      return true;
    }

    if ((cnt & 31) == 0 && Serial.available() && pollSerial()) return false;
  }
}

// ---------- main ----------

void setup() {
  Serial.begin(BAUD);
  vccmV = readVcc();
  // ADC clock = 16 MHz / 16 = 1 MHz: fast, but above the 200 kHz the datasheet
  // recommends for full 10-bit accuracy, so samples are reduced to 8 bit.
  ADCSRA = _BV(ADEN) | _BV(ADPS2);
}

void loop() {
  pollSerial();
  if (cMode == MODE_STOP || cMask == 0) return;

  if (acquire()) {
    if (cMode == MODE_SINGLE) cMode = MODE_STOP;
    if (++frameCount >= 100) {                 // refresh the Vcc reading now and then
      frameCount = 0;
      uint16_t v = readVcc();
      ADCSRA = _BV(ADEN) | _BV(ADPS2);
      if (v > 3000 && v < 5600) vccmV = v;
    }
  }
}