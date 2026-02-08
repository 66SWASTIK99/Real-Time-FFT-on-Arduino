#define FS        512
#define N         128
#define HOP_SIZE  64
#define PI        3.1415

#define DIV_2    (1.0f / 2.0f)
#define DIV_6    (1.0f / 6.0f)
#define DIV_24   (1.0f / 24.0f)
#define DIV_120  (1.0f / 120.0f)    // 5!

int8_t circBuf[N];   //circular buffer
float fftReal[N];
float fftImag[N];

volatile uint16_t writeIndex = 0;
volatile uint16_t samplesCollected = 0;
volatile bool fftReady = false;

// BIT REVERSAL LOOK UP TABLE (7-bit -> 128)
// only 64 needed due to swapping
const uint8_t reverse7Table[64]={
  0, 64, 32, 96, 16, 80, 48,112,
  8, 72, 40,104, 24, 88, 56,120,
  4, 68, 36,100, 20, 84, 52,116,
 12, 76, 44,108, 28, 92, 60,124,
  2, 66, 34, 98, 18, 82, 50,114,
 10, 74, 42,106, 26, 90, 58,122,
  6, 70, 38,102, 22, 86, 54,118,
 14, 78, 46,110, 30, 94, 62,126
};

// float squareRoot(float number){
//   if (number == 0) return 0;  
//   float guess = number / 2.0; //initial guess
//   float error = 0.09;     // accuracy
//   float diff;

//   do {
//     float newGuess = 0.5 * (guess + number / guess);
//     diff = newGuess - guess;
//     if (diff < 0) diff = -diff; // absolute value
//     guess = newGuess;
//   } while (diff > error);

//   return guess;
// }

//MACLAURIN SIN, COS
inline float fastSin(float x) {
  while (x > PI) x -= 2 * PI; 
  while (x < -PI) x += 2 * PI;
  float x2 = x * x;
  // x * (1 - x²/6 + x⁴/120 - x⁶/5040)
  return x * (1.0f +
              x2 * (-DIV_6 +
              x2 * ( DIV_120
              )));
}

inline float fastCos(float x) {
  while (x > PI) x -= 2 * PI; 
  while (x < -PI) x += 2 * PI;
  float x2 = x * x;
  // 1 - x²/2 + x⁴/24 - x⁶/720
  return 1.0f +
         x2 * (-DIV_2 +
         x2 * ( DIV_24 
         ));
}

//HANN WINDOW
void applyHannWindow(float *real){
  for (int n = 0; n < N; n++) {
    float angle = (2.0 * PI * n) / (N - 1);
    float w = 0.5 * (1.0 - fastCos(angle));
    real[n] *= w;
  }
}

//BIT REVERSAL
void bitReversalPermutation(float *real, float *imag){
  for (uint8_t i = 0; i < N; i++){
    uint8_t j = reverse7Table[i];
    if (j > i) {
      float tmp = real[i];
      real[i] = real[j];
      real[j] = tmp;

      tmp = imag[i];
      imag[i] = imag[j];
      imag[j] = tmp;
    }
  }
}

//FFT
void fft(float *real, float *imag){
  bitReversalPermutation(real, imag);

  for (uint8_t stage = 1; stage <= 7; stage++){
    uint8_t m = 1 << stage;
    uint8_t m2 = m >> 1;

    float theta = -2.0 * PI / m;
    float wm_real = fastCos(theta);
    float wm_imag = fastSin(theta);

    for (uint8_t k = 0; k < N; k += m){
      float w_real = 1.0;
      float w_imag = 0.0;

      for (uint8_t j = 0; j < m2; j++){
        uint8_t t = k + j + m2;
        uint8_t u = k + j;

        float tr = w_real * real[t] - w_imag * imag[t];
        float ti = w_real * imag[t] + w_imag * real[t];

        real[t] = real[u] - tr;
        imag[t] = imag[u] - ti;

        real[u] += tr;
        imag[u] += ti;

        // w = w * wm
        float temp = w_real;
        w_real = temp * wm_real - w_imag * wm_imag;
        w_imag = temp * wm_imag + w_imag * wm_real;
      }
    }
  }
}

void setupTimer1_512Hz() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 31249;        // 16 MHz / (1 * 31250) = 512 Hz
  TCCR1B = 0x09;       // CTC, prescaler = 1
  TIMSK1 = 0x02;       // enable compare interrupt
  sei();
}

ISR(TIMER1_COMPA_vect) {
  float sample = (5.0 / 1023.0) * analogRead(A0);     // conversion to actual value

  circBuf[writeIndex] = sample*10;
  writeIndex = (writeIndex + 1) % N;

  samplesCollected++;

  // trigger FFT every HOP_SIZE samples after initial fill
  if (samplesCollected >= N &&
      ((samplesCollected - N) % HOP_SIZE == 0)) {
    fftReady = true;
  }
}

void setup() {
  Serial.begin(230400);
  analogReference(DEFAULT);

  for (int i = 0; i < N; i++) {
    circBuf[i] = 0.0;
    fftReal[i]  = fftImag[i]  = 0.0;
  }

  setupTimer1_512Hz();
}

void copyCircularToFFT() {
  uint16_t idx = writeIndex;
  for (int i = 0; i < N; i++) {
    fftReal[i] = circBuf[idx]/10;
    fftImag[i] = 0.0;
    idx = (idx + 1) % N;
  }
}

void loop() {
  if (fftReady) {
    fftReady = false;

    copyCircularToFFT();

    applyHannWindow(fftReal);
    fft(fftReal, fftImag);

    Serial.write(0xAA);
    for (int i = 0; i < N / 2; i++) {
      float mag = fftReal[i] * fftReal[i] + fftImag[i] * fftImag[i];
      //float approx = squareRoot(mag);
      Serial.write((byte *)&mag, 4);
    }
  }
}