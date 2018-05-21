/*note to self
   Be at peace. This is the Final Version Tracked on Git

  button:
  # set boolean values to denote what mode we are in or what function is in oepration currently.
  # set double to keep track of clock time so that the button push is not accepted unless there
  is a 2 second delay.

  #put a marking at the end of the text file raw data so that when we reach the end,
  we know that FFT is over.

*/
int buttonState = 0;
int lastButtonState = 0;
int buttonPushCounter = 0;
int buttonPin = 2; //D4 is GPIO2
unsigned long buttonTime = 0;
unsigned long ButtonPress;
unsigned long nextButtonPress;
boolean fftNotDone = true;
boolean resultsNotDone = true;

int fileCount = 1; //when deleting multiple files.
double amp = 0; //saves amplitude of the most dominant Hz in the sample that is FFTed
int stringConcat = 1; //count variable used to concat minutes worth of analyzed data.
//int fftCount = 0;
String line; //string used to read in cur time from server.
String stringToSend; //string that is read and will be sen to server
int numOfDataSent = 0; //variable that keeps count of how many mintues worth of data was sent to server
int varForDataCount = 0; //close dataFile every 500 recordings from MPU. Sampling rate is 50 so every 10 seconds.
String acelVal;
int firstTime = 1;

//1=recordData 2=fft 3=sendToServer
int control = 1;
String timeFileName = "time.txt";
String resultFileName = "result.txt";

//****************************    MPU6050 headers + variables + definitions   ************************************
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 15  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 //
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size   (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//needed for samping
// Idling
//unsigned long firstMillis = 0 ;
unsigned long firstMicros = 0 ;
unsigned long thisMicros = 0 ;
unsigned long idx = 1 ;

//********************************   SD card headers + variables    ***********************************
#include <SD.h>

//for sd card and saving in it.
String dataString = "";
String dataString_t = "";
const int chipSelect = 15;
float y, p, r;
float ax, ay, az;
File dataFile1, dataFile2, dataFile3, results, timer;

//***************************   webserver related headers + variables   *****************************

#include <ESP8266WiFi.h>
#include "I2Cdev.h"
#include <SPI.h>
String buff;

char *ssid = "BCI Lab";
char *password = "bcilab317a";
//char *content ="{\"n\":\"daehoandmyeonghu\"}";

//**************************   FFT headers + variables + definitions   ************************************
#include "arduinoFFT.h"
//#define SAMPLES 512         //appx. 10 seconds of data (must be power of 2)
//#define SAMPLES 2048         //appx. 40 seconds of data
#define SAMPLES 1024          //appx. 20 seconds of data
#define SAMPLING_FREQUENCY 50 //Hz, must be less than 10000 due to ADC

arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

//lets take the average of peaks.
int count = 0;




// ======================================================================================================
//                          /$$$$$$  /$$$$$$$$ /$$$$$$$$ /$$   /$$ /$$$$$$$
//                         /$$__  $$| $$_____/|__  $$__/| $$  | $$| $$__  $$
//                        | $$  \__/| $$         | $$   | $$  | $$| $$  \ $$
//                        |  $$$$$$ | $$$$$      | $$   | $$  | $$| $$$$$$$/
//                         \____  $$| $$__/      | $$   | $$  | $$| $$____/
//                         /$$  \ $$| $$         | $$   | $$  | $$| $$
//                        |  $$$$$$/| $$$$$$$$   | $$   |  $$$$$$/| $$
//                         \______/ |________/   |__/    \______/ |__/
// ======================================================================================================
void setup() {


  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  //   mpu.setDMPEnabled(false);


  //*********************************************    MPU6050 Setup   *****************************************
//  const uint8_t sda = D7;
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //sensor offsets
  mpu.setXGyroOffset(-1249);
  mpu.setYGyroOffset(-1627);
  mpu.setZGyroOffset(1104);
  mpu.setXAccelOffset(179); // 1688 factory default for my test chip
  mpu.setYAccelOffset(-67); // 1688 factory default for my test chip
  mpu.setZAccelOffset(0); //

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(0), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //*********************************************    SD card Setup   *****************************************
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  } else {
    Serial.println("SD card opened successfully");
  }
  firstMicros = micros() ;
  mpu.setDMPEnabled(false);

  //*********************************************    FFT  Setup   *****************************************
  // For recording data
  //    SD.remove(timeFileName);
  //    SD.remove(resultFileName);
  if (control == 1) {
    for (int x = 1; x < 4; x++) {
      SD.remove(String(x) + ".TXT");
      Serial.println("Erased " + String(x) + ".TXT");
    }

    dataFile3 = SD.open("3.TXT", FILE_WRITE);
    dataFile2 = SD.open("2.TXT", FILE_WRITE);
    dataFile1 = SD.open("1.TXT", FILE_WRITE);

    SD.remove(timeFileName);
    SD.remove(resultFileName);
    timer = SD.open(timeFileName, FILE_WRITE);
    results = SD.open(resultFileName, FILE_WRITE);
  }
  
//  // For FFT
  if (control == 2) {
    dataFile1 = SD.open("1.txt");
    dataFile2 = SD.open("2.txt");
    dataFile3 = SD.open("3.txt");
  }
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  

}

// ======================================================================================================
//       /$$      /$$  /$$$$$$  /$$$$$$ /$$   /$$       /$$        /$$$$$$   /$$$$$$  /$$$$$$$
//      | $$$    /$$$ /$$__  $$|_  $$_/| $$$ | $$      | $$       /$$__  $$ /$$__  $$| $$__  $$
//      | $$$$  /$$$$| $$  \ $$  | $$  | $$$$| $$      | $$      | $$  \ $$| $$  \ $$| $$  \ $$
//      | $$ $$/$$ $$| $$$$$$$$  | $$  | $$ $$ $$      | $$      | $$  | $$| $$  | $$| $$$$$$$/
//      | $$  $$$| $$| $$__  $$  | $$  | $$  $$$$      | $$      | $$  | $$| $$  | $$| $$____/
//      | $$\  $ | $$| $$  | $$  | $$  | $$\  $$$      | $$      | $$  | $$| $$  | $$| $$
//      | $$ \/  | $$| $$  | $$ /$$$$$$| $$ \  $$      | $$$$$$$$|  $$$$$$/|  $$$$$$/| $$
//      |__/     |__/|__/  |__/|______/|__/  \__/      |________/ \______/  \______/ |__/
// ======================================================================================================
void loop() {
  //   //get time stamp
  if (firstTime == 1 && control == 1) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("Connecting to \'");


    Serial.print(ssid);
    Serial.println("\'");

    WiFi.begin(ssid, password);

    //ap에 접속이 될때까지 대기
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected.");

    WiFiClient httpClient;

    char *host = "capstone.cafe24app.com";
    char *url = "/v/date"; 
    int port = 80;

    if (!httpClient.connect(host, port)) {
      Serial.println("[WiFi] connection failed...");
      return;
    }

    Serial.println("[WiFi] request from the client was handled...");
    // http의 get 메세지 사용 웹페이지 요청
    Serial.println("Connected to server");

    httpClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (httpClient.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout !");
        httpClient.stop();
        return;
      }
    }

    //웹 서버로부터 수신된 데이터를 줄 단위로 출력
    while (httpClient.available()) {
      line = httpClient.readStringUntil('\r');
      Serial.println(line);
    }

    httpClient.stop();


    dataString = ("{\"time\":" + String(line) + ",\"numOfDataSent\":\"" + String(numOfDataSent) + "\",\"user_no\":\"2\",\"data\":[");
    timer.println(dataString);
//    Serial.println(dataString);
    timer.close();
//    results.close();




    mpu.setDMPEnabled(true);
    ButtonPress = millis();
        firstTime = 0;
  }

  // read the pushbutton input pin

//  boolean safe = false;
  buttonState = digitalRead(buttonPin);
if(ButtonPress - millis() > 300){
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter  
    ButtonPress = millis();
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:


        buttonPushCounter++;
        control++;
        if(control >5){
          control = 1;
        }
      
      Serial.println("on");
      Serial.print("number of button pushes: ");
      Serial.println(buttonPushCounter);
      Serial.print("control is");
      Serial.println(control);
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
    // Delay a little bit to avoid bouncing
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
}

  if (control == 1) {
//    Serial.println("RECORDING START");
    recordData();

  } else if (control == 2) {
      Serial.println("FFT START");
      if(firstTime ==0){
        dataFile1.print(String("FINISHED"));
        dataFile2.print(String("FINISHED"));
        dataFile3.print(String("FINISHED"));

        dataFile1.flush();dataFile2.flush();dataFile3.flush();
          firstTime =1;
        dataFile1.close();
        dataFile2.close();
        dataFile3.close();
        dataFile1 = SD.open("1.txt");
        dataFile2 = SD.open("2.txt");
        dataFile3 = SD.open("3.txt");
      }
      
    doFFT();
  } else if (control == 3) {
    Serial.println("start server");
    mpu.setDMPEnabled(false);
    sendtoServer();
    control = 4;
    Serial.println("done sending data");
  }


}

//                               /$$$$$$$$ /$$$$$$$$ /$$$$$$$$
//                              | $$_____/| $$_____/|__  $$__/
//                              | $$      | $$         | $$
//                              | $$$$$   | $$$$$      | $$
//                              | $$__/   | $$__/      | $$
//                              | $$      | $$         | $$
//                              | $$      | $$         | $$
//                              |__/      |__/         |__/

void doFFT() {
  double x_hz, y_hz, z_hz, x_amp, y_amp, z_amp, x_use, y_use, z_use, x_peakSum, y_peakSum, z_peakSum;
  int verification_x, verification_y, verification_z;
  //X Y Z values are all saved in separate files.
  //We are repeating this loop below 9 times because each
  //FFT is 20 seconds and we have three axis information.
  //Then we take an average of the three values to calculate an average for 1 minute.
  for (int xyz = 0; xyz < 9; xyz++) {
    unsigned long StartTime = millis();

    /*Read Saved Data*/
    for (int i = 0; i < SAMPLES; i++) {
      if (xyz % 3 == 0) {
        acelVal = dataFile1.readStringUntil('\n'); 
        if (acelVal == "FINISHED") {
                 Serial.print("acelVal was ");
                 Serial.println(acelVal);
                  dataFile1.close();
                  dataFile2.close();
                  dataFile3.close();
          control = 3;
          return;

        }
        vReal[i] = acelVal.toFloat();
        vImag[i] = 0;
      } else if (xyz % 3 == 1) {
        acelVal = dataFile2.readStringUntil('\n');
                if (acelVal == "FINISHED") {
                 Serial.print("acelVal was ");
                 Serial.println(acelVal);
                  dataFile1.close();
                  dataFile2.close();
                  dataFile3.close();
          control = 3;
          return;

        }
        vReal[i] = acelVal.toFloat();
        vImag[i] = 0;
      } else if (xyz % 3 == 2) {
        acelVal = dataFile3.readStringUntil('\n');
                if (acelVal == "FINISHED") {
                 Serial.print("acelVal was ");
                 Serial.println(acelVal);
                  dataFile1.close();
                  dataFile2.close();
                  dataFile3.close();
          control = 3;
          return;

        }
        vReal[i] = acelVal.toFloat();
        vImag[i] = 0;
      }
      //            while(micros() < (microseconds + sampling_period_us)){
      //            }

    }

    /*FFT*/
    /*HANN or Hanning type windowing is the most widely used windowing type
      and is appropriate for most cases. It has good frequency resolution which
      we need to diagnose Parkinsons and has good reduced spectral leakage.

      But FLT_TOP or flat top seems like a good replacement candidate because
      flat tops are specialized for Sine waves where AMPLITUDE is important*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    amp = FFT.Amplitude(vReal, SAMPLES);

    //acceptable = 1 if amplitude of peak frequency is above 2 SD of the data. 0 Otherwise
    vReal[0] = 0;
    int acceptable = FFT.dataCheck(vReal, amp);
    double meanis = FFT.meanis(vReal, amp);
    double SDis = FFT.SDis(vReal, amp);

    //Measuring how much time FFT takes.
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;

    Serial.print("FFT took ");
    Serial.print(ElapsedTime);
    Serial.print(" milliseconds. Peak is ");
    Serial.println(peak);     //Print out what frequency is the most dominant.
    Serial.print("Amplitude is ");
    Serial.println(amp);
    Serial.print("SD is ");
    Serial.println(SDis);
    Serial.print("Mean is ");
    Serial.println(meanis);
    Serial.print("Result of Data Check is ");
    Serial.println(acceptable);
    Serial.println("");

    if (xyz % 3 == 0) {
      x_peakSum = x_peakSum + peak;
      x_hz = peak;
      x_amp = amp;
      verification_x = verification_x + acceptable;
    } else if (xyz % 3 == 1) {
      y_peakSum = y_peakSum + peak;
      y_hz = peak;
      y_amp = amp;
      verification_y = verification_y + acceptable;
    } else if (xyz % 3 == 2) {
      z_peakSum = z_peakSum + peak;
      z_hz = peak;
      z_amp = amp;
      verification_z = verification_z + acceptable;
    }

    count = count + 1;
    if (count == 9) {
      Serial.print("Average peak x y z for one minute is ");

      Serial.println(String(x_peakSum / 3) + " " + String(y_peakSum / 3) + " " + String(z_peakSum / 3));
      x_hz = x_peakSum / 3;
      y_hz = y_peakSum / 3;
      z_hz = z_peakSum / 3;

      x_peakSum = 0;
      y_peakSum = 0;
      z_peakSum = 0;
      count = 0;

      if (verification_z == 3) {
        x_use = 1;
      } else {
        x_use = 0;
      }
      if (verification_y == 3) {
        y_use = 1;
      } else {
        y_use = 0;
      }
      if (verification_z == 3) {
        z_use = 1;
      } else {
        z_use = 0;
      }
      verification_x = 0;
      verification_y = 0;
      verification_z = 0;
    }

  }//xyz loop end

  //Write results to file.
  //  results = SD.open(resultFileName, FILE_WRITE);

  if (stringConcat % 7 == 0) {
    dataString = ("{\"x_hz\":\"" + String(x_hz) + "\",\"x_amp\":\"" + String(x_amp) + "\",\"x_use\":\"" + String(x_use) + "\",\"y_hz\":\"" + String(y_hz) + "\",\"y_amp\":\"" + String(y_amp) + "\",\"y_use\":\"" + String(y_use) + "\",\"z_hz\":\"" + String(z_hz) + "\",\"z_amp\":\"" + String(z_amp) + "\",\"z_use\":\"" + String(z_use) + "\"},");
    results.println(dataString);
    stringConcat = 1;
  } else {
    dataString = ("{\"x_hz\":\"" + String(x_hz) + "\",\"x_amp\":\"" + String(x_amp) + "\",\"x_use\":\"" + String(x_use) + "\",\"y_hz\":\"" + String(y_hz) + "\",\"y_amp\":\"" + String(y_amp) + "\",\"y_use\":\"" + String(y_use) + "\",\"z_hz\":\"" + String(z_hz) + "\",\"z_amp\":\"" + String(z_amp) + "\",\"z_use\":\"" + String(z_use) + "\"},");
    stringConcat = stringConcat + 1;
    results.print(dataString);
  }
  //  fftCount = fftCount+1;
  Serial.println(dataString);
  results.flush();
}

//         /$$$$$$$  /$$$$$$$$  /$$$$$$   /$$$$$$  /$$$$$$$  /$$$$$$$        /$$$$$$$   /$$$$$$  /$$$$$$$$ /$$$$$$
//        | $$__  $$| $$_____/ /$$__  $$ /$$__  $$| $$__  $$| $$__  $$      | $$__  $$ /$$__  $$|__  $$__//$$__  $$
//        | $$  \ $$| $$      | $$  \__/| $$  \ $$| $$  \ $$| $$  \ $$      | $$  \ $$| $$  \ $$   | $$  | $$  \ $$
//        | $$$$$$$/| $$$$$   | $$      | $$  | $$| $$$$$$$/| $$  | $$      | $$  | $$| $$$$$$$$   | $$  | $$$$$$$$
//        | $$__  $$| $$__/   | $$      | $$  | $$| $$__  $$| $$  | $$      | $$  | $$| $$__  $$   | $$  | $$__  $$
//        | $$  \ $$| $$      | $$    $$| $$  | $$| $$  \ $$| $$  | $$      | $$  | $$| $$  | $$   | $$  | $$  | $$
//        | $$  | $$| $$$$$$$$|  $$$$$$/|  $$$$$$/| $$  | $$| $$$$$$$/      | $$$$$$$/| $$  | $$   | $$  | $$  | $$
//        |__/  |__/|________/ \______/  \______/ |__/  |__/|_______/       |_______/ |__/  |__/   |__/  |__/  |__/
void recordData() {
  varForDataCount = varForDataCount + 1;
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    ax = aaReal.x;
    ay = aaReal.y;
    az = aaReal.z;
#endif

    do {
      thisMicros = micros();
    } while (thisMicros - firstMicros < idx * 20000);
    idx = idx + 1;
    thisMicros = thisMicros / 1000000;
    Serial.println(String(ax) + " " + String(ay) + " " + String(az));

    dataFile1.println(String(ax));
    dataFile2.println(String(ay));
    dataFile3.println(String(az));

    if (varForDataCount % 1024 == 0) {
      dataFile1.flush();
      dataFile2.flush();
      dataFile3.flush();
    }


  }
}


//                /$$$$$$  /$$$$$$$$ /$$$$$$$  /$$    /$$ /$$$$$$$$ /$$$$$$$
//               /$$__  $$| $$_____/| $$__  $$| $$   | $$| $$_____/| $$__  $$
//              | $$  \__/| $$      | $$  \ $$| $$   | $$| $$      | $$  \ $$
//              |  $$$$$$ | $$$$$   | $$$$$$$/|  $$ / $$/| $$$$$   | $$$$$$$/
//               \____  $$| $$__/   | $$__  $$ \  $$ $$/ | $$__/   | $$__  $$
//               /$$  \ $$| $$      | $$  \ $$  \  $$$/  | $$      | $$  \ $$
//              |  $$$$$$/| $$$$$$$$| $$  | $$   \  $/   | $$$$$$$$| $$  | $$
//               \______/ |________/|__/  |__/    \_/    |________/|__/  |__/
void sendtoServer() {
  // fftCount =0;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Connecting to \'");
  Serial.print(ssid);
  Serial.println("\'");

  WiFi.begin(ssid, password);

  //ap에 접속이 될때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");

  WiFiClient httpClient;

  char *host = "capstone.cafe24app.com";
  int port = 80;

  if (!httpClient.connect(host, port)) {
    Serial.println("[WiFi] connection failed...");
    return;
  }

  //  Serial.println("[WiFi] request from the client was handled...");
  // http의 get 메세지 사용 웹페이지 요청
  Serial.println("Connected to server");

  timer = SD.open(timeFileName);

  if (timer) {
    //read first line which contains time information FFT data
    stringToSend = timer.readStringUntil('\n');
    stringToSend = stringToSend + timer.readStringUntil('\n');
    String timeAndUserNo = stringToSend;
    String start = timeAndUserNo.substring(0, timeAndUserNo.indexOf("Sent") + 7);
    String finish = timeAndUserNo.substring(timeAndUserNo.indexOf("Sent") + 8);
    Serial.println(start);
    Serial.println(finish);
    timer.close();
    stringToSend = "";
    results = SD.open(resultFileName);
    // read from the file until there's nothing else in it:
    while (results.available()) {
      if (!httpClient.connect(host, port)) {
        Serial.println("[WiFi] connection failed...");
        return;
      }
      //each line has 7 minutes worth of data.
      //so here in the loop we are sending 5 lines making it 35 min worth of data
      for (int x = 0; x < 5; x++) {
        buff = results.readStringUntil('\n');
        //                        Serial.println("read Number " + String(x) + " and buff : " + buff);
        stringToSend = stringToSend + buff;
      }

      //after concatenating the data, get rid of the hanging comma at the end
      //and properly end the string by adding ]}.
      stringToSend.remove(stringToSend.lastIndexOf(","));
      stringToSend = stringToSend.substring(0, stringToSend.length()) + "]}";

      //if it is not the first line, we need to concatenate {"data":[ to fit data transfer protocol
      buff = stringToSend;
//      Serial.println("buff is " + buff);
      stringToSend = start + String(numOfDataSent);
      stringToSend = stringToSend + finish + buff;

      httpClient.println("POST /v/data HTTP/1.1");
      httpClient.println("Host: capstone.cafe24app.com");
      httpClient.println("Connection: close");
      httpClient.println("Content-Type: application/json");
      httpClient.print("Content-Length: ");
      httpClient.println(stringToSend.length());
      httpClient.println();
      httpClient.println(stringToSend);
      Serial.println(stringToSend);
      stringToSend = "";
      numOfDataSent = numOfDataSent + 35;

      //      웹 서버로부터 수신된 데이터를 줄 단위로 출력
      while (httpClient.available()) {
        String line = httpClient.readStringUntil('\r');
        line.remove(8);
        Serial.print(line);
      }

      //      delay(100);
      yield();
      httpClient.stop();
    }

    // close the file:
    results.close();
    numOfDataSent = 0;

  } else {
    // if the file didn't open, print an error:
    Serial.println("Error Opening File");

  }
}
