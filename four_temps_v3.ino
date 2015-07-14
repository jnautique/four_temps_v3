/*************************************************** 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
 /* This example does a test of the TCP client capability:
  * Initialization
  * Optional: SSID scan
  * AP connection
  * DHCP printout
  * DNS lookup
  * Optional: Ping
  * Connect to website and print out webpage contents
  * Disconnect
SmartConfig is still beta and kind of works but is not fully vetted!
It might not work on all networks!
*/

#include <avr/wdt.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "utility/debug.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "nautique"           // cannot be longer than 32 characters!
#define WLAN_PASS       "ginger_jc"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "api.wunderground.com"
char WEBPAGE_SUPERIOR[]   = "/api/0d57fb7ea2f1b187/conditions/q/CO/Superior.xml";
//char WEBPAGE_SUPERIOR[]   = "/api/0d57fb7ea2f1b187/conditions/q/pws:mhoza2.xml";  // Alaska - cold
//char WEBPAGE_SUPERIOR[]   = "/api/0d57fb7ea2f1b187/conditions/q/pws:inorther33.xml"; // Austrailia - hot
char WEBPAGE_LINCOLN[]    = "/api/0d57fb7ea2f1b187/conditions/q/CA/Lincoln.xml";
char WEBPAGE_PARKER[]     = "/api/0d57fb7ea2f1b187/conditions/q/SD/Parker.xml";
char WEBPAGE_NEWCASTLE[]  = "/api/0d57fb7ea2f1b187/conditions/q/CA/Newcastle.xml";


// Define Constants
// Max string length may have to be adjusted depending on data to be extracted
#define MAX_STRING_LEN  20

// LED constants
#define LEDBLINK_PIN	7 		// I/O pin connected to LED
#define LEDBLINK_MS		1000	// Blink rate (in ms)

// Setup vars
char tagStr[MAX_STRING_LEN] = "";
char dataStr[MAX_STRING_LEN] = "";
char dataStrNoDec[MAX_STRING_LEN] = "";
char tmpStr[MAX_STRING_LEN] = "";
char endTag[3] = {'<', '/', '\0'};
int len;
int timeout_count = 0;
int iterations = 0;
static unsigned int  ledStatus = LOW;  // Last set LED mode.

bool fast_lookup = true;
// Flags to differentiate XML tags from document elements (ie. data)
boolean tagFlag = false;
boolean dataFlag = false;

volatile int counter;      // Count number of times ISR is called.
volatile int countmax = 30; // Arbitrarily selected 3 for this example.
                          // Timer expires after about 24 secs if
                          // 8 sec interval is selected below.


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;

// Setup software serial ports.  We never receive any data on the serial lines, so 
// they all share pin 8 for receive.

SoftwareSerial mySerialA(8, A0); // RX, TX
SoftwareSerial mySerialB(8, A1); // RX, TX
SoftwareSerial mySerialC(8, A2); // RX, TX
SoftwareSerial mySerialD(8, A3); // RX, TX

void setup(void)
{

  // Set LEDBLINK pin to output
  pinMode(LEDBLINK_PIN, OUTPUT);
  
  mySerialA.begin(9600);
  mySerialB.begin(9600);
  mySerialC.begin(9600);
  mySerialD.begin(9600);
  
  mySerialA.print("----");
  mySerialB.print("----");
  mySerialC.print("----");
  mySerialD.print("----");
  
 
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
}

void loop(void)
{
  
  
  for (int i=0; i<4; i++) {
    
    watchdogEnable(); // set up watchdog timer in interrupt-only mode
    
    int timeout=0;
    bool timeout_assert = false;
 
 	  // Blink LED
 	  // Toggle LED.
    ledStatus = (ledStatus==HIGH ? LOW : HIGH);

    // Set LED pin status.
    digitalWrite(LEDBLINK_PIN, ledStatus);
 	  //ledBlink();
    
    Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
    if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
      Serial.println(F("Failed!"));
      while(1);
    }
     
    Serial.println(F("Connected!"));
    /* Wait for DHCP to complete */
    Serial.println(F("Request DHCP"));
    while (!cc3000.checkDHCP() && !timeout_assert){
      delay(100); 
      timeout++;
      if (timeout > 320) {
        timeout_assert = true;
        
      }
    }  
    
    timeout = 0;
    /* Display the IP address DNS, Gateway, etc. */  
    while (!displayConnectionDetails() && !timeout_assert) {
      delay(1000);
      timeout++;
      if (timeout > 32) {
        timeout_assert = true;
        Serial.println("Timeout");
      }
    }
    
    ip = 0;
    timeout=0;
    // Try looking up the website's IP address
    Serial.print(WEBSITE); Serial.print(F(" -> "));
    while (ip == 0 && !timeout_assert) {
      if (! cc3000.getHostByName(WEBSITE, &ip)) {
        Serial.println(F("Couldn't resolve!"));
      }
      delay(500);
      timeout++;
      if (timeout > 320) {
        timeout_assert = true;
        Serial.println("Timeout");
      }
    }
    
    cc3000.printIPdotsRev(ip);
    
    Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
    if (www.connected()) {
      www.fastrprint(F("GET "));
      if (i == 0) {
        www.fastrprint(WEBPAGE_SUPERIOR);
        Serial.println();
        Serial.println("Superior");
      } else if (i == 1) {
        www.fastrprint(WEBPAGE_LINCOLN);
        Serial.println();
        Serial.println("Lincoln");
      } else if (i == 2) {
        www.fastrprint(WEBPAGE_NEWCASTLE);
        Serial.println();
        Serial.println("Newcastle");
      } else if (i == 3) {
        www.fastrprint(WEBPAGE_PARKER);
        Serial.println();
        Serial.println("Parker");
      } else {
        www.fastrprint(WEBPAGE_PARKER);
        Serial.println("Default - Parker");
      }
      www.fastrprint(F(" HTTP/1.1\r\n"));
      www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
      www.fastrprint(F("\r\n"));
      www.println();
    } else {
      Serial.println(F("Connection failed"));    
      return;
    }
    //Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
    Serial.println(F("-------------------------------------"));
    
    /* Read data until either the connection is closed, or the idle timeout is reached. */ 
    unsigned long lastRead = millis();
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
      while (www.available() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
        char c = www.read();
        serialEvent(c, i);
        lastRead = millis();
      }
    }
    
    
    www.close();
    Serial.println(F("-------------------------------------"));
    
    /* You need to make sure to clean up after yourself or the CC3000 can freak out */
    /* the next time your try to connect ... */
    Serial.println(F("\n\nDisconnecting"));
    cc3000.disconnect();
    
    // Turn off the WDT
    wdt_disable();
    
    if (timeout_assert) {
      Serial.println("#### Timeout Asserted ####");
    }
  
  Serial.println("Waiting . . .");
  
  // The first time through, lookup all temps quickly
  // After first time through, wait 3 minutes between temperature lookup
  if (!fast_lookup)
    delay(240000); // Get a new update every 3 minutes
    
  Serial.println("Timeout count:");
  Serial.println(timeout_count);
  iterations++;
  Serial.println("Iteration count:");
  Serial.println(iterations);
  
  
  }
  
  fast_lookup=false;
 
}

/**************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
//  uint8_t valid, rssi, sec, index;
  uint8_t valid, rssi, sec;
  uint32_t index;
  char ssidname[33]; 

//  index = cc3000.startSSIDscan();
  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

//  Serial.print(F("Networks found: ")); Serial.println(&index);
  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}



// Process each char from web
float serialEvent(char inChar, int itr) {

  // Read a char
  // char inChar = client.read();
  //Serial.print(".");
 
  if (inChar == '<') {
     addChar(inChar, tmpStr);
     tagFlag = true;
     dataFlag = false;

  } else if (inChar == '>') {
     addChar(inChar, tmpStr);

     if (tagFlag) {      
        //
        //strncpy(tagStr, tmpStr, strlen(tmpStr)+1);
        for (int i=0; i<MAX_STRING_LEN; i++) {
          tagStr[i] = tmpStr[i];
        }
     }

     // Clear tmp
     clearStr(tmpStr);

     tagFlag = false;
     dataFlag = true;      
     
  } else if (inChar != 10) {
     if (tagFlag) {
        // Add tag char to string
        addChar(inChar, tmpStr);
        
        int cur_s = 0;
        for (cur_s=0; tmpStr[cur_s] != '\0'; cur_s++);
        
        bool strng_match = true;
        
        if (cur_s==2) {
        
          for (int i=0; i<3; i++) {
            if (tmpStr[i] != endTag[i]);
              strng_match = false;
          }
        } else {
          strng_match = false;
        }

        // Check for </XML> end tag, ignore it
        if ( tagFlag && strcmp(tmpStr, endTag) == 0 ) {
        //if ( tagFlag && strng_match) {
           clearStr(tmpStr);
           tagFlag = false;
           dataFlag = false;
        }
     }
     
     if (dataFlag) {
        // Add data char to string
        addChar(inChar, dataStr);
     }
  }  
 
  // If a LF, process the line
  if (inChar == 10 ) {

    //Serial.print("tagStr: ");
    //Serial.println(tagStr);
    ///Find specific tags and print data
    if (matchTag("<temp_f>", 8)) {
      Serial.println("Temp: ");
      Serial.print(dataStr);
      Serial.println();
        
      int dec_location = 0;
      int j = 0;
        
      // Remove decimal
      for (int i=0; i<MAX_STRING_LEN+1; i++) {
        if (dataStr[i] == '.') {
        //if (i == 2) {
          Serial.println("Found decimal at:");
          dec_location = i;
          Serial.println(dec_location);
          j--;
        } else {
          dataStrNoDec[j] = dataStr[i];
        }
      j++;
      }
      
      // Find the current string length
      int cur_s = 0;
      for (cur_s=0; dataStr[cur_s] != '\0'; cur_s++);
        
      if (dec_location == 0) {
        dataStrNoDec[cur_s] = '0';
      }
      
      char my_space[4];
      
      for (cur_s=0; dataStrNoDec[cur_s] != '\0'; cur_s++);
      
      //Serial.println("Current chars:");
      //Serial.println(cur_s);
      
      if (cur_s == 2) {
        my_space[0] = ' ';
        my_space[1] = ' ';
        my_space[2] = '\0';
      } else if (cur_s == 3){
        my_space[0] = ' ';
        my_space[1] = '\0';
        my_space[2] = '\0';
      } else {
        my_space[0] = '\0';
        my_space[1] = '\0';
        my_space[2] = '\0';
      }
      
      
      //for (int i=0; i<MAX_STRING_LEN; i++) {
      //  Serial.println(i);
      //  Serial.println(dataStrNoDec[i]);
      //}
        
        
      Serial.println(dataStrNoDec);
      Serial.println("Decimal location:");
      Serial.println(dec_location);
        
      if (itr == 0) {
        mySerialA.write('v'); //Reset the display - this forces the cursor to return to the beginning 
        mySerialA.print(my_space);
        mySerialA.print(dataStrNoDec);
        mySerialA.write(0x77);  // Decimal, colon, apostrophe control command
        mySerialA.write((1<<2) ); // Turns tenth place decimal
      } else if (itr == 1) {
        mySerialB.write('v'); //Reset the display - this forces the cursor to return to the beginning 
        mySerialB.print(my_space);
        mySerialB.print(dataStrNoDec);
        mySerialB.write(0x77);  // Decimal, colon, apostrophe control command
        mySerialB.write((1<<2) ); // Turns tenth place decimal
        
      } else if (itr == 2) {
        mySerialC.write('v'); //Reset the display - this forces the cursor to return to the beginning 
        mySerialC.print(my_space);
        mySerialC.print(dataStrNoDec);
        mySerialC.write(0x77);  // Decimal, colon, apostrophe control command
        mySerialC.write((1<<2) ); // Turns tenth place decimal
        
      } else if (itr == 3) {
        mySerialD.write('v'); //Reset the display - this forces the cursor to return to the beginning 
        mySerialD.print(my_space);
        mySerialD.print(dataStrNoDec);
        mySerialD.write(0x77);  // Decimal, colon, apostrophe control command
        mySerialD.write((1<<2) ); // Turns tenth place decimal
      }
    }
     
    //Serial.println("Clearing...");
    // Clear all strings
    clearStr(tmpStr);
    clearStr(tagStr);
    clearStr(dataStr);
    clearStr(dataStrNoDec);

    // Clear Flags
    tagFlag = false;
    dataFlag = false;
  }
}

/////////////////////
// Other Functions //
/////////////////////

// Function to clear a string
void clearStr (char* str) {
  int len = MAX_STRING_LEN;
  for (int c = 0; c < len; c++) {
     str[c] = '\0';
  }
}


//Function to add a char to a string and check its length
void addChar (char ch, char* str) {
  char *tagMsg  = "<TRUNCATED_TAG>";
  char *dataMsg = "-TRUNCATED_DATA-";
  
  int s = 0;
  for (s=0; str[s] != '\0'; s++);

  // Check the max size of the string to make sure it doesn't grow too
  // big.  If string is beyond MAX_STRING_LEN assume it is unimportant
  // and replace it with a warning message.
  //if (strlen(str) > MAX_STRING_LEN - 2) {
  if (s > MAX_STRING_LEN - 2) {
    if (tagFlag) {
      clearStr(tagStr);
      //strcpy(tagStr,tagMsg);
      for (int i=0; i<MAX_STRING_LEN; i++) {
        tagStr[i] = tagMsg[i];
      }
    }
    
    if (dataFlag) {
      clearStr(dataStr);
      //strcpy(dataStr,dataMsg);
      for (int i=0; i<MAX_STRING_LEN; i++) {
        dataStr[i] = dataMsg[i];
      }
    }

    // Clear the temp buffer and flags to stop current processing 
    clearStr(tmpStr);
    tagFlag = false;
    dataFlag = false;

  } else {
    // Add char to string
    //str[strlen(str)] = ch;
    str[s] = ch;
  }
}

// Function to check the current tag for a specific string
boolean matchTag (char* searchTag, int tag_length) {
  //if ( strcmp(tagStr, searchTag) == 0 ) {
  //  return true;
  //} else {
  //  return false;
  //}
  
  bool match = false;
  
  for (int i = 0; i<tag_length; i++) {
    if (tagStr[i] == searchTag[i]) {
      match = true;
    } else {
      match = false;
      break;
    }
  }
  
  return match;
  
}

/////////////////////////////////////////////////////////////////////
// LED Heartbeat function by Allen C. Huffman (www.appleause.com)
/////////////////////////////////////////////////////////////////////
void ledBlink()
{
  static unsigned int  ledStatus = LOW;  // Last set LED mode.
  static unsigned long ledBlinkTime = 0; // LED blink time.

  // LED blinking heartbeat. Yes, we are alive.
  // For explanation, see:
  // http://playground.arduino.cc/Code/TimingRollover
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    ledStatus = (ledStatus==HIGH ? LOW : HIGH);

    // Set LED pin status.
    digitalWrite(LEDBLINK_PIN, ledStatus);

    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+LEDBLINK_MS;
  }
} // end of ledBlink()


void watchdogEnable()
{
 counter=0;
 cli();                              // disable interrupts

 MCUSR = 0;                          // reset status register flags

                                     // Put timer in interrupt-only mode:                                        
 WDTCSR |= 0b00011000;               // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                     // using bitwise OR assignment (leaves other bits unchanged).
 WDTCSR =  0b01000000 | 0b100001;    // set WDIE (interrupt enable...7th from left, on left side of bar)
                                     // clr WDE (reset enable...4th from left)
                                     // and set delay interval (right side of bar) to 8 seconds,
                                     // using bitwise OR operator.

 sei();                              // re-enable interrupts
 //wdt_reset();                      // this is not needed...timer starts without it

 // delay interval patterns:
 //  16 ms:     0b000000
 //  500 ms:    0b000101
 //  1 second:  0b000110
 //  2 seconds: 0b000111
 //  4 seconds: 0b100000
 //  8 seconds: 0b100001

}

ISR(WDT_vect) // watchdog timer interrupt service routine
{
 counter+=1;

 if (counter < countmax)
 {
   wdt_reset(); // start timer again (still in interrupt-only mode)
 }
 else             // then change timer to reset-only mode with short (16 ms) fuse
 {
   
   MCUSR = 0;                          // reset flags

                                       // Put timer in reset-only mode:
   WDTCSR |= 0b00011000;               // Enter config mode.
   WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                       // set WDE (reset enable...4th from left), and set delay interval
                                       // reset system in 16 ms...
                                       // unless wdt_disable() in loop() is reached first

   //wdt_reset(); // not needed
 }
}