/*
  UDPSendReceive.pde:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender
 
 A Processing sketch is included at the end of file that can be used to send 
 and received messages for testing with a computer.
 
 created 21 Aug 2010
 by Michael Margolis
 
 This code is in the public domain.
 */


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10,0,0,1);

unsigned int localPort = 8888;      // local port to listen on



// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  // start the Ethernet and UDP:

  Ethernet.begin(mac,ip);
  Udp.begin(localPort);

  Serial.begin(9600);
}

struct message {
  char         space;
  int x:16;
  int y:16;
  int z:16;
};

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  union {
    char c[7];
    message m;
  };
//  buffer buf;
  if(packetSize)
  {
    Udp.read(c,UDP_TX_PACKET_MAX_SIZE);

    IPAddress remote = Udp.remoteIP();
    
        for (int i =0; i < 4; i++)
        {
          Serial.print(remote[i], DEC);
          if (i < 3)
          {
            Serial.print(".");
          }
        }
        Serial.print("\n");  
        // read the packet into packetBufffer
        
      Serial.print(m.x);
      Serial.print(" ");

      Serial.print(m.y);
      Serial.print(" ");
      Serial.println(m.z); 
  }
  
}


