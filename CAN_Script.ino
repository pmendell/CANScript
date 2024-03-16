// CAN Script Processor
/*
 * This sketch reads from and writes to a CAN Bus using an MCP2515 interface and the MCP_CAN_lib (https://github.com/Longan-Labs/Arduino_CAN_BUS_MCP2515)
 * By default, the sketch's loop simply checks to see if there is a message waiting to be read from the interface and prints it, if there is one.
 * The sketch goes further in several ways:
 * 
 * 1. Individual CAN messages can be easily composed on the inbound Serial stream and sent out via CAN. If using the Arduino IDE, you can simply type
 *    a command of the form 0xXXX (with up to 8 additional 0xXX arguments) and hit 'send'.
 *   
 * 2. Longer sequences of CAN messages can be composed in a static array and, once compiled/uploaded, can be sent out via CAN on demand. By default, sending
 *    an empty string to Serial will transmit a single message (the next in the sequence) on the CAN Bus. Sending the string "RUN" on Serial will result in 
 *    all further messages in the sequence being sent in rapid succession (one per pass through loop() ).  See the usage() and runCommand() functions for
 *    bells and whistles.
 * 
 * 3. An enhanced read mode can be used to suppress printing of any messages that have been seen previously. This is exceptionally useful
 *    for reverse engineering an existing CAN installation.  For example, to narrow down the message ids associated with a certain function, one
 *    can begin a capture through the sketch and prime it with all the chatter associated with the CAN Bus at startup, clear the output log, then
 *    initiate the function under investigation. The resulting message log will then show the messages triggered by the function with decent suppression 
 *    of much of the normal background chatter on an active bus.  Note that the message history cache can be reset by sending the string "CLEAR" to
 *    the Serial stream
 */
#include <mcp_can.h>
#include <SPI.h>


#define CANARGSLEN 8                                  // length, in bytes, of CAN message arguments. 
#define MSGKEYSIZ (CANARGSLEN+2)                      // key is a concatenation of 2-byte id and argument bytes 
#define CACHELEN 1000                                 // allocation size for previously seen message cache. Requires approx this # times MSGBYTES of storage
#define LINEBUFSIZ 128                                // allocation for string manipulation buffers. Likely overkill.

#define _QUIET        // Implements a message cache to suppress printout of previously seen messages
#define _CLIPABLE     // Formats captured messages so that they can be copy and pasted directly into the commandMessages array below
//#define _TRANSMIT     // Sets MCP interface into "Normal" mode and allows active. Otherwise, it will be set to LOOPBACK

#define INT_PIN 2                               // CANBUS Interface INT pin (Works well with ESP8266)
#define CS_PIN 15                               // CANBUS Interface CS pin  (Works well with ESP8266)
MCP_CAN _CANINTF(CS_PIN);                       // Declares an MCP_CAN Interface.  Argument correponds to CS pin above

 
/*
 * Miscellaneous globals
 */
unsigned int _msgIndex = 0;                     // track offset of outbound command messages sent
bool _pendingSend = false;                      // indicates whether outbound message list is exhausted
unsigned int _msgReceivedCount=0;               // count of inbound CAN messages received (for logging purposes)
unsigned int _msgSentCount=0;                   // count of outbound CAN messages [successfully] sent (for logging purposes)
unsigned int cacheSize = 0;                     // high water mark of message cache utilization
unsigned int _cycle = 0;
/*
 * Structure of each message cache entry. 
 * Currently just a concatenation of message id and arguments but expected to
 * be extended into a linked list for greater efficiency and/or augmented with
 * a counter to indicate the quantify of each distinct message received
 */
struct _msgHashNode
{
  unsigned char key[MSGKEYSIZ];              // key is a concatenation of the id (2 bytes) + argument bytes
  uint16_t count;               
};
#define MSGSIZ sizeof(_msgHashNode)

/*
 *  The following is an array of CAN messages to be transmitted.
 *  Each message consists of a 3-byte hex CAN message ID followed
 *  by up to 8 (see CANARGSLEN) 2-byte arguments.  
 *  NOTES:
 *  1. All values must must be expressed as hex codes of the form 0xXX or 0xXXX 
 *     as they will be parsed via sscanf.  
 *  2. Just leave off any un-necessary trailing parameters so that the code can count parameters correctly   
 *  3. Each invocation of loop() will transmit precisely one message until the list is exhausted. 
 *  4. The list must be terminated with a message id of 0x00!!!
 */
const char *commandMessages[] = {
/*
 * The following script will command movements of the sliding doors on a 5th gen Dodge Caravan.
 */

/*
"0x001 0x11 0x11 0x11 0x11 0x11",     // open left
"0x002 0x11 0x22 0x22 0x22 0x22",     // open left
"0x003 0x11 0x22 0x33 0x33 0x33",     // close left
"0x004 0x11 0x22 0x33 0x44 0x44",     // open right
"0x005 0x11 0x22 0x33 0x44 0x55",     // toggle both (opens left, closes right)
"0x001 0x11 0x11 0x11 0x11 0x11",     // open left
"0x002 0x11 0x22 0x22 0x22 0x22",     // open left
"0x003 0x11 0x22 0x33 0x33 0x33",     // close left
"0x003 0x11 0x22 0x33 0x33 0x33",     // close left

"0x003 0x11 0x22 0x33 0x33 0x33",     // close left

"0x004 0x11 0x22 0x33 0x44 0x44",     // open right
"0x005 0x11 0x22 0x33 0x44 0x55",     // toggle both (opens left, closes right)
/*
"0x244 0x00 0x00 0x00 0x00 0x20",     // open right
"0x244 0x00 0x00 0x00 0x00 0x30",     // close both
"0x244 0x.. 0x.. 0x.. 0x.. 0x??,"                 // TOGGLES SLIDING DOORS when unlocked. Closes open doors when locked ?? 0x10=left, 0x20=right 0x30=both 
"0x244 0x00 0x00 0x00 0x00 0x10",     // open left
"0x244 0x00 0x00 0x00 0x00 0x10",     // close left
"0x244 0x00 0x00 0x00 0x00 0x20",     // open right
"0x244 0x00 0x00 0x00 0x00 0x30",     // toggle both (opens left, closes right)
"0x244 0x00 0x00 0x00 0x00 0x20",     // open right
"0x244 0x00 0x00 0x00 0x00 0x30",     // close both
"0x244 0x.. 0x.. 0x.. 0x.. 0x??,"                 // TOGGLES SLIDING DOORS when unlocked. Closes open doors when locked ?? 0x10=left, 0x20=right 0x30=both 
"0x244 0x00 0x00 0x00 0x00 0x10",     // open left
"0x244 0x00 0x00 0x00 0x00 0x10",     // close left
"0x244 0x00 0x00 0x00 0x00 0x20",     // open right
"0x244 0x00 0x00 0x00 0x00 0x30",     // toggle both (opens left, closes right)
"0x244 0x00 0x00 0x00 0x00 0x20",     // open right
"0x244 0x00 0x00 0x00 0x00 0x30",     // close both

*/
"0x00"                                            // END OF SCRIPT SENTINEL -- DO NOT REMOVE!                           
};


/*
 * Observations on a 2014 Chrysler Town & Country
 */

/*
 * Power Mode Indicators  -- The vehicle emits these periodically to indicate the position of the ignition key
 * 0x20B 0x00 0x00                = key out
 * 0x20B 0x01 0x00                = key in
 * 0x20B 0x61 0x00                = key turned to ACCY
 * 0x20B 0x81 0x00                = key turned to RUN
 * 0x20B 0xA1 0x00                = key turned to START
 * See also 0x208 for "Key off - accessory delay" behaviour below.
 */

/*
 * Key Off Accessory Delay
 * 
 * The Key Off Accessory Delay is the interval during which accessories like the radio, the inverter and the windows
 * can be operated after the ignition key is turned to "off" or removed from the vehicle
 * The steering wheel menu (EVIC) allows this to be set to "none", 45 seconds, 5 minutes or 10 minutes.
 * It is possible to override this setting and choose another value via the CAN Bus.  Furthermore, this override appears to survive
 * vehicle power cycling. Going back through the menu forces it back to a good value.
 * NOTES: 
 *   1) Power to accessories and windows will stop if one of the front doors is opened and the key 
 *      needs to be inserted and turned to ACCY or RUN to reset the timer.
 *   2) The command appears to require at sequence of three messages. I suspect these are two different but related   
 *      commands but all appear to be necessary and the order appears to be important.
 * The sequence to change the delay appears to be:
    "0x205 0x10 0x03 0x??",                       // arg3 is the number of 15 second intervals.  
                                                  // e.g. 0x30 = 48*15 = 720 seconds = 12 minutes48*15 = 720 seconds
    "0x208 0x00 0xA2 0x64 0x1E 0x00 0x?? 0x??",   // arg 6/7 is the number of seconds
                                                  // e.g. arg6=0x02 arg7=D0 = 0x02D0 = 720 seconds
                                                  // Note that the vehicle sometimes changes this number slighly (e.g. to 0x02D0 -> 0x02DA)
    "0x205 0x00 0x00 0x00",                       // Apparently mandatory command.  Perhaps this is a 'commit' instruction. 
 *
 */

 
/*
 * Door Control
 * 
 * 0x244 0x.. 0x.. 0x.. 0x.. 0x?? Controls sliding doors.
 * When car is unlocked, command acts as toggle, opening closed doors and closing opened one.
 * When car is locked, command will only cause doors to close
 * Values for ??:
 *    0x10=left sliding door
 *    0x20=right sliding door 
 *    Note that ORing works. i.e. 0x30 will toggle both doors at once
//0x2B0 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF   // UNLOCKS CAR then moves toggles both sliding doors (args seem to differ from 0x244)
//0x2CF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF   // registers an "UNLOCK DOORS TO OPERATE" message - even when doors are unlocked!
//0x2D1 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF   // registers an "UNLOCK DOORS TO OPERATE" message - even when doors are unlocked!
 */

/*
 * Liftgate Control
 * The liftgate can be toggled to raise or lower using a single commend
 * The bus appears to emit status messages at the start and end of a sequence as well as a flurry of intermediate positions indicators
 *
//0x249 0x05 0x01 0x00 0xC0 0x03 0xC3             // TOGGLE liftgate (full cycle, as if triggered by remote, hazard lights flash etc) 
 * The open sequence consists of:
 * 0x239 command to initiate opening
 * "0x2C8 0x40 0x00 0x00 0x00 0x00 0x00"          // start of liftgate opening
 * "0x308 0x13 0x00 0x??"                         // series with last octed ranging from ~0x02 to 0xC7, presumably indicating position 
 * "0x2C8 0x60 0x00 0x00 0x00 0x00 0x00"          // liftgate opening complete
 * 
 *  The closing sequence consists of:
 * 0x239 command to initiate closing
 * "0x2C8 0x20 0x00 0x00 0x00 0x00 0x00"          // start of liftgate closing
 * "0x308 0x13 0x00 0x??"                         // series with last octed ranging from ~0xC7 to 0x02, presumably indicating position 
 * "0x2C8 0x80 0x00 0x00 0x00 0x00 0x00"          // liftgate closing complete
 * 
 */

/*
 * Windows Control
 * Notes:
 *  1) Only passenger side front door and sliding door windows appear on CAN Bus.
 *  2) Driver's window appears to be isolated between door buttons pad (LIN Bus) and front left door module.
 *  3) Power rear vent actuators likewise do not appear to communicate on CAN Bus
 *  4) Front passenger window command found for "full down" and "full up" but needs to be followed by a "stop" command.
 *  5) Sliding door window commands each lower or raise the designated window(s) by approx. 1 inch per message.
 *  6) Some window commands appear to be followed by an ACK but the pattern is not completely clear
 *  7) Windows will only move if vehicle power is on, including during "Key Out Accessory Delay" mode (see above)
//0x3F3 0x00 0x00 0x00 0x00                       // Stops front passenger window movement.  To open then close do multiple steps: 0x10, 0x00, 0x60 (0x00 for good measure)
//0x3F3 0x00 0x00 0x10 0x00                       // Lowers front passenger window completely  (does it require a stop???)
//0x3F3 0x00 0x00 0x60 0x00                       // Raises front passenger window completely but fails if preceded by the full open command (Power must be on)
//0x3F3 0x00 0x00 0x00 0x10                       // Lowers right sliding door window ~1"  [ACK by 0x2CF 0x00 0x48 0x00??]
//0x3F3 0x00 0x00 0x00 0x20                       // Raises right sliding door window ~1"  [ACK by 0x2CF 0x00 0x28 0x00??]
//0x3F3 0x00 0x00 0x00 0x01                       // Lowers left sliding door window ~1"  (ACK presumed but not yet found)
//0x3F3 0x00 0x00 0x00 0x02                       // Raises left sliding door window ~1"  (ACK presumed but not yet found)
 *
 */

/*
 * Other miscellaneous sequences identified
 * Note that little to no effort has been made to decipher arguments
 * and some may be unnecessary
 *
//0x2A8 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF   // Three sweeps of FRONT wiper
//0x2E5 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF   // Three sweeps of REAR wiper
//0x208 0x03 0xA2 0x64 0x1E 0x00 0x02 0x58        // Flash hazard lights once similar to commanded sliding door/tailgate movement
//0x208 0x13 0xA2 0x64 0x1E 0x00 0x02 0x58        // No apparent difference from that above but note different Arg1 value.
 *
 */



#ifdef _QUIET
 _msgHashNode _msgCache[CACHELEN];
/* 
 *  Implementation of a history cache to cut down on noise associated with 
 *  messages that have been seen previously.
 *  The current implementation is both inefficient and tacky as it
 *  merely performs a linear search. This really deserves a hashset or
 *  a sorted list + binary search!
 *  The somewhat convoluted switch() block provides convenient places to
 *  add conditions to include or exclude certain messages from the cache 
 *  and/or running log.
 */

bool cacheMessage(unsigned int rxId, unsigned char args[])
{
  bool bRetVal = false;
  switch(rxId) 
  {
    case 0xffe:
    case 0x3e6:                // 1 second heartbeat?
    bRetVal = true;         // anything under this case is always suppressed
      break;

    case 000:                 // anything under this case is always printed
    //case 0x20B:
    //case 0x3F3:
    //case 0x2CF:
    //case 0x2EB:
    case 0x205:
    case 0x208:
      bRetVal = false;
      break;

    default:

      _msgHashNode curVal;
      //curVal.key[0] = rxId; 
      //curVal.key[1] =(rxId >> 8);
      memcpy(&curVal.key[0], &rxId, 2);
      memcpy(&curVal.key[2], &args[0], CANARGSLEN);
      
      _msgHashNode *msg = _msgCache;
      for( int lcv = 0; lcv < cacheSize; lcv++)
      {
        if( (memcmp(msg, &curVal, MSGKEYSIZ)) == 0)
        {
          msg->count++;
          bRetVal = true;
          break;
        }
        msg++;
      }
      if( bRetVal == false )
      {
        // add message to cache
        msg = &_msgCache[cacheSize++];
        memcpy(msg, &curVal, MSGSIZ);
        msg->count=1;
      }
      break;
  } // end of switch
  return bRetVal;
}

#endif _QUIET

/*
 * Parses a single line from the "commands" array and constructs
 * a CAN message for transmittal
 */
unsigned long buildMessage(int *argCnt, unsigned char txBuf[], const char *msgLine)
{
  unsigned int id = 0;
  unsigned int arg;
  int argIdx = 0;
  char lineBuf[strlen(msgLine)];
  strcpy(lineBuf, msgLine);
  *argCnt = 0;

  char *tok = strtok(lineBuf, " ");
  if( tok != NULL )
  {
    sscanf(tok,"%x", &id);
    tok=strtok(NULL, " ");
    while (tok != NULL && argIdx < CANARGSLEN) {
      //Serial.println(tok);
      sscanf(tok,"%x", &arg);
      txBuf[argIdx] = arg;
      argIdx++;
      tok=strtok(NULL, " ");
    }
  }
  *argCnt = argIdx;
  return id;
}

/*
 * Reads a message from the CAN interface
 * Note that this function assumes that the availability
 * of a message has already been determined by the caller.
 */
void readMessage()
{
  long unsigned int rxId;
  unsigned char rxBuf[CANARGSLEN];
  unsigned char len = 0;
  char msgString[LINEBUFSIZ];
  memset(rxBuf, (unsigned char) 0, CANARGSLEN);                        
  _CANINTF.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  _msgReceivedCount++;
#ifdef _QUIET
  if(!cacheMessage(rxId, rxBuf))
#else
  if(true)
#endif // _QUIET
  {
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
#ifdef _CLIPABLE
      sprintf(msgString, "\"0x%.8lX", (rxId & 0x1FFFFFFF));
    else
      sprintf(msgString, "\"0x%.3lX", rxId);
#else
      sprintf(msgString, "RECV(%d) 0x%.8lX", _msgReceivedCount, (rxId & 0x1FFFFFFF));
    else
      sprintf(msgString, "RECV(%d) 0x%.3lX", _msgReceivedCount, rxId);
#endif // _CLIPABLE

    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
#ifdef _CLIPABLE
    Serial.printf("\",\n");
#else
    Serial.println(); 
#endif _CLIPABLE   
  }
  /*
  else
  {
    //Serial.println("DUPE RECEIVED");
  }
  */
}

/*
 * Physical transmit of a fully formed message through the MCP_2515 interface
 */
void transmit(unsigned int txId, int argCnt, unsigned char *txBuf)
{
  byte sndStat = _CANINTF.sendMsgBuf(txId, argCnt, txBuf);
  if(sndStat == CAN_OK){
    _msgSentCount++;
    Serial.printf("SEND(%d) 0x%.3X ", _msgIndex, txId);
    for(byte i = 0; i<argCnt; i++)
    {
        Serial.printf( " 0x%.2X",  txBuf[i]);
    }
    Serial.println();
  } else {
    Serial.printf("Error Sending Message... rc=%d\n",sndStat);
  }
}

/*
 * Build a message from an ad hoc command string
 * then transmit it
 */
void sendMessage(const char *msgString)
{
  int argCnt;
  long unsigned int txId =0;
  unsigned char txBuf[CANARGSLEN];
  txId = buildMessage(&argCnt, txBuf, msgString);
  transmit(txId, argCnt, txBuf);
}
  
/*
 * Build a CAN message from the canned message list
 * then transmit it
 */
void sendMessage(unsigned int idx)
{
  int argCnt;
  long unsigned int txId =0;
  unsigned char txBuf[CANARGSLEN];
  txId = buildMessage(&argCnt, txBuf, commandMessages[idx]);  
  if( txId == 0 )
  {
    Serial.println("No more messages to send");
    _pendingSend = false;
  }
  else
  {
    _msgIndex++;
    delay(3);
    transmit(txId, argCnt, txBuf);
  }
}

void processCmd(String cmdString)
{
  char *tok = strtok((char*)cmdString.c_str(), " :");
  char *cmdToken = NULL;
  char *argToken = NULL;
  if( tok != NULL )
  {
    // ignore first token as it is the literal "CMD";
    cmdToken=strtok(NULL, " ");
    if(cmdToken != NULL )
    {
      argToken = strtok(NULL, " ");
    }
  }
  Serial.printf("Command: %s %s\n", ((cmdToken == NULL) ? "no command given" : cmdToken), ((argToken==NULL)? "" : argToken) );

  String command = cmdToken;
  command.trim();
  command.toUpperCase();
  char lineBuf[LINEBUFSIZ];

  if(command.startsWith("KEYOUT"))
  {
    unsigned int timeoutSeconds = 1800;    // default to 30 mins
    if(argToken != NULL)
    {
      sscanf(argToken,"%d",&timeoutSeconds);
    }
    sprintf(lineBuf,"0x205 0x10 0x03 0x%.2X", (unsigned int) timeoutSeconds/15);
    sendMessage(lineBuf);
    delay(2);
    sendMessage(lineBuf);
    delay(2);
    sendMessage("0x205 0x00 0x00 0x00");
  }
  else if(command.equals("LEFTDOOR"))
  {
    sendMessage("0x244 0x00 0x00 0x00 0x00 0x10");
  }
  else if(command.equals("RIGHTDOOR"))
  {
    sendMessage("0x244 0x00 0x00 0x00 0x00 0x20");
  }
  else if(command.equals("BOTHDOORS"))
  {
    sendMessage("0x244 0x00 0x00 0x00 0x00 0x30");
  }
  else if(command.equals("LIFTGATE") || command.equals("TAILGATE"))
  {
    sendMessage("0x249 0x05 0x01 0x00 0xC0 0x03 0xC3");
  }
  else if(command.equals("FRONTWINDOWDOWN"))
  {
    sendMessage("0x3F3 0x00 0x00 0x10 0x00");
    delay(1000);
    sendMessage("0x3F3 0x00 0x00 0x00 0x00");  
  }
  else if(command.equals("FRONTWINDOWUP"))
  {
    sendMessage("0x3F3 0x00 0x00 0x60 0x00");
    delay(1000);
    sendMessage("0x3F3 0x00 0x00 0x00 0x00");   }
  else if(command.equals("RIGHTSLIDERDOWN"))
  {
    for(int lcv = 0; lcv < 12; lcv++)
    {
      sendMessage("0x3F3 0x00 0x00 0x00 0x10"); 
      delay(3);
    }
  }
  else if(command.equals("RIGHTSLIDERUP"))
 {
    for(int lcv = 0; lcv < 12; lcv++)
    {
      sendMessage("0x3F3 0x00 0x00 0x00 0x20"); 
      delay(3);
    }
  }
  else if(command.equals("RIGHTSLIDERDOWN"))
 {
    for(int lcv = 0; lcv < 12; lcv++)
    {
      sendMessage("0x3F3 0x00 0x00 0x00 0x01"); 
      delay(3);
    }
  }
  else if(command.equals("LEFTSLIDERUP"))
 {
    for(int lcv = 0; lcv < 12; lcv++)
    {
      sendMessage("0x3F3 0x00 0x00 0x00 0x02"); 
      delay(3);
    }
  }
  else if(command.equals("FRONTWIPER"))
  {
    sendMessage("0x2A8 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF");
  }
  else if(command.equals("REARWIPER"))
  {
    sendMessage("0x2E5 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF");
  }
  else
  {
    Serial.printf("Unknown command %s %s\nUsage:\n", ((cmdToken == NULL) ? "no command given" : cmdToken), ((argToken==NULL)? "" : argToken) );
    Serial.println("KEYOUT [seconds]");
    Serial.println("LEFTDOOR");
    Serial.println("RIGHTDOOR");
    Serial.println("BOTHDOORS");
    Serial.println("LIFTGATE");
    Serial.println("FRONTWINDOWDOWN");
    Serial.println("FRONTWINDOWUP");
    Serial.println("RIGHTSLIDERDOWN");
    Serial.println("RIGHTSLIDERUP");
    Serial.println("RIGHTSLIDERDOWN");
    Serial.println("LEFTSLIDERUP");
    Serial.println("FRONTWIPER");
    Serial.println("REARWIPER");
  }
}

void cmdStep()
{
  Serial.println("Run one step");
  if(_cycle > 0 )
  {
    char msgString[LINEBUFSIZ];
    sprintf(msgString, "0x%.3X 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF", _cycle++);
    Serial.printf("Transmitting: %s\n", msgString);
    sendMessage(msgString);
  }
  else
  {
    sendMessage(_msgIndex);  
  }
  _pendingSend = false;
}

void runScript()
{
  Serial.println("Will play static message list");
  _msgIndex = 0;                      // reset static message list pointer
  _pendingSend = true;                // allows free running of script
}

void restartScript()
{
  Serial.println("Resetting static message list pointer");
  _msgIndex = 0;                      // reset static message list pointer
  _pendingSend = false;               // prevents free running of script
}

void startCycle(String s)
{
  if( s.length() > 5 )
    sscanf(s.substring(6).c_str(),"%x",&_cycle);
  if(_cycle == 0)
    _cycle = 0x200;        
  Serial.printf("Cycle [re]starting at 0x%.3x", _cycle);
}

void clearCache()
{
#ifdef _QUIET
  Serial.println("Clearing message cache");
  cacheSize = 0;
  _msgReceivedCount=0;                        
  _msgSentCount=0; 
#endif _QUIET    
}


void dumpStats()
{
  Serial.printf("\n%d received\n%d sent\n%d cached\n", _msgReceivedCount, _msgSentCount, cacheSize );  
}

void dumpCache()
{
  _msgHashNode *curMsg = (_msgHashNode*) _msgCache;
  uint16_t id;

  Serial.print("msgId  ");
  for(int i = 0; i < CANARGSLEN; i++)
    Serial.printf("Arg%1d ", i+1 );
  Serial.println("Count");
  Serial.print("=======");
  for(int i = 0; i < CANARGSLEN; i++)
    Serial.printf("=====", i+1 );
  Serial.println("=====");

  for( int lcv = 0; lcv < cacheSize; lcv++)
  {
    memcpy(&id, curMsg, 2);
    Serial.printf("0x%.3X ", id);
    for(int i = 0; i < CANARGSLEN; i++)
      Serial.printf("0x%.2X ", curMsg->key[i+2]);
    Serial.printf("%5d\n", curMsg->count);
    curMsg++;
    
  }
  dumpStats();
}

void usage()
{
  Serial.println("Usage:");
  Serial.println("Empty command -> Send next static message in normal mode or next sequential command in \"CYCLE\" mode");
  Serial.println("RUN           -> Send all pending messages from the current point in the list");
  Serial.println("RESTART       -> Reset static list to first message");
  Serial.println("CLEAR         -> Clear captured message cache (if enabled at compile time)");
  Serial.println("STATS         -> Report counts of messages received, sent and cached");
  Serial.println("DUMP          -> Display cache of received messages with counts");  
  Serial.println("CMD argument] -> Send a named message sequences. (no argument will list commands available"); 
  Serial.println("CYCLE [0x???] -> Empty \"sends\" will step through commands ids to the bus in sequence");
  Serial.println("                   Commands will be followed by eight 0xFF parameters" );
  Serial.println("                   If optional argument is not specified, sequence will begin with 0x200");
  Serial.println("You may also enter a line of the form \"0x?? [0x?? 0x?? ... 0x??]\" with up to 8 parameters");
}

/*
 * Read Serial interface for commands
 */
void readCommand()
{
  String s = Serial.readString();
  s.trim();
  s.toUpperCase();
  int len = s.length();
  if( len == 0 )                    cmdStep();
  else if( s.startsWith("CMD"))     processCmd(s);
  else if( s.equals("RUN"))         runScript();
  else if( s.equals("RESTART"))     restartScript();
  else if( s.equals("CLEAR"))       clearCache();
  else if( s.equals("STATS"))       dumpStats();
  else if( s.equals("DUMP"))        dumpCache();
  else if( s.startsWith("CYCLE"))   startCycle(s);
  else if(len >=5 && len <= 45 )       // minimal sanity check on a message of the form "0xFFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF"
  {
    Serial.println("Will attempt to form and send ad hoc message");
    sendMessage(s.c_str());
  }
  else
  {
    Serial.printf("Invalid command: %s\n", s);
    usage();
  }
}

void setup()
{
  Serial.begin(500000);

  // Initialize MCP2515 interface
  Serial.println("About to initialize MCP2515 Library Script Test...");
  delay(1000);
  if(_CANINTF.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)   // CANBus for 2014 Chrysler T&C
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

#ifdef _TRANSMIT  
  _CANINTF.setMode(MCP_NORMAL) ;          // Set operation mode to normal so the MCP2515 sends ACKs to received data.
#else
  _CANINTF.setMode(MCP_LOOPBACK) ;        // Messages sent to the MCP2515 are re-received by the device without actually being sent on the CAN bus
#endif _TRANSMIT

  pinMode(INT_PIN, INPUT);                // Configuring pin for /INT input

  Serial.println("Remember to \"Send\" to commence message transmittal");
  Serial.flush();
}

void loop()
{

  // receive an inbound message, if one is available
  if(!digitalRead(INT_PIN))                         // If CAN Interface's INT_PIN pin is low, read receive buffer
  {
    readMessage();
  }

  // send an outbound message, if any are pending.  
  /*  Note that at most one message will be sent per loop(). This is a crude throttling mechanism
   *  that seems to do a good job of pacing outbound messages and avoiding buffer exhaustion on 
   *  the MCP2515 in cases of long script sequences when the bus is already very active.
   */
  if(_pendingSend)
  {
    sendMessage(_msgIndex);
  }

  // check for an ad hoc command
  if (Serial.available() > 0)
  {
    readCommand();
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
