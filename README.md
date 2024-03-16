Arduino sketch which reads from and writes to a CAN Bus using an MCP2515 interface

This sketch depends upon the the MCP_CAN_lib (https://github.com/Longan-Labs/Arduino_CAN_BUS_MCP2515)
 
By default, the sketch's loop simply checks to see if there is a message waiting to be read from the interface and prints it, if there is one. 

The sketch goes further in several ways:
 
1. Individual CAN messages can be easily composed on the inbound Serial stream and sent out via CAN. If using the Arduino IDE, you can simply type
   a command of the form 0xXXX (with up to 8 additional 0xXX arguments) and hit 'send'.
  
2. Longer sequences of CAN messages can be composed in a static array and, once compiled/uploaded, can be sent out via CAN on demand. By default, sending
   an empty string to Serial will transmit a single message (the next in the sequence) on the CAN Bus. Sending the string "RUN" on Serial will result in
   all further messages in the sequence being sent in rapid succession (one per pass through loop() ).  See the usage() and runCommand() functions for
   bells and whistles.
   
3. An enhanced read mode can be used to suppress printing of any messages that have been seen previously. This is exceptionally useful
   for reverse engineering an existing CAN installation.  For example, to narrow down the message ids associated with a certain function, one
   can begin a capture through the sketch and prime it with all the chatter associated with the CAN Bus at startup, clear the output log, then
   initiate the function under investigation. The resulting message log will then show the messages triggered by the function with decent suppression 
   of much of the normal background chatter on an active bus.  Note that the message history cache can be reset by sending the string "CLEAR" to
   the Serial stream.

The code and its operation is best understood by reading the extensive comments in the sketch. The CANBus commands embedded in the sketch are specific 
to one particular vehicle and are provided for illustration only.

*** NOTE: NAIVELY INTERACTING WITH YOUR VEHICLE'S CANBUS IS AN INHERENTLY RISKY ACTIVITY THAT COULD RESULT IN MATERIAL HARM TO YOUR VEHICLE. YOUR DECISION
TO DO SO, WITH OR WITHOUT THIS CODE, IS ENTIRELY YOUR OWN AND NO WARRANTY, RESPONSIBITY OR COMMITTMENT TO SUPPORT IS IMPLIED.  
