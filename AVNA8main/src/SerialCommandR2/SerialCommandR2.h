/* ****************************************************************************** 
SerialCommandR is a modification of SerialCommand
1 - Sets default end of command, term, to '#'
2 - Makes term programable with method .setTerm(char t).
3 - Always allows comma and space as token separators
4 - Removes leading spaces and term characters on commands
5 - Added provision for response, set by .setResonnse(uint16_t resp)
6 - Changed diagnostic character echo to Hex to see everything.
7 - Increased buffer size to 50 for long commands
Bob Larkin  21 Jan 2017
* Added processCh(uint8_t c) to allow external character inputs
*       RSL 25 Jan 2020
* 
SerialCommand - An Arduino library to tokenize and parse commands received over
a serial port. 
Copyright (C) 2011-2013 Steven Cogswell  <steven.cogswell@gmail.com>
http://awtfy.com

Version 20131021A.   

Version History:
May 11 2011 - Initial version
May 13 2011 -   Prevent overwriting bounds of SerialCommandCallback[] array in addCommand()
            defaultHandler() for non-matching commands
Mar 2012 - Some const char * changes to make compiler happier about deprecated warnings.  
           Arduino 1.0 compatibility (Arduino.h header) 
Oct 2013 - SerialCommand object can be created using a SoftwareSerial object, for SoftwareSerial
           support.  Requires #include <SoftwareSerial.h> in your sketch even if you don't use 
           a SoftwareSerial port in the project.  sigh.   See Example Sketch for usage. 
Oct 2013 - Conditional compilation for the SoftwareSerial support, in case you really, really
           hate it and want it removed.  

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
***********************************************************************************/
#ifndef SerialCommandR2_h
#define SerialCommandR2_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// If you want to use SerialCommand with the hardware serial port only, and want to disable
// SoftwareSerial support, and thus don't have to use "#include <SoftwareSerial.h>" in your
// sketches, then uncomment this define for SERIALCOMMAND_HARDWAREONLY, and comment out the 
// corresponding #undef line.  
//
// You don't have to use SoftwareSerial features if this is not defined, you can still only use 
// the Hardware serial port, just that this way lets you get out of having to include 
// the SoftwareSerial.h header. 
#define SERIALCOMMAND_HARDWAREONLY 1
//#undef SERIALCOMMAND_HARDWAREONLY

#ifdef SERIALCOMMAND_HARDWAREONLY
//   #warning "Warning: Building SerialCommand without SoftwareSerial Support"
#endif

#ifndef SERIALCOMMAND_HARDWAREONLY 
#include <SoftwareSerial.h>  
#endif

#include <string.h>


#define SERIALCOMMANDBUFFER 50
// Was 10 RSL, new value is fine for Teensy, but excessive for most Arduino memory sizes:
#define MAXSERIALCOMMANDS   100
// Was 2  RSL:
#define MAXDELIMETER 5
// Was '#'
#define DEFAULT_TERM '\r'

#define SERIALCOMMANDDEBUG 1
#undef SERIALCOMMANDDEBUG      // Comment this out to run the library in debug mode (verbose messages)

// Response variations.  ECHO_FULL_COMMAND replies with full received line, even if command is not valid.
// ECHO_COMMAND ((first token only) and ECHO_OK only respond if command is valid.  With EOL.
// ECHO_ONE responds with numerical 1 if command is valid and numerical 0 if invalid.  No EOL.
#define NO_RESPONSE 0
#define ECHO_COMMAND 1
#define ECHO_FULL_COMMAND 2
#define ECHO_OK  3
#define ECHO_ONE 4

class SerialCommand
{
    public:
        SerialCommand();      // Constructor
        #ifndef SERIALCOMMAND_HARDWAREONLY
        SerialCommand(SoftwareSerial &SoftSer);  // Constructor for using SoftwareSerial objects
        #endif

        void clearBuffer();   // Sets the command buffer to all '\0' (nulls)
        char *next();         // returns pointer to next token found in command buffer (for getting arguments to commands)
        void readSerial();    // Main entry point.
        int16_t processCh(char);  // Alternate for readSerial() whith character already read  Rev Jan 2020
        //      processCh(char)  returns 0=still filling buffer,  1=invalid command found  2=valid command found
        void setTerm(char);   // Command termination character, like '#'
        void setResponse(uint16_t);  // Response, if desired.  default:NO_RESPONSE
        void addCommand(const char *, void(*)());   // Add commands to processing dictionary
        void addDefaultHandler(void (*function)());    // A handler to call when no valid command received. 
    
    private:
        char inChar;                        // A character read from the serial stream 
        char buffer[SERIALCOMMANDBUFFER];   // Buffer of stored characters while waiting for terminator character
        int16_t  bufPos;                        // Current position in the buffer
        char delim[MAXDELIMETER+1];         // null-terminated list of character to be used as delimeters for tokenizing (default " ")
        char term;                          // Character that signals end of command, as a string
        uint16_t response;
        char *token;                        // Returned token from the command buffer as returned by strtok_r
        char *last;                         // State variable used by strtok_r during processing
        typedef struct _callback {
            char command[SERIALCOMMANDBUFFER];
            void (*function)();
        } SerialCommandCallback;            // Data structure to hold Command/Handler function key-value pairs
        int16_t numCommand;
        SerialCommandCallback CommandList[MAXSERIALCOMMANDS];   // Actual definition for command/handler array
        void (*defaultHandler)();           // Pointer to the default handler function 
        bool usingSoftwareSerial;           // A boolean to see if we're using SoftwareSerial object or not
        #ifndef SERIALCOMMAND_HARDWAREONLY 
        SoftwareSerial *SoftSerial;         // Pointer to a user-created SoftwareSerial object
        #endif
};

#endif //SerialCommand_h
