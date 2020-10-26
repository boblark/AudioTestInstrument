/******************************************************************************* 
SerialCommandR  -  Modified from Steven Cogswell Llibrary noted below.
Bob Larkin 21 Jan 2017 - See .h file for notes.
* 
SerialCommand - An Arduino library to tokenize and parse commands received over
a serial port. 
Copyright (C) 2011-2013 Steven Cogswell  <steven.cogswell@gmail.com>
http://awtfy.com

See SerialCommand.h for version history. 

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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h" 
#endif

#include "SerialCommandR2.h"

#include <string.h>
#ifndef SERIALCOMMAND_HARDWAREONLY
#include <SoftwareSerial.h>
#endif

// Constructor makes sure some things are set.    Untested by RSL
SerialCommand::SerialCommand()
    {
    usingSoftwareSerial=false;
    // term was '\r'  but that was not received from Serial Monitor---Why?
    term = DEFAULT_TERM;       // '#' return character, default terminator for command, programmable
    delim[0]= DEFAULT_TERM;    // for finding tokens
    delim[1] = ' ';            // Space always available
    delim[2] = ',';            // Comma
    delim[3] = '\n';           // newline doesn't appear for me, but here for safety
    delim[4] = '\r';           // same for CR
    response = NO_RESPONSE;
    numCommand=0;              // Number of callback handlers installed
    clearBuffer(); 
    }

#ifndef SERIALCOMMAND_HARDWAREONLY
// Constructor to use a SoftwareSerial object
SerialCommand::SerialCommand(SoftwareSerial &_SoftSer)
    {
    usingSoftwareSerial=true; 
    SoftSerial = &_SoftSer;
    strncpy(delim," ",MAXDELIMETER);  // strtok_r needs a null-terminated string
    term = DEFAULT_TERM;       // '#' return character, default terminator for command, programmable
    delim[0]= DEFAULT_TERM;    // for finding tokens
    delim[1] = ' ';            // Space always available
    delim[2] = ',';            // Comma
    delim[3] = '\n';           // newline doesn't appear for me, but here for safety
    delim[4] = '\r';           // same for CR
    numCommand=0;              // Number of callback handlers installed
    clearBuffer(); 
    }
#endif

// Set command termination character, term, to over-ride DEFAULT_TERM
void SerialCommand::setTerm(char t)
    {
    term = t;          // Change terminator character
    delim[0] = t;      // All endings should also be deliminators
    }

// Set command response, response, to over-ride NO_RESPONSE
void SerialCommand::setResponse(uint16_t r)
    {
    response = r;          // Change serial response to commands 
    }
//
// Initialize the command buffer being processed to all null characters
//
void SerialCommand::clearBuffer()
    {
    for (int i=0; i<SERIALCOMMANDBUFFER; i++) 
        buffer[i]='\0';
    bufPos=0; 
    }

// Retrieve the next token ("word" or "argument") from the Command buffer.  
// returns a NULL if no more tokens exist.   
char *SerialCommand::next() 
    {
    char *nextToken;
    nextToken = strtok_r(NULL, delim, &last); 
    return nextToken; 
    }

// This checks the Serial stream for characters, and assembles them into a buffer.  
// When the terminator character (default '\r') is seen, it starts parsing the 
// buffer for a prefix command, and calls handlers setup by addCommand() member
void SerialCommand::readSerial() 
    {
    // If we're using the Hardware port, check it.   Otherwise check the user-created SoftwareSerial Port
    #ifdef SERIALCOMMAND_HARDWAREONLY
    while (Serial.available() > 0) 
    #else
    while ((usingSoftwareSerial && Serial.available() > 0) || (usingSoftwareSerial && SoftSerial->available() > 0) )
    #endif
        {
        if (!usingSoftwareSerial)
            {
            // Hardware serial port
            inChar=Serial.read();   // Read single available character, there may be more waiting
            }
        else
            {
            #ifndef SERIALCOMMAND_HARDWAREONLY
            // SoftwareSerial port
            inChar = SoftSerial->read();   // Read single available character, there may be more waiting
            #endif
            }
        processCh(inChar);
	    }
    }


// Alternate for readSerial() whith character already read  Rev Jan 2020
// This checks the Serial stream for characters, and assembles them into a buffer.  
// When the terminator character (default '\r') is seen, it starts parsing the 
// buffer for a prefix command, and calls handlers setup by addCommand() member.
// Returns 0=still filling buffer,  1=invalid command found  2=valid command found
int16_t SerialCommand::processCh(char ccc)
    {
		char *c;
		int16_t returnValue;
		uint16_t i; 
        boolean matched; 
        inChar = ccc;                  // <<<WOULD SEEM THAT inChar could be local

        returnValue = 0;
        #ifdef SERIALCOMMANDDEBUG
        // Echo back to serial stream
        Serial.print("0x");
        if ((uint8_t)inChar<0x10) {Serial.print("0");}
        Serial.print((uint8_t)inChar, HEX);
        Serial.print("  term=");

        Serial.print("0x");
        Serial.print((uint8_t)term, HEX);
        Serial.println("");
        #endif

        if (inChar==term)
            {     // Check for the terminator, meaning end of command

            #ifdef SERIALCOMMANDDEBUG
            Serial.print("Received: "); 
            Serial.println(buffer);
            #endif

            if (response==ECHO_FULL_COMMAND)
                Serial.println(buffer);
            // Look for any leading spaces or blank commands
            c = buffer;
            bufPos=0;           // Reset to start of buffer
            while (*c==' ' || *c==term || *c=='\n' || *c=='\r')
                 {
                 c++;
                 bufPos++;
                 }
            token = strtok_r(buffer,delim,&last);   // Search for command at start of buffer
            if (token == NULL) return 0;            // End of command delimiter not found yet
            matched=false; 
            for (i=0; i<numCommand; i++)            // Try all commands
                {

                #ifdef SERIALCOMMANDDEBUG
                Serial.print("Comparing ["); 
                Serial.print(token); 
                Serial.print("] to [");
                Serial.print(CommandList[i].command);
                Serial.println("]");
                #endif
                
                // Compare the found command against the list of known commands for a match
                if (strncmp(token,CommandList[i].command,SERIALCOMMANDBUFFER) == 0) 
                    {
                    #ifdef SERIALCOMMANDDEBUG
                    Serial.print("Matched Command: "); 
                    Serial.println(token);
                    #endif

                    returnValue = 2;
                    if(response==ECHO_COMMAND)
                        Serial.println(token);
                    else if (response==ECHO_OK)
                        Serial.println("OK");
                    else if (response==ECHO_ONE)
                        Serial.print(1);
                    // Execute the stored handler function for the command
                    (*CommandList[i].function)(); 
                    clearBuffer(); 
                    matched=true; 
                    break; 
                    }
                }
            if (matched==false)
                {
                returnValue = 1;
                (*defaultHandler)(); 
                clearBuffer(); 
                if (response==ECHO_ONE)
                        Serial.print(0);
                }
            }
        // printable includes space, tab, \n, \r, etc.  Excludes control ch.
        if (isprint(inChar))   // Only "printable" characters into the buffer
            {
            buffer[bufPos++]=inChar;   // Put character into buffer
            buffer[bufPos]='\0';  // Null terminate
            if (bufPos > SERIALCOMMANDBUFFER-1) bufPos=0; // wrap buffer around if full  
            }
        return returnValue;
        }
 




// Adds a "command" and a handler function to the list of available commands.  
// This is used for matching a found token in the buffer, and gives the pointer
// to the handler function to deal with it. 
void SerialCommand::addCommand(const char *command, void (*function)())
{
    if (numCommand < MAXSERIALCOMMANDS) {
        #ifdef SERIALCOMMANDDEBUG
        Serial.print(numCommand); 
        Serial.print("-"); 
        Serial.print("Adding command for "); 
        Serial.println(command); 
        #endif
        
        strncpy(CommandList[numCommand].command,command,SERIALCOMMANDBUFFER); 
        CommandList[numCommand].function = function; 
        numCommand++; 
    } else {
        // In this case, you tried to push more commands into the buffer than it is compiled to hold.  
        // Not much we can do since there is no real visible error assertion, we just ignore adding
        // the command
        #ifdef SERIALCOMMANDDEBUG
        Serial.println("Too many handlers - recompile changing MAXSERIALCOMMANDS"); 
        #endif 
    }
}

// This sets up a handler to be called in the event that the receveived command string
// isn't in the list of things with handlers.
void SerialCommand::addDefaultHandler(void (*function)())
{
    defaultHandler = function;
}
