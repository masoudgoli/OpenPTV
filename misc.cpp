/* Copyright (c) 2011, G. Andrew Stone, Toasted Circuits, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Neither the name of Toasted Circuits, Inc. nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <Arduino.h>
#include "misc.h"

void pgm_ps(const prog_char *pfmt, ... )
{        
  char fmt[81]; // resulting string limited to 80 chars
  char tmp[81];
  for (byte b=0;b<80;b++)
    {
      fmt[b] = pgm_read_byte(pfmt+b);
      if (fmt[b] == 0) break;
    }
  fmt[80] = 0;  // terminate it just in case overrun

  va_list args;
  va_start (args, pfmt );
  vsnprintf(tmp, 81, fmt, args);
  va_end (args);
  tmp[80] = 0;
//  Serial.print(tmp);
}

//?? Printf-style to the serial
void ps(const char *fmt, ... )
{
        char tmp[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, 128, fmt, args);
        va_end (args);
//        Serial.print(tmp);
}
