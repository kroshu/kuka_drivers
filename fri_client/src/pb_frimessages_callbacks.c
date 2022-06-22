/**

 The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software "KUKA Sunrise.Connectivity FRI Client SDK" is targeted to work in
conjunction with the "KUKA Sunrise.Connectivity FastRobotInterface" toolkit.
In the following, the term "software" refers to all material directly
belonging to the provided SDK "Software development kit", particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2019 
KUKA Roboter GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.




 \file
 \version {1.15}
 */
#include <stdio.h>
#include <stdlib.h> 

#include <pb_frimessages_callbacks.h>
#include <nanopb/pb_encode.h>
#include <nanopb/pb_decode.h>

bool encode_repeatedDouble(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
   size_t i = 0;

   tRepeatedDoubleArguments* arguments = 0;
   size_t count = 0;
   double* values = 0;

   if (arg == NULL || *arg == NULL)
   {
      return false;
   }
   arguments = ((tRepeatedDoubleArguments*) (*arg));

   count = arguments->max_size;
   values = arguments->value;

   for (i = 0; i < count; i++)
   {

      if (!pb_encode_tag_for_field(stream, field))
      {
         return false;
      }

      if (!pb_encode_fixed64(stream, &values[i]))
      {
         return false;
      }
   }
   return true;
}

bool decode_repeatedDouble(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
   tRepeatedDoubleArguments* arguments = 0;
   size_t i = 0;
   double* values = 0;

   if (arg == NULL || *arg == NULL)
   {
      return false;
   }

   arguments = (tRepeatedDoubleArguments*) *arg;
   i = arguments->size;
   values = arguments->value;

   if (values == NULL)
   {
      return true;
   }

   if (!pb_decode_fixed64(stream, &values[i]))
   {
      return false;
   }

   arguments->size++;
   if (arguments->size >= arguments->max_size)
   {
      arguments->size = 0;
   }
   return true;
}

bool encode_repeatedInt(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
   int i = 0;
   tRepeatedIntArguments* arguments = 0;
   int count = 0;
   int64_t* values = 0;

   if (arg == NULL || *arg == NULL)
   {
      return false;
   }
   arguments = (tRepeatedIntArguments*) *arg;
   count = arguments->max_size;
   values = arguments->value;
   for (i = 0; i < count; i++)
   {

      if (!pb_encode_tag_for_field(stream, field))
      {
         return false;
      }
      if (!pb_encode_varint(stream, values[i]))
      {
         return false;
      }
   }
   return true;
}

bool decode_repeatedInt(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
   tRepeatedIntArguments* arguments = 0;
   size_t i = 0;
   uint64_t* values = 0;

   if (arg == NULL || *arg == NULL)
   {
      return false;
   }
   arguments = (tRepeatedIntArguments*) *arg;

   i = arguments->size;
   values = (uint64_t*) arguments->value;
   if (values == NULL)
   {
      return true;
   }

   if (!pb_decode_varint(stream, &values[i]))
   {
      return false;
   }

   arguments->size++;
   if (arguments->size >= arguments->max_size)
   {
      arguments->size = 0;
   }
   return true;
}

void map_repeatedDouble(eNanopbCallbackDirection dir, int numDOF, pb_callback_t *values, tRepeatedDoubleArguments *arg)
{
   // IMPORTANT: the callbacks are stored in a union, therefor a message object
   // must be exclusive defined for transmission or reception
   if (dir == FRI_MANAGER_NANOPB_ENCODE)
   {
      values->funcs.encode = &encode_repeatedDouble;
   }
   else
   {
      values->funcs.decode = &decode_repeatedDouble;
   }
   // map the local container data to the message data fields
   arg->max_size = numDOF;
   arg->size = 0;
   if (numDOF > 0)
   {
      arg->value = (double*) malloc(numDOF * sizeof(double));

   }
   values->arg = arg;
}

void map_repeatedInt(eNanopbCallbackDirection dir, int numDOF, pb_callback_t *values, tRepeatedIntArguments *arg)
{
   // IMPORTANT: the callbacks are stored in a union, therefor a message object
   // must be exclusive defined for transmission or reception
   if (dir == FRI_MANAGER_NANOPB_ENCODE)
   {
      // set the encode callback function
      values->funcs.encode = &encode_repeatedInt;
   }
   else
   {
      // set the decode callback function
      values->funcs.decode = &decode_repeatedInt;
   }
   // map the robot drive state from the container to message field
   arg->max_size = numDOF;
   arg->size = 0;
   if (numDOF > 0)
   {
      arg->value = (int64_t*) malloc(numDOF * sizeof(int64_t));

   }
   values->arg = arg;
}

void init_repeatedDouble(tRepeatedDoubleArguments *arg)
{
   arg->size = 0;
   arg->max_size = 0;
   arg->value = NULL;
}

void init_repeatedInt(tRepeatedIntArguments *arg)
{
   arg->size = 0;
   arg->max_size = 0;
   arg->value = NULL;
}

void free_repeatedDouble(tRepeatedDoubleArguments *arg)
{
   if (arg->value != NULL)
      free(arg->value);
}

void free_repeatedInt(tRepeatedIntArguments *arg)
{
   if (arg->value != NULL)
      free(arg->value);
}
