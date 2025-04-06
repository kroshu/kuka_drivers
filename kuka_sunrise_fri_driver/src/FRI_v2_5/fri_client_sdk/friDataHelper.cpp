/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.FRI� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2021
KUKA Deutschland GmbH
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
\version {2.5}
*/

#include <cmath>

#include <fri_client_sdk/friDataHelper.h>

namespace KUKA
{
   namespace FRI
   {
      void DataHelper::convertTrafoMatrixToQuaternion(const double (&matrixTrafo)[3][4],
         double (&quaternionTrafo)[7])
      {

         double s = 1.0 + matrixTrafo[0][0] + matrixTrafo[1][1] + matrixTrafo[2][2];  // = 4 w^2
         const double epsilon = 1e-11;

         quaternionTrafo[QUAT_TX] = matrixTrafo[0][3];
         quaternionTrafo[QUAT_TY] = matrixTrafo[1][3];
         quaternionTrafo[QUAT_TZ] = matrixTrafo[2][3];

         if (s > epsilon)
         {
            quaternionTrafo[QUAT_QW] = 0.5 * sqrt(s); // > 0.5 * sqrt(epsilon) = 0.5 * 1e-6
            s = 0.25 / quaternionTrafo[QUAT_QW]; // = 1/(4w)
            quaternionTrafo[QUAT_QX] = (matrixTrafo[2][1] - matrixTrafo[1][2]) * s; // 4wx/(4w)
            quaternionTrafo[QUAT_QY] = (matrixTrafo[0][2] - matrixTrafo[2][0]) * s; // 4wy/(4w)
            quaternionTrafo[QUAT_QZ] = (matrixTrafo[1][0] - matrixTrafo[0][1]) * s; // 4wz/(4w)
         }
         else
         {
            // w is very small (or even vanishing)
            if ((matrixTrafo[0][0] > matrixTrafo[1][1]) && (matrixTrafo[0][0] > matrixTrafo[2][2]))
            {
               // => |x| = max{|x|,|y|,|z|}   (and |x| > |w|);   choose x > 0
               quaternionTrafo[QUAT_QX] = 0.5 * sqrt(1.0 + matrixTrafo[0][0] - matrixTrafo[1][1] - matrixTrafo[2][2]);
               s = 0.25 / quaternionTrafo[QUAT_QX]; // 1/(4x)
               quaternionTrafo[QUAT_QW] = (matrixTrafo[2][1] - matrixTrafo[1][2]) * s;   // 4wx/(4x)
               quaternionTrafo[QUAT_QY] = (matrixTrafo[0][1] + matrixTrafo[1][0]) * s;   // 4yx/(4x)
               quaternionTrafo[QUAT_QZ] = (matrixTrafo[0][2] + matrixTrafo[2][0]) * s;   // 4zx/(4x)
            }
            else if (matrixTrafo[1][1] > matrixTrafo[2][2])
            {
               // => |y| = max{|x|,|y|,|z|}   (and |y| > |w|);   choose y > 0
               quaternionTrafo[QUAT_QY] = 0.5 * sqrt(1.0 + matrixTrafo[1][1] - matrixTrafo[0][0] - matrixTrafo[2][2]);
               s = 0.25 / quaternionTrafo[QUAT_QY]; // 1/(4y)
               quaternionTrafo[QUAT_QW] = (matrixTrafo[0][2] - matrixTrafo[2][0]) * s;   // 4wy/(4y)
               quaternionTrafo[QUAT_QX] = (matrixTrafo[0][1] + matrixTrafo[1][0]) * s;   // 4xy/(4y)
               quaternionTrafo[QUAT_QZ] = (matrixTrafo[1][2] + matrixTrafo[2][1]) * s;   // 4zy/(4y)
            }
            else
            {
               // => |z| = max{|x|,|y|,|z|}   (and |z| > |w|);   choose z > 0
               quaternionTrafo[QUAT_QZ] = 0.5 * sqrt(1.0 + matrixTrafo[2][2] - matrixTrafo[0][0] - matrixTrafo[1][1]);
               s = 0.25 / quaternionTrafo[QUAT_QZ]; // 1/(4z)
               quaternionTrafo[QUAT_QW] = (matrixTrafo[1][0] - matrixTrafo[0][1]) * s;   // 4wz/(4z)
               quaternionTrafo[QUAT_QX] = (matrixTrafo[0][2] + matrixTrafo[2][0]) * s;   // 4xz/(4z)
               quaternionTrafo[QUAT_QY] = (matrixTrafo[1][2] + matrixTrafo[2][1]) * s;   // 4yz/(4z)
            }
         }

         // normalize result to ensure that we obtain a unit quaternion
         // (should be superfluous but in case of numerical problems ...)
         const double rotProduct =
            quaternionTrafo[QUAT_QX] * quaternionTrafo[QUAT_QX] +
            quaternionTrafo[QUAT_QY] * quaternionTrafo[QUAT_QY] +
            quaternionTrafo[QUAT_QZ] * quaternionTrafo[QUAT_QZ];

         const double norm = sqrt(quaternionTrafo[QUAT_QW] * quaternionTrafo[QUAT_QW] + rotProduct);

         // normalize to unit length, to obtain an orientation representing quaternion
         if (norm > epsilon)
         {
            quaternionTrafo[QUAT_QW] /= norm;
            quaternionTrafo[QUAT_QX] /= norm;
            quaternionTrafo[QUAT_QY] /= norm;
            quaternionTrafo[QUAT_QZ] /= norm;
         }
         else
         {
            // input has vanishing length and is thus far away from a reasonable,
            // orientation representing quaternion :-(
            // generally normalize vanishing quaternions to the identity
            quaternionTrafo[QUAT_QW] = 1.0;
            quaternionTrafo[QUAT_QX] = 0;
            quaternionTrafo[QUAT_QY] = 0;
            quaternionTrafo[QUAT_QZ] = 0;
         }

      }

      void DataHelper::convertTrafoQuaternionToMatrix(const double(&quaternionTrafo)[7],
         double(&matrixTrafo)[3][4])
      {

         const double qW = quaternionTrafo[QUAT_QW];
         const double qX = quaternionTrafo[QUAT_QX];
         const double qY = quaternionTrafo[QUAT_QY];
         const double qZ = quaternionTrafo[QUAT_QZ];

         // conversion for unit quaternion to transformation matrix
         matrixTrafo[0][0] = 1 - 2 * ((qY * qY) + (qZ * qZ));
         matrixTrafo[0][1] = 2 * ((qX * qY) - (qW * qZ));
         matrixTrafo[0][2] = 2 * ((qX * qZ) + (qW * qY));
         matrixTrafo[0][3] = quaternionTrafo[QUAT_TX];

         matrixTrafo[1][0] = 2 * ((qX * qY) + (qW * qZ));
         matrixTrafo[1][1] = 1 - 2 * ((qX * qX) + (qZ * qZ));
         matrixTrafo[1][2] = 2 * ((qY * qZ) - (qW * qX));
         matrixTrafo[1][3] = quaternionTrafo[QUAT_TY];

         matrixTrafo[2][0] = 2 * ((qX * qZ) - (qW * qY));
         matrixTrafo[2][1] = 2 * ((qY * qZ) + (qW * qX));
         matrixTrafo[2][2] = 1 - 2 * ((qX * qX) + (qY * qY));
         matrixTrafo[2][3] = quaternionTrafo[QUAT_TZ];

      }

   }
}
