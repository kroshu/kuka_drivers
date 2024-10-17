/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.FRI” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
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
#ifndef _KUKA_FRI_DATA_HELPER_H
#define _KUKA_FRI_DATA_HELPER_H

#include <cmath>

namespace KUKA
{
   namespace FRI
   {

      /**
      * \brief Data helper class containing conversion functions.
      */
      struct DataHelper
      {

         /**
         * \brief Helper enum to access quaternion entries.
         */
         enum QUATERNION_IDX
         {
            QUAT_TX = 0,
            QUAT_TY,
            QUAT_TZ,
            QUAT_QW,
            QUAT_QX,
            QUAT_QY,
            QUAT_QZ
         };


         /**
         * \brief Function to convert a matrix transformation to a normalized quaternion transformation.
         *
         * The resulting quaternion transformation is provided as [t_x, t_y, t_z, q_w, q_x, q_y, q_z],
         * with a unit quaternion, i.e. the length of vector [q_w, q_x, q_y, q_z] must be 1.
         * The input transformation matrix has 3x4 elements. It consists of a rotational matrix (3x3 elements)
         * and a translational vector (3x1 elements). The complete transformation matrix has the
         * following structure:
         * [Transformation(3x4)] = [Rotation(3x3) | Translation(3x1) ]
         *
         * @param[in]  matrixTrafo       given matrix transformation
         * @param[out] quaternionTrafo   resulting quaternion transformation
         */
         static void convertTrafoMatrixToQuaternion(const double (&matrixTrafo)[3][4],
            double (&quaternionTrafo)[7]);


         /**
         * \brief Function to convert a quaternion transformation to a matrix transformation.
         *
         * The input quaternion transformation must be provided as [t_x, t_y, t_z, q_w, q_x, q_y, q_z],
         * with a unit quaternion, i.e. the length of vector [q_w, q_x, q_y, q_z] must be 1.
         * The output transformation matrix has 3x4 elements. It consists of a rotational matrix (3x3 elements)
         * and a translational vector (3x1 elements). The complete transformation matrix has the
         * following structure:
         * [Transformation(3x4)] = [Rotation(3x3) | Translation(3x1) ]
         *
         * @param[in]  quaternionTrafo    given quaternion transformation
         * @param[out] matrixTrafo        resulting matrix transformation
         */
         static void convertTrafoQuaternionToMatrix(const double(&quaternionTrafo)[7],
            double(&matrixTrafo)[3][4]);

      };

   }
}

#endif // _KUKA_FRI_DATA_HELPER_H
