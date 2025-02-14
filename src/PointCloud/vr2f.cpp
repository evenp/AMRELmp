/*  Copyright 2024 Philippe Even and Phuc Ngo,
      authors of paper:
      Even, P., and Ngo, P., 2021,
      Automatic forest road extraction fromLiDAR data of mountainous areas.
      In the First International Joint Conference of Discrete Geometry
      and Mathematical Morphology (Springer LNCS 12708), pp. 93-106.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "vr2f.h"


Vr2f::Vr2f ()
{
  xv = 1.0f;
  yv = 0.0f;
}


Vr2f::Vr2f (float x, float y)
{
  xv = x;
  yv = y;
}


Vr2f::Vr2f (const Vr2f &v)
{
  xv = v.xv;
  yv = v.yv;
}
