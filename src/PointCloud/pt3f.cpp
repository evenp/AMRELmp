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

#include "pt3f.h"
#include <cmath>


Pt3f::Pt3f ()
{
  xp = 0.0f;
  yp = 0.0f;
  zp = 0.0f;
}


Pt3f::Pt3f (float x, float y, float z)
{
  xp = x;
  yp = y;
  zp = z;
}


Pt3f::Pt3f (const Pt3f &p)
{
  xp = p.xp;
  yp = p.yp;
  zp = p.zp;
}


float Pt3f::distance (const Pt3f &p) const
{
  return ((float) sqrt ((p.xp - xp) * (p.xp - xp)
                        + (p.yp - yp) * (p.yp - yp)
                        + (p.zp - zp) * (p.zp - zp)));
}


bool Pt3f::greaterThan (const Pt3f &p) const
{
  if (xp > p.xp) return true;
  if (xp < p.xp) return false;
  if (yp > p.yp) return true;
  if (yp < p.yp) return false;
  return (zp > p.zp);
}


void Pt3f::normalize ()
{
  float n = sqrt (xp * xp + yp * yp + zp * zp);
  if (n != 0.0f)
  {
    xp /= n;
    yp /= n;
    zp /= n;
  }
}
