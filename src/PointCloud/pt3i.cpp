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

#include "pt3i.h"


Pt3i::Pt3i ()
{
  xp = 0;
  yp = 0;
  zp = 0;
  nb = 0;
}


Pt3i::Pt3i (int x, int y, int z)
{
  xp = x;
  yp = y;
  zp = z;
  nb = 0;
}


Pt3i::Pt3i (const Pt3i &p)
{
  xp = p.xp;
  yp = p.yp;
  zp = p.zp;
  nb = 0;
}


bool Pt3i::greaterThan (const Pt3i &p) const
{
  if (xp > p.xp) return true;
  if (xp < p.xp) return false;
  if (yp > p.yp) return true;
  if (yp < p.yp) return false;
  return (zp > p.zp);
}


bool Pt3i::find (Pt3i p)
{
  if (p.xp == xp && p.yp == yp)
  {
    nb++;
    return true;
  }
  return false;
}
