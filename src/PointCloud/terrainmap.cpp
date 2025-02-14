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

#include <iostream>
#include <fstream>
#include <inttypes.h>
#include <cmath>
#include "asmath.h"
#include "terrainmap.h"

const int TerrainMap::SHADE_HILL = 0;
const int TerrainMap::SHADE_SLOPE = 1;
const int TerrainMap::SHADE_EXP_SLOPE = 2;

const float TerrainMap::RELIEF_AMPLI = 5.0f;
const float TerrainMap::LIGHT_ANGLE_INCREMENT = 0.03f;

const int TerrainMap::DEFAULT_PAD_SIZE = 3;
const std::string TerrainMap::NVM_SUFFIX = std::string (".nvm");

const float TerrainMap::MM2M = 0.001f;
const double TerrainMap::EPS = 0.001;


TerrainMap::TerrainMap ()
{
  nmap = NULL;
  arr_files = NULL;
  iwidth = 0;
  iheight = 0;
  twidth = 0;
  theight = 0;
  cell_size = 0.0f;
  x_min = 0.0;
  y_min = 0.0;
  no_data = 0.0;
  shading = SHADE_HILL;
  light_angle = 0.0f;
  light_v1.set (- ASF_SQRT2_2, 0.0f, ASF_SQRT2_2);
  light_v2.set (0.25f, - ASF_SQRT3_2 / 2, ASF_SQRT3_2);
  light_v3.set (0.25f, ASF_SQRT3_2 / 2, ASF_SQRT3_2);
  slopiness = 1;
  pad_size = DEFAULT_PAD_SIZE;
  pad_w = pad_size;
  pad_h = pad_size;
  pad_ref = -1;
  ts_cot = 1;
  ts_rot = 1;
}


TerrainMap::~TerrainMap ()
{
  clear ();
}


void TerrainMap::clear ()
{
  if (arr_files != NULL)
  {
    for (int i = 0; i < ts_cot * ts_rot; i ++)
      if (arr_files[i] != NULL) delete arr_files[i];
    delete [] arr_files;
  }
  arr_files = NULL;
  if (nmap != NULL) delete [] nmap;
  nmap = NULL;
  input_layout.clear ();
  input_fullnames.clear ();
  input_nicknames.clear ();
  input_xmins.clear ();
  input_ymins.clear ();
}


int TerrainMap::get (int i, int j) const
{ 
  if (shading == SHADE_HILL)
  {
    float val1 = light_v1.scalar (nmap[j * iwidth + i]);
    if (val1 < 0.0f) val1 = 0.;
    float val2 = light_v2.scalar (nmap[j * iwidth + i]);
    if (val2 < 0.0f) val2 = 0.;
    float val3 = light_v3.scalar (nmap[j * iwidth + i]);
    if (val3 < 0.0f) val3 = 0.;
    float val = val1 + (val2 + val3) / 2;
    return (int) (val * 100);
  }
  else if (shading == SHADE_SLOPE)
  {
    Pt3f *pt = nmap + j * iwidth + i;
    return (255 - (int) (sqrt (pt->x() * pt->x() + pt->y() * pt->y()) * 255));
  }
  else if (shading == SHADE_EXP_SLOPE)
  {
    Pt3f *pt = nmap + j * iwidth + i;
    double alph = 1. - pt->x () * pt->x () - pt->y () * pt->y ();
    for (int sl = slopiness; sl > 1; sl --) alph *= alph;
    return ((int) (alph * 255));
  }
  else return 0;
}


int TerrainMap::get (int i, int j, int shading_type) const
{
  if (shading_type == SHADE_HILL)
  {
    float val1 = light_v1.scalar (nmap[j * iwidth + i]);
    if (val1 < 0.0f) val1 = 0.;
    float val2 = light_v2.scalar (nmap[j * iwidth + i]);
    if (val2 < 0.0f) val2 = 0.;
    float val3 = light_v3.scalar (nmap[j * iwidth + i]);
    if (val3 < 0.0f) val3 = 0.;
    float val = val1 + (val2 + val3) / 2;
    return (int) (val * 100);
  }
  else if (shading_type == SHADE_SLOPE)
  {
    Pt3f *pt = nmap + j * iwidth + i;
    return (255 - (int) (sqrt (pt->x() * pt->x() + pt->y() * pt->y()) * 255));
  }
  else if (shading_type == SHADE_EXP_SLOPE)
  {
    Pt3f *pt = nmap + j * iwidth + i;
    double alph = 1. - pt->x () * pt->x () - pt->y () * pt->y ();
    if (alph < 0.) alph = 0.;  // saturation
    for (int sl = slopiness; sl > 1; sl --) alph *= alph;
    return ((int) (alph * 255));
  }
  else return 0;
}


double TerrainMap::getSlopeFactor (int i, int j, int slp) const
{
  Pt3f *pt = nmap + j * iwidth + i;
  double alph = 1. - pt->x () * pt->x () - pt->y () * pt->y ();
  if (alph < 0.) alph = 0.;  // saturation
  for (int sl = slp; sl > 1; sl --) alph *= alph;
  return (alph);
}


void TerrainMap::toggleShadingType ()
{
  if (++shading > SHADE_EXP_SLOPE) shading = SHADE_HILL;
}


void TerrainMap::incLightAngle (int val)
{
  light_angle += LIGHT_ANGLE_INCREMENT * val;
  if (light_angle < 0.0f) light_angle += ASF_2PI;
  else if (light_angle >= ASF_2PI) light_angle -= ASF_2PI;

  float ang = light_angle;
  light_v1.set (- (float) (cos (ang) * ASF_SQRT2_2),
                - (float) (sin (ang) * ASF_SQRT2_2), ASF_SQRT2_2);
  ang += ASF_2PI_3;
  light_v2.set (- (float) (cos (ang) / 2),
                - (float) (sin (ang) / 2), ASF_SQRT3_2);
  ang += ASF_2PI_3;
  light_v3.set (- (float) (cos (ang) / 2),
                - (float) (sin (ang) / 2), ASF_SQRT3_2);
}


void TerrainMap::setLightAngle (float val)
{
  light_angle = val;
  if (light_angle < 0.0f) light_angle += ASF_2PI;
  else if (light_angle >= ASF_2PI) light_angle -= ASF_2PI;

  float ang = light_angle;
  light_v1.set (- (float) (cos (ang) * ASF_SQRT2_2),
                - (float) (sin (ang) * ASF_SQRT2_2), ASF_SQRT2_2);
  ang += ASF_2PI_3;
  light_v2.set (- (float) (cos (ang) / 2),
                - (float) (sin (ang) / 2), ASF_SQRT3_2);
  ang += ASF_2PI_3;
  light_v3.set (- (float) (cos (ang) / 2),
                - (float) (sin (ang) / 2), ASF_SQRT3_2);
}


void TerrainMap::incSlopinessFactor (int inc)
{
  slopiness += inc;
  if (slopiness < 1) slopiness = 1;
}

void TerrainMap::setSlopinessFactor (int val)
{
  slopiness = val;
  if (slopiness < 1) slopiness = 1;
}


Pt2i TerrainMap::closestFlatArea (const Pt2i &pt, int srad, int frad, int sfact)
{
  int sxmin = pt.x () - srad, sxmax = pt.x () + srad + 1;
  int symin = pt.y () - srad, symax = pt.y () + srad + 1;
  if (sxmin < 0) sxmin = 0;
  if (symin < 0) symin = 0;
  if (sxmax > iwidth) sxmax = iwidth;
  if (symax > iheight) symax = iheight;

  int fxmin = sxmin - frad, fxmax = sxmax + frad;
  int fymin = symin - frad, fymax = symax + frad;
  if (fxmin < 0) fxmin = 0;
  if (fymin < 0) fymin = 0;
  if (fxmax > iwidth) fxmax = iwidth;
  if (fymax > iheight) fymax = iheight;

  int sw = sxmax - sxmin, sh = symax - symin, swh = sw * sh;
  double *val = new double[swh];
  int *cpt = new int[swh];
  for (int i = 0; i < swh; i++)
  {
    val[i] = 0.;
    cpt[i] = 0;
  }

  for (int fi = fxmin; fi < fxmax; fi ++)
  {
    int lxmin = fi - frad - sxmin;
    int lxmax = fi + frad + 1 - sxmin;
    if (lxmin < 0) lxmin = 0;
    if (lxmax > sw) lxmax = sw;
    for (int fj = fymin; fj < fymax; fj ++)
    {
      double dval = getSlopeFactor (fi, iheight - 1 - fj, sfact);
      int lymin = fj - frad - symin;
      int lymax = fj + frad + 1 - symin;
      if (lymin < 0) lymin = 0;
      if (lymax > sh) lymax = sh;
      for (int lj = lymin; lj < lymax; lj ++)
        for (int li = lxmin; li < lxmax; li ++)
        {
          val[lj * sw + li] += dval;
          cpt[lj * sw + li] ++;
        }
    }
  }

  int cmax = 0;
  val[0] /= cpt[0];
  for (int i = 1; i < swh; i ++)
  {
    val[i] /= cpt[i];
    if (val[i] > val[cmax]) cmax = i;
  }
  delete [] val;
  delete [] cpt;
  return (Pt2i (sxmin + cmax % sw, symin + cmax / sw));
}


bool TerrainMap::addNormalMapFile (const std::string &name)
{
  std::ifstream dtmf (name, std::ios::in);
  if (! dtmf.is_open ()) return false;
  dtmf.close ();
  input_fullnames.push_back (name);
  return true;
}


bool TerrainMap::assembleMap (int cols, int rows, int64_t xmin, int64_t ymin,
                              bool padding)
{
  int locw = 0, loch = 0, loci = 0, locj = 0;
  float wmap = 0.0f, hmap = 0.0f, locs = 0.0f, locxmin = 0.0f, locymin = 0.0f;
  if (padding)
  {
    ts_cot = cols;
    ts_rot = rows;
  }
  twidth = 0;
  theight = 0;
  x_min = (double) (xmin) * MM2M;
  y_min = (double) (ymin) * MM2M;
  if (padding)
  {
    arr_files = new std::string *[cols * rows];
    for (int i = 0; i < cols * rows; i++) arr_files[i] = NULL;
  }
  std::vector<std::string>::iterator it = input_fullnames.begin ();
  while (it != input_fullnames.end ())
  {
    std::ifstream nvmf (it->c_str (), std::ios::in | std::ifstream::binary);
    if (! nvmf.is_open ())
      std::cout << "File " << *it << " can't be opened" << std::endl;
    else
    {
      nvmf.read ((char *) (&locw), sizeof (int));
      nvmf.read ((char *) (&loch), sizeof (int));
      nvmf.read ((char *) (&locs), sizeof (float));
      nvmf.read ((char *) (&locxmin), sizeof (float));
      nvmf.read ((char *) (&locymin), sizeof (float));
      if (twidth != 0)
      {
        bool ok = true;
        if (locw != twidth)
        {
          std::cout << *it << " : distinct width" << std::endl;
          ok = false;
        }
        if (loch != theight)
        {
          std::cout << *it << " : distinct height" << std::endl;
          ok = false;
        }
        if (locs != cell_size)
        {
          std::cout << *it << " : distinct cell size" << std::endl;
          ok = false;
        }
        if (padding)
        {
          double dx = ((double) ((int) (locxmin + 0.5f))) - x_min;
          if (dx < 0.0) dx = - dx;
          if (((int) (dx + 0.5f)) % ((int) (wmap + 0.5f)) != 0)
          {
            std::cout << *it << " : X axis aperiodicity" << std::endl;
            ok = false;
          }
          double dy = ((double) ((int) (locymin + 0.5f))) - y_min;
          if (dy < 0.0) dy = - dy;
          if (((int) (dy + 0.5f)) % ((int) (hmap + 0.5f)) != 0)
          {
            std::cout << *it << " : Y axis aperiodicity" << std::endl;
            ok = false;
          }
        }
        if (! ok)
        {
          nvmf.close ();
          return false;
        }
      }
      else
      {
        twidth = locw;
        theight = loch;
        cell_size = locs;
        iwidth = cols * twidth;
        iheight = rows * theight;
        if (! padding)
        {
          if (nmap != NULL) delete [] nmap;
          nmap = new Pt3f[iwidth * iheight];
        }
      }
      wmap = twidth * cell_size;
      hmap = theight * cell_size;
      loci = (int) ((locxmin - x_min + wmap / 2) / wmap);
      locj = (int) ((locymin - y_min + hmap / 2) / hmap);
      if (padding) arr_files[locj * cols + loci] = &(*it);
      else
      {
        Pt3f *line = nmap + iwidth * (iheight - 1);
        line -= locj * theight * iwidth;
        line += loci * twidth;
        for (int j = 0; j < theight; j++)
        {
          nvmf.read ((char *) line, twidth * sizeof (Pt3f));
          line -= iwidth;
        }
      }
      nvmf.close ();
    }
    it ++;
  }
  return true;
}


bool TerrainMap::loadNormalMapInfo (const std::string &name)
{
  std::ifstream nvmf (name.c_str (), std::ios::in | std::ifstream::binary);
  if (! nvmf.is_open ())
  {
    std::cout << "File " << name << " can't be opened" << std::endl;
    return false;
  }
  float x, y;
  nvmf.read ((char *) (&twidth), sizeof (int));
  nvmf.read ((char *) (&theight), sizeof (int));
  nvmf.read ((char *) (&cell_size), sizeof (float));
  nvmf.read ((char *) (&x), sizeof (float));
  nvmf.read ((char *) (&y), sizeof (float));
  nvmf.close ();
  x_min = (double) (x + 0.5f);
  y_min = (double) (y + 0.5f);
  iwidth = twidth;
  iheight = theight;
  return true;
}


void TerrainMap::setPadSize (int val)
{
  if (val >= 0 && val % 2 == 1)
  {
    pad_size = val;
    pad_w = pad_size;
    pad_h = pad_size;
  }
}


void TerrainMap::adjustPadSize ()
{
  if (pad_w > ts_cot) pad_w = ts_cot;
  if (pad_h > ts_rot) pad_h = ts_rot;
}


int TerrainMap::nextPad (unsigned char *map)
{
  if (pad_ref == -1)
  {
    pad_ref = 0;
    if (nmap != NULL) delete [] nmap;
    nmap = new Pt3f[twidth];
    for (int j = 0; j < pad_h; j ++)
      for (int i = 0; i < pad_w; i ++)
        loadMap (j * ts_cot + i,
                 map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                     + i * twidth);
  }
  else if (((pad_ref / ts_cot) / (pad_h - 2)) % 2 == 1)
  {
    if (pad_ref % ts_cot == 0)
    {
      if (pad_ref + ts_cot * pad_h >= ts_cot * ts_rot)
      {
        // getting out
        pad_ref = -1;
        if (nmap != NULL) delete [] nmap;
        nmap = NULL;
      }
      else
      {
        // climbing up on left side to next row
        pad_ref += ts_cot * (pad_h - 2);
        int pad_eh = pad_h;
        if (pad_ref / ts_cot + pad_h > ts_rot)
          pad_eh -= pad_ref / ts_cot + pad_h - ts_rot;
        unsigned char *fmap = map + 2 * theight * pad_w * twidth;
        unsigned char *tmap = map + pad_h * theight * pad_w * twidth;
        for (int j = 0; j < 2 * theight; j ++)
          for (int i = 0; i < pad_w * twidth; i ++) *--tmap = *--fmap;
        for (int j = 2; j < pad_eh; j ++)
          for (int i = 0; i < pad_w; i ++)
            loadMap ((pad_ref / ts_cot + j) * ts_cot + pad_ref % ts_cot + i,
                     map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                         + i * twidth);
        for (int j = pad_eh; j < pad_h; j ++)
          for (int i = 0; i < pad_w; i ++)
            clearMap (map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                         + i * twidth, pad_w, twidth, theight);
      }
    }
    else
    {
      // going left to next column
      pad_ref -= pad_w - 2;
      int pad_eh = pad_h;
      if (pad_ref / ts_cot + pad_h > ts_rot)
        pad_eh -= pad_ref / ts_cot + pad_h - ts_rot;
      unsigned char *fmap = map + (pad_h - pad_eh) * theight * pad_w * twidth
                                + 2 * twidth;
      unsigned char *tmap = fmap + (pad_w - 2) * twidth;
      for (int j = 0; j < pad_eh * theight; j ++)
      {
        for (int i = 0; i < 2 * twidth; i ++) *--tmap = *--fmap;
        fmap += (pad_w + 2) * twidth;
        tmap += (pad_w + 2) * twidth;
      }
      for (int j = 0; j < pad_eh; j ++)
        for (int i = 0; i < pad_w - 2; i ++)
          loadMap ((pad_ref / ts_cot + j) * ts_cot + pad_ref % ts_cot + i,
                   map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                       + i * twidth);
    }
  }
  else
  {
    if ((pad_ref % ts_cot) + pad_w >= ts_cot)
    {
      if (pad_ref + ts_cot * pad_h >= ts_cot * ts_rot)
      {
        // getting out
        pad_ref = -1;
        if (nmap != NULL) delete [] nmap;
        nmap = NULL;
      }
      else
      {
        // climbing up on right side to next row
        pad_ref += ts_cot * (pad_h - 2);
        int pad_ew = pad_w;
        if (pad_ref % ts_cot + pad_w > ts_cot)
          pad_ew -= pad_ref % ts_cot + pad_w - ts_cot;
        int pad_eh = pad_h;
        if (pad_ref / ts_cot + pad_h > ts_rot)
          pad_eh -= pad_ref / ts_cot + pad_h - ts_rot;
        unsigned char *fmap = map + 2 * theight * pad_w * twidth;
        unsigned char *tmap = map + pad_h * theight * pad_w * twidth;
        for (int j = 0; j < 2 * theight; j ++)
        {
          fmap -= (pad_w - pad_ew) * twidth;
          tmap -= (pad_w - pad_ew) * twidth;
          for (int i = 0; i < pad_ew * twidth; i ++) *--tmap = *--fmap;
        }
        for (int j = 2; j < pad_eh; j ++)
          for (int i = 0; i < pad_ew; i ++)
            loadMap ((pad_ref / ts_cot + j) * ts_cot + pad_ref % ts_cot + i,
                     map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                         + i * twidth);
        for (int j = pad_eh; j < pad_h; j ++)
          for (int i = 0; i < pad_ew; i ++)
            clearMap (map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                         + i * twidth, pad_w, twidth, theight);
      }
    }
    else
    {
      // going right to next column
      pad_ref += pad_w - 2;
      int pad_ew = pad_w;
      if (pad_ref % ts_cot + pad_w > ts_cot)
        pad_ew -= pad_ref % ts_cot + pad_w - ts_cot;
      int pad_eh = pad_h;
      if (pad_ref / ts_cot + pad_h > ts_rot)
        pad_eh -= pad_ref / ts_cot + pad_h - ts_rot;
      unsigned char *tmap = map + (pad_h - pad_eh) * theight * pad_w * twidth;
      unsigned char *fmap = tmap + (pad_w - 2) * twidth;
      for (int j = 0; j < pad_eh * theight; j ++)
      {
        for (int i = 0; i < 2 * twidth; i ++) *tmap++ = *fmap++;
        fmap += (pad_w - 2) * twidth;
        tmap += (pad_w - 2) * twidth;
      }
      for (int j = 0; j < pad_eh; j ++)
      {
        for (int i = 2; i < pad_ew; i ++)
        {
          loadMap ((pad_ref / ts_cot + j) * ts_cot + pad_ref % ts_cot + i,
                   map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                       + i * twidth);
        }
        for (int i = pad_ew; i < pad_w; i ++)
        {
          clearMap (map + ((pad_h - j) * theight - 1) * (pad_w * twidth)
                        + i * twidth, pad_w, twidth, theight);
        }
      }
    }
  }
  return pad_ref;
}


bool TerrainMap::getLayoutInfo (std::string &name, double &xmin, double &ymin,
                                Pt2i lay)
{
  int index = -1, i = 0;
  for (std::vector<Pt2i>::iterator it = input_layout.begin ();
       index == -1 && it != input_layout.end (); it++, i++)
    if (it->x () == lay.x () && it->y () == lay.y ()) index = i;
  if (index != -1)
  {
    name = input_nicknames[index];
    xmin = input_xmins[index];
    ymin = input_ymins[index];
  }
  return (index != -1);
}


bool TerrainMap::loadMap (int k, unsigned char *submap)
{
//  std::cout << "MTILE " << k << " : "
//       << (arr_files[k] == NULL ? "NULL" : *arr_files[k]) << std::endl;
  int locw = 0, loch = 0;
  float locs = 0.0f, locxmin = 0.0f, locymin = 0.0f;

  if (arr_files[k] != NULL)
  {
    std::ifstream nvmf (arr_files[k]->c_str (),
                        std::ios::in | std::ifstream::binary);
    if (! nvmf.is_open ())
      std::cout << "File " << *arr_files[k] << " can't be opened" << std::endl;
    nvmf.read ((char *) (&locw), sizeof (int));
    nvmf.read ((char *) (&loch), sizeof (int));
    nvmf.read ((char *) (&locs), sizeof (float));
    nvmf.read ((char *) (&locxmin), sizeof (float));
    nvmf.read ((char *) (&locymin), sizeof (float));
    if (locw != twidth)
    {
      std::cout << "File " << *arr_files[k] << " inconsistent width"
                << std::endl;
      nvmf.close ();
      return false;
    }
    if (loch != theight)
    {
      std::cout << "File " << *arr_files[k] << " inconsistent height"
                << std::endl;
      nvmf.close ();
      return false;
    }
    if (locs != cell_size)
    {
      std::cout << "File " << *arr_files[k] << " inconsistent cell size"
                << std::endl;
      nvmf.close ();
      return false;
    }

    unsigned char *pmap = submap;
    for (int j = 0; j < theight; j++)
    {
      nvmf.read ((char *) nmap, twidth * sizeof (Pt3f));
      //traite la ligne nmap pour construire submap
      for (int i = 0; i < twidth; i ++)
      {
        int val = 255 - (int) (sqrt (nmap[i].x () * nmap[i].x ()
                                     + nmap[i].y () * nmap[i].y ()) * 255);
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        *pmap++ = val;
      }
      pmap -= (pad_w + 1) * twidth;
    }
    nvmf.close ();
  }
  else
  {
    unsigned char *pmap = submap;
    for (int j = 0; j < theight; j++)
    {
      for (int i = 0; i < twidth; i ++) *pmap ++ = 0;
      pmap -= (pad_w + 1) * twidth;
    }
  }
  return true;
}


void TerrainMap::clearMap (unsigned char *submap, int pw, int w, int h)
{
  for (int j = 0; j < h; j++)
  {
    for (int i = 0; i < w; i++) *submap++ = 0;
    submap -= (pw + 1) * w;
  }
}


void TerrainMap::saveFirstNormalMap (const std::string &name) const
{
  std::ofstream nvmf (name.c_str (), std::ios::out | std::ofstream::binary);
  if (! nvmf.is_open ())
    std::cout << "File " << name << " can't be created" << std::endl;
  else
  {
    nvmf.write ((char *) (&twidth), sizeof (int));
    nvmf.write ((char *) (&theight), sizeof (int));
    nvmf.write ((char *) (&cell_size), sizeof (float));
    float fxm = (float) input_xmins.front ();
    nvmf.write ((char *) (&fxm), sizeof (float));
    float fym = (float) input_ymins.front ();
    nvmf.write ((char *) (&fym), sizeof (float));
    Pt2i txy = input_layout.front ();
    Pt3f *line = nmap + iwidth * (iheight - 1);
    line -= txy.y () * theight * iwidth;
    line += txy.x () * twidth;
    for (int j = 0; j < theight; j++)
    {
      nvmf.write ((char *) line, twidth * sizeof (Pt3f));
      line -= iwidth;
    }
    nvmf.close ();
  }
}

void TerrainMap::saveLoadedNormalMaps (const std::string &dir) const
{
  std::vector<std::string>::const_iterator it = input_nicknames.begin ();
  std::vector<double>::const_iterator xit = input_xmins.begin ();
  std::vector<double>::const_iterator yit = input_ymins.begin ();
  std::vector<Pt2i>::const_iterator lit = input_layout.begin ();
  while (it != input_nicknames.end ())
  {
    std::string name (dir);
    name += *it + NVM_SUFFIX;
    std::ofstream nvmf (name.c_str (), std::ios::out | std::ofstream::binary);
    if (! nvmf.is_open ())
      std::cout << "File " << name << " can't be created" << std::endl;
    else
    {
      nvmf.write ((char *) (&twidth), sizeof (int));
      nvmf.write ((char *) (&theight), sizeof (int));
      nvmf.write ((char *) (&cell_size), sizeof (float));
      float fxm = (float) (*xit);
      nvmf.write ((char *) (&fxm), sizeof (float));
      float fym = (float) (*yit);
      nvmf.write ((char *) (&fym), sizeof (float));
      Pt2i txy (*lit);
      Pt3f *line = nmap + iwidth * (iheight - 1);
      line -= txy.y () * theight * iwidth;
      line += txy.x () * twidth;
      for (int j = 0; j < theight; j++)
      {
        nvmf.write ((char *) line, twidth * sizeof (Pt3f));
        line -= iwidth;
      }
      nvmf.close ();
    }
    it ++;
    xit ++;
    yit ++;
    lit ++;
  }
}


bool TerrainMap::addDtmFile (const std::string &name, bool verb, bool grid_ref)
{
  std::ifstream dtmf (name.c_str (), std::ios::in);
  if (! dtmf.is_open ())
  {
    if (verb) std::cout << "File " << name << " can't be opened" << std::endl;
    return false;
  }
  char val[100];  // IGN
  int width = 0, height = 0;
  double xllc = 0., yllc = 0., nodata = 0.;
  float csize = 0.0f;

  dtmf >> val;
  dtmf >> width;
  if (grid_ref) width --;
  dtmf >> val;
  dtmf >> height;
  if (grid_ref) height --;
  dtmf >> val;
  dtmf >> xllc;
  xllc = (double) ((int) (xllc + 0.5f));
  dtmf >> val;
  dtmf >> yllc;
  yllc = (double) ((int) (yllc + 0.5f));
  dtmf >> val;
  dtmf >> csize;

  if (iwidth == 0)
  {
    twidth = width;
    theight = height;
    iwidth = width;
    iheight = height;
    x_min = xllc;
    y_min = yllc;
    cell_size = csize;
    no_data = nodata;
    input_layout.push_back (Pt2i (0, 0));
  }
  else
  {
    if (width != twidth)
    {
      std::cout << "File " << name << " inconsistent width" << std::endl;
      return false;
    }
    if (height != theight)
    {
      std::cout << "File " << name << " inconsistent height" << std::endl;
      return false;
    }
    if (csize != cell_size)
    {
      std::cout << "File " << name << " inconsistent cell size" << std::endl;
      return false;
    }

    double shift = ((xllc - x_min) / csize) / width;
    int xshift = (int) (shift + (shift < 0 ? - 0.5 : 0.5));
    double err = xllc - (x_min + xshift * csize * width);
    if (err < - EPS || err > EPS)
    {
      std::cout << "File " << name << " : xllc irregular" << std::endl;
      return false;
    }
    shift = ((yllc - y_min) / csize) / height;
    int yshift = (int) (shift + (shift < 0 ? - 0.5 : 0.5));
    err = yllc - (y_min + yshift * csize * height);
    if (err < - EPS || err > EPS)
    {
      std::cout << "File " << name << " : yllc irregular" << std::endl;
      return false;
    }
    if (xshift < 0 || yshift < 0)
    {
      std::vector<Pt2i>::iterator it = input_layout.begin ();
      while (it != input_layout.end ())
      {
        if (xshift < 0) it->set (it->x () - xshift, it->y ());
        if (yshift < 0) it->set (it->x (), it->y () - yshift);
        it ++;
      }
      if (xshift < 0)
      {
        iwidth -= xshift * width;
        xshift = 0;
        x_min = xllc;
      }
      if (yshift < 0)
      {
        iheight -= yshift * height;
        yshift = 0;
        y_min = yllc;
      }
    }
    input_layout.push_back (Pt2i (xshift, yshift));
    if (iwidth / width <= xshift) iwidth = (xshift + 1) * width;
    if (iheight / height <= yshift) iheight = (yshift + 1) * height;
  }

  input_fullnames.push_back (name);
  input_xmins.push_back (xllc);
  input_ymins.push_back (yllc);
  return true;
}


void TerrainMap::addDtmName (const std::string &name)
{
  input_nicknames.push_back (name);
}


bool TerrainMap::createMapFromDtm (bool verb, bool grid_ref)
{
  int isz = (grid_ref ? (iwidth + 1) * (iheight + 1) : iwidth * iheight);
  double *hval = new double[isz];
  for (int i = 0; i < isz; i++) hval[i] = no_data;

  std::vector<Pt2i>::iterator it = input_layout.begin ();
  std::vector<std::string>::iterator itn = input_fullnames.begin ();
  while (it != input_layout.end ())
  {
    int dx = it->x () * twidth;
    int dy = (iheight / theight - 1 - it->y ()) * theight;
    if (verb) std::cout << "Opening " << *itn << std::endl;
    std::ifstream dtmf (itn->c_str (), std::ios::in);
    if (! dtmf.is_open ()) return false;
    char val[15];
    double hv = 0.0, nodata = 0.0;
    for (int i = 0; i < 11; i++) dtmf >> val;
    dtmf >> nodata;

    int loc_th = (grid_ref ? theight + 1 : theight);
    int loc_tw = (grid_ref ? twidth + 1 : twidth);
    for (int j = 0; j < loc_th; j++)
      for (int i = 0; i < loc_tw; i++)
      {
        dtmf >> hv;
        hval[(dy + j) * iwidth + dx + i] = (hv == nodata ? no_data : hv);
      }
    dtmf.close ();
    it ++;
    itn ++;
  }

  if (nmap != NULL) delete [] nmap;
  nmap = new Pt3f[iwidth * iheight];
  Pt3f *nval = nmap;
  double dhx, dhy;
  if (grid_ref)
  {
    for (int j = 0; j < iheight; j++)
    {
      for (int i = 0; i < iwidth; i++)
      {
        dhy = (hval[(j+1) * iwidth + i] - hval[j * iwidth + i])
                   * 2 * RELIEF_AMPLI;
        dhx = (hval[j * iwidth + i + 1] - hval[j * iwidth + i])
                   * 2 * RELIEF_AMPLI;
        nval->set (- (float) dhx, - (float) dhy, 1.0f);
        nval->normalize ();
        nval++;
      }
    }
  }
  else
  {
    for (int j = 0; j < iheight; j++)
    {
      for (int i = 0; i < iwidth; i++)
      {
        if (j == iheight - 1)
          dhy = (hval[j * iwidth + i] - hval[(j-1) * iwidth + i])
                * 2 * RELIEF_AMPLI;
        else if (j == 0)
          dhy = (hval[(j+1) * iwidth + i] - hval[j * iwidth + i])
                * 2 * RELIEF_AMPLI;
        else dhy = (hval[(j+1) * iwidth + i] - hval[(j-1) * iwidth + i])
                   * RELIEF_AMPLI;
        if (i == iwidth - 1)
          dhx = (hval[j * iwidth + i] - hval[j * iwidth + i-1])
                * 2 * RELIEF_AMPLI;
        else if (i == 0)
          dhx = (hval[j * iwidth + i+1] - hval[j * iwidth + i])
                * 2 * RELIEF_AMPLI;
        else dhx = (hval[j * iwidth + i+1] - hval[j * iwidth + i-1])
                   * RELIEF_AMPLI;

        nval->set (- (float) dhx, - (float) dhy, 1.0f);
        nval->normalize ();
        nval++;
      }
    }
  }
  delete [] hval;
  return true;
}


bool TerrainMap::loadDtmMapInfo (const std::string &name)
{
  std::ifstream dtmf (name.c_str (), std::ios::in);
  if (! dtmf.is_open ())
  {
    std::cout << "File " << name << " can't be opened" << std::endl;
    return false;
  }
  char val[20];
  double xllc = 0., yllc = 0.;
  dtmf >> val;
  dtmf >> twidth;
  dtmf >> val;
  dtmf >> theight;
  dtmf >> val;
  dtmf >> xllc;
  dtmf >> val;
  dtmf >> yllc;
  dtmf >> val;
  dtmf >> cell_size;
  dtmf.close ();
  x_min = (double) ((int) (xllc + 0.5f));
  y_min = (double) ((int) (yllc + 0.5f));
  iwidth = twidth;
  iheight = theight;
  return true;
}


void TerrainMap::saveSubMap (int imin, int jmin, int imax, int jmax) const
{
  int nw = imax - imin, nh = jmax - jmin;
  float xm = (float) ((int) (x_min + (double) imin * cell_size + 0.5));
  float ym = (float) ((int) (y_min + (double) jmin * cell_size + 0.5));

  std::ofstream nvmf ("nvm/newtile.nvm", std::ios::out | std::ofstream::binary);
  if (! nvmf.is_open ())
    std::cout << "nvm/newtile.nvm can't be created" << std::endl;
  else
  {
    nvmf.write ((char *) (&nw), sizeof (int));
    nvmf.write ((char *) (&nh), sizeof (int));
    nvmf.write ((char *) (&cell_size), sizeof (float));
    nvmf.write ((char *) (&xm), sizeof (float));
    nvmf.write ((char *) (&ym), sizeof (float));
    Pt3f *line = nmap + iwidth * (iheight - 1);
    line -= jmin * iwidth;
    line += imin;
    for (int j = 0; j < nh; j++)
    {
      nvmf.write ((char *) line, nw * sizeof (Pt3f));
      line -= iwidth;
    }
    nvmf.close ();
  }
}


void TerrainMap::checkArrangement ()
{
  for (int i = 0; i < (iheight / theight) * (iwidth / twidth); i++)
    std::cout << "DTM TILE " << i << " : "
         << (arr_files[i] == NULL ? "NULL" : *arr_files[i]) << std::endl;
}


/*
void TerrainMap::trace ()
{
  std::cout << "Iwidth = " << iwidth << ", Iheight = " << iheight << std::endl;
  std::cout << "Twidth = " << twidth << ", Theight = " << theight << std::endl;
  std::cout << "Xmin = " << (int64_t) (x_min * 1000 + 0.5)
               << ", Ymin = " << (int64_t) (y_min * 1000 + 0.5) << std::endl;
  std::cout << "Csize = " << cell_size << ", Nodata = " << no_data << std::endl;
}


void TerrainMap::traceNvmFileInfo (const std::string &name)
{
  std::cout << "Tracing " << name << std::endl;
  std::ifstream nvmf (name.c_str (), std::ios::in | std::ifstream::binary);
  if (! nvmf.is_open ())
    std::cout << "File " << name << " can't be opened" << std::endl;
  else
  {
    int locw, loch;
    float locs, locxmin, locymin;
    nvmf.read ((char *) (&locw), sizeof (int));
    nvmf.read ((char *) (&loch), sizeof (int));
    std::cout << "Width = " << locw << " Height = " << loch << std::endl;
    nvmf.read ((char *) (&locs), sizeof (float));
    std::cout << "Csize = " << locs << std::endl;
    nvmf.read ((char *) (&locxmin), sizeof (float));
    nvmf.read ((char *) (&locymin), sizeof (float));
    std::cout << "Xmin = " << (int) (locxmin + 0.5f)
              << " Ymin = " << (int) (locymin + 0.5f) << std::endl;
  }
}
*/
