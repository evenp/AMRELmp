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

#include "ctrackdetector.h"
#include <cmath>
#include <algorithm>


const int CTrackDetector::RESULT_NONE = 0;
const int CTrackDetector::RESULT_OK = 1;
const int CTrackDetector::RESULT_FAIL_TOO_NARROW_INPUT = -1;
const int CTrackDetector::RESULT_FAIL_NO_AVAILABLE_SCAN = -2;
const int CTrackDetector::RESULT_FAIL_NO_CENTRAL_PLATEAU = -3;
const int CTrackDetector::RESULT_FAIL_NO_CONSISTENT_SEQUENCE = -4;
const int CTrackDetector::RESULT_FAIL_NO_BOUNDS = -5;
const int CTrackDetector::RESULT_FAIL_TOO_HECTIC_PLATEAUX = -6;
const int CTrackDetector::RESULT_FAIL_TOO_SPARSE_PLATEAUX = -7;
const int CTrackDetector::RESULT_FAIL_DISCONNECT = -8;

const float CTrackDetector::MAX_TRACK_WIDTH = 6.0f;
const int CTrackDetector::DEFAULT_PLATEAU_LACK_TOLERANCE = 11;
const int CTrackDetector::NOBOUNDS_TOLERANCE = 10;
const int CTrackDetector::INITIAL_TRACK_EXTENT = 6;
const int CTrackDetector::DEFAULT_POS_AND_HEIGHT_REGISTER_SIZE = 8;
const int CTrackDetector::DEFAULT_UNSTABILITY_REGISTER_SIZE = 6;
const int CTrackDetector::DEFAULT_MIN_DENSITY = 60;
const float CTrackDetector::DEFAULT_MAX_SHIFT_LENGTH = 1.65f;
const float CTrackDetector::LN_UNSTAB = 0.25f;
const int CTrackDetector::NB_UNSTAB = 2;
const float CTrackDetector::POS_INCR = 0.05f;

const int CTrackDetector::NB_SIDE_TRIALS = 5;


CTrackDetector::CTrackDetector ()
{
  auto_p = false;
  connect_on = false;
  ptset = NULL;
  profileRecordOn = false;
  plateau_lack_tolerance = DEFAULT_PLATEAU_LACK_TOLERANCE;
  initial_track_extent = INITIAL_TRACK_EXTENT; // direction precalculation on
  density_insensitive = false;
  density_pruning = true;
  min_density = DEFAULT_MIN_DENSITY;
  shift_length_pruning = true;
  max_shift_length = DEFAULT_MAX_SHIFT_LENGTH;
  fct = NULL;
  fstatus = RESULT_NONE;
  ict = NULL;
  istatus = RESULT_NONE;
  pfeat.setMinLength (CarriageTrack::MIN_WIDTH);
  pfeat.setMaxLength (CarriageTrack::MAX_WIDTH);
  posht_nb = DEFAULT_POS_AND_HEIGHT_REGISTER_SIZE;
  lpok = new bool[posht_nb];
  lpos = new float[posht_nb];
  lhok = new bool[posht_nb];
  lht = new float[posht_nb];
  initial_ref = 0;
  initial_refs = 0.0f;
  initial_refe = 0.0f;
  initial_refh = 0.0f;
  initial_unbounded = true;
  unstab_nb = DEFAULT_UNSTABILITY_REGISTER_SIZE;
  spos = new float[unstab_nb];
  epos = new float[unstab_nb];
  spok = new bool[unstab_nb];
  epok = new bool[unstab_nb];
  resetRegisters ();
  out_count = 0;
}


CTrackDetector::~CTrackDetector ()
{
  clear ();
}


void CTrackDetector::clear ()
{
  if (fct != NULL) delete fct;
  fct = NULL;
  fstatus = RESULT_NONE;
  if (ict != NULL) delete ict;
  ict = NULL;
  istatus = RESULT_NONE;
}


void CTrackDetector::preserveDetection ()
{
  fct = NULL;
}


void CTrackDetector::setPointsGrid (IPtTileSet *data, int width, int height,
                                    int subdiv, float cellsize)
{
  ptset = data;
  this->subdiv = subdiv;
  this->csize = cellsize;
  scanp.setSize (width * subdiv, height * subdiv);
  discanp.setSize (width, height);
}


CarriageTrack *CTrackDetector::detect (const Pt2i &p1, const Pt2i &p2)
{
  // Cleans up former detection
  clear ();

  // Checks input stroke length
  ip1.set (p1);
  ip2.set (p2);
  fp1.set (p1);
  fp2.set (p2);
  Vr2f p12 (csize * (p2.x () - p1.x ()), csize * (p2.y () - p1.y ()));
  float l12 = (float) sqrt (p12.x () * p12.x () + p12.y () * p12.y ());
  if (l12 < MAX_TRACK_WIDTH)
  {
    fstatus = RESULT_FAIL_TOO_NARROW_INPUT;
    if (initial_track_extent != 0) istatus = RESULT_FAIL_TOO_NARROW_INPUT;
    return NULL;
  }

  // Start detection trials in automatic mode
  if (auto_p) detect ();
  // Initial detection (or final if no initial detection)
  else detect (initial_track_extent);

  if (ict != NULL && istatus != RESULT_FAIL_NO_CENTRAL_PLATEAU)
  {
    // Aligns input stroke orthogonally to detected carriage track
    float fact = csize / (p12.x () * p12.x () + p12.y () * p12.y ());
    Pt2f p1f (csize * (p1.x () + 0.5f), csize * (p1.y () + 0.5f));
    Vr2f lshift;
    if (p12.x () > 0)
      if (p12.y () > 0)
        if (p12.x () > p12.y ())
          lshift.set (- (p12.x () * p12.y ()) * fact,
                      (p12.x () * p12.x ()) * fact);
        else
          lshift.set (- (p12.y () * p12.y ()) * fact,
                      (p12.x () * p12.y ()) * fact);
      else
        if (p12.x () > - p12.y ())
          lshift.set (- (p12.x () * p12.y ()) * fact,
                      (p12.x () * p12.x ()) * fact);
        else
          lshift.set ((p12.y () * p12.y ()) * fact,
                      - (p12.x () * p12.y ()) * fact);
    else
      if (p12.y () > 0)
        if (- p12.x () > p12.y ())
          lshift.set ((p12.x () * p12.y ()) * fact,
                      - (p12.x () * p12.x ()) * fact);
        else
          lshift.set (- (p12.y () * p12.y ()) * fact,
                      (p12.x () * p12.y ()) * fact);
      else
        if (p12.x () < p12.y ())
          lshift.set ((p12.x () * p12.y ()) * fact,
                      - (p12.x () * p12.x ()) * fact);
        else
          lshift.set ((p12.y () * p12.y ()) * fact,
                      - (p12.x () * p12.y ()) * fact);
    std::vector<Pt2f> pc;
    Plateau *rpl = ict->plateau (0);
    float rplc = (rpl->internalEnd () + rpl->internalStart ()) / (2 * l12);
    pc.push_back (Pt2f (p1f.x () + p12.x () * rplc,
                        p1f.y () + p12.y () * rplc));
    for (int i = 1; i < initial_track_extent ; i++)
    {
      rpl = ict->plateau (i);
      if (rpl != NULL && rpl->reliable ())
      {
        rplc = (rpl->internalEnd () + rpl->internalStart ()) / (2 * l12);
        pc.push_back (Pt2f (p1f.x () + i * lshift.x () + p12.x () * rplc,
                            p1f.y () + i * lshift.y () + p12.y () * rplc));
      }
      rpl = ict->plateau (-i);
      if (rpl != NULL && rpl->reliable ())
      {
        rplc = (rpl->internalEnd () + rpl->internalStart ()) / (2 * l12);
        pc.push_back (Pt2f (p1f.x () - i * lshift.x () + p12.x () * rplc,
                            p1f.y () - i * lshift.y () + p12.y () * rplc));
      }
    }
    if ((int) (pc.size ()) > initial_track_extent)
    {
      alignInput (pc);

      // redetection using aligned stroke
      detect (0);
    }
  }
  if (fct != NULL)
  {
    if (fstatus == RESULT_FAIL_NO_CONSISTENT_SEQUENCE) return NULL;
    if (shift_length_pruning
        && (fct->relativeShiftLength () > max_shift_length))
    {
      fct->setStatus (RESULT_FAIL_TOO_HECTIC_PLATEAUX);
      fstatus = RESULT_FAIL_TOO_HECTIC_PLATEAUX;
      return NULL;
    }
    if (density_pruning
        && fct->nbHoles () * 100 > fct->spread () * (100 - min_density))
    {
      fct->setStatus (RESULT_FAIL_TOO_SPARSE_PLATEAUX);
      fstatus = RESULT_FAIL_TOO_SPARSE_PLATEAUX;
      return NULL;
    }
  }
  return (fct);
}


void CTrackDetector::detect (int exlimit)
{
  Pt2i p1, p2;
  getInputStroke (p1, p2, exlimit != 0);
  Vr2f p12 (csize * (p2.x () - p1.x ()), csize * (p2.y () - p1.y ()));
  Pt2f p1f (csize * (p1.x () + 0.5f), csize * (p1.y () + 0.5f));
  float l12 = (float) sqrt (p12.x () * p12.x () + p12.y () * p12.y ());
  Vr2f dss_pos (p1.x () + (p2.x () - p1.x ()) * 0.5f,
                p1.y () + (p2.y () - p1.y ()) * 0.5f);
  Vr2i dss_n (p1.vectorTo (p2));
  if (dss_n.x () < 0) dss_n.invert ();
  float valc = dss_n.x () * dss_pos.x () + dss_n.y () * dss_pos.y ();
  int scan0_shift = (int) (valc < 0.0f ? valc - 0.5f : valc + 0.5f);

  // Creates adaptive directional scanners for point cloud and display
  DirectionalScanner *ds = scanp.getScanner (
    Pt2i (p1.x () * subdiv + subdiv / 2, p1.y () * subdiv + subdiv / 2),
    Pt2i (p2.x () * subdiv + subdiv / 2, p2.y () * subdiv + subdiv / 2),
    true);
  ds->releaseClearance ();
  DirectionalScanner *disp = discanp.getScanner (p1, p2, true);

  // Gets the central scan of the point cloud
  std::vector<Pt2i> pix;
  int nbp = ds->first (pix);
  for (int i = 0; nbp != 0 && i < subdiv / 2; i++)
    nbp = ds->nextOnRight (pix);
  nbp = 1;
  for (int i = 0; nbp != 0 && i < subdiv - 1 - subdiv / 2; i++)
    nbp = ds->nextOnLeft (pix);
  if (pix.empty ())
  {
    if (exlimit != 0)
      istatus = RESULT_FAIL_NO_AVAILABLE_SCAN;
    else fstatus = RESULT_FAIL_NO_AVAILABLE_SCAN;
    delete ds;
    delete disp;
    return;
  }

  // Gets the central scan for display
  std::vector<Pt2i> dispix;
  disp->first (dispix);

  // Gets and sorts scanned points by distance to first stroke point
  std::vector<Pt2f> cpts;
  std::vector<Pt2i>::iterator it = pix.begin ();
  while (it != pix.end ())
  {
    std::vector<Pt3f> ptcl;
    if (! ptset->collectPoints (ptcl, it->x (), it->y ())) out_count ++;
    std::vector<Pt3f>::iterator pit = ptcl.begin ();
    while (pit != ptcl.end ())
    {
      Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
      cpts.push_back (Pt2f (pcl.scalarProduct (p12) / l12, pit->z ()));
      pit ++;
    }
    it ++;
  }
  sort (cpts.begin (), cpts.end (), compIFurther);

  // Detects the central plateau
  CarriageTrack *ct = new CarriageTrack ();
  ct->setDetectionSeed (p1, p2, csize);
  if (exlimit != 0) ict = ct;
  else fct = ct;
  Plateau *cpl = new Plateau (&pfeat, scan0_shift);
  bool success = cpl->detect (cpts);
  if ((! success) && (! cpl->noOptimalHeight ()))
  {
    Plateau *cpl2 = new Plateau (&pfeat, scan0_shift);
    success = cpl2->detect (cpts, false, cpl->getMinHeight ());
    if (success)
    {
      // Keeps solution which is better or nearer to optimal width
      float dw = cpl->estimatedWidth () - pfeat.startLength ();
      if (dw < 0.0f) dw = - dw;
      float dw2 = cpl2->estimatedWidth () - pfeat.startLength ();
      if (dw2 < 0.0f) dw2 = - dw2;
      if (cpl->getStatus () == Plateau::PLATEAU_RES_OPTIMAL_HEIGHT_UNDER_USED
          && dw <= dw2)
      {
        cpl->acceptResult ();
        delete cpl2;
      }
      else
      {
        delete cpl;
        cpl = cpl2;
      }
    }
    else delete cpl2;
  }
  if (profileRecordOn) ct->start (cpl, dispix, cpts,
                                  scanp.isLastScanReversed ());
  else ct->start (cpl, dispix, scanp.isLastScanReversed ());
  if (success) ct->accept (0);
  else
  {
    ct->setStatus (RESULT_FAIL_NO_CENTRAL_PLATEAU);
    if (exlimit != 0)
      istatus = RESULT_FAIL_NO_CENTRAL_PLATEAU;
    else fstatus = RESULT_FAIL_NO_CENTRAL_PLATEAU;
    delete ds;
    delete disp;
    return;
  }

  // Updates reference pattern and detects next plateaux on each side
  if (cpl->bounded ()) initial_unbounded = false;
  initial_refs = cpl->internalStart ();
  initial_refe = cpl->internalEnd ();
  initial_refh = cpl->getMinHeight ();
  DirectionalScanner *ds2 = ds->getCopy ();
  DirectionalScanner *disp2 = disp->getCopy ();

  resetRegisters (cpl->reliable (),
                  cpl->estimatedCenter (), cpl->getMinHeight ());
  track (true, scanp.isLastScanReversed (), exlimit,
         ds, disp, p1f, p12, l12, initial_refs, initial_refe, initial_refh);
  bool firstUnbounded = initial_unbounded;

  resetRegisters (cpl->reliable (),
                  cpl->estimatedCenter (), cpl->getMinHeight ());
  track (false, scanp.isLastScanReversed (), exlimit,
         ds2, disp2, p1f, p12, l12, initial_refs, initial_refe, initial_refh);

  // second chance for first side if the central plateau was not bounded
  //   and a bound was found on the second side.
  if (firstUnbounded && ! initial_unbounded)
  {
    resetRegisters (cpl->reliable (),
                    cpl->estimatedCenter (), cpl->getMinHeight ());
    
    track (true, scanp.isLastScanReversed (), exlimit,
           ds, disp, p1f, p12, l12, initial_refs, initial_refe, initial_refh);
  }
  if (pfeat.tailMinSize () != 0 && ct->prune (pfeat.tailMinSize ()))
  {
    ct->setStatus (RESULT_FAIL_NO_CONSISTENT_SEQUENCE);
    if (exlimit != 0) istatus = RESULT_FAIL_NO_CONSISTENT_SEQUENCE;
    else fstatus = RESULT_FAIL_NO_CONSISTENT_SEQUENCE;
  }
  delete ds;
  delete disp;
}


void CTrackDetector::detect ()
{
  Pt2i p1, p2;
  getInputStroke (p1, p2);
  Vr2f p12 (csize * (p2.x () - p1.x ()), csize * (p2.y () - p1.y ()));
  Pt2f p1f (csize * (p1.x () + 0.5f), csize * (p1.y () + 0.5f));
  float l12 = (float) sqrt (p12.x () * p12.x () + p12.y () * p12.y ());
  Vr2f dss_pos (p1.x () + (p2.x () - p1.x ()) * 0.5f,
                p1.y () + (p2.y () - p1.y ()) * 0.5f);
  Vr2i dss_n (p1.vectorTo (p2));
  if (dss_n.x () < 0) dss_n.invert ();
  float valc = dss_n.x () * dss_pos.x () + dss_n.y () * dss_pos.y ();
  int scan0_shift = (int) (valc < 0.0f ? valc - 0.5f : valc + 0.5f);

  // Creates adaptive directional scanners for point cloud and display
  DirectionalScanner *ds = scanp.getScanner (
    Pt2i (p1.x () * subdiv + subdiv / 2, p1.y () * subdiv + subdiv / 2),
    Pt2i (p2.x () * subdiv + subdiv / 2, p2.y () * subdiv + subdiv / 2),
    true);
  ds->releaseClearance ();
  DirectionalScanner *disp = discanp.getScanner (p1, p2, true);

  // Gets the central scan of the point cloud
  std::vector<Pt2i> pix;
  int nbp = ds->first (pix);
  for (int i = 0; nbp != 0 && i < subdiv / 2; i++)
    nbp = ds->nextOnRight (pix);
  nbp = 1;
  for (int i = 0; nbp != 0 && i < subdiv - 1 - subdiv / 2; i++)
    nbp = ds->nextOnLeft (pix);
  if (pix.empty ())
  {
    fstatus = RESULT_FAIL_NO_AVAILABLE_SCAN;
    delete ds;
    delete disp;
    return;
  }

  // Gets the central scan for display
  std::vector<Pt2i> dispix;
  disp->first (dispix);

  // Gets and sorts scanned points by distance to first stroke point
  std::vector<Pt2f> cpts;
  std::vector<Pt2i>::iterator it = pix.begin ();
  while (it != pix.end ())
  {
    std::vector<Pt3f> ptcl;
    if (! ptset->collectPoints (ptcl, it->x (), it->y ())) out_count ++;
    std::vector<Pt3f>::iterator pit = ptcl.begin ();
    while (pit != ptcl.end ())
    {
      Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
      cpts.push_back (Pt2f (pcl.scalarProduct (p12) / l12, pit->z ()));
      pit ++;
    }
    it ++;
  }
  sort (cpts.begin (), cpts.end (), compIFurther);

  // Creates the carriage track
  fct = new CarriageTrack ();
  fct->setDetectionSeed (p1, p2, csize);

  float *tests = new float[NB_SIDE_TRIALS * 2];
  for (int i = 0; i < NB_SIDE_TRIALS; i ++)
  {
    tests[2 * i] = pfeat.firstPlateauSearchDistance () * (i + 1);
    tests[2 * i + 1] = - pfeat.firstPlateauSearchDistance () * (i + 1);
  }
  Plateau *cpl = new Plateau (&pfeat, scan0_shift);
  bool found = (pfeat.isNetBuildOn () ?
    cpl->track (cpts, NULL, 0, 0.0f, l12) :
    cpl->track (cpts, 0.0f, l12, 0.0f, 0.0f, 0));
  for (int ptest = 0; ptest != NB_SIDE_TRIALS * 2; ptest++)
  {
    Plateau *cpl2 = new Plateau (&pfeat, scan0_shift);
    bool success = (pfeat.isNetBuildOn () ?
      cpl2->track (cpts, NULL, 0, tests[ptest], l12) :
      cpl2->track (cpts, 0.0f, l12, 0.0f, tests[ptest], 0));
    if (success) found = true;
    if (success && cpl2->thinerThan (cpl))
    {
      delete cpl;
      cpl = cpl2;
    }
    else delete cpl2;
  }
  if (profileRecordOn) fct->start (cpl, dispix, cpts,
                                   scanp.isLastScanReversed ());
  else fct->start (cpl, dispix, scanp.isLastScanReversed ());
  if (pfeat.isNetBuildOn ())
  {
    if (cpl->consistentWidth ()) fct->accept (0);
  }
  else if (found) fct->accept (0);
  if (! found)
  {
    fct->setStatus (RESULT_FAIL_NO_CENTRAL_PLATEAU);
    fstatus = RESULT_FAIL_NO_CENTRAL_PLATEAU;
    delete ds;
    delete disp;
    return;
  }

  // Updates reference pattern and detects next plateaux on each side
  if (cpl->bounded ()) initial_unbounded = false;
  initial_ref = 0;
  initial_refs = cpl->internalStart ();
  initial_refe = cpl->internalEnd ();
  initial_refh = cpl->getMinHeight ();
  DirectionalScanner *ds2 = ds->getCopy ();
  DirectionalScanner *disp2 = disp->getCopy ();

  resetRegisters (cpl->reliable (),
                  cpl->estimatedCenter (), cpl->getMinHeight ());
  if (pfeat.isNetBuildOn ())
    track (true, scanp.isLastScanReversed (), 0, ds, disp, p1f, p12, l12, cpl);
  else track (true, scanp.isLastScanReversed (), 0, ds, disp,
              p1f, p12, l12, initial_refs, initial_refe, initial_refh);
  bool firstUnbounded = initial_unbounded;

  resetRegisters (cpl->reliable (),
                  cpl->estimatedCenter (), cpl->getMinHeight ());
  if (pfeat.isNetBuildOn ())
    track (false, scanp.isLastScanReversed (), 0, ds2, disp2,
           p1f, p12, l12, fct->plateau (initial_ref));
  else track (false, scanp.isLastScanReversed (), 0, ds2, disp2,
              p1f, p12, l12, initial_refs, initial_refe, initial_refh);

  // second chance for first side if the central plateau was not bounded
  //   and a bound was found on the second side.
  if (firstUnbounded && ! initial_unbounded)
  {
    resetRegisters (cpl->reliable (),
                    cpl->estimatedCenter (), cpl->getMinHeight ());
    if (pfeat.isNetBuildOn ())
      track (true, scanp.isLastScanReversed (), 0, ds, disp,
             p1f, p12, l12, fct->plateau (initial_ref));
    else track (true, scanp.isLastScanReversed (), 0, ds, disp,
                p1f, p12, l12, initial_refs, initial_refe, initial_refh);
  }
  if (pfeat.tailMinSize () != 0 && fct->prune (pfeat.tailMinSize ()))
  {
    fct->setStatus (RESULT_FAIL_NO_CONSISTENT_SEQUENCE);
    fstatus = RESULT_FAIL_NO_CONSISTENT_SEQUENCE;
  }
  delete ds;
  delete disp;
}


void CTrackDetector::track (bool onright, bool reversed, int exlimit,
                            DirectionalScanner *ds, DirectionalScanner *disp,
                            Pt2f p1f, Vr2f p12, float l12,
                            float refs, float refe, float refh)
{
  bool search = true;
  int nbfail = 0;
  int num = (onright ? -1 : 1);
  if (onright) exlimit = - exlimit;
  CarriageTrack *ct = (exlimit != 0 ? ict : fct);
  ct->clear (onright);
  int confdist = 1;
  Pt2i ss_p1, ss_p2;
  getInputStroke (ss_p1, ss_p2, exlimit != 0);
  Vr2i ss_p12 (ss_p1.vectorTo (ss_p2));
  float ss_l12 = (float) sqrt (ss_p12.norm2 ());
  Vr2i dss_n (ss_p12);
  if (dss_n.x () < 0) dss_n.invert ();
  while (search && num != exlimit)
  {
    // Adaptive scan recentering on reference pattern
    float pcenter = (refs + refe) / 2;
    float posx = ss_p1.x () + (ss_p12.x () / ss_l12) * pcenter / csize;
    float posy = ss_p1.y () + (ss_p12.y () / ss_l12) * pcenter / csize;
    float valc = dss_n.x () * posx + dss_n.y () * posy;
    int scan_shift = (int) (valc < 0.0f ? valc - 0.5f : valc + 0.5f);
    disp->bindTo (dss_n.x (), dss_n.y (), scan_shift);
    ds->bindTo (dss_n.x (), dss_n.y (), scan_shift * subdiv + subdiv / 2);

    // Collects next scan points and sorts them by distance
    std::vector<Pt2i> pix;
    std::vector<Pt2i> dispix;
    if ((onright && ! reversed) || (reversed && ! onright))
      disp->nextOnRight (dispix);
    else disp->nextOnLeft (dispix);
    if (dispix.empty ()) search = false;
    else
      for (int i = 0; search && i < subdiv; i++)
        if ((onright && ! reversed) || (reversed && ! onright))
        {
          if (ds->nextOnRight (pix) == 0) search = false;
        }
        else if (ds->nextOnLeft (pix) == 0) search = false;
    if (pix.empty ()) search = false;
    else
    {
      std::vector<Pt2f> pts;
      std::vector<Pt2i>::iterator it = pix.begin ();
      while (it != pix.end ())
      {
        std::vector<Pt3f> ptcl;
        if (! ptset->collectPoints (ptcl, it->x (), it->y ())) out_count ++;
        std::vector<Pt3f>::iterator pit = ptcl.begin ();
        while (pit != ptcl.end ())
        {
          Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
          pts.push_back (Pt2f (pcl.scalarProduct (p12) / l12, pit->z ()));
          pit ++;
        }
        it ++;
      }
      sort (pts.begin (), pts.end (), compIFurther);

      // Detects the plateau and updates the track section
      Plateau *pl = new Plateau (&pfeat, scan_shift);
      pl->track (pts, refs, refe, refh, 0.0f, confdist);
      if (pl->getStatus () != Plateau::PLATEAU_RES_OK)
      {
        Plateau *pl2 = new Plateau (&pfeat, scan_shift);
        pl2->track (pts, refs, refe, refh,
                    pfeat.plateauSearchDistance (), confdist);
        if (pl2->getStatus () != Plateau::PLATEAU_RES_OK)
        {
          delete pl2;
          Plateau *pl3 = new Plateau (&pfeat, scan_shift);
          pl3->track (pts, refs, refe, refh,
                      -pfeat.plateauSearchDistance (), confdist);
          if (pl3->getStatus () != Plateau::PLATEAU_RES_OK)
            delete pl3;
          else
          {
            delete pl;
            pl = pl3;
          }
        }
        else
        {
          delete pl;
          pl = pl2;
        }
      }
      if (profileRecordOn) ct->add (onright, pl, dispix, pts);
      else ct->add (onright, pl, dispix);

      // Ends tracking after a given amount of failures (point lacks apart).
      if (pl->getStatus () == Plateau::PLATEAU_RES_OK) nbfail = 0;
      else if (density_insensitive || pl->hasEnoughPoints ())
        if (++nbfail >= plateau_lack_tolerance) search = false;
      // no lack count increment otherwise

      // Manages start bounds setting
      if (search && initial_unbounded)
      {
        if (pl->bounded () && pl->isAccepted ())
        {
          initial_unbounded = false;
          initial_refs = pl->internalStart ();
          initial_refe = pl->internalEnd ();
        }
        else
          if (num == NOBOUNDS_TOLERANCE || num == - NOBOUNDS_TOLERANCE)
          {
            ct->setStatus (RESULT_FAIL_NO_BOUNDS);
            if (exlimit != 0) istatus = RESULT_FAIL_NO_BOUNDS;
            else fstatus = RESULT_FAIL_NO_BOUNDS;
            search = false;
          }
      }

      if (search)
      {
        // Estimates deviation and slope
        pl->setDeviation (updatePosition (pl->possible (),
                                          pl->estimatedCenter ()));
        pl->setSlope (updateHeight (pl->consistentHeight (),
                                    pl->getMinHeight ()));

        // Updates reference pattern for next plateau detection
        if (pl->possible ())
        {
          refs = pl->estimatedStart ();
          refe = pl->estimatedEnd ();
        }
        if (pfeat.isDeviationPredictionOn () || ! pl->possible ())
        {
          refs += pl->estimatedDeviation ();
          refe += pl->estimatedDeviation ();
        }
        if (pl->consistentHeight ()) refh = pl->getMinHeight ();
        if (pfeat.isSlopePredictionOn () || ! pl->consistentHeight ())
          refh += pl->estimatedSlope ();
      }

      // Conditionally accepts the plateau and former consistent ones.
      if (pl->getStatus () == Plateau::PLATEAU_RES_OK && pl->reliable ())
      {
        ct->accept (num);
        float c1 = pl->estimatedCenter ();
        Plateau *lpl = ct->plateau (num < 0 ? num + confdist : num - confdist);
        float dc = (lpl->estimatedCenter () - c1) / confdist;
        for (int i = 1; i < confdist; i ++)
        {
          int locnum = (num < 0 ? num + i : num - i);
          lpl = ct->plateau (locnum);
          if (lpl->contains (c1 + dc * i)) ct->accept (locnum);
        }
        confdist = 1;
      }
      else confdist ++;
    }
    num += (onright ? -1 : 1);
  }
}


// AMRELnet version
void CTrackDetector::track (bool onright, bool reversed, int exlimit,
                            DirectionalScanner *ds, DirectionalScanner *disp,
                            Pt2f p1f, Vr2f p12, float l12, Plateau *ref)
{
  bool search = true;
  int nbfail = 0;
  int num = (onright ? -1 : 1);
  if (onright) exlimit = - exlimit;
  CarriageTrack *ct = (exlimit != 0 ? ict : fct);
  ct->clear (onright);
  int confdist = 1;
  Pt2i ss_p1, ss_p2;
  getInputStroke (ss_p1, ss_p2, exlimit != 0);
  Vr2i ss_p12 (ss_p1.vectorTo (ss_p2));
  float ss_l12 = (float) sqrt (ss_p12.norm2 ());
  Vr2i dss_n (ss_p12);
  if (dss_n.x () < 0) dss_n.invert ();
  while (search && num != exlimit)
  {
    // Adaptive scan recentering on reference pattern
    float pcenter = ref->estimatedCenter ();
    float posx = ss_p1.x () + (ss_p12.x () / ss_l12) * pcenter / csize;
    float posy = ss_p1.y () + (ss_p12.y () / ss_l12) * pcenter / csize;
    float valc = dss_n.x () * posx + dss_n.y () * posy;
    int scan_shift = (int) (valc < 0.0f ? valc - 0.5f : valc + 0.5f);
    disp->bindTo (dss_n.x (), dss_n.y (), scan_shift);
    ds->bindTo (dss_n.x (), dss_n.y (), scan_shift * subdiv + subdiv / 2);

    // Collects next scan points and sorts them by distance
    std::vector<Pt2i> pix;
    std::vector<Pt2i> dispix;
    if ((onright && ! reversed) || (reversed && ! onright))
      disp->nextOnRight (dispix);
    else disp->nextOnLeft (dispix);
    if (dispix.empty ()) search = false;
    else
      for (int i = 0; search && i < subdiv; i++)
        if ((onright && ! reversed) || (reversed && ! onright))
        {
          if (ds->nextOnRight (pix) == 0) search = false;
        }
        else if (ds->nextOnLeft (pix) == 0) search = false;
    if (pix.empty ()) search = false;
    else
    {
      std::vector<Pt2f> pts;
      std::vector<Pt2i>::iterator it = pix.begin ();
      while (it != pix.end ())
      {
        std::vector<Pt3f> ptcl;
        if (! ptset->collectPoints (ptcl, it->x (), it->y ())) out_count ++;
        std::vector<Pt3f>::iterator pit = ptcl.begin ();
        while (pit != ptcl.end ())
        {
          Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
          pts.push_back (Pt2f (pcl.scalarProduct (p12) / l12, pit->z ()));
          pit ++;
        }
        it ++;
      }

      // Detects the plateau and updates the track section
      Plateau *pl = new Plateau (&pfeat, scan_shift);
      sort (pts.begin (), pts.end (), compIFurther);
      pl->track (pts, ref, confdist, 0.0f, 0.0f);
      if (pl->getStatus () != Plateau::PLATEAU_RES_OK)
      {
        float *retests = new float[NB_SIDE_TRIALS * 2];
        for (int i = 0; i < NB_SIDE_TRIALS; i ++)
        {
          retests[2 * i] = pfeat.plateauSearchDistance () * (i + 1);
          retests[2 * i + 1] = - pfeat.plateauSearchDistance () * (i + 1);
        }
        bool tracking = true;
        for (int i = 0; tracking && i < NB_SIDE_TRIALS * 2; i++)
        {
          Plateau *pl2 = new Plateau (&pfeat, scan_shift);
          pl2->track (pts, ref, confdist, retests[i], 0.0f);
          if (pl2->getStatus () > pl->getStatus ())
          {
            delete pl;
            pl = pl2;
            if (pl->getStatus () == Plateau::PLATEAU_RES_OK) tracking = false;
          }
          else delete pl2;
        }
      }
      if (profileRecordOn) ct->add (onright, pl, dispix, pts);
      else ct->add (onright, pl, dispix);

      // Ends tracking when meeting an obstacle.
      if (pfeat.isNetBuildOn () && pl->impassable ()) search = false;

      // Ends tracking after a given amount of failures (point lacks apart).
      if (pl->getStatus () == Plateau::PLATEAU_RES_OK) nbfail = 0;
      else if (density_insensitive || pl->hasEnoughPoints ())
        if (++nbfail >= plateau_lack_tolerance) search = false;
      // no lack count increment otherwise

      // Manages start bounds setting
      if (search && initial_unbounded)
      {
        if (pl->bounded () && pl->isAccepted ())
        {
          initial_unbounded = false;
          initial_ref = num;
        }
        else
          if (num == NOBOUNDS_TOLERANCE || num == - NOBOUNDS_TOLERANCE)
          {
            ct->setStatus (RESULT_FAIL_NO_BOUNDS);
            if (exlimit != 0) istatus = RESULT_FAIL_NO_BOUNDS;
            else fstatus = RESULT_FAIL_NO_BOUNDS;
            search = false;
          }
      }

      if (search)
      {
        // Estimates deviation and slope
        pl->setDeviation (updatePosition (pl->possible (),
                                          pl->estimatedCenter ()));
        pl->setSlope (updateHeight (pl->consistentHeight (),
                                    pl->getMinHeight ()));
      }
      //if (pl->possible ()) ref = pl;
      ref = pl;

      // Conditionally accepts the plateau and former consistent ones.
      if (pl->getStatus () == Plateau::PLATEAU_RES_OK && pl->reliable ())
      {
        if (connect_on)
        {
          int locnum = num;
          Plateau *hpl = pl;
          Plateau *fpl = ct->plateau (num < 0 ? num + confdist
                                              : num - confdist);
          int cdist = confdist;
          bool cleaning = true;
          while (cleaning && cdist != 1)
          {
            locnum += (locnum < 0 ? 1 : -1);
            Plateau *lpl = ct->plateau (locnum);
            cleaning = lpl->fit (hpl, fpl, cdist);
            hpl = lpl;
            cdist --;
          }
          if (cleaning)
          {
            for (int i = 1; i < confdist; i++)
              ct->accept (num < 0 ? num + i : num - i);
            ct->accept (num);
          }
          else
          {
            search = false;
            ct->setStatus (RESULT_FAIL_DISCONNECT);
            if (exlimit != 0) istatus = RESULT_FAIL_DISCONNECT;
            else fstatus = RESULT_FAIL_DISCONNECT;
          }
        }
        else
        {
          int locnum = num;
          Plateau *lpl = NULL;
          ct->accept (num);
          float c1 = pl->estimatedCenter ();
          lpl = ct->plateau (num < 0 ? num + confdist
                                              : num - confdist);
          float dc = (lpl->estimatedCenter () - c1) / confdist;
          bool ok = true;
          for (int i = 1; ok && i < confdist; i ++)
          {
            locnum += (locnum < 0 ? 1 : -1);
            lpl = ct->plateau (locnum);
            if (lpl->contains (c1 + dc * i)) ct->accept (locnum);
          }
        }
        confdist = 1;
        if (! pl->isConnectedTo (ct->plateau (num < 0 ? num + 1 : num - 1)))
        {
          ct->setStatus (RESULT_FAIL_DISCONNECT);
          if (exlimit != 0) istatus = RESULT_FAIL_DISCONNECT;
          else fstatus = RESULT_FAIL_DISCONNECT;
          search = false;
        }
      }
      else
      {
        confdist ++;
      }
    }
    num += (onright ? -1 : 1);
  }
}


void CTrackDetector::resetRegisters (bool ok, float pos, float ht)
{
  for (int i = 0; i < unstab_nb; i++)
  {
    spos[i] = 0.0f;
    epos[i] = 0.0f;
    spok[i] = false;
    epok[i] = false;
  }
  for (int i = 1; i < posht_nb; i++)
  {
    lpok[i] = false;
    lpos[i] = 0.0f;
    lhok[i] = false;
    lht[i] = 0.0f;
  }
  lpok[0] = ok;
  lpos[0] = pos;
  lhok[0] = ok;
  lht[0] = ht;
}


float CTrackDetector::updatePosition (bool ok, float pos)
{
  int nbok = 0, last = -1, first = -1;
  for (int i = posht_nb - 1; i > 0; i--)
  {
    lpok[i] = lpok[i-1];
    lpos[i] = lpos[i-1];
    if (lpok[i])
    {
      if (nbok != 0) last = i;
      else first = i;
      nbok ++;
    }
  }
  lpok[0] = ok;
  lpos[0] = pos;
  if (ok)
  {
    if (nbok != 0) last = 0;
    else first = 0;
    nbok ++;
  }

  if (nbok <= 1) return 0.0f;
  float dtrend = 0.0f, trend = (lpos[last] - lpos[first]) / (first - last);
  if (nbok == 2) return (trend);
  int last2 = -1;
  for (int i = first - 1; i > last; i --)
  {
    if (lpok[i])
    {
      if (dtrend == 0.0f)
      {
        dtrend = (lpos[last] - lpos[i]) / (i - last) - trend;
        last2 = i;
      }
      else if (((lpos[last] - lpos[i]) / (i - last) - trend) * dtrend < 0.0f)
        return (trend);
      else last2 = i;
    }
  }
  return ((lpos[last] - lpos[last2]) / (last2 - last));
}


float CTrackDetector::updateHeight (bool ok, float ht)
{
  int nbok = 0, last = -1, first = -1;
  for (int i = posht_nb - 1; i > 0; i--)
  {
    lhok[i] = lhok[i-1];
    lht[i] = lht[i-1];
    if (lhok[i])
    {
      if (nbok != 0) last = i;
      else first = i;
      nbok ++;
    }
  }
  lhok[0] = ok;
  lht[0] = ht;
  if (ok)
  {
    if (nbok != 0) last = 0;
    else first = 0;
    nbok ++;
  }

  if (nbok <= 1) return 0.0f;
  float dtrend = 0.0f, trend = (lht[last] - lht[first]) / (first - last);
  if (nbok == 2) return (trend);
  int last2 = -1;
  for (int i = first - 1; i > last; i --)
  {
    if (lhok[i])
    {
      if (dtrend == 0.0f)
      {
        dtrend = (lht[last] - lht[i]) / (i - last) - trend;
        last2 = i;
      }
      else if (((lht[last] - lht[i]) / (i - last) - trend) * dtrend < 0.0f)
        return (trend);
      else last2 = i;
    }
  }
  return (lht[last] - lht[last2]) / (last2 - last);
}


int CTrackDetector::boundsStability (float slast, float elast,
                                     bool sok, bool eok, float trw)
{
  for (int i = unstab_nb - 1; i > 0; i--)
  {
    spos[i] = spos[i-1];
    epos[i] = epos[i-1];
    spok[i] = spok[i-1];
    epok[i] = epok[i-1];
  }
  spos[0] = slast;
  epos[0] = elast;
  spok[0] = sok;
  epok[0] = eok;

  if (trw > pfeat.maxLength ())
  {
    float spath = 0.0f, epath = 0.0f;
    int snok = (sok ? 1 : 0), enok = (eok ? 1 : 0);
    for (int i = unstab_nb - 1; i > 0; i--)
    {
      spath += spos[i-1] < spos[i] ? spos[i] - spos[i-1] : spos[i-1] - spos[i];
      epath += epos[i-1] < epos[i] ? epos[i] - epos[i-1] : epos[i-1] - epos[i];
      if (spok[i]) snok ++;
      if (epok[i]) enok ++;
    }
    if (spath - epath > LN_UNSTAB * unstab_nb) return -1;
    if (epath - spath > LN_UNSTAB * unstab_nb) return 1;
  }
  return 0;
}


bool CTrackDetector::compIFurther (Pt2f p1, Pt2f p2)
{
  return (floor (p2.x () * 1000) > floor (p1.x () * 1000)
          || (floor (p2.x () * 1000) == floor (p1.x () * 1000)
              && floor (p2.y () * 1000) > floor (p1.y () * 1000)));
}

bool CTrackDetector::compLFurther (Pt3f p1, Pt3f p2)
{
  return (floor (p2.x () * 1000) > floor (p1.x () * 1000)
          || (floor (p2.x () * 1000) == floor (p1.x () * 1000)
              && floor (p2.y () * 1000) > floor (p1.y () * 1000)));
}


void CTrackDetector::incPlateauLackTolerance (int dir)
{
  setPlateauLackTolerance (plateau_lack_tolerance + dir);
}

void CTrackDetector::setPlateauLackTolerance (int nb)
{
  plateau_lack_tolerance = nb;
  if (plateau_lack_tolerance < 0) plateau_lack_tolerance = 0;
}


void CTrackDetector::incMaxShiftLength (int inc)
{
  setMaxShiftLength (max_shift_length + inc * POS_INCR);
}

void CTrackDetector::setMaxShiftLength (float val)
{
  max_shift_length = val;
  if (max_shift_length < 0.0f) max_shift_length = 0.0f;
}


void CTrackDetector::incMinDensity (int inc)
{
  setMinDensity (min_density + inc);
}

void CTrackDetector::setMinDensity (int val)
{
  min_density = val;
  if (min_density > 100) min_density = 100;
  else if (min_density < 0) min_density = 0;
}


void CTrackDetector::alignInput (const std::vector<Pt2f> &pts)
{
  // Get coordinates expectation
  std::vector<Pt2f>::const_iterator it = pts.begin ();
  float xmin = it->x ();
  float xmax = xmin;
  float ymin = it->y ();
  float ymax = ymin;
  float xm = xmin;
  float ym = ymin;
  it ++;
  while (it != pts.end ())
  {
    xm += it->x ();
    ym += it->y ();
    if (it->x () < xmin) xmin = it->x ();
    else if (it->x () > xmax) xmax = it->x ();
    if (it->y () < ymin) ymin = it->y ();
    else if (it->y () > ymax) ymax = it->y ();
    it ++;
  }
  xm /= (int) (pts.size ());
  ym /= (int) (pts.size ());

  // Get coordiantes variance
  float xv = 0.0f, yv = 0.0f, xyv = 0.0f;
  do
  {
    it --;
    xv += (it->x () - xm) * (it->x () - xm);
    yv += (it->y () - ym) * (it->y () - ym);
    xyv += (it->x () - xm) * (it->y () - ym);
  }
  while (it != pts.begin ());
  xv /= (int) (pts.size ());
  yv /= (int) (pts.size ());
  xyv /= (int) (pts.size ());

  // Get linear approx ax - by = c
  float a = 1.0f, b = 1.0f, c = 0.0f;
  if (xmax - xmin > ymax - ymin)
  {
    a = xyv / xv;
    c = a * xm - ym;
  }
  else
  {
    b = xyv / yv;
    c = xm - b * ym;
  }

  // Get new stroke
  float d = b * it->x () + a * it->y ();
  float den = a * a + b * b;
  float x = (a * c + b * d) / den;
  float y = (a * d - b * c) / den;
  den = sqrt (den);
  int dir = (a * (ip2.x () - ip1.x ()) - b * (ip2.y () - ip1.y ()) < 0 ?
             -1 : 1);
  //float length = l12 / (2 * dir * den);
  float length = CarriageTrack::MAX_WIDTH * dir;
  fp1.set ((int) ((x - a * length) / csize),
           (int) ((y + b * length) / csize));
  fp2.set ((int) ((x + a * length) / csize),
           (int) ((y - b * length) / csize));
  Vr2f p12 (csize * (fp2.x () - fp1.x ()), csize * (fp2.y () - fp1.y ()));
}


void CTrackDetector::testScanShiftExtraction () const
{
  Vr2f vev[] = {Vr2f (2, 1), Vr2f (1, 2), Vr2f (-1, 2), Vr2f (-2, 1),
                Vr2f (-2, -1), Vr2f (-1, -2), Vr2f (1, -2), Vr2f (2, -1)};
  Vr2f lshift;
  float fact = 0.2f;
  for (int i = 0; i < 8; i++)
  {
    if (vev[i].x () > 0)
      if (vev[i].y () > 0)
        if (vev[i].x () > vev[i].y ())
          lshift.set (- (vev[i].x () * vev[i].y ()) * fact,
                      (vev[i].x () * vev[i].x ()) * fact);
        else
          lshift.set (- (vev[i].y () * vev[i].y ()) * fact,
                      (vev[i].x () * vev[i].y ()) * fact);
      else
        if (vev[i].x () > - vev[i].y ())
          lshift.set (- (vev[i].x () * vev[i].y ()) * fact,
                      (vev[i].x () * vev[i].x ()) * fact);
        else
          lshift.set ((vev[i].y () * vev[i].y ()) * fact,
                      - (vev[i].x () * vev[i].y ()) * fact);
    else
      if (vev[i].y () > 0)
        if (- vev[i].x () > vev[i].y ())
          lshift.set ((vev[i].x () * vev[i].y ()) * fact,
                      - (vev[i].x () * vev[i].x ()) * fact);
        else
          lshift.set (- (vev[i].y () * vev[i].y ()) * fact,
                      (vev[i].x () * vev[i].y ()) * fact);
      else
        if (vev[i].x () < vev[i].y ())
          lshift.set ((vev[i].x () * vev[i].y ()) * fact,
                      - (vev[i].x () * vev[i].x ()) * fact);
        else
          lshift.set ((vev[i].y () * vev[i].y ()) * fact,
                      - (vev[i].x () * vev[i].y ()) * fact);
    // std::cout << "DIR = (" << vev[i].x () << ", " << vev[i].y ()
    //           << ") ---> (" << lshift.x () << ", " << lshift.y () << ")"
    //           << std::endl;
  }
}


void CTrackDetector::labelPoints (CarriageTrack *ct)
{
  if (! ct->isValid ()) return;

  Pt2i ctp1 (ct->getSeedStart ());
  Pt2i ctp2 (ct->getSeedEnd ());
  Pt2f p1f (csize * (ctp1.x () + 0.5f), csize * (ctp1.y () + 0.5f));
  Vr2f p12 (csize * (ctp2.x () - ctp1.x ()), csize * (ctp2.y () - ctp1.y ()));
  float l12 = (float) sqrt (p12.x () * p12.x () + p12.y () * p12.y ());
  int a = ctp2.x () - ctp1.x ();
  int b = ctp2.y () - ctp1.y ();
  if (a < 0.)
  {
    a = -a;
    b = -b;
  }
  DirectionalScanner *ds = scanp.getScanner (
    Pt2i (ctp1.x () * subdiv + subdiv / 2, ctp1.y () * subdiv + subdiv / 2),
    Pt2i (ctp2.x () * subdiv + subdiv / 2, ctp2.y () * subdiv + subdiv / 2),
    true);
  ds->releaseClearance ();

  std::vector<Pt2i> pix0;
  int nbp = ds->first (pix0);
  for (int i = 0; nbp != 0 && i < subdiv / 2; i++)
    nbp = ds->nextOnRight (pix0);
  nbp = 1;
  for (int i = 0; nbp != 0 && i < subdiv - 1 - subdiv / 2; i++)
    nbp = ds->nextOnLeft (pix0);

  Plateau *pl = ct->plateau (0);
  if (pl->isAccepted ())
  {
    std::vector<Pt3f> cpts;
    std::vector<int> tls;
    std::vector<int> lbs;
    std::vector<Pt2i>::iterator it = pix0.begin ();
    int labind = 0;
    while (it != pix0.end ())
    {
      std::vector<Pt3f> ptcl;
      ptset->collectPointsAndLabels (ptcl, tls, lbs, it->x (), it->y ());
      std::vector<Pt3f>::iterator pit = ptcl.begin ();
      while (pit != ptcl.end ())
      {
        Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
        cpts.push_back (Pt3f (pcl.scalarProduct (p12) / l12,
                              pit->z (), labind + 0.1f));
        pit ++;
        labind ++;
      }
      it ++;
    }
    sort (cpts.begin (), cpts.end (), compLFurther);
    int s_num = pl->startIndex (), e_num = pl->endIndex ();
    if ((int) cpts.size () > e_num)
    {
      std::vector<Pt3f>::iterator it = cpts.begin () + s_num;
      for (int i = s_num; i != e_num && it != cpts.end (); i++)
      {
        int ind = (int) (it->z ());
        ptset->labelAsTrack (tls[ind], lbs[ind]);
        it ++;
      }
    }
  }

  bool search = true;
  for (int i = -1; i >= - ct->getRightScanCount (); i--)
  {
    Plateau *pl = ct->plateau (i);
    ds->bindTo (a, b, pl->scanShift () * subdiv + subdiv / 2);
    std::vector<Pt2i> pix;
    for (int i = 0; search && i < subdiv; i++)
      if (scanp.isLastScanReversed ())
      {
        if (ds->nextOnLeft (pix) == 0) search = false;
      }
      else if (ds->nextOnRight (pix) == 0) search = false;

    if (pl->isAccepted ())
    {
      std::vector<Pt3f> cpts;
      std::vector<int> tls;
      std::vector<int> lbs;
      std::vector<Pt2i>::iterator it = pix.begin ();
      int labind = 0;
      while (it != pix.end ())
      {
        std::vector<Pt3f> ptcl;
        ptset->collectPointsAndLabels (ptcl, tls, lbs, it->x (), it->y ());
        std::vector<Pt3f>::iterator pit = ptcl.begin ();
        while (pit != ptcl.end ())
        {
          Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
          cpts.push_back (Pt3f (pcl.scalarProduct (p12) / l12,
                                pit->z (), labind + 0.1f));
          pit ++;
          labind ++;
        }
        it ++;
      }
      sort (cpts.begin (), cpts.end (), compLFurther);
      int s_num = pl->startIndex (), e_num = pl->endIndex ();
      if ((int) cpts.size () > e_num)
      {
        std::vector<Pt3f>::iterator it = cpts.begin () + s_num;
        for (int i = s_num; i != e_num && it != cpts.end (); i++)
        {
          int ind = (int) (it->z ());
          ptset->labelAsTrack (tls[ind], lbs[ind]);
          it ++;
        }
      }
    }
  }

  search = true;
  for (int i = 1; i <= ct->getLeftScanCount (); i++)
  {
    Plateau *pl = ct->plateau (i);
    ds->bindTo (a, b, pl->scanShift () * subdiv + subdiv / 2);
    std::vector<Pt2i> pix;
    for (int i = 0; search && i < subdiv; i++)
      if (scanp.isLastScanReversed ())
      {
        if (ds->nextOnRight (pix) == 0) search = false;
      }
      else if (ds->nextOnLeft (pix) == 0) search = false;

    if (pl->isAccepted ())
    {
      std::vector<Pt3f> cpts;
      std::vector<int> tls;
      std::vector<int> lbs;
      std::vector<Pt2i>::iterator it = pix.begin ();
      int labind = 0;
      while (it != pix.end ())
      {
        std::vector<Pt3f> ptcl;
        ptset->collectPointsAndLabels (ptcl, tls, lbs, it->x (), it->y ());
        std::vector<Pt3f>::iterator pit = ptcl.begin ();
        while (pit != ptcl.end ())
        {
          Vr2f pcl (pit->x () - p1f.x (), pit->y () - p1f.y ());
          cpts.push_back (Pt3f (pcl.scalarProduct (p12) / l12,
                                pit->z (), labind + 0.1f));
          pit ++;
          labind ++;
        }
        it ++;
      }
      sort (cpts.begin (), cpts.end (), compLFurther);
      int s_num = pl->startIndex (), e_num = pl->endIndex ();
      if ((int) cpts.size () > e_num)
      {
        std::vector<Pt3f>::iterator it = cpts.begin () + s_num;
        for (int i = s_num; i != e_num && it != cpts.end (); i++)
        {
          int ind = (int) (it->z ());
          ptset->labelAsTrack (tls[ind], lbs[ind]);
          it ++;
        }
      }
    }
  }
}
