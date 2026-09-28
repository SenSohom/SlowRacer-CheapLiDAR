"""
================================================================================
 wall_finder.py
================================================================================
Drop-in module: given 2-D lidar hits from the car, decide which physical wall
each hit belongs to (wall1 = outer/right, wall2 = inner/left), roughly where
that wall is, and track it confidently over time.

Pure Python 3, zero dependencies. See CONTEXT.md (shipped alongside this file)
for the full background, the track's geometry, the math derivation, and the
validated accuracy/speed numbers. This file is the runnable implementation.

CONVENTIONS
  wall1 = wall on the car's RIGHT while driving the lap (outer loop)
  wall2 = wall on the car's LEFT  while driving the lap (inner loop)
  scan  = list of (r, theta) hits. r = range, theta = bearing in radians,
          0 = straight ahead, +theta = LEFT (ROS LaserScan convention)
  section = 'straight' | 'left' | 'right' | 'U_left' | 'U_right'
          (which part of the repeating track the car is currently on;
           section_at() below looks this up from lap odometry)
  units = feet by default; pass WallFinder(units='m') for metres

QUICK USE
    wf, trk = WallFinder(), WallTracker(WallFinder(), which=1)
    section = section_at(odom_ft)                       # where on the lap the car is
    lab = wf.label_scan(scan, section)                   # label once per scan
    f1  = wf.find(scan, section, 1, lab)                 # WallFix or None
    f2  = wf.find(scan, section, 2, lab)                 # reuses the same labelling
    fix = trk.update(scan, section, lab)                 # confirmed + smoothed wall1

FILE MAP (search these banners to jump around)
  1. CONSTANTS & LAP TABLE      - track geometry baked in as data
  2. SECTION LOOKUP             - section_at(): where on the lap is the car
  3. GEOMETRY HELPERS           - small math shared by every rule below
  4. RULE: STRAIGHT SECTIONS    - which side of the car a wall line is on
  5. RULE: TURNS incl. U-TURNS  - inside/outside the turn's centre line
  6. RULE: LONE-HIT FALLBACK    - same rules for a single hit, no partner
  7. SCAN -> WALL PIECES        - splitting a scan into continuous wall segments
  8. CONFIDENCE VOTING          - turning many point-pair guesses into one label
  9. FULL-SCAN LABELLING        - label_scan(): ties 7+8 together
 10. WALL LOCATION              - find(): roughly where a labelled wall is
 11. TRACKING OVER TIME         - WallTracker: confirm + smooth across scans
 12. SENSOR ADAPTER             - from_laserscan(): ROS message -> scan
 13. SELF-TEST                  - run this file directly to sanity-check + time it
================================================================================
"""
import math, bisect
from collections import namedtuple
from itertools import chain
from operator import itemgetter


# ==============================================================================
# 1. CONSTANTS & LAP TABLE
# ==============================================================================
# R_EFF: effective turn radius (ft) of each turn TYPE, at the lane centre line.
# It is the average, over every turn of that type on this track, of
# (tightest centre-line radius + median centre-line radius) / 2 - i.e. the
# "typical" curve shape, not any one exact corner. Left/right 90 deg turns and
# left/right U-turns (~180 deg) all use the SAME turn equation (section 5) -
# only this radius and the turn's handedness differ between them.
R_EFF = {
    'left':    28.9,   # 90-degree left turn
    'right':   23.1,   # 90-degree right turn
    'U_left':  23.2,   # U-turn, turning left  (~180 degrees)
    'U_right': 17.1,   # U-turn, turning right (~180 degrees)
}

# LAP_LEN / LAP: the track's lap distance (ft, along the lane centre) and the
# ordered table of (start_distance_ft, section_type) used to look up which
# section the car is in from its odometry. Measured from the reference track
# drawing; rescale via section_at(s, lap_len=...) if your car's lap differs.
LAP_LEN = 1392.2
LAP = [
    (0.0,    'straight'), (63.5,   'left'),     (116.9,  'straight'),
    (187.3,  'left'),     (225.5,  'straight'), (250.5,  'left'),
    (301.3,  'right'),    (322.9,  'straight'), (411.6,  'U_right'),
    (482.8,  'U_left'),   (585.5,  'straight'), (737.5,  'U_left'),
    (894.1,  'right'),    (936.3,  'straight'), (1019.1, 'left'),
    (1058.4, 'U_right'),  (1193.2, 'straight'), (1280.5, 'U_left'),
    (1352.3, 'straight'),
]

# WallFix: what find() (section 10) hands back for a wall it located.
#   dist      - closest range to that wall (ft, or m)
#   bearing   - angle to that closest point (rad, car frame)
#   direction - the wall's own running angle relative to the car's heading
#               (0 = wall runs parallel to the car)
#   conf      - vote agreement behind this label, 0.5 (guessed) .. 1.0 (unanimous)
#   n         - how many hits made up this wall in the scan
WallFix = namedtuple('WallFix', 'dist bearing direction conf n')


# ==============================================================================
# 2. SECTION LOOKUP
# ==============================================================================
def section_at(s_ft, lap_len=LAP_LEN):
    """Which section of the lap the car is in, given distance travelled since
    the start line (mid bottom straight, heading right, lap runs CCW).
    If the car's own measured lap length differs from LAP_LEN, pass it as
    lap_len= and the table is rescaled proportionally."""
    s = (s_ft % lap_len) * LAP_LEN / lap_len
    return [sec for start, sec in LAP if start <= s][-1]


# ==============================================================================
# 3. GEOMETRY HELPERS  (shared by the straight rule, the turn rule, and voting)
# ==============================================================================
def _wrap(a):
    """Wrap an angle (rad) into (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _chord2(a, b):
    """Squared straight-line distance between two polar hits a=(r,theta), b=(r,theta)
    (law of cosines). Kept squared - and compared against squared thresholds
    everywhere below - purely so the hot paths never need a sqrt."""
    return a[0]*a[0] + b[0]*b[0] - 2*a[0]*b[0]*math.cos(b[1] - a[1])


def _chord(a, b):
    """Actual (non-squared) distance between two hits. Used only where a real
    ft/m value is needed, e.g. inside the wall-classification math itself."""
    return math.sqrt(max(0.0, _chord2(a, b)))


def _spread(n):
    """Yield indices 0..n-1 in coarse-to-fine order (middle, quarters, eighths,
    ...), lazily - each new index fills the current biggest gap. Used by the
    confidence-voting step (section 8) so a vote that stops early only pays
    for the samples it actually used, and so the FIRST few samples it does
    take are spread across the whole wall piece rather than clustered at one
    end (checking only the nearest points first was tried and is biased: it
    over-weights whichever points happened to get pulled closer by noise)."""
    seen, d = set(), 2
    while len(seen) < n:
        for i in (j * n // d for j in range(1, d, 2)):
            if i not in seen:
                seen.add(i)
                yield i
        d *= 2


class WallFinder:
    def __init__(self, units='ft', max_range=30, min_range=0.5, gap=2.5, min_pair=2.0,
                 step=3, agree=0.8, span=4.0, max_votes=64):
        """
        units      - 'ft' (default) or 'm'. All the *_range/gap/pair/span args
                     below are given in this unit and converted internally.
        max_range  - ignore hits farther than this (ft/m). Accuracy drops past
                     ~1.5 track-widths because hits start belonging to the
                     NEXT/PREVIOUS section under the CURRENT section's rule.
        min_range  - ignore hits closer than this (guards against the car's
                     own body / sensor noise floor).
        gap        - max distance (ft/m) between neighbouring hits for them to
                     count as the same continuous wall piece (section 7).
        min_pair   - when classifying hit i, its partner hit must be at least
                     this far away along the same piece (too-close pairs make
                     the classification math numerically unstable).
        step       - confidence-vote batch size: check `step` more point-pairs
                     at a time before re-checking whether `agree` has been
                     reached (section 8).
        agree      - fraction of vote agreement needed to call a wall piece
                     confirmed (section 8).
        span       - how far (ft/m) around the single closest hit to look when
                     estimating a wall's direction/angle in find() (section 10).
        max_votes  - hard cap on point-pairs sampled per wall piece; bounds the
                     worst-case time per scan (one real spike existed without
                     this: an undecided piece would sample EVERY point).
        """
        k = 0.3048 if units == 'm' else 1.0
        self.k, self.R = k, {s: r * k for s, r in R_EFF.items()}
        self.max_r, self.min_r = max_range * k, min_range * k
        self.gap2, self.pair2, self.span2 = (gap*k)**2, (min_pair*k)**2, (span*k)**2
        self.step, self.agree, self.max_votes = step, agree, max_votes

    # --------------------------------------------------------------------------
    # 4. RULE: STRAIGHT SECTIONS
    # --------------------------------------------------------------------------
    # On a straight, the two walls just run alongside the car. The only
    # question is which SIDE of the car a given wall line is on.
    def _straight_pair(self, a, b, L):
        """Two hits a=(r1,t1), b=(r2,t2), L = distance between them (already
        computed by the caller). Returns 1 (wall1/right) or 2 (wall2/left).

        rho = signed perpendicular distance from the car to the line through
        a and b (positive = line passes on the car's right), derived from the
        triangle area formula in polar form:
            rho = r1*r2*sin(t2-t1) / L
        This sign depends on which hit is "first", so it's re-oriented to
        always mean the same thing by checking whether the line points
        forward (b further along +x than a) or backward, and flipping if not.
        """
        r1, t1 = a
        r2, t2 = b
        rho = r1 * r2 * math.sin(t2 - t1) / L
        points_backward = r2 * math.cos(t2) < r1 * math.cos(t1)
        return 1 if (rho > 0) != points_backward else 2

    def _straight_point(self, a):
        """Lone hit on a straight (no partner): simply which side of the car
        it's on. sin(theta) < 0 means the hit is to the car's right = wall1."""
        r, t = a
        return 1 if math.sin(t) < 0 else 2

    # --------------------------------------------------------------------------
    # 5. RULE: TURNS  (90-degree left/right AND U-turns - same equation)
    # --------------------------------------------------------------------------
    # A 90-degree turn and a U-turn are geometrically the same shape (an arc
    # around some centre point) - a U-turn is just a turn with ~180 degrees of
    # sweep instead of ~90, and usually a tighter effective radius. So one pair
    # of functions handles 'left', 'right', 'U_left' AND 'U_right': only the
    # radius (R_EFF, looked up by section name) and the turn's handedness
    # (left vs right) change.
    #
    # The idea: work out where the TURN'S CENTRE is, relative to the car, then
    # check whether the wall LINE through the two hits passes closer to that
    # centre than the lane's own centre-line radius does. Closer = inner wall,
    # farther = outer wall. Which physical wall (wall1/wall2) that maps to
    # flips between left-handed and right-handed turns.
    def _turn_centre(self, section, o):
        """Where the turn's centre is, in the car's own polar frame, given the
        car's sideways offset `o` from the lane centre (+o = car is left of
        centre; from lane_offset(), section 9). Returns (distance, bearing,
        is_left_turn)."""
        is_left = section in ('left', 'U_left')
        # If the car sits `o` left of the lane centre, and the lane's centre
        # sweeps around a point R_EFF to the LEFT of centre (left turn) or
        # R_EFF to the RIGHT (right turn / U-right), then the turn centre is
        # this far to the car's left:
        lateral = (o + self.R[section]) if is_left else (o - self.R[section])
        distance, bearing = abs(lateral), math.copysign(math.pi / 2, lateral)
        return distance, bearing, is_left

    def _turn_side(self, is_inner, is_left_turn):
        """Map "is this the inner wall of the turn?" + "is the turn a left or
        right turn?" to the physical wall1/wall2 label.
          left turn:  inner wall = wall2 (left/inner loop), outer = wall1
          right turn: inner wall = wall1 (right/outer... - see note below),
                      outer = wall2
        (For a RIGHT turn the car curves toward wall1, so the wall it turns
        into - the inner one - is wall1 itself; the far wall is wall2. This is
        the mirror image of the left-turn case.)"""
        if is_left_turn:
            return 2 if is_inner else 1
        return 1 if is_inner else 2

    def _turn_pair(self, a, b, L, section, o):
        """Two hits on a turn. Computes the distance from the turn's centre to
        the LINE through a and b (again via the polar triangle-area formula,
        this time relative to the centre point rather than the car), and
        compares it to R_EFF (the lane's own centre-line radius, i.e. midway
        between the two walls) to decide inner vs outer."""
        r1, t1 = a
        r2, t2 = b
        c, tc, is_left = self._turn_centre(section, o)
        d = abs(r1*r2*math.sin(t2-t1) + r2*c*math.sin(tc-t2) + c*r1*math.sin(t1-tc)) / L
        return self._turn_side(d < self.R[section], is_left)

    def _turn_point(self, a, section, o):
        """Lone hit on a turn (no partner): straight-line (non-squared) polar
        distance from the turn's centre to the hit, compared to R_EFF."""
        r, t = a
        c, tc, is_left = self._turn_centre(section, o)
        d = math.sqrt(r*r + c*c - 2*r*c*math.cos(t - tc))
        return self._turn_side(d < self.R[section], is_left)

    # --------------------------------------------------------------------------
    # 6. RULE: LONE-HIT FALLBACK  (dispatch straight vs turn to sections 4/5)
    # --------------------------------------------------------------------------
    def label_pair(self, a, b, section, o=0.0):
        """Public entry point: classify the wall through two hits
        a=(r1,t1), b=(r2,t2) -> 1 (wall1) or 2 (wall2).
        o = the car's sideways offset from the lane centre, left-positive
            (see lane_offset(), section 9) - only used on turns."""
        L = _chord(a, b)
        if L < 1e-6:                                    # a and b are the same point
            return self.label_point(a, section, o)
        if section == 'straight':
            return self._straight_pair(a, b, L)
        return self._turn_pair(a, b, L, section, o)

    def label_point(self, a, section, o=0.0):
        """Public entry point: classify a single hit with no partner on its
        wall piece, using the same underlying rule as label_pair()."""
        if section == 'straight':
            return self._straight_point(a)
        return self._turn_point(a, section, o)

    # ==========================================================================
    # 7. SCAN -> WALL PIECES
    # ==========================================================================
    def lane_offset(self, scan, angles=None):
        """How far the lane centre is to the car's LEFT (ft/m, signed):
        (range at +90 deg  -  range at -90 deg) / 2.
        Needed by the turn rule (section 5) so it knows where "the lane
        centre" actually is relative to the car right now, not just in
        theory. `scan` must be sorted by angle (label_scan does this); the
        beam nearest +-90 deg is found by binary search, not a linear scan."""
        angles = angles or [h[1] for h in scan]

        def at(target_angle):
            i = bisect.bisect_left(angles, target_angle)
            j = min((i - 1, i % len(scan)), key=lambda j: abs(_wrap(angles[j] - target_angle)))
            return scan[j][0] if abs(_wrap(angles[j] - target_angle)) < math.radians(10) else None

        left, right = at(math.pi / 2), at(-math.pi / 2)
        return (left - right) / 2 if left is not None and right is not None else 0.0

    def split(self, scan):
        """Break a (angle-sorted) scan into continuous wall PIECES: runs of
        hits where each neighbour is closer than `gap`. This is the one pass
        that must touch every hit, so the squared-chord check is inlined
        rather than calling _chord2() (measurably faster on dense scans)."""
        g2, cos = self.gap2, math.cos
        pieces = [[scan[0]]]
        for a, b in zip(scan, scan[1:]):
            if a[0]*a[0] + b[0]*b[0] - 2*a[0]*b[0]*cos(b[1]-a[1]) < g2:
                pieces[-1].append(b)
            else:
                pieces.append([b])
        # a scan covers the full circle, so the last piece may wrap around
        # and rejoin the first one across the +-180 deg seam:
        if len(pieces) > 1 and _chord2(scan[-1], scan[0]) < g2:
            pieces[0] = pieces.pop() + pieces[0]
        return pieces

    def _partner(self, piece, i):
        """Find a hit in `piece` to pair with piece[i] for classification:
        the nearest one (by index) that's still at least `min_pair` away in
        actual distance. Searches outward from i in both directions and
        stops at the first qualifying hit (lazy - never builds a full list)."""
        for j in chain(range(i + 1, len(piece)), range(i - 1, -1, -1)):
            if _chord2(piece[i], piece[j]) >= self.pair2:
                return j
        return i + 1 if i + 1 < len(piece) else i - 1   # piece too short: best effort

    # ==========================================================================
    # 8. CONFIDENCE VOTING
    # ==========================================================================
    def label_piece(self, piece, section, o=0.0):
        """Decide ONE wall label for an entire wall piece (a list of hits from
        split(), section 7) by voting: repeatedly classify a (hit, partner)
        pair using label_pair(), sampling pairs coarse-to-fine across the
        whole piece (_spread(), section 3) rather than nearest-first.

        Stops early once `agree` fraction of the votes so far agree (checked
        every `step` samples) - this is the "confirm" step: cheap on an easy,
        unambiguous piece, and only spends more samples on an ambiguous one.
        Never samples more than `max_votes`, so one stubborn piece can't blow
        the time budget for the whole scan.

        Returns (wall, confidence): wall is 1 or 2 (majority); confidence is
        the winning fraction, 0.5 (a bare majority, or a single lone hit which
        always gets 0.5) up to 1.0 (every sample agreed)."""
        if len(piece) == 1:
            return self.label_point(piece[0], section, o), 0.5

        votes, used = [0, 0], 0
        for i in _spread(len(piece)):
            vote = self.label_pair(piece[i], piece[self._partner(piece, i)], section, o)
            votes[vote - 1] += 1
            used += 1
            confirmed = used % self.step == 0 and max(votes) >= self.agree * used
            if confirmed or used >= self.max_votes:
                break
        winner = 1 if votes[0] >= votes[1] else 2
        confidence = max(votes) / used
        return winner, confidence

    # ==========================================================================
    # 9. FULL-SCAN LABELLING
    # ==========================================================================
    def label_scan(self, scan, section):
        """The main entry point for a whole scan. Filters to [min_r, max_r],
        sorts by angle, works out the lane offset once (section 7), splits
        into wall pieces (section 7), and votes a label for each piece
        (section 8).

        Returns a list of (piece, wall, confidence) tuples - one per
        continuous wall piece seen in this scan. Feed this same list into
        find() (section 10) for BOTH walls and into your own mapping code,
        instead of re-labelling the scan multiple times."""
        lo, hi, pi = self.min_r, self.max_r, math.pi
        in_range = ((r, t if -pi <= t < pi else _wrap(t)) for r, t in scan if lo < r <= hi)
        scan = sorted(in_range, key=itemgetter(1))
        if not scan:
            return []
        offset = self.lane_offset(scan, [h[1] for h in scan])
        return [(piece, *self.label_piece(piece, section, offset)) for piece in self.split(scan)]

    # ==========================================================================
    # 10. WALL LOCATION
    # ==========================================================================
    def _side_hits(self, piece, k, step):
        """Hits in `piece` within `span` of piece[k], walking away from index
        k in direction `step` (+1 or -1). Keeps walking a bit past `span`
        (to 2x it) so a noisy point that flickers just outside `span` doesn't
        get dropped, without scanning the whole piece."""
        out, i, far2 = [], k + step, 4 * self.span2
        while 0 <= i < len(piece):
            d2 = _chord2(piece[i], piece[k])
            if d2 > far2:
                break
            if d2 <= self.span2:
                out.append(piece[i])
            i += step
        return out

    def find(self, scan, section, which=1, labels=None):
        """Roughly locate wall `which` (1 or 2) in this scan, or return None
        if it isn't seen. Pass labels=label_scan(scan, section) to reuse one
        labelling instead of re-labelling the scan for every call (do this
        when calling find() for both walls, or alongside your own mapping).

        Picks the wall piece closest to the car, then looks at a small window
        of hits (`span` wide) around its single closest point to estimate
        the wall's running direction. Returns a WallFix (section 1) or None.
        """
        labels = self.label_scan(scan, section) if labels is None else labels
        pieces = [(piece, conf) for piece, wall, conf in labels if wall == which]
        if not pieces:
            return None
        pieces = [pc for pc in pieces if len(pc[0]) > 1] or pieces   # prefer a real piece over a lone hit

        piece, confidence = min(pieces, key=lambda pc: min(pc[0])[0])   # nearest piece to the car
        ranges = [h[0] for h in piece]
        k = ranges.index(min(ranges))                                    # index of the single closest hit

        window = self._side_hits(piece, k, -1)[::-1] + [piece[k]] + self._side_hits(piece, k, 1)
        (r1, t1), (r2, t2) = window[0], window[-1]
        direction = math.atan2(r2*math.sin(t2) - r1*math.sin(t1), r2*math.cos(t2) - r1*math.cos(t1))
        if math.cos(direction) < 0:                       # keep the direction pointing "forward"
            direction = _wrap(direction + math.pi)
        if len(window) == 1:                               # no neighbours at all: assume square-on
            direction = _wrap(piece[k][1] - math.copysign(math.pi / 2, piece[k][1]))

        robust_dist = sorted(h[0] for h in window)[:3]      # median of the 3 closest = robust vs one bad hit
        dist = robust_dist[len(robust_dist) // 2]
        n_hits = sum(len(p) for p, _ in pieces)
        return WallFix(dist, piece[k][1], direction, confidence, n_hits)


# ==============================================================================
# 11. TRACKING OVER TIME
# ==============================================================================
class WallTracker:
    """Confirms one wall (wall1 or wall2) over SEVERAL scans, and smooths its
    position, instead of trusting a single scan's find() result outright.

    update() returns the tracked WallFix only once the wall has been seen
    consistently `need` times in a row (a miss, or the wall jumping more than
    `jump`, counts the streak back down by one instead of resetting it to
    zero outright - so one bad scan doesn't throw away a good lock). While
    locked on, the position is exponentially smoothed with weight `alpha`
    on each new observation."""

    def __init__(self, finder, which=1, need=3, min_conf=0.7, jump=3.0, alpha=0.5):
        self.f, self.which, self.need, self.min_conf, self.alpha = finder, which, need, min_conf, alpha
        self.jump = jump * finder.k     # convert to the finder's own units (ft or m)
        self.fix, self.count = None, 0

    def update(self, scan, section, labels=None):
        """Call once per scan. Returns the confirmed+smoothed WallFix, or None
        if the wall isn't confirmed yet. `labels` optionally reuses an
        already-computed label_scan() result (see find(), section 10)."""
        f = self.f.find(scan, section, self.which, labels)

        if f is None or f.conf < self.min_conf:
            self.count = max(self.count - 1, 0)                 # missed / unsure: streak decays
        elif self.fix is None or abs(f.dist - self.fix.dist) > self.jump:
            self.fix, self.count = f, 1                          # new lock, or the wall jumped: restart
        else:
            a, prev = self.alpha, self.fix                       # smooth toward the new observation
            self.fix = WallFix(
                prev.dist + a * (f.dist - prev.dist),
                _wrap(prev.bearing + a * _wrap(f.bearing - prev.bearing)),
                _wrap(prev.direction + a * _wrap(f.direction - prev.direction)),
                f.conf, f.n,
            )
            self.count = min(self.count + 1, 2 * self.need)

        return self.fix if self.count >= self.need else None


# ==============================================================================
# 12. SENSOR ADAPTER
# ==============================================================================
def from_laserscan(ranges, angle_min, angle_increment, mount_yaw=0.0):
    """Convert a ROS sensor_msgs/LaserScan's `ranges` array into this module's
    scan format: [(r, theta), ...]. `mount_yaw` (rad) corrects for the lidar
    being mounted rotated relative to the car's nose - it's added to every
    beam's angle. Angles are NOT wrapped here; label_scan() (section 9)
    wraps the handful that end up outside (-pi, pi] on its own."""
    a0 = angle_min + mount_yaw
    return [(r, a0 + i * angle_increment) for i, r in enumerate(ranges) if math.isfinite(r) and r > 0]


# ==============================================================================
# 13. SELF-TEST  (python3 wall_finder.py)
# ==============================================================================
if __name__ == '__main__':
    import time

    def corridor(n_beams):
        """A synthetic straight corridor: wall1 (right) 7 ft away, wall2 (left) 13 ft away."""
        angles = (-math.pi + 2 * math.pi * i / n_beams for i in range(n_beams))
        return [(abs((-7 if t < 0 else 13) / math.sin(t)), t) for t in angles if abs(math.sin(t)) > 0.05]

    wf = WallFinder()
    for wall in (1, 2):
        fix = wf.find(corridor(360), 'straight', wall)
        print(f'wall{wall}: {fix.dist:.1f} ft at bearing {math.degrees(fix.bearing):.0f} deg, '
              f'wall angle {round(math.degrees(fix.direction))} deg, conf {fix.conf:.2f}')

    for n_beams in (360, 1440):
        scan = corridor(n_beams)
        t0 = time.perf_counter()
        for _ in range(100):
            lab = wf.label_scan(scan, 'straight')
            wf.find(scan, 'straight', 1, lab)
            wf.find(scan, 'straight', 2, lab)
        ms_per_scan = 10 * (time.perf_counter() - t0)
        print(f'{n_beams} beams: {ms_per_scan:.2f} ms per scan (label + both walls) on this machine')
