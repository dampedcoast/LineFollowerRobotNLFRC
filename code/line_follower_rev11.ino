// ====== PD Line Follower — ESP32-S3 + H-bridge + 15ch analog IR ============
// corr = Kp*error + Kd*(error - lastError), signed drive, non-blocking pivot.
//
// ============================================================================
// [rev11] — DAY-2 ARENA DECODED (vector-PDF analysis) + ZONE-GAP CONCEPT
// ============================================================================
//   ARENA FACTS (measured programmatically from DAY_2_ARENA_V_2 PDF):
//     * The sheet holds TWO PRINTS OF THE SAME 3m x 3m LAYOUT, rotated 180
//       degrees to each other (raster match 99.85%). A 180-deg rotation
//       PRESERVES HANDEDNESS: driving START->END, left turns stay left
//       turns on either print -> ONE tune covers both. DIR_SWAP stays 0,
//       Y_SIDE / EDGE_SWAP are identical on both prints.
//     * Line width 17 mm, same as day 1. The START straight runs through
//       giant "NLFRC" letter glyphs that act as crossbars / short branches
//       (same family as the day-1 start bar + cathode bars -> XING /
//       edge-latch already cover them).
//     * TWO REAL INVERTED WHITE-ON-BLACK ZONES on the route (day 1: none):
//         Z1 "freeform": large black region (local thickness up to 350 mm)
//            with a WHITE curved Y-fork figure inside.
//         Z2 "grid": two 275x749 mm black slabs, a 17 mm white channel
//            between them, crossed by a 63 mm white band -> a white
//            hourglass + crossbar figure (forks INSIDE the zone).
//     * ONE LINE BREAK: an ~8-20 mm white stand-off where the black line
//       stops short of Z1's edge. In addition, EVERY zone exit dumps the
//       bot onto plain white until the black line resumes -> zone exits
//       BEHAVE LIKE LINE GAPS.
//     * 5 CHECKPOINT BARS (180x17 mm, red square alongside) cross the
//       line: full-bar blip for a few ms -> XING state rides through,
//       edge-latch GUARD 1 suppresses latching. No handling needed.
//     * Shared 250 mm FULL-BLACK END PAD between the two prints ->
//       FULL_BLACK_STOP confirmed correct (all-line -> probe -> FINISH,
//       and running off the pad during the probe finishes immediately
//       because plain white reads all-line in inverted polarity).
//
//   TUNE CHANGES vs rev10 defaults (all remain live web knobs):
//     * Y_MIN 2 -> 4  (MANDATORY): arms the GAP-FWD forward probe. With
//       Y_MIN=2 a plain 2ch line keeps biasHold latched, every loss is
//       marked "tip likely" and the bot REVERSES — at a zone exit that
//       reverses it back INTO the zone, and it hard-fails the Z1 line
//       break. Diode tips still latch Y-bias: tips widen to 4ch (day-1
//       calibration), exactly at the new threshold.
//     * XING_MIN 8 -> 10: kills the ring-trunk XING/INV? flicker (trunk
//       reads 7-8ch) that rev10 flagged, while zone entries, checkpoint
//       bars and the END pad light the full bar (15ch) with margin.
//     * INV_CONFIRM_MS 180 -> 280: inside Z2 the 63 mm white cross-band
//       dwells > 180 ms under ~0.35 m/s and would false-fire INV-OUT
//       mid-zone. 280 clears the band; the 17 mm checkpoint bars (few ms
//       dwell) and real zone faces are unaffected.
//     * INV_WHITE 1800: CALIBRATE ON THE MAT — park the bar inside a
//       zone, read raws off the web UI before the first run.
//     * EDGE_HOLD_MS stays 200: stub/comb features exist on this track
//       too; short expiry is still what prevents stale-latch hijacks.
//
//   NEW CONCEPT — ZONE-GAP WINDOW (GAP_EXPECT_MS, default 700):
//     * INV-OUT arms a timer. While it runs, a lost line goes STRAIGHT to
//       the forward probe, unconditionally — overriding the inEvent /
//       biasHold "tip likely" veto — because right after a zone exit the
//       line is EXPECTED to be missing (white stand-off before the black
//       line resumes). Without this, the event side-lock (kept alive
//       during losses by design) could reverse the bot back into the
//       zone it just left. Edge latches are cleared at INV-OUT too, so a
//       stale slab-edge latch can't hijack the exit.
//   Resolved from rev10's open questions: line gaps DO exist -> Y_MIN=4
//   locked in; XING_MIN raised as pre-planned; no mandatory right-only T
//   found on this layout -> EDGE_SWAP stays 0 unless the mat says otherwise.
//
// ============================================================================
// [rev10] — DAY-2 BUILD: field-validated tune locked in, nothing hardcoded
// ============================================================================
//   * All tuning defaults below are the DAY-1 VALIDATED set (ran the NLFRC
//     mat). The rev9 WARNING flags on Y_MIN / EDGE_TURN_MS / EDGE_CONFIRM_MS
//     / EDGE_HOLD_MS are downgraded to notes — track results beat theory.
//     Short EDGE_HOLD (200) is correct for this mat: the dead-end stub comb
//     arms edge latches constantly, and fast expiry stops a stale stub
//     latch from hijacking a later turn.
//   * PIVOT_TIMEOUT_MS added to the web UI (was the last non-tunable knob).
//   * ZERO track-specific routing: no TURN_SEQ, no slot tables, no lap
//     counting. Every routed decision is sensor-driven (freshest edge latch
//     wins); Y_SIDE is the only preference fallback — itself a live knob.
//
//   MAT COVERAGE (START -> END, rev11 updated for the day-2 layout):
//     start bar + "NLFRC" letter crossbars  -> XING blip / edge-latch turns
//     long straights                        -> PD + BOOST_XP in BOOST_BAND
//     freeform inverted zone Z1 (white Y)   -> all-line confirm -> INV-IN,
//                                              fork inside via edge/Y_SIDE
//     line break at Z1 edge                 -> ZONE-GAP / GAP-FWD probe
//     circled-triangle "hazard" symbols     -> Y-BIAS hug + edge-latch turn
//     checkpoint bars (x5)                  -> XING blip, no latch (guard)
//     grid inverted zone Z2 (hourglass)     -> INV-IN, ride the channel,
//                                              INV_CONFIRM_MS=280 rides the
//                                              63mm white cross-band
//     zone exits onto white                 -> INV-OUT + ZONE-GAP window
//     ring cluster ("grapes")               -> wide-cluster Y-BIAS edge hug,
//                                              XING_MIN=10 stops flicker
//     dead-end stub comb                    -> straight through (latch
//                                              expires, line never lost)
//     sawtooth zigzag                       -> pivots + ZZ mode (base drop)
//     square-wave staircase                 -> pivot / edge turn / ERR-HOLD
//     S-waves / serpentine coils / loops    -> PD; self-crossing = XING or
//                                              nearest-cluster continuity
//     END pad (250mm, shared)               -> all-line confirm -> probe ->
//                                              FULL_BLACK_STOP
//
//   DAY-2 LIVE-TUNE NOTES (web UI, no reflash):
//     * If XING/INV? still flickers in the rings on SLOW runs, raise
//       XING_MIN to 11-12 — entries/pad light all 15, huge margin.
//     * If the bot false-toggles INV-OUT inside Z2 (watch Polarity on the
//       UI), raise INV_CONFIRM_MS toward 350 — or keep speed up in-zone.
//     * If a zone exit still reverses, raise GAP_EXPECT_MS to 1000.
//     * Both prints are the same course: tune once, run either.
//
// ============================================================================
// [rev9] — HARDCODED TURNS REMOVED, ROUTING IS FULLY EDGE-DRIVEN
// ============================================================================
//   * TURN_SEQ (the 12-slot hardcoded direction table) is DELETED. Every
//     routed decision now comes from the EDGE-LATCH concept:
//       - fork target selection      -> freshest edge latch side
//       - sticky Y-BIAS edge choice  -> freshest edge latch side
//       - edge-turn direction        -> the latched side itself
//       - fallback spin direction    -> latched side held by the event
//     Only when NO latch exists does routing fall back to Y_SIDE.
//   * TURN_COOLDOWN_MS still exists but now only controls how long an
//     event keeps its chosen side locked (quiet time to expire inEvent).
//
// ============================================================================
// [rev8] ORIGINAL TUNE SCALE (error in CHANNEL units -7..+7) + WINNER
//   CONCEPTS: ERR-HOLD (keep steering with last correction on side loss)
//   and STRAIGHT-LINE BOOST (BOOST_XP when centered).
//
// ============================================================================
// [rev6] — EDGE-LATCH TURN: acute tips, diode, 90-deg corners by EDGE sensors
// ============================================================================
//     1. DETECT+SAVE: while tracking, if an OUTER EDGE sensor (ch 0..EDGE_ZONE-1
//        right side, ch 14..15-EDGE_ZONE left side) reads the line while the
//        MAIN line is still near the center of the bar, that side is LATCHED
//        (debounced EDGE_CONFIRM_MS, remembered EDGE_HOLD_MS). Guards: no
//        latch during XING (all-line) and none if no cluster is near center.
//     2. WAIT: nothing happens while the line is still under the bar; PD
//        keeps aligning with the track.
//     3. TRIGGER: the moment the bar LOSES the line with a fresh latch ->
//        EDGE-TURN starts instead of blind recovery.
//     4. TURN: the motor on the LATCHED side reverses, the other drives
//        forward -> in-place pivot toward the saved branch.
//     5. EXIT: keep rotating (EDGE_BLANK_MS blind at start) until a cluster
//        shows PAST CENTER on the OPPOSITE half (EDGE_EXIT_POS overshoot);
//        lastError is seeded so PD immediately corrects back and keeps
//        aligning with the track.
//     6. SAFETY: EDGE_TURN_MS timeout -> vertex-spin recovery.
//   DIR_SWAP mirrors ALL routed directions; EDGE_SWAP mirrors only the
//   edge-turn direction — both live-tunable.
//
//   LOST-LINE ROTATION DIRECTION (priority order):
//     0. ZONE-GAP window active -> no rotation, forward probe (rev11)
//     1. edge latch side (this concept)
//     2. event-held side (the latch that opened the event)
//     3. lastPos heuristic: spins toward the side the line was last seen
//
// ============================================================================
// CALIBRATION FINDINGS (serial tool, day-1 track — geometry family matches):
//   * Diode entry & acute angles present as ONE cluster that widens to a
//     MEASURED MAX WIDTH OF 4 channels.
//   * Circle/stop-sign trunk reads 7-8 channels wide.
//   * BROWNOUT_RST seen during logging -> power the logic rail from battery.
//
// TRACK-FAILURE FIXES (marked points):
//   [BOX] INV_PROBE_MS 400 -> 800; INV-IN locks the cluster NEAREST CENTER.
//   [SPIKE] LOST-BACK reverses until CH15 (sensor AT the pivot axis) is on
//     the line, so the fallback spin happens AT the vertex.
//   [XING] INV_CONFIRM_MS above crossing dwell time (perpendicular line).
//
// STICKY Y-BIAS: wide cluster (width>=Y_MIN for SIGN_CONFIRM_MS) latches
//   the chosen edge until width <= Y_EXIT_W for Y_EXIT_MS.
//
// SENSOR WIRING: index 0=RIGHTMOST, 14=LEFTMOST. error>0 = line LEFT.
// WEB TUNING: WiFi "LineFollower" (pass 12345678) -> http://192.168.4.1

#include <WiFi.h>
#include <WebServer.h>

#define IN1 6
#define IN2 7
#define IN3 4
#define IN4 5
#define ENA 9
#define ENB 8

#define S0  10
#define S1  11
#define S2  12
#define S3  13
#define OUT2 2

#define button 1

int THRESHOLD = 1800;
int NOISE     = 1000;

// ALL 15 bar sensors active — the outer channels ARE the edge-latch
// detectors. Never drop them on this track.
const int order[15] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
#define CENTER_CH 15
#define MAXC 4

// sticky-bias release
int Y_EXIT_W  = 3;    // day-1 validated (release width; see Y_MIN note)
int Y_EXIT_MS = 125;

// acute-tip fallback recovery geometry
int LOST_BACK_MS  = 900;
int SPIN_BLANK_MS = 225;

int raw[16];

// ====== TUNABLE PARAMS (rev11: day-2 arena-confirmed set) ===================
float Kp              = 40.0;
float Kd              = 165.0;
int   base            = 150;
int   BOOST_XP        = 30;    // straight-line speed boost (0 = off)
float BOOST_BAND      = 0.5;   // |error| <= this counts as centered (ch)
int   SPEED_LIMIT     = 255;
int   ERR_HOLD_MS     = 250;
float PIVOT_THRESHOLD = 5.0;   // channel units
int   PIVOT_SPEED     = 90;
int   MAX_REVERSE     = 255;
int   SPIN_SPEED      = 90;
int   MUX_SETTLE_US   = 60;

// zigzag
int           ZZ_BASE      = 110;
int           ZZ_TRIGGER   = 2;
unsigned long ZZ_WINDOW_MS = 150;
unsigned long ZZ_EXIT_MS   = 75;
int           ZZ_CENTER_MS = 75;

// inverted + intersections
int   INV_WHITE      = 1800;   // CALIBRATE ON THE MAT before run 1: park the
                               // bar inside an inverted zone, read raws.
int   XING_MIN       = 8;     // rev11: was 8. Ring trunk reads 7-8ch and
                               // flickered XING/INV? on slow runs; zone
                               // faces / checkpoint bars / END pad light
                               // the full 15ch. Raise to 11-12 live if the
                               // rings still flicker.
int   INV_CONFIRM_MS = 280;    // rev11: was 180. Z2's 63mm white cross-band
                               // dwells >180ms below ~0.35 m/s and would
                               // false-fire INV-OUT mid-zone. Checkpoint
                               // bars (17mm) are still far below this.
int   XING_SPEED     = 75;
float XING_GAIN      = 1.0;
int   INV_PROBE_MS   = 800;

// routing / clusters (channel units)
int   Y_SIDE          = 1;     // fallback ONLY when no edge latch exists.
                               // Same value works on BOTH prints (180-deg
                               // rotation preserves handedness).
int   Y_MIN           = 4;     // rev11: was 2 — MANDATORY for this arena.
                               // The track has a real line break at Z1 and
                               // every inverted-zone exit behaves like a
                               // gap; Y_MIN=2 kept biasHold latched on a
                               // plain 2ch line, flagged losses "tip
                               // likely" and skipped GAP-FWD (reversed —
                               // straight back into the zone). Diode tips
                               // widen to 4ch (day-1 calib) so tips still
                               // latch Y-bias at exactly this threshold.
float SIGN_GAIN       = 0.6;
int   SIGN_CONFIRM_MS = 50;
float SIGN_MIN_FRAC   = 0.12;
float JUMP_MAX        = 6.0;n

// ---- EDGE-LATCH TURN --------------------------------------------------------
int   EDGE_ZONE       = 2;
int   EDGE_CONFIRM_MS = 100;   // day-1 validated
int   EDGE_HOLD_MS    = 200;   // day-1 validated — short expiry is what
                               // keeps the stub comb from leaving stale
                               // latches. Day-2 layout has combs too: keep.
int   EDGE_TURN_SPEED = 80;
int   EDGE_TURN_MS    = 200;   // day-1 validated — acts as a directional
                               // kick; the vertex spin finishes the turn.
int   EDGE_BLANK_MS   = 60;
float EDGE_EXIT_POS   = 0.25;
int   EDGE_SWAP       = 0;     // 1 = mirror the edge-turn direction only.
                               // No mandatory right-only T found on the
                               // day-2 layout; leave 0 unless the mat says
                               // otherwise. Same value on both prints.
int   DIR_SWAP        = 0;     // 1 = mirror ALL routed directions. NOT
                               // needed between the two prints (rotation,
                               // not mirror). Keep 0.

// rev9: event side-lock quiet time (formerly the TURN_SEQ cooldown)
unsigned long TURN_COOLDOWN_MS = 125;

// lost-line search cycle
int   GAP_FWD_MS     = 250;
int   GAP_SPEED      = 80;
int   GAP_EXPECT_MS  = 800;    // rev11 ZONE-GAP: window armed at INV-OUT
                               // during which a lost line forces the
                               // forward probe (overrides inEvent/biasHold
                               // "tip likely"). Raise to 1000 live if a
                               // zone exit still reverses.
int   SPIN_360_MS    = 2500;
int   SEARCH_FWD_MS  = 300;

// kick-start
int   KICK_PWM       = 220;
int   KICK_MS        = 60;
int   KICK_MIN_CMD   = 60;

// finish behavior
bool  FULL_BLACK_STOP    = true;
int   FULL_BLACK_BACK_MS = 400;
// ============================================================================

unsigned long PIVOT_TIMEOUT_MS = 900;   // rev10: now web-tunable

// ===== runtime state =====
float lastError = 0;
float heldErr   = 0;      // winner error-hold memory (channel units)
float lastPos   = 7.0;
float trackPos  = 7.0;
bool  running   = false;
bool  centerHit = false;
bool  lastButton = HIGH;
unsigned long lastButtonMs = 0;
unsigned long lastCornerMs = 0;
int   cornerStreak = 0;
bool  zigzagMode   = false;
bool  inverted     = false;

int   lineCount = 0;
int   nSig      = 0;
float sigPos[MAXC];
int   sigLo[MAXC], sigHi[MAXC];

int   ctlMode = 0;                   // 0=PD/track 1=FORK 2=Y-BIAS
bool  pivotActive = false;
bool  pivotLeft   = false;
unsigned long pivotUntil = 0;
int   pivotFails  = 0;
unsigned long blackBackUntil = 0;
unsigned long invProbeUntil  = 0;

unsigned long wideSince      = 0;
unsigned long allLineSince   = 0;
unsigned long lostSince      = 0;
unsigned long spinStart      = 0;
unsigned long centerOnSince  = 0;

// ---- committed lost-recovery runtime ----
bool          recoverLock    = false;
unsigned long spinBlankUntil = 0;

// ---- rev11 ZONE-GAP runtime ----
unsigned long gapExpectUntil = 0;

// ---- sticky Y-BIAS runtime ----
bool          biasHold    = false;
unsigned long narrowSince = 0;

// ---- rev9: event runtime (side lock, no sequence) ----
bool          inEvent       = false;
int           activeSide    = 0;
unsigned long eventLastSeen = 0;

// ---- edge-latch runtime ----
unsigned long edgeLseen  = 0;
unsigned long edgeRseen  = 0;
unsigned long edgeLlatch = 0;
unsigned long edgeRlatch = 0;
bool          edgeTurnActive = false;
bool          edgeTurnLeft   = false;
unsigned long edgeTurnStart  = 0;
unsigned long edgeTurnBlank  = 0;

// ===== telemetry =====
char   bar[16];
float  t_pos = 7.0, t_err = 0, t_corr = 0;
int    t_L = 0, t_R = 0;
int    t_wmax = 0;
bool   t_online = true;
String t_state  = "idle";
unsigned long loopUs = 0, loopMaxUs = 0;

WebServer server(80);

// ================================================================
int readCh(int ch) {
  digitalWrite(S0,(ch>>0)&1); digitalWrite(S1,(ch>>1)&1);
  digitalWrite(S2,(ch>>2)&1); digitalWrite(S3,(ch>>3)&1);
  delayMicroseconds(MUX_SETTLE_US);
  return analogRead(OUT2);
}

void readSensors() {
  double wsum = 0, tot = 0;
  double cw[MAXC] = {}, ct[MAXC] = {};
  int    lo[MAXC], hi[MAXC];
  int    nClu = 0;
  lineCount = 0;
  int gapRun = 100;

  for (int i = 0; i < 15; i++) {
    int ch = order[i];
    raw[ch] = readCh(ch);

    bool   isLine;
    double w = 0;
    if (!inverted) {
      isLine = (raw[ch] > THRESHOLD);
      if (raw[ch] > NOISE) w = (double)(raw[ch] - NOISE);
    } else {
      isLine = (raw[ch] < INV_WHITE);
      if (raw[ch] < INV_WHITE) w = (double)(INV_WHITE - raw[ch]);
    }

    bar[i] = isLine ? 'B' : 'W';
    if (isLine) lineCount++;

    if (w > 0) {
      if (gapRun >= 2 && nClu < MAXC) { lo[nClu] = i; hi[nClu] = i; nClu++; }
      gapRun = 0;
      int c = nClu - 1;
      if (c >= 0) {
        hi[c] = i;
        double w2 = w * w;
        cw[c] += w2 * i;  ct[c] += w2;
        wsum  += w2 * i;  tot   += w2;
      }
    } else {
      gapRun++;
    }
  }
  bar[15] = '\0';

  nSig = 0;
  for (int c = 0; c < nClu; c++) {
    if (tot > 0 && ct[c] >= (double)SIGN_MIN_FRAC * tot) {
      sigPos[nSig] = (ct[c] > 0) ? (float)(cw[c] / ct[c]) : 7.0f;
      sigLo[nSig]  = lo[c];
      sigHi[nSig]  = hi[c];
      nSig++;
    }
  }

  t_wmax = 0;
  for (int c = 0; c < nSig; c++) {
    int w = sigHi[c] - sigLo[c] + 1;
    if (w > t_wmax) t_wmax = w;
  }

  raw[CENTER_CH] = readCh(CENTER_CH);
  centerHit = inverted ? (raw[CENTER_CH] < INV_WHITE)
                       : (raw[CENTER_CH] > THRESHOLD);
  if (centerHit) { if (centerOnSince == 0) centerOnSince = millis(); }
  else             centerOnSince = 0;

  t_online = (tot > 0);
  if (t_online) t_pos = (float)(wsum / tot);
}

// ================================================================
// mirror helper. 1=LEFT 2=RIGHT 0=nearest.
int applyDirSwap(int s) {
  if (!DIR_SWAP) return s;
  return (s == 1) ? 2 : (s == 2) ? 1 : 0;
}

// rev9: SENSOR-DRIVEN ROUTING — the freshest edge latch decides the side.
// No latch -> Y_SIDE fallback. (Replaces the hardcoded TURN_SEQ table.)
int liveSide() {
  unsigned long now = millis();
  bool L = (edgeLlatch != 0) && (now - edgeLlatch <= (unsigned long)EDGE_HOLD_MS);
  bool R = (edgeRlatch != 0) && (now - edgeRlatch <= (unsigned long)EDGE_HOLD_MS);
  if (L && R) return (edgeLlatch >= edgeRlatch) ? 1 : 2;  // newer latch wins
  if (L) return 1;
  if (R) return 2;
  return Y_SIDE;
}

int routeSide() {
  if (inEvent) return activeSide;          // side locked for this event
  return applyDirSwap(liveSide());
}

// rev9: event = side lock only. Opens on a fork/Y detection, snapshots the
// current edge-driven side, expires TURN_COOLDOWN_MS after last sighting.
void eventTick(int mode) {
  if (mode != 0) {
    if (!inEvent) {
      inEvent    = true;
      activeSide = applyDirSwap(liveSide());
    }
    eventLastSeen = millis();
  } else if (inEvent &&
             millis() - eventLastSeen > TURN_COOLDOWN_MS) {
    inEvent = false;
  }
}

// ================================================================
// EDGE SCAN — detect line on the OUTER edge channels while the MAIN line
// is still near center, and SAVE that state. bar[i]: i=0 RIGHTMOST,
// i=14 LEFTMOST. 'B' == on the line (works in the inverted zone too).
void edgeScan() {
  unsigned long now = millis();

  if (edgeLlatch && now - edgeLlatch > (unsigned long)EDGE_HOLD_MS) edgeLlatch = 0;
  if (edgeRlatch && now - edgeRlatch > (unsigned long)EDGE_HOLD_MS) edgeRlatch = 0;

  if (edgeTurnActive) return;

  // GUARD 1: crossing / pad / zone face lights everything -> not a branch
  if (lineCount >= XING_MIN) { edgeLseen = 0; edgeRseen = 0; return; }

  // GUARD 2: main line must still be near center
  bool mainCentered = false;
  for (int c = 0; c < nSig; c++)
    if (fabs(sigPos[c] - 7.0f) < 3.0f) { mainCentered = true; break; }
  if (!mainCentered) { edgeLseen = 0; edgeRseen = 0; return; }

  int zone = constrain(EDGE_ZONE, 1, 4);
  bool L = false, R = false;
  for (int k = 0; k < zone; k++) {
    if (bar[14 - k] == 'B') L = true;
    if (bar[k]      == 'B') R = true;
  }

  if (L) {
    if (edgeLseen == 0) edgeLseen = now;
    if (now - edgeLseen >= (unsigned long)EDGE_CONFIRM_MS) edgeLlatch = now;
  } else edgeLseen = 0;

  if (R) {
    if (edgeRseen == 0) edgeRseen = now;
    if (now - edgeRseen >= (unsigned long)EDGE_CONFIRM_MS) edgeRlatch = now;
  } else edgeRseen = 0;
}

void clearEdgeLatch() {
  edgeLseen = 0; edgeRseen = 0;
  edgeLlatch = 0; edgeRlatch = 0;
}

// ================================================================
int lastLdir = 1, lastRdir = 1;
unsigned long lKickUntil = 0, rKickUntil = 0;

void drive(int L, int R, bool noKick = false) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  unsigned long now = millis();

  int ldir = (L >= 0) ? 1 : -1;
  int rdir = (R >= 0) ? 1 : -1;
  if (!noKick) {
    if (abs(L) >= KICK_MIN_CMD && ldir != lastLdir) lKickUntil = now + KICK_MS;
    if (abs(R) >= KICK_MIN_CMD && rdir != lastRdir) rKickUntil = now + KICK_MS;
  } else { lKickUntil = 0; rKickUntil = 0; }
  if (L != 0) lastLdir = ldir;
  if (R != 0) lastRdir = rdir;

  int lp = abs(L), rp = abs(R);
  if (!noKick) {
    if (now < lKickUntil && lp >= KICK_MIN_CMD && lp < KICK_PWM) lp = KICK_PWM;
    if (now < rKickUntil && rp >= KICK_MIN_CMD && rp < KICK_PWM) rp = KICK_PWM;
  }

  if (L >= 0) { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  }
  else        { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); }
  if (R >= 0) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  }
  else        { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }
  ledcWrite(ENA, lp);
  ledcWrite(ENB, rp);
  t_L = L; t_R = R;
}

void stopMotors() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  ledcWrite(ENA,0); ledcWrite(ENB,0);
  t_L = 0; t_R = 0;
}

void resetState() {
  lastError = 0;  heldErr = 0;
  trackPos  = 7.0;  lastPos = 7.0;
  zigzagMode = false;  cornerStreak = 0;
  inverted   = false;  ctlMode = 0;
  pivotActive = false; pivotFails = 0;
  blackBackUntil = 0;  invProbeUntil = 0;
  wideSince = 0;       allLineSince = 0;
  lostSince = 0;       spinStart = 0;
  centerOnSince = 0;
  recoverLock = false; spinBlankUntil = 0;
  gapExpectUntil = 0;
  biasHold = false;    narrowSince = 0;
  inEvent = false; activeSide = 0; eventLastSeen = 0;
  clearEdgeLatch();
  edgeTurnActive = false; edgeTurnStart = 0; edgeTurnBlank = 0;
}

void finishReached() {
  inverted      = false;
  invProbeUntil = 0;
  allLineSince  = 0;
  if (FULL_BLACK_STOP) {
    stopMotors();
    running = false;
    t_state = "FINISH";
  } else {
    blackBackUntil = millis() + FULL_BLACK_BACK_MS;
    t_state = "BLACK-BACK";
    drive(-GAP_SPEED, -GAP_SPEED, true);
  }
  t_err = 0; t_corr = 0;
}

// ================================================================
void control() {

  // edge concept step 1 runs every cycle — detect & save.
  edgeScan();

  // ---- 0. BLACK-BACK in progress ------------------------------------------
  if (blackBackUntil != 0) {
    if (millis() < blackBackUntil) {
      t_state = "BLACK-BACK";
      drive(-GAP_SPEED, -GAP_SPEED, true);
      t_err = 0; t_corr = 0;
      return;
    }
    blackBackUntil = 0;
    lostSince   = millis() - (unsigned long)(GAP_FWD_MS + LOST_BACK_MS) - 1;
    spinStart   = 0;
    recoverLock = true;
  }

  // ---- 1. INVERTED-ZONE PROBE ----------------------------------------------
  if (invProbeUntil != 0) {
    if (t_online && lineCount < XING_MIN) {
      invProbeUntil = 0;
      // BOX FIX: lock the cluster NEAREST CENTER.
      int b = 0;
      for (int c = 1; c < nSig; c++)
        if (fabs(sigPos[c] - 7.0f) < fabs(sigPos[b] - 7.0f)) b = c;
      trackPos = sigPos[b];
      lastPos  = trackPos;
      lastError = 0;
      heldErr   = 0;                 // rev11: don't carry pre-zone steering
      t_state = "INV-IN";
      // fall through to normal control this cycle
    }
    else if (t_online && lineCount >= XING_MIN) {
      finishReached();
      return;
    }
    else if (millis() < invProbeUntil) {
      t_state = "INV?";
      drive(XING_SPEED, XING_SPEED);
      t_err = 0; t_corr = 0;
      return;
    }
    else {
      finishReached();
      return;
    }
  }

  // ---- 2. PIVOT in progress -------------------------------------------------
  if (pivotActive) {
    bool nearCenter = false;
    float centerPos = 7.0f;
    for (int c = 0; c < nSig; c++)
      if (fabs(sigPos[c] - 7.0f) < 2.0f) { nearCenter = true; centerPos = sigPos[c]; }

    if (t_online && nearCenter) {
      pivotActive = false;
      pivotFails  = 0;
      trackPos = centerPos;  lastPos = trackPos;
      lastError = 0;  ctlMode = 0;
    } else if (millis() >= pivotUntil) {
      pivotActive = false;
      pivotFails++;
      lastError = 0;  ctlMode = 0;
      if (pivotFails >= 2) {
        pivotFails = 0;
        lostSince   = millis() - (unsigned long)(GAP_FWD_MS + LOST_BACK_MS) - 1;
        spinStart   = 0;
        recoverLock = true;
      }
    } else {
      t_state = zigzagMode ? (pivotLeft ? "ZZ_L"     : "ZZ_R")
                           : (pivotLeft ? "CORNER_L" : "CORNER_R");
      if (pivotLeft) drive(-PIVOT_SPEED,  PIVOT_SPEED, true);
      else           drive( PIVOT_SPEED, -PIVOT_SPEED, true);
      t_err = t_pos - 7.0f; t_corr = 0;
      return;
    }
  }

  // ---- 2.5 EDGE-TURN in progress ----------------------------------------------
  if (edgeTurnActive) {
    unsigned long ph = millis() - edgeTurnStart;
    bool blanked = (millis() < edgeTurnBlank);

    bool  exitNow = false;
    float exPos   = 7.0f;
    if (!blanked) {
      for (int c = 0; c < nSig; c++) {
        float p = sigPos[c];
        if (edgeTurnLeft  && p <= 7.0f - EDGE_EXIT_POS) { exitNow = true; exPos = p; break; }
        if (!edgeTurnLeft && p >= 7.0f + EDGE_EXIT_POS) { exitNow = true; exPos = p; break; }
      }
    }

    if (exitNow) {
      // hand back to PD with the overshoot pre-loaded so it immediately
      // corrects back the other way and keeps aligning with the track.
      edgeTurnActive = false;
      clearEdgeLatch();
      trackPos = exPos;  lastPos = exPos;
      lastError = exPos - 7.0f;
      heldErr   = lastError;
      ctlMode = 0;
      lostSince = 0; spinStart = 0; recoverLock = false;
      if (inEvent) eventLastSeen = millis();
      // fall through to normal control this same cycle
    } else if (ph > (unsigned long)EDGE_TURN_MS) {
      edgeTurnActive = false;
      clearEdgeLatch();
      lostSince   = millis() - (unsigned long)(GAP_FWD_MS + LOST_BACK_MS) - 1;
      spinStart   = 0;
      recoverLock = true;
    } else {
      t_state = edgeTurnLeft ? "EDGE_L" : "EDGE_R";
      // "reverse the other motor": left turn = LEFT motor reversed.
      if (edgeTurnLeft) drive(-EDGE_TURN_SPEED,  EDGE_TURN_SPEED, true);
      else              drive( EDGE_TURN_SPEED, -EDGE_TURN_SPEED, true);
      if (inEvent) eventLastSeen = millis();
      t_err = 0; t_corr = 0;
      return;
    }
  }

  // ---- 3. ALL-LINE: crossing / zone boundary / pad entry ---------------------
  if (t_online && lineCount >= XING_MIN) {
    if (allLineSince == 0) allLineSince = millis();

    if (millis() - allLineSince > (unsigned long)INV_CONFIRM_MS) {
      allLineSince = 0;
      if (!inverted) {
        inverted      = true;
        invProbeUntil = millis() + INV_PROBE_MS;
        t_state = "INV?";
        drive(XING_SPEED, XING_SPEED);
      } else {
        // INV-OUT: zone exit onto white background. The black line resumes
        // after a stand-off — arm the ZONE-GAP window so the coming loss
        // goes to the forward probe, never into reverse. Stale slab-edge
        // latches must not hijack the exit either.
        inverted = false;
        trackPos = 7.0f;
        lastPos  = 7.0f;               // rev11: guarantee wasCentered
        heldErr  = 0;                  // rev11: no stale ERR-HOLD steering
        gapExpectUntil = millis() + (unsigned long)GAP_EXPECT_MS;
        clearEdgeLatch();
        t_state  = "INV-OUT";
        drive(XING_SPEED, XING_SPEED);
      }
      lastError = 0;
      t_err = 0; t_corr = 0;
      return;
    }

    t_state = "XING";
    float e = t_pos - 7.0f;
    int   c = (int)(XING_GAIN * Kp * e);
    drive(XING_SPEED - c, XING_SPEED + c);
    lastError = 0;
    lastPos   = t_pos;
    t_err = e; t_corr = c;
    return;
  }
  allLineSince = 0;

  // ---- 3.5 EDGE-TURN TRIGGER ----------------------------------------------------
  if (!t_online && !recoverLock && !edgeTurnActive &&
      millis() >= gapExpectUntil) {          // rev11: no edge turns inside
                                             // the ZONE-GAP window — the
                                             // loss is an EXPECTED gap.
    unsigned long now = millis();
    bool haveL = (edgeLlatch != 0) && (now - edgeLlatch <= (unsigned long)EDGE_HOLD_MS);
    bool haveR = (edgeRlatch != 0) && (now - edgeRlatch <= (unsigned long)EDGE_HOLD_MS);

    if (haveL || haveR) {
      bool left;
      if (haveL && haveR) left = (edgeLlatch >= edgeRlatch);  // newer wins
      else                left = haveL;
      if (EDGE_SWAP) left = !left;

      edgeTurnActive = true;
      edgeTurnLeft   = left;
      edgeTurnStart  = now;
      edgeTurnBlank  = now + (unsigned long)EDGE_BLANK_MS;
      lostSince = 0; spinStart = 0;
      t_state = left ? "EDGE_L" : "EDGE_R";
      if (left) drive(-EDGE_TURN_SPEED,  EDGE_TURN_SPEED, true);
      else      drive( EDGE_TURN_SPEED, -EDGE_TURN_SPEED, true);
      if (inEvent) eventLastSeen = millis();
      t_err = 0; t_corr = 0;
      return;
    }
  }

  // ---- 4. LINE LOST (no edge latch) -------------------------------------------
  bool  nearC   = false;
  float nearPos = 7.0f;
  for (int c = 0; c < nSig; c++)
    if (fabs(sigPos[c] - 7.0f) < 2.0f) { nearC = true; nearPos = sigPos[c]; }

  if (!t_online || recoverLock) {
    if (lostSince == 0) { lostSince = millis(); spinStart = 0; }
    if (inEvent) eventLastSeen = millis();
    unsigned long el = millis() - lostSince;
    bool wasCentered = fabs(lastPos - 7.0f) < 2.5f;
    bool tipLikely   = inEvent || biasHold;

    // rev11 ZONE-GAP: right after INV-OUT the line is EXPECTED to be
    // missing (zone-exit stand-off / the Z1 line break). Force the
    // forward probe unconditionally — the event side-lock stays alive
    // during losses by design and must not drag us back into the zone.
    if (!recoverLock && millis() < gapExpectUntil) {
      t_state = "ZONE-GAP";
      drive(GAP_SPEED, GAP_SPEED);
      t_err = 0; t_corr = 0;
      return;
    }

    // A0) WINNER ERROR-HOLD: line slid off the SIDE -> keep steering with
    //     the LAST correction before committing to recovery.
    if (!recoverLock && !wasCentered && !tipLikely &&
        el < (unsigned long)ERR_HOLD_MS) {
      t_state = "ERR-HOLD";
      float c = Kp * heldErr;
      drive(constrain((int)(base - c), -SPEED_LIMIT, SPEED_LIMIT),
            constrain((int)(base + c), -SPEED_LIMIT, SPEED_LIMIT), true);
      t_err = heldErr; t_corr = c;
      return;
    }

    // A) forward probe: plain centered line gap
    if (!recoverLock && wasCentered && !tipLikely &&
        el < (unsigned long)GAP_FWD_MS) {
      t_state = "GAP-FWD";
      drive(GAP_SPEED, GAP_SPEED);
      t_err = 0; t_corr = 0;
      return;
    }

    // B) COMMIT and reverse until CH15 (pivot axis) is back on the line
    if ((wasCentered || tipLikely) &&
        el < (unsigned long)(GAP_FWD_MS + LOST_BACK_MS)) {
      recoverLock = true;
      if (!centerHit) {
        t_state = "LOST-BACK";
        drive(-GAP_SPEED, -GAP_SPEED, true);
        t_err = 0; t_corr = 0;
        return;
      }
      lostSince = millis() - (unsigned long)(GAP_FWD_MS + LOST_BACK_MS) - 1;
    }

    // C) spin at the vertex.
    recoverLock = true;
    if (spinStart == 0) {
      spinStart      = millis();
      spinBlankUntil = spinStart + (unsigned long)SPIN_BLANK_MS;
    }
    unsigned long ph = millis() - spinStart;
    int  rs       = inEvent ? activeSide : 0;
    bool spinLeft = (rs == 1) ? true : (rs == 2) ? false : (lastPos > 7.0f);

    bool acquired = (millis() >= spinBlankUntil) && nearC;
    if (acquired) {
      recoverLock = false;
      lostSince = 0; spinStart = 0;
      trackPos = nearPos;  lastPos = nearPos;
      lastError = nearPos - 7.0f;      // seed D-term: no derivative kick
      heldErr   = lastError;
      // fall through to normal target selection this same cycle
    } else if (ph < (unsigned long)SPIN_360_MS) {
      t_state = "LOST-SPIN";
      if (spinLeft) drive(-SPIN_SPEED,  SPIN_SPEED, true);
      else          drive( SPIN_SPEED, -SPIN_SPEED, true);
      lastError = spinLeft ? 7.0f : -7.0f;
      t_err = lastError; t_corr = 0;
      return;
    } else if (ph < (unsigned long)(SPIN_360_MS + SEARCH_FWD_MS)) {
      t_state = "LOST-FWD";
      drive(GAP_SPEED, GAP_SPEED);
      lastError = spinLeft ? 7.0f : -7.0f;
      t_err = lastError; t_corr = 0;
      return;
    } else {
      spinStart      = millis();
      spinBlankUntil = spinStart + (unsigned long)SPIN_BLANK_MS;
      t_state = "LOST-SPIN";
      if (spinLeft) drive(-SPIN_SPEED,  SPIN_SPEED, true);
      else          drive( SPIN_SPEED, -SPIN_SPEED, true);
      t_err = 0; t_corr = 0;
      return;
    }
  }
  lostSince = 0; spinStart = 0;

  // ---- 5. TARGET SELECTION with POSITION CONTINUITY + EDGE ROUTING -----------
  int   mode = 0;
  float pos;
  int   side = routeSide();   // rev9: edge-latch driven, Y_SIDE fallback

  if (nSig <= 1) {
    pos = (nSig == 1) ? sigPos[0] : trackPos;
  } else {
    int cand[MAXC], nCand = 0;
    for (int c = 0; c < nSig; c++)
      if (fabs(sigPos[c] - trackPos) <= JUMP_MAX) cand[nCand++] = c;

    if (nCand == 1) {
      pos = sigPos[cand[0]];
    } else if (nCand >= 2) {
      mode = 1;                              // genuine fork
      if      (side == 1) pos = sigPos[cand[nCand - 1]];  // leftmost cand
      else if (side == 2) pos = sigPos[cand[0]];          // rightmost cand
      else {
        int best = cand[0];
        for (int k = 1; k < nCand; k++)
          if (fabs(sigPos[cand[k]] - trackPos) < fabs(sigPos[best] - trackPos))
            best = cand[k];
        pos = sigPos[best];
      }
    } else {
      int best = 0;
      for (int c = 1; c < nSig; c++)
        if (fabs(sigPos[c] - trackPos) < fabs(sigPos[best] - trackPos))
          best = c;
      pos = sigPos[best];
    }
  }

  // ---- STICKY Y-BIAS: wide single cluster -> commit to chosen edge ----------
  int width1 = (nSig == 1) ? (sigHi[0] - sigLo[0] + 1) : 0;

  if (side != 0 && nSig == 1) {
    if (!biasHold) {
      if (width1 >= Y_MIN) {
        if (wideSince == 0) wideSince = millis();
        if (millis() - wideSince >= (unsigned long)SIGN_CONFIRM_MS)
          biasHold = true;
      } else wideSince = 0;
    } else {
      if (width1 <= Y_EXIT_W) {
        if (narrowSince == 0) narrowSince = millis();
        if (millis() - narrowSince >= (unsigned long)Y_EXIT_MS) {
          biasHold = false; narrowSince = 0; wideSince = 0;
        }
      } else narrowSince = 0;
    }
    if (biasHold && mode == 0) {
      mode = 2;
      pos  = (side == 1) ? (sigHi[0] - 0.5f) : (sigLo[0] + 0.5f);
    }
  } else {
    if (nSig != 1) { biasHold = false; narrowSince = 0; }
    wideSince = 0;
  }

  eventTick(mode);

  trackPos = pos;
  lastPos  = pos;

  float error = pos - 7.0f;
  if (mode != ctlMode) lastError = error;
  ctlMode = mode;

  // ---- 6. PIVOT ENTRY & SNAP-TURN DETECT ---------------------------------------
  bool seeingBothLines = ((nSig >= 2) || (nSig == 1 && (sigHi[0] - sigLo[0] >= 7)));
  bool snapTurnTrigger = seeingBothLines && centerHit;

  if (fabs(error) >= PIVOT_THRESHOLD && !snapTurnTrigger) {
    pivotActive = true;
    pivotLeft   = (error > 0);
    pivotUntil  = millis() + PIVOT_TIMEOUT_MS;

    if (millis() - lastCornerMs < ZZ_WINDOW_MS) cornerStreak++;
    else                                        cornerStreak = 1;
    lastCornerMs = millis();
    if (cornerStreak >= ZZ_TRIGGER) zigzagMode = true;

    t_state = zigzagMode ? (pivotLeft ? "ZZ_L"     : "ZZ_R")
                         : (pivotLeft ? "CORNER_L" : "CORNER_R");
    if (pivotLeft) drive(-PIVOT_SPEED,  PIVOT_SPEED, true);
    else           drive( PIVOT_SPEED, -PIVOT_SPEED, true);
    lastError = 0;
    t_err = error; t_corr = 0;
    return;
  }

  // ---- 7. ZIGZAG EXIT ------------------------------------------------------------
  if (zigzagMode) {
    bool timerExit  = (millis() - lastCornerMs > ZZ_EXIT_MS);
    bool centerExit = (centerOnSince != 0 &&
                       millis() - centerOnSince > (unsigned long)ZZ_CENTER_MS &&
                       fabs(error) < 1.5f);
    if (timerExit || centerExit) { zigzagMode = false; cornerStreak = 0; }
  }

  // ---- 8. PD on the selected line --------------------------------------------------
  int eff_base = zigzagMode ? ZZ_BASE : base;

  // WINNER STRAIGHT-LINE BOOST: slow while correcting, fast when centered.
  if (mode == 0 && !zigzagMode && fabs(error) <= BOOST_BAND)
    eff_base += BOOST_XP;

  float active_Kp = Kp;
  float active_Kd = Kd;
  float gain;

  if (snapTurnTrigger) {
    active_Kp = Kp * 2.0;
    active_Kd = Kd * 1.5;
    gain      = 1.0f;
    t_state = "SNAP-TURN";
  } else {
    gain = (mode == 0) ? 1.0f : SIGN_GAIN;
    t_state = (mode == 1) ? "FORK"
            : (mode == 2) ? "Y-BIAS"
            : (zigzagMode ? "PD-ZZ" : "PD");
  }

  float corr = gain * (active_Kp * error + active_Kd * (error - lastError));
  lastError  = error;
  heldErr    = error;                 // winner error-hold memory

  int revLim = (base > 0) ? (int)((long)MAX_REVERSE * eff_base / base)
                          : MAX_REVERSE;
  if (mode != 0 && !snapTurnTrigger && revLim > 0) revLim = 0;
  if (revLim > SPEED_LIMIT) revLim = SPEED_LIMIT;

  drive(constrain((int)(eff_base - corr), -revLim, SPEED_LIMIT),
        constrain((int)(eff_base + corr), -revLim, SPEED_LIMIT));
  t_err = error; t_corr = corr;
}

// ================================================================
//  WEB PAGE
// ================================================================
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>PD Tuner</title><style>
body{font-family:system-ui;background:#0f1115;color:#eee;margin:0;padding:16px}
.card{background:#1a1d24;border-radius:12px;padding:12px 14px;margin-bottom:10px}
.prow{display:flex;justify-content:space-between;align-items:center;margin:6px 0}
.prow span{font-size:15px}
input[type=number]{width:110px;padding:8px;font-size:17px;text-align:right;
 background:#0c0f15;color:#4ea1ff;border:1px solid #2a3140;border-radius:8px;font-weight:700}
.bar{font-family:monospace;font-size:20px;letter-spacing:3px;text-align:center;margin:8px 0}
.B{color:#ff5555}.W{color:#444}
button{width:100%;padding:14px;font-size:18px;border:0;border-radius:10px;color:#fff;font-weight:700}
.go{background:#1f9d55}.stop{background:#d64545}
.stat{display:flex;justify-content:space-between;font-size:13px;margin:2px 0}
.val{font-weight:700;color:#4ea1ff}
.off{color:#ffb020}.hit{color:#1f9d55;font-weight:700}
.inv{color:#c678dd;font-weight:700}
.fin{color:#f0c040;font-weight:700}
</style></head><body>
<h2>PD Line Tuner</h2>
<div class='card'><button id='b' class='go' onclick='tg()'>START</button></div>
<div class='card'>
<div class='bar' id='bar'>...............</div>
<div class='stat'><span>State</span><span class='val' id='st'>-</span></div>
<div class='stat'><span>Status</span><span class='val' id='on'>-</span></div>
<div class='stat'><span>Polarity</span><span id='pol'>-</span></div>
<div class='stat'><span>Line count / clusters</span><span class='val' id='lc'>-</span></div>
<div class='stat'><span>Max cluster width</span><span class='val' id='w'>-</span></div>
<div class='stat'><span>Edge latch L/R</span><span class='val' id='el'>-</span></div>
<div class='stat'><span>Route side / event</span><span class='val' id='rt'>-</span></div>
<div class='stat'><span>Track pos</span><span class='val' id='tp'>-</span></div>
<div class='stat'><span>Center CH15</span><span id='ct'>-</span></div>
<div class='stat'><span>Error / Corr</span><span class='val' id='e'>-</span></div>
<div class='stat'><span>L / R PWM</span><span class='val' id='m'>-</span></div>
<div class='stat'><span>Loop us (max)</span><span class='val' id='lp'>-</span></div>
</div>
<div id='params'></div>
<script>
const P=[
 ['Kp',0.5],['Kd',5],['base',5],
 ['BOOST_XP',5],['BOOST_BAND',0.25],['SPEED_LIMIT',5],['ERR_HOLD_MS',25],
 ['THRESHOLD',50],['NOISE',50],
 ['PIVOT_THRESHOLD',0.5],['PIVOT_SPEED',5],['PIVOT_TIMEOUT_MS',50],
 ['ZZ_BASE',5],['ZZ_TRIGGER',1],['ZZ_WINDOW_MS',25],
 ['ZZ_EXIT_MS',25],['ZZ_CENTER_MS',25],
 ['INV_WHITE',10],['XING_MIN',1],['INV_CONFIRM_MS',10],
 ['XING_SPEED',5],['XING_GAIN',0.05],['INV_PROBE_MS',25],
 ['Y_SIDE',1],['Y_MIN',1],['SIGN_GAIN',0.05],
 ['SIGN_CONFIRM_MS',10],['SIGN_MIN_FRAC',0.02],['JUMP_MAX',0.5],
 ['Y_EXIT_W',1],['Y_EXIT_MS',10],
 ['EDGE_ZONE',1],['EDGE_CONFIRM_MS',10],['EDGE_HOLD_MS',50],
 ['EDGE_TURN_SPEED',5],['EDGE_TURN_MS',50],['EDGE_BLANK_MS',10],
 ['EDGE_EXIT_POS',0.25],['EDGE_SWAP',1],['DIR_SWAP',1],
 ['TURN_COOLDOWN_MS',25],
 ['GAP_FWD_MS',25],['GAP_SPEED',5],['GAP_EXPECT_MS',50],
 ['LOST_BACK_MS',50],['SPIN_BLANK_MS',25],
 ['SPIN_360_MS',50],['SEARCH_FWD_MS',25],
 ['KICK_PWM',5],['KICK_MS',10],['KICK_MIN_CMD',5],
 ['FULL_BLACK_STOP',1],['FULL_BLACK_BACK_MS',25],
 ['MAX_REVERSE',5],['SPIN_SPEED',5],['MUX_SETTLE_US',5]
];
P.forEach(p=>{
 const[n,st]=p;const d=document.createElement('div');
 d.className='card';
 d.innerHTML=`<div class='prow'><span>${n}</span>
 <input type='number' step='${st}' id='s_${n}'
 onchange="fetch('/set?p=${n}&v='+this.value)"></div>`;
 document.getElementById('params').appendChild(d);
});
const g=id=>document.getElementById(id);
function tg(){fetch('/toggle').then(r=>r.text()).then(t=>sb(t==='1'));}
function sb(r){const b=g('b');b.innerText=r?'STOP':'START';b.className=r?'stop':'go';}
function poll(){
 fetch('/data').then(r=>r.json()).then(j=>{
  for(const k in j.params){
   const s=g('s_'+k);
   if(s&&document.activeElement!==s)s.value=j.params[k];
  }
  let s='';
  for(const c of j.bar)s+=`<span class='${c}'>${c}</span>`;
  g('bar').innerHTML=s;
  g('st').innerText=j.state;
  g('tp').innerText=j.tp;
  g('el').innerText=(j.eL?'L':'-')+' / '+(j.eR?'R':'-');
  g('rt').innerText=(j.side==1?'LEFT':j.side==2?'RIGHT':'nearest')+(j.inev?' (event)':'');
  g('e').innerText=j.err+' / '+j.corr;
  g('m').innerText=j.L+' / '+j.R;
  g('lp').innerText=j.loop+' ('+j.loopmax+')';
  g('lc').innerText=j.lc+' / '+j.nc;
  g('w').innerText=j.w;
  const pol=g('pol');
  if(j.inv){pol.innerText='INVERTED';pol.className='inv';}
  else{pol.innerText='normal';pol.className='';}
  const o=g('on');
  if(j.state==='FINISH'){o.innerText='FINISHED';o.className='fin';}
  else if(j.online){o.innerText='ON LINE';o.className='val';}
  else{o.innerText='LOST';o.className='val off';}
  const ct=g('ct');
  if(j.center){ct.innerText='ON LINE';ct.className='hit';}
  else{ct.innerText='off';ct.className='';}
  sb(j.run);
 }).catch(()=>{});
}
poll();setInterval(poll,300);
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200,"text/html",PAGE); }

void handleSet() {
  String p = server.arg("p");
  float  v = server.arg("v").toFloat();
  if      (p=="Kp")               Kp=v;
  else if (p=="Kd")               Kd=v;
  else if (p=="base")             base=(int)v;
  else if (p=="BOOST_XP")         BOOST_XP=(int)v;
  else if (p=="BOOST_BAND")       BOOST_BAND=v;
  else if (p=="SPEED_LIMIT")      SPEED_LIMIT=constrain((int)v,0,255);
  else if (p=="ERR_HOLD_MS")      ERR_HOLD_MS=(int)v;
  else if (p=="THRESHOLD")        THRESHOLD=(int)v;
  else if (p=="NOISE")            NOISE=(int)v;
  else if (p=="PIVOT_THRESHOLD")  PIVOT_THRESHOLD=v;
  else if (p=="PIVOT_SPEED")      PIVOT_SPEED=(int)v;
  else if (p=="PIVOT_TIMEOUT_MS") PIVOT_TIMEOUT_MS=(unsigned long)v;
  else if (p=="ZZ_BASE")          ZZ_BASE=(int)v;
  else if (p=="ZZ_TRIGGER")       ZZ_TRIGGER=(int)v;
  else if (p=="ZZ_WINDOW_MS")     ZZ_WINDOW_MS=(unsigned long)v;
  else if (p=="ZZ_EXIT_MS")       ZZ_EXIT_MS=(unsigned long)v;
  else if (p=="ZZ_CENTER_MS")     ZZ_CENTER_MS=(int)v;
  else if (p=="INV_WHITE")        INV_WHITE=(int)v;
  else if (p=="XING_MIN")         XING_MIN=(int)v;
  else if (p=="INV_CONFIRM_MS")   INV_CONFIRM_MS=(int)v;
  else if (p=="XING_SPEED")       XING_SPEED=(int)v;
  else if (p=="XING_GAIN")        XING_GAIN=v;
  else if (p=="INV_PROBE_MS")     INV_PROBE_MS=(int)v;
  else if (p=="Y_SIDE")           Y_SIDE=(int)v;
  else if (p=="Y_MIN")            Y_MIN=(int)v;
  else if (p=="SIGN_GAIN")        SIGN_GAIN=v;
  else if (p=="SIGN_CONFIRM_MS")  SIGN_CONFIRM_MS=(int)v;
  else if (p=="SIGN_MIN_FRAC")    SIGN_MIN_FRAC=v;
  else if (p=="JUMP_MAX")         JUMP_MAX=v;
  else if (p=="Y_EXIT_W")         Y_EXIT_W=(int)v;
  else if (p=="Y_EXIT_MS")        Y_EXIT_MS=(int)v;
  else if (p=="EDGE_ZONE")        EDGE_ZONE=constrain((int)v,1,4);
  else if (p=="EDGE_CONFIRM_MS")  EDGE_CONFIRM_MS=(int)v;
  else if (p=="EDGE_HOLD_MS")     EDGE_HOLD_MS=(int)v;
  else if (p=="EDGE_TURN_SPEED")  EDGE_TURN_SPEED=(int)v;
  else if (p=="EDGE_TURN_MS")     EDGE_TURN_MS=(int)v;
  else if (p=="EDGE_BLANK_MS")    EDGE_BLANK_MS=(int)v;
  else if (p=="EDGE_EXIT_POS")    EDGE_EXIT_POS=v;
  else if (p=="EDGE_SWAP")        EDGE_SWAP=(int)v?1:0;
  else if (p=="DIR_SWAP")         DIR_SWAP=(int)v?1:0;
  else if (p=="TURN_COOLDOWN_MS") TURN_COOLDOWN_MS=(unsigned long)v;
  else if (p=="GAP_FWD_MS")       GAP_FWD_MS=(int)v;
  else if (p=="GAP_SPEED")        GAP_SPEED=(int)v;
  else if (p=="GAP_EXPECT_MS")    GAP_EXPECT_MS=(int)v;
  else if (p=="LOST_BACK_MS")     LOST_BACK_MS=(int)v;
  else if (p=="SPIN_BLANK_MS")    SPIN_BLANK_MS=(int)v;
  else if (p=="SPIN_360_MS")      SPIN_360_MS=(int)v;
  else if (p=="SEARCH_FWD_MS")    SEARCH_FWD_MS=(int)v;
  else if (p=="KICK_PWM")         KICK_PWM=(int)v;
  else if (p=="KICK_MS")          KICK_MS=(int)v;
  else if (p=="KICK_MIN_CMD")     KICK_MIN_CMD=(int)v;
  else if (p=="FULL_BLACK_STOP")  FULL_BLACK_STOP=(bool)(int)v;
  else if (p=="FULL_BLACK_BACK_MS") FULL_BLACK_BACK_MS=(int)v;
  else if (p=="MAX_REVERSE")      MAX_REVERSE=(int)v;
  else if (p=="SPIN_SPEED")       SPIN_SPEED=(int)v;
  else if (p=="MUX_SETTLE_US")    MUX_SETTLE_US=(int)v;
  server.send(200,"text/plain","ok");
}

void handleToggle() {
  running = !running;
  if (!running) stopMotors();
  resetState();
  server.send(200,"text/plain", running?"1":"0");
}

void handleData() {
  unsigned long now = millis();
  bool eL = (edgeLlatch != 0) && (now - edgeLlatch <= (unsigned long)EDGE_HOLD_MS);
  bool eR = (edgeRlatch != 0) && (now - edgeRlatch <= (unsigned long)EDGE_HOLD_MS);

  String j = "{\"params\":{";
  j += "\"Kp\":"+String(Kp,1)+",\"Kd\":"+String(Kd,0)+",\"base\":"+String(base)+",";
  j += "\"BOOST_XP\":"+String(BOOST_XP)+",\"BOOST_BAND\":"+String(BOOST_BAND,2)+",";
  j += "\"SPEED_LIMIT\":"+String(SPEED_LIMIT)+",\"ERR_HOLD_MS\":"+String(ERR_HOLD_MS)+",";
  j += "\"THRESHOLD\":"+String(THRESHOLD)+",\"NOISE\":"+String(NOISE)+",";
  j += "\"PIVOT_THRESHOLD\":"+String(PIVOT_THRESHOLD,1)+",\"PIVOT_SPEED\":"+String(PIVOT_SPEED)+",";
  j += "\"PIVOT_TIMEOUT_MS\":"+String(PIVOT_TIMEOUT_MS)+",";
  j += "\"ZZ_BASE\":"+String(ZZ_BASE)+",\"ZZ_TRIGGER\":"+String(ZZ_TRIGGER)+",";
  j += "\"ZZ_WINDOW_MS\":"+String(ZZ_WINDOW_MS)+",\"ZZ_EXIT_MS\":"+String(ZZ_EXIT_MS)+",";
  j += "\"ZZ_CENTER_MS\":"+String(ZZ_CENTER_MS)+",";
  j += "\"INV_WHITE\":"+String(INV_WHITE)+",\"XING_MIN\":"+String(XING_MIN)+",";
  j += "\"INV_CONFIRM_MS\":"+String(INV_CONFIRM_MS)+",\"XING_SPEED\":"+String(XING_SPEED)+",";
  j += "\"XING_GAIN\":"+String(XING_GAIN,2)+",\"INV_PROBE_MS\":"+String(INV_PROBE_MS)+",";
  j += "\"Y_SIDE\":"+String(Y_SIDE)+",\"Y_MIN\":"+String(Y_MIN)+",";
  j += "\"SIGN_GAIN\":"+String(SIGN_GAIN,2)+",";
  j += "\"SIGN_CONFIRM_MS\":"+String(SIGN_CONFIRM_MS)+",";
  j += "\"SIGN_MIN_FRAC\":"+String(SIGN_MIN_FRAC,2)+",";
  j += "\"JUMP_MAX\":"+String(JUMP_MAX,1)+",";
  j += "\"Y_EXIT_W\":"+String(Y_EXIT_W)+",\"Y_EXIT_MS\":"+String(Y_EXIT_MS)+",";
  j += "\"EDGE_ZONE\":"+String(EDGE_ZONE)+",\"EDGE_CONFIRM_MS\":"+String(EDGE_CONFIRM_MS)+",";
  j += "\"EDGE_HOLD_MS\":"+String(EDGE_HOLD_MS)+",\"EDGE_TURN_SPEED\":"+String(EDGE_TURN_SPEED)+",";
  j += "\"EDGE_TURN_MS\":"+String(EDGE_TURN_MS)+",\"EDGE_BLANK_MS\":"+String(EDGE_BLANK_MS)+",";
  j += "\"EDGE_EXIT_POS\":"+String(EDGE_EXIT_POS,2)+",";
  j += "\"EDGE_SWAP\":"+String(EDGE_SWAP)+",\"DIR_SWAP\":"+String(DIR_SWAP)+",";
  j += "\"TURN_COOLDOWN_MS\":"+String(TURN_COOLDOWN_MS)+",";
  j += "\"GAP_FWD_MS\":"+String(GAP_FWD_MS)+",\"GAP_SPEED\":"+String(GAP_SPEED)+",";
  j += "\"GAP_EXPECT_MS\":"+String(GAP_EXPECT_MS)+",";
  j += "\"LOST_BACK_MS\":"+String(LOST_BACK_MS)+",\"SPIN_BLANK_MS\":"+String(SPIN_BLANK_MS)+",";
  j += "\"SPIN_360_MS\":"+String(SPIN_360_MS)+",\"SEARCH_FWD_MS\":"+String(SEARCH_FWD_MS)+",";
  j += "\"KICK_PWM\":"+String(KICK_PWM)+",\"KICK_MS\":"+String(KICK_MS)+",";
  j += "\"KICK_MIN_CMD\":"+String(KICK_MIN_CMD)+",";
  j += "\"FULL_BLACK_STOP\":"+String(FULL_BLACK_STOP?1:0)+",";
  j += "\"FULL_BLACK_BACK_MS\":"+String(FULL_BLACK_BACK_MS)+",";
  j += "\"MAX_REVERSE\":"+String(MAX_REVERSE)+",\"SPIN_SPEED\":"+String(SPIN_SPEED)+",";
  j += "\"MUX_SETTLE_US\":"+String(MUX_SETTLE_US);
  j += "},\"bar\":\""+String(bar)+"\",\"tp\":"+String(trackPos,2)+",";
  j += "\"eL\":"+String(eL?"true":"false")+",\"eR\":"+String(eR?"true":"false")+",";
  j += "\"side\":"+String(routeSide())+",\"inev\":"+String(inEvent?"true":"false")+",";
  j += "\"err\":"+String(t_err,2)+",\"corr\":"+String(t_corr,0)+",";
  j += "\"L\":"+String(t_L)+",\"R\":"+String(t_R)+",";
  j += "\"online\":"+String(t_online?"true":"false")+",";
  j += "\"inv\":"+String(inverted?"true":"false")+",";
  j += "\"lc\":"+String(lineCount)+",\"nc\":"+String(nSig)+",\"w\":"+String(t_wmax)+",";
  j += "\"center\":"+String(centerHit?"true":"false")+",";
  j += "\"state\":\""+t_state+"\",";
  j += "\"loop\":"+String(loopUs)+",\"loopmax\":"+String(loopMaxUs)+",";
  j += "\"run\":"+String(running?"true":"false")+"}";
  server.send(200,"application/json",j);
}

// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcAttach(ENA,1000,8);
  ledcAttach(ENB,1000,8);

  pinMode(S0,OUTPUT); pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT); pinMode(S3,OUTPUT);
  pinMode(button, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  stopMotors();

  WiFi.softAP("LineFollower","12345678");
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());

  server.on("/",       handleRoot);
  server.on("/set",    handleSet);
  server.on("/data",   handleData);
  server.on("/toggle", handleToggle);
  server.begin();
}

// ================================================================
void loop() {
  unsigned long t0 = micros();
  server.handleClient();
  readSensors();

  bool b = digitalRead(button);
  if (b == LOW && lastButton == HIGH && millis() - lastButtonMs > 250) {
    running = !running;
    if (!running) stopMotors();
    resetState();
    lastButtonMs = millis();
  }
  lastButton = b;

  if (running) control();
  else { stopMotors(); t_state = "stopped"; }

  loopUs = micros() - t0;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
  delay(4);
}
