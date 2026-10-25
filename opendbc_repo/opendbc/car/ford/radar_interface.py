# BluePilot: collections used by STEER_ASSIST_DATA radar tracking
import collections
import numpy as np
from typing import cast
from collections import defaultdict
from math import cos, sin
from dataclasses import dataclass
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x174  # MRR_Header_SensorCoverage
DELPHI_MRR_RADAR_MSG_COUNT = 64
# BluePilot: DELPHI_MRR_64 / STEER_ASSIST_DATA radar constants
DELPHI_MRR_RADAR_MSG_COUNT_64 = 22  # 22 messages in CANFD

DELPHI_MRR_RADAR_RANGE_COVERAGE = {0: 42, 1: 164, 2: 45, 3: 175}  # scan index to detection range (m)
DELPHI_MRR_MIN_LONG_RANGE_DIST = 30  # meters
DELPHI_MRR_CLUSTER_THRESHOLD = 5  # meters, lateral distance and relative velocity are weighted

STEER_ASSIST_DATA_MSGS = 0x3d7

@dataclass
class Cluster:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


def cluster_points(pts_l: list[list[float]], pts2_l: list[list[float]], max_dist: float) -> list[int]:
  """
  Clusters a collection of points based on another collection of points. This is useful for correlating clusters through time.
  Points in pts2 not close enough to any point in pts are assigned -1.
  Args:
    pts_l: List of points to base the new clusters on
    pts2_l: List of points to cluster using pts
    max_dist: Max distance from cluster center to candidate point

  Returns:
    List of cluster indices for pts2 that correspond to pts
  """

  if not len(pts2_l):
    return []

  if not len(pts_l):
    return [-1] * len(pts2_l)

  max_dist_sq = max_dist ** 2
  pts = np.array(pts_l)
  pts2 = np.array(pts2_l)

  # Compute squared norms
  pts_norm_sq = np.sum(pts ** 2, axis=1)
  pts2_norm_sq = np.sum(pts2 ** 2, axis=1)

  # Compute squared Euclidean distances using the identity
  # dist_sq[i, j] = ||pts2[i]||^2 + ||pts[j]||^2 - 2 * pts2[i] . pts[j]
  dist_sq = pts2_norm_sq[:, np.newaxis] + pts_norm_sq[np.newaxis, :] - 2 * np.dot(pts2, pts.T)
  dist_sq = np.maximum(dist_sq, 0.0)

  # Find the closest cluster for each point and assign its index
  closest_clusters = np.argmin(dist_sq, axis=1)
  closest_dist_sq = dist_sq[np.arange(len(pts2)), closest_clusters]
  cluster_idxs = np.where(closest_dist_sq < max_dist_sq, closest_clusters, -1)

  return cast(list[int], cluster_idxs.tolist())


def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  msg_n = len(DELPHI_ESR_RADAR_MSGS)
  messages = list(zip(DELPHI_ESR_RADAR_MSGS, [20] * msg_n, strict=True))

  return CANParser(RADAR.DELPHI_ESR, messages, CanBus(CP).radar)


def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  messages = [
    ("MRR_Header_InformationDetections", 33),
    ("MRR_Header_SensorCoverage", 33),
  ]

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 33)]

  return CANParser(RADAR.DELPHI_MRR, messages, CanBus(CP).radar)


# BluePilot: CAN-FD 64-message MRR (DELPHI_MRR_64) radar parser
def _create_delphi_mrr_radar_can_parser_64(CP) -> CANParser:
  messages = []

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT_64 + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 20)]

  return CANParser(RADAR.DELPHI_MRR_64, messages, CanBus(CP).radar)

# BluePilot: Steer_Assist_Data (Cmbb) pseudo-radar parser
def _create_steer_assist_data(CP) -> CANParser:
  messages = [("Steer_Assist_Data", 20)]
  return CANParser(RADAR.STEER_ASSIST_DATA, messages, CanBus(CP).camera)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    self.points: list[list[float]] = []
    self.clusters: list[Cluster] = []
    # BluePilot: vRel history buffer for STEER_ASSIST_DATA
    self.vRelCol = {}

    self.updated_messages = set()
    self.track_id = 0
    self.radar = DBC[CP.carFingerprint].get(Bus.radar)
    self.scan_index_invalid_cnt = 0
    self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = 0
    # BluePilot: post-recovery hold for radarUnavailableTemporary, see _update_delphi_mrr
    self.radar_unavailable_hold_frames = 0
    if CP.radarUnavailable:
      self.rcp = None
    elif self.radar == RADAR.DELPHI_ESR:
      self.rcp = _create_delphi_esr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_ESR_RADAR_MSGS[-1]
      self.valid_cnt = {key: 0 for key in DELPHI_ESR_RADAR_MSGS}
    elif self.radar == RADAR.DELPHI_MRR:
      self.rcp = _create_delphi_mrr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_MRR_RADAR_HEADER_ADDR
    # BluePilot: DELPHI_MRR_64 / STEER_ASSIST_DATA radar setup branches
    elif self.radar == RADAR.DELPHI_MRR_64:
      self.rcp = _create_delphi_mrr_radar_can_parser_64(CP)
      self.trigger_msg = DELPHI_MRR_RADAR_START_ADDR + DELPHI_MRR_RADAR_MSG_COUNT_64 - 1
    elif self.radar == RADAR.STEER_ASSIST_DATA:
      self.rcp = _create_steer_assist_data(CP)
      self.trigger_msg = STEER_ASSIST_DATA_MSGS
    else:
      raise ValueError(f"Unsupported radar: {self.radar}")

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()
    elif self.radar == RADAR.DELPHI_MRR:
      _update = self._update_delphi_mrr(ret)
      if not _update:
        return None
    # BluePilot: DELPHI_MRR_64 / STEER_ASSIST_DATA radar update branches
    elif self.radar == RADAR.DELPHI_MRR_64:
      _update = self._update_delphi_mrr_64(ret)
      if not _update:
        return None
    elif self.radar == RADAR.STEER_ASSIST_DATA:
      _update = self._update_steer_assist_data()
      if not _update:
        return None

    ret.points = list(self.pts.values())
    return ret

  # BluePilot: Steer_Assist_Data (Cmbb) radar point update
  def _update_steer_assist_data(self):
    msg = self.rcp.vl["Steer_Assist_Data"]

    dRel = msg['CmbbObjDistLong_L_Actl']
    confidence = msg['CmbbObjConfdnc_D_Stat']
    new_track = False

    # if dRel < 1022:
    if confidence > 0:
      if 0 not in self.pts:
        self.pts[0] = structs.RadarData.RadarPoint()
        self.pts[0].trackId = self.track_id
        self.vRelCol[0] = collections.deque(maxlen=20)
        self.track_id += 1
        new_track = True

      yRel = msg['CmbbObjDistLat_L_Actl']
      vRel = msg['CmbbObjRelLong_V_Actl']
      yvRel = msg['CmbbObjRelLat_V_Actl']
      if not new_track:
        # if this is a newly created track - we don't have historical data so skip it
        # if we are on the same track
        # Let's see if we are moving:
        #   positive gap - lead is moving faster than us
        #   negative gap - lead is moving slower than us
        dDiff = dRel - self.pts[0].dRel
        if (abs(vRel) < 1.0e-2):
          self.vRelCol[0].append(dDiff)
          vRel = sum(self.vRelCol[0])
          calc = 1
        else:
          if len(self.vRelCol[0]) > 0:
            self.vRelCol[0].clear()

        if abs(self.pts[0].vRel - vRel) > 2 or abs(self.pts[0].dRel - dRel) > 5:
          self.pts[0].trackId = self.track_id
          if len(self.vRelCol[0]) > 0:
            self.vRelCol[0].clear()
          self.track_id += 1

      self.pts[0].dRel = dRel  # from front of car
      self.pts[0].yRel = yRel  # in car frame's y axis, left is positive
      self.pts[0].vRel = vRel
      self.pts[0].aRel = float('nan')
      self.pts[0].yvRel = yvRel
      self.pts[0].measured = True
    else:
      if 0 in self.pts:
        del self.pts[0]
        del self.vRelCol[0]
    return True

  def _update_delphi_esr(self):
    for ii in sorted(self.updated_messages):
      cpt = self.rcp.vl[ii]

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] = 0    # reset counter

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] += 1
      else:
        self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)
      #print ii, self.valid_cnt[ii], cpt['VALID'], cpt['X_Rel'], cpt['Angle']

      # radar point only valid if there have been enough valid measurements
      if self.valid_cnt[ii] > 0:
        if ii not in self.pts:
          self.pts[ii] = structs.RadarData.RadarPoint()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].dRel = cpt['X_Rel']  # from front of car
        self.pts[ii].yRel = cpt['X_Rel'] * cpt['Angle'] * CV.DEG_TO_RAD  # in car frame's y axis, left is positive
        self.pts[ii].vRel = cpt['V_Rel']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')
        self.pts[ii].measured = True
      else:
        if ii in self.pts:
          del self.pts[ii]

  # BluePilot: how long to keep reporting radarUnavailableTemporary after the raw scan-index
  # freeze (below) clears, in radar frames (~33Hz, MRR_Header_InformationDetections). See the
  # hold-frames block in _update_delphi_mrr for why.
  RADAR_UNAVAILABLE_HOLD_FRAMES = 33  # ~1s

  def _update_delphi_mrr(self, ret: structs.RadarData):
    headerScanIndex = int(self.rcp.vl["MRR_Header_InformationDetections"]['CAN_SCAN_INDEX']) & 0b11

    # In reverse, the radar continually sends the last messages. Mark this as invalid
    if (self.prev_headerScanIndex + 1) % 4 != headerScanIndex:
      self.radar_unavailable_cnt += 1
    else:
      self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = headerScanIndex

    if self.radar_unavailable_cnt >= 5:
      self.pts.clear()
      self.points.clear()
      self.clusters.clear()
      ret.errors.radarUnavailableTemporary = True
      # BluePilot: reset every frame we're still actually frozen, so the hold below always
      # starts counting down from the real recovery moment, not from when the freeze began.
      self.radar_unavailable_hold_frames = self.RADAR_UNAVAILABLE_HOLD_FRAMES
      return True

    # BluePilot: the raw freeze above clears as soon as a fresh CAN_SCAN_INDEX shows up, but on
    # Reverse->Drive the Delphi MRR takes noticeably longer (~0.6-0.9s observed on a Ford
    # Explorer MK6) to actually resume producing valid detections than it does to un-freeze the
    # header index. Without this hold, selfdrived's generic commIssue catch-all can race into
    # that gap the instant radarUnavailableTemporary clears: this flag's own NO_ENTRY+SOFT_DISABLE
    # tier normally suppresses the generic check entirely while it's set (has_disable_events in
    # selfdrived.py), so holding it open a beat longer keeps that existing suppression covering
    # the real recovery window instead of dropping a beat early. See BluePilotDev/bluepilot#188.
    if self.radar_unavailable_hold_frames > 0:
      self.radar_unavailable_hold_frames -= 1
      self.pts.clear()
      self.points.clear()
      self.clusters.clear()
      ret.errors.radarUnavailableTemporary = True
      return True

    # Use points with Doppler coverage of +-60 m/s, reduces similar points
    if headerScanIndex not in (2, 3):
      return False

    if DELPHI_MRR_RADAR_RANGE_COVERAGE[headerScanIndex] != int(self.rcp.vl["MRR_Header_SensorCoverage"]["CAN_RANGE_COVERAGE"]):
      self.scan_index_invalid_cnt += 1
    else:
      self.scan_index_invalid_cnt = 0

    # Rarely MRR_Header_InformationDetections can fail to send a message. The scan index is skipped in this case
    if self.scan_index_invalid_cnt >= 5:
      ret.errors.wrongConfig = True

    for ii in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
      msg = self.rcp.vl[f"MRR_Detection_{ii:03d}"]

      # SCAN_INDEX rotates through 0..3 on each message for different measurement modes
      # Indexes 0 and 2 have a max range of ~40m, 1 and 3 are ~170m (MRR_Header_SensorCoverage->CAN_RANGE_COVERAGE)
      # Indexes 0 and 1 have a Doppler coverage of +-71 m/s, 2 and 3 have +-60 m/s
      scanIndex = msg[f"CAN_SCAN_INDEX_2LSB_{ii:02d}"]

      # Throw out old measurements. Very unlikely to happen, but is proper behavior
      if scanIndex != headerScanIndex:
        continue

      valid = bool(msg[f"CAN_DET_VALID_LEVEL_{ii:02d}"])

      # TODO: verify this is correct for CANFD as well - copied from CAN version
      # Long range measurement mode is more sensitive and can detect the road surface
      dist = msg[f"CAN_DET_RANGE_{ii:02d}"]  # m [0|255.984]
      if scanIndex in (1, 3) and dist < DELPHI_MRR_MIN_LONG_RANGE_DIST:
        valid = False

      if valid:
        azimuth = msg[f"CAN_DET_AZIMUTH_{ii:02d}"]              # rad [-3.1416|3.13964]
        distRate = msg[f"CAN_DET_RANGE_RATE_{ii:02d}"]          # m/s [-128|127.984]
        dRel = cos(azimuth) * dist                              # m from front of car
        yRel = -sin(azimuth) * dist                             # in car frame's y axis, left is positive

        self.points.append([dRel, yRel * 2, distRate * 2])

    # Cluster and publish using stored points once we've cycled through all 4 scan modes
    if headerScanIndex != 3:
      return False

    self.do_clustering()
    return True

  # BluePilot: DELPHI_MRR_64 radar update
  def _update_delphi_mrr_64(self, ret: structs.RadarData):
    # Ensure all point IDs match. Note that this message is sent first, but trigger_msg waits for the last message to come in
    headerScanIndex = int(self.rcp.vl["MRR_Detection_001"]['CAN_SCAN_INDEX_2LSB_01_01'])

    # TODO: Verify the below is correct for CANFD as well - copied from CAN version
    #       Use points with Doppler coverage of +-60 m/s, reduces similar points
    if headerScanIndex in (0, 1):
      return False

    for ii in range(1, DELPHI_MRR_RADAR_MSG_COUNT_64 + 1):
      msg = self.rcp.vl[f"MRR_Detection_{ii:03d}"]

      # all messages have 6 points except the last one
      maxRangeID = 6 if ii < DELPHI_MRR_RADAR_MSG_COUNT_64 else 3
      for iii in range(1, maxRangeID + 1):

        # SCAN_INDEX rotates through 0..3
        # TODO: Verify the below is correct for CANFD as well - copied from CAN version
        #       Indexes 0 and 2 have a max range of ~40m, 1 and 3 are ~170m (MRR_Header_SensorCoverage->CAN_RANGE_COVERAGE)
        #       Indexes 0 and 1 have a Doppler coverage of +-71 m/s, 2 and 3 have +-60 m/s
        scanIndex = msg[f"CAN_SCAN_INDEX_2LSB_{ii:02d}_{iii:02d}"]

        # Throw out old measurements. Very unlikely to happen, but is proper behavior
        if scanIndex != headerScanIndex:
          continue

        valid = bool(msg[f"CAN_DET_VALID_LEVEL_{ii:02d}_{iii:02d}"])

        # Long range measurement mode is more sensitive and can detect the road surface
        dist = msg[f"CAN_DET_RANGE_{ii:02d}_{iii:02d}"]  # m [0|255.984]
        if scanIndex in (1, 3) and dist < DELPHI_MRR_MIN_LONG_RANGE_DIST:
          valid = False

        if valid:
          azimuth = msg[f"CAN_DET_AZIMUTH_{ii:02d}_{iii:02d}"]              # rad [-3.1416|3.13964]
          distRate = msg[f"CAN_DET_RANGE_RATE_{ii:02d}_{iii:02d}"]          # m/s [-128|127.984]
          dRel = cos(azimuth) * dist                                        # m from front of car
          yRel = sin(azimuth) * dist                                        # in car frame's y axis, right is positive

          self.points.append([dRel, yRel * 2, distRate * 2])

    if headerScanIndex != 3:
      return True

    # Update the points once we've cycled through all 4 scan modes
    self.do_clustering()
    return True

  # Do the common work for CAN and CANFD clustering and prepare the points to be used for liveTracks
  def do_clustering(self):
    # Cluster points from this cycle against the centroids from the previous cycle
    prev_keys = [[p.dRel, p.yRel * 2, p.vRel * 2] for p in self.clusters]
    labels = cluster_points(prev_keys, self.points, DELPHI_MRR_CLUSTER_THRESHOLD)

    points_by_track_id = defaultdict(list)
    for idx, label in enumerate(labels):
      if label != -1:
        points_by_track_id[self.clusters[label].trackId].append(self.points[idx])
      else:
        points_by_track_id[self.track_id].append(self.points[idx])
        self.track_id += 1

    self.clusters = []
    for idx, (track_id, pts) in enumerate(points_by_track_id.items()):
      dRel = [p[0] for p in pts]
      min_dRel = min(dRel)
      dRel = sum(dRel) / len(dRel)

      yRel = [p[1] for p in pts]
      yRel = sum(yRel) / len(yRel) / 2

      vRel = [p[2] for p in pts]
      vRel = sum(vRel) / len(vRel) / 2

      # FIXME: creating capnp RadarPoint and accessing attributes are both expensive, so we store a dataclass and reuse the RadarPoint
      self.clusters.append(Cluster(dRel=dRel, yRel=yRel, vRel=vRel, trackId=track_id))

      if idx not in self.pts:
        self.pts[idx] = structs.RadarData.RadarPoint(measured=True, aRel=float('nan'), yvRel=float('nan'))

      self.pts[idx].dRel = min_dRel
      self.pts[idx].yRel = yRel
      self.pts[idx].vRel = vRel
      self.pts[idx].trackId = track_id

    for idx in range(len(points_by_track_id), len(self.pts)):
      del self.pts[idx]

    self.points = []
