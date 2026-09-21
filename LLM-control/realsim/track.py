"""Identity, smoothing and validity over a sequence of frames.

A single frame gives poses; a sequence has to say *whose* pose and *how old*.
Three things live here and nothing else does them:

Association
    Colour alone does not identify a block when two of them are the same
    colour.  Candidates of one colour are matched to the catalogue entries of
    that colour by nearest previous position within a gate, greedily from the
    best fit down; whatever is left over is assigned in a deterministic order
    so a cold start is repeatable.

Smoothing
    An exponential filter on position, and on yaw the same filter taken
    around the circle of the block's own symmetry -- averaging 89 and 1
    degree the ordinary way gives 45, which is a pose the block was never in.
    Raw and filtered values are both reported; the filter never replaces the
    measurement in the log.

Validity
    ``detected`` this frame, ``stale`` when the last good fit is older than
    ``stale_after_s`` but still recent enough to be worth reporting, and
    ``lost`` after ``lost_after_s``.  A stale pose is always returned with the
    timestamp it was measured at and its age; it is never re-stamped with the
    current time, because a mirrored scene built from a five-second-old pose
    and one built from a fresh one are not the same claim.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from . import estimate, geometry


@dataclass
class TrackSettings:
    """Gates and time constants for following blocks between frames."""

    #: A detection further than this from the last filtered position is a
    #: different object, not this one moved.
    association_gate_m: float = 0.08
    #: Exponential filter weight on the newest measurement; 1.0 disables it.
    position_alpha: float = 0.5
    yaw_alpha: float = 0.5
    #: A jump larger than this resets the filter instead of dragging the
    #: estimate across the gap: the block was moved, it is not noise.
    reset_jump_m: float = 0.02
    reset_yaw_rad: float = math.radians(20.0)
    stale_after_s: float = 0.5
    lost_after_s: float = 5.0


@dataclass
class Track:
    """One catalogue block followed through time."""

    block: object
    last_valid: dict = None
    filtered_position: np.ndarray = None
    filtered_yaw: float = None
    observations: int = 0
    misses: int = 0
    history: list = field(default_factory=list)

    def state(self, now, settings):
        if self.last_valid is None:
            return 'lost', None
        age = now - self.last_valid['observed_at_monotonic']
        if age <= settings.stale_after_s:
            return 'detected', age
        if age <= settings.lost_after_s:
            return 'stale', age
        return 'lost', age

    def update(self, candidate, now, settings):
        position = np.asarray(candidate['position'], dtype=float)
        yaw = float(candidate['yaw_rad'])
        step = self.block.symmetry_step_rad
        jumped = (self.filtered_position is None or
                  np.linalg.norm(position[:2] - self.filtered_position[:2]) >
                  settings.reset_jump_m or
                  geometry.yaw_difference(yaw, self.filtered_yaw, step) >
                  settings.reset_yaw_rad)
        if jumped:
            self.filtered_position, self.filtered_yaw = position, yaw
            self.history = []
        else:
            alpha = settings.position_alpha
            self.filtered_position = (alpha * position +
                                      (1 - alpha) * self.filtered_position)
            self.filtered_yaw = geometry.circular_mean(
                [yaw, self.filtered_yaw], step,
                [settings.yaw_alpha, 1 - settings.yaw_alpha])
        self.history.append((now, position.copy(), yaw))
        self.history = self.history[-30:]
        self.observations += 1
        self.misses = 0
        self.last_valid = dict(candidate)
        self.last_valid['observed_at_monotonic'] = now
        self.last_valid['filter_reset'] = bool(jumped)
        return jumped

    def jitter(self):
        """Spread of the recent raw measurements, when the block held still.

        Reported only if the block has not moved much over the window; a
        standard deviation taken across a real move is not jitter.
        """
        if len(self.history) < 5:
            return None
        positions = np.array([entry[1] for entry in self.history])
        if float(np.linalg.norm(np.ptp(positions, axis=0)[:2])) > 0.005:
            return None
        yaws = [entry[2] for entry in self.history]
        step = self.block.symmetry_step_rad
        mean_yaw = geometry.circular_mean(yaws, step)
        spread = [geometry.yaw_difference(value, mean_yaw, step) for value in yaws]
        return {'samples': len(self.history),
                'position_std_mm': (1000 * positions[:, :2].std(axis=0)).tolist(),
                'yaw_rms_deg': float(math.degrees(
                    math.sqrt(float(np.mean(np.square(spread))))))}


class Tracker:
    """Follows every block in a catalogue across frames of one camera."""

    def __init__(self, catalogue, settings=None, fit_settings=None):
        self.catalogue = catalogue
        self.settings = settings or TrackSettings()
        self.fit_settings = fit_settings or estimate.FitSettings()
        self.tracks = {block.id: Track(block) for block in catalogue.blocks}

    def seeds_for(self, colour):
        """Previous poses of this colour, used to narrow the pose search."""
        seeds = []
        for track in self.tracks.values():
            if track.block.colour == colour and track.filtered_position is not None:
                seeds.append({'position': track.filtered_position.tolist(),
                              'yaw_rad': track.filtered_yaw})
        return seeds

    def update(self, rgb, calibration, now, support_heights=None):
        """Estimate every block in one frame and fold it into the tracks.

        ``support_heights`` optionally names the plane each block rests on,
        for a scene that has been deliberately stacked.  Anything not named
        is assumed to be on the table, and a block that is in fact stacked
        then fails its own height check rather than being placed on the table.
        """
        support_heights = support_heights or {}
        detections, rejected = {}, []
        for colour, group in self.catalogue.by_colour().items():
            # One search per distinct (support height, size): the estimator
            # fits one box model, so blocks that differ in either cannot share
            # a pass even when they share a colour.
            subgroups = {}
            for block in group:
                key = (support_heights.get(block.id, 0.0), tuple(block.size_m))
                subgroups.setdefault(key, []).append(block)
            for (plane_z, _), members in sorted(subgroups.items()):
                candidates = estimate.estimate_block(
                    rgb, calibration, members[0], plane_z=plane_z,
                    settings=self.fit_settings, expected_count=len(members),
                    seeds=self.seeds_for(colour))
                accepted = [entry for entry in candidates if entry['valid']]
                rejected.extend(entry for entry in candidates if not entry['valid'])
                for block_id, candidate in self._assign(members, accepted).items():
                    detections[block_id] = candidate
        for block_id, candidate in detections.items():
            candidate['block_id'] = block_id
            self.tracks[block_id].update(candidate, now, self.settings)
        for block_id, track in self.tracks.items():
            if block_id not in detections:
                track.misses += 1
        return detections, rejected

    def _assign(self, members, candidates):
        """Match candidates of one colour to the catalogue blocks of that colour.

        Best fit first, to its nearest unclaimed track within the gate; then
        anything still unmatched, in a fixed order, so a cold start with two
        identical blocks is at least repeatable -- and says so, because the
        two are genuinely indistinguishable until one of them moves.
        """
        assignment, free = {}, list(members)
        remaining = list(candidates)
        for candidate in list(remaining):
            best, best_distance = None, self.settings.association_gate_m
            for block in free:
                track = self.tracks[block.id]
                if track.filtered_position is None:
                    continue
                distance = float(np.linalg.norm(
                    np.asarray(candidate['position'][:2]) -
                    track.filtered_position[:2]))
                if distance <= best_distance:
                    best, best_distance = block, distance
            if best is not None:
                assignment[best.id] = candidate
                candidate['association'] = {'method': 'nearest previous pose',
                                            'distance_m': best_distance}
                free.remove(best)
                remaining.remove(candidate)
        for block, candidate in zip(
                sorted(free, key=lambda item: item.id),
                sorted(remaining, key=lambda item: (round(item['position'][0], 3),
                                                    round(item['position'][1], 3)))):
            assignment[block.id] = candidate
            candidate['association'] = {
                'method': 'first sight, ordered by position',
                'note': ('blocks of the same colour and size cannot be told apart '
                         'on first sight; this assignment is a convention, not a '
                         'measurement')
                if len(members) > 1 else 'only one block of this colour'}
        return assignment
