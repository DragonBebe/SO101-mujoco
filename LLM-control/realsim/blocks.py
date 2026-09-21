"""What the real blocks are, and where each number came from.

Only one thing in this file is measured by the camera at run time: nothing.
Every value here is a property of the physical object that has to be entered
by hand, so each carries a ``source`` string and the loader refuses a value
whose source says it was guessed.  The split matters downstream: a block's
size is what makes the monocular estimate possible at all, while its mass and
friction are simulation defaults that no part of this project has measured.

A block description is deliberately close to ``so101_nexus.objects``: an
axis-aligned box with a half-extent and a named colour, so the simulation
side can build the matching ``CubeObject`` without a translation table.
"""
from dataclasses import dataclass, field
from pathlib import Path
import json
import math

import numpy as np

from nexus_vision import perception

ROOT = Path(__file__).resolve().parent
DEFAULT_CATALOGUE = ROOT / 'config/blocks.json'
SCHEMA = 'realsim/blocks/1'
#: Colour names ``nexus_vision.perception`` defines a hue band for.  A block
#: is named with one of these, and may narrow or move the band with
#: ``hue_deg`` -- see :class:`ColourSpec`.
DETECTABLE_COLOURS = ('red', 'orange', 'yellow', 'green', 'blue')
#: ``perception.locate_color``'s own saturation and value floors, reused so a
#: block that says nothing about colour behaves exactly as that function does.
DEFAULT_MIN_SATURATION, DEFAULT_MIN_VALUE = 0.35, 0.20
#: A size whose source says it was never measured cannot be used: the whole
#: monocular estimate is scaled by it.
UNMEASURED = ('guess', 'guessed', 'unknown', 'assumed', 'todo', '')


@dataclass(frozen=True)
class ColourSpec:
    """Which pixels belong to one block, in hue/saturation/value.

    ``perception``'s bands were drawn for *rendered* colours, where green is
    120 degrees and blue is 240.  Real paint is not there: the teal cube on
    this table sits at hue 164, five degrees from the 170-degree boundary
    between ``perception``'s green and blue, so its shaded face crosses into
    blue and the mask loses a corner -- which the area check then, correctly,
    refuses.  A block may therefore carry its own band, measured from an
    image of that block with ``run_realsim.sh colours``.

    The default is exactly ``perception``'s band for the named colour, so a
    catalogue that says nothing about hue behaves as ``locate_color`` does.
    """

    name: str
    #: One or more ``(low, high)`` half-open bands in degrees.  A band that
    #: wraps past 360 is stored as two, the way ``perception`` stores red.
    hue_ranges: tuple
    min_saturation: float = DEFAULT_MIN_SATURATION
    min_value: float = DEFAULT_MIN_VALUE
    #: Saturation and value used only for *rendering* the block in the
    #: simulation; they say nothing about detection, whose floors are above.
    saturation: float = 0.85
    value: float = 0.80
    source: str = "perception's band for this colour name"

    def describe(self):
        return {'name': self.name,
                'hue_deg': [list(band) for band in self.hue_ranges],
                'min_saturation': self.min_saturation,
                'min_value': self.min_value,
                'render_rgb': [round(channel, 4)
                               for channel in self.representative_rgb()],
                'source': self.source}

    def representative_rgb(self):
        """A colour inside this band, for rendering the block in simulation.

        The mirror should look like the table it mirrors: a cube rendered in
        a canonical green when the real one is teal would be detected by a
        band the real one is not, which is exactly the bug this method was
        added to stop the self-test from hiding.

        The hue is the band's circular midpoint, so a band written across the
        wrap (red) does not average to cyan.
        """
        import colorsys

        import numpy as np

        mids, widths = [], []
        for low, high in self.hue_ranges:
            mids.append((low + high) / 2.0)
            widths.append(high - low)
        radians = np.radians(np.asarray(mids))
        weights = np.asarray(widths, dtype=float)
        hue = float(np.degrees(np.arctan2(
            float(np.sum(weights * np.sin(radians))),
            float(np.sum(weights * np.cos(radians)))))) % 360.0
        return tuple(colorsys.hsv_to_rgb(hue / 360.0, self.saturation,
                                         self.value))

    def overlaps(self, other):
        for low, high in self.hue_ranges:
            for other_low, other_high in other.hue_ranges:
                if low < other_high and other_low < high:
                    return True
        return False


def default_colour_spec(name):
    return ColourSpec(name=name,
                      hue_ranges=tuple(tuple(band) for band
                                       in perception._COLOR_HUE_RANGES[name]))


def colour_spec_from(name, entry):
    """A block's colour band: its own if it gives one, otherwise the default."""
    if 'hue_deg' not in entry:
        spec = default_colour_spec(name)
        if any(key in entry for key in ('min_saturation', 'min_value')):
            spec = ColourSpec(
                name=name, hue_ranges=spec.hue_ranges,
                min_saturation=float(entry.get('min_saturation',
                                               DEFAULT_MIN_SATURATION)),
                min_value=float(entry.get('min_value', DEFAULT_MIN_VALUE)),
                source=str(entry.get('colour_source',
                                     "perception's hue band, own saturation/value")))
        return spec
    band = entry['hue_deg']
    if len(band) != 2:
        raise ValueError(f"{entry.get('id')}: hue_deg must be [low, high] in degrees")
    low, high = (float(value) for value in band)
    if not all(0.0 <= value <= 360.0 for value in (low, high)):
        raise ValueError(f"{entry.get('id')}: hue_deg must lie in [0, 360]")
    # A band written across the wrap (e.g. [350, 10] for red) becomes two.
    ranges = ((low, high),) if low < high else ((low, 360.0), (0.0, high))
    saturation = float(entry.get('min_saturation', DEFAULT_MIN_SATURATION))
    value = float(entry.get('min_value', DEFAULT_MIN_VALUE))
    if not (0.0 <= saturation <= 1.0 and 0.0 <= value <= 1.0):
        raise ValueError(f"{entry.get('id')}: min_saturation and min_value are "
                         'fractions in [0, 1]')
    appearance = entry.get('render_hsv', {})
    return ColourSpec(name=name, hue_ranges=ranges, min_saturation=saturation,
                      min_value=value,
                      saturation=float(appearance.get('saturation', 0.85)),
                      value=float(appearance.get('value', 0.80)),
                      source=str(entry.get('colour_source',
                                           'measured for this block; no source given')))


@dataclass(frozen=True)
class Block:
    """One physical block and the simulated body that stands for it."""

    id: str
    colour: str
    #: Full side lengths in metres, ``[x, y, z]`` in the block's own frame,
    #: whose origin is the geometric centre of the box.
    size_m: tuple
    size_source: str
    #: True only when someone put a calliper on this block.  A nominal or
    #: catalogue size still works, but it scales every reported position, so
    #: the flag travels into every scene record instead of being forgotten.
    size_measured: bool
    mass_kg: float
    mass_source: str
    friction: tuple
    friction_source: str
    count_index: int = 1
    note: str = ''
    #: Which pixels are this block's.  Built by the loader; a Block made in
    #: code without one falls back to ``perception``'s band for its colour,
    #: so nothing has to pass a spec just to use the default.
    colour_spec: object = None

    def __post_init__(self):
        if self.colour_spec is None:
            object.__setattr__(self, 'colour_spec',
                               default_colour_spec(self.colour))

    @property
    def half_extent(self):
        return np.asarray(self.size_m, dtype=float) / 2.0

    @property
    def height_m(self):
        """Height of the block as it lies flat: the z side length."""
        return float(self.size_m[2])

    @property
    def footprint_m(self):
        return (float(self.size_m[0]), float(self.size_m[1]))

    @property
    def symmetry_step_rad(self):
        """Yaw step that maps the box onto itself.

        A square footprint repeats every 90 degrees, any other box every 180.
        The estimator cannot see past this, so it is a property of the object,
        not a tuning knob.
        """
        square = math.isclose(self.size_m[0], self.size_m[1], rel_tol=1e-6,
                              abs_tol=1e-9)
        return math.pi / 2 if square else math.pi

    @property
    def is_cube(self):
        return (math.isclose(self.size_m[0], self.size_m[1], rel_tol=1e-6) and
                math.isclose(self.size_m[0], self.size_m[2], rel_tol=1e-6))

    @property
    def diagonal_m(self):
        """Largest horizontal extent, used to gate colour blobs by size."""
        return float(np.hypot(self.size_m[0], self.size_m[1]))

    def describe(self):
        """The block as it appears in a scene record: values with provenance."""
        return {
            'id': self.id, 'colour': self.colour,
            'size_m': [float(value) for value in self.size_m],
            'size_source': self.size_source,
            'size_measured': bool(self.size_measured),
            'height_m': self.height_m,
            'symmetry_step_deg': math.degrees(self.symmetry_step_rad),
            'colour_spec': (self.colour_spec.describe() if self.colour_spec
                            else None),
            'shape': 'box',
            'origin': 'geometric centre of the box',
            'physics': {'mass_kg': self.mass_kg, 'mass_source': self.mass_source,
                        'friction': list(self.friction),
                        'friction_source': self.friction_source},
            'note': self.note,
        }


@dataclass
class Catalogue:
    """The blocks expected on the table, plus how they were measured."""

    blocks: list
    measured_at: str = ''
    source_path: str = ''
    note: str = ''
    tolerances: dict = field(default_factory=dict)

    def by_id(self, block_id):
        for block in self.blocks:
            if block.id == block_id:
                return block
        raise KeyError(f'no block {block_id!r} in the catalogue')

    def by_colour(self):
        """Blocks grouped by colour, since the detector works one colour at a time."""
        groups = {}
        for block in self.blocks:
            groups.setdefault(block.colour, []).append(block)
        return groups

    def describe(self):
        return {'schema': SCHEMA, 'measured_at': self.measured_at,
                'source': self.source_path, 'note': self.note,
                'tolerances': dict(self.tolerances),
                'blocks': [block.describe() for block in self.blocks]}


def _size(entry):
    """A block's full side lengths, from a cube edge or three sides."""
    if 'edge_mm' in entry:
        edge = float(entry['edge_mm']) / 1000.0
        return (edge, edge, edge)
    if 'size_mm' in entry:
        values = [float(value) / 1000.0 for value in entry['size_mm']]
        if len(values) != 3:
            raise ValueError(f"{entry.get('id')}: size_mm must have three values")
        return tuple(values)
    raise ValueError(f"{entry.get('id')}: give edge_mm or size_mm (millimetres)")


def load(path=None):
    """Read a catalogue, refusing anything that would silently be wrong."""
    path = Path(path or DEFAULT_CATALOGUE)
    if not path.exists():
        raise FileNotFoundError(
            f'no block catalogue at {path}. Copy '
            f'{DEFAULT_CATALOGUE.relative_to(ROOT.parent)} and fill in the '
            'measured sizes of your blocks.')
    data = json.loads(path.read_text())
    if data.get('schema') != SCHEMA:
        raise ValueError(f'{path} is not a {SCHEMA} catalogue')
    blocks, seen = [], set()
    for entry in data.get('blocks', []):
        identifier = str(entry.get('id', '')).strip()
        if not identifier or identifier in seen:
            raise ValueError(f'block ids must be present and unique; got {identifier!r}')
        seen.add(identifier)
        colour = str(entry.get('colour', '')).strip().lower()
        if colour not in DETECTABLE_COLOURS:
            raise ValueError(
                f'{identifier}: colour {colour!r} is not one this pipeline can '
                f'threshold ({", ".join(DETECTABLE_COLOURS)})')
        size = _size(entry)
        if not all(0.005 <= value <= 0.20 for value in size):
            raise ValueError(f'{identifier}: side lengths {size} are outside 5..200 mm')
        size_source = str(entry.get('size_source', '')).strip()
        if size_source.lower() in UNMEASURED:
            raise ValueError(
                f'{identifier}: size_source is {size_source!r}. The block size sets '
                'the scale of every position this pipeline reports, so it has to be '
                'measured and the measurement named here.')
        physics = entry.get('physics', {})
        blocks.append(Block(
            id=identifier, colour=colour, size_m=size, size_source=size_source,
            size_measured=bool(entry.get('size_measured', False)),
            mass_kg=float(physics.get('mass_kg', 0.01)),
            mass_source=str(physics.get('mass_source', 'simulation default, not measured')),
            friction=tuple(physics.get('friction', (1.0, 0.005, 0.0001))),
            friction_source=str(physics.get('friction_source',
                                            'MuJoCo default, not measured')),
            count_index=int(entry.get('count_index', 1)),
            note=str(entry.get('note', '')),
            colour_spec=colour_spec_from(colour, entry),
        ))
    if not blocks:
        raise ValueError(f'{path} lists no blocks')
    # Two blocks whose hue bands overlap but whose colours differ cannot be
    # told apart by any of this: one would claim the other's pixels.
    for index, block in enumerate(blocks):
        for other in blocks[index + 1:]:
            if block.colour == other.colour:
                continue
            if block.colour_spec.overlaps(other.colour_spec):
                raise ValueError(
                    f'{block.id} and {other.id} have overlapping hue bands '
                    f'({block.colour_spec.hue_ranges} and '
                    f'{other.colour_spec.hue_ranges}); no threshold can separate '
                    'them. Narrow one of them with hue_deg.')
    return Catalogue(blocks=blocks, measured_at=str(data.get('measured_at', '')),
                     source_path=str(path), note=str(data.get('note', '')),
                     tolerances=dict(data.get('tolerances', {})))
