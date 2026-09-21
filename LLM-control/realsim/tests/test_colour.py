"""Per-block colour bands: real paint is not where rendered colours are.

The teal cube on this table sits at hue 164, five degrees below the 170 that
``perception`` uses to divide green from blue, so its shaded face crossed
into blue and the mask lost a corner.  A block may therefore carry its own
band; these hold that the band is honoured, that two bands cannot overlap,
and that the simulation renders a block in its own colour rather than a
canonical one -- otherwise a self-test on rendered frames would pass with a
band the real block fails.
"""
import json
import math

import numpy as np
import pytest

from realsim import blocks, estimate, geometry
from realsim.tests.test_estimate import CUBE, draw
from realsim.tests.test_geometry import camera


def teal_block(**changes):
    entry = {'id': 'teal_cube', 'colour': 'green', 'edge_mm': 30.0,
             'size_source': 'test', 'size_measured': True,
             'hue_deg': [145.0, 182.0], 'min_saturation': 0.25}
    entry.update(changes)
    return entry


def catalogue_file(tmp_path, entries, name='blocks.json'):
    path = tmp_path / name
    path.write_text(json.dumps({'schema': blocks.SCHEMA, 'blocks': entries}))
    return path


def test_a_block_without_a_band_gets_perceptions_own():
    from nexus_vision import perception

    block = blocks.Block(id='x', colour='green', size_m=(0.03,) * 3,
                         size_source='t', size_measured=True, mass_kg=0.01,
                         mass_source='t', friction=(1, 0, 0), friction_source='t')
    assert block.colour_spec.hue_ranges == tuple(
        tuple(band) for band in perception._COLOR_HUE_RANGES['green'])
    assert block.colour_spec.min_saturation == 0.35


def test_a_band_written_across_the_wrap_becomes_two(tmp_path):
    entries = [teal_block(id='wrapped', colour='red', hue_deg=[350.0, 12.0])]
    spec = blocks.load(catalogue_file(tmp_path, entries)).blocks[0].colour_spec
    assert spec.hue_ranges == ((350.0, 360.0), (0.0, 12.0))


def test_overlapping_bands_of_different_colours_are_refused(tmp_path):
    entries = [teal_block(id='teal', colour='green', hue_deg=[145.0, 200.0]),
               teal_block(id='blue', colour='blue', hue_deg=[190.0, 265.0])]
    with pytest.raises(ValueError, match='overlapping hue bands'):
        blocks.load(catalogue_file(tmp_path, entries))


def test_bands_that_just_meet_are_allowed(tmp_path):
    entries = [teal_block(id='teal', colour='green', hue_deg=[145.0, 182.0]),
               teal_block(id='blue', colour='blue', hue_deg=[182.0, 265.0])]
    assert len(blocks.load(catalogue_file(tmp_path, entries)).blocks) == 2


def test_a_custom_band_finds_a_colour_perceptions_band_would_split(tmp_path):
    """A teal cube, at the hue the real one has, on either band."""
    calibration = camera()
    teal = blocks.load(catalogue_file(tmp_path, [teal_block()])).blocks[0]
    default = blocks.Block(id='teal_cube', colour='green', size_m=teal.size_m,
                           size_source='t', size_measured=True, mass_kg=0.01,
                           mass_source='t', friction=(1, 0, 0), friction_source='t')
    # Paint the cube the colour the real one is: hue 164, the value and
    # saturation measured off the table.
    import colorsys

    body = tuple(int(255 * channel)
                 for channel in colorsys.hsv_to_rgb(164 / 360, 0.46, 0.46))
    shaded = tuple(int(255 * channel)
                   for channel in colorsys.hsv_to_rgb(174 / 360, 0.40, 0.30))
    image = draw(calibration, [(teal, 0.26, 0.0, math.radians(20), 0.0)])
    mask = np.all(image == (20, 190, 20), axis=2)
    image[mask] = body
    # One face darker and a few degrees bluer, as a real shaded face is.
    rows, columns = np.nonzero(mask)
    edge = (columns.min() + columns.max()) // 2
    image[:, edge:][mask[:, edge:]] = shaded

    wide = estimate.estimate_block(image, calibration, teal)
    assert wide and wide[0]['valid'], wide[0]['reasons'] if wide else 'nothing found'
    assert np.linalg.norm(np.asarray(wide[0]['position'][:2]) - [0.26, 0.0]) < 0.002

    narrow = estimate.estimate_block(image, calibration, default)
    # perception's band stops at 170, so the shaded face is missing and the
    # area check refuses the fit rather than reporting a shrunken cube.
    assert not narrow or not narrow[0]['valid']


def test_the_rendered_colour_sits_inside_the_band():
    spec = blocks.ColourSpec(name='green', hue_ranges=((145.0, 182.0),))
    import colorsys

    hue = colorsys.rgb_to_hsv(*spec.representative_rgb())[0] * 360
    assert 145.0 <= hue < 182.0


def test_the_rendered_colour_of_a_wrapped_band_stays_red():
    spec = blocks.ColourSpec(name='red', hue_ranges=((0.0, 15.0), (345.0, 360.0)))
    red, green, blue = spec.representative_rgb()
    assert red > green and red > blue


@pytest.mark.skipif(not pytest.importorskip('importlib').util.find_spec('so101_nexus'),
                    reason='simulation side only')
def test_the_mirror_paints_each_block_its_own_colour(tmp_path):
    from realsim.mirror import MirrorSimulation

    catalogue = blocks.load()
    simulation = MirrorSimulation(catalogue, tmp_path / 'paint')
    try:
        for block in catalogue.blocks:
            geom = list(simulation.slots[block.id].geom_ids)[0]
            np.testing.assert_allclose(simulation.model.geom_rgba[geom][:3],
                                       block.colour_spec.representative_rgb(),
                                       atol=1e-6)
    finally:
        simulation.close()


def test_the_command_line_builds_in_either_environment():
    """Both sides build the same parser; neither has the other's dependencies.

    The simulation environment has no OpenCV and the real one has no
    ``so101_nexus``, so anything a subcommand needs must be imported inside
    that subcommand, never at module level or as an argument default.
    """
    from realsim.cli import build_parser

    parser = build_parser()
    commands = set(parser._subparsers._group_actions[0].choices)
    assert commands == {'check', 'drift', 'scale-check', 'colours', 'measure',
                        'observe', 'live', 'arm', 'mirror', 'snapshot', 'export',
                        'replay', 'selftest', 'arm-stream', 'sync', 'arm-replay'}
    # Parsing must work too: an argument default that imports is the trap.
    args = parser.parse_args(['drift'])
    assert args.limit == pytest.approx(3.0)
    assert parser.parse_args(['mirror', '--scene', 'x.json']).scene == 'x.json'


def test_every_command_runs_in_the_environment_it_needs():
    """The launcher's routing table and the parser must not drift apart.

    ``run_realsim.sh`` picks the interpreter from a shell ``case`` before any
    Python runs, so a command left out of it silently runs on the side that
    lacks its dependencies -- an ImportError instead of a result.
    """
    import re
    from pathlib import Path

    from realsim.cli import build_parser

    launcher = (Path(__file__).resolve().parents[2] / 'run_realsim.sh').read_text()
    match = re.search(r'^\s*([\w|-]+)\)\s*side=sim', launcher, re.MULTILINE)
    assert match, 'the launcher no longer routes anything to the simulation side'
    routed = set(match.group(1).split('|'))

    parser = build_parser()
    choices = parser._subparsers._group_actions[0].choices
    declared = {name for name, sub in choices.items()
                if sub.get_default('side') == 'sim'}
    assert routed == declared, (
        f'run_realsim.sh routes {sorted(routed)} to the simulation side but the '
        f'parser declares {sorted(declared)}')
    # And nothing is left without a side at all.
    assert all(sub.get_default('side') in ('sim', 'real') for sub in choices.values())
