from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.320
CASE_D = 0.140
DECK_Z = 0.016


def _media_wheel_mesh() -> object:
    """A bored, lightly grooved encoder wheel, authored around local +Z."""
    outer_r = 0.014
    bore_r = 0.0048
    width = 0.020
    wheel = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(bore_r)
        .extrude(width)
        .translate((0.0, 0.0, -width / 2.0))
    )

    # Shallow axial grooves around the outside read as the rubber grip texture
    # of a media/volume roller without compromising the central bore.
    for i in range(28):
        angle = 360.0 * i / 28.0
        cutter = (
            cq.Workplane("XY")
            .box(0.0032, 0.0009, width + 0.002)
            .translate((outer_r + 0.0007, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        wheel = wheel.cut(cutter)
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trackpoint_keyboard")

    model.material("case_graphite", rgba=(0.055, 0.058, 0.063, 1.0))
    model.material("deck_black", rgba=(0.010, 0.011, 0.013, 1.0))
    model.material("key_black", rgba=(0.030, 0.032, 0.036, 1.0))
    model.material("key_top", rgba=(0.095, 0.100, 0.110, 1.0))
    model.material("legend_gray", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("track_red", rgba=(0.82, 0.035, 0.025, 1.0))
    model.material("rubber_black", rgba=(0.002, 0.002, 0.003, 1.0))
    model.material("wheel_rubber", rgba=(0.018, 0.018, 0.020, 1.0))
    model.material("shaft_metal", rgba=(0.55, 0.56, 0.58, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_W, CASE_D, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="case_graphite",
        name="lower_tray",
    )
    case.visual(
        Box((0.292, 0.112, 0.0045)),
        origin=Origin(xyz=(0.0, -0.002, 0.01375)),
        material="deck_black",
        name="deck",
    )
    # Raised tray rim around the compact key field.
    case.visual(
        Box((CASE_W, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + 0.005, 0.011)),
        material="case_graphite",
        name="front_lip",
    )
    case.visual(
        Box((CASE_W, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - 0.005, 0.011)),
        material="case_graphite",
        name="rear_lip",
    )
    case.visual(
        Box((0.010, CASE_D, 0.022)),
        origin=Origin(xyz=(-CASE_W / 2.0 + 0.005, 0.0, 0.011)),
        material="case_graphite",
        name="side_lip_0",
    )
    case.visual(
        Box((0.010, CASE_D, 0.022)),
        origin=Origin(xyz=(CASE_W / 2.0 - 0.005, 0.0, 0.011)),
        material="case_graphite",
        name="side_lip_1",
    )

    # Four small deck tabs form the visible opening for the compliant trackpoint
    # mount without using a solid collar that would collide with the nub.
    for idx, (x, y, sx, sy) in enumerate(
        (
            (-0.0078, -0.006, 0.0032, 0.0115),
            (0.0078, -0.006, 0.0032, 0.0115),
            (0.0, -0.0138, 0.0115, 0.0032),
            (0.0, 0.0018, 0.0115, 0.0032),
        )
    ):
        case.visual(
            Box((sx, sy, 0.0022)),
            origin=Origin(xyz=(x, y, DECK_Z + 0.0011)),
            material="rubber_black",
            name=f"track_collar_{idx}",
        )

    # Media wheel pod: a saddle and two ears carry a metal shaft at the top edge.
    wheel_xyz = (0.118, 0.071, 0.043)
    case.visual(
        Box((0.046, 0.020, 0.008)),
        origin=Origin(xyz=(wheel_xyz[0], wheel_xyz[1], 0.023)),
        material="case_graphite",
        name="wheel_pod",
    )
    case.visual(
        Box((0.006, 0.022, 0.030)),
        origin=Origin(xyz=(wheel_xyz[0] - 0.014, wheel_xyz[1], 0.036)),
        material="case_graphite",
        name="wheel_ear_0",
    )
    case.visual(
        Box((0.006, 0.022, 0.030)),
        origin=Origin(xyz=(wheel_xyz[0] + 0.014, wheel_xyz[1], 0.036)),
        material="case_graphite",
        name="wheel_ear_1",
    )
    case.visual(
        Cylinder(radius=0.0032, length=0.038),
        origin=Origin(xyz=wheel_xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material="shaft_metal",
        name="wheel_shaft",
    )
    case.visual(
        Cylinder(radius=0.006, length=0.0030),
        origin=Origin(
            xyz=(wheel_xyz[0] - 0.0115, wheel_xyz[1], wheel_xyz[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="shaft_metal",
        name="shaft_clip_0",
    )
    case.visual(
        Cylinder(radius=0.006, length=0.0030),
        origin=Origin(
            xyz=(wheel_xyz[0] + 0.0115, wheel_xyz[1], wheel_xyz[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="shaft_metal",
        name="shaft_clip_1",
    )

    # Staggered compact key field.  Each key is its own short-travel plunger.
    key_xs = [
        -0.128,
        -0.106,
        -0.084,
        -0.062,
        -0.040,
        -0.016,
        0.016,
        0.040,
        0.062,
        0.084,
        0.106,
        0.128,
    ]
    row_ys = [0.038, 0.016, -0.006, -0.028]
    row_steps = [0.004, 0.003, 0.002, 0.001]
    for row, y in enumerate(row_ys):
        for col, x in enumerate(key_xs):
            name = f"key_{row}_{col}"
            key = model.part(name)
            step = row_steps[row]
            key.visual(
                Box((0.0055, 0.0055, 0.0140)),
                origin=Origin(xyz=(0.0, 0.0, 0.0070)),
                material="rubber_black",
                name="stem",
            )
            key.visual(
                Box((0.0190, 0.0180, 0.0080)),
                origin=Origin(xyz=(0.0, 0.0, 0.0125 + step)),
                material="key_black",
                name="skirt",
            )
            key.visual(
                Box((0.0170, 0.0160, 0.0040)),
                origin=Origin(xyz=(0.0, 0.0, 0.0185 + step)),
                material="key_top",
                name="cap",
            )
            # A few small raised bars break up the key field and imply legends.
            if row in (0, 2) and col in (3, 6, 9):
                key.visual(
                    Box((0.0065, 0.0010, 0.0007)),
                    origin=Origin(xyz=(0.0, -0.0035, 0.02075 + step)),
                    material="legend_gray",
                    name="legend_bar",
                )

            model.articulation(
                f"case_to_{name}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key,
                origin=Origin(xyz=(x, y, DECK_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=8.0, velocity=0.40, lower=0.0, upper=0.004
                ),
            )

    bottom_specs = [
        (-0.128, 0.019, "key_4_0"),
        (-0.106, 0.019, "key_4_1"),
        (-0.084, 0.019, "key_4_2"),
        (-0.060, 0.025, "key_4_3"),
        (0.000, 0.070, "key_4_4"),
        (0.060, 0.025, "key_4_5"),
        (0.084, 0.019, "key_4_6"),
        (0.106, 0.019, "key_4_7"),
        (0.128, 0.019, "key_4_8"),
    ]
    for x, width, name in bottom_specs:
        key = model.part(name)
        key.visual(
            Box((0.0055, 0.0055, 0.0140)),
            origin=Origin(xyz=(0.0, 0.0, 0.0070)),
            material="rubber_black",
            name="stem",
        )
        key.visual(
            Box((width, 0.0180, 0.0080)),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material="key_black",
            name="skirt",
        )
        key.visual(
            Box((max(width - 0.002, 0.014), 0.0160, 0.0040)),
            origin=Origin(xyz=(0.0, 0.0, 0.0185)),
            material="key_top",
            name="cap",
        )
        model.articulation(
            f"case_to_{name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, -0.050, DECK_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=9.0, velocity=0.40, lower=0.0, upper=0.004
            ),
        )

    # Compliant two-axis trackpoint.  The black stem pitches at the deck, and
    # the red textured nub gets a second slight tilt at the stem top.
    track_stem = model.part("track_stem")
    track_stem.visual(
        Cylinder(radius=0.0032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="rubber_black",
        name="flex_stem",
    )
    model.articulation(
        "case_to_track_stem",
        ArticulationType.REVOLUTE,
        parent=case,
        child=track_stem,
        origin=Origin(xyz=(0.0, -0.006, DECK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.0, lower=-0.22, upper=0.22),
    )

    track_nub = model.part("track_nub")
    track_nub.visual(
        Cylinder(radius=0.0046, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material="track_red",
        name="nub_neck",
    )
    track_nub.visual(
        Sphere(radius=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material="track_red",
        name="nub_cap",
    )
    model.articulation(
        "stem_to_track_nub",
        ArticulationType.REVOLUTE,
        parent=track_stem,
        child=track_nub,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.0, lower=-0.18, upper=0.18),
    )

    media_wheel = model.part("media_wheel")
    media_wheel.visual(
        mesh_from_cadquery(_media_wheel_mesh(), "media_wheel"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="wheel_rubber",
        name="wheel_body",
    )
    model.articulation(
        "case_to_media_wheel",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=media_wheel,
        origin=Origin(xyz=wheel_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    case = object_model.get_part("case")
    key = object_model.get_part("key_2_5")
    key_joint = object_model.get_articulation("case_to_key_2_5")
    wheel = object_model.get_part("media_wheel")
    wheel_joint = object_model.get_articulation("case_to_media_wheel")
    stem_joint = object_model.get_articulation("case_to_track_stem")
    nub_joint = object_model.get_articulation("stem_to_track_nub")

    ctx.expect_gap(
        key,
        case,
        axis="z",
        positive_elem="stem",
        negative_elem="deck",
        min_gap=0.0,
        max_gap=0.0005,
        name="home key stem rests above deck",
    )
    rest_key_pos = ctx.part_world_position(key)
    with ctx.pose({key_joint: 0.004}):
        pressed_key_pos = ctx.part_world_position(key)
    ctx.check(
        "home key moves downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.0035,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    ctx.expect_within(
        wheel,
        case,
        axes="x",
        inner_elem="wheel_body",
        outer_elem="wheel_shaft",
        margin=0.0,
        name="media wheel rides on shaft length",
    )
    ctx.expect_gap(
        wheel,
        case,
        axis="x",
        positive_elem="wheel_body",
        negative_elem="shaft_clip_0",
        min_gap=0.0,
        max_gap=0.0030,
        name="wheel retained by first clip",
    )
    ctx.expect_gap(
        case,
        wheel,
        axis="x",
        positive_elem="shaft_clip_1",
        negative_elem="wheel_body",
        min_gap=0.0,
        max_gap=0.0030,
        name="wheel retained by second clip",
    )
    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: 1.25}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "media wheel spins in place",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_wheel_pos, spun_wheel_pos)),
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    nub = object_model.get_part("track_nub")
    rest_nub = ctx.part_world_position(nub)
    with ctx.pose({stem_joint: 0.18, nub_joint: -0.12}):
        tilted_nub = ctx.part_world_position(nub)
    ctx.check(
        "trackpoint pivots slightly",
        rest_nub is not None
        and tilted_nub is not None
        and (
            (tilted_nub[0] - rest_nub[0]) ** 2
            + (tilted_nub[1] - rest_nub[1]) ** 2
        )
        ** 0.5
        > 0.0015,
        details=f"rest={rest_nub}, tilted={tilted_nub}",
    )

    return ctx.report()


object_model = build_object_model()
