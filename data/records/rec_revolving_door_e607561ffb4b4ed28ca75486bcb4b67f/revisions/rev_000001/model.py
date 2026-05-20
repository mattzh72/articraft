from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="all_glass_square_revolving_door")

    glass = Material("clear_slightly_green_glass", rgba=(0.68, 0.92, 1.0, 0.34))
    thick_edge = Material("polished_green_glass_edge", rgba=(0.38, 0.85, 0.78, 0.55))
    bearing = Material("satin_stainless_bearing", rgba=(0.74, 0.76, 0.76, 1.0))

    drum = model.part("drum")

    # A square glass drum with four doorway gaps at the side centers.  The
    # transparent floor and roof plates tie the short corner wall segments into
    # one physical stationary assembly.
    half_span = 1.10
    wall_t = 0.030
    wall_h = 2.36
    base_t = 0.040
    roof_t = 0.040
    wall_z = base_t + wall_h / 2.0 - 0.010
    wall_len = 0.62

    drum.visual(
        Box((2.28, 2.28, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=glass,
        name="square_glass_floor",
    )
    drum.visual(
        Box((2.28, 2.28, roof_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t + wall_h - 0.020 + roof_t / 2.0)),
        material=glass,
        name="square_glass_roof",
    )

    side_offsets = (-0.73, 0.73)
    for x in side_offsets:
        drum.visual(
            Box((wall_len, wall_t, wall_h)),
            origin=Origin(xyz=(x, half_span, wall_z)),
            material=glass,
            name=f"wall_y_pos_{0 if x < 0 else 1}",
        )
        drum.visual(
            Box((wall_len, wall_t, wall_h)),
            origin=Origin(xyz=(x, -half_span, wall_z)),
            material=glass,
            name=f"wall_y_neg_{0 if x < 0 else 1}",
        )
    for y in side_offsets:
        drum.visual(
            Box((wall_t, wall_len, wall_h)),
            origin=Origin(xyz=(half_span, y, wall_z)),
            material=glass,
            name=f"wall_x_pos_{0 if y < 0 else 1}",
        )
        drum.visual(
            Box((wall_t, wall_len, wall_h)),
            origin=Origin(xyz=(-half_span, y, wall_z)),
            material=glass,
            name=f"wall_x_neg_{0 if y < 0 else 1}",
        )

    # Four small stationary bearing pads around each pivot leave a clear center
    # for the rotating pin while still showing the supported hub axis.
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.118, 0.0, 0.038, 0.120),
            (-0.118, 0.0, 0.038, 0.120),
            (0.0, 0.118, 0.120, 0.038),
            (0.0, -0.118, 0.120, 0.038),
        )
    ):
        drum.visual(
            Box((sx, sy, 0.018)),
            origin=Origin(xyz=(x, y, base_t + 0.009)),
            material=bearing,
            name=f"floor_bearing_pad_{idx}",
        )
        drum.visual(
            Box((sx, sy, 0.018)),
            origin=Origin(xyz=(x, y, base_t + wall_h - 0.029)),
            material=bearing,
            name=f"roof_bearing_pad_{idx}",
        )

    rotor = model.part("rotor")
    hub_w = 0.135
    hub_h = wall_h - 0.040
    hub_center_z = base_t + hub_h / 2.0
    wing_h = 2.18
    wing_t = 0.026
    wing_len = 1.30
    wing_center = wing_len / 2.0
    wing_z = base_t + 0.105 + wing_h / 2.0

    rotor.visual(
        Box((hub_w, hub_w, hub_h)),
        origin=Origin(xyz=(0.0, 0.0, hub_center_z)),
        material=glass,
        name="square_hub_post",
    )
    rotor.visual(
        Cylinder(radius=0.073, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, base_t + 0.025)),
        material=bearing,
        name="lower_pivot_pin",
    )
    rotor.visual(
        Cylinder(radius=0.073, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, base_t + wall_h - 0.045)),
        material=bearing,
        name="upper_pivot_pin",
    )

    # The four frameless rectangular wings are radial glass sheets aimed at the
    # square drum corners in the zero pose.  Each slightly enters the square hub
    # post so the hub visibly carries the sheets.
    for idx, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        rotor.visual(
            Box((wing_len, wing_t, wing_h)),
            origin=Origin(
                xyz=(math.cos(angle) * wing_center, math.sin(angle) * wing_center, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=glass,
            name=f"glass_wing_{idx}",
        )
        rotor.visual(
            Box((0.026, 0.036, wing_h)),
            origin=Origin(
                xyz=(math.cos(angle) * wing_len, math.sin(angle) * wing_len, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=thick_edge,
            name=f"wing_edge_{idx}",
        )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
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

    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "rotor uses a continuous vertical hub joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        margin=0.0,
        name="four wings fit inside the square glass drum footprint",
    )
    ctx.expect_gap(
        rotor,
        drum,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lower_pivot_pin",
        negative_elem="square_glass_floor",
        name="lower pivot pin is seated on the floor bearing",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.0,
            name="rotated wing assembly remains within square drum",
        )
    ctx.check(
        "spin rotates in place about the central post",
        rest_pos is not None and turned_pos is not None and abs(rest_pos[0] - turned_pos[0]) < 1e-6 and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
