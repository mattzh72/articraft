from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    """Return a CadQuery box in model coordinates."""
    return cq.Workplane("XY").box(*size).translate(center)


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> cq.Workplane:
    body = _box(*boxes[0])
    for size, center in boxes[1:]:
        body = body.union(_box(size, center))
    return body


def _side_plate_geometry() -> cq.Workplane:
    """Grounded side mounting plate with through screw holes."""
    length = 0.66
    thickness = 0.006
    height = 0.125
    # The plate occupies negative Y; its outside face sits just behind the channel web.
    plate = _box((length, thickness, height), (length / 2.0, -0.0060, 0.0))

    # Drill four through holes along the wall plate.  The cutters run through Y.
    for x, z, radius in (
        (0.075, 0.037, 0.009),
        (0.210, -0.037, 0.009),
        (0.430, 0.037, 0.009),
        (0.585, -0.037, 0.009),
    ):
        cutter = (
            cq.Workplane("XY")
            .cylinder(thickness * 4.0, radius)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((x, -0.0060, z))
        )
        plate = plate.cut(cutter)
    return plate


def _outer_channel_geometry() -> cq.Workplane:
    """Fixed C-channel that is bolted to the side plate and opens outward."""
    length = 0.60
    # Slight overlaps between members make the stamped channel one fused mesh.
    return _union_boxes(
        [
            ((length, 0.0032, 0.060), (length / 2.0, -0.0016, 0.0)),  # wall web
            ((length, 0.0340, 0.0042), (length / 2.0, 0.0140, 0.0280)),  # upper flange
            ((length, 0.0340, 0.0042), (length / 2.0, 0.0140, -0.0280)),  # lower flange
            ((length, 0.0045, 0.0145), (length / 2.0, 0.0282, 0.0190)),  # upper return lip
            ((length, 0.0045, 0.0145), (length / 2.0, 0.0282, -0.0190)),  # lower return lip
        ]
    )


def _middle_runner_geometry() -> cq.Workplane:
    """Intermediate runner captured by the outer channel and carrying the inner member."""
    length = 0.56
    return _union_boxes(
        [
            ((length, 0.0055, 0.0380), (length / 2.0, 0.0110, 0.0)),  # central web
            ((length, 0.0185, 0.0042), (length / 2.0, 0.0155, 0.0205)),  # upper race
            ((length, 0.0185, 0.0042), (length / 2.0, 0.0155, -0.0205)),  # lower race
            ((length, 0.0230, 0.0035), (length / 2.0, 0.0220, 0.0)),  # bridge toward inner track
            ((length, 0.0035, 0.0220), (length / 2.0, 0.0280, 0.0)),  # outboard guide wall
            ((length, 0.0095, 0.0032), (length / 2.0, 0.0310, 0.01015)),  # upper inner guide
            ((length, 0.0095, 0.0032), (length / 2.0, 0.0310, -0.01015)),  # lower inner guide
        ]
    )


def _inner_runner_geometry() -> cq.Workplane:
    """Outboard drawer-side runner with a projecting slide tongue."""
    length = 0.52
    return _union_boxes(
        [
            ((length, 0.0060, 0.0460), (length / 2.0, 0.0510, 0.0)),  # drawer mounting flange
            ((length, 0.0115, 0.0060), (length / 2.0, 0.04275, 0.0)),  # sliding tongue
            ((length, 0.0040, 0.0100), (length / 2.0, 0.0470, 0.0180)),  # upper stiffening bead
            ((length, 0.0040, 0.0100), (length / 2.0, 0.0470, -0.0180)),  # lower stiffening bead
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_three_stage_extension_unit")

    zinc = model.material("brushed_zinc", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_zinc = model.material("dark_galvanized_steel", rgba=(0.34, 0.36, 0.36, 1.0))
    bright_steel = model.material("polished_runner_steel", rgba=(0.86, 0.87, 0.84, 1.0))
    warm_steel = model.material("slightly_worn_steel", rgba=(0.63, 0.64, 0.60, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_geometry(), "side_plate_shell", tolerance=0.0008),
        origin=Origin(),
        material=dark_zinc,
        name="plate_shell",
    )

    outer_channel = model.part("outer_channel")
    outer_length = 0.60
    for name, size, center in [
        ("channel_web", (outer_length, 0.0030, 0.0600), (outer_length / 2.0, -0.0015, 0.0)),
        ("upper_flange", (outer_length, 0.0330, 0.0040), (outer_length / 2.0, 0.0135, 0.0280)),
        ("lower_flange", (outer_length, 0.0330, 0.0040), (outer_length / 2.0, 0.0135, -0.0280)),
        ("upper_lip", (outer_length, 0.0040, 0.0140), (outer_length / 2.0, 0.0280, 0.0190)),
        ("lower_lip", (outer_length, 0.0040, 0.0140), (outer_length / 2.0, 0.0280, -0.0190)),
    ]:
        outer_channel.visual(Box(size), origin=Origin(xyz=center), material=zinc, name=name)

    middle_runner = model.part("middle_runner")
    middle_length = 0.56
    for name, size, center in [
        ("center_web", (middle_length, 0.0055, 0.0380), (middle_length / 2.0, 0.0110, 0.0)),
        ("upper_race", (middle_length, 0.01975, 0.0042), (middle_length / 2.0, 0.016125, 0.0205)),
        ("lower_race", (middle_length, 0.01975, 0.0042), (middle_length / 2.0, 0.016125, -0.0205)),
        ("inner_bridge", (middle_length, 0.0230, 0.0035), (middle_length / 2.0, 0.0220, 0.0)),
        ("guide_wall", (middle_length, 0.0035, 0.0220), (middle_length / 2.0, 0.0280, 0.0)),
        ("upper_guide", (middle_length, 0.0095, 0.0032), (middle_length / 2.0, 0.0310, 0.0104)),
        ("lower_guide", (middle_length, 0.0095, 0.0032), (middle_length / 2.0, 0.0310, -0.0104)),
    ]:
        middle_runner.visual(Box(size), origin=Origin(xyz=center), material=warm_steel, name=name)

    inner_runner = model.part("inner_runner")
    inner_length = 0.52
    for name, size, center in [
        ("drawer_flange", (inner_length, 0.0060, 0.0460), (inner_length / 2.0, 0.0470, 0.0)),
        ("slide_tongue", (inner_length, 0.0110, 0.0176), (inner_length / 2.0, 0.0390, 0.0)),
        ("upper_bead", (inner_length, 0.0040, 0.0100), (inner_length / 2.0, 0.0440, 0.0180)),
        ("lower_bead", (inner_length, 0.0040, 0.0100), (inner_length / 2.0, 0.0440, -0.0180)),
    ]:
        inner_runner.visual(Box(size), origin=Origin(xyz=center), material=bright_steel, name=name)

    model.articulation(
        "plate_to_outer",
        ArticulationType.FIXED,
        parent=side_plate,
        child=outer_channel,
        origin=Origin(),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=middle_runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.31),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.31),
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

    side_plate = object_model.get_part("side_plate")
    outer_channel = object_model.get_part("outer_channel")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "serial prismatic stages",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.articulation_type == ArticulationType.PRISMATIC
        and outer_slide.parent == "outer_channel"
        and outer_slide.child == "middle_runner"
        and inner_slide.parent == "middle_runner"
        and inner_slide.child == "inner_runner",
        details="The middle and inner runners must be joined by serial prismatic slides.",
    )

    ctx.expect_gap(
        outer_channel,
        side_plate,
        axis="y",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="channel_web",
        negative_elem="plate_shell",
        name="outer channel seats on grounded side plate",
    )
    ctx.expect_overlap(
        outer_channel,
        side_plate,
        axes="xz",
        min_overlap=0.055,
        elem_a="channel_web",
        elem_b="plate_shell",
        name="outer channel has broad plate support",
    )

    ctx.expect_within(
        middle_runner,
        outer_channel,
        axes="z",
        margin=0.002,
        inner_elem="center_web",
        outer_elem="channel_web",
        name="middle runner sits inside outer channel height",
    )
    ctx.expect_overlap(
        middle_runner,
        outer_channel,
        axes="x",
        min_overlap=0.50,
        elem_a="center_web",
        elem_b="channel_web",
        name="middle runner retained when collapsed",
    )
    ctx.expect_overlap(
        inner_runner,
        middle_runner,
        axes="x",
        min_overlap=0.48,
        elem_a="slide_tongue",
        elem_b="inner_bridge",
        name="inner runner retained when collapsed",
    )

    rest_inner = ctx.part_world_position(inner_runner)
    with ctx.pose({outer_slide: 0.31, inner_slide: 0.31}):
        ctx.expect_overlap(
            middle_runner,
            outer_channel,
            axes="x",
            min_overlap=0.24,
            elem_a="center_web",
            elem_b="channel_web",
            name="middle runner retains insertion at full travel",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            min_overlap=0.18,
            elem_a="slide_tongue",
            elem_b="inner_bridge",
            name="inner runner retains insertion at full travel",
        )
        extended_inner = ctx.part_world_position(inner_runner)

    ctx.check(
        "positive travel extends the drawer-side runner",
        rest_inner is not None
        and extended_inner is not None
        and extended_inner[0] > rest_inner[0] + 0.60,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
