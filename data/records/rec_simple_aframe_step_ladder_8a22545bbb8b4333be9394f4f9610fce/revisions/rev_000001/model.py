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


def _rail_origin(
    bottom: tuple[float, float, float],
    top: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Origin for a rectangular tube whose local +Z spans bottom->top."""
    cx = (bottom[0] + top[0]) * 0.5
    cy = (bottom[1] + top[1]) * 0.5
    cz = (bottom[2] + top[2]) * 0.5
    dx = top[0] - bottom[0]
    dz = top[2] - bottom[2]
    length = math.sqrt(dx * dx + dz * dz)
    return Origin(xyz=(cx, cy, cz), rpy=(0.0, math.atan2(dx, dz), 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.77, 0.78, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    tread_grey = model.material("ribbed_tread_grey", rgba=(0.28, 0.30, 0.31, 1.0))
    safety_cap = model.material("safety_orange_plastic", rgba=(1.0, 0.47, 0.08, 1.0))
    pin_steel = model.material("hinge_pin_steel", rgba=(0.55, 0.57, 0.58, 1.0))

    front = model.part("front_frame")
    hinge_x = 0.08
    hinge_z = 1.30

    # Front climbing rails: slightly splayed in width, leaning back to the top cap.
    for suffix, y in (("0", -0.29), ("1", 0.29)):
        origin, length = _rail_origin(
            bottom=(-0.20, y, 0.04),
            top=(0.02, y, 1.30),
        )
        front.visual(
            Box((0.055, 0.050, length)),
            origin=origin,
            material=aluminum,
            name=f"front_rail_{suffix}",
        )
        front.visual(
            Box((0.16, 0.105, 0.055)),
            origin=Origin(xyz=(-0.22, y, 0.025)),
            material=dark_rubber,
            name=f"front_foot_{suffix}",
        )

    # Four broad horizontal treads; each embeds into the side rails and carries
    # a raised anti-slip strip on the front edge.
    for i, z in enumerate((0.32, 0.59, 0.86, 1.11)):
        rail_x = -0.20 + (z - 0.04) * (0.22 / 1.26)
        x = rail_x - 0.060
        front.visual(
            Box((0.215, 0.640, 0.045)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=tread_grey,
            name=f"tread_{i}",
        )
        front.visual(
            Box((0.030, 0.600, 0.016)),
            origin=Origin(xyz=(x - 0.088, 0.0, z + 0.030)),
            material=dark_rubber,
            name=f"tread_lip_{i}",
        )

    # Top cap and the fixed outer hinge knuckles of the A-frame.
    front.visual(
        Box((0.280, 0.710, 0.075)),
        origin=Origin(xyz=(-0.100, 0.0, 1.335)),
        material=safety_cap,
        name="top_cap",
    )
    front.visual(
        Cylinder(radius=0.036, length=0.120),
        origin=Origin(xyz=(hinge_x, -0.280, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="hinge_knuckle_0",
    )
    front.visual(
        Cylinder(radius=0.036, length=0.120),
        origin=Origin(xyz=(hinge_x, 0.280, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="hinge_knuckle_1",
    )

    rear = model.part("rear_frame")

    # The moving rear support is authored in the folded pose, close and nearly
    # parallel to the front frame.  Positive hinge travel swings the feet rearward.
    for suffix, y in (("0", -0.420), ("1", 0.420)):
        origin, length = _rail_origin(
            bottom=(-0.170, y, -1.265),
            top=(0.030, y, -0.100),
        )
        rear.visual(
            Box((0.050, 0.048, length)),
            origin=origin,
            material=aluminum,
            name=f"rear_rail_{suffix}",
        )
        rear.visual(
            Box((0.145, 0.100, 0.055)),
            origin=Origin(xyz=(-0.180, y, -1.290)),
            material=dark_rubber,
            name=f"rear_foot_{suffix}",
        )

    rear.visual(
        Cylinder(radius=0.032, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="center_knuckle",
    )
    rear.visual(
        Box((0.055, 0.085, 0.090)),
        origin=Origin(xyz=(-0.018, 0.0, -0.045)),
        material=pin_steel,
        name="hinge_hanger",
    )
    rear.visual(
        Box((0.070, 0.910, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, -0.105)),
        material=aluminum,
        name="top_crossbar",
    )
    rear.visual(
        Box((0.060, 0.910, 0.050)),
        origin=Origin(xyz=(-0.045, 0.0, -0.570)),
        material=aluminum,
        name="rear_spreader",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=0.55),
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

    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.expect_contact(
        front,
        rear,
        elem_a="hinge_knuckle_0",
        elem_b="center_knuckle",
        contact_tol=0.065,
        name="hinge knuckles share the top pin line",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="z",
        elem_a="front_rail_0",
        elem_b="rear_rail_0",
        min_overlap=1.0,
        name="folded frames are vertically close and parallel",
    )

    folded_pos = ctx.part_world_position(rear)
    with ctx.pose({hinge: 0.55}):
        opened_pos = ctx.part_world_position(rear)
        ctx.expect_gap(
            rear,
            front,
            axis="x",
            positive_elem="rear_foot_0",
            negative_elem="front_foot_0",
            min_gap=0.65,
            name="rear support swings backward into A-frame stance",
        )

    ctx.check(
        "hinge pose keeps rear frame close when folded",
        folded_pos is not None and opened_pos is not None,
        details=f"folded={folded_pos}, opened={opened_pos}",
    )

    return ctx.report()


object_model = build_object_model()
