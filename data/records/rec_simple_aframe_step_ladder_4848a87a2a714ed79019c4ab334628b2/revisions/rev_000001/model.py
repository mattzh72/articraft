from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _yz_beam_origin(
    x: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
) -> tuple[Origin, float]:
    """Origin for a member whose long local Z axis runs between two YZ points."""
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    roll = -math.atan2(dy, dz)
    return Origin(xyz=(x, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(roll, 0.0, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.75, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    blue_plastic = model.material("blue_plastic", rgba=(0.05, 0.16, 0.46, 1.0))
    warning_label = model.material("yellow_warning_label", rgba=(0.95, 0.74, 0.10, 1.0))

    front = model.part("front_frame")

    # Two angled aluminum side rails define the climbing face.
    for x, rail_name in ((-0.285, "side_rail_0"), (0.285, "side_rail_1")):
        origin, length = _yz_beam_origin(x, -0.43, 0.055, -0.055, 1.385)
        front.visual(
            Box((0.060, 0.046, length)),
            origin=origin,
            material=aluminum,
            name=rail_name,
        )

    # Wide horizontal treads bridge both front rails.
    tread_specs = (
        (0.285, "tread_0"),
        (0.560, "tread_1"),
        (0.835, "tread_2"),
        (1.110, "tread_3"),
    )
    for z, tread_name in tread_specs:
        y_on_rail = -0.43 + (z - 0.055) / (1.385 - 0.055) * 0.375
        front.visual(
            Box((0.650, 0.205, 0.042)),
            origin=Origin(xyz=(0.0, y_on_rail - 0.010, z)),
            material=aluminum,
            name=tread_name,
        )
        # Low black ribs make the steps read as non-slip ladder treads.
        front.visual(
            Box((0.560, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, y_on_rail - 0.075, z + 0.023)),
            material=dark_rubber,
            name=f"{tread_name}_grip_front",
        )
        front.visual(
            Box((0.560, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, y_on_rail + 0.050, z + 0.023)),
            material=dark_rubber,
            name=f"{tread_name}_grip_rear",
        )

    # A fixed lower cross brace/rest bar stiffens the climbing frame without
    # adding a second movable loop.
    front.visual(
        Box((0.700, 0.052, 0.055)),
        origin=Origin(xyz=(0.0, -0.420, 0.105)),
        material=aluminum,
        name="front_lower_brace",
    )

    # Rubber foot pads are mounted directly onto the front rail ends.
    for x, foot_name in ((-0.285, "front_foot_0"), (0.285, "front_foot_1")):
        front.visual(
            Box((0.135, 0.155, 0.070)),
            origin=Origin(xyz=(x, -0.445, 0.035)),
            material=dark_rubber,
            name=foot_name,
        )

    # Molded top cap with a shallow tool tray/rest and raised lips.
    front.visual(
        Box((0.720, 0.205, 0.085)),
        origin=Origin(xyz=(0.0, -0.090, 1.410)),
        material=blue_plastic,
        name="top_cap",
    )
    front.visual(
        Box((0.585, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, -0.180, 1.476)),
        material=blue_plastic,
        name="tray_front_lip",
    )
    front.visual(
        Box((0.585, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, -0.005, 1.476)),
        material=blue_plastic,
        name="tray_rear_lip",
    )
    for x, rim_name in ((-0.325, "tray_side_lip_0"), (0.325, "tray_side_lip_1")):
        front.visual(
            Box((0.050, 0.195, 0.050)),
            origin=Origin(xyz=(x, -0.092, 1.476)),
            material=blue_plastic,
            name=rim_name,
        )
    front.visual(
        Box((0.250, 0.045, 0.006)),
        origin=Origin(xyz=(0.0, -0.095, 1.454)),
        material=warning_label,
        name="tray_warning_label",
    )

    # Front hinge barrel and brackets mounted at the rear of the top cap.  The
    # rear frame uses separate outer knuckles so the hinge reads as one pin line
    # without broad part overlap.
    front.visual(
        Cylinder(radius=0.026, length=0.375),
        origin=Origin(xyz=(0.0, 0.080, 1.385), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    front.visual(
        Cylinder(radius=0.016, length=0.760),
        origin=Origin(xyz=(0.0, 0.080, 1.385), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hinge_pin",
    )
    for x, bracket_name in ((-0.105, "hinge_bracket_0"), (0.105, "hinge_bracket_1")):
        front.visual(
            Box((0.070, 0.095, 0.070)),
            origin=Origin(xyz=(x, 0.030, 1.385)),
            material=blue_plastic,
            name=bracket_name,
        )

    rear = model.part("rear_frame")

    # Child frame origin is the hinge line.  At q=0 the rear support legs are
    # open behind the climbing frame and the positive joint motion folds them
    # toward the front frame.
    for x, leg_name in ((-0.295, "rear_leg_0"), (0.295, "rear_leg_1")):
        origin, length = _yz_beam_origin(x, 0.660, -1.320, 0.055, -0.085)
        rear.visual(
            Box((0.055, 0.045, length)),
            origin=origin,
            material=aluminum,
            name=leg_name,
        )
        rear.visual(
            Box((0.120, 0.150, 0.070)),
            origin=Origin(xyz=(x, 0.675, -1.340)),
            material=dark_rubber,
            name=f"rear_foot_{0 if x < 0 else 1}",
        )
        rear.visual(
            Box((0.060, 0.035, 0.100)),
            origin=Origin(xyz=(x, 0.040, -0.050)),
            material=aluminum,
            name=f"knuckle_web_{0 if x < 0 else 1}",
        )

    # Coaxial outer hinge knuckles are part of the rear frame and flank the
    # fixed center barrel on the front frame.
    for x, knuckle_name in ((-0.280, "hinge_knuckle_0"), (0.280, "hinge_knuckle_1")):
        rear.visual(
            Cylinder(radius=0.034, length=0.120),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=knuckle_name,
        )

    rear.visual(
        Box((0.660, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.545, -1.105)),
        material=aluminum,
        name="lower_brace",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.080, 1.385)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.5, lower=0.0, upper=0.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    for knuckle_name in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.allow_overlap(
            front,
            rear,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The visible hinge pin is intentionally captured through simplified solid rear hinge knuckles.",
        )
        ctx.expect_within(
            front,
            rear,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.001,
            name=f"hinge pin is centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            front,
            rear,
            axes="x",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.10,
            name=f"hinge pin passes through {knuckle_name}",
        )

    ctx.check(
        "single top hinge",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        rear,
        front,
        axis="y",
        positive_elem="lower_brace",
        negative_elem="front_lower_brace",
        min_gap=0.32,
        name="rear support opens behind front frame",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="x",
        elem_a="top_cap",
        elem_b="lower_brace",
        min_overlap=0.55,
        name="front and rear frames share ladder width",
    )

    rest_aabb = ctx.part_element_world_aabb(rear, elem="lower_brace")
    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    with ctx.pose({hinge: 0.72}):
        folded_aabb = ctx.part_element_world_aabb(rear, elem="lower_brace")
        folded_y = None if folded_aabb is None else (folded_aabb[0][1] + folded_aabb[1][1]) * 0.5
    ctx.check(
        "rear frame folds toward climbing frame",
        rest_y is not None and folded_y is not None and folded_y < rest_y - 0.45,
        details=f"rest_y={rest_y}, folded_y={folded_y}",
    )

    return ctx.report()


object_model = build_object_model()
