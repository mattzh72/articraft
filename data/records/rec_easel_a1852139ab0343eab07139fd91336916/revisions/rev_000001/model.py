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


ALUMINIUM = Material("brushed_aluminium", rgba=(0.78, 0.80, 0.78, 1.0))
BLACK = Material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
RUBBER = Material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
POSTER_WHITE = Material("white_poster", rgba=(0.96, 0.95, 0.89, 1.0))
POSTER_BLUE = Material("blue_poster_graphic", rgba=(0.05, 0.20, 0.65, 1.0))
POSTER_ORANGE = Material("orange_poster_graphic", rgba=(0.95, 0.38, 0.09, 1.0))


def _box_between(part, name: str, a, b, size_xy, material) -> None:
    """Add a rectangular tube whose local +Z runs from point a to point b.

    The easel members lie in XZ or YZ planes, so a single-axis rotation is
    enough and keeps the model legible.
    """

    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > abs(dx):
        rpy = (math.atan2(-dy, dz), 0.0, 0.0)
    else:
        rpy = (0.0, math.atan2(dx, dz), 0.0)
    part.visual(
        Box((size_xy[0], size_xy[1], length)),
        origin=Origin(
            xyz=((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_poster_easel")

    front = model.part("front_frame")

    # Two narrow angled aluminium front legs, tied into a rigid front frame.
    _box_between(front, "front_leg_0", (-0.36, 0.0, 0.04), (-0.23, 0.0, 1.47), (0.035, 0.028), ALUMINIUM)
    _box_between(front, "front_leg_1", (0.36, 0.0, 0.04), (0.23, 0.0, 1.47), (0.035, 0.028), ALUMINIUM)
    front.visual(Box((0.72, 0.030, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.12)), material=ALUMINIUM, name="lower_crossbar")
    front.visual(Box((0.62, 0.026, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.86)), material=ALUMINIUM, name="middle_crossbar")
    front.visual(Box((0.74, 0.040, 0.050)), origin=Origin(xyz=(0.0, -0.002, 1.47)), material=ALUMINIUM, name="crown_block")
    front.visual(Box((0.16, 0.070, 0.035)), origin=Origin(xyz=(-0.36, 0.0, 0.022)), material=BLACK, name="front_foot_0")
    front.visual(Box((0.16, 0.070, 0.035)), origin=Origin(xyz=(0.36, 0.0, 0.022)), material=BLACK, name="front_foot_1")

    # A removable poster is included to make the lip tray and top clamp read
    # mechanically: the tray supports its bottom edge and the clamp compresses
    # its top edge.
    front.visual(Box((0.52, 0.006, 0.82)), origin=Origin(xyz=(0.0, 0.012, 0.91)), material=POSTER_WHITE, name="poster_panel")
    front.visual(Box((0.46, 0.002, 0.055)), origin=Origin(xyz=(0.0, 0.0145, 1.18)), material=POSTER_BLUE, name="poster_header")
    front.visual(Box((0.38, 0.002, 0.040)), origin=Origin(xyz=(0.0, 0.0145, 0.83)), material=POSTER_ORANGE, name="poster_stripe")

    # Dark elongated slider slots mounted in the front legs for the tray pins.
    front.visual(Box((0.014, 0.006, 0.52)), origin=Origin(xyz=(-0.31, 0.015, 0.69)), material=BLACK, name="slot_0")
    front.visual(Box((0.014, 0.006, 0.52)), origin=Origin(xyz=(0.31, 0.015, 0.69)), material=BLACK, name="slot_1")

    # Crown hinge hardware for the rear brace leg.  Outboard plates hold a pin,
    # while the rear leg carries the central sleeve.
    front.visual(Box((0.026, 0.080, 0.070)), origin=Origin(xyz=(-0.085, -0.028, 1.47)), material=ALUMINIUM, name="crown_cheek_0")
    front.visual(Box((0.026, 0.080, 0.070)), origin=Origin(xyz=(0.085, -0.028, 1.47)), material=ALUMINIUM, name="crown_cheek_1")
    front.visual(Cylinder(radius=0.012, length=0.24), origin=Origin(xyz=(0.0, -0.052, 1.47), rpy=(0.0, math.pi / 2.0, 0.0)), material=ALUMINIUM, name="crown_pin")

    # Stanchions at the top edge carry the poster clamp hinge.
    front.visual(Box((0.034, 0.030, 0.165)), origin=Origin(xyz=(-0.33, 0.032, 1.39)), material=ALUMINIUM, name="clamp_stanchion_0")
    front.visual(Box((0.034, 0.030, 0.165)), origin=Origin(xyz=(0.33, 0.032, 1.39)), material=ALUMINIUM, name="clamp_stanchion_1")

    rear = model.part("rear_leg")
    rear.visual(Cylinder(radius=0.024, length=0.11), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=ALUMINIUM, name="rear_hinge_sleeve")
    rear.visual(Box((0.070, 0.060, 0.105)), origin=Origin(xyz=(0.0, -0.022, -0.040)), material=ALUMINIUM, name="rear_yoke_neck")
    _box_between(rear, "rear_leg_bar", (0.0, -0.050, -0.070), (0.0, -0.65, -1.45), (0.034, 0.028), ALUMINIUM)
    rear.visual(Box((0.18, 0.060, 0.030)), origin=Origin(xyz=(0.0, -0.65, -1.45)), material=BLACK, name="rear_foot")

    tray = model.part("lip_tray")
    tray.visual(Box((0.68, 0.120, 0.020)), origin=Origin(xyz=(0.0, 0.095, -0.010)), material=ALUMINIUM, name="tray_shelf")
    tray.visual(Box((0.64, 0.026, 0.060)), origin=Origin(xyz=(0.0, 0.044, 0.030)), material=ALUMINIUM, name="back_lip")
    tray.visual(Box((0.68, 0.018, 0.045)), origin=Origin(xyz=(0.0, 0.154, 0.012)), material=ALUMINIUM, name="front_lip")
    tray.visual(Box((0.060, 0.018, 0.110)), origin=Origin(xyz=(-0.31, 0.026, 0.035)), material=ALUMINIUM, name="slider_plate_0")
    tray.visual(Box((0.060, 0.018, 0.110)), origin=Origin(xyz=(0.31, 0.026, 0.035)), material=ALUMINIUM, name="slider_plate_1")
    tray.visual(Cylinder(radius=0.024, length=0.020), origin=Origin(xyz=(-0.31, 0.050, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=BLACK, name="thumb_knob_0")
    tray.visual(Cylinder(radius=0.024, length=0.020), origin=Origin(xyz=(0.31, 0.050, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=BLACK, name="thumb_knob_1")

    clamp = model.part("top_clamp")
    clamp.visual(Cylinder(radius=0.018, length=0.56), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=ALUMINIUM, name="clamp_hinge_tube")
    clamp.visual(Box((0.035, 0.018, 0.120)), origin=Origin(xyz=(-0.26, 0.0, -0.060)), material=ALUMINIUM, name="clamp_arm_0")
    clamp.visual(Box((0.035, 0.018, 0.120)), origin=Origin(xyz=(0.26, 0.0, -0.060)), material=ALUMINIUM, name="clamp_arm_1")
    clamp.visual(Box((0.62, 0.020, 0.045)), origin=Origin(xyz=(0.0, 0.0, -0.065)), material=ALUMINIUM, name="clamp_bar")
    clamp.visual(Box((0.58, 0.016, 0.040)), origin=Origin(xyz=(0.0, -0.014, -0.065)), material=RUBBER, name="rubber_pad")

    model.articulation(
        "crown_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, -0.052, 1.47)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=front,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.32),
    )
    model.articulation(
        "clamp_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.035, 1.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_leg")
    tray = object_model.get_part("lip_tray")
    clamp = object_model.get_part("top_clamp")
    crown_hinge = object_model.get_articulation("crown_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    clamp_hinge = object_model.get_articulation("clamp_hinge")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="crown_pin",
        elem_b="rear_hinge_sleeve",
        reason="The rear brace sleeve is intentionally captured around the crown hinge pin.",
    )
    ctx.allow_overlap(
        front,
        rear,
        elem_a="crown_pin",
        elem_b="rear_yoke_neck",
        reason="The hinge yoke neck locally wraps the captured crown pin and sleeve.",
    )
    ctx.allow_overlap(
        front,
        clamp,
        elem_a="poster_panel",
        elem_b="rubber_pad",
        reason="The soft rubber pad intentionally compresses the poster top edge in the clamped pose.",
    )

    ctx.expect_overlap(front, rear, axes="x", elem_a="crown_pin", elem_b="rear_hinge_sleeve", min_overlap=0.09, name="rear sleeve spans crown pin")
    ctx.expect_overlap(front, rear, axes="x", elem_a="crown_pin", elem_b="rear_yoke_neck", min_overlap=0.05, name="rear yoke wraps crown pin")
    ctx.expect_gap(clamp, front, axis="y", positive_elem="rubber_pad", negative_elem="poster_panel", max_penetration=0.006, max_gap=0.003, name="clamp pad lightly compresses poster")
    ctx.expect_overlap(clamp, front, axes="xz", elem_a="rubber_pad", elem_b="poster_panel", min_overlap=0.025, name="clamp pad covers poster top edge")
    ctx.expect_within(tray, front, axes="x", inner_elem="slider_plate_0", outer_elem="slot_0", margin=0.030, name="tray plate follows first slot")
    ctx.expect_within(tray, front, axes="x", inner_elem="slider_plate_1", outer_elem="slot_1", margin=0.030, name="tray plate follows second slot")

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.32}):
        raised_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(tray, front, axes="z", elem_a="slider_plate_0", elem_b="slot_0", min_overlap=0.08, name="raised tray remains in first slot")
        ctx.expect_overlap(tray, front, axes="z", elem_a="slider_plate_1", elem_b="slot_1", min_overlap=0.08, name="raised tray remains in second slot")
    ctx.check(
        "tray slide raises lip tray",
        rest_tray_pos is not None and raised_tray_pos is not None and raised_tray_pos[2] > rest_tray_pos[2] + 0.30,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    closed_bar = ctx.part_element_world_aabb(clamp, elem="clamp_bar")
    with ctx.pose({clamp_hinge: 1.0}):
        open_bar = ctx.part_element_world_aabb(clamp, elem="clamp_bar")
    ctx.check(
        "top clamp swings up and forward",
        closed_bar is not None
        and open_bar is not None
        and ((open_bar[0][2] + open_bar[1][2]) * 0.5) > ((closed_bar[0][2] + closed_bar[1][2]) * 0.5) + 0.025
        and ((open_bar[0][1] + open_bar[1][1]) * 0.5) > ((closed_bar[0][1] + closed_bar[1][1]) * 0.5) + 0.02,
        details=f"closed={closed_bar}, open={open_bar}",
    )

    open_foot = ctx.part_element_world_aabb(rear, elem="rear_foot")
    with ctx.pose({crown_hinge: 1.20}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot")
    ctx.check(
        "rear brace folds toward front frame",
        open_foot is not None
        and folded_foot is not None
        and ((folded_foot[0][1] + folded_foot[1][1]) * 0.5) > ((open_foot[0][1] + open_foot[1][1]) * 0.5) + 0.25
        and folded_foot[0][2] > open_foot[0][2] + 0.10,
        details=f"open={open_foot}, folded={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
