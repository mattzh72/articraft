from __future__ import annotations

from math import pi

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


RUBBER = Material("deep_olive_rubber", rgba=(0.06, 0.075, 0.055, 1.0))
BLACK = Material("black_sealed_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
GLASS = Material("green_blue_coated_glass", rgba=(0.10, 0.38, 0.42, 0.62))
MARKING = Material("engraved_white_marking", rgba=(0.86, 0.88, 0.82, 1.0))
RED = Material("compass_red", rgba=(0.82, 0.05, 0.035, 1.0))
BRASS = Material("dull_brass", rgba=(0.55, 0.43, 0.19, 1.0))


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _add_barrel_body(part, *, side: float, name_prefix: str) -> None:
    """Build one rubber-armored porro-prism binocular half.

    The local X axis is the optical axis: +X is the wide objective end and -X is
    the eyepiece end.  Positive Y is the left half; negative Y is the right half.
    """

    y = side * 0.070
    outer_y = side * 0.104
    inner_y = 0.020 if side > 0.0 else -0.029
    main_tube_name = "left_main_tube" if name_prefix == "left" else "right_main_tube"

    part.visual(
        _x_cylinder(0.031, 0.158),
        origin=Origin(xyz=(0.002, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=RUBBER,
        name=main_tube_name,
    )
    part.visual(
        _x_cylinder(0.043, 0.056),
        origin=Origin(xyz=(0.087, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=RUBBER,
        name=f"{name_prefix}_objective_bell",
    )
    part.visual(
        _x_cylinder(0.045, 0.010),
        origin=Origin(xyz=(0.119, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=BLACK,
        name=f"{name_prefix}_front_seal",
    )
    part.visual(
        _x_cylinder(0.032, 0.004),
        origin=Origin(xyz=(0.126, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=GLASS,
        name=f"{name_prefix}_objective_lens",
    )
    part.visual(
        _x_cylinder(0.024, 0.044),
        origin=Origin(xyz=(-0.091, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=BLACK,
        name=f"{name_prefix}_eyecup",
    )
    part.visual(
        _x_cylinder(0.0265, 0.010),
        origin=Origin(xyz=(-0.068, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=BLACK,
        name=f"{name_prefix}_diopter_ring",
    )
    part.visual(
        _x_cylinder(0.017, 0.003),
        origin=Origin(xyz=(-0.1145, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=GLASS,
        name=f"{name_prefix}_eyepiece_glass",
    )

    for i, x in enumerate((-0.042, -0.022, -0.002, 0.018, 0.038)):
        part.visual(
            _x_cylinder(0.033, 0.006),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=BLACK,
            name=f"{name_prefix}_grip_rib_{i}",
        )

    for i, x in enumerate((0.069, 0.100)):
        part.visual(
            _x_cylinder(0.046, 0.005),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=BLACK,
            name=f"{name_prefix}_bell_gasket_{i}",
        )

    # Rectangular porro-prism shoulder and raised rubber side pad.  These overlap
    # the cylindrical tube within the same part to read as one molded armor shell.
    part.visual(
        Box((0.106, 0.052, 0.031)),
        origin=Origin(xyz=(-0.006, y, 0.034)),
        material=RUBBER,
        name=f"{name_prefix}_prism_shoulder",
    )
    part.visual(
        Box((0.088, 0.012, 0.042)),
        origin=Origin(xyz=(0.003, outer_y, 0.002)),
        material=BLACK,
        name=f"{name_prefix}_outer_grip_pad",
    )
    part.visual(
        Box((0.068, 0.009, 0.026)),
        origin=Origin(xyz=(-0.002, outer_y, 0.004)),
        material=RUBBER,
        name=f"{name_prefix}_raised_grip_field",
    )

    # Inner bridge pad where each optical half ties into the central hinge line.
    part.visual(
        Box((0.036, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, inner_y, 0.0)),
        material=RUBBER,
        name=f"{name_prefix}_hinge_bridge",
    )
    part.visual(
        Box((0.035, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, inner_y, 0.0)),
        material=RUBBER,
        name=f"{name_prefix}_pin_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_porro_prism_binocular")

    left_body = model.part("left_body")
    _add_barrel_body(left_body, side=1.0, name_prefix="left")

    # The center hinge pin is part of the left/root half.  The right half carries
    # fork knuckles at the front and rear, leaving a visible non-overlapping
    # central pin segment for the revolute joint.
    left_body.visual(
        _x_cylinder(0.008, 0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=BRASS,
        name="hinge_pin",
    )
    left_body.visual(
        Box((0.050, 0.052, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, 0.036)),
        material=RUBBER,
        name="focus_saddle",
    )
    left_body.visual(
        Box((0.050, 0.022, 0.017)),
        origin=Origin(xyz=(-0.020, 0.035, 0.035)),
        material=RUBBER,
        name="focus_support",
    )

    right_body = model.part("right_body")
    _add_barrel_body(right_body, side=-1.0, name_prefix="right")
    for i, x in enumerate((-0.094, 0.094)):
        right_body.visual(
            _x_cylinder(0.011, 0.032),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=BLACK,
            name=f"right_hinge_knuckle_{i}",
        )
        right_body.visual(
            Box((0.032, 0.036, 0.014)),
            origin=Origin(xyz=(x, -0.026, 0.0)),
            material=RUBBER,
            name=f"right_knuckle_yoke_{i}",
        )
    right_body.visual(
        Box((0.216, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=RUBBER,
        name="right_hinge_rail",
    )
    right_body.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.025, -0.070, 0.049)),
        material=BLACK,
        name="compass_recess",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="knurled_wheel",
    )
    focus_wheel.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=BRASS,
        name="focus_axle",
    )
    focus_wheel.visual(
        Box((0.007, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=MARKING,
        name="index_mark",
    )

    compass_housing = model.part("compass_housing")
    compass_housing.visual(
        Cylinder(radius=0.024, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=BLACK,
        name="compass_pod",
    )
    compass_housing.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=GLASS,
        name="compass_window",
    )
    compass_housing.visual(
        Box((0.032, 0.004, 0.002)),
        origin=Origin(xyz=(0.004, 0.0, 0.0185)),
        material=RED,
        name="compass_pointer",
    )
    compass_housing.visual(
        Box((0.004, 0.026, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0188)),
        material=MARKING,
        name="north_south_mark",
    )

    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=0.8, lower=-0.24, upper=0.24),
    )
    model.articulation(
        "focus_wheel_axis",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_wheel,
        origin=Origin(xyz=(-0.020, 0.0, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "compass_axis",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=compass_housing,
        origin=Origin(xyz=(0.025, -0.070, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=2.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    compass_housing = object_model.get_part("compass_housing")
    central_hinge = object_model.get_articulation("central_hinge")
    focus_axis = object_model.get_articulation("focus_wheel_axis")
    compass_axis = object_model.get_articulation("compass_axis")

    for knuckle_name in ("right_hinge_knuckle_0", "right_hinge_knuckle_1"):
        ctx.allow_overlap(
            left_body,
            right_body,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The brass center hinge pin is intentionally captured inside the right-side hinge knuckle.",
        )
        ctx.expect_overlap(
            left_body,
            right_body,
            axes="x",
            min_overlap=0.020,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            name=f"{knuckle_name} retains hinge-pin insertion",
        )
        ctx.expect_within(
            left_body,
            right_body,
            axes="yz",
            margin=0.001,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            name=f"{knuckle_name} surrounds the hinge pin radially",
        )

    ctx.expect_overlap(
        left_body,
        right_body,
        axes="x",
        min_overlap=0.14,
        elem_a="left_main_tube",
        elem_b="right_main_tube",
        name="paired optical barrels align along the objective axis",
    )
    ctx.expect_gap(
        focus_wheel,
        left_body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        elem_a="knurled_wheel",
        elem_b="focus_saddle",
        name="focus wheel is seated on the center saddle",
    )
    ctx.expect_contact(
        compass_housing,
        right_body,
        elem_a="compass_pod",
        elem_b="compass_recess",
        contact_tol=0.001,
        name="compass pod sits in the right barrel recess",
    )

    rest_right_aabb = ctx.part_world_aabb(right_body)
    with ctx.pose({central_hinge: central_hinge.motion_limits.upper}):
        folded_right_aabb = ctx.part_world_aabb(right_body)
    ctx.check(
        "right barrel folds about the central longitudinal hinge",
        rest_right_aabb is not None
        and folded_right_aabb is not None
        and folded_right_aabb[0][2] < rest_right_aabb[0][2] - 0.006,
        details=f"rest={rest_right_aabb}, folded={folded_right_aabb}",
    )

    rest_mark = ctx.part_element_world_aabb(focus_wheel, elem="index_mark")
    with ctx.pose({focus_axis: 0.7}):
        turned_mark = ctx.part_element_world_aabb(focus_wheel, elem="index_mark")
    ctx.check(
        "focus wheel index mark moves when the wheel rotates",
        rest_mark is not None
        and turned_mark is not None
        and abs(turned_mark[0][0] - rest_mark[0][0]) > 0.004,
        details=f"rest={rest_mark}, turned={turned_mark}",
    )

    rest_pointer = ctx.part_element_world_aabb(compass_housing, elem="compass_pointer")
    with ctx.pose({compass_axis: 0.9}):
        turned_pointer = ctx.part_element_world_aabb(compass_housing, elem="compass_pointer")
    ctx.check(
        "compass housing rotates in its right barrel socket",
        rest_pointer is not None
        and turned_pointer is not None
        and abs(turned_pointer[0][1] - rest_pointer[0][1]) > 0.006,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
