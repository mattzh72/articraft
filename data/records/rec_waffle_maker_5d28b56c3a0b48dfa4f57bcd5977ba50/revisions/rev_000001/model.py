from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.280
BASE_D = 0.235
BASE_H = 0.047
FEET_H = 0.006

PLATE_W = 0.206
PLATE_D = 0.172
PLATE_THICKNESS = 0.007
RIB_HEIGHT = 0.004
RIB_WIDTH = 0.006

LID_FORWARD = 0.218
LID_REAR = 0.002
LID_D = LID_FORWARD + LID_REAR
LID_W = 0.278
LID_H = 0.038
LID_BOTTOM = -0.004
LID_CENTER_Y = (LID_REAR - LID_FORWARD) * 0.5

HINGE_Y = BASE_D * 0.5 - 0.012
HINGE_Z = FEET_H + BASE_H + 0.006

KNOB_Y = -0.026
KNOB_Z = FEET_H + 0.026
KNOB_X = BASE_W * 0.5 + 0.006


def _waffle_plate_shape(*, width: float, depth: float) -> cq.Workplane:
    plate = cq.Workplane("XY").rect(width, depth).extrude(PLATE_THICKNESS)
    usable_w = width - 0.024
    usable_d = depth - 0.024

    for fraction in (-0.25, 0.0, 0.25):
        rib = (
            cq.Workplane("XY")
            .rect(RIB_WIDTH, usable_d)
            .extrude(RIB_HEIGHT)
            .translate((usable_w * fraction, 0.0, PLATE_THICKNESS))
        )
        plate = plate.union(rib)

    for fraction in (-0.25, 0.0, 0.25):
        rib = (
            cq.Workplane("XY")
            .rect(usable_w, RIB_WIDTH)
            .extrude(RIB_HEIGHT)
            .translate((0.0, usable_d * fraction, PLATE_THICKNESS))
        )
        plate = plate.union(rib)

    return plate


def _base_shell_shape() -> cq.Workplane:
    body = cq.Workplane("XY").rect(BASE_W, BASE_D).extrude(BASE_H)
    body = body.edges("|Z").fillet(0.020)
    body = body.edges(">Z").fillet(0.008)

    pocket = (
        cq.Workplane("XY")
        .rect(PLATE_W + 0.026, PLATE_D + 0.022)
        .extrude(0.021)
        .translate((0.0, -0.006, BASE_H - 0.017))
    )
    body = body.cut(pocket)

    finger_notch = (
        cq.Workplane("XY")
        .rect(0.110, 0.016)
        .extrude(0.010)
        .translate((0.0, -BASE_D * 0.5 + 0.008, BASE_H - 0.008))
    )
    return body.cut(finger_notch)


def _lid_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .rect(LID_W, LID_D)
        .extrude(LID_H)
        .translate((0.0, LID_CENTER_Y, LID_BOTTOM))
    )
    shell = shell.edges("|Z").fillet(0.019)
    shell = shell.edges(">Z").fillet(0.008)

    cavity = (
        cq.Workplane("XY")
        .rect(PLATE_W + 0.030, PLATE_D + 0.028)
        .extrude(LID_H - 0.010)
        .translate((0.0, LID_CENTER_Y - 0.004, LID_BOTTOM - 0.001))
    )
    shell = shell.cut(cavity)

    barrel = (
        cq.Workplane("XY")
        .circle(0.0085)
        .extrude(0.070, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, 0.001, 0.004))
    )
    return shell.union(barrel)


def _knob_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.019)
        .extrude(0.0025)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    grip = (
        cq.Workplane("XY")
        .circle(0.0165)
        .extrude(0.016)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0025, 0.0, 0.0))
    )
    pointer = cq.Workplane("XY").box(0.006, 0.003, 0.010).translate((0.015, 0.0, 0.0125))
    return collar.union(grip).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_waffle_maker")

    body_black = model.material("body_black", rgba=(0.14, 0.14, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.78, 0.79, 1.0))
    plate_dark = model.material("plate_dark", rgba=(0.19, 0.20, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "waffle_maker_base_shell"),
        origin=Origin(xyz=(0.0, 0.0, FEET_H)),
        material=body_black,
        name="housing",
    )
    base.visual(
        mesh_from_cadquery(_waffle_plate_shape(width=PLATE_W, depth=PLATE_D), "waffle_maker_lower_plate"),
        origin=Origin(xyz=(0.0, -0.006, 0.0345)),
        material=plate_dark,
        name="lower_plate",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.098, -0.078),
            (0.098, -0.078),
            (-0.090, 0.073),
            (0.090, 0.073),
        )
    ):
        base.visual(
            Cylinder(radius=0.009, length=FEET_H),
            origin=Origin(xyz=(x_pos, y_pos, FEET_H * 0.5)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(BASE_W * 0.5 + 0.003, KNOB_Y, KNOB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="knob_collar",
    )
    for index, x_pos in enumerate((-0.102, 0.102)):
        base.visual(
            Box((0.036, 0.016, 0.018)),
            origin=Origin(xyz=(x_pos, BASE_D * 0.5 - 0.009, FEET_H + BASE_H - 0.003)),
            material=body_black,
            name=f"hinge_yoke_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, FEET_H + BASE_H + 0.015)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "waffle_maker_lid_shell"),
        material=body_black,
        name="shell",
    )
    lid.visual(
        mesh_from_cadquery(_waffle_plate_shape(width=PLATE_W - 0.006, depth=PLATE_D - 0.006), "waffle_maker_upper_plate"),
        origin=Origin(xyz=(0.0, LID_CENTER_Y - 0.004, -0.012)),
        material=plate_dark,
        name="upper_plate",
    )
    lid.visual(
        Box((0.240, 0.204, 0.014)),
        origin=Origin(xyz=(0.0, LID_CENTER_Y - 0.004, 0.006)),
        material=plate_dark,
        name="plate_backer",
    )
    lid.visual(
        Box((0.226, 0.168, 0.002)),
        origin=Origin(xyz=(0.0, LID_CENTER_Y - 0.010, 0.033)),
        material=steel,
        name="top_panel",
    )
    lid.visual(
        Box((0.126, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.222, 0.010)),
        material=trim_black,
        name="handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D + 0.018, LID_H)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.108, 0.014)),
    )

    side_knob = model.part("side_knob")
    side_knob.visual(
        mesh_from_cadquery(_knob_shape(), "waffle_maker_side_knob"),
        material=trim_black,
        name="knob",
    )
    side_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.018),
        mass=0.08,
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "base_to_side_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=side_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    side_knob = object_model.get_part("side_knob")
    lid_hinge = object_model.get_articulation("base_to_lid")
    knob_joint = object_model.get_articulation("base_to_side_knob")

    lid_limits = lid_hinge.motion_limits
    knob_limits = knob_joint.motion_limits

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        max_gap=0.003,
        max_penetration=0.0,
        name="cooking plates nearly meet when closed",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.160,
        name="upper and lower plates align in plan",
    )
    ctx.expect_contact(
        side_knob,
        base,
        elem_a="knob",
        elem_b="knob_collar",
        name="browning knob mounts against side collar",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle")
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="upper_plate",
                negative_elem="lower_plate",
                min_gap=0.035,
                name="upper plate lifts clear when open",
            )
            open_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle")
        ctx.check(
            "lid handle rises when opening",
            closed_handle_aabb is not None
            and open_handle_aabb is not None
            and open_handle_aabb[0][2] > closed_handle_aabb[1][2] + 0.080,
            details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
        )

    ctx.check(
        "browning control uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_limits}",
    )

    return ctx.report()


object_model = build_object_model()
