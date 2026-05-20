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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.34
BASE_DEPTH = 0.23
BASE_HEIGHT = 0.025
BASE_HOUSING_RADIUS = 0.062
BASE_HOUSING_HEIGHT = 0.028

YOKE_COLLAR_RADIUS = 0.054
YOKE_COLLAR_HEIGHT = 0.024
YOKE_TRUNNION_CENTER_Z = 0.168
YOKE_ARM_THICKNESS = 0.020
YOKE_ARM_DEPTH = 0.040
YOKE_ARM_HEIGHT = 0.190
YOKE_BRIDGE_HEIGHT = 0.016
YOKE_BRIDGE_DEPTH = 0.048
YOKE_ARM_INNER_X = 0.112
YOKE_ARM_CENTER_X = YOKE_ARM_INNER_X + (YOKE_ARM_THICKNESS / 2.0)
YOKE_BOSS_RADIUS = 0.022
YOKE_BOSS_LENGTH = 0.010

CAN_RADIUS = 0.090
CAN_INNER_RADIUS = 0.082
CAN_LENGTH = 0.300
CAN_FRONT_OPEN_OVERHANG = 0.005
CAN_REAR_CAP_THICKNESS = 0.015
BEZEL_RADIUS = 0.098
BEZEL_DEPTH = 0.020
LENS_RADIUS = 0.076
LENS_DEPTH = 0.004
TRUNNION_RADIUS = 0.012
TRUNNION_LENGTH = 0.022
TRUNNION_CENTER_X = CAN_RADIUS + (TRUNNION_LENGTH / 2.0)

FLAP_WIDTH = 0.092
FLAP_LENGTH = 0.080
FLAP_THICKNESS = 0.004
FLAP_HINGE_RADIUS = 0.005
FLAP_CENTER_BARREL_LENGTH = 0.044
FLAP_SIDE_LUG_LENGTH = 0.018
FLAP_LUG_CENTER_X = 0.034
FLAP_HINGE_Y = -0.145
FLAP_HINGE_Z = 0.095
FLAP_OPENING_WIDTH = 0.080
FLAP_OPENING_LENGTH = 0.072
FLAP_OPENING_CENTER_Y = -0.101
FLAP_OPENING_CENTER_Z = 0.095


def _cylinder_along_x(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _cylinder_along_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    housing = (
        cq.Workplane("XY")
        .circle(BASE_HOUSING_RADIUS)
        .extrude(BASE_HOUSING_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    return plate.union(housing)


def _make_can_shape() -> cq.Workplane:
    shell = _cylinder_along_y(CAN_RADIUS, CAN_LENGTH)
    inner_void = _cylinder_along_y(
        CAN_INNER_RADIUS,
        CAN_LENGTH - CAN_REAR_CAP_THICKNESS + CAN_FRONT_OPEN_OVERHANG,
    ).translate((0.0, 0.010, 0.0))
    shell = shell.cut(inner_void)

    bezel = _cylinder_along_y(BEZEL_RADIUS, BEZEL_DEPTH).translate(
        (0.0, (CAN_LENGTH / 2.0) + (BEZEL_DEPTH / 2.0), 0.0)
    )
    bezel_void = _cylinder_along_y(LENS_RADIUS + 0.004, BEZEL_DEPTH + 0.004).translate(
        (0.0, (CAN_LENGTH / 2.0) + (BEZEL_DEPTH / 2.0), 0.0)
    )
    shell = shell.union(bezel.cut(bezel_void))

    hatch_cut = (
        cq.Workplane("XY")
        .box(FLAP_OPENING_WIDTH, FLAP_OPENING_LENGTH, 0.050)
        .translate((0.0, FLAP_OPENING_CENTER_Y, FLAP_OPENING_CENTER_Z))
    )
    shell = shell.cut(hatch_cut)

    return shell


def _make_flap_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(FLAP_WIDTH, FLAP_LENGTH, FLAP_THICKNESS)
        .translate((0.0, (FLAP_LENGTH / 2.0) + 0.006, -FLAP_THICKNESS / 2.0))
    )
    center_barrel = _cylinder_along_x(FLAP_HINGE_RADIUS, FLAP_CENTER_BARREL_LENGTH)
    hinge_leaf = (
        cq.Workplane("XY")
        .box(FLAP_CENTER_BARREL_LENGTH, 0.010, FLAP_THICKNESS)
        .translate((0.0, 0.010, -FLAP_THICKNESS / 2.0))
    )
    return panel.union(center_barrel).union(hinge_leaf)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_spotlight")

    base_finish = model.material("base_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    can_finish = model.material("can_finish", rgba=(0.20, 0.20, 0.21, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.10, 0.12, 0.14, 0.85))
    metal_finish = model.material("metal_finish", rgba=(0.52, 0.54, 0.57, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material=base_finish,
        name="base_shell",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=YOKE_COLLAR_RADIUS, length=YOKE_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, YOKE_COLLAR_HEIGHT / 2.0)),
        material=frame_finish,
        name="pan_collar",
    )
    yoke.visual(
        Box((2.0 * YOKE_ARM_CENTER_X + YOKE_ARM_THICKNESS, YOKE_BRIDGE_DEPTH, YOKE_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, YOKE_COLLAR_HEIGHT + (YOKE_BRIDGE_HEIGHT / 2.0))),
        material=frame_finish,
        name="yoke_bridge",
    )
    for x_sign in (-1.0, 1.0):
        yoke.visual(
            Box((YOKE_ARM_THICKNESS, YOKE_ARM_DEPTH, YOKE_ARM_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_sign * YOKE_ARM_CENTER_X,
                    0.0,
                    YOKE_COLLAR_HEIGHT + (YOKE_ARM_HEIGHT / 2.0),
                )
            ),
            material=frame_finish,
            name=f"arm_{0 if x_sign < 0.0 else 1}",
        )
        yoke.visual(
            Cylinder(radius=YOKE_BOSS_RADIUS, length=YOKE_BOSS_LENGTH),
            origin=Origin(
                xyz=(
                    x_sign
                    * (YOKE_ARM_CENTER_X + (YOKE_ARM_THICKNESS / 2.0) + (YOKE_BOSS_LENGTH / 2.0)),
                    0.0,
                    YOKE_COLLAR_HEIGHT + YOKE_TRUNNION_CENTER_Z,
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=frame_finish,
            name=f"boss_{0 if x_sign < 0.0 else 1}",
        )
    yoke.visual(
        Box((0.080, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, -0.012, YOKE_COLLAR_HEIGHT + YOKE_BRIDGE_HEIGHT + 0.010)),
        material=frame_finish,
        name="tilt_block",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_make_can_shape(), "can_shell"),
        material=can_finish,
        name="can_shell",
    )
    for x_sign in (-1.0, 1.0):
        can.visual(
            Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
            origin=Origin(
                xyz=(x_sign * TRUNNION_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal_finish,
            name=f"trunnion_{0 if x_sign < 0.0 else 1}",
        )
        can.visual(
            Cylinder(radius=FLAP_HINGE_RADIUS, length=FLAP_SIDE_LUG_LENGTH),
            origin=Origin(
                xyz=(x_sign * FLAP_LUG_CENTER_X, FLAP_HINGE_Y, FLAP_HINGE_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=can_finish,
            name=f"hinge_lug_{0 if x_sign < 0.0 else 1}",
        )
        can.visual(
            Box((FLAP_SIDE_LUG_LENGTH, 0.012, 0.010)),
            origin=Origin(
                xyz=(x_sign * FLAP_LUG_CENTER_X, FLAP_HINGE_Y + 0.006, FLAP_HINGE_Z - 0.005)
            ),
            material=can_finish,
            name=f"lug_tab_{0 if x_sign < 0.0 else 1}",
        )
    can.visual(
        Cylinder(radius=0.080, length=0.012),
        origin=Origin(
            xyz=(0.0, (CAN_LENGTH / 2.0) - 0.004, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_finish,
        name="front_lens",
    )

    flap = model.part("service_flap")
    flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "service_flap"),
        material=metal_finish,
        name="flap_panel",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BASE_HOUSING_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.6,
            lower=-2.1,
            upper=2.1,
        ),
    )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, YOKE_COLLAR_HEIGHT + YOKE_TRUNNION_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.8,
            lower=-0.85,
            upper=1.15,
        ),
    )
    model.articulation(
        "can_to_service_flap",
        ArticulationType.REVOLUTE,
        parent=can,
        child=flap,
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    flap = object_model.get_part("service_flap")
    pan_joint = object_model.get_articulation("base_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_can")
    flap_joint = object_model.get_articulation("can_to_service_flap")

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx.expect_origin_distance(
        yoke,
        base,
        axes="xy",
        max_dist=0.001,
        name="yoke pans about the centered base pivot",
    )
    ctx.expect_gap(
        can,
        base,
        axis="z",
        min_gap=0.090,
        name="can body clears the floor base at rest",
    )
    ctx.expect_overlap(
        flap,
        can,
        axes="xy",
        elem_a="flap_panel",
        elem_b="can_shell",
        min_overlap=0.060,
        name="service flap sits over the rear shell opening",
    )

    front_rest = elem_center(can, "front_lens")
    flap_rest = elem_center(flap, "flap_panel")

    with ctx.pose({pan_joint: 0.85}):
        front_pan = elem_center(can, "front_lens")
    ctx.check(
        "pan joint swings the can sideways on the base",
        front_rest is not None
        and front_pan is not None
        and front_pan[0] > front_rest[0] + 0.10
        and front_pan[1] < front_rest[1] - 0.03,
        details=f"rest={front_rest}, panned={front_pan}",
    )

    with ctx.pose({tilt_joint: 0.80}):
        front_tilt = elem_center(can, "front_lens")
    ctx.check(
        "tilt hinge raises the beam axis",
        front_rest is not None
        and front_tilt is not None
        and front_tilt[2] > front_rest[2] + 0.09,
        details=f"rest={front_rest}, tilted={front_tilt}",
    )

    with ctx.pose({flap_joint: 1.05}):
        flap_open = elem_center(flap, "flap_panel")
    ctx.check(
        "service flap opens upward from the rear shell hinge",
        flap_rest is not None
        and flap_open is not None
        and flap_open[2] > flap_rest[2] + 0.025
        and flap_open[1] < flap_rest[1] - 0.015,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    return ctx.report()


object_model = build_object_model()
