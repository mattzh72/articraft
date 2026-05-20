from __future__ import annotations

from math import pi

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


BODY_L = 0.62
BODY_W = 0.35
BODY_H = 0.40
BODY_Z = 0.04
TOP_Z = BODY_Z + BODY_H
WALL = 0.014
FLOOR = 0.018

DRAWER_OPEN_Z = BODY_Z + 0.23
DRAWER_H = 0.076
DRAWER_PANEL_T = 0.018
DRAWER_PANEL_W = 0.326
DRAWER_PANEL_H = 0.082
DRAWER_TRAY_L = 0.204
DRAWER_TRAY_W = 0.298
DRAWER_TRAVEL = 0.13
DRAWER_SHELF_T = 0.010
DRAWER_SHELF_Z = DRAWER_OPEN_Z - 0.008
DRAWER_ROOF_T = 0.010
DRAWER_ROOF_Z = DRAWER_OPEN_Z + DRAWER_H + 0.003 + (DRAWER_ROOF_T / 2.0)
DRAWER_CAVITY_L = 0.205

LID_L = 0.604
LID_W = 0.360
LID_TOP_T = 0.018
LID_SKIRT_T = 0.012
LID_SKIRT_H = 0.032
LID_FRONT_LIP_H = 0.050
LID_OPEN = 1.15

HANDLE_TUBE_R = 0.011
HANDLE_TUBE_L = 0.60
HANDLE_SPAN = 0.22
HANDLE_TRAVEL = 0.20
GUIDE_W = 0.050
GUIDE_T = 0.020
GUIDE_H = 0.20
GUIDE_Z = 0.27
GUIDE_Y = HANDLE_SPAN / 2.0
HANDLE_X = -(BODY_L / 2.0) - 0.027
HANDLE_JOINT_Z = 0.37

WHEEL_R = 0.082
WHEEL_W = 0.050
WHEEL_X = -(BODY_L / 2.0) - 0.085
WHEEL_Y = (BODY_W / 2.0) + 0.030
WHEEL_Z = WHEEL_R


def _add_wheel(part, *, rubber, steel) -> None:
    spin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        origin=spin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=WHEEL_R * 0.64, length=WHEEL_W * 0.68),
        origin=spin,
        material=steel,
        name="rim",
    )
    part.visual(
        Cylinder(radius=WHEEL_R * 0.22, length=WHEEL_W * 0.96),
        origin=spin,
        material=steel,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    shell = model.material("shell", rgba=(0.18, 0.19, 0.20, 1.0))
    lid_color = model.material("lid_color", rgba=(0.12, 0.13, 0.14, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.22, 0.23, 0.24, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    trim = model.material("trim", rgba=(0.78, 0.34, 0.10, 1.0))

    bin_part = model.part("bin")
    bin_part.visual(
        Box((BODY_L, BODY_W, FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + (FLOOR / 2.0))),
        material=shell,
        name="floor",
    )
    bin_part.visual(
        Box((BODY_L, WALL, BODY_H - FLOOR)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_W / 2.0) - (WALL / 2.0),
                BODY_Z + FLOOR + ((BODY_H - FLOOR) / 2.0),
            )
        ),
        material=shell,
        name="side_0",
    )
    bin_part.visual(
        Box((BODY_L, WALL, BODY_H - FLOOR)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_W / 2.0) + (WALL / 2.0),
                BODY_Z + FLOOR + ((BODY_H - FLOOR) / 2.0),
            )
        ),
        material=shell,
        name="side_1",
    )
    bin_part.visual(
        Box((WALL, BODY_W - (2.0 * WALL), BODY_H - FLOOR)),
        origin=Origin(
            xyz=(
                -(BODY_L / 2.0) + (WALL / 2.0),
                0.0,
                BODY_Z + FLOOR + ((BODY_H - FLOOR) / 2.0),
            )
        ),
        material=shell,
        name="rear_wall",
    )
    bin_part.visual(
        Box((WALL, BODY_W - (2.0 * WALL), DRAWER_OPEN_Z - BODY_Z)),
        origin=Origin(
            xyz=(
                (BODY_L / 2.0) - (WALL / 2.0),
                0.0,
                BODY_Z + ((DRAWER_OPEN_Z - BODY_Z) / 2.0),
            )
        ),
        material=shell,
        name="front_lower",
    )
    bin_part.visual(
        Box((WALL, BODY_W - (2.0 * WALL), TOP_Z - (DRAWER_OPEN_Z + DRAWER_H))),
        origin=Origin(
            xyz=(
                (BODY_L / 2.0) - (WALL / 2.0),
                0.0,
                DRAWER_OPEN_Z
                + DRAWER_H
                + ((TOP_Z - (DRAWER_OPEN_Z + DRAWER_H)) / 2.0),
            )
        ),
        material=shell,
        name="front_upper",
    )
    bin_part.visual(
        Box((DRAWER_CAVITY_L, BODY_W - (2.0 * WALL), DRAWER_SHELF_T)),
        origin=Origin(
            xyz=(
                (BODY_L / 2.0) - WALL - (DRAWER_CAVITY_L / 2.0),
                0.0,
                DRAWER_SHELF_Z,
            )
        ),
        material=shell,
        name="drawer_shelf",
    )
    bin_part.visual(
        Box((DRAWER_CAVITY_L, BODY_W - (2.0 * WALL), DRAWER_ROOF_T)),
        origin=Origin(
            xyz=(
                (BODY_L / 2.0) - WALL - (DRAWER_CAVITY_L / 2.0),
                0.0,
                DRAWER_ROOF_Z,
            )
        ),
        material=shell,
        name="drawer_roof",
    )
    bin_part.visual(
        Box((WALL, BODY_W - (2.0 * WALL), DRAWER_H + 0.014)),
        origin=Origin(
            xyz=(
                (BODY_L / 2.0) - WALL - DRAWER_CAVITY_L - (WALL / 2.0),
                0.0,
                DRAWER_OPEN_Z + (DRAWER_H / 2.0),
            )
        ),
        material=shell,
        name="drawer_bulkhead",
    )
    for sign in (-1.0, 1.0):
        bin_part.visual(
            Box((DRAWER_CAVITY_L - 0.020, 0.006, 0.008)),
            origin=Origin(
                xyz=(
                    (BODY_L / 2.0) - WALL - ((DRAWER_CAVITY_L - 0.020) / 2.0),
                    sign * ((BODY_W / 2.0) - WALL - 0.003),
                    DRAWER_OPEN_Z + (DRAWER_H * 0.48),
                )
            ),
            material=steel,
            name=f"runner_{0 if sign < 0.0 else 1}",
        )
    for sign in (-1.0, 1.0):
        bin_part.visual(
            Box((0.10, 0.050, BODY_Z)),
            origin=Origin(
                xyz=((BODY_L / 2.0) - 0.08, sign * 0.105, BODY_Z / 2.0)
            ),
            material=shell,
            name=f"foot_{0 if sign < 0.0 else 1}",
        )
    bin_part.visual(
        Box((0.060, BODY_W - 0.060, 0.050)),
        origin=Origin(xyz=(WHEEL_X + 0.018, 0.0, WHEEL_Z + 0.015)),
        material=shell,
        name="axle_beam",
    )
    for sign in (-1.0, 1.0):
        bin_part.visual(
            Box((0.060, 0.052, 0.100)),
            origin=Origin(
                xyz=(WHEEL_X + 0.018, sign * 0.154, WHEEL_Z + 0.025)
            ),
            material=shell,
            name=f"wheel_bracket_{0 if sign < 0.0 else 1}",
        )
    for sign in (-1.0, 1.0):
        bin_part.visual(
            Box((GUIDE_T, GUIDE_W, GUIDE_H)),
            origin=Origin(
                xyz=(
                    HANDLE_X,
                    sign * GUIDE_Y,
                    GUIDE_Z,
                )
            ),
            material=shell,
            name=f"guide_{0 if sign < 0.0 else 1}",
        )
    for sign in (-1.0, 1.0):
        bin_part.visual(
            Box((0.052, 0.040, 0.120)),
            origin=Origin(
                xyz=(
                    -(BODY_L / 2.0) - 0.012,
                    sign * 0.148,
                    WHEEL_Z + 0.040,
                )
            ),
            material=shell,
            name=f"axle_support_{0 if sign < 0.0 else 1}",
        )
    bin_part.visual(
        Box((0.070, 0.100, 0.170)),
        origin=Origin(xyz=(-(BODY_L / 2.0) + 0.035, 0.0, BODY_Z + 0.235)),
        material=shell,
        name="rear_housing",
    )
    bin_part.visual(
        Box((0.034, 0.100, 0.006)),
        origin=Origin(xyz=((BODY_L / 2.0) - 0.017, 0.0, TOP_Z - 0.003)),
        material=trim,
        name="front_trim",
    )

    lid_part = model.part("lid")
    lid_part.visual(
        Box((LID_L, LID_W, LID_TOP_T)),
        origin=Origin(xyz=(LID_L / 2.0, 0.0, LID_TOP_T / 2.0)),
        material=lid_color,
        name="top_skin",
    )
    lid_part.visual(
        Box((LID_L - 0.010, LID_SKIRT_T, LID_SKIRT_H)),
        origin=Origin(
            xyz=((LID_L - 0.010) / 2.0, (LID_W / 2.0) + (LID_SKIRT_T / 2.0), -0.004)
        ),
        material=lid_color,
        name="side_skirt_0",
    )
    lid_part.visual(
        Box((LID_L - 0.010, LID_SKIRT_T, LID_SKIRT_H)),
        origin=Origin(
            xyz=((LID_L - 0.010) / 2.0, -((LID_W / 2.0) + (LID_SKIRT_T / 2.0)), -0.004)
        ),
        material=lid_color,
        name="side_skirt_1",
    )
    lid_part.visual(
        Box((LID_SKIRT_T, LID_W + 0.004, LID_FRONT_LIP_H)),
        origin=Origin(xyz=(LID_L + (LID_SKIRT_T / 2.0), 0.0, -0.010)),
        material=lid_color,
        name="front_lip",
    )
    lid_part.visual(
        Box((0.052, 0.010, 0.020)),
        origin=Origin(xyz=(LID_L + 0.034, 0.115, -0.016)),
        material=steel,
        name="latch_0",
    )
    lid_part.visual(
        Box((0.052, 0.010, 0.020)),
        origin=Origin(xyz=(LID_L + 0.034, -0.115, -0.016)),
        material=steel,
        name="latch_1",
    )

    drawer_part = model.part("drawer")
    drawer_part.visual(
        Box((DRAWER_PANEL_T, DRAWER_PANEL_W, DRAWER_PANEL_H)),
        origin=Origin(xyz=(DRAWER_PANEL_T / 2.0, 0.0, 0.0)),
        material=drawer_color,
        name="front_panel",
    )
    drawer_part.visual(
        Box((DRAWER_TRAY_L, DRAWER_TRAY_W, 0.008)),
        origin=Origin(
            xyz=(
                -(DRAWER_TRAY_L / 2.0),
                0.0,
                -(DRAWER_H / 2.0) + 0.010,
            )
        ),
        material=drawer_color,
        name="tray",
    )
    drawer_part.visual(
        Box((DRAWER_TRAY_L - 0.010, 0.006, 0.048)),
        origin=Origin(
            xyz=(
                -((DRAWER_TRAY_L - 0.010) / 2.0) - 0.005,
                (DRAWER_TRAY_W / 2.0) - 0.003,
                -(DRAWER_H / 2.0) + 0.030,
            )
        ),
        material=drawer_color,
        name="side_0",
    )
    drawer_part.visual(
        Box((DRAWER_TRAY_L - 0.010, 0.006, 0.048)),
        origin=Origin(
            xyz=(
                -((DRAWER_TRAY_L - 0.010) / 2.0) - 0.005,
                -((DRAWER_TRAY_W / 2.0) - 0.003),
                -(DRAWER_H / 2.0) + 0.030,
            )
        ),
        material=drawer_color,
        name="side_1",
    )
    drawer_part.visual(
        Box((0.006, DRAWER_TRAY_W - 0.012, 0.046)),
        origin=Origin(
            xyz=(-(DRAWER_TRAY_L) + 0.003, 0.0, -(DRAWER_H / 2.0) + 0.029)
        ),
        material=drawer_color,
        name="rear_wall",
    )
    drawer_part.visual(
        Cylinder(radius=0.008, length=0.150),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pull",
    )

    handle_part = model.part("handle")
    handle_part.visual(
        Cylinder(radius=HANDLE_TUBE_R, length=HANDLE_TUBE_L),
        origin=Origin(xyz=(0.0, GUIDE_Y, 0.020)),
        material=handle_metal,
        name="right_tube",
    )
    handle_part.visual(
        Cylinder(radius=HANDLE_TUBE_R, length=HANDLE_TUBE_L),
        origin=Origin(xyz=(0.0, -GUIDE_Y, 0.020)),
        material=handle_metal,
        name="left_tube",
    )
    handle_part.visual(
        Cylinder(radius=0.016, length=HANDLE_SPAN),
        origin=Origin(xyz=(0.0, 0.0, 0.320), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="grip",
    )

    wheel_0 = model.part("wheel_0")
    _add_wheel(wheel_0, rubber=rubber, steel=steel)

    wheel_1 = model.part("wheel_1")
    _add_wheel(wheel_1, rubber=rubber, steel=steel)

    model.articulation(
        "bin_to_lid",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=lid_part,
        origin=Origin(xyz=(-(BODY_L / 2.0) + 0.016, 0.0, TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=0.0, upper=LID_OPEN),
    )
    model.articulation(
        "bin_to_drawer",
        ArticulationType.PRISMATIC,
        parent=bin_part,
        child=drawer_part,
        origin=Origin(xyz=((BODY_L / 2.0), 0.0, DRAWER_OPEN_Z + (DRAWER_H / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "bin_to_handle",
        ArticulationType.PRISMATIC,
        parent=bin_part,
        child=handle_part,
        origin=Origin(xyz=(HANDLE_X, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )
    model.articulation(
        "bin_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=bin_part,
        child=wheel_0,
        origin=Origin(xyz=(WHEEL_X, -WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )
    model.articulation(
        "bin_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=bin_part,
        child=wheel_1,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bin_part = object_model.get_part("bin")
    lid_part = object_model.get_part("lid")
    drawer_part = object_model.get_part("drawer")
    handle_part = object_model.get_part("handle")
    wheel_0 = object_model.get_articulation("bin_to_wheel_0")
    wheel_1 = object_model.get_articulation("bin_to_wheel_1")
    lid_hinge = object_model.get_articulation("bin_to_lid")
    drawer_slide = object_model.get_articulation("bin_to_drawer")
    handle_slide = object_model.get_articulation("bin_to_handle")

    ctx.allow_overlap(
        handle_part,
        bin_part,
        elem_a="left_tube",
        elem_b="guide_0",
        reason="The telescoping handle tube is intentionally represented sliding through a simplified rear guide block.",
    )
    ctx.allow_overlap(
        handle_part,
        bin_part,
        elem_a="right_tube",
        elem_b="guide_1",
        reason="The telescoping handle tube is intentionally represented sliding through a simplified rear guide block.",
    )

    with ctx.pose({lid_hinge: 0.0, drawer_slide: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            lid_part,
            bin_part,
            axis="z",
            positive_elem="top_skin",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid seats on the bin rim",
        )
        ctx.expect_overlap(
            lid_part,
            bin_part,
            axes="xy",
            elem_a="top_skin",
            min_overlap=0.28,
            name="lid covers the bin opening",
        )
        ctx.expect_gap(
            drawer_part,
            bin_part,
            axis="x",
            positive_elem="front_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="drawer front closes flush",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid_part, elem="front_lip")
    with ctx.pose({lid_hinge: LID_OPEN, handle_slide: 0.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid_part, elem="front_lip")
        ctx.expect_gap(
            lid_part,
            handle_part,
            axis="x",
            min_gap=0.01,
            name="open lid stays forward of the rear handle",
        )
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    drawer_rest = ctx.part_world_position(drawer_part)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        drawer_open = ctx.part_world_position(drawer_part)
        ctx.expect_overlap(
            drawer_part,
            bin_part,
            axes="x",
            elem_a="tray",
            elem_b="drawer_roof",
            min_overlap=0.055,
            name="drawer remains engaged on its runners",
        )
    ctx.check(
        "drawer slides forward",
        drawer_rest is not None
        and drawer_open is not None
        and drawer_open[0] > drawer_rest[0] + 0.10,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    handle_rest = ctx.part_world_position(handle_part)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        handle_open = ctx.part_world_position(handle_part)
        ctx.expect_within(
            handle_part,
            bin_part,
            axes="xy",
            inner_elem="left_tube",
            outer_elem="guide_0",
            margin=0.002,
            name="left handle tube stays centered in its guide",
        )
        ctx.expect_overlap(
            handle_part,
            bin_part,
            axes="z",
            elem_a="left_tube",
            elem_b="guide_0",
            min_overlap=0.05,
            name="left handle tube stays engaged in its guide",
        )
        ctx.expect_within(
            handle_part,
            bin_part,
            axes="xy",
            inner_elem="right_tube",
            outer_elem="guide_1",
            margin=0.002,
            name="right handle tube stays centered in its guide",
        )
        ctx.expect_overlap(
            handle_part,
            bin_part,
            axes="z",
            elem_a="right_tube",
            elem_b="guide_1",
            min_overlap=0.05,
            name="right handle tube stays engaged in its guide",
        )
    ctx.check(
        "handle extends upward",
        handle_rest is not None
        and handle_open is not None
        and handle_open[2] > handle_rest[2] + 0.12,
        details=f"rest={handle_rest}, open={handle_open}",
    )

    ctx.check(
        "rear wheels use continuous spin joints",
        (
            wheel_0.articulation_type == ArticulationType.CONTINUOUS
            and wheel_1.articulation_type == ArticulationType.CONTINUOUS
            and wheel_0.motion_limits is not None
            and wheel_1.motion_limits is not None
            and wheel_0.motion_limits.lower is None
            and wheel_0.motion_limits.upper is None
            and wheel_1.motion_limits.lower is None
            and wheel_1.motion_limits.upper is None
        ),
        details=(
            f"wheel_0={wheel_0.articulation_type}, limits={wheel_0.motion_limits}; "
            f"wheel_1={wheel_1.articulation_type}, limits={wheel_1.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
