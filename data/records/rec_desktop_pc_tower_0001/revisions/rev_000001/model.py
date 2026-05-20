from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    AssetContext,
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


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_W = 0.235
BODY_D = 0.465
BODY_H = 0.468
FOOT_H = 0.017
TOTAL_H = BODY_H + FOOT_H
WALL_T = 0.0022

PANEL_T = 0.004
PANEL_DEPTH = BODY_D - 0.024
PANEL_HEIGHT = BODY_H - 0.034
PANEL_FRAME = 0.022
PANEL_RAIL_DEPTH = PANEL_DEPTH - 0.050
PANEL_RAIL_Y = 0.025 + PANEL_RAIL_DEPTH * 0.5
SIDE_PANEL_PROUD = 0.0012
FRONT_INTAKE_TOP_MOUNT_Z = FOOT_H + BODY_H - 0.028
FRONT_INTAKE_CENTER_OFFSET_Z = FOOT_H + BODY_H * 0.5 - FRONT_INTAKE_TOP_MOUNT_Z


def _centered_box(sx: float, sy: float, sz: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _bottom_box(
    sx: float,
    sy: float,
    sz: float,
    center_xy: tuple[float, float],
    z0: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((center_xy[0], center_xy[1], z0))
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _build_chassis_shape() -> cq.Workplane:
    shell = _bottom_box(BODY_W, BODY_D, BODY_H, (0.0, 0.0), FOOT_H)
    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - WALL_T + 0.004,
            BODY_D - 2.0 * WALL_T,
            BODY_H - 2.0 * WALL_T,
            centered=(True, True, False),
        )
        .translate((-(WALL_T + 0.004) * 0.5, 0.0, FOOT_H + WALL_T))
    )
    shell = shell.cut(cavity)

    front_open = _centered_box(
        BODY_W - 0.038,
        0.028,
        BODY_H - 0.092,
        (0.0, BODY_D * 0.5 - 0.010, FOOT_H + BODY_H * 0.5),
    )
    shell = shell.cut(front_open)

    io_cut = _centered_box(
        0.054,
        0.028,
        0.026,
        (-0.062, -BODY_D * 0.5 + 0.004, FOOT_H + BODY_H - 0.110),
    )
    psu_cut = _centered_box(
        0.146,
        0.030,
        0.086,
        (0.006, -BODY_D * 0.5 + 0.004, FOOT_H + 0.083),
    )
    rear_fan_cut = _cylinder_y(
        0.061,
        0.032,
        (0.056, -BODY_D * 0.5 + 0.004, FOOT_H + BODY_H - 0.104),
    )
    shell = shell.cut(io_cut).cut(psu_cut).cut(rear_fan_cut)

    for idx in range(7):
        slot = _centered_box(
            0.110,
            0.030,
            0.012,
            (0.018, -BODY_D * 0.5 + 0.004, FOOT_H + 0.168 + idx * 0.017),
        )
        shell = shell.cut(slot)

    for x_pos in (-0.048, -0.024, 0.0, 0.024, 0.048):
        top_slot = _centered_box(
            0.010,
            BODY_D * 0.34,
            0.012,
            (x_pos, -0.010, FOOT_H + BODY_H - 0.003),
        )
        shell = shell.cut(top_slot)

    shroud = _bottom_box(
        BODY_W - 0.030,
        BODY_D - 0.180,
        0.112,
        (0.0, 0.032),
        FOOT_H,
    )
    cable_bar = _bottom_box(
        0.018,
        BODY_D - 0.200,
        0.158,
        (0.050, 0.044),
        FOOT_H + 0.112,
    )
    shell = shell.union(shroud).union(cable_bar)

    for sx in (-0.084, 0.084):
        for sy in (-0.154, 0.154):
            foot = _bottom_box(0.034, 0.078, FOOT_H, (sx, sy), 0.0)
            shell = shell.union(foot)

    return shell


def _build_front_intake_shape() -> cq.Workplane:
    width = BODY_W - 0.022
    height = BODY_H - 0.040
    thickness = 0.012

    grille = cq.Workplane("XY").box(width, thickness, height)
    grille = grille.cut(cq.Workplane("XY").box(width - 0.018, thickness + 0.004, height - 0.018))

    for idx in range(-7, 8):
        slat = cq.Workplane("XY").box(0.003, thickness * 0.78, height - 0.050).translate((idx * 0.013, 0.0, 0.0))
        grille = grille.union(slat)

    for z_pos in (-0.125, 0.0, 0.125):
        ring = (
            cq.Workplane("XZ")
            .center(0.0, z_pos)
            .circle(0.060)
            .circle(0.053)
            .extrude(thickness * 0.42, both=True)
        )
        hub = (
            cq.Workplane("XZ")
            .center(0.0, z_pos)
            .circle(0.015)
            .extrude(thickness * 0.60, both=True)
        )
        cross_v = cq.Workplane("XY").box(0.005, thickness * 0.60, 0.096).translate((0.0, 0.0, z_pos))
        cross_h = cq.Workplane("XY").box(0.096, thickness * 0.60, 0.005).translate((0.0, 0.0, z_pos))
        grille = grille.union(ring).union(hub).union(cross_v).union(cross_h)

    return grille


def _build_rear_io_shape() -> cq.Workplane:
    thickness = 0.006
    panel = cq.Workplane("XY").box(BODY_W - 0.008, thickness, BODY_H - 0.014)

    io_hole = cq.Workplane("XY").box(0.054, thickness + 0.004, 0.026).translate((-0.062, 0.0, 0.124))
    psu_hole = cq.Workplane("XY").box(0.146, thickness + 0.004, 0.086).translate((0.006, 0.0, -0.168))
    fan_hole = cq.Workplane("XZ").center(0.056, 0.130).circle(0.061).extrude(thickness + 0.004, both=True)
    rear = panel.cut(io_hole).cut(psu_hole).cut(fan_hole)

    for idx in range(7):
        z_pos = -0.066 + idx * 0.017
        slot_cut = cq.Workplane("XY").box(0.110, thickness + 0.004, 0.012).translate((0.018, 0.0, z_pos))
        slot_cover = cq.Workplane("XY").box(0.112, thickness * 0.60, 0.013).translate((0.018, thickness * 0.20, z_pos))
        rear = rear.cut(slot_cut).union(slot_cover)

    fan_ring = (
        cq.Workplane("XZ")
        .center(0.056, 0.130)
        .circle(0.068)
        .circle(0.060)
        .extrude(thickness * 0.42, both=True)
    )
    fan_cross_v = cq.Workplane("XY").box(0.006, thickness * 0.58, 0.110).translate((0.056, 0.0, 0.130))
    fan_cross_h = cq.Workplane("XY").box(0.110, thickness * 0.58, 0.006).translate((0.056, 0.0, 0.130))
    rear = rear.union(fan_ring).union(fan_cross_v).union(fan_cross_h)

    for x_pos in (-0.040, -0.010, 0.020, 0.050):
        psu_bar = cq.Workplane("XY").box(0.005, thickness * 0.56, 0.072).translate((x_pos, 0.0, -0.168))
        rear = rear.union(psu_bar)

    return rear


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="enthusiast_pc_tower", assets=ASSETS)

    powder_black = model.material("powder_black", rgba=(0.12, 0.13, 0.15, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.41, 0.43, 0.45, 1.0))
    glass = model.material("glass", rgba=(0.28, 0.34, 0.38, 0.28))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_build_chassis_shape(), "chassis.obj", assets=ASSETS),
        material=powder_black,
    )
    chassis.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, TOTAL_H)),
        mass=10.8,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.5)),
    )

    front_intake = model.part("front_intake")
    front_intake.visual(
        mesh_from_cadquery(_build_front_intake_shape(), "front_intake.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, FRONT_INTAKE_CENTER_OFFSET_Z)),
        material=charcoal,
    )
    for z_pos in (-0.125, 0.0, 0.125):
        front_intake.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(
                xyz=(0.0, 0.0, z_pos + FRONT_INTAKE_CENTER_OFFSET_Z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
        )
    front_intake.inertial = Inertial.from_geometry(
        Box((BODY_W - 0.022, 0.014, BODY_H - 0.040)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, FRONT_INTAKE_CENTER_OFFSET_Z)),
    )

    rear_io = model.part("rear_io")
    rear_io.visual(
        mesh_from_cadquery(_build_rear_io_shape(), "rear_io.obj", assets=ASSETS),
        material=steel,
    )
    rear_io.inertial = Inertial.from_geometry(
        Box((BODY_W - 0.008, 0.008, BODY_H - 0.014)),
        mass=0.8,
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((0.006, 0.018, PANEL_HEIGHT)),
        origin=Origin(xyz=(-0.003, 0.009, PANEL_HEIGHT * 0.5)),
        material=powder_black,
        name="hinge_spine",
    )
    side_panel.visual(
        Box((PANEL_T, PANEL_RAIL_DEPTH, PANEL_FRAME)),
        origin=Origin(
            xyz=(
                -(PANEL_T * 0.5 + SIDE_PANEL_PROUD),
                PANEL_RAIL_Y,
                PANEL_HEIGHT - PANEL_FRAME * 0.5,
            )
        ),
        material=powder_black,
        name="top_rail",
    )
    side_panel.visual(
        Box((PANEL_T, PANEL_RAIL_DEPTH, PANEL_FRAME)),
        origin=Origin(
            xyz=(
                -(PANEL_T * 0.5 + SIDE_PANEL_PROUD),
                PANEL_RAIL_Y,
                PANEL_FRAME * 0.5,
            )
        ),
        material=powder_black,
        name="bottom_rail",
    )
    side_panel.visual(
        Box((0.006, PANEL_FRAME, PANEL_HEIGHT - 0.034)),
        origin=Origin(
            xyz=(
                -(0.003 + SIDE_PANEL_PROUD),
                PANEL_DEPTH - PANEL_FRAME * 0.5,
                (PANEL_HEIGHT - 0.034) * 0.5 + 0.017,
            )
        ),
        material=powder_black,
        name="front_rail",
    )
    side_panel.visual(
        Box((0.003, PANEL_DEPTH - 2.0 * PANEL_FRAME + 0.004, PANEL_HEIGHT - 2.0 * PANEL_FRAME + 0.004)),
        origin=Origin(
            xyz=(
                -0.0033,
                PANEL_DEPTH * 0.5,
                PANEL_HEIGHT * 0.5,
            )
        ),
        material=glass,
        name="tempered_glass",
    )
    side_panel.visual(
        Box((0.006, 0.030, 0.080)),
        origin=Origin(xyz=(-(0.003 + SIDE_PANEL_PROUD), PANEL_DEPTH - 0.010, PANEL_HEIGHT - 0.100)),
        material=powder_black,
        name="pull_lip",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.006, PANEL_DEPTH, PANEL_HEIGHT)),
        mass=1.9,
        origin=Origin(xyz=(-0.003, PANEL_DEPTH * 0.5, PANEL_HEIGHT * 0.5)),
    )

    case_center_z = FOOT_H + BODY_H * 0.5

    model.articulation(
        "chassis_to_front_intake",
        ArticulationType.FIXED,
        parent="chassis",
        child="front_intake",
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.002, FRONT_INTAKE_TOP_MOUNT_Z)),
    )
    model.articulation(
        "chassis_to_rear_io",
        ArticulationType.FIXED,
        parent="chassis",
        child="rear_io",
        origin=Origin(xyz=(0.0, -BODY_D * 0.5, case_center_z)),
    )
    model.articulation(
        "chassis_to_side_panel",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="side_panel",
        origin=Origin(xyz=(-BODY_W * 0.5, -PANEL_DEPTH * 0.5, FOOT_H + 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_contact("front_intake", "chassis")
    ctx.expect_aabb_overlap("front_intake", "chassis", axes="xz", min_overlap=0.16)

    ctx.expect_aabb_contact("rear_io", "chassis")
    ctx.expect_aabb_overlap("rear_io", "chassis", axes="xz", min_overlap=0.14)

    ctx.expect_aabb_contact("side_panel", "chassis")
    ctx.expect_aabb_overlap("side_panel", "chassis", axes="yz", min_overlap=0.20)
    ctx.expect_joint_motion_axis(
        "chassis_to_side_panel",
        "side_panel",
        world_axis="x",
        direction="negative",
        min_delta=0.05,
    )

    with ctx.pose(chassis_to_side_panel=1.20):
        ctx.expect_aabb_overlap("side_panel", "chassis", axes="yz", min_overlap=0.10)
        ctx.expect_aabb_overlap("side_panel", "chassis", axes="z", min_overlap=0.30)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
