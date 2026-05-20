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

BASE_LENGTH = 0.165
BASE_WIDTH = 0.050
BASE_TOP_Z = 0.012
MAGAZINE_HINGE_X = -0.066
MAGAZINE_HINGE_Z = 0.025
MAGAZINE_LENGTH = 0.150
MAGAZINE_WIDTH = 0.028
ARM_WIDTH = 0.034


def _extrude_xz_profile(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )


def _ycylinder(length: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _make_base_shell_shape() -> cq.Workplane:
    body = _extrude_xz_profile(
        [
            (-0.082, 0.0),
            (-0.082, 0.004),
            (-0.068, 0.008),
            (-0.036, 0.0115),
            (0.050, 0.0120),
            (0.072, 0.0105),
            (0.082, 0.0065),
            (0.082, 0.0),
        ],
        BASE_WIDTH,
    )

    nose_pad = (
        cq.Workplane("XY")
        .box(0.036, 0.034, 0.003)
        .translate((0.053, 0.0, BASE_TOP_Z + 0.0012))
    )

    return body.union(nose_pad)


def _make_hinge_tower_shape() -> cq.Workplane:
    left_cheek = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.019)
        .translate((MAGAZINE_HINGE_X - 0.008, -0.0155, 0.0175))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.019)
        .translate((MAGAZINE_HINGE_X - 0.008, 0.0155, 0.0175))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.035, 0.008)
        .translate((MAGAZINE_HINGE_X - 0.010, 0.0, 0.012))
    )
    tower_foot = (
        cq.Workplane("XY")
        .box(0.010, 0.016, 0.006)
        .translate((MAGAZINE_HINGE_X - 0.012, 0.0, 0.010))
    )
    left_boss = _ycylinder(0.0035, 0.0045).translate((MAGAZINE_HINGE_X - 0.0073, -0.01475, MAGAZINE_HINGE_Z))
    right_boss = _ycylinder(0.0035, 0.0045).translate((MAGAZINE_HINGE_X - 0.0073, 0.01475, MAGAZINE_HINGE_Z))

    return left_cheek.union(right_cheek).union(rear_bridge).union(tower_foot).union(left_boss).union(right_boss)


def _make_magazine_shape() -> cq.Workplane:
    shell = _extrude_xz_profile(
        [
            (0.000, -0.0045),
            (0.008, 0.0040),
            (0.022, 0.0070),
            (0.106, 0.0075),
            (0.130, 0.0065),
            (0.148, 0.0025),
            (0.150, -0.0010),
            (0.145, -0.0062),
            (0.034, -0.0070),
            (0.008, -0.0062),
        ],
        MAGAZINE_WIDTH,
    )
    center_relief = (
        cq.Workplane("XY")
        .box(0.024, 0.016, 0.014)
        .translate((0.009, 0.0, 0.0005))
    )
    left_rib = (
        cq.Workplane("XY")
        .box(0.012, 0.007, 0.010)
        .translate((0.008, -0.0105, 0.001))
    )
    right_rib = (
        cq.Workplane("XY")
        .box(0.012, 0.007, 0.010)
        .translate((0.008, 0.0105, 0.001))
    )
    left_knuckle = _ycylinder(0.0065, 0.0048).translate((0.002, -0.0095, 0.0))
    right_knuckle = _ycylinder(0.0065, 0.0048).translate((0.002, 0.0095, 0.0))

    return (
        shell.cut(center_relief)
        .union(left_rib)
        .union(right_rib)
        .union(left_knuckle)
        .union(right_knuckle)
    )


def _make_arm_shape() -> cq.Workplane:
    shell = _extrude_xz_profile(
        [
            (0.000, -0.0012),
            (0.006, 0.0018),
            (0.022, 0.0042),
            (0.092, 0.0056),
            (0.122, 0.0052),
            (0.136, 0.0030),
            (0.139, 0.0003),
            (0.136, -0.0018),
            (0.018, -0.0022),
            (0.004, -0.0015),
        ],
        ARM_WIDTH,
    )
    front_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.004)
        .translate((0.121, 0.0, 0.0005))
    )
    rear_cap = (
        cq.Workplane("XY")
        .box(0.010, ARM_WIDTH, 0.005)
        .translate((0.004, 0.0, 0.001))
    )
    return shell.union(front_pad).union(rear_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    die_cast = model.material("die_cast", rgba=(0.71, 0.73, 0.76, 1.0))
    magazine_metal = model.material("magazine_metal", rgba=(0.62, 0.65, 0.69, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shell_shape(), "base_shell"),
        material=die_cast,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_make_hinge_tower_shape(), "hinge_tower"),
        material=die_cast,
        name="hinge_tower",
    )
    base.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(MAGAZINE_HINGE_X - 0.010, 0.0, 0.012)),
        material=die_cast,
        name="tower_bridge",
    )
    base.visual(
        Box((0.0024, 0.009, 0.008)),
        origin=Origin(xyz=(MAGAZINE_HINGE_X - 0.004, -0.0095, 0.025)),
        material=die_cast,
        name="hinge_pad_0",
    )
    base.visual(
        Box((0.0024, 0.009, 0.008)),
        origin=Origin(xyz=(MAGAZINE_HINGE_X - 0.004, 0.0095, 0.025)),
        material=die_cast,
        name="hinge_pad_1",
    )

    magazine = model.part("magazine")
    magazine.visual(
        mesh_from_cadquery(_make_magazine_shape(), "magazine"),
        material=magazine_metal,
        name="magazine_shell",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "arm"),
        material=die_cast,
        name="arm_shell",
    )

    anvil = model.part("anvil")
    anvil.visual(
        Box((0.018, 0.012, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0012)),
        material=magazine_metal,
        name="anvil_plate",
    )
    anvil.visual(
        Cylinder(radius=0.0032, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material=magazine_metal,
        name="anvil_pivot",
    )

    model.articulation(
        "base_to_magazine",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(xyz=(MAGAZINE_HINGE_X, 0.0, MAGAZINE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    model.articulation(
        "magazine_to_arm",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=arm,
        origin=Origin(xyz=(0.008, 0.0, 0.0085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=-0.30,
            upper=0.55,
        ),
    )

    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(0.050, 0.0, 0.0147)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    arm = object_model.get_part("arm")
    anvil = object_model.get_part("anvil")
    magazine_hinge = object_model.get_articulation("base_to_magazine")
    arm_hinge = object_model.get_articulation("magazine_to_arm")
    anvil_hinge = object_model.get_articulation("base_to_anvil")

    ctx.expect_overlap(
        magazine,
        base,
        axes="x",
        elem_a="magazine_shell",
        elem_b="base_shell",
        min_overlap=0.10,
        name="magazine spans the desk footprint",
    )

    with ctx.pose({magazine_hinge: 0.0}):
        ctx.expect_gap(
            magazine,
            base,
            axis="z",
            positive_elem="magazine_shell",
            negative_elem="base_shell",
            max_gap=0.020,
            max_penetration=0.0,
            name="closed magazine sits just above the base",
        )

    limits = magazine_hinge.motion_limits
    closed_mag = ctx.part_element_world_aabb(magazine, elem="magazine_shell")
    if limits is not None and limits.upper is not None:
        with ctx.pose({magazine_hinge: limits.upper}):
            opened_mag = ctx.part_element_world_aabb(magazine, elem="magazine_shell")
        ctx.check(
            "magazine nose lifts to reload",
            closed_mag is not None
            and opened_mag is not None
            and opened_mag[1][2] > closed_mag[1][2] + 0.055,
            details=f"closed={closed_mag}, opened={opened_mag}",
        )

    ctx.expect_overlap(
        arm,
        magazine,
        axes="x",
        elem_a="arm_shell",
        elem_b="magazine_shell",
        min_overlap=0.10,
        name="arm covers most of the magazine length",
    )

    arm_rest = ctx.part_element_world_aabb(arm, elem="arm_shell")
    if arm_hinge.motion_limits is not None:
        with ctx.pose({arm_hinge: arm_hinge.motion_limits.lower}):
            arm_pressed = ctx.part_element_world_aabb(arm, elem="arm_shell")
        with ctx.pose({arm_hinge: arm_hinge.motion_limits.upper}):
            arm_lifted = ctx.part_element_world_aabb(arm, elem="arm_shell")
        ctx.check(
            "arm presses downward",
            arm_rest is not None
            and arm_pressed is not None
            and arm_pressed[0][2] < arm_rest[0][2] - 0.004,
            details=f"rest={arm_rest}, pressed={arm_pressed}",
        )
        ctx.check(
            "arm also lifts upward",
            arm_rest is not None
            and arm_lifted is not None
            and arm_lifted[1][2] > arm_rest[1][2] + 0.018,
            details=f"rest={arm_rest}, lifted={arm_lifted}",
        )

    ctx.expect_overlap(
        anvil,
        base,
        axes="xy",
        elem_a="anvil_plate",
        elem_b="base_shell",
        min_overlap=0.010,
        name="anvil stays within the nose pad footprint",
    )

    with ctx.pose({anvil_hinge: math.pi}):
        ctx.expect_overlap(
            anvil,
            base,
            axes="xy",
            elem_a="anvil_plate",
            elem_b="base_shell",
            min_overlap=0.010,
            name="anvil remains seated after reversal",
        )

    return ctx.report()


object_model = build_object_model()
