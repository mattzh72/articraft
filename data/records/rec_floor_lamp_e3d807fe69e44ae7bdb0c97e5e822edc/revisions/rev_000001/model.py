from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_SIZE = (0.42, 0.28, 0.045)
COLUMN_X = -0.13
COLUMN_RADIUS = 0.026
COLUMN_HEIGHT = 1.25
ARM_LENGTH = 0.62
ARM_Z = 0.035
SHOULDER_Z = BASE_SIZE[2] - 0.001 + COLUMN_HEIGHT + 0.035


def _shade_shell() -> cq.Workplane:
    """Thin conical frustum with an open bottom, authored in shade-local space."""
    outer = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.045))
        .circle(0.055)
        .workplane(offset=-0.175)
        .circle(0.150)
        .loft(combine=True, ruled=True)
    )
    inner_cutter = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.052))
        .circle(0.046)
        .workplane(offset=-0.188)
        .circle(0.150)
        .loft(combine=True, ruled=True)
    )
    return outer.cut(inner_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("warm_cream", rgba=(0.92, 0.86, 0.72, 1.0))
    model.material("warm_light", rgba=(1.0, 0.78, 0.34, 0.85))

    base = model.part("base_column")
    base.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
        material="matte_black",
        name="rectangular_base",
    )
    base.visual(
        Box((0.105, 0.085, 0.018)),
        origin=Origin(xyz=(COLUMN_X, 0.0, BASE_SIZE[2] + 0.008)),
        material="dark_metal",
        name="column_plinth",
    )
    base.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(
            xyz=(COLUMN_X, 0.0, BASE_SIZE[2] - 0.001 + COLUMN_HEIGHT / 2.0)
        ),
        material="brushed_steel",
        name="fixed_column",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.037),
        origin=Origin(xyz=(COLUMN_X, 0.0, SHOULDER_Z - 0.0185)),
        material="dark_metal",
        name="shoulder_socket",
    )
    for ix, x in enumerate((-0.145, 0.145)):
        for iy, y in enumerate((-0.085, 0.085)):
            base.visual(
                Cylinder(radius=0.010, length=0.003),
                origin=Origin(xyz=(x, y, BASE_SIZE[2] + 0.0015)),
                material="brushed_steel",
                name=f"base_screw_{ix}_{iy}",
            )

    arm = model.part("swing_arm")
    arm.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="dark_metal",
        name="shoulder_turntable",
    )
    arm.visual(
        Cylinder(radius=0.018, length=ARM_LENGTH - 0.050),
        origin=Origin(
            xyz=((ARM_LENGTH - 0.050) / 2.0, 0.0, ARM_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="brushed_steel",
        name="arm_tube",
    )
    arm.visual(
        Box((0.050, 0.088, 0.030)),
        origin=Origin(xyz=(ARM_LENGTH - 0.055, 0.0, ARM_Z)),
        material="dark_metal",
        name="clevis_bridge",
    )
    arm.visual(
        Box((0.050, 0.012, 0.080)),
        origin=Origin(xyz=(ARM_LENGTH - 0.010, -0.039, ARM_Z)),
        material="dark_metal",
        name="clevis_cheek_0",
    )
    arm.visual(
        Box((0.050, 0.012, 0.080)),
        origin=Origin(xyz=(ARM_LENGTH - 0.010, 0.039, ARM_Z)),
        material="dark_metal",
        name="clevis_cheek_1",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.022, length=0.066),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_metal",
        name="shade_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material="dark_metal",
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material="dark_metal",
        name="top_cap",
    )
    shade.visual(
        mesh_from_cadquery(_shade_shell(), "conical_shade_shell"),
        material="warm_cream",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material="dark_metal",
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material="warm_light",
        name="glowing_bulb",
    )

    model.articulation(
        "shoulder_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(COLUMN_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=18.0, velocity=1.0),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, ARM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.75, effort=6.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_column")
    arm = object_model.get_part("swing_arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("shoulder_swing")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.check(
        "two named revolute joints",
        len(object_model.articulations) == 2
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and shade_tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="shoulder_turntable",
        negative_elem="shoulder_socket",
        max_gap=0.002,
        max_penetration=0.0,
        name="arm turntable seats on column socket",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="shoulder_turntable",
        elem_b="shoulder_socket",
        min_overlap=0.085,
        name="shoulder pivot is centered over fixed column",
    )
    ctx.expect_overlap(
        shade,
        arm,
        axes="xz",
        elem_a="shade_trunnion",
        elem_b="clevis_cheek_0",
        min_overlap=0.030,
        name="shade trunnion crosses first clevis cheek",
    )
    ctx.expect_overlap(
        shade,
        arm,
        axes="xz",
        elem_a="shade_trunnion",
        elem_b="clevis_cheek_1",
        min_overlap=0.030,
        name="shade trunnion crosses second clevis cheek",
    )
    ctx.expect_contact(
        arm,
        shade,
        elem_a="clevis_cheek_1",
        elem_b="shade_trunnion",
        contact_tol=0.001,
        name="upper clevis cheek supports shade trunnion",
    )
    ctx.expect_contact(
        arm,
        shade,
        elem_a="clevis_cheek_0",
        elem_b="shade_trunnion",
        contact_tol=0.001,
        name="lower clevis cheek supports shade trunnion",
    )

    with ctx.pose({shoulder: 0.0, shade_tilt: 0.0}):
        rest_shade_origin = ctx.part_world_position(shade)
        rest_bulb_aabb = ctx.part_element_world_aabb(shade, elem="glowing_bulb")

    with ctx.pose({shoulder: 0.75, shade_tilt: 0.0}):
        swung_shade_origin = ctx.part_world_position(shade)

    ctx.check(
        "shoulder swing carries shade sideways",
        rest_shade_origin is not None
        and swung_shade_origin is not None
        and swung_shade_origin[1] > rest_shade_origin[1] + 0.35,
        details=f"rest={rest_shade_origin}, swung={swung_shade_origin}",
    )

    with ctx.pose({shoulder: 0.0, shade_tilt: 0.55}):
        tilted_bulb_aabb = ctx.part_element_world_aabb(shade, elem="glowing_bulb")

    if rest_bulb_aabb is not None and tilted_bulb_aabb is not None:
        rest_bulb_x = (rest_bulb_aabb[0][0] + rest_bulb_aabb[1][0]) / 2.0
        tilted_bulb_x = (tilted_bulb_aabb[0][0] + tilted_bulb_aabb[1][0]) / 2.0
    else:
        rest_bulb_x = None
        tilted_bulb_x = None
    ctx.check(
        "shade tilt aims lit opening outward",
        rest_bulb_x is not None
        and tilted_bulb_x is not None
        and tilted_bulb_x > rest_bulb_x + 0.045,
        details=f"rest_bulb_x={rest_bulb_x}, tilted_bulb_x={tilted_bulb_x}",
    )

    return ctx.report()


object_model = build_object_model()
