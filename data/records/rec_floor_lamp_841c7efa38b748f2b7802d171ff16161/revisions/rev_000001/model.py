from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHOULDER_Z = 1.32
UPPER_ARM_LENGTH = 0.54
FOREARM_LENGTH = 0.46


def _shade_shell():
    """Thin, open conical frustum with annular top and bottom rims."""
    height = 0.18
    top_outer = 0.058
    bottom_outer = 0.142
    wall = 0.006
    z_top = -0.060
    z_bottom = z_top - height

    return LatheGeometry.from_shell_profiles(
        outer_profile=[(top_outer, z_top), (bottom_outer, z_bottom)],
        inner_profile=[(top_outer - wall, z_top), (bottom_outer - wall, z_bottom)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    model.material("dark_metal", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("brushed_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    model.material("warm_cream", rgba=(0.92, 0.86, 0.70, 1.0))
    model.material("warm_glass", rgba=(1.0, 0.86, 0.45, 0.78))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.185, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material="matte_black",
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.029, length=1.245),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material="dark_metal",
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.044, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material="brushed_steel",
        name="base_collar",
    )
    stand.visual(
        Box((0.080, 0.140, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - 0.046)),
        material="dark_metal",
        name="top_crosshead",
    )
    for y, yoke_name in ((-0.050, "shoulder_yoke_0"), (0.050, "shoulder_yoke_1")):
        stand.visual(
            Box((0.044, 0.014, 0.092)),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z)),
            material="dark_metal",
            name=yoke_name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.025, length=0.086),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="shoulder_hub",
    )
    for y in (-0.022, 0.022):
        upper_arm.visual(
            Cylinder(radius=0.008, length=UPPER_ARM_LENGTH - 0.025),
            origin=Origin(
                xyz=((UPPER_ARM_LENGTH + 0.025) / 2.0, y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="brushed_steel",
            name=f"upper_tube_{0 if y < 0 else 1}",
        )
    upper_arm.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="elbow_inner_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="shoulder_knob",
    )

    forearm = model.part("forearm")
    for y, elbow_fork_name, tube_name, shade_fork_name in (
        (-0.041, "elbow_fork_0", "forearm_tube_0", "shade_fork_0"),
        (0.041, "elbow_fork_1", "forearm_tube_1", "shade_fork_1"),
    ):
        forearm.visual(
            Box((0.046, 0.012, 0.054)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material="brushed_steel",
            name=elbow_fork_name,
        )
        forearm.visual(
            Cylinder(radius=0.0065, length=FOREARM_LENGTH - 0.020),
            origin=Origin(
                xyz=(FOREARM_LENGTH / 2.0, y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="brushed_steel",
            name=tube_name,
        )
        forearm.visual(
            Box((0.046, 0.012, 0.052)),
            origin=Origin(xyz=(FOREARM_LENGTH, y, 0.0)),
            material="brushed_steel",
            name=shade_fork_name,
        )
    for x in (0.080, FOREARM_LENGTH - 0.065):
        forearm.visual(
            Cylinder(radius=0.007, length=0.116),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="brushed_steel",
            name=f"forearm_crossbar_{0 if x < 0.2 else 1}",
        )
    forearm.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.054, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="matte_black",
        name="elbow_knob",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="tilt_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material="brushed_steel",
        name="drop_stem",
    )
    shade.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material="brushed_steel",
        name="lamp_socket",
    )
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        shade.visual(
            Box((0.052, 0.007, 0.006)),
            origin=Origin(xyz=(0.038 * math.cos(yaw), 0.038 * math.sin(yaw), -0.063), rpy=(0.0, 0.0, yaw)),
            material="brushed_steel",
            name=f"shade_spoke_{i}",
        )
    shade.visual(
        mesh_from_geometry(_shade_shell(), "conical_shade"),
        material="warm_cream",
        name="conical_shade",
    )
    shade.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material="warm_glass",
        name="bulb",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.65, upper=1.05),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.4, lower=-1.35, upper=1.45),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=shade,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.check(
        "has three revolute lamp joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, shade_tilt)),
        details="Expected shoulder, elbow, and shade tilt to be revolute joints.",
    )
    ctx.expect_overlap(
        upper_arm,
        stand,
        axes="y",
        min_overlap=0.040,
        elem_a="shoulder_hub",
        elem_b="top_crosshead",
        name="shoulder hub is captured by column head width",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a="elbow_fork_0",
        elem_b="elbow_inner_hub",
        contact_tol=0.001,
        name="elbow fork straddles upper arm hub",
    )
    ctx.expect_contact(
        shade,
        forearm,
        elem_a="tilt_trunnion",
        elem_b="shade_fork_0",
        contact_tol=0.001,
        name="shade trunnion sits between fork cheeks",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.75}):
        raised_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder raises the arm",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.20,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({elbow: 0.80}):
        bent_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow moves the lamp head",
        rest_shade_pos is not None
        and bent_shade_pos is not None
        and bent_shade_pos[2] > rest_shade_pos[2] + 0.12,
        details=f"rest={rest_shade_pos}, bent={bent_shade_pos}",
    )

    rest_shade_box = ctx.part_element_world_aabb(shade, elem="bulb")
    with ctx.pose({shade_tilt: 0.55}):
        tilted_shade_box = ctx.part_element_world_aabb(shade, elem="bulb")
    if rest_shade_box is not None and tilted_shade_box is not None:
        rest_center = tuple((rest_shade_box[0][i] + rest_shade_box[1][i]) / 2.0 for i in range(3))
        tilted_center = tuple((tilted_shade_box[0][i] + tilted_shade_box[1][i]) / 2.0 for i in range(3))
    else:
        rest_center = tilted_center = None
    ctx.check(
        "shade tilt rotates the conical shade",
        rest_center is not None
        and tilted_center is not None
        and abs(tilted_center[0] - rest_center[0]) > 0.045,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


object_model = build_object_model()
