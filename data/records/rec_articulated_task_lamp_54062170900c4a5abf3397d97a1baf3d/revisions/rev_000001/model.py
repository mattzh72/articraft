from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PLATE_THICKNESS = 0.008
PLATE_WIDTH = 0.074
PLATE_HEIGHT = 0.150
PLATE_SCREW_Z = 0.043

BRACKET_BLOCK_END_X = 0.016
BRACKET_BLOCK_WIDTH = 0.028
BRACKET_BLOCK_HEIGHT = 0.048
BRACKET_HINGE_X = 0.030
BRACKET_CHEEK_LENGTH = BRACKET_HINGE_X - BRACKET_BLOCK_END_X
BRACKET_CHEEK_THICKNESS = 0.006
BRACKET_FORK_CENTER_Z = 0.008
BRACKET_KNUCKLE_RADIUS = 0.0095
BRACKET_KNUCKLE_LENGTH = 0.006

ARM_WIDTH = 0.016
ARM_THICKNESS = 0.010
ARM_ROOT_LUG_RADIUS = 0.0085
ARM_ROOT_LUG_LENGTH = 0.010
ARM_FORK_CENTER_Z = 0.008
ARM_TIP_KNUCKLE_RADIUS = 0.0090
ARM_TIP_KNUCKLE_LENGTH = 0.006
ARM_CLEVIS_REACH = 0.022
ARM_BRIDGE_THICKNESS = 0.006
ARM_BEAM_START_X = 0.006
ARM_0_LENGTH = 0.180
ARM_1_LENGTH = 0.155

TIP_YOKE_CENTER_Y = 0.0065
TIP_YOKE_EAR_RADIUS = 0.0070
TIP_YOKE_EAR_LENGTH = 0.005
TIP_YOKE_BRIDGE_WIDTH = 0.0050
TIP_YOKE_REACH = 0.018

SHADE_LENGTH = 0.086
SHADE_BACK_RADIUS = 0.022
SHADE_FRONT_RADIUS = 0.070
SHADE_WALL = 0.0024
SHADE_OFFSET_X = 0.010
SHADE_OFFSET_Z = -0.024

DIMMER_X = 0.071
DIMMER_Z = 0.0195
DIMMER_BOSS_RADIUS = 0.0070
DIMMER_BOSS_LENGTH = 0.010
DIMMER_SHAFT_RADIUS = 0.0026
DIMMER_SHAFT_LENGTH = 0.006


def _shade_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(SHADE_BACK_RADIUS)
        .workplane(offset=SHADE_LENGTH)
        .circle(SHADE_FRONT_RADIUS)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=SHADE_WALL)
        .circle(SHADE_BACK_RADIUS - SHADE_WALL)
        .workplane(offset=SHADE_LENGTH - SHADE_WALL)
        .circle(SHADE_FRONT_RADIUS - SHADE_WALL)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _arm_beam_length(total_length: float, tip_reach: float) -> float:
    return total_length - ARM_BEAM_START_X - tip_reach


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_task_lamp")

    model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("charcoal_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("steel", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("socket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("warm_glass", rgba=(0.96, 0.94, 0.88, 0.78))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(PLATE_THICKNESS / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="plate",
    )
    wall_bracket.visual(
        Box((BRACKET_BLOCK_END_X - PLATE_THICKNESS, BRACKET_BLOCK_WIDTH, BRACKET_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(
                PLATE_THICKNESS + (BRACKET_BLOCK_END_X - PLATE_THICKNESS) / 2.0,
                0.0,
                0.0,
            )
        ),
        material="powder_black",
        name="block",
    )
    for index, z_pos in enumerate((-PLATE_SCREW_Z, PLATE_SCREW_Z)):
        wall_bracket.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(PLATE_THICKNESS + 0.0015, 0.0, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material="steel",
            name=f"screw_{index}",
        )
    for name, z_pos in (("cheek_0", -BRACKET_FORK_CENTER_Z), ("cheek_1", BRACKET_FORK_CENTER_Z)):
        wall_bracket.visual(
            Box((BRACKET_CHEEK_LENGTH, ARM_WIDTH, BRACKET_CHEEK_THICKNESS)),
            origin=Origin(
                xyz=(BRACKET_BLOCK_END_X + BRACKET_CHEEK_LENGTH / 2.0, 0.0, z_pos),
            ),
            material="powder_black",
            name=name,
        )
    for name, z_pos in (("knuckle_0", -BRACKET_FORK_CENTER_Z), ("knuckle_1", BRACKET_FORK_CENTER_Z)):
        wall_bracket.visual(
            Cylinder(radius=BRACKET_KNUCKLE_RADIUS, length=BRACKET_KNUCKLE_LENGTH),
            origin=Origin(xyz=(BRACKET_HINGE_X, 0.0, z_pos)),
            material="powder_black",
            name=name,
        )

    arm_0 = model.part("arm_0")
    arm_0.visual(
        Cylinder(radius=ARM_ROOT_LUG_RADIUS, length=ARM_ROOT_LUG_LENGTH),
        origin=Origin(),
        material="charcoal_metal",
        name="root_lug",
    )
    arm_0.visual(
        Box((_arm_beam_length(ARM_0_LENGTH, ARM_CLEVIS_REACH), ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(
            xyz=(
                ARM_BEAM_START_X + _arm_beam_length(ARM_0_LENGTH, ARM_CLEVIS_REACH) / 2.0,
                0.0,
                0.0,
            )
        ),
        material="charcoal_metal",
        name="beam",
    )
    for name, z_pos in (("bridge_0", -ARM_FORK_CENTER_Z), ("bridge_1", ARM_FORK_CENTER_Z)):
        arm_0.visual(
            Box((ARM_CLEVIS_REACH, ARM_WIDTH, ARM_BRIDGE_THICKNESS)),
            origin=Origin(xyz=(ARM_0_LENGTH - ARM_CLEVIS_REACH / 2.0, 0.0, z_pos)),
            material="charcoal_metal",
            name=name,
        )
    for name, z_pos in (("tip_knuckle_0", -ARM_FORK_CENTER_Z), ("tip_knuckle_1", ARM_FORK_CENTER_Z)):
        arm_0.visual(
            Cylinder(radius=ARM_TIP_KNUCKLE_RADIUS, length=ARM_TIP_KNUCKLE_LENGTH),
            origin=Origin(xyz=(ARM_0_LENGTH, 0.0, z_pos)),
            material="charcoal_metal",
            name=name,
        )

    arm_1 = model.part("arm_1")
    arm_1.visual(
        Cylinder(radius=ARM_ROOT_LUG_RADIUS, length=ARM_ROOT_LUG_LENGTH),
        origin=Origin(),
        material="charcoal_metal",
        name="root_lug",
    )
    arm_1.visual(
        Box((_arm_beam_length(ARM_1_LENGTH, TIP_YOKE_REACH), ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(
            xyz=(
                ARM_BEAM_START_X + _arm_beam_length(ARM_1_LENGTH, TIP_YOKE_REACH) / 2.0,
                0.0,
                0.0,
            )
        ),
        material="charcoal_metal",
        name="beam",
    )
    for name, y_pos in (("yoke_bridge_0", -TIP_YOKE_CENTER_Y), ("yoke_bridge_1", TIP_YOKE_CENTER_Y)):
        arm_1.visual(
            Box((TIP_YOKE_REACH, TIP_YOKE_BRIDGE_WIDTH, ARM_THICKNESS)),
            origin=Origin(xyz=(ARM_1_LENGTH - TIP_YOKE_REACH / 2.0, y_pos, 0.0)),
            material="charcoal_metal",
            name=name,
        )
    for name, y_pos in (("yoke_0", -TIP_YOKE_CENTER_Y), ("yoke_1", TIP_YOKE_CENTER_Y)):
        arm_1.visual(
            Cylinder(radius=TIP_YOKE_EAR_RADIUS, length=TIP_YOKE_EAR_LENGTH),
            origin=Origin(xyz=(ARM_1_LENGTH, y_pos, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="charcoal_metal",
            name=name,
        )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="trunnion",
    )
    shade.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, -0.010)),
        material="charcoal_metal",
        name="strap",
    )
    shade.visual(
        mesh_from_cadquery(_shade_shell(), "task_lamp_shade_shell"),
        origin=Origin(xyz=(SHADE_OFFSET_X, 0.0, SHADE_OFFSET_Z)),
        material="powder_black",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.020, 0.0, SHADE_OFFSET_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="socket_black",
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.037, 0.0, SHADE_OFFSET_Z)),
        material="warm_glass",
        name="bulb",
    )
    shade.visual(
        Cylinder(radius=DIMMER_BOSS_RADIUS, length=DIMMER_BOSS_LENGTH),
        origin=Origin(
            xyz=(DIMMER_X - DIMMER_BOSS_LENGTH / 2.0, 0.0, DIMMER_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="powder_black",
        name="knob_boss",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=DIMMER_SHAFT_RADIUS, length=DIMMER_SHAFT_LENGTH),
        origin=Origin(xyz=(DIMMER_SHAFT_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.010,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=18, depth=0.0005),
                center=False,
            ),
            "task_lamp_dimmer_knob",
        ),
        origin=Origin(xyz=(DIMMER_SHAFT_LENGTH, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="knob_black",
        name="knob_cap",
    )

    model.articulation(
        "bracket_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=arm_0,
        origin=Origin(xyz=(BRACKET_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=18.0, velocity=1.8),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=arm_1,
        origin=Origin(xyz=(ARM_0_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.30, upper=2.30, effort=14.0, velocity=2.0),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=shade,
        origin=Origin(xyz=(ARM_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.95, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "dimmer",
        ArticulationType.CONTINUOUS,
        parent=shade,
        child=knob,
        origin=Origin(xyz=(DIMMER_X, 0.0, DIMMER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_bracket = object_model.get_part("wall_bracket")
    arm_1 = object_model.get_part("arm_1")
    shade = object_model.get_part("shade")
    knob = object_model.get_part("knob")

    bracket_hinge = object_model.get_articulation("bracket_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    shade_tilt = object_model.get_articulation("shade_tilt")
    dimmer = object_model.get_articulation("dimmer")

    ctx.expect_contact(
        knob,
        shade,
        elem_a="knob_shaft",
        elem_b="knob_boss",
        name="dimmer shaft seats against the shade boss",
    )
    ctx.expect_origin_gap(
        knob,
        shade,
        axis="x",
        min_gap=0.060,
        max_gap=0.080,
        name="dimmer knob sits on the front half of the shade",
    )
    ctx.expect_origin_gap(
        object_model.get_part("arm_0"),
        wall_bracket,
        axis="x",
        min_gap=0.024,
        max_gap=0.036,
        name="first arm hinge stands proud of the wall plate",
    )

    rest_elbow = ctx.part_world_position(arm_1)
    with ctx.pose({bracket_hinge: 0.80}):
        swung_elbow = ctx.part_world_position(arm_1)
    ctx.check(
        "wall bracket hinge swings the arm outward sideways",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[1] > rest_elbow[1] + 0.10,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )

    rest_shade = ctx.part_world_position(shade)
    with ctx.pose({elbow_hinge: 1.10}):
        folded_shade = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge folds the outer arm across the wall bracket sweep",
        rest_shade is not None
        and folded_shade is not None
        and folded_shade[1] > rest_shade[1] + 0.08
        and folded_shade[0] < rest_shade[0] - 0.03,
        details=f"rest={rest_shade}, folded={folded_shade}",
    )

    rest_knob = ctx.part_world_position(knob)
    with ctx.pose({shade_tilt: 0.55}):
        tilted_knob = ctx.part_world_position(knob)
    ctx.check(
        "shade tilt pitches the light head downward",
        rest_knob is not None
        and tilted_knob is not None
        and tilted_knob[2] < rest_knob[2] - 0.020,
        details=f"rest={rest_knob}, tilted={tilted_knob}",
    )

    ctx.check(
        "front dimmer uses continuous rotation",
        dimmer.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={dimmer.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
