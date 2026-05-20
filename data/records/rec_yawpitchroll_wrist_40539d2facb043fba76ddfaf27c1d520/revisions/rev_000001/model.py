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


def _tube_along_x(outer_radius: float, inner_radius: float, length: float, *, x_center: float):
    """CadQuery tube whose cylindrical axis is the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x_center - length / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_three_axis_wrist")

    aluminum = model.material("satin_aluminum", color=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("dark_anodized", color=(0.05, 0.055, 0.06, 1.0))
    steel = model.material("bearing_steel", color=(0.38, 0.40, 0.41, 1.0))
    cap_black = model.material("black_screw_heads", color=(0.01, 0.01, 0.012, 1.0))
    blue = model.material("blue_output_flange", color=(0.12, 0.22, 0.55, 1.0))
    orange = model.material("orange_index_mark", color=(1.0, 0.48, 0.08, 1.0))

    # Broad, very low root stage.  The dark top race gives the yaw stage a
    # visible bearing surface without making the root look as tall as the wrist.
    root = model.part("root_stage")
    root.visual(
        Box((0.36, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark,
        name="base_plate",
    )
    root.visual(
        Cylinder(radius=0.082, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
        material=steel,
        name="yaw_race",
    )
    for i, (x, y) in enumerate(
        ((-0.135, -0.085), (-0.135, 0.085), (0.135, -0.085), (0.135, 0.085))
    ):
        root.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(x, y, 0.0295)),
            material=cap_black,
            name=f"mount_screw_{i}",
        )

    # Compact yaw base: a turntable disk, short central hub, and two cheek
    # supports for the pitch axis.  It is intentionally much smaller in plan
    # than the root plate.
    yaw = model.part("yaw_base")
    yaw.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="turntable",
    )
    yaw.visual(
        Cylinder(radius=0.041, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=aluminum,
        name="yaw_hub",
    )
    for suffix, y in (("0", 0.047), ("1", -0.047)):
        yaw.visual(
            Box((0.055, 0.018, 0.058)),
            origin=Origin(xyz=(0.0, y, 0.067)),
            material=aluminum,
            name=f"pitch_lug_{suffix}",
        )
        yaw.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(
                xyz=(0.0, y + (0.010 if y > 0.0 else -0.010), 0.067),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"pitch_bearing_cap_{suffix}",
        )

    # Short pitch yoke.  Its child frame is exactly on the pitch trunnion
    # centerline, so the roll output visibly tips about the bearing axis.
    pitch = model.part("pitch_yoke")
    pitch.visual(
        Cylinder(radius=0.018, length=0.076),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pitch_trunnion",
    )
    pitch.visual(
        Box((0.052, 0.062, 0.034)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=aluminum,
        name="yoke_bridge",
    )
    for suffix, y in (("0", 0.040), ("1", -0.040)):
        pitch.visual(
            Box((0.076, 0.018, 0.048)),
            origin=Origin(xyz=(0.070, y, 0.0)),
            material=aluminum,
            name=f"roll_cheek_{suffix}",
        )
    pitch.visual(
        mesh_from_cadquery(
            _tube_along_x(0.032, 0.020, 0.024, x_center=0.094),
            "hollow_roll_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=steel,
        name="roll_collar",
    )

    # Small output roll flange with an asymmetric index tab so roll motion is
    # legible.  A slim shaft passes through the hollow collar; the blue flange
    # remains distinctly smaller than the yoke and yaw base.
    roll = model.part("roll_flange")
    roll.visual(
        Cylinder(radius=0.016, length=0.046),
        origin=Origin(xyz=(-0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="output_shaft",
    )
    roll.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bearing_shoulder",
    )
    roll.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="flange_disk",
    )
    roll.visual(
        Box((0.006, 0.012, 0.014)),
        origin=Origin(xyz=(0.017, 0.0, 0.042)),
        material=orange,
        name="index_tab",
    )
    for i, (y, z) in enumerate(((0.022, 0.022), (-0.022, 0.022), (-0.022, -0.022), (0.022, -0.022))):
        roll.visual(
            Cylinder(radius=0.0045, length=0.0025),
            origin=Origin(xyz=(0.015, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cap_black,
            name=f"flange_screw_{i}",
        )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=pitch,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch,
        child=roll,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.5, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_stage")
    yaw = object_model.get_part("yaw_base")
    pitch = object_model.get_part("pitch_yoke")
    roll = object_model.get_part("roll_flange")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    wrist_joints = (yaw_joint, pitch_joint, roll_joint)
    ctx.check(
        "three revolute wrist joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in wrist_joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_gap(
        yaw,
        root,
        axis="z",
        positive_elem="turntable",
        negative_elem="yaw_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw turntable seats on root race",
    )
    ctx.expect_gap(
        yaw,
        pitch,
        axis="y",
        positive_elem="pitch_lug_0",
        negative_elem="pitch_trunnion",
        max_gap=0.001,
        max_penetration=0.0,
        name="positive pitch lug meets trunnion",
    )
    ctx.expect_gap(
        pitch,
        yaw,
        axis="y",
        positive_elem="pitch_trunnion",
        negative_elem="pitch_lug_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="negative pitch lug meets trunnion",
    )
    ctx.expect_within(
        roll,
        pitch,
        axes="yz",
        inner_elem="output_shaft",
        outer_elem="roll_collar",
        margin=0.004,
        name="roll shaft is centered in collar",
    )
    ctx.expect_overlap(
        roll,
        pitch,
        axes="x",
        elem_a="output_shaft",
        elem_b="roll_collar",
        min_overlap=0.020,
        name="roll shaft retained through collar",
    )
    ctx.expect_contact(
        roll,
        pitch,
        elem_a="bearing_shoulder",
        elem_b="roll_collar",
        contact_tol=0.001,
        name="roll shoulder seats against collar",
    )

    root_aabb = ctx.part_world_aabb(root)
    yaw_aabb = ctx.part_world_aabb(yaw)
    roll_aabb = ctx.part_world_aabb(roll)
    if root_aabb is not None and yaw_aabb is not None and roll_aabb is not None:
        root_dx = root_aabb[1][0] - root_aabb[0][0]
        yaw_dx = yaw_aabb[1][0] - yaw_aabb[0][0]
        roll_dy = roll_aabb[1][1] - roll_aabb[0][1]
        ctx.check(
            "stages step down from root to output",
            root_dx > yaw_dx * 2.5 and yaw_dx > roll_dy * 1.25,
            details=f"root_dx={root_dx:.3f}, yaw_dx={yaw_dx:.3f}, roll_dy={roll_dy:.3f}",
        )
    else:
        ctx.fail("stages step down from root to output", "missing part AABB")

    rest_roll_pos = ctx.part_world_position(roll)
    with ctx.pose({pitch_joint: 0.55}):
        pitched_roll_pos = ctx.part_world_position(roll)
    ctx.check(
        "positive pitch raises output flange",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.04,
        details=f"rest={rest_roll_pos}, pitched={pitched_roll_pos}",
    )

    rest_tab = ctx.part_element_world_aabb(roll, elem="index_tab")
    with ctx.pose({roll_joint: 0.85}):
        rolled_tab = ctx.part_element_world_aabb(roll, elem="index_tab")
    if rest_tab is not None and rolled_tab is not None:
        rest_y = (rest_tab[0][1] + rest_tab[1][1]) / 2.0
        rolled_y = (rolled_tab[0][1] + rolled_tab[1][1]) / 2.0
        ctx.check(
            "roll joint spins flange index tab",
            abs(rolled_y - rest_y) > 0.020,
            details=f"rest_y={rest_y:.3f}, rolled_y={rolled_y:.3f}",
        )
    else:
        ctx.fail("roll joint spins flange index tab", "missing index-tab AABB")

    return ctx.report()


object_model = build_object_model()
