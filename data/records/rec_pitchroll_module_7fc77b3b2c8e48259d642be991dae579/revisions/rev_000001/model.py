from __future__ import annotations

import math

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


PITCH_Z = 0.145
ROLL_JOINT_Y = 0.095


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pitch_roll_wrist")

    yoke_mat = Material("matte_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    cartridge_mat = Material("hard_anodized_bluegray", rgba=(0.18, 0.23, 0.28, 1.0))
    flange_mat = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_mat = Material("black_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.280, 0.200, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=yoke_mat,
        name="base_plate",
    )
    for idx, x in enumerate((-0.095, 0.095)):
        yoke.visual(
            Box((0.045, 0.140, 0.182)),
            origin=Origin(xyz=(x, 0.0, 0.124)),
            material=yoke_mat,
            name=f"side_cheek_{idx}",
        )
    yoke.visual(
        Box((0.238, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.076, 0.073)),
        material=yoke_mat,
        name="rear_bridge",
    )
    for idx, x in enumerate((-0.126, 0.126)):
        yoke.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(x, 0.0, PITCH_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=yoke_mat,
            name=f"bearing_cap_{idx}",
        )

    pitch_carrier = model.part("pitch_carrier")
    # Roll cartridge housing, centered on the pitch axis but projecting forward.
    pitch_carrier.visual(
        Cylinder(radius=0.043, length=0.140),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_mat,
        name="cartridge_shell",
    )
    pitch_carrier.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_mat,
        name="front_ring",
    )
    pitch_carrier.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, -0.061, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_mat,
        name="rear_cap",
    )
    # Side trunnion pin and collars define the pitch bearing line.
    pitch_carrier.visual(
        Cylinder(radius=0.014, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_mat,
        name="pitch_trunnion",
    )
    for idx, x in enumerate((-0.0645, 0.0645)):
        pitch_carrier.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cartridge_mat,
            name=f"trunnion_hub_{idx}",
        )

    flange = model.part("flange")
    flange.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flange_mat,
        name="flange_disk",
    )
    flange.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flange_mat,
        name="center_boss",
    )
    flange.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.039, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flange_mat,
        name="pilot_nose",
    )
    for idx, (x, z) in enumerate(((0.038, 0.0), (0.0, 0.038), (-0.038, 0.0), (0.0, -0.038))):
        flange.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(x, 0.0205, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=f"bolt_{idx}",
        )
    flange.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.023, 0.047)),
        material=dark_mat,
        name="index_mark",
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=pitch_carrier,
        origin=Origin(xyz=(0.0, 0.0, PITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_carrier,
        child=flange,
        origin=Origin(xyz=(0.0, ROLL_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yoke = object_model.get_part("yoke")
    pitch_carrier = object_model.get_part("pitch_carrier")
    flange = object_model.get_part("flange")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.check(
        "pitch joint uses side trunnion axis",
        tuple(round(v, 6) for v in pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "roll joint uses flange axis",
        tuple(round(v, 6) for v in roll_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll_joint.axis}",
    )
    ctx.expect_within(
        pitch_carrier,
        yoke,
        axes="yz",
        inner_elem="pitch_trunnion",
        margin=0.0,
        name="trunnion stays centered in yoke band",
    )
    ctx.expect_overlap(
        pitch_carrier,
        yoke,
        axes="x",
        elem_a="pitch_trunnion",
        min_overlap=0.14,
        name="trunnion spans both yoke cheeks",
    )
    ctx.expect_gap(
        flange,
        pitch_carrier,
        axis="y",
        positive_elem="flange_disk",
        negative_elem="front_ring",
        min_gap=0.0,
        max_gap=0.001,
        name="flange sits just proud of cartridge face",
    )

    closed_pos = ctx.part_world_position(flange)
    with ctx.pose({pitch_joint: 0.70}):
        pitched_pos = ctx.part_world_position(flange)
    ctx.check(
        "positive pitch lifts output flange",
        closed_pos is not None and pitched_pos is not None and pitched_pos[2] > closed_pos[2] + 0.04,
        details=f"closed={closed_pos}, pitched={pitched_pos}",
    )

    def _aabb_center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    index_rest = ctx.part_element_world_aabb(flange, elem="index_mark")
    with ctx.pose({roll_joint: 1.0}):
        index_rolled = ctx.part_element_world_aabb(flange, elem="index_mark")
    rest_x = _aabb_center_x(index_rest)
    rolled_x = _aabb_center_x(index_rolled)
    ctx.check(
        "roll joint rotates flange face marker",
        rest_x is not None and rolled_x is not None and rolled_x > rest_x + 0.025,
        details=f"rest_x={rest_x}, rolled_x={rolled_x}",
    )

    return ctx.report()


object_model = build_object_model()
