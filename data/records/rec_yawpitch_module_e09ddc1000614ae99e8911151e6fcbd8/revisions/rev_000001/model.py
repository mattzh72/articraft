from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


PEDESTAL_BASE = 0.18
PEDESTAL_HEIGHT = 0.145
YAW_FLANGE_RADIUS = 0.058
YAW_HEAD_HEIGHT = 0.144
PITCH_AXIS_Y = 0.018
PITCH_AXIS_Z = 0.110


def _pedestal_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(PEDESTAL_BASE, PEDESTAL_BASE, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.060, -0.060),
                (-0.060, 0.060),
                (0.060, -0.060),
                (0.060, 0.060),
            ]
        )
        .circle(0.008)
        .extrude(0.022, both=False)
    )

    tower = (
        cq.Workplane("XY")
        .workplane(offset=0.018)
        .rect(0.108, 0.108)
        .workplane(offset=0.102)
        .rect(0.078, 0.078)
        .loft(combine=True)
    )

    bearing_plinth = (
        cq.Workplane("XY")
        .workplane(offset=0.120)
        .circle(0.060)
        .extrude(PEDESTAL_HEIGHT - 0.120)
    )

    return base_plate.cut(mount_holes).union(tower).union(bearing_plinth)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_yaw_pitch_module")

    pedestal_finish = model.material("pedestal_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    head_finish = model.material("head_finish", rgba=(0.62, 0.65, 0.69, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.17, 0.18, 0.20, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal"),
        material=pedestal_finish,
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, PEDESTAL_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT / 2.0)),
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=YAW_FLANGE_RADIUS, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=head_finish,
        name="yaw_flange",
    )
    yaw_base.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=head_finish,
        name="yaw_drum",
    )
    yaw_base.visual(
        Box((0.074, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=head_finish,
        name="yaw_neck",
    )
    yaw_base.visual(
        Box((0.140, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.018, 0.079)),
        material=head_finish,
        name="pitch_bridge",
    )
    yaw_base.visual(
        Box((0.014, 0.052, 0.058)),
        origin=Origin(xyz=(-0.063, 0.018, 0.115)),
        material=head_finish,
        name="left_cheek",
    )
    yaw_base.visual(
        Box((0.014, 0.052, 0.058)),
        origin=Origin(xyz=(0.063, 0.018, 0.115)),
        material=head_finish,
        name="right_cheek",
    )
    yaw_base.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, YAW_HEAD_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.018, YAW_HEAD_HEIGHT / 2.0)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Box((0.104, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=yoke_finish,
        name="rear_hub",
    )
    pitch_yoke.visual(
        Box((0.018, 0.104, 0.060)),
        origin=Origin(xyz=(-0.043, 0.060, 0.035)),
        material=yoke_finish,
        name="left_arm",
    )
    pitch_yoke.visual(
        Box((0.018, 0.104, 0.060)),
        origin=Origin(xyz=(0.043, 0.060, 0.035)),
        material=yoke_finish,
        name="right_arm",
    )
    pitch_yoke.visual(
        Box((0.086, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.103, 0.035)),
        material=yoke_finish,
        name="front_crossbar",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(-0.054, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=yoke_finish,
        name="left_trunnion",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=yoke_finish,
        name="right_trunnion",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.148, 0.130, 0.070)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
    )

    model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, PITCH_AXIS_Y, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.55,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        yaw_base,
        pedestal,
        name="yaw base is seated on the pedestal thrust face",
    )
    ctx.expect_contact(
        pitch_yoke,
        yaw_base,
        name="pitch yoke is carried by the yaw-base trunnion supports",
    )

    yaw_rest = ctx.part_world_position(pitch_yoke)
    with ctx.pose({yaw_joint: 0.85}):
        yaw_turned = ctx.part_world_position(pitch_yoke)
    ctx.check(
        "yaw joint swings the carried yoke around the vertical axis",
        yaw_rest is not None
        and yaw_turned is not None
        and abs(yaw_turned[0] - yaw_rest[0]) > 0.010
        and abs(yaw_turned[1] - yaw_rest[1]) > 0.004,
        details=f"rest={yaw_rest}, turned={yaw_turned}",
    )

    pitch_rest_aabb = ctx.part_world_aabb(pitch_yoke)
    with ctx.pose({pitch_joint: 0.70}):
        pitch_up_aabb = ctx.part_world_aabb(pitch_yoke)
    ctx.check(
        "positive pitch lifts the forward yoke structure upward",
        pitch_rest_aabb is not None
        and pitch_up_aabb is not None
        and pitch_up_aabb[1][2] > pitch_rest_aabb[1][2] + 0.020,
        details=f"rest={pitch_rest_aabb}, pitched={pitch_up_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
