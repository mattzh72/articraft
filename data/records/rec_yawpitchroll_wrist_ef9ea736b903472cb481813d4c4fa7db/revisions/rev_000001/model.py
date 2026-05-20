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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_yoke_spindle_wrist")

    cast_iron = Material("mat_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_steel = Material("mat_dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    brushed = Material("mat_brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bearing_black = Material("mat_bearing_shadow", rgba=(0.02, 0.022, 0.025, 1.0))
    faceplate_mat = Material("mat_faceplate", rgba=(0.46, 0.48, 0.50, 1.0))
    mark_mat = Material("mat_index_mark", rgba=(0.02, 0.025, 0.03, 1.0))

    cyl_x = (0.0, math.pi / 2.0, 0.0)
    cyl_y = (-math.pi / 2.0, 0.0, 0.0)

    base = model.part("base")
    base.visual(
        Box((0.62, 0.50, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="ground_plate",
    )
    base.visual(
        Cylinder(radius=0.250, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="fixed_flange",
    )
    base.visual(
        Cylinder(radius=0.180, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=cast_iron,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
        material=brushed,
        name="yaw_bearing_stator",
    )
    for i, (x, y) in enumerate(
        ((0.245, 0.185), (-0.245, 0.185), (-0.245, -0.185), (0.245, -0.185))
    ):
        base.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(xyz=(x, y, 0.047)),
            material=brushed,
            name=f"mount_bolt_{i}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.190, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="turntable_disk",
    )
    yaw_stage.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed,
        name="rotating_race",
    )
    yaw_stage.visual(
        Box((0.140, 0.500, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=cast_iron,
        name="fork_foot",
    )
    yaw_stage.visual(
        Box((0.120, 0.120, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=cast_iron,
        name="center_pedestal",
    )
    for suffix, y in (("0", 0.220), ("1", -0.220)):
        yaw_stage.visual(
            Box((0.095, 0.060, 0.370)),
            origin=Origin(xyz=(0.0, y, 0.250)),
            material=cast_iron,
            name=f"pitch_cheek_{suffix}",
        )
        yaw_stage.visual(
            Cylinder(radius=0.072, length=0.060),
            origin=Origin(xyz=(0.0, y, 0.430), rpy=cyl_y),
            material=brushed,
            name=f"bearing_cap_{suffix}",
        )
        yaw_stage.visual(
            mesh_from_geometry(TorusGeometry(radius=0.047, tube=0.004), f"bore_shadow_{suffix}"),
            origin=Origin(xyz=(0.0, 0.190 if y > 0.0 else -0.190, 0.430), rpy=cyl_y),
            material=bearing_black,
            name=f"bore_shadow_{suffix}",
        )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        Cylinder(radius=0.035, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=brushed,
        name="trunnion_shaft",
    )
    pitch_frame.visual(
        Box((0.080, 0.280, 0.160)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bridge",
    )
    for suffix, y in (("0", 0.130), ("1", -0.130)):
        pitch_frame.visual(
            Box((0.250, 0.035, 0.100)),
            origin=Origin(xyz=(0.110, y, 0.0)),
            material=dark_steel,
            name=f"side_arm_{suffix}",
        )
    pitch_frame.visual(
        Box((0.180, 0.100, 0.100)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=dark_steel,
        name="hub_saddle",
    )
    pitch_frame.visual(
        Cylinder(radius=0.075, length=0.130),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=cyl_x),
        material=brushed,
        name="bearing_housing",
    )
    pitch_frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.050, tube=0.006), "roll_bore_shadow"),
        origin=Origin(xyz=(0.165, 0.0, 0.0), rpy=cyl_x),
        material=bearing_black,
        name="roll_bore_shadow",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.030, length=0.130),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=cyl_x),
        material=brushed,
        name="spindle_shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=cyl_x),
        material=brushed,
        name="center_boss",
    )
    roll_spindle.visual(
        Cylinder(radius=0.135, length=0.035),
        origin=Origin(xyz=(0.1475, 0.0, 0.0), rpy=cyl_x),
        material=faceplate_mat,
        name="faceplate_disk",
    )
    roll_spindle.visual(
        Box((0.006, 0.018, 0.052)),
        origin=Origin(xyz=(0.168, 0.0, 0.068)),
        material=mark_mat,
        name="index_mark",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.expect_gap(
        yaw_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="turntable_disk",
        negative_elem="yaw_bearing_stator",
        name="yaw turntable sits on bearing stator",
    )
    ctx.expect_contact(
        yaw_stage,
        pitch_frame,
        elem_a="bearing_cap_0",
        elem_b="trunnion_shaft",
        contact_tol=0.001,
        name="pitch trunnion is captured by first bearing cap",
    )
    ctx.expect_contact(
        yaw_stage,
        pitch_frame,
        elem_a="bearing_cap_1",
        elem_b="trunnion_shaft",
        contact_tol=0.001,
        name="pitch trunnion is captured by second bearing cap",
    )
    ctx.expect_contact(
        pitch_frame,
        roll_spindle,
        elem_a="bearing_housing",
        elem_b="spindle_shaft",
        contact_tol=0.001,
        name="roll spindle exits the pitch-frame bearing",
    )

    face_at_rest = ctx.part_element_world_aabb(roll_spindle, elem="faceplate_disk")
    with ctx.pose({pitch: 0.65}):
        face_pitch_up = ctx.part_element_world_aabb(roll_spindle, elem="faceplate_disk")
    ctx.check(
        "positive pitch raises the spindle nose",
        face_at_rest is not None
        and face_pitch_up is not None
        and (face_pitch_up[0][2] + face_pitch_up[1][2]) > (face_at_rest[0][2] + face_at_rest[1][2]) + 0.030,
        details=f"rest={face_at_rest}, pitched={face_pitch_up}",
    )

    mark_at_rest = ctx.part_element_world_aabb(roll_spindle, elem="index_mark")
    with ctx.pose({roll: math.pi / 2.0}):
        mark_rolled = ctx.part_element_world_aabb(roll_spindle, elem="index_mark")
    ctx.check(
        "roll joint rotates the faceplate index mark",
        mark_at_rest is not None
        and mark_rolled is not None
        and abs(
            ((mark_at_rest[0][1] + mark_at_rest[1][1]) / 2.0)
            - ((mark_rolled[0][1] + mark_rolled[1][1]) / 2.0)
        )
        > 0.040,
        details=f"rest={mark_at_rest}, rolled={mark_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
