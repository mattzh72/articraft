from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_three_axis_wrist")

    dark_cast = model.material("dark_cast_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    warm_grey = model.material("warm_grey_paint", rgba=(0.44, 0.46, 0.47, 1.0))
    blue = model.material("service_blue", rgba=(0.06, 0.22, 0.45, 1.0))
    graphite = model.material("graphite_black", rgba=(0.02, 0.025, 0.03, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.65, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))

    cyl_x = (0.0, math.pi / 2.0, 0.0)
    cyl_y = (math.pi / 2.0, 0.0, 0.0)

    body = model.part("body")
    body.visual(
        Box((0.56, 0.42, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_cast,
        name="grounded_base",
    )
    body.visual(
        Box((0.36, 0.28, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=warm_grey,
        name="raised_housing",
    )
    body.visual(
        Cylinder(radius=0.170, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=warm_grey,
        name="upper_bearing",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.135, 0.012, radial_segments=48, tubular_segments=12), "yaw_bearing_ring_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=steel,
        name="yaw_bearing_ring",
    )
    body.visual(
        Box((0.42, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.220, 0.095)),
        material=rubber,
        name="rear_cable_gland",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.150, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=blue,
        name="rotor_disk",
    )
    yaw_stage.visual(
        Cylinder(radius=0.104, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_cast,
        name="rotor_neck",
    )
    yaw_stage.visual(
        Box((0.210, 0.205, 0.080)),
        origin=Origin(xyz=(0.075, 0.0, 0.105)),
        material=blue,
        name="saddle_block",
    )
    yaw_stage.visual(
        Box((0.185, 0.305, 0.045)),
        origin=Origin(xyz=(0.105, 0.0, 0.135)),
        material=blue,
        name="cheek_bridge",
    )
    for suffix, y in (("pos", 0.134), ("neg", -0.134)):
        yaw_stage.visual(
            Box((0.080, 0.036, 0.075)),
            origin=Origin(xyz=(0.165, y, 0.145)),
            material=blue,
            name=f"pitch_cheek_{suffix}_lower",
        )
        yaw_stage.visual(
            Box((0.080, 0.036, 0.075)),
            origin=Origin(xyz=(0.165, y, 0.305)),
            material=blue,
            name=f"pitch_cheek_{suffix}_upper",
        )
    yaw_stage.visual(
        mesh_from_geometry(
            TorusGeometry(0.047, 0.012, radial_segments=36, tubular_segments=10).rotate_x(math.pi / 2.0),
            "pitch_bearing_pos_mesh",
        ),
        origin=Origin(xyz=(0.165, 0.134, 0.225)),
        material=steel,
        name="pitch_bearing_pos",
    )
    yaw_stage.visual(
        mesh_from_geometry(
            TorusGeometry(0.047, 0.012, radial_segments=36, tubular_segments=10).rotate_x(math.pi / 2.0),
            "pitch_bearing_neg_mesh",
        ),
        origin=Origin(xyz=(0.165, -0.134, 0.225)),
        material=steel,
        name="pitch_bearing_neg",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.035, length=0.305),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=steel,
        name="pitch_pin",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.065, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=warm_grey,
        name="pitch_hub",
    )
    pitch_cradle.visual(
        Box((0.270, 0.035, 0.090)),
        origin=Origin(xyz=(0.200, 0.110, 0.0)),
        material=warm_grey,
        name="side_arm_pos",
    )
    pitch_cradle.visual(
        Box((0.270, 0.035, 0.090)),
        origin=Origin(xyz=(0.200, -0.110, 0.0)),
        material=warm_grey,
        name="side_arm_neg",
    )
    pitch_cradle.visual(
        Box((0.051, 0.285, 0.075)),
        origin=Origin(xyz=(0.0885, 0.0, 0.0)),
        material=warm_grey,
        name="rear_bridge",
    )
    pitch_cradle.visual(
        mesh_from_geometry(
            TorusGeometry(0.079, 0.015, radial_segments=48, tubular_segments=12).rotate_y(math.pi / 2.0),
            "roll_bearing_ring_mesh",
        ),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=steel,
        name="roll_bearing_ring",
    )

    roll_flange = model.part("roll_flange")
    roll_flange.visual(
        Cylinder(radius=0.040, length=0.190),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=cyl_x),
        material=steel,
        name="roll_shaft",
    )
    roll_flange.visual(
        Cylinder(radius=0.064, length=0.105),
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=cyl_x),
        material=graphite,
        name="roll_motor",
    )
    roll_flange.visual(
        Cylinder(radius=0.092, length=0.026),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=cyl_x),
        material=steel,
        name="tool_flange_plate",
    )
    roll_flange.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.089, 0.0, 0.0), rpy=cyl_x),
        material=dark_cast,
        name="tool_pilot",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        y = 0.056 * math.cos(angle)
        z = 0.056 * math.sin(angle)
        roll_flange.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(0.083, y, z), rpy=cyl_x),
            material=graphite,
            name=f"flange_bolt_{index}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.165, 0.0, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_flange,
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_flange = object_model.get_part("roll_flange")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.allow_overlap(
        pitch_cradle,
        roll_flange,
        elem_a="roll_bearing_ring",
        elem_b="roll_motor",
        reason=(
            "The compact roll bearing is modeled as an annular contact band; "
            "the roll motor journal is intentionally seated in that ring."
        ),
    )
    for bearing_name in ("pitch_bearing_pos", "pitch_bearing_neg"):
        ctx.allow_overlap(
            pitch_cradle,
            yaw_stage,
            elem_a="pitch_pin",
            elem_b=bearing_name,
            reason=(
                "The pitch trunnion pin is intentionally captured in the "
                "bearing ring contact band."
            ),
        )

    ctx.check(
        "three revolute wrist stages",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (yaw, pitch, roll)),
        details=f"joint types={[j.articulation_type for j in (yaw, pitch, roll)]}",
    )
    ctx.check(
        "stage order is yaw pitch roll",
        (yaw.parent, yaw.child, pitch.parent, pitch.child, roll.parent, roll.child)
        == ("body", "yaw_stage", "yaw_stage", "pitch_cradle", "pitch_cradle", "roll_flange"),
        details=f"sequence={(yaw.parent, yaw.child, pitch.parent, pitch.child, roll.parent, roll.child)}",
    )
    ctx.check("yaw axis is vertical", tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is transverse", tuple(round(v, 3) for v in pitch.axis) == (0.0, -1.0, 0.0))
    ctx.check("roll axis is tool axis", tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0))

    ctx.expect_gap(
        yaw_stage,
        body,
        axis="z",
        positive_elem="rotor_disk",
        negative_elem="upper_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower rotary disk seats on body bearing",
    )
    ctx.expect_gap(
        yaw_stage,
        pitch_cradle,
        axis="y",
        positive_elem="pitch_bearing_pos",
        negative_elem="pitch_hub",
        min_gap=0.020,
        max_gap=0.060,
        name="positive pitch side has visible bearing clearance",
    )
    ctx.expect_gap(
        pitch_cradle,
        yaw_stage,
        axis="y",
        positive_elem="pitch_hub",
        negative_elem="pitch_bearing_neg",
        min_gap=0.020,
        max_gap=0.060,
        name="negative pitch side has visible bearing clearance",
    )
    for bearing_name in ("pitch_bearing_pos", "pitch_bearing_neg"):
        ctx.expect_within(
            pitch_cradle,
            yaw_stage,
            axes="xz",
            inner_elem="pitch_pin",
            outer_elem=bearing_name,
            margin=0.0,
            name=f"pitch pin is centered in {bearing_name}",
        )
        ctx.expect_overlap(
            pitch_cradle,
            yaw_stage,
            axes="y",
            elem_a="pitch_pin",
            elem_b=bearing_name,
            min_overlap=0.020,
            name=f"pitch pin passes through {bearing_name}",
        )
    ctx.expect_gap(
        pitch_cradle,
        roll_flange,
        axis="y",
        positive_elem="side_arm_pos",
        negative_elem="roll_motor",
        min_gap=0.025,
        max_gap=0.055,
        name="roll motor clears positive cradle arm",
    )
    ctx.expect_gap(
        roll_flange,
        pitch_cradle,
        axis="y",
        positive_elem="roll_motor",
        negative_elem="side_arm_neg",
        min_gap=0.025,
        max_gap=0.055,
        name="roll motor clears negative cradle arm",
    )
    ctx.expect_within(
        roll_flange,
        pitch_cradle,
        axes="yz",
        inner_elem="roll_motor",
        outer_elem="roll_bearing_ring",
        margin=0.0,
        name="roll motor journal is centered in bearing ring",
    )
    ctx.expect_overlap(
        roll_flange,
        pitch_cradle,
        axes="x",
        elem_a="roll_motor",
        elem_b="roll_bearing_ring",
        min_overlap=0.025,
        name="roll motor remains inserted through bearing ring",
    )

    rest_roll_pos = ctx.part_world_position(roll_flange)
    with ctx.pose({pitch: 0.65}):
        pitched_roll_pos = ctx.part_world_position(roll_flange)
    ctx.check(
        "positive pitch lifts the roll flange",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.10,
        details=f"rest={rest_roll_pos}, pitched={pitched_roll_pos}",
    )

    return ctx.report()


object_model = build_object_model()
