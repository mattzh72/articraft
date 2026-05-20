from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_yaw_pitch_roll_wrist")

    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark = Material("anodized_black", rgba=(0.04, 0.045, 0.05, 1.0))
    orange = Material("axis_orange", rgba=(0.95, 0.36, 0.05, 1.0))
    blue = Material("bearing_blue", rgba=(0.05, 0.16, 0.32, 1.0))

    model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    model.material("anodized_black", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("axis_orange", rgba=(0.95, 0.36, 0.05, 1.0))
    model.material("bearing_blue", rgba=(0.05, 0.16, 0.32, 1.0))

    # Fixed mounting plate with visible bolt heads and a low, stationary yaw race.
    root_plate = model.part("root_plate")
    root_plate.visual(
        Box((0.46, 0.36, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=steel,
        name="base_slab",
    )
    root_plate.visual(
        Cylinder(radius=0.135, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark,
        name="yaw_race",
    )
    for ix, x in enumerate((-0.18, 0.18)):
        for iy, y in enumerate((-0.13, 0.13)):
            root_plate.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(x, y, 0.036)),
                material=dark,
                name=f"bolt_head_{ix}_{iy}",
            )

    # Rotating vertical stage: a turntable and open two-sided yoke, not a closed housing.
    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=blue,
        name="turntable",
    )
    yaw_stage.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=steel,
        name="vertical_column",
    )
    yaw_stage.visual(
        Box((0.095, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=steel,
        name="lower_yoke_bridge",
    )
    for i, y in enumerate((-0.16, 0.16)):
        yaw_stage.visual(
            Box((0.026, 0.030, 0.210)),
            origin=Origin(xyz=(-0.075, y, 0.275)),
            material=steel,
            name=f"side_post_{i}_0",
        )
        yaw_stage.visual(
            Box((0.026, 0.030, 0.210)),
            origin=Origin(xyz=(0.075, y, 0.275)),
            material=steel,
            name=f"side_post_{i}_1",
        )
        yaw_stage.visual(
            Box((0.176, 0.030, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.175)),
            material=steel,
            name=f"lower_side_rail_{i}",
        )
        yaw_stage.visual(
            Box((0.176, 0.030, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.375)),
            material=steel,
            name=f"upper_side_rail_{i}",
        )
        yaw_stage.visual(
            Cylinder(radius=0.040, length=0.060),
            origin=Origin(xyz=(0.0, y, 0.320), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"pitch_bearing_{i}",
        )
        yaw_stage.visual(
            Box((0.039, 0.030, 0.018)),
            origin=Origin(xyz=(-0.058, y, 0.320)),
            material=steel,
            name=f"bearing_web_{i}_0",
        )
        yaw_stage.visual(
            Box((0.039, 0.030, 0.018)),
            origin=Origin(xyz=(0.058, y, 0.320)),
            material=steel,
            name=f"bearing_web_{i}_1",
        )

    # The cradle is a lightweight open rectangular carrier centered on the pitch axis.
    pitch_cradle = model.part("pitch_cradle")
    for i, y in enumerate((-0.115, 0.115)):
        pitch_cradle.visual(
            Cylinder(radius=0.026, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=orange,
            name=f"pitch_trunnion_{i}",
        )
        pitch_cradle.visual(
            Box((0.070, 0.030, 0.140)),
            origin=Origin(xyz=(0.0, -0.066 if y < 0.0 else 0.066, 0.0)),
            material=steel,
            name=f"pitch_side_web_{i}",
        )
    pitch_cradle.visual(
        Cylinder(radius=0.045, length=0.115),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="roll_bearing",
    )
    pitch_cradle.visual(
        Box((0.255, 0.140, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=steel,
        name="upper_cradle_rail",
    )
    pitch_cradle.visual(
        Box((0.255, 0.140, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="lower_cradle_rail",
    )
    for i, y in enumerate((-0.053, 0.053)):
        pitch_cradle.visual(
            Box((0.060, 0.018, 0.160)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"bearing_strut_{i}",
        )
        pitch_cradle.visual(
            Box((0.026, 0.018, 0.160)),
            origin=Origin(xyz=(-0.112, y, 0.0)),
            material=steel,
            name=f"end_post_{i}_0",
        )
        pitch_cradle.visual(
            Box((0.026, 0.018, 0.160)),
            origin=Origin(xyz=(0.112, y, 0.0)),
            material=steel,
            name=f"end_post_{i}_1",
        )

    # Axial roll member: a visible through-spindle and tool mounting nose.
    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.022, length=0.360),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=orange,
        name="spindle_shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.193, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="tool_flange",
    )
    roll_spindle.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.238, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="tool_stub",
    )
    roll_spindle.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(-0.193, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="rear_cap",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=root_plate,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root_plate = object_model.get_part("root_plate")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.check(
        "three named revolute axes",
        (
            yaw_joint.axis == (0.0, 0.0, 1.0)
            and pitch_joint.axis == (0.0, 1.0, 0.0)
            and roll_joint.axis == (1.0, 0.0, 0.0)
        ),
        details=f"axes were yaw={yaw_joint.axis}, pitch={pitch_joint.axis}, roll={roll_joint.axis}",
    )

    ctx.expect_contact(
        yaw_stage,
        root_plate,
        elem_a="turntable",
        elem_b="yaw_race",
        contact_tol=0.001,
        name="yaw turntable seats on fixed race",
    )

    for i in (0, 1):
        trunnion_name = f"pitch_trunnion_{i}"
        ctx.allow_overlap(
            yaw_stage,
            pitch_cradle,
            elem_a=f"pitch_bearing_{i}",
            elem_b=trunnion_name,
            reason="The pitch trunnion is intentionally captured in the solid bearing-boss proxy.",
        )
        ctx.expect_within(
            pitch_cradle,
            yaw_stage,
            axes="xz",
            inner_elem=trunnion_name,
            outer_elem=f"pitch_bearing_{i}",
            margin=0.001,
            name=f"pitch trunnion centered in bearing {i}",
        )
        ctx.expect_overlap(
            pitch_cradle,
            yaw_stage,
            axes="y",
            elem_a=trunnion_name,
            elem_b=f"pitch_bearing_{i}",
            min_overlap=0.010,
            name=f"pitch trunnion retained in bearing {i}",
        )

    ctx.allow_overlap(
        pitch_cradle,
        roll_spindle,
        elem_a="roll_bearing",
        elem_b="spindle_shaft",
        reason="The axial spindle is intentionally shown captured by the solid roll-bearing sleeve proxy.",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_cradle,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="roll_bearing",
        margin=0.001,
        name="roll spindle centered in bearing sleeve",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_cradle,
        axes="x",
        elem_a="spindle_shaft",
        elem_b="roll_bearing",
        min_overlap=0.100,
        name="roll spindle retained through bearing sleeve",
    )

    with ctx.pose({pitch_joint: 0.80}):
        ctx.expect_gap(
            roll_spindle,
            root_plate,
            axis="z",
            min_gap=0.060,
            name="pitched tool clears fixed base",
        )

    return ctx.report()


object_model = build_object_model()
