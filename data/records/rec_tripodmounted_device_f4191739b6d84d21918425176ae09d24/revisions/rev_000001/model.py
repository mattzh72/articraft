from __future__ import annotations

from math import cos, pi, sin

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
)


LEG_AZIMUTHS = (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)
LEG_PITCH = 1.18


def add_leg(model: ArticulatedObject, name: str, *, body: str, foot: str):
    leg = model.part(name)

    leg.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.004, 0.0, -0.006), rpy=(0.5 * pi, 0.0, 0.0)),
        material="accent",
        name="hinge_pin",
    )

    leg.visual(
        Box((0.010, 0.020, 0.014)),
        origin=Origin(xyz=(0.013, 0.0, -0.006)),
        material=body,
        name="hinge_block",
    )

    leg_length = 0.31
    start_x = 0.012
    start_z = -0.008
    leg.visual(
        Box((leg_length, 0.020, 0.014)),
        origin=Origin(
            xyz=(
                start_x + 0.5 * leg_length * cos(LEG_PITCH),
                0.0,
                start_z - 0.5 * leg_length * sin(LEG_PITCH),
            ),
            rpy=(0.0, LEG_PITCH, 0.0),
        ),
        material=body,
        name="tube",
    )

    foot_x = start_x + leg_length * cos(LEG_PITCH)
    foot_z = start_z - leg_length * sin(LEG_PITCH)
    leg.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(foot_x + 0.003, 0.0, foot_z - 0.004)),
        material=foot,
        name="foot",
    )

    return leg


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tripod_device")

    model.material("body", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    model.material("accent", rgba=(0.44, 0.46, 0.50, 1.0))

    crown = model.part("crown")
    for index, theta in enumerate(LEG_AZIMUTHS):
        crown.visual(
            Box((0.060, 0.014, 0.014)),
            origin=Origin(
                xyz=(0.022 * cos(theta), 0.022 * sin(theta), 0.0),
                rpy=(0.0, 0.0, theta + 0.5 * pi),
            ),
            material="body",
            name=f"ring_{index}",
        )
        crown.visual(
            Box((0.040, 0.014, 0.014)),
            origin=Origin(
                xyz=(0.045 * cos(theta), 0.045 * sin(theta), 0.0),
                rpy=(0.0, 0.0, theta),
            ),
            material="body",
            name=f"arm_{index}",
        )
        crown.visual(
            Box((0.012, 0.014, 0.058)),
            origin=Origin(
                xyz=(0.020 * cos(theta), 0.020 * sin(theta), 0.036),
                rpy=(0.0, 0.0, theta),
            ),
            material="body",
            name=f"rib_{index}",
        )
        crown.visual(
            Box((0.060, 0.014, 0.010)),
            origin=Origin(
                xyz=(0.022 * cos(theta), 0.022 * sin(theta), 0.070),
                rpy=(0.0, 0.0, theta + 0.5 * pi),
            ),
            material="body",
            name=f"clamp_{index}",
        )

    legs = []
    for index, theta in enumerate(LEG_AZIMUTHS):
        leg = add_leg(model, f"leg_{index}", body="body", foot="rubber")
        legs.append(leg)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(0.065 * cos(theta), 0.065 * sin(theta), 0.0),
                rpy=(0.0, 0.0, theta),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=2.0,
                lower=-1.25,
                upper=0.22,
            ),
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.011, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="aluminum",
        name="tube",
    )
    column.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="accent",
        name="seat",
    )
    column.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material="body",
        name="top_cap",
    )

    model.articulation(
        "crown_to_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.16,
            lower=0.0,
            upper=0.06,
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="body",
        name="pan_disk",
    )
    pan_head.visual(
        Box((0.020, 0.036, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="body",
        name="neck",
    )
    for side, y in enumerate((-0.022, 0.022)):
        pan_head.visual(
            Box((0.010, 0.008, 0.036)),
            origin=Origin(xyz=(0.0, -0.021 if side == 0 else 0.021, 0.052)),
            material="body",
            name=f"arm_{side}",
        )

    model.articulation(
        "column_to_pan",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Box((0.012, 0.034, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="aluminum",
        name="trunnion",
    )
    tilt_head.visual(
        Box((0.022, 0.024, 0.022)),
        origin=Origin(xyz=(0.019, 0.0, 0.013)),
        material="body",
        name="support",
    )
    tilt_head.visual(
        Box((0.062, 0.044, 0.006)),
        origin=Origin(xyz=(0.031, 0.0, 0.026)),
        material="aluminum",
        name="plate",
    )
    tilt_head.visual(
        Box((0.026, 0.016, 0.003)),
        origin=Origin(xyz=(0.038, 0.0, 0.0305)),
        material="rubber",
        name="pad",
    )

    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.1,
        ),
    )

    top_knob = model.part("top_knob")
    top_knob.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="accent",
        name="shaft",
    )
    top_knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="body",
        name="knob",
    )
    top_knob.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material="accent",
        name="cap",
    )

    model.articulation(
        "tilt_to_knob",
        ArticulationType.CONTINUOUS,
        parent=tilt_head,
        child=top_knob,
        origin=Origin(xyz=(0.044, 0.012, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    leg_0 = object_model.get_part("leg_0")
    column = object_model.get_part("column")
    tilt_head = object_model.get_part("tilt_head")
    top_knob = object_model.get_part("top_knob")

    leg_joint = object_model.get_articulation("crown_to_leg_0")
    slide_joint = object_model.get_articulation("crown_to_column")
    pan_joint = object_model.get_articulation("column_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_tilt")
    knob_joint = object_model.get_articulation("tilt_to_knob")

    rest_leg_aabb = ctx.part_world_aabb(leg_0)
    crown_aabb = ctx.part_world_aabb(crown)
    ctx.check(
        "deployed leg reaches well below the crown",
        rest_leg_aabb is not None
        and crown_aabb is not None
        and rest_leg_aabb[0][2] < crown_aabb[0][2] - 0.22,
        details=f"leg_0={rest_leg_aabb}, crown={crown_aabb}",
    )

    folded_leg_aabb = None
    if leg_joint.motion_limits is not None and leg_joint.motion_limits.lower is not None:
        with ctx.pose({leg_joint: leg_joint.motion_limits.lower}):
            folded_leg_aabb = ctx.part_world_aabb(leg_0)
    ctx.check(
        "leg folds upward toward the crown",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[0][2] > rest_leg_aabb[0][2] + 0.12,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    rest_tilt_pos = ctx.part_world_position(tilt_head)
    extended_tilt_pos = None
    extended_column_aabb = None
    if slide_joint.motion_limits is not None and slide_joint.motion_limits.upper is not None:
        with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
            extended_tilt_pos = ctx.part_world_position(tilt_head)
            extended_column_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "center column extends upward",
        rest_tilt_pos is not None
        and extended_tilt_pos is not None
        and extended_tilt_pos[2] > rest_tilt_pos[2] + 0.05,
        details=f"rest={rest_tilt_pos}, extended={extended_tilt_pos}",
    )
    ctx.check(
        "extended column remains retained below the crown top",
        crown_aabb is not None
        and extended_column_aabb is not None
        and extended_column_aabb[0][2] < crown_aabb[1][2] - 0.01,
        details=f"extended_column={extended_column_aabb}, crown={crown_aabb}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="plate")
    tilted_plate_aabb = None
    if tilt_joint.motion_limits is not None and tilt_joint.motion_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
            tilted_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="plate")
    ctx.check(
        "top plate tilts nose up",
        rest_plate_aabb is not None
        and tilted_plate_aabb is not None
        and tilted_plate_aabb[1][2] > rest_plate_aabb[1][2] + 0.02,
        details=f"rest={rest_plate_aabb}, tilted={tilted_plate_aabb}",
    )

    ctx.expect_gap(
        top_knob,
        tilt_head,
        axis="z",
        positive_elem="shaft",
        negative_elem="plate",
        max_gap=0.0005,
        max_penetration=0.0,
        name="locking knob seats on the top plate",
    )
    ctx.expect_within(
        top_knob,
        tilt_head,
        axes="xy",
        inner_elem="knob",
        outer_elem="plate",
        margin=0.0,
        name="locking knob stays inside the top plate footprint",
    )

    ctx.check(
        "pan and locking knob use continuous rotation joints",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"pan={pan_joint.articulation_type}, knob={knob_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
