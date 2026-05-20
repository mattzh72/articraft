from __future__ import annotations

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_lift_module")

    cast_dark = model.material("cast_dark", rgba=(0.08, 0.085, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    guide_steel = model.material("guide_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.26, 0.52, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.018, 1.0))

    stator = model.part(
        "stator",
        inertial=Inertial.from_geometry(
            Box((0.42, 0.32, 0.035)),
            mass=3.2,
            origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        ),
    )
    stator.visual(
        Box((0.42, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_dark,
        name="floor_plate",
    )
    stator.visual(
        Cylinder(radius=0.125, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=graphite,
        name="fixed_bearing",
    )
    stator.visual(
        Box((0.34, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.151, 0.040)),
        material=rubber,
        name="front_foot",
    )
    stator.visual(
        Box((0.34, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.151, 0.040)),
        material=rubber,
        name="rear_foot",
    )

    lower_stage = model.part(
        "lower_stage",
        inertial=Inertial.from_geometry(
            Cylinder(radius=0.18, length=0.065),
            mass=5.0,
            origin=Origin(xyz=(0.0, 0.0, 0.035)),
        ),
    )
    lower_stage.visual(
        Cylinder(radius=0.18, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=graphite,
        name="turntable_disk",
    )
    lower_stage.visual(
        Box((0.24, 0.16, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=cast_dark,
        name="gearbox_hump",
    )
    lower_stage.visual(
        Box((0.056, 0.120, 0.430)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=cast_dark,
        name="mast_column",
    )
    lower_stage.visual(
        Box((0.070, 0.178, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=graphite,
        name="lower_rail_clamp",
    )
    lower_stage.visual(
        Box((0.070, 0.178, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=graphite,
        name="upper_rail_clamp",
    )
    lower_stage.visual(
        Box((0.100, 0.150, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.512)),
        material=graphite,
        name="mast_cap",
    )
    lower_stage.visual(
        Cylinder(radius=0.006, length=0.380),
        origin=Origin(xyz=(0.014, 0.076, 0.285)),
        material=guide_steel,
        name="guide_rail_0",
    )
    lower_stage.visual(
        Cylinder(radius=0.006, length=0.380),
        origin=Origin(xyz=(0.014, -0.076, 0.285)),
        material=guide_steel,
        name="guide_rail_1",
    )

    carriage = model.part(
        "carriage",
        inertial=Inertial.from_geometry(
            Box((0.038, 0.125, 0.155)),
            mass=0.72,
            origin=Origin(xyz=(0.019, 0.0, 0.0775)),
        ),
    )
    carriage.visual(
        Box((0.038, 0.125, 0.155)),
        origin=Origin(xyz=(0.019, 0.0, 0.0775)),
        material=carriage_blue,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.115, 0.145, 0.022)),
        origin=Origin(xyz=(0.088, 0.0, 0.135)),
        material=satin_metal,
        name="payload_tray",
    )
    carriage.visual(
        Box((0.012, 0.145, 0.050)),
        origin=Origin(xyz=(0.143, 0.0, 0.159)),
        material=satin_metal,
        name="tray_lip",
    )
    carriage.visual(
        Box((0.078, 0.018, 0.075)),
        origin=Origin(xyz=(0.073, 0.055, 0.090)),
        material=carriage_blue,
        name="side_rib_0",
    )
    carriage.visual(
        Box((0.078, 0.018, 0.075)),
        origin=Origin(xyz=(0.073, -0.055, 0.090)),
        material=carriage_blue,
        name="side_rib_1",
    )

    model.articulation(
        "stator_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=stator,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-3.14159, upper=3.14159),
    )
    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=carriage,
        origin=Origin(xyz=(0.028, 0.0, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.17),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stator = object_model.get_part("stator")
    lower_stage = object_model.get_part("lower_stage")
    carriage = object_model.get_part("carriage")
    pan_joint = object_model.get_articulation("stator_to_lower_stage")
    lift_joint = object_model.get_articulation("mast_to_carriage")

    ctx.expect_gap(
        lower_stage,
        stator,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        name="yaw stage is seated on the stator bearing",
    )
    ctx.expect_contact(
        carriage,
        lower_stage,
        elem_a="slide_plate",
        elem_b="mast_column",
        contact_tol=0.0005,
        name="carriage slide plate bears on mast face",
    )

    lower_mass = lower_stage.inertial.mass if lower_stage.inertial is not None else 0.0
    carriage_mass = carriage.inertial.mass if carriage.inertial is not None else 999.0
    ctx.check(
        "rotating base is heavier than lifted carriage",
        lower_mass > carriage_mass * 4.0,
        details=f"lower_stage mass={lower_mass}, carriage mass={carriage_mass}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift_joint: 0.17}):
        ctx.expect_contact(
            carriage,
            lower_stage,
            elem_a="slide_plate",
            elem_b="mast_column",
            contact_tol=0.0005,
            name="raised carriage remains guided by mast",
        )
        ctx.expect_within(
            carriage,
            lower_stage,
            axes="z",
            inner_elem="slide_plate",
            outer_elem="mast_column",
            margin=0.002,
            name="raised slide plate stays within mast height",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic lift moves carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.16,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({pan_joint: 1.5708}):
        turned_pos = ctx.part_world_position(carriage)
    ctx.check(
        "yaw joint carries the mast and carriage around the base",
        turned_pos is not None and abs(turned_pos[1]) > 0.020,
        details=f"turned carriage origin={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
