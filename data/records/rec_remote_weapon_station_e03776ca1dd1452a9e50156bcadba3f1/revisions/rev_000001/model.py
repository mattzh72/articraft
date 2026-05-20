from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.35, height=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        name="pedestal_base"
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.3, height=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="turret_base"
    )
    # Armored side plates
    turret.visual(
        Box((0.3, 0.05, 0.4)),
        origin=Origin(xyz=(0.0, 0.225, 0.25)), # Left plate
        name="left_plate"
    )
    turret.visual(
        Box((0.3, 0.05, 0.4)),
        origin=Origin(xyz=(0.0, -0.225, 0.25)), # Right plate
        name="right_plate"
    )

    cradle = model.part("cradle")
    # Central pitch shaft connecting the two sides
    cradle.visual(
        Cylinder(radius=0.05, height=0.4),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        name="pitch_shaft"
    )
    # Boxy sensor unit on the left side
    cradle.visual(
        Box((0.2, 0.14, 0.2)),
        origin=Origin(xyz=(0.05, 0.12, 0.0)),
        name="sensor_unit"
    )
    # Receiver for the barrel on the right side
    cradle.visual(
        Box((0.15, 0.1, 0.15)),
        origin=Origin(xyz=(0.0, -0.12, 0.0)),
        name="receiver"
    )
    # Machine-gun-like barrel
    cradle.visual(
        Cylinder(radius=0.02, height=0.6),
        origin=Origin(xyz=(0.375, -0.12, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="barrel"
    )

    model.articulation(
        "pedestal_to_turret",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0)
    )

    model.articulation(
        "turret_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.3)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.3, upper=1.2)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    pitch_joint = object_model.get_articulation("turret_to_cradle")
    
    ctx.expect_gap(turret, pedestal, axis="z", min_gap=0.0, max_gap=0.001, name="turret sits on pedestal")
    ctx.expect_within(cradle, turret, axes="y", inner_elem="pitch_shaft", margin=0.001, name="cradle shaft fits between plates")
    
    rest_pos = ctx.part_element_world_aabb(cradle, elem="barrel")
    with ctx.pose({pitch_joint: 1.0}):
        pitched_pos = ctx.part_element_world_aabb(cradle, elem="barrel")
        
    if rest_pos and pitched_pos:
        ctx.check(
            "barrel pitches upwards",
            pitched_pos[1][2] > rest_pos[1][2] + 0.1,
            details="Barrel should move upwards when pitch increases."
        )
    
    return ctx.report()

object_model = build_object_model()