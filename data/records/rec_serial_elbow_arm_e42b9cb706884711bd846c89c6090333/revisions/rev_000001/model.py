from __future__ import annotations

from math import atan2, pi, sqrt

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
    model = ArticulatedObject(name="trunnion_root_elbow_arm")

    cast_iron = Material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    bearing = Material("dark_bearing_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    orange = Material("safety_orange_paint", rgba=(0.95, 0.38, 0.06, 1.0))
    plate_steel = Material("brushed_plate_steel", rgba=(0.62, 0.64, 0.63, 1.0))

    y_axis_cylinder = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    shoulder_z = 0.80
    elbow_x = 0.62
    forearm_tip = (0.52, 0.0, -0.25)
    forearm_len = sqrt(forearm_tip[0] ** 2 + forearm_tip[2] ** 2)
    forearm_angle = atan2(-forearm_tip[2], forearm_tip[0])

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.70, 0.50, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.04)),
        material=cast_iron,
        name="floor_plate",
    )
    pedestal.visual(
        Box((0.24, 0.24, 0.58)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=cast_iron,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.34, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=cast_iron,
        name="trunnion_saddle",
    )
    pedestal.visual(
        Box((0.22, 0.06, 0.32)),
        origin=Origin(xyz=(0.0, 0.18, shoulder_z)),
        material=cast_iron,
        name="shoulder_cheek_0",
    )
    pedestal.visual(
        Box((0.22, 0.06, 0.32)),
        origin=Origin(xyz=(0.0, -0.18, shoulder_z)),
        material=cast_iron,
        name="shoulder_cheek_1",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.046),
        origin=Origin(xyz=(0.0, 0.233, shoulder_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="shoulder_cap_0",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.046),
        origin=Origin(xyz=(0.0, -0.233, shoulder_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="shoulder_cap_1",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.125, length=0.30),
        origin=y_axis_cylinder,
        material=bearing,
        name="shoulder_hub",
    )
    for i, y in enumerate((0.125, -0.125)):
        upper_link.visual(
            Box((0.50, 0.04, 0.075)),
            origin=Origin(xyz=(0.29, y, 0.0)),
            material=orange,
            name=f"upper_strap_{i}",
        )
    upper_link.visual(
        Box((0.16, 0.06, 0.22)),
        origin=Origin(xyz=(elbow_x, 0.14, 0.0)),
        material=orange,
        name="elbow_lug_0",
    )
    upper_link.visual(
        Box((0.16, 0.06, 0.22)),
        origin=Origin(xyz=(elbow_x, -0.14, 0.0)),
        material=orange,
        name="elbow_lug_1",
    )
    upper_link.visual(
        Cylinder(radius=0.083, length=0.036),
        origin=Origin(xyz=(elbow_x, 0.188, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="elbow_cap_0",
    )
    upper_link.visual(
        Cylinder(radius=0.083, length=0.036),
        origin=Origin(xyz=(elbow_x, -0.188, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="elbow_cap_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.078, length=0.22),
        origin=y_axis_cylinder,
        material=bearing,
        name="elbow_hub",
    )
    forearm.visual(
        Box((forearm_len, 0.095, 0.07)),
        origin=Origin(
            xyz=(forearm_tip[0] / 2.0, 0.0, forearm_tip[2] / 2.0),
            rpy=(0.0, forearm_angle, 0.0),
        ),
        material=orange,
        name="forearm_beam",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box((0.05, 0.28, 0.20)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=plate_steel,
        name="plate_slab",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.10, upper=0.90),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-1.00, upper=1.35),
    )
    model.articulation(
        "plate_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=forearm_tip, rpy=(0.0, forearm_angle, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check(
        "serial shoulder and elbow are revolute",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "shoulder and elbow axes are parallel",
        tuple(shoulder.axis) == (0.0, 1.0, 0.0) and tuple(elbow.axis) == (0.0, 1.0, 0.0),
    )

    shoulder_radius = upper_link.get_visual("shoulder_hub").geometry.radius
    elbow_radius = forearm.get_visual("elbow_hub").geometry.radius
    ctx.check(
        "shoulder hub is visibly larger than elbow hub",
        shoulder_radius >= elbow_radius * 1.45,
        details=f"shoulder radius={shoulder_radius}, elbow radius={elbow_radius}",
    )

    ctx.expect_contact(
        pedestal,
        upper_link,
        elem_a="shoulder_cheek_0",
        elem_b="shoulder_hub",
        contact_tol=0.001,
        name="shoulder hub is clear inside positive trunnion cheek",
    )
    ctx.expect_contact(
        upper_link,
        pedestal,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_1",
        contact_tol=0.001,
        name="shoulder hub is clear inside negative trunnion cheek",
    )
    ctx.expect_contact(
        upper_link,
        forearm,
        elem_a="elbow_lug_0",
        elem_b="elbow_hub",
        contact_tol=0.001,
        name="elbow hub is clear inside positive clevis lug",
    )
    ctx.expect_contact(
        forearm,
        upper_link,
        elem_a="elbow_hub",
        elem_b="elbow_lug_1",
        contact_tol=0.001,
        name="elbow hub is clear inside negative clevis lug",
    )
    ctx.expect_contact(
        end_plate,
        forearm,
        elem_a="plate_slab",
        elem_b="forearm_beam",
        contact_tol=0.003,
        name="plain end plate seats on the forearm end",
    )

    rest_elbow_position = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.45}):
        moved_elbow_position = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder joint swings the serial arm in the XZ plane",
        rest_elbow_position is not None
        and moved_elbow_position is not None
        and abs(moved_elbow_position[1] - rest_elbow_position[1]) < 1e-6
        and abs(moved_elbow_position[2] - rest_elbow_position[2]) > 0.10,
        details=f"rest={rest_elbow_position}, moved={moved_elbow_position}",
    )

    rest_plate_position = ctx.part_world_position(end_plate)
    with ctx.pose({elbow: 0.60}):
        bent_plate_position = ctx.part_world_position(end_plate)
    ctx.check(
        "elbow joint bends the forearm about the parallel supported axis",
        rest_plate_position is not None
        and bent_plate_position is not None
        and abs(bent_plate_position[1] - rest_plate_position[1]) < 1e-6
        and abs(bent_plate_position[2] - rest_plate_position[2]) > 0.08,
        details=f"rest={rest_plate_position}, bent={bent_plate_position}",
    )

    return ctx.report()


object_model = build_object_model()
