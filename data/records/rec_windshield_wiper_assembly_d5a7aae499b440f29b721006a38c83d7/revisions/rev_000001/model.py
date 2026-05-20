from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    cast_aluminum = model.material("cast_aluminum", color=(0.55, 0.57, 0.55, 1.0))
    satin_black = model.material("satin_black", color=(0.01, 0.012, 0.011, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.0, 0.0, 0.0, 1.0))
    bare_steel = model.material("bare_steel", color=(0.72, 0.70, 0.66, 1.0))

    base = model.part("motor_housing")
    base.visual(
        Box((0.30, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_black,
        name="mounting_plinth",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=cast_aluminum,
        name="raised_pedestal",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.17),
        origin=Origin(xyz=(-0.035, 0.0, 0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_aluminum,
        name="motor_can",
    )
    base.visual(
        Box((0.17, 0.055, 0.075)),
        origin=Origin(xyz=(0.040, 0.0, 0.105)),
        material=cast_aluminum,
        name="gearcase_neck",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=bare_steel,
        name="spindle_post",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.271)),
        material=bare_steel,
        name="spindle_cap",
    )
    for i, (x, y) in enumerate(
        ((-0.115, -0.075), (-0.115, 0.075), (0.115, -0.075), (0.115, 0.075))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, y, 0.045)),
            material=bare_steel,
            name=f"base_bolt_{i}",
        )

    arm = model.part("sweep_arm")
    tapered_arm_profile = [
        (0.032, -0.028),
        (0.190, -0.021),
        (0.520, -0.014),
        (0.612, -0.018),
        (0.612, 0.018),
        (0.520, 0.014),
        (0.190, 0.021),
        (0.032, 0.028),
    ]
    arm.visual(
        mesh_from_geometry(ExtrudeGeometry(tapered_arm_profile, 0.018), "tapered_sweep_arm"),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_black,
        name="tapered_arm",
    )
    arm.visual(
        Cylinder(radius=0.056, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_black,
        name="hub_disk",
    )
    arm.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=bare_steel,
        name="hub_nut",
    )
    arm.visual(
        Box((0.430, 0.010, 0.012)),
        origin=Origin(xyz=(0.330, 0.0, 0.030)),
        material=bare_steel,
        name="raised_rib",
    )
    arm.visual(
        Box((0.014, 0.062, 0.054)),
        origin=Origin(xyz=(0.613, 0.0, 0.001)),
        material=satin_black,
        name="tip_fork_root",
    )
    arm.visual(
        Box((0.014, 0.062, 0.054)),
        origin=Origin(xyz=(0.667, 0.0, 0.001)),
        material=satin_black,
        name="tip_fork_outer",
    )
    arm.visual(
        Box((0.068, 0.014, 0.012)),
        origin=Origin(xyz=(0.640, 0.0, 0.028)),
        material=satin_black,
        name="tip_fork_bridge",
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bare_steel,
        name="roll_barrel",
    )
    blade.visual(
        Box((0.034, 0.040, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=bare_steel,
        name="carrier_saddle",
    )
    blade.visual(
        Box((0.026, 0.520, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=satin_black,
        name="blade_spine",
    )
    rubber_profile = [(-0.010, 0.0), (0.010, 0.0), (0.0, -0.044)]
    blade.visual(
        mesh_from_geometry(ExtrudeGeometry(rubber_profile, 0.500), "rubber_wiper_edge"),
        origin=Origin(xyz=(0.0, 0.0, -0.096), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="rubber_edge",
    )
    blade.visual(
        Box((0.032, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, -0.270, -0.090)),
        material=satin_black,
        name="blade_end_0",
    )
    blade.visual(
        Box((0.032, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.270, -0.090)),
        material=satin_black,
        name="blade_end_1",
    )

    model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.05, upper=1.05),
    )

    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=3.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    blade = object_model.get_part("blade_carrier")
    spindle = object_model.get_articulation("spindle_sweep")
    roll = object_model.get_articulation("blade_roll")

    cap_aabb = ctx.part_element_world_aabb(base, elem="spindle_cap")
    ctx.check(
        "spindle joint sits above tall base",
        cap_aabb is not None and cap_aabb[1][2] >= 0.279,
        details=f"spindle_cap_aabb={cap_aabb}",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="hub_disk",
        negative_elem="spindle_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="arm hub rests on spindle cap",
    )
    ctx.expect_origin_distance(
        arm,
        blade,
        axes="xy",
        min_dist=0.60,
        max_dist=0.68,
        name="blade carrier is mounted at arm tip",
    )

    rest_tip = ctx.part_world_position(blade)
    with ctx.pose({spindle: 0.85}):
        swept_tip = ctx.part_world_position(blade)
    ctx.check(
        "arm sweeps blade around spindle",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.40
        and swept_tip[0] < rest_tip[0] - 0.15,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    rest_rubber_aabb = ctx.part_element_world_aabb(blade, elem="rubber_edge")
    with ctx.pose({roll: 0.35}):
        rolled_rubber_aabb = ctx.part_element_world_aabb(blade, elem="rubber_edge")
    rest_height = (
        rest_rubber_aabb[1][2] - rest_rubber_aabb[0][2]
        if rest_rubber_aabb is not None
        else None
    )
    rolled_height = (
        rolled_rubber_aabb[1][2] - rolled_rubber_aabb[0][2]
        if rolled_rubber_aabb is not None
        else None
    )
    ctx.check(
        "blade carrier rolls about arm axis",
        rest_height is not None and rolled_height is not None and rolled_height > rest_height + 0.12,
        details=f"rest_height={rest_height}, rolled_height={rolled_height}",
    )

    return ctx.report()


object_model = build_object_model()
