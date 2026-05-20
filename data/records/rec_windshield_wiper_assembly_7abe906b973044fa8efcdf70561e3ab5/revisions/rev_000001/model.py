from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tapered_arm_geometry(
    *,
    x0: float,
    x1: float,
    width0: float,
    width1: float,
    thickness: float,
    z_center: float,
) -> MeshGeometry:
    """A closed tapered rectangular beam, wider at the spindle than at the tip."""
    z0 = z_center - thickness / 2.0
    z1 = z_center + thickness / 2.0
    hw0 = width0 / 2.0
    hw1 = width1 / 2.0
    vertices = [
        (x0, -hw0, z0),
        (x0, hw0, z0),
        (x0, hw0, z1),
        (x0, -hw0, z1),
        (x1, -hw1, z0),
        (x1, hw1, z0),
        (x1, hw1, z1),
        (x1, -hw1, z1),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (1, 5, 6),
        (1, 6, 2),
        (0, 3, 7),
        (0, 7, 4),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _rubber_blade_geometry(
    *,
    length: float,
    x_center: float,
    top_width: float,
    z_top: float,
    z_tip: float,
) -> MeshGeometry:
    """A triangular rubber squeegee prism extruded along the blade length."""
    hy = length / 2.0
    hx = top_width / 2.0
    vertices = [
        (x_center - hx, -hy, z_top),
        (x_center + hx, -hy, z_top),
        (x_center, -hy, z_tip),
        (x_center - hx, hy, z_top),
        (x_center + hx, hy, z_top),
        (x_center, hy, z_tip),
    ]
    faces = [
        (0, 2, 1),
        (3, 4, 5),
        (0, 1, 4),
        (0, 4, 3),
        (1, 2, 5),
        (1, 5, 4),
        (2, 0, 3),
        (2, 3, 5),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_windshield_wiper")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.017, 1.0))
    graphite = model.material("graphite_housing", rgba=(0.10, 0.105, 0.11, 1.0))
    dark_metal = model.material("dark_zinc_metal", rgba=(0.26, 0.27, 0.28, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.65, 0.66, 0.64, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.28, 0.16, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="mounting_foot",
    )
    housing.visual(
        Cylinder(radius=0.078, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=satin_black,
        name="gearbox_dome",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=dark_metal,
        name="spindle_sleeve",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=brushed_steel,
        name="spindle",
    )
    for index, x in enumerate((-0.090, 0.090)):
        housing.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=dark_metal,
            name=f"mount_boss_{index}",
        )
        housing.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.077)),
            material=brushed_steel,
            name=f"bolt_head_{index}",
        )

    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_metal,
        name="hub",
    )
    arm.visual(
        Cylinder(radius=0.023, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=brushed_steel,
        name="retaining_nut",
    )
    arm.visual(
        mesh_from_geometry(
            _tapered_arm_geometry(
                x0=0.030,
                x1=0.590,
                width0=0.046,
                width1=0.026,
                thickness=0.014,
                z_center=0.022,
            ),
            "tapered_sweep_arm",
        ),
        material=satin_black,
        name="tapered_beam",
    )
    arm.visual(
        Box((0.430, 0.012, 0.010)),
        origin=Origin(xyz=(0.300, 0.0, 0.034)),
        material=dark_metal,
        name="center_rib",
    )
    arm.visual(
        Cylinder(radius=0.023, length=0.035),
        origin=Origin(xyz=(0.6025, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="roll_socket",
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Cylinder(radius=0.017, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="roll_barrel",
    )
    carrier.visual(
        Box((0.036, 0.080, 0.045)),
        origin=Origin(xyz=(0.055, 0.0, -0.034)),
        material=dark_metal,
        name="center_yoke",
    )
    carrier.visual(
        Box((0.045, 0.660, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, -0.064)),
        material=dark_metal,
        name="blade_rail",
    )
    carrier.visual(
        mesh_from_geometry(
            _rubber_blade_geometry(
                length=0.690,
                x_center=0.055,
                top_width=0.024,
                z_top=-0.073,
                z_tip=-0.138,
            ),
            "triangular_rubber_blade",
        ),
        material=rubber,
        name="rubber_blade",
    )
    for index, y in enumerate((-0.300, 0.300)):
        carrier.visual(
            Box((0.034, 0.034, 0.024)),
            origin=Origin(xyz=(0.055, y, -0.061)),
            material=satin_black,
            name=f"blade_clip_{index}",
        )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=carrier,
        origin=Origin(xyz=(0.620, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    carrier = object_model.get_part("blade_carrier")
    sweep = object_model.get_articulation("arm_sweep")
    roll = object_model.get_articulation("blade_roll")

    ctx.check(
        "assembly uses the two requested revolute joints",
        sweep.articulation_type == ArticulationType.REVOLUTE
        and roll.articulation_type == ArticulationType.REVOLUTE,
        details=f"sweep={sweep.articulation_type}, roll={roll.articulation_type}",
    )
    ctx.check(
        "sweep joint is vertical spindle axis",
        tuple(round(v, 6) for v in sweep.axis) == (0.0, 0.0, 1.0),
        details=f"axis={sweep.axis}",
    )
    ctx.check(
        "blade roll joint runs along the arm",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )

    with ctx.pose({sweep: 0.0, roll: 0.0}):
        ctx.expect_gap(
            arm,
            housing,
            axis="z",
            positive_elem="hub",
            negative_elem="spindle",
            max_gap=0.002,
            max_penetration=0.0,
            name="arm hub seats on the spindle",
        )
        ctx.expect_gap(
            carrier,
            arm,
            axis="x",
            positive_elem="roll_barrel",
            negative_elem="roll_socket",
            max_gap=0.002,
            max_penetration=0.0,
            name="blade roll barrel is captured at the arm tip",
        )
        ctx.expect_overlap(
            carrier,
            arm,
            axes="z",
            elem_a="roll_barrel",
            elem_b="roll_socket",
            min_overlap=0.015,
            name="tip roll barrel is coaxial with the socket",
        )

    with ctx.pose({sweep: 0.0, roll: 0.0}):
        rest_tip = ctx.part_world_position(carrier)
    with ctx.pose({sweep: 0.70, roll: 0.0}):
        swept_tip = ctx.part_world_position(carrier)
    ctx.check(
        "positive sweep moves the blade carrier around the spindle",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.20
        and swept_tip[0] < rest_tip[0] - 0.08,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    with ctx.pose({sweep: 0.0, roll: 0.0}):
        flat_aabb = ctx.part_element_world_aabb(carrier, elem="rubber_blade")
    with ctx.pose({sweep: 0.0, roll: 0.35}):
        rolled_aabb = ctx.part_element_world_aabb(carrier, elem="rubber_blade")
    flat_z = flat_aabb[1][2] - flat_aabb[0][2] if flat_aabb else 0.0
    rolled_z = rolled_aabb[1][2] - rolled_aabb[0][2] if rolled_aabb else 0.0
    ctx.check(
        "positive blade roll visibly tilts the long blade",
        rolled_z > flat_z + 0.10,
        details=f"flat_z={flat_z}, rolled_z={rolled_z}",
    )

    return ctx.report()


object_model = build_object_model()
