from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _canopy_shell() -> cq.Workplane:
    """Shallow tapered ceiling canopy, open-looking by silhouette."""
    return (
        cq.Workplane("XY")
        .circle(0.135)
        .workplane(offset=-0.070)
        .circle(0.072)
        .loft(combine=True)
    )


def _motor_shell() -> cq.Workplane:
    """Galvanized rectangular motor box with softened vertical corners."""
    return cq.Workplane("XY").box(0.380, 0.280, 0.140).edges("|Z").fillet(0.022)


def _wood_blade() -> cq.Workplane:
    """Wide reclaimed-wood ceiling-fan blade, rooted on local +X."""
    length = 0.500
    root_w = 0.145
    tip_w = 0.178
    thickness = 0.016
    pts = [
        (0.000, -root_w * 0.50),
        (length * 0.82, -tip_w * 0.50),
        (length * 0.98, -tip_w * 0.34),
        (length, 0.000),
        (length * 0.98, tip_w * 0.34),
        (length * 0.82, tip_w * 0.50),
        (0.000, root_w * 0.50),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(thickness).translate(
        (0.0, 0.0, -thickness * 0.5)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farmhouse_three_blade_ceiling_fan")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    darker_metal = model.material("darkened_steel", rgba=(0.25, 0.27, 0.27, 1.0))
    wood = model.material("reclaimed_wood", rgba=(0.49, 0.34, 0.20, 1.0))
    dark_wood = model.material("dark_wood_grain", rgba=(0.22, 0.13, 0.07, 1.0))
    screw_metal = model.material("blackened_screws", rgba=(0.04, 0.04, 0.035, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(_canopy_shell(), "tapered_canopy", tolerance=0.0008),
        material=galvanized,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.142, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=galvanized,
        name="ceiling_plate",
    )
    canopy.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=darker_metal,
        name="lower_collar",
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.016, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=darker_metal,
        name="rod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=galvanized,
        name="top_collar",
    )
    downrod.visual(
        Cylinder(radius=0.038, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.329)),
        material=galvanized,
        name="bottom_collar",
    )

    motor = model.part("motor_housing")
    motor.visual(
        mesh_from_cadquery(_motor_shell(), "rectangular_motor_housing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=galvanized,
        name="housing_shell",
    )
    motor.visual(
        Cylinder(radius=0.048, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=galvanized,
        name="rod_receiver",
    )
    motor.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.189)),
        material=darker_metal,
        name="lower_bearing_plate",
    )
    # Raised galvanized seams and strap-like side stiffeners make the box read as sheet metal.
    for side, y in (("side_0", 0.143), ("side_1", -0.143)):
        motor.visual(
            Box((0.310, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.075)),
            material=darker_metal,
            name=f"{side}_upper_rib",
        )
        motor.visual(
            Box((0.310, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.145)),
            material=darker_metal,
            name=f"{side}_lower_rib",
        )
    for end, x in (("end_0", 0.193), ("end_1", -0.193)):
        motor.visual(
            Box((0.006, 0.205, 0.095)),
            origin=Origin(xyz=(x, 0.0, -0.110)),
            material=darker_metal,
            name=f"{end}_panel",
        )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.072, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=darker_metal,
        name="hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.092, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=galvanized,
        name="hub_lip",
    )

    blade_mesh = mesh_from_cadquery(_wood_blade(), "wide_reclaimed_blade", tolerance=0.0008)
    for index in range(3):
        yaw = index * math.tau / 3.0
        cx = math.cos(yaw)
        sy = math.sin(yaw)
        blade_assembly.visual(
            blade_mesh,
            origin=Origin(xyz=(0.235 * cx, 0.235 * sy, -0.078), rpy=(0.0, 0.0, yaw)),
            material=wood,
            name=f"blade_{index}",
        )
        blade_assembly.visual(
            Box((0.240, 0.044, 0.012)),
            origin=Origin(xyz=(0.160 * cx, 0.160 * sy, -0.069), rpy=(0.0, 0.0, yaw)),
            material=galvanized,
            name=f"iron_{index}",
        )
        # Two long, slightly proud grain strips and screw heads give each blade a reclaimed board look.
        for strip in range(2):
            offset = -0.035 if strip == 0 else 0.039
            blade_assembly.visual(
                Box((0.430, 0.006, 0.002)),
                origin=Origin(
                    xyz=(
                        (0.470 * cx) - (offset * sy),
                        (0.470 * sy) + (offset * cx),
                        -0.0705,
                    ),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_wood,
                name=f"grain_{index}_{strip}",
            )
        for screw, radius in enumerate((0.115, 0.225)):
            blade_assembly.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(radius * cx, radius * sy, -0.061)),
                material=screw_metal,
                name=f"screw_{index}_{screw}",
            )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
    )
    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
    )
    model.articulation(
        "motor_to_blades",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=22.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor = object_model.get_part("motor_housing")
    blades = object_model.get_part("blade_assembly")
    spin = object_model.get_articulation("motor_to_blades")

    ctx.check(
        "blade assembly rotates continuously",
        spin is not None and spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint={spin!r}",
    )
    ctx.check(
        "three wide wood blades",
        blades is not None
        and all(blades.get_visual(f"blade_{index}") is not None for index in range(3)),
        details="Expected exactly the three named broad wood blade visuals.",
    )
    ctx.expect_contact(
        canopy,
        downrod,
        elem_a="lower_collar",
        elem_b="top_collar",
        contact_tol=0.002,
        name="downrod seats in canopy collar",
    )
    ctx.expect_contact(
        downrod,
        motor,
        elem_a="bottom_collar",
        elem_b="rod_receiver",
        contact_tol=0.002,
        name="downrod mounts to motor receiver",
    )
    ctx.expect_gap(
        motor,
        blades,
        axis="z",
        positive_elem="lower_bearing_plate",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.003,
        name="rotating hub is tucked under motor bearing",
    )
    with ctx.pose({spin: math.pi / 3.0}):
        rotated_pos = ctx.part_world_position(blades)
        ctx.check(
            "continuous joint accepts rotation pose",
            rotated_pos is not None,
            details=f"rotated_pos={rotated_pos}",
        )

    return ctx.report()


object_model = build_object_model()
