from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _cylinder(radius: float, height: float, z_center: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )


def _ring(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )


def _carrier_shape(
    radius: float,
    thickness: float,
    bore_radius: float,
    *,
    lobe_extra: float,
    lobe_width: float,
    fixture_hole_radius: float,
) -> cq.Workplane:
    """Annular rotary carrier with an integral radial indexing lobe and hub."""
    arm_length = radius + lobe_extra
    platter = _cylinder(radius, thickness)
    arm = cq.Workplane("XY").box(arm_length, lobe_width, thickness).translate(
        (arm_length / 2.0, 0.0, 0.0)
    )
    rounded_nose = _cylinder(lobe_width / 2.0, thickness).translate(
        (arm_length, 0.0, 0.0)
    )
    hub = _ring(
        bore_radius + 0.022,
        bore_radius,
        0.022,
        z_center=thickness / 2.0 + 0.009,
    )
    rim = _ring(
        radius,
        radius - 0.008,
        0.008,
        z_center=thickness / 2.0 + 0.003,
    )
    shape = platter.union(arm).union(rounded_nose).union(hub).union(rim)

    bore_cutter = _cylinder(bore_radius, thickness + 0.080)
    shape = shape.cut(bore_cutter)

    # Through fixture holes make the carriers read as usable rotary plates and
    # make their angular pose visible even when viewed from above.
    hole_circle = radius * 0.58
    for angle_deg in (115.0, 245.0):
        angle = math.radians(angle_deg)
        x = hole_circle * math.cos(angle)
        y = hole_circle * math.sin(angle)
        hole = _cylinder(fixture_hole_radius, thickness + 0.080).translate(
            (x, y, 0.0)
        )
        shape = shape.cut(hole)

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_coaxial_rotary_fixture")

    painted_iron = model.material("painted_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_edge = model.material("dark_edge", rgba=(0.015, 0.017, 0.018, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    thrust_bronze = model.material("thrust_bronze", rgba=(0.78, 0.52, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.011, 1.0))
    carrier_materials = [
        model.material("carrier_blue", rgba=(0.10, 0.28, 0.70, 1.0)),
        model.material("carrier_green", rgba=(0.10, 0.48, 0.32, 1.0)),
        model.material("carrier_orange", rgba=(0.82, 0.38, 0.10, 1.0)),
        model.material("carrier_silver", rgba=(0.62, 0.64, 0.66, 1.0)),
    ]

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.190, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=painted_iron,
        name="ground_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.125, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=dark_edge,
        name="raised_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.062, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=painted_iron,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.021, length=0.615),
        origin=Origin(xyz=(0.0, 0.0, 0.3575)),
        material=polished_steel,
        name="vertical_spindle",
    )

    carrier_specs = [
        ("carrier_0", 0.155, 0.030, 0.230, 0.036, 0.034, 0.013),
        ("carrier_1", 0.125, 0.032, 0.335, 0.032, 0.031, 0.011),
        ("carrier_2", 0.095, 0.032, 0.440, 0.029, 0.027, 0.009),
        ("carrier_3", 0.070, 0.030, 0.545, 0.026, 0.024, 0.007),
    ]

    # Fixed bronze thrust washers sit just under each rotating carrier.  They
    # make the support path visible while leaving a fine running clearance.
    for index, (_, _, thickness, center_z, _, _, _) in enumerate(carrier_specs):
        carrier_bottom = center_z - thickness / 2.0
        washer_thickness = 0.012
        # A tiny intentional preload overlap keeps the bearing stack visibly
        # grounded instead of leaving a floating air gap.
        washer_top = carrier_bottom + 0.0004
        pedestal.visual(
            Cylinder(radius=0.058, length=washer_thickness),
            origin=Origin(xyz=(0.0, 0.0, washer_top - washer_thickness / 2.0)),
            material=thrust_bronze,
            name=f"thrust_washer_{index}",
        )

    pedestal.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.637)),
        material=polished_steel,
        name="retaining_cap",
    )
    for x in (-0.125, 0.125):
        for y in (-0.125, 0.125):
            pedestal.visual(
                Box((0.045, 0.045, 0.012)),
                origin=Origin(xyz=(x, y, 0.003)),
                material=rubber,
                name=f"rubber_pad_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )

    for index, (name, radius, thickness, center_z, lobe_extra, lobe_width, hole_radius) in enumerate(
        carrier_specs
    ):
        carrier = model.part(name)
        carrier.visual(
            mesh_from_cadquery(
                _carrier_shape(
                    radius,
                    thickness,
                    0.032,
                    lobe_extra=lobe_extra,
                    lobe_width=lobe_width,
                    fixture_hole_radius=hole_radius,
                ),
                f"{name}_platter",
                tolerance=0.0008,
                angular_tolerance=0.06,
            ),
            material=carrier_materials[index],
            name="carrier_platter",
        )
        model.articulation(
            f"pedestal_to_{name}",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=carrier,
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-math.pi,
                upper=math.pi,
                effort=8.0,
                velocity=2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carrier_names = [f"carrier_{index}" for index in range(4)]
    joint_names = [f"pedestal_to_{name}" for name in carrier_names]
    pedestal = object_model.get_part("pedestal")
    joints = [object_model.get_articulation(name) for name in joint_names]
    carriers = [object_model.get_part(name) for name in carrier_names]

    ctx.check(
        "four independent carrier revolutes",
        len(joints) == 4
        and all(j.axis == (0.0, 0.0, 1.0) for j in joints)
        and all(getattr(j.parent, "name", j.parent) == "pedestal" for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    centers = [ctx.part_world_position(carrier) for carrier in carriers]
    ctx.check(
        "carrier origins share spindle centerline",
        all(center is not None and abs(center[0]) < 1e-6 and abs(center[1]) < 1e-6 for center in centers),
        details=f"centers={centers}",
    )
    ctx.check(
        "carriers step upward in order",
        all(centers[i] is not None and centers[i + 1] is not None and centers[i + 1][2] > centers[i][2] + 0.08 for i in range(3)),
        details=f"centers={centers}",
    )

    for index, carrier in enumerate(carriers):
        ctx.allow_overlap(
            carrier,
            pedestal,
            elem_a="carrier_platter",
            elem_b=f"thrust_washer_{index}",
            reason="The bronze thrust washer is intentionally modeled with a tiny axial preload into the underside of the rotating carrier.",
        )
        ctx.expect_gap(
            carrier,
            pedestal,
            axis="z",
            positive_elem="carrier_platter",
            negative_elem=f"thrust_washer_{index}",
            max_penetration=0.0006,
            max_gap=0.0002,
            name=f"carrier_{index} has thrust washer running clearance",
        )
        ctx.expect_overlap(
            carrier,
            pedestal,
            axes="xy",
            elem_a="carrier_platter",
            elem_b=f"thrust_washer_{index}",
            min_overlap=0.030,
            name=f"carrier_{index} is radially supported over washer",
        )

    rest_aabb = ctx.part_world_aabb(carriers[0])
    with ctx.pose({joints[0]: 1.2, joints[1]: -0.7, joints[2]: 0.9, joints[3]: -1.1}):
        posed_aabb = ctx.part_world_aabb(carriers[0])
        posed_centers = [ctx.part_world_position(carrier) for carrier in carriers]

    ctx.check(
        "coaxial carrier rotation preserves centerline",
        all(center is not None and abs(center[0]) < 1e-6 and abs(center[1]) < 1e-6 for center in posed_centers),
        details=f"posed_centers={posed_centers}",
    )
    ctx.check(
        "asymmetric lobe visibly changes pose",
        rest_aabb is not None
        and posed_aabb is not None
        and abs((rest_aabb[1][0] - rest_aabb[0][0]) - (posed_aabb[1][0] - posed_aabb[0][0])) > 0.010,
        details=f"rest_aabb={rest_aabb}, posed_aabb={posed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
