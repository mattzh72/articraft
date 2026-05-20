from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


BLADE_COUNT = 5
ROTOR_JOINT_Z = -0.590


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _airfoil_section(
    radius: float,
    half_width: float,
    center_z: float,
    thickness: float,
    pitch_deg: float,
) -> list[tuple[float, float, float]]:
    """A slim pitched section for one residential fan blade."""
    slope = math.tan(math.radians(pitch_deg))

    def z_at(y: float, offset: float) -> float:
        return center_z + slope * y + offset

    return [
        (radius, -half_width, z_at(-half_width, -0.10 * thickness)),
        (radius, -0.62 * half_width, z_at(-0.62 * half_width, 0.46 * thickness)),
        (radius, 0.05 * half_width, z_at(0.05 * half_width, 0.56 * thickness)),
        (radius, 0.72 * half_width, z_at(0.72 * half_width, 0.36 * thickness)),
        (radius, half_width, z_at(half_width, 0.02 * thickness)),
        (radius, 0.58 * half_width, z_at(0.58 * half_width, -0.48 * thickness)),
        (radius, -0.10 * half_width, z_at(-0.10 * half_width, -0.58 * thickness)),
        (radius, -0.78 * half_width, z_at(-0.78 * half_width, -0.36 * thickness)),
    ]


def _build_blade_mesh(angle: float) -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _airfoil_section(0.255, 0.045, -0.066, 0.014, 8.0),
                _airfoil_section(0.310, 0.074, -0.068, 0.015, 7.0),
                _airfoil_section(0.430, 0.079, -0.071, 0.014, 6.0),
                _airfoil_section(0.565, 0.071, -0.073, 0.012, 5.0),
                _airfoil_section(0.680, 0.056, -0.075, 0.010, 4.0),
                _airfoil_section(0.710, 0.018, -0.075, 0.008, 4.0),
            ]
        )
    )
    return blade.rotate_z(angle)


def _build_canopy_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, -0.162),
            (0.048, -0.162),
            (0.072, -0.145),
            (0.104, -0.103),
            (0.142, -0.042),
            (0.162, 0.000),
            (0.000, 0.000),
        ],
        segments=72,
    )


def _build_upper_motor_mesh() -> MeshGeometry:
    shell = LatheGeometry(
        [
            (0.000, -0.574),
            (0.154, -0.574),
            (0.176, -0.552),
            (0.188, -0.514),
            (0.181, -0.474),
            (0.146, -0.438),
            (0.080, -0.421),
            (0.000, -0.421),
        ],
        segments=88,
    )
    top_neck = CylinderGeometry(radius=0.050, height=0.070, radial_segments=48).translate(
        0.0, 0.0, -0.404
    )
    lower_trim = CylinderGeometry(radius=0.165, height=0.012, radial_segments=72).translate(
        0.0, 0.0, -0.570
    )
    return _merge_geometries([shell, top_neck, lower_trim])


def _build_lower_rotor_housing_mesh() -> MeshGeometry:
    body = LatheGeometry(
        [
            (0.000, 0.000),
            (0.132, 0.000),
            (0.162, -0.026),
            (0.176, -0.068),
            (0.158, -0.112),
            (0.092, -0.146),
            (0.000, -0.146),
        ],
        segments=88,
    )
    belt = CylinderGeometry(radius=0.180, height=0.014, radial_segments=72).translate(
        0.0, 0.0, -0.060
    )
    bottom_cap = LatheGeometry(
        [
            (0.000, -0.164),
            (0.064, -0.164),
            (0.080, -0.151),
            (0.072, -0.136),
            (0.000, -0.136),
        ],
        segments=64,
    )
    return _merge_geometries([body, belt, bottom_cap])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan")

    ceiling_white = model.material("ceiling_white", rgba=(0.90, 0.88, 0.84, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.62, 0.60, 0.55, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.025, 0.024, 0.022, 1.0))
    walnut = model.material("walnut_blade", rgba=(0.38, 0.20, 0.10, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.17, 0.13, 0.10, 1.0))

    mount = model.part("ceiling_mount")
    mount.visual(
        Box((0.70, 0.70, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=ceiling_white,
        name="ceiling_panel",
    )
    mount.visual(
        mesh_from_geometry(_build_canopy_mesh(), "canopy_bell"),
        material=satin_nickel,
        name="canopy_bell",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, -0.294)),
        material=satin_nickel,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.043, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.164)),
        material=satin_nickel,
        name="upper_collar",
    )
    mount.visual(
        Cylinder(radius=0.049, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.417)),
        material=satin_nickel,
        name="lower_collar",
    )
    mount.visual(
        mesh_from_geometry(_build_upper_motor_mesh(), "upper_motor"),
        material=satin_nickel,
        name="upper_motor",
    )
    # A visible clevis/yoke around the lower downrod, tied into the motor neck.
    for side, x in (("0", -0.062), ("1", 0.062)):
        mount.visual(
            Box((0.026, 0.030, 0.125)),
            origin=Origin(xyz=(x, 0.0, -0.455)),
            material=satin_nickel,
            name=f"yoke_arm_{side}",
        )
    mount.visual(
        Box((0.150, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.410)),
        material=satin_nickel,
        name="yoke_bridge",
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        radius = 0.183
        mount.visual(
            Box((0.005, 0.026, 0.040)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.520),
                rpy=(0.0, 0.0, angle),
            ),
            material=shadow_black,
            name=f"vent_slot_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_build_lower_rotor_housing_mesh(), "lower_motor"),
        material=satin_nickel,
        name="lower_motor",
    )
    rotor.visual(
        Cylinder(radius=0.073, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.176)),
        material=satin_nickel,
        name="bottom_finial",
    )
    rotor.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin_nickel,
        name="bearing_socket",
    )
    for index in range(BLADE_COUNT):
        angle = index * math.tau / BLADE_COUNT
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            mesh_from_geometry(_build_blade_mesh(angle), f"blade_{index}"),
            material=walnut,
            name=f"blade_{index}",
        )
        rotor.visual(
            Box((0.205, 0.026, 0.018)),
            origin=Origin(
                xyz=(0.202 * c, 0.202 * s, -0.064),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_bronze,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            Box((0.112, 0.098, 0.014)),
            origin=Origin(
                xyz=(0.318 * c, 0.318 * s, -0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_bronze,
            name=f"blade_clamp_{index}",
        )
        for screw_index, tangent_offset in enumerate((-0.030, 0.030)):
            x = 0.324 * c - tangent_offset * s
            y = 0.324 * s + tangent_offset * c
            rotor.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(xyz=(x, y, -0.055)),
                material=shadow_black,
                name=f"blade_screw_{index}_{screw_index}",
            )

    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.60),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.70, length=0.060),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, ROTOR_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("ceiling_mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("fan_spin")

    ctx.check("has_stationary_mount", mount is not None, "Expected an installed ceiling mount.")
    ctx.check("has_rotor", rotor is not None, "Expected a rotating blade rotor.")
    ctx.check("has_spin_joint", spin is not None, "Expected a continuous fan-spin articulation.")
    if mount is None or rotor is None or spin is None:
        return ctx.report()

    ctx.expect_gap(
        mount,
        rotor,
        axis="z",
        min_gap=0.002,
        max_gap=0.030,
        positive_elem="upper_motor",
        negative_elem="lower_motor",
        name="motor halves have bearing clearance",
    )
    ctx.expect_overlap(
        mount,
        rotor,
        axes="xy",
        min_overlap=0.20,
        elem_a="upper_motor",
        elem_b="lower_motor",
        name="rotor is coaxial under motor housing",
    )

    aabb = ctx.part_world_aabb(rotor)
    ctx.check("rotor_aabb_available", aabb is not None, "Expected rotor AABB.")
    if aabb is not None:
        mins, maxs = aabb
        span_x = float(maxs[0] - mins[0])
        span_y = float(maxs[1] - mins[1])
        ctx.check(
            "residential_blade_span",
            max(span_x, span_y) >= 1.30,
            details=f"span_x={span_x:.3f}, span_y={span_y:.3f}",
        )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.radians(25.0)}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "blade_sweeps_about_vertical_axis",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(float(rest_aabb[0][0]) - float(turned_aabb[0][0])) > 0.02
        and abs(float(rest_aabb[1][1]) - float(turned_aabb[1][1])) > 0.02,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
