from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _blade_profile(
    *,
    length: float,
    root_half_width: float,
    mid_half_width: float,
    tip_radius: float,
    tip_segments: int = 12,
) -> list[tuple[float, float]]:
    tip_center_x = length - tip_radius
    outline: list[tuple[float, float]] = [
        (0.0, -root_half_width),
        (0.16 * length, -0.92 * root_half_width),
        (0.42 * length, -mid_half_width),
        (tip_center_x, -tip_radius),
    ]
    for step in range(1, tip_segments):
        angle = -math.pi / 2.0 + (math.pi * step / tip_segments)
        outline.append(
            (
                tip_center_x + tip_radius * math.cos(angle),
                tip_radius * math.sin(angle),
            )
        )
    outline.extend(
        [
            (tip_center_x, tip_radius),
            (0.42 * length, mid_half_width),
            (0.16 * length, 0.92 * root_half_width),
            (0.0, root_half_width),
        ]
    )
    return outline


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hugger_ceiling_fan")

    canopy_white = model.material("canopy_white", rgba=(0.94, 0.95, 0.96, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.67, 0.68, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    wood_veneer = model.material("wood_veneer", rgba=(0.60, 0.43, 0.27, 1.0))

    blade_length = 0.49
    blade_root_radius = 0.148
    blade_thickness = 0.009
    housing_radius = 0.138

    blade_mesh = _save_mesh(
        "fan_blade",
        ExtrudeGeometry.from_z0(
            _blade_profile(
                length=blade_length,
                root_half_width=0.056,
                mid_half_width=0.071,
                tip_radius=0.060,
            ),
            blade_thickness,
        ),
    )
    canopy_mesh = _save_mesh(
        "ceiling_canopy_shell",
        LatheGeometry(
            [
                (0.0, 0.052),
                (0.050, 0.052),
                (0.070, 0.045),
                (0.082, 0.028),
                (0.078, 0.012),
                (0.056, 0.000),
                (0.0, 0.000),
            ],
            segments=56,
        ),
    )
    motor_shell_mesh = _save_mesh(
        "motor_shell",
        LatheGeometry(
            [
                (0.0, -0.006),
                (0.040, -0.008),
                (0.090, -0.015),
                (0.128, -0.030),
                (0.136, -0.058),
                (0.128, -0.086),
                (0.112, -0.102),
                (0.080, -0.112),
                (0.0, -0.112),
            ],
            segments=64,
        ),
    )
    lower_cap_mesh = _save_mesh(
        "lower_switch_cap",
        LatheGeometry(
            [
                (0.0, -0.096),
                (0.050, -0.100),
                (0.084, -0.108),
                (0.098, -0.124),
                (0.088, -0.140),
                (0.060, -0.150),
                (0.0, -0.150),
            ],
            segments=56,
        ),
    )

    ceiling_canopy = model.part("ceiling_canopy")
    ceiling_canopy.visual(
        canopy_mesh,
        material=canopy_white,
        name="canopy_shell",
    )
    ceiling_canopy.visual(
        Cylinder(radius=0.043, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_metal,
        name="bearing_plate",
    )
    ceiling_canopy.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_metal,
        name="mount_collar",
    )
    ceiling_canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.082, length=0.052),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.040, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=satin_metal,
        name="thrust_collar",
    )
    fan_rotor.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_metal,
        name="axle_shaft",
    )
    fan_rotor.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_metal,
        name="top_cap",
    )
    fan_rotor.visual(
        motor_shell_mesh,
        material=canopy_white,
        name="motor_shell",
    )
    fan_rotor.visual(
        Cylinder(radius=housing_radius, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=satin_metal,
        name="rim_band",
    )
    fan_rotor.visual(
        lower_cap_mesh,
        material=canopy_white,
        name="switch_cap",
    )
    fan_rotor.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.146)),
        material=satin_metal,
        name="bottom_cap",
    )

    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        radial_rpy = (0.0, 0.0, angle)

        fan_rotor.visual(
            Box((0.046, 0.045, 0.012)),
            origin=Origin(xyz=(0.131, 0.0, -0.056), rpy=radial_rpy),
            material=dark_metal,
            name=f"iron_inner_pad_{index}",
        )
        fan_rotor.visual(
            Box((0.090, 0.022, 0.008)),
            origin=Origin(xyz=(0.170, 0.0, -0.060), rpy=radial_rpy),
            material=dark_metal,
            name=f"iron_arm_{index}",
        )
        fan_rotor.visual(
            Box((0.070, 0.048, 0.008)),
            origin=Origin(xyz=(0.197, 0.0, -0.058), rpy=radial_rpy),
            material=dark_metal,
            name=f"iron_outer_pad_{index}",
        )
        for bolt_index, (bolt_x, bolt_y) in enumerate(((0.178, -0.018), (0.214, 0.018))):
            fan_rotor.visual(
                Cylinder(radius=0.004, length=0.015),
                origin=Origin(xyz=(bolt_x, bolt_y, -0.055), rpy=radial_rpy),
                material=satin_metal,
                name=f"blade_bolt_{index}_{bolt_index}",
            )
        fan_rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(blade_root_radius, 0.0, -0.060), rpy=radial_rpy),
            material=wood_veneer,
            name=f"blade_{index}",
        )

    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.64, length=0.16),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "canopy_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=ceiling_canopy,
        child=fan_rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_canopy = object_model.get_part("ceiling_canopy")
    fan_rotor = object_model.get_part("fan_rotor")
    spin = object_model.get_articulation("canopy_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        fan_rotor,
        ceiling_canopy,
        elem_a="thrust_collar",
        elem_b="bearing_plate",
        name="rotor is physically supported by the canopy bearing",
    )
    ctx.expect_gap(
        ceiling_canopy,
        fan_rotor,
        axis="z",
        positive_elem="canopy_shell",
        negative_elem="motor_shell",
        min_gap=0.004,
        max_gap=0.018,
        name="hugger motor housing sits just below the short canopy",
    )

    limits = spin.motion_limits
    axis_ok = all(math.isclose(component, expected, abs_tol=1e-6) for component, expected in zip(spin.axis, (0.0, 0.0, 1.0)))
    continuous_ok = limits is not None and limits.lower is None and limits.upper is None
    ctx.check(
        "blade assembly uses a vertical continuous spin axle",
        axis_ok and continuous_ok,
        details=f"axis={spin.axis}, lower={None if limits is None else limits.lower}, upper={None if limits is None else limits.upper}",
    )

    def _aabb_center(aabb):
        return tuple((a + b) * 0.5 for a, b in zip(aabb[0], aabb[1])) if aabb is not None else None

    blade_center_rest = _aabb_center(ctx.part_element_world_aabb(fan_rotor, elem="blade_0"))
    with ctx.pose({spin: math.pi / 5.0}):
        blade_center_turned = _aabb_center(ctx.part_element_world_aabb(fan_rotor, elem="blade_0"))

    blade_motion_ok = False
    motion_details = f"rest={blade_center_rest}, turned={blade_center_turned}"
    if blade_center_rest is not None and blade_center_turned is not None:
        rest_radius = math.hypot(blade_center_rest[0], blade_center_rest[1])
        turned_radius = math.hypot(blade_center_turned[0], blade_center_turned[1])
        blade_motion_ok = (
            abs(blade_center_rest[2] - blade_center_turned[2]) < 0.01
            and math.dist(blade_center_rest[:2], blade_center_turned[:2]) > 0.12
            and abs(rest_radius - turned_radius) < 0.02
        )
        motion_details = (
            f"rest={blade_center_rest}, turned={blade_center_turned}, "
            f"rest_radius={rest_radius:.3f}, turned_radius={turned_radius:.3f}"
        )
    ctx.check(
        "continuous joint actually spins the blade set around the hub",
        blade_motion_ok,
        details=motion_details,
    )

    canopy_aabb = ctx.part_world_aabb(ceiling_canopy)
    rotor_aabb = ctx.part_world_aabb(fan_rotor)
    overall_ok = False
    overall_details = f"canopy={canopy_aabb}, rotor={rotor_aabb}"
    if canopy_aabb is not None and rotor_aabb is not None:
        tip_radius = 0.0
        for index in range(5):
            blade_aabb = ctx.part_element_world_aabb(fan_rotor, elem=f"blade_{index}")
            if blade_aabb is None:
                continue
            for x in (blade_aabb[0][0], blade_aabb[1][0]):
                for y in (blade_aabb[0][1], blade_aabb[1][1]):
                    tip_radius = max(tip_radius, math.hypot(x, y))

        min_corner = tuple(min(a, b) for a, b in zip(canopy_aabb[0], rotor_aabb[0]))
        max_corner = tuple(max(a, b) for a, b in zip(canopy_aabb[1], rotor_aabb[1]))
        span_z = max_corner[2] - min_corner[2]
        diameter = 2.0 * tip_radius
        overall_ok = 1.22 <= diameter <= 1.34 and 0.18 <= span_z <= 0.24
        overall_details = (
            f"tip_radius={tip_radius:.3f}, diameter={diameter:.3f}, span_z={span_z:.3f}, "
            f"canopy_z={canopy_aabb}, rotor_z={rotor_aabb}"
        )
    ctx.check(
        "fan keeps realistic hugger-fan proportions",
        overall_ok,
        details=overall_details,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
