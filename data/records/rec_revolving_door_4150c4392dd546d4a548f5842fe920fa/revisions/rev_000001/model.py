from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _cylindrical_shell_segment(
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    start_angle: float,
    end_angle: float,
    samples: int,
) -> MeshGeometry:
    geom = MeshGeometry()

    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(samples + 1):
        t = i / samples
        angle = start_angle + (end_angle - start_angle) * t
        ca = cos(angle)
        sa = sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z0))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z1))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z0))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z1))

    for i in range(samples):
        _add_quad(
            geom,
            outer_bottom[i],
            outer_bottom[i + 1],
            outer_top[i + 1],
            outer_top[i],
        )
        _add_quad(
            geom,
            inner_bottom[i],
            inner_top[i],
            inner_top[i + 1],
            inner_bottom[i + 1],
        )
        _add_quad(
            geom,
            outer_top[i],
            outer_top[i + 1],
            inner_top[i + 1],
            inner_top[i],
        )
        _add_quad(
            geom,
            outer_bottom[i],
            inner_bottom[i],
            inner_bottom[i + 1],
            outer_bottom[i + 1],
        )

    _add_quad(
        geom,
        outer_bottom[0],
        outer_top[0],
        inner_top[0],
        inner_bottom[0],
    )
    _add_quad(
        geom,
        outer_bottom[-1],
        inner_bottom[-1],
        inner_top[-1],
        outer_top[-1],
    )

    return geom


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_wing_revolving_door")

    frame_metal = model.material("frame_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    canopy_metal = model.material("canopy_metal", rgba=(0.32, 0.33, 0.35, 1.0))
    floor_finish = model.material("floor_finish", rgba=(0.60, 0.60, 0.60, 1.0))
    fixed_glass = model.material("fixed_glass", rgba=(0.62, 0.76, 0.84, 0.32))
    door_glass = model.material("door_glass", rgba=(0.74, 0.87, 0.94, 0.30))

    outer_radius = 1.40
    wall_thickness = 0.05
    inner_radius = outer_radius - wall_thickness
    floor_thickness = 0.05
    clear_height = 2.20
    canopy_thickness = 0.16
    canopy_radius = outer_radius + 0.05

    shaft_radius = 0.09
    hub_radius = 0.16
    wing_thickness = 0.03
    wing_width = 0.065
    wing_inner_start = 0.06
    wing_outer_end = inner_radius - 0.015
    wing_length = wing_outer_end - wing_inner_start
    wing_center = wing_inner_start + wing_length * 0.5

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=outer_radius, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=floor_finish,
        name="floor_disc",
    )
    housing.visual(
        Cylinder(radius=canopy_radius, length=canopy_thickness),
        origin=Origin(
            xyz=(0.0, 0.0, floor_thickness + clear_height + canopy_thickness * 0.5)
        ),
        material=canopy_metal,
        name="canopy",
    )

    portal_half_angle = 35.0 * pi / 180.0
    wall_spans = (
        ("left_drum_glass", portal_half_angle, pi - portal_half_angle),
        ("right_drum_glass", pi + portal_half_angle, 2.0 * pi - portal_half_angle),
    )
    for name, start_angle, end_angle in wall_spans:
        wall_mesh = mesh_from_geometry(
            _cylindrical_shell_segment(
                inner_radius=inner_radius,
                outer_radius=outer_radius,
                z0=floor_thickness,
                z1=floor_thickness + clear_height,
                start_angle=start_angle,
                end_angle=end_angle,
                samples=28,
            ),
            name,
        )
        housing.visual(wall_mesh, material=fixed_glass, name=name)

    post_depth = wall_thickness + 0.03
    post_width = 0.08
    post_radius = inner_radius + post_depth * 0.5
    for index, angle in enumerate(
        (
            portal_half_angle,
            pi - portal_half_angle,
            pi + portal_half_angle,
            2.0 * pi - portal_half_angle,
        ),
        start=1,
    ):
        housing.visual(
            Box((post_depth, post_width, clear_height)),
            origin=Origin(
                xyz=(
                    post_radius * cos(angle),
                    post_radius * sin(angle),
                    floor_thickness + clear_height * 0.5,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame_metal,
            name=f"portal_post_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=shaft_radius, length=clear_height),
        origin=Origin(xyz=(0.0, 0.0, clear_height * 0.5)),
        material=frame_metal,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=hub_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_metal,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=hub_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, clear_height - 0.04)),
        material=frame_metal,
        name="top_hub",
    )

    wing_angles = (
        ("wing_0", 0.0),
        ("wing_90", 0.5 * pi),
        ("wing_180", pi),
        ("wing_270", 1.5 * pi),
    )
    stile_width = 0.04
    stile_center = wing_outer_end - stile_width * 0.5
    for wing_name, angle in wing_angles:
        rotor.visual(
            Box((wing_length, wing_width, clear_height)),
            origin=Origin(
                xyz=(wing_center * cos(angle), wing_center * sin(angle), clear_height * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=door_glass,
            name=wing_name,
        )
        rotor.visual(
            Box((stile_width, wing_width + 0.01, clear_height)),
            origin=Origin(
                xyz=(stile_center * cos(angle), stile_center * sin(angle), clear_height * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame_metal,
            name=f"{wing_name}_stile",
        )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("housing_to_rotor")

    floor_disc = housing.get_visual("floor_disc")
    canopy = housing.get_visual("canopy")
    shaft = rotor.get_visual("shaft")
    wing_0 = rotor.get_visual("wing_0")
    wing_90 = rotor.get_visual("wing_90")

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

    limits = spin.motion_limits
    ctx.check(
        "rotor uses a vertical continuous joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )
    ctx.expect_gap(
        rotor,
        housing,
        axis="z",
        positive_elem=wing_0,
        negative_elem=floor_disc,
        max_gap=0.001,
        max_penetration=0.0,
        name="wing panels start at the floor plane",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="z",
        positive_elem=canopy,
        negative_elem=wing_0,
        max_gap=0.001,
        max_penetration=0.0,
        name="wing panels reach the canopy soffit",
    )
    ctx.expect_contact(
        rotor,
        housing,
        elem_a=shaft,
        elem_b=floor_disc,
        name="central shaft bears on the floor pivot",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="xy",
        margin=0.0,
        name="rotor stays inside the drum footprint",
    )

    wing_0_rest = _aabb_center(ctx.part_element_world_aabb(rotor, elem=wing_0.name))
    wing_90_rest = _aabb_center(ctx.part_element_world_aabb(rotor, elem=wing_90.name))
    ctx.check(
        "two neighboring wings are arranged at right angles",
        wing_0_rest is not None
        and wing_90_rest is not None
        and wing_0_rest[0] > 0.55
        and abs(wing_0_rest[1]) < 0.08
        and wing_90_rest[1] > 0.55
        and abs(wing_90_rest[0]) < 0.08
        and abs(wing_0_rest[0] - wing_90_rest[1]) < 0.10,
        details=f"wing_0={wing_0_rest}, wing_90={wing_90_rest}",
    )

    with ctx.pose({spin: 0.5 * pi}):
        wing_0_rotated = _aabb_center(ctx.part_element_world_aabb(rotor, elem=wing_0.name))

    ctx.check(
        "continuous joint spins the wing assembly around the shaft",
        wing_0_rest is not None
        and wing_0_rotated is not None
        and wing_0_rotated[1] > 0.55
        and abs(wing_0_rotated[0]) < 0.08
        and abs(wing_0_rotated[1] - wing_0_rest[0]) < 0.10,
        details=f"rest={wing_0_rest}, rotated={wing_0_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
