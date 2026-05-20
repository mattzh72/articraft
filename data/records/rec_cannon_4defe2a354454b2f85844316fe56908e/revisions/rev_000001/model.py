from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos((2.0 * pi * i) / segments), cy + radius * sin((2.0 * pi * i) / segments))
        for i in range(segments)
    ]


def _wheel_wood_mesh(
    *,
    radius: float,
    width: float,
    hub_radius: float,
    hub_length: float,
    spoke_count: int,
):
    half_width = width * 0.5
    rim_profile = [
        (radius * 0.76, -half_width * 0.70),
        (radius * 0.90, -half_width),
        (radius * 0.97, -half_width * 0.88),
        (radius * 0.995, -half_width * 0.28),
        (radius * 0.995, half_width * 0.28),
        (radius * 0.97, half_width * 0.88),
        (radius * 0.90, half_width),
        (radius * 0.76, half_width * 0.70),
        (radius * 0.72, half_width * 0.26),
        (radius * 0.72, -half_width * 0.26),
        (radius * 0.76, -half_width * 0.70),
    ]
    geom = LatheGeometry(rim_profile, segments=72)
    geom.merge(CylinderGeometry(hub_radius, hub_length, radial_segments=28))
    geom.merge(CylinderGeometry(hub_radius * 0.38, hub_length * 1.10, radial_segments=20))
    geom.merge(
        CylinderGeometry(hub_radius * 1.22, hub_length * 0.18, radial_segments=24).translate(
            0.0, 0.0, hub_length * 0.34
        )
    )
    geom.merge(
        CylinderGeometry(hub_radius * 1.22, hub_length * 0.18, radial_segments=24).translate(
            0.0, 0.0, -hub_length * 0.34
        )
    )

    spoke_inner = hub_radius * 0.92
    spoke_outer = radius * 0.74
    spoke_center = (spoke_inner + spoke_outer) * 0.5
    spoke_length = (spoke_outer - spoke_inner) + 0.035
    spoke_radius = width * 0.14
    for spoke_index in range(spoke_count):
        angle = (2.0 * pi * spoke_index) / spoke_count + (pi / spoke_count)
        spoke = CylinderGeometry(spoke_radius, spoke_length, radial_segments=14)
        spoke.rotate_y(pi / 2.0).rotate_z(angle).translate(
            spoke_center * cos(angle),
            spoke_center * sin(angle),
            0.0,
        )
        geom.merge(spoke)

    return geom


def _wheel_tire_mesh(*, radius: float, width: float):
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.972, -half_width * 0.86),
        (radius, -half_width * 0.86),
        (radius, half_width * 0.86),
        (radius * 0.972, half_width * 0.86),
        (radius * 0.964, half_width * 0.58),
        (radius * 0.964, -half_width * 0.58),
        (radius * 0.972, -half_width * 0.86),
    ]
    return LatheGeometry(tire_profile, segments=72)


def _carriage_cheek_mesh(*, thickness: float, trunnion_center: tuple[float, float], notch_radius: float):
    tx, tz = trunnion_center
    arc_points = [
        (tx + notch_radius * cos(angle), tz + notch_radius * sin(angle))
        for angle in [radians(deg) for deg in range(140, 39, -20)]
    ]
    outer_profile = [
        (-0.80, -0.02),
        (-0.70, 0.10),
        (-0.52, 0.18),
        (-0.26, 0.28),
        (tx - 0.14, 0.50),
        *arc_points,
        (tx + 0.22, 0.54),
        (0.67, 0.52),
        (0.74, 0.34),
        (0.67, 0.14),
        (0.50, 0.04),
        (0.10, 0.00),
        (-0.46, -0.01),
    ]
    return ExtrudeGeometry(outer_profile, thickness, center=True).rotate_x(pi / 2.0)


def _quoin_mesh(*, length: float, width: float, rear_height: float, front_height: float):
    profile = [
        (-length * 0.5, 0.0),
        (length * 0.5, 0.0),
        (length * 0.5, front_height),
        (-length * 0.5, rear_height),
    ]
    return ExtrudeGeometry(profile, width, center=True).rotate_x(pi / 2.0)


def _barrel_shell_mesh():
    outer_profile = [
        (0.02, -0.42),
        (0.16, -0.42),
        (0.19, -0.31),
        (0.19, -0.18),
        (0.18, 0.08),
        (0.165, 0.44),
        (0.145, 1.10),
        (0.133, 1.55),
        (0.148, 1.74),
        (0.160, 1.92),
    ]
    inner_profile = [
        (0.080, -0.08),
        (0.086, 0.02),
        (0.091, 1.82),
        (0.085, 1.90),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="napoleonic_field_cannon")

    carriage_green = model.material("carriage_green", rgba=(0.29, 0.34, 0.19, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.46, 0.31, 0.18, 1.0))
    aged_iron = model.material("aged_iron", rgba=(0.26, 0.27, 0.29, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.49, 0.41, 0.23, 1.0))

    wheel_radius = 0.72
    wheel_width = 0.12
    track_half = 0.72
    trunnion_x = 0.32
    trunnion_z = 0.38

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.42, 0.30, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, -0.02)),
        material=carriage_green,
        name="carriage_bed",
    )
    carriage.visual(
        Box((1.55, 0.18, 0.14)),
        origin=Origin(xyz=(-0.92, 0.0, -0.05)),
        material=carriage_green,
        name="trail_beam",
    )
    carriage.visual(
        Box((0.22, 0.26, 0.30)),
        origin=Origin(xyz=(-1.80, 0.0, 0.00)),
        material=carriage_green,
        name="trail_end",
    )
    carriage.visual(
        Box((0.24, 0.92, 0.18)),
        origin=Origin(xyz=(0.00, 0.0, -0.12)),
        material=carriage_green,
        name="axle_tree",
    )
    carriage.visual(
        Box((0.18, 0.64, 0.18)),
        origin=Origin(xyz=(0.42, 0.0, 0.00)),
        material=carriage_green,
        name="front_transom",
    )
    carriage.visual(
        Box((0.18, 0.50, 0.16)),
        origin=Origin(xyz=(-0.56, 0.0, -0.02)),
        material=carriage_green,
        name="rear_transom",
    )

    cheek_mesh = _save_mesh(
        "carriage_cheek",
        _carriage_cheek_mesh(thickness=0.05, trunnion_center=(trunnion_x, trunnion_z), notch_radius=0.061),
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, 0.31, 0.0)),
        material=carriage_green,
        name="left_cheek",
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -0.31, 0.0)),
        material=carriage_green,
        name="right_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.20),
        origin=Origin(xyz=(0.02, 0.51, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aged_iron,
        name="left_axle_stub",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.20),
        origin=Origin(xyz=(0.02, -0.51, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aged_iron,
        name="right_axle_stub",
    )
    carriage.visual(
        Box((0.34, 0.06, 0.08)),
        origin=Origin(xyz=(trunnion_x + 0.02, 0.31, trunnion_z + 0.11)),
        material=aged_iron,
        name="left_cap_square",
    )
    carriage.visual(
        Box((0.34, 0.06, 0.08)),
        origin=Origin(xyz=(trunnion_x + 0.02, -0.31, trunnion_z + 0.11)),
        material=aged_iron,
        name="right_cap_square",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.20, 1.20, 0.90)),
        mass=520.0,
        origin=Origin(xyz=(-0.70, 0.0, 0.10)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        _save_mesh(
            "left_wheel_wood",
            _wheel_wood_mesh(
                radius=wheel_radius,
                width=wheel_width,
                hub_radius=0.105,
                hub_length=0.20,
                spoke_count=12,
            ),
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wheel_wood,
        name="wheel_wood",
    )
    left_wheel.visual(
        _save_mesh("left_wheel_tire", _wheel_tire_mesh(radius=wheel_radius, width=wheel_width)),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aged_iron,
        name="tire_band",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=118.0,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        _save_mesh(
            "right_wheel_wood",
            _wheel_wood_mesh(
                radius=wheel_radius,
                width=wheel_width,
                hub_radius=0.105,
                hub_length=0.20,
                spoke_count=12,
            ),
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wheel_wood,
        name="wheel_wood",
    )
    right_wheel.visual(
        _save_mesh("right_wheel_tire", _wheel_tire_mesh(radius=wheel_radius, width=wheel_width)),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aged_iron,
        name="tire_band",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=118.0,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("cannon_barrel_shell", _barrel_shell_mesh()),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.205, length=0.055),
        origin=Origin(xyz=(-0.23, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="base_ring",
    )
    barrel.visual(
        Cylinder(radius=0.058, length=0.10),
        origin=Origin(xyz=(0.0, 0.235, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.058, length=0.10),
        origin=Origin(xyz=(0.0, -0.235, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.052, length=0.12),
        origin=Origin(xyz=(-0.47, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.068),
        origin=Origin(xyz=(-0.58, 0.0, 0.0)),
        material=gunmetal,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.153, length=0.07),
        origin=Origin(xyz=(1.89, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_ring",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((2.34, 0.40, 0.40)),
        mass=620.0,
        origin=Origin(xyz=(0.74, 0.0, 0.0)),
    )

    quoin = model.part("quoin")
    quoin.visual(
        _save_mesh("quoin_body", _quoin_mesh(length=0.32, width=0.22, rear_height=0.05, front_height=0.12)),
        material=wheel_wood,
        name="quoin_body",
    )
    quoin.inertial = Inertial.from_geometry(
        Box((0.32, 0.22, 0.12)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(0.02, track_half, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=12.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(0.02, -track_half, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=12.0),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(trunnion_x, 0.0, trunnion_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.6,
            lower=radians(-5.0),
            upper=radians(18.0),
        ),
    )
    model.articulation(
        "quoin_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=quoin,
        origin=Origin(xyz=(0.00, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.15,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    quoin = object_model.get_part("quoin")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    quoin_slide = object_model.get_articulation("quoin_slide")

    ctx.expect_gap(
        quoin,
        carriage,
        axis="z",
        positive_elem="quoin_body",
        negative_elem="carriage_bed",
        max_gap=0.002,
        max_penetration=0.0,
        name="quoin rests on the carriage bed",
    )
    ctx.expect_overlap(
        barrel,
        quoin,
        axes="xy",
        min_overlap=0.10,
        elem_a="barrel_shell",
        elem_b="quoin_body",
        name="quoin stays beneath the breech footprint",
    )
    ctx.expect_gap(
        barrel,
        quoin,
        axis="z",
        positive_elem="barrel_shell",
        negative_elem="quoin_body",
        min_gap=0.01,
        max_gap=0.08,
        name="quoin sits just below the breech",
    )
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="y",
        min_dist=1.40,
        max_dist=1.46,
        name="wheel track matches a field carriage stance",
    )

    muzzle_rest = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
    quoin_rest = ctx.part_world_position(quoin)
    with ctx.pose({barrel_elevation: barrel_elevation.motion_limits.upper}):
        muzzle_elevated = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
    with ctx.pose({quoin_slide: quoin_slide.motion_limits.upper}):
        quoin_extended = ctx.part_world_position(quoin)

    ctx.check(
        "barrel elevates upward at the trunnions",
        muzzle_rest is not None
        and muzzle_elevated is not None
        and muzzle_elevated[0][2] > muzzle_rest[0][2] + 0.22,
        details=f"rest={muzzle_rest}, elevated={muzzle_elevated}",
    )
    ctx.check(
        "quoin slides forward along the bed",
        quoin_rest is not None and quoin_extended is not None and quoin_extended[0] > quoin_rest[0] + 0.12,
        details=f"rest={quoin_rest}, extended={quoin_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
