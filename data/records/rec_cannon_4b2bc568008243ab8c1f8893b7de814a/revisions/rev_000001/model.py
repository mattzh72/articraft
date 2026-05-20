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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_spoked_wheel(
    part,
    mesh_prefix: str,
    *,
    wheel_radius: float,
    wheel_width: float,
    spoke_count: int,
    hub_radius: float,
    rim_radius: float,
    tire_radius: float,
    wood_material,
    steel_material,
    dark_steel,
) -> None:
    wheel_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    wood_rim = _save_mesh(
        f"{mesh_prefix}_wood_rim",
        TorusGeometry(
            radius=rim_radius,
            tube=0.034,
            radial_segments=18,
            tubular_segments=72,
        ).rotate_y(math.pi / 2.0),
    )
    steel_tire = _save_mesh(
        f"{mesh_prefix}_steel_tire",
        TorusGeometry(
            radius=tire_radius,
            tube=0.012,
            radial_segments=16,
            tubular_segments=80,
        ).rotate_y(math.pi / 2.0),
    )
    part.visual(wood_rim, material=wood_material, name="wood_rim")
    part.visual(steel_tire, material=steel_material, name="steel_tire")

    part.visual(
        Cylinder(radius=hub_radius, length=wheel_width),
        origin=wheel_axis,
        material=dark_steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=hub_radius * 1.35, length=wheel_width * 0.22),
        origin=Origin(
            xyz=(-wheel_width * 0.32, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wood_material,
        name="inner_hub_flange",
    )
    part.visual(
        Cylinder(radius=hub_radius * 1.25, length=wheel_width * 0.18),
        origin=Origin(
            xyz=(wheel_width * 0.34, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wood_material,
        name="outer_hub_flange",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.34, length=wheel_width * 0.72),
        origin=wheel_axis,
        material=steel_material,
        name="axle_bore_collarette",
    )

    spoke_start = hub_radius * 0.86
    spoke_end = rim_radius * 1.02
    for spoke_index in range(spoke_count):
        angle = (2.0 * math.pi * spoke_index) / spoke_count
        inner = (
            0.0,
            math.cos(angle) * spoke_start,
            math.sin(angle) * spoke_start,
        )
        outer = (
            0.0,
            math.cos(angle) * spoke_end,
            math.sin(angle) * spoke_end,
        )
        _add_member(
            part,
            inner,
            outer,
            radius=0.017,
            material=wood_material,
            name=f"spoke_{spoke_index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_artillery_gun")

    carriage_olive = model.material("carriage_olive", rgba=(0.39, 0.43, 0.28, 1.0))
    barrel_gunmetal = model.material("barrel_gunmetal", rgba=(0.25, 0.27, 0.29, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.54, 0.56, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.55, 0.42, 0.25, 1.0))
    muzzle_black = model.material("muzzle_black", rgba=(0.05, 0.05, 0.06, 1.0))

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.72, 2.45, 1.18)),
        mass=480.0,
        origin=Origin(xyz=(0.0, -0.20, 0.59)),
    )

    carriage.visual(
        Box((0.82, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, -0.02, 0.62)),
        material=carriage_olive,
        name="axle_bed",
    )
    carriage.visual(
        Box((0.66, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, -0.16, 0.72)),
        material=carriage_olive,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.62, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.56, 0.62)),
        material=carriage_olive,
        name="front_transom",
    )
    carriage.visual(
        Box((0.56, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.12, 0.64)),
        material=carriage_olive,
        name="lower_cradle_tie",
    )
    carriage.visual(
        Box((0.08, 0.80, 0.36)),
        origin=Origin(xyz=(0.32, 0.18, 0.80)),
        material=carriage_olive,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.08, 0.80, 0.36)),
        origin=Origin(xyz=(-0.32, 0.18, 0.80)),
        material=carriage_olive,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.12, 0.10, 0.22)),
        origin=Origin(xyz=(0.24, 0.04, 0.70)),
        material=carriage_olive,
        name="left_saddle_block",
    )
    carriage.visual(
        Box((0.12, 0.10, 0.22)),
        origin=Origin(xyz=(-0.24, 0.04, 0.70)),
        material=carriage_olive,
        name="right_saddle_block",
    )
    carriage.visual(
        Cylinder(radius=0.08, length=0.20),
        origin=Origin(xyz=(0.51, -0.02, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_axle_stub",
    )
    carriage.visual(
        Cylinder(radius=0.08, length=0.20),
        origin=Origin(xyz=(-0.51, -0.02, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_axle_stub",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.78),
        origin=Origin(xyz=(0.0, -0.02, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_tube",
    )
    carriage.visual(
        Box((0.12, 1.46, 0.12)),
        origin=Origin(xyz=(0.39, -0.88, 0.55), rpy=(0.0, 0.0, 0.19)),
        material=carriage_olive,
        name="left_trail",
    )
    carriage.visual(
        Box((0.12, 1.46, 0.12)),
        origin=Origin(xyz=(-0.39, -0.88, 0.55), rpy=(0.0, 0.0, -0.19)),
        material=carriage_olive,
        name="right_trail",
    )
    carriage.visual(
        Box((0.18, 0.05, 0.24)),
        origin=Origin(xyz=(0.52, -1.57, 0.49), rpy=(0.0, 0.0, 0.19)),
        material=steel,
        name="left_spade",
    )
    carriage.visual(
        Box((0.18, 0.05, 0.24)),
        origin=Origin(xyz=(-0.52, -1.57, 0.49), rpy=(0.0, 0.0, -0.19)),
        material=steel,
        name="right_spade",
    )
    _add_member(
        carriage,
        (0.26, -0.08, 0.66),
        (0.30, 0.52, 0.70),
        radius=0.035,
        material=carriage_olive,
        name="left_front_brace",
    )
    _add_member(
        carriage,
        (-0.26, -0.08, 0.66),
        (-0.30, 0.52, 0.70),
        radius=0.035,
        material=carriage_olive,
        name="right_front_brace",
    )
    _add_member(
        carriage,
        (0.22, -0.12, 0.68),
        (0.33, -0.46, 0.58),
        radius=0.030,
        material=carriage_olive,
        name="left_trail_brace",
    )
    _add_member(
        carriage,
        (-0.22, -0.12, 0.68),
        (-0.33, -0.46, 0.58),
        radius=0.030,
        material=carriage_olive,
        name="right_trail_brace",
    )

    barrel = model.part("barrel")
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=1.78),
        mass=235.0,
        origin=Origin(xyz=(0.0, 0.66, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    barrel_profile = [
        (0.11, -0.22),
        (0.15, -0.20),
        (0.16, -0.12),
        (0.16, -0.02),
        (0.13, 0.06),
        (0.115, 0.16),
        (0.102, 0.34),
        (0.094, 0.58),
        (0.084, 0.92),
        (0.074, 1.24),
        (0.066, 1.48),
        (0.062, 1.58),
        (0.074, 1.61),
        (0.072, 1.64),
        (0.0, 1.64),
    ]
    barrel_shell = _save_mesh(
        "artillery_barrel_shell",
        LatheGeometry(barrel_profile, segments=72).rotate_x(-math.pi / 2.0),
    )
    barrel.visual(
        barrel_shell,
        material=barrel_gunmetal,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.065, length=0.56),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_gunmetal,
        name="trunnion_crosshead",
    )
    barrel.visual(
        Cylinder(radius=0.145, length=0.18),
        origin=Origin(xyz=(0.0, 0.03, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=barrel_gunmetal,
        name="breech_ring",
    )
    barrel.visual(
        Cylinder(radius=0.082, length=0.09),
        origin=Origin(xyz=(0.0, 1.57, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=barrel_gunmetal,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.038, length=0.08),
        origin=Origin(xyz=(0.0, 1.60, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=muzzle_black,
        name="muzzle_bore",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.56, length=0.18),
        mass=46.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        left_wheel,
        "artillery_left_wheel",
        wheel_radius=0.56,
        wheel_width=0.18,
        spoke_count=12,
        hub_radius=0.105,
        rim_radius=0.48,
        tire_radius=0.515,
        wood_material=wheel_wood,
        steel_material=steel,
        dark_steel=dark_steel,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.56, length=0.18),
        mass=46.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        right_wheel,
        "artillery_right_wheel",
        wheel_radius=0.56,
        wheel_width=0.18,
        spoke_count=12,
        hub_radius=0.105,
        rim_radius=0.48,
        tire_radius=0.515,
        wood_material=wheel_wood,
        steel_material=steel,
        dark_steel=dark_steel,
    )

    model.articulation(
        "carriage_to_barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.28, 0.84)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.7,
            lower=-0.12,
            upper=0.65,
        ),
    )
    model.articulation(
        "carriage_to_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(0.70, -0.02, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=16.0),
    )
    model.articulation(
        "carriage_to_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.70, -0.02, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    barrel_elevation = object_model.get_articulation("carriage_to_barrel_elevation")
    left_spin = object_model.get_articulation("carriage_to_left_wheel_spin")
    right_spin = object_model.get_articulation("carriage_to_right_wheel_spin")

    ctx.check(
        "barrel elevation uses the trunnion axis",
        barrel_elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(barrel_elevation.axis) == (1.0, 0.0, 0.0),
        details=f"type={barrel_elevation.articulation_type}, axis={barrel_elevation.axis}",
    )
    ctx.check(
        "wheel joints are continuous axle spins",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left=({left_spin.articulation_type}, {left_spin.axis}), "
            f"right=({right_spin.articulation_type}, {right_spin.axis})"
        ),
    )

    with ctx.pose({barrel_elevation: 0.0}):
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="trunnion_crosshead",
            contact_tol=0.002,
            name="barrel trunnion crosshead seats against the carriage cheeks",
        )
        ctx.expect_contact(
            left_wheel,
            carriage,
            elem_a="hub_shell",
            elem_b="left_axle_stub",
            contact_tol=0.002,
            name="left wheel hub seats on the left axle stub",
        )
        ctx.expect_contact(
            right_wheel,
            carriage,
            elem_a="hub_shell",
            elem_b="right_axle_stub",
            contact_tol=0.002,
            name="right wheel hub seats on the right axle stub",
        )
        ctx.expect_origin_distance(
            left_wheel,
            right_wheel,
            axes="x",
            min_dist=1.35,
            name="wheels are spaced across the wide carriage",
        )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
    with ctx.pose({barrel_elevation: barrel_elevation.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
    ctx.check(
        "barrel elevates upward",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.45,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    left_rest = ctx.part_world_position(left_wheel)
    with ctx.pose({left_spin: math.pi / 2.0}):
        left_spun = ctx.part_world_position(left_wheel)
    ctx.check(
        "left wheel spins in place about its hub",
        left_rest is not None
        and left_spun is not None
        and abs(left_rest[0] - left_spun[0]) < 1e-6
        and abs(left_rest[1] - left_spun[1]) < 1e-6
        and abs(left_rest[2] - left_spun[2]) < 1e-6,
        details=f"rest={left_rest}, spun={left_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
