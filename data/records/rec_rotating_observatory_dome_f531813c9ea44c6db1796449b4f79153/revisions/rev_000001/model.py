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
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.68, 1.0))
    steel = model.material("steel", rgba=(0.39, 0.42, 0.46, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    dome_white = model.material("dome_white", rgba=(0.88, 0.90, 0.92, 1.0))
    rail_grey = model.material("rail_grey", rgba=(0.48, 0.50, 0.53, 1.0))

    base = model.part("base_ring")
    base.visual(
        Cylinder(radius=2.55, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="foundation_ring",
    )
    base.visual(
        Cylinder(radius=0.38, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_steel,
        name="central_plinth",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=2.18, tube=0.085, radial_segments=18, tubular_segments=72),
            "stationary_track_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=rail_grey,
        name="stationary_track_ring",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=2.02, tube=0.050, radial_segments=16, tubular_segments=72),
            "inner_guard_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=dark_steel,
        name="inner_guard_ring",
    )

    column_count = 8
    for index in range(column_count):
        angle = index * math.tau / column_count
        c = math.cos(angle)
        s = math.sin(angle)
        lower = (1.78 * c, 1.78 * s, 0.24)
        upper = (2.03 * c, 2.03 * s, 0.96)
        brace_top = (1.05 * c, 1.05 * s, 0.86)
        _add_member(base, lower, upper, 0.065, steel, name=f"column_{index}")
        _add_member(base, lower, brace_top, 0.035, dark_steel, name=f"brace_{index}")
        base.visual(
            Box((0.18, 0.12, 0.11)),
            origin=Origin(
                xyz=(2.10 * c, 2.10 * s, 1.02),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"roller_bracket_{index}",
        )
        base.visual(
            Cylinder(radius=0.045, length=0.18),
            origin=Origin(
                xyz=(2.18 * c, 2.18 * s, 1.135),
                rpy=(math.pi / 2.0, 0.0, angle + math.pi / 2.0),
            ),
            material=rail_grey,
            name=f"support_roller_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((5.2, 5.2, 1.35)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
    )

    dome = model.part("dome_shell")
    lower_shell = LatheGeometry.from_shell_profiles(
        [
            (2.14, 0.00),
            (2.14, 0.70),
            (2.07, 1.02),
            (1.86, 1.46),
        ],
        [
            (2.05, 0.05),
            (2.05, 0.68),
            (1.98, 0.98),
            (1.77, 1.40),
        ],
        segments=88,
    )
    dome.visual(
        mesh_from_geometry(lower_shell, "dome_lower_shell"),
        material=dome_white,
        name="lower_shell",
    )
    meridian_strip = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (1.82, 0.0, 1.46),
                (1.62, 0.0, 1.76),
                (1.26, 0.0, 2.12),
                (0.78, 0.0, 2.48),
                (0.18, 0.0, 2.82),
            ],
            profile=rounded_rect_profile(0.34, 0.05, radius=0.012, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
        ),
        "dome_meridian_strip",
    )
    upper_angles = (
        -2.95,
        -2.50,
        -2.05,
        -1.60,
        -1.15,
        -0.70,
        -0.42,
        0.42,
        0.70,
        1.15,
        1.60,
        2.05,
        2.50,
        2.95,
    )
    for index, angle in enumerate(upper_angles):
        dome.visual(
            meridian_strip,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=dome_white,
            name=f"upper_shell_strip_{index:02d}",
        )
    dome.visual(
        mesh_from_geometry(
            TorusGeometry(radius=2.13, tube=0.07, radial_segments=18, tubular_segments=80),
            "rotating_bogie_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="rotating_bogie_ring",
    )
    dome.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.90, tube=0.04, radial_segments=14, tubular_segments=72),
            "inside_stiffener_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=rail_grey,
        name="inside_stiffener_ring",
    )
    dome.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.82, tube=0.05, radial_segments=14, tubular_segments=72),
            "crown_transition_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        material=rail_grey,
        name="crown_transition_ring",
    )
    slit_cheek = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (1.83, 0.0, 1.47),
                (1.61, 0.0, 1.80),
                (1.28, 0.0, 2.16),
                (0.88, 0.0, 2.50),
                (0.40, 0.0, 2.76),
            ],
            profile=rounded_rect_profile(0.10, 0.07, radius=0.018, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "slit_cheek_rail",
    )
    dome.visual(
        slit_cheek,
        origin=Origin(rpy=(0.0, 0.0, 0.31)),
        material=dark_steel,
        name="left_slit_cheek",
    )
    dome.visual(
        slit_cheek,
        origin=Origin(rpy=(0.0, 0.0, -0.31)),
        material=dark_steel,
        name="right_slit_cheek",
    )
    dome.visual(
        Box((0.11, 0.07, 0.13)),
        origin=Origin(xyz=(1.72, 0.325, 1.55), rpy=(0.0, -0.68, 0.0)),
        material=dark_steel,
        name="left_hinge_support",
    )
    dome.visual(
        Box((0.11, 0.07, 0.13)),
        origin=Origin(xyz=(1.72, -0.325, 1.55), rpy=(0.0, -0.68, 0.0)),
        material=dark_steel,
        name="right_hinge_support",
    )
    dome.visual(
        Box((0.08, 0.62, 0.08)),
        origin=Origin(xyz=(1.66, 0.0, 1.48), rpy=(0.0, -0.68, 0.0)),
        material=rail_grey,
        name="shutter_sill",
    )
    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=2.2, length=2.95),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
    )

    shutter = model.part("shutter_leaf")
    shutter.visual(
        Cylinder(radius=0.032, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="leaf_hinge_barrel",
    )
    shutter.visual(
        Box((0.045, 0.58, 0.92)),
        origin=Origin(xyz=(-0.279, 0.0, 0.342), rpy=(0.0, -0.68, 0.0)),
        material=dome_white,
        name="leaf_panel",
    )
    shutter.visual(
        Box((0.085, 0.52, 0.12)),
        origin=Origin(xyz=(-0.519, 0.0, 0.636), rpy=(0.0, -0.68, 0.0)),
        material=rail_grey,
        name="leaf_top_stiffener",
    )
    shutter.visual(
        Box((0.050, 0.18, 0.10)),
        origin=Origin(xyz=(-0.150, 0.0, 0.185), rpy=(0.0, -0.68, 0.0)),
        material=rail_grey,
        name="leaf_inner_rib",
    )
    shutter.inertial = Inertial.from_geometry(
        Box((0.70, 0.62, 1.02)),
        mass=180.0,
        origin=Origin(xyz=(-0.28, 0.0, 0.35)),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.5),
    )
    model.articulation(
        "dome_to_shutter_leaf",
        ArticulationType.REVOLUTE,
        parent=dome,
        child=shutter,
        origin=Origin(xyz=(1.74, 0.0, 1.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_ring")
    dome = object_model.get_part("dome_shell")
    leaf = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("base_to_dome_rotation")
    shutter = object_model.get_articulation("dome_to_shutter_leaf")

    ctx.expect_origin_distance(
        base,
        dome,
        axes="xy",
        max_dist=1e-6,
        name="dome stays centered on the rotary stage",
    )
    ctx.expect_contact(
        leaf,
        dome,
        elem_a="leaf_hinge_barrel",
        elem_b="left_hinge_support",
        contact_tol=0.02,
        name="left hinge knuckle seats in the left support",
    )
    ctx.expect_contact(
        leaf,
        dome,
        elem_a="leaf_hinge_barrel",
        elem_b="right_hinge_support",
        contact_tol=0.02,
        name="right hinge knuckle seats in the right support",
    )
    ctx.expect_overlap(
        leaf,
        dome,
        axes="y",
        elem_a="leaf_panel",
        elem_b="shutter_sill",
        min_overlap=0.54,
        name="closed shutter spans the slit width at the sill",
    )

    rest_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
    with ctx.pose({shutter: 1.2}):
        open_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
    ctx.check(
        "shutter opens outward and upward",
        rest_leaf_aabb is not None
        and open_leaf_aabb is not None
        and open_leaf_aabb[1][0] > rest_leaf_aabb[1][0] + 0.30
        and open_leaf_aabb[1][2] > rest_leaf_aabb[1][2] + 0.06,
        details=f"rest={rest_leaf_aabb}, open={open_leaf_aabb}",
    )

    rest_leaf_pos = ctx.part_world_position(leaf)
    with ctx.pose({dome_rotation: math.pi / 2.0}):
        quarter_turn_leaf_pos = ctx.part_world_position(leaf)
    ctx.check(
        "dome rotation carries the shutter around the vertical axis",
        rest_leaf_pos is not None
        and quarter_turn_leaf_pos is not None
        and abs(quarter_turn_leaf_pos[0]) < 0.08
        and quarter_turn_leaf_pos[1] > rest_leaf_pos[0] - 0.08,
        details=f"rest={rest_leaf_pos}, quarter_turn={quarter_turn_leaf_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
