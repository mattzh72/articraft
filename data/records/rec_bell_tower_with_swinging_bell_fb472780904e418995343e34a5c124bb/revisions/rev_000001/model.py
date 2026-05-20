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
    mesh_from_geometry,
    section_loft,
)


def _rect_loop(size_x: float, size_y: float, z: float) -> list[tuple[float, float, float]]:
    hx = size_x * 0.5
    hy = size_y * 0.5
    return [
        (-hx, -hy, z),
        (0.0, -hy, z),
        (hx, -hy, z),
        (hx, 0.0, z),
        (hx, hy, z),
        (0.0, hy, z),
        (-hx, hy, z),
        (-hx, 0.0, z),
    ]


def _build_roof_shell():
    return section_loft(
        [
            _rect_loop(3.55, 3.55, 0.00),
            _rect_loop(3.10, 3.10, 0.12),
            _rect_loop(2.50, 2.50, 0.55),
            _rect_loop(0.48, 0.48, 1.06),
        ]
    )


def _build_bonsho_shell():
    outer_profile = [
        (0.08, 0.00),
        (0.16, -0.10),
        (0.25, -0.28),
        (0.36, -0.58),
        (0.46, -0.95),
        (0.52, -1.26),
        (0.55, -1.50),
        (0.52, -1.64),
    ]
    inner_profile = [
        (0.03, -0.02),
        (0.08, -0.12),
        (0.14, -0.30),
        (0.24, -0.60),
        (0.33, -0.95),
        (0.39, -1.25),
        (0.43, -1.49),
        (0.45, -1.58),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="japanese_temple_bell_pavilion")

    timber = model.material("timber", rgba=(0.44, 0.30, 0.19, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.30, 0.20, 0.12, 1.0))
    roof_tile = model.material("roof_tile", rgba=(0.18, 0.18, 0.20, 1.0))
    stone = model.material("stone", rgba=(0.58, 0.58, 0.56, 1.0))
    bell_iron = model.material("bell_iron", rgba=(0.18, 0.22, 0.20, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.50, 0.37, 0.23, 1.0))
    iron_hardware = model.material("iron_hardware", rgba=(0.24, 0.24, 0.25, 1.0))

    pavilion = model.part("pavilion")

    post_half_span = 1.25
    post_size = 0.18
    post_height = 2.50
    beam_z = 2.53

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            x_pos = x_sign * post_half_span
            y_pos = y_sign * post_half_span
            pavilion.visual(
                Box((0.32, 0.32, 0.12)),
                origin=Origin(xyz=(x_pos, y_pos, 0.06)),
                material=stone,
            )
            pavilion.visual(
                Box((post_size, post_size, post_height)),
                origin=Origin(xyz=(x_pos, y_pos, post_height * 0.5)),
                material=timber,
            )

    pavilion.visual(
        Box((0.24, 2.72, 0.20)),
        origin=Origin(xyz=(post_half_span, 0.0, beam_z)),
        material=dark_timber,
        name="front_beam",
    )
    pavilion.visual(
        Box((0.24, 2.72, 0.20)),
        origin=Origin(xyz=(-post_half_span, 0.0, beam_z)),
        material=dark_timber,
        name="rear_beam",
    )
    pavilion.visual(
        Box((2.72, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, post_half_span, beam_z)),
        material=dark_timber,
        name="right_beam",
    )
    pavilion.visual(
        Box((2.72, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, -post_half_span, beam_z)),
        material=dark_timber,
        name="left_beam",
    )

    pavilion.visual(
        Box((0.24, 1.07, 0.24)),
        origin=Origin(xyz=(0.0, -0.745, 2.49)),
        material=dark_timber,
        name="bell_cross_beam_left",
    )
    pavilion.visual(
        Box((0.24, 1.07, 0.24)),
        origin=Origin(xyz=(0.0, 0.745, 2.49)),
        material=dark_timber,
        name="bell_cross_beam_right",
    )
    pavilion.visual(
        Box((0.18, 0.54, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.59)),
        material=dark_timber,
        name="bell_yoke_cap",
    )
    pavilion.visual(
        Box((2.32, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, post_half_span, 1.02)),
        material=dark_timber,
        name="lower_right_rail",
    )
    pavilion.visual(
        Box((2.32, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, -post_half_span, 1.02)),
        material=dark_timber,
        name="lower_left_rail",
    )
    pavilion.visual(
        Box((0.14, 2.32, 0.16)),
        origin=Origin(xyz=(post_half_span, 0.0, 1.02)),
        material=dark_timber,
        name="lower_front_rail",
    )
    pavilion.visual(
        Box((0.14, 2.32, 0.16)),
        origin=Origin(xyz=(-post_half_span, 0.0, 1.02)),
        material=dark_timber,
        name="lower_rear_rail",
    )

    pavilion.visual(
        Box((2.88, 2.88, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.63)),
        material=dark_timber,
        name="roof_plenum",
    )
    pavilion.visual(
        mesh_from_geometry(_build_roof_shell(), "roof_shell"),
        origin=Origin(xyz=(0.0, 0.0, 2.60)),
        material=roof_tile,
        name="roof_shell",
    )
    pavilion.visual(
        Cylinder(radius=0.09, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 3.80)),
        material=roof_tile,
    )
    pavilion.visual(
        Box((0.16, 0.16, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 3.95)),
        material=roof_tile,
    )

    pavilion.visual(
        Box((0.12, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, -0.19, 2.37)),
        material=iron_hardware,
        name="bell_bearing_left",
    )
    pavilion.visual(
        Box((0.12, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, 0.19, 2.37)),
        material=iron_hardware,
        name="bell_bearing_right",
    )

    pavilion.visual(
        Box((0.12, 0.08, 0.14)),
        origin=Origin(xyz=(1.08, -0.60, 2.36)),
        material=iron_hardware,
        name="striker_bearing_left",
    )
    pavilion.visual(
        Box((0.12, 0.08, 0.14)),
        origin=Origin(xyz=(1.08, 0.60, 2.36)),
        material=iron_hardware,
        name="striker_bearing_right",
    )
    pavilion.inertial = Inertial.from_geometry(
        Box((3.55, 3.55, 4.05)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 2.02)),
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.025, length=0.30),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=iron_hardware,
        name="pivot_axle",
    )
    bell.visual(
        Box((0.06, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=iron_hardware,
        name="hanger_block",
    )
    bell.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(0.0, -0.07, -0.13)),
        material=iron_hardware,
    )
    bell.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(0.0, 0.07, -0.13)),
        material=iron_hardware,
    )
    bell.visual(
        mesh_from_geometry(_build_bonsho_shell(), "bonsho_shell"),
        material=bell_iron,
        name="bell_shell",
    )
    bell.visual(
        Box((0.04, 0.34, 0.26)),
        origin=Origin(xyz=(0.50, 0.0, -1.02)),
        material=bell_iron,
        name="strike_pad",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.54, length=1.66),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.84)),
    )

    striker = model.part("striker")
    striker.visual(
        Cylinder(radius=0.024, length=1.12),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=iron_hardware,
        name="pivot_shaft",
    )
    striker.visual(
        Box((0.10, 1.00, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=iron_hardware,
    )
    striker.visual(
        Box((0.08, 0.10, 0.92)),
        origin=Origin(xyz=(0.0, -0.46, -0.48)),
        material=weathered_wood,
        name="left_arm",
    )
    striker.visual(
        Box((0.08, 0.10, 0.92)),
        origin=Origin(xyz=(0.0, 0.46, -0.48)),
        material=weathered_wood,
        name="right_arm",
    )
    striker.visual(
        Box((0.14, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -0.55, -0.95)),
        material=weathered_wood,
    )
    striker.visual(
        Box((0.14, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.55, -0.95)),
        material=weathered_wood,
    )
    striker.visual(
        Cylinder(radius=0.09, length=1.60),
        origin=Origin(xyz=(0.0, 0.0, -1.02), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=weathered_wood,
        name="main_log",
    )
    striker.inertial = Inertial.from_geometry(
        Box((0.22, 1.60, 1.12)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, -0.62)),
    )

    model.articulation(
        "pavilion_to_bell",
        ArticulationType.REVOLUTE,
        parent=pavilion,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 2.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-0.22,
            upper=0.22,
        ),
    )
    model.articulation(
        "pavilion_to_striker",
        ArticulationType.REVOLUTE,
        parent=pavilion,
        child=striker,
        origin=Origin(xyz=(1.08, 0.0, 2.36)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pavilion = object_model.get_part("pavilion")
    bell = object_model.get_part("bell")
    striker = object_model.get_part("striker")
    bell_joint = object_model.get_articulation("pavilion_to_bell")
    striker_joint = object_model.get_articulation("pavilion_to_striker")

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
        bell,
        pavilion,
        elem_a="pivot_axle",
        elem_b="bell_bearing_left",
        name="bell axle bears on left hanger block",
    )
    ctx.expect_contact(
        bell,
        pavilion,
        elem_a="pivot_axle",
        elem_b="bell_bearing_right",
        name="bell axle bears on right hanger block",
    )
    ctx.expect_contact(
        striker,
        pavilion,
        elem_a="pivot_shaft",
        elem_b="striker_bearing_left",
        name="striker shaft bears on left hanger block",
    )
    ctx.expect_contact(
        striker,
        pavilion,
        elem_a="pivot_shaft",
        elem_b="striker_bearing_right",
        name="striker shaft bears on right hanger block",
    )

    with ctx.pose({bell_joint: 0.0, striker_joint: 0.0}):
        ctx.expect_gap(
            striker,
            bell,
            axis="x",
            positive_elem="main_log",
            negative_elem="strike_pad",
            min_gap=0.35,
            max_gap=0.60,
            name="striker rests clear of the bell strike pad",
        )

    with ctx.pose({striker_joint: 0.40}):
        ctx.expect_gap(
            striker,
            bell,
            axis="x",
            positive_elem="main_log",
            negative_elem="strike_pad",
            min_gap=0.03,
            max_gap=0.18,
            name="striker swings inward toward the bell without penetrating it",
        )

    bell_rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    striker_rest_aabb = ctx.part_element_world_aabb(striker, elem="main_log")
    with ctx.pose({bell_joint: 0.15}):
        bell_swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({striker_joint: 0.40}):
        striker_swung_aabb = ctx.part_element_world_aabb(striker, elem="main_log")

    bell_rest_center = _aabb_center(bell_rest_aabb)
    bell_swung_center = _aabb_center(bell_swung_aabb)
    striker_rest_center = _aabb_center(striker_rest_aabb)
    striker_swung_center = _aabb_center(striker_swung_aabb)

    ctx.check(
        "bell swings around the center y-axis pivot",
        bell_rest_center is not None
        and bell_swung_center is not None
        and bell_swung_center[0] < bell_rest_center[0] - 0.08,
        details=f"rest={bell_rest_center}, swung={bell_swung_center}",
    )
    ctx.check(
        "striker pendulum swings toward the bell",
        striker_rest_center is not None
        and striker_swung_center is not None
        and striker_swung_center[0] < striker_rest_center[0] - 0.25,
        details=f"rest={striker_rest_center}, swung={striker_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
