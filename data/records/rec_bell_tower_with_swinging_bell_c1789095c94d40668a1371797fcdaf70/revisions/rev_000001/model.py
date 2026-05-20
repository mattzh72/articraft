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


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    return [
        (-half_w, 0.0),
        (half_w, 0.0),
        (half_w, height),
        (-half_w, height),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 20,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _arched_opening_profile(
    *,
    width: float,
    base_z: float,
    spring_z: float,
    arch_segments: int = 20,
) -> list[tuple[float, float]]:
    radius = width * 0.5
    points: list[tuple[float, float]] = [
        (width * 0.5, base_z),
        (width * 0.5, spring_z),
    ]
    for index in range(1, arch_segments):
        angle = index * math.pi / arch_segments
        points.append((radius * math.cos(angle), spring_z + radius * math.sin(angle)))
    points.extend(
        [
            (-width * 0.5, spring_z),
            (-width * 0.5, base_z),
        ]
    )
    return points


def _build_arch_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    opening_width: float,
    opening_base_z: float,
    opening_spring_z: float,
):
    panel = ExtrudeWithHolesGeometry(
        _rect_profile(width, height),
        [
            _arched_opening_profile(
                width=opening_width,
                base_z=opening_base_z,
                spring_z=opening_spring_z,
            )
        ],
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    panel.rotate_x(math.pi / 2.0)
    return panel


def _build_bell_shell_mesh():
    outer_profile = [
        (0.10, 0.00),
        (0.18, -0.06),
        (0.30, -0.16),
        (0.44, -0.36),
        (0.54, -0.66),
        (0.60, -0.96),
        (0.62, -1.12),
        (0.61, -1.18),
    ]
    inner_profile = [
        (0.04, -0.03),
        (0.09, -0.09),
        (0.18, -0.20),
        (0.32, -0.40),
        (0.43, -0.69),
        (0.51, -0.97),
        (0.55, -1.10),
        (0.54, -1.14),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_campanile")

    brick = model.material("brick", rgba=(0.58, 0.25, 0.18, 1.0))
    stone = model.material("stone", rgba=(0.72, 0.68, 0.61, 1.0))
    timber = model.material("timber", rgba=(0.42, 0.28, 0.15, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.30, 0.20, 0.11, 1.0))
    iron = model.material("iron", rgba=(0.20, 0.22, 0.24, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.14, 0.15, 0.16, 1.0))

    arch_panel_mesh = mesh_from_geometry(
        _build_arch_panel_mesh(
            width=3.44,
            height=2.95,
            thickness=0.28,
            opening_width=1.86,
            opening_base_z=0.35,
            opening_spring_z=1.55,
        ),
        "campanile_arch_panel",
    )
    bell_shell_mesh = mesh_from_geometry(_build_bell_shell_mesh(), "campanile_bell_shell")

    tower = model.part("tower")
    tower.visual(
        Box((5.40, 5.40, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=stone,
        name="base_plinth",
    )
    tower.visual(
        Box((4.80, 4.80, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=stone,
        name="upper_plinth",
    )

    shaft_bottom_z = 0.79
    shaft_height = 12.45
    shaft_center_z = shaft_bottom_z + shaft_height * 0.5
    tower.visual(
        Box((0.55, 4.20, shaft_height)),
        origin=Origin(xyz=(1.825, 0.0, shaft_center_z)),
        material=brick,
        name="east_shaft_wall",
    )
    tower.visual(
        Box((0.55, 4.20, shaft_height)),
        origin=Origin(xyz=(-1.825, 0.0, shaft_center_z)),
        material=brick,
        name="west_shaft_wall",
    )
    tower.visual(
        Box((4.20, 0.55, shaft_height)),
        origin=Origin(xyz=(0.0, 1.825, shaft_center_z)),
        material=brick,
        name="north_shaft_wall",
    )
    tower.visual(
        Box((4.20, 0.55, shaft_height)),
        origin=Origin(xyz=(0.0, -1.825, shaft_center_z)),
        material=brick,
        name="south_shaft_wall",
    )

    tower.visual(
        Box((4.60, 4.60, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 13.41)),
        material=stone,
        name="belfry_floor",
    )

    corner_pier_height = 3.10
    corner_pier_center_z = 13.58 + corner_pier_height * 0.5
    for sx in (-1.99, 1.99):
        for sy in (-1.99, 1.99):
            tower.visual(
                Box((0.62, 0.62, corner_pier_height)),
                origin=Origin(xyz=(sx, sy, corner_pier_center_z)),
                material=brick,
            )

    face_plane = 2.16
    panel_bottom_z = 13.62
    tower.visual(
        arch_panel_mesh,
        origin=Origin(xyz=(0.0, face_plane, panel_bottom_z)),
        material=brick,
        name="front_arch_panel",
    )
    tower.visual(
        arch_panel_mesh,
        origin=Origin(xyz=(0.0, -face_plane, panel_bottom_z), rpy=(0.0, 0.0, math.pi)),
        material=brick,
        name="rear_arch_panel",
    )
    tower.visual(
        arch_panel_mesh,
        origin=Origin(xyz=(face_plane, 0.0, panel_bottom_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=brick,
        name="right_arch_panel",
    )
    tower.visual(
        arch_panel_mesh,
        origin=Origin(xyz=(-face_plane, 0.0, panel_bottom_z), rpy=(0.0, 0.0, -math.pi / 2.0)),
        material=brick,
        name="left_arch_panel",
    )

    column_radius = 0.15
    column_length = 1.20
    column_center_z = 13.97 + column_length * 0.5
    capital_center_z = 15.245
    for sign in (-1.0, 1.0):
        x_pos = sign * 0.78
        tower.visual(
            Cylinder(radius=column_radius, length=column_length),
            origin=Origin(xyz=(x_pos, face_plane, column_center_z)),
            material=stone,
        )
        tower.visual(
            Cylinder(radius=column_radius, length=column_length),
            origin=Origin(xyz=(x_pos, -face_plane, column_center_z)),
            material=stone,
        )
        tower.visual(
            Cylinder(radius=column_radius, length=column_length),
            origin=Origin(xyz=(face_plane, x_pos, column_center_z)),
            material=stone,
        )
        tower.visual(
            Cylinder(radius=column_radius, length=column_length),
            origin=Origin(xyz=(-face_plane, x_pos, column_center_z)),
            material=stone,
        )

        tower.visual(
            Box((0.32, 0.24, 0.15)),
            origin=Origin(xyz=(x_pos, face_plane, capital_center_z)),
            material=stone,
        )
        tower.visual(
            Box((0.32, 0.24, 0.15)),
            origin=Origin(xyz=(x_pos, -face_plane, capital_center_z)),
            material=stone,
        )
        tower.visual(
            Box((0.24, 0.32, 0.15)),
            origin=Origin(xyz=(face_plane, x_pos, capital_center_z)),
            material=stone,
        )
        tower.visual(
            Box((0.24, 0.32, 0.15)),
            origin=Origin(xyz=(-face_plane, x_pos, capital_center_z)),
            material=stone,
        )

    tower.visual(
        Box((4.95, 4.95, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 16.77)),
        material=stone,
        name="cornice_ring",
    )
    tower.visual(
        Box((4.30, 4.30, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 17.02)),
        material=brick,
        name="ceiling_slab",
    )
    tower.visual(
        Box((2.60, 0.34, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 16.78)),
        material=dark_timber,
        name="tie_beam",
    )
    tower.visual(
        Box((0.10, 0.18, 0.22)),
        origin=Origin(xyz=(-1.25, 0.0, 16.56)),
        material=iron,
        name="left_bearing_block",
    )
    tower.visual(
        Box((0.10, 0.18, 0.22)),
        origin=Origin(xyz=(1.25, 0.0, 16.56)),
        material=iron,
        name="right_bearing_block",
    )
    tower.inertial = Inertial.from_geometry(
        Box((5.40, 5.40, 17.30)),
        mass=52000.0,
        origin=Origin(xyz=(0.0, 0.0, 8.65)),
    )

    bell_assembly = model.part("bell_assembly")
    bell_assembly.visual(
        Box((2.10, 0.26, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=dark_timber,
        name="yoke_beam",
    )
    bell_assembly.visual(
        Cylinder(radius=0.08, length=0.10),
        origin=Origin(xyz=(-1.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="left_journal",
    )
    bell_assembly.visual(
        Cylinder(radius=0.08, length=0.10),
        origin=Origin(xyz=(1.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="right_journal",
    )
    bell_assembly.visual(
        Box((0.12, 0.12, 0.22)),
        origin=Origin(xyz=(-1.05, 0.0, -0.08)),
        material=iron,
        name="left_yoke_cheek",
    )
    bell_assembly.visual(
        Box((0.12, 0.12, 0.22)),
        origin=Origin(xyz=(1.05, 0.0, -0.08)),
        material=iron,
        name="right_yoke_cheek",
    )
    bell_assembly.visual(
        Box((0.16, 0.20, 0.94)),
        origin=Origin(xyz=(-0.44, 0.0, -0.59)),
        material=timber,
        name="left_yoke_arm",
    )
    bell_assembly.visual(
        Box((0.16, 0.20, 0.94)),
        origin=Origin(xyz=(0.44, 0.0, -0.59)),
        material=timber,
        name="right_yoke_arm",
    )
    bell_assembly.visual(
        Box((1.00, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.15, -0.88)),
        material=timber,
        name="front_yoke_spreader",
    )
    bell_assembly.visual(
        Box((1.00, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.15, -0.88)),
        material=timber,
        name="rear_yoke_spreader",
    )
    bell_assembly.visual(
        Box((0.88, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.76)),
        material=iron,
        name="crown_block",
    )
    bell_assembly.visual(
        Box((0.12, 0.12, 0.22)),
        origin=Origin(xyz=(-0.20, 0.0, -0.91)),
        material=iron,
        name="left_crown_lug",
    )
    bell_assembly.visual(
        Box((0.12, 0.12, 0.22)),
        origin=Origin(xyz=(0.20, 0.0, -0.91)),
        material=iron,
        name="right_crown_lug",
    )
    bell_assembly.visual(
        Box((0.14, 0.12, 0.28)),
        origin=Origin(xyz=(0.0, -0.16, -0.915)),
        material=iron,
        name="front_canon",
    )
    bell_assembly.visual(
        Box((0.14, 0.12, 0.28)),
        origin=Origin(xyz=(0.0, 0.16, -0.915)),
        material=iron,
        name="rear_canon",
    )
    bell_assembly.visual(
        Box((0.03, 0.12, 0.26)),
        origin=Origin(xyz=(-0.125, 0.0, -0.89)),
        material=iron,
        name="left_clapper_hanger",
    )
    bell_assembly.visual(
        Box((0.03, 0.12, 0.26)),
        origin=Origin(xyz=(0.125, 0.0, -0.89)),
        material=iron,
        name="right_clapper_hanger",
    )
    bell_assembly.visual(
        bell_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
        material=dark_iron,
        name="bell_shell",
    )
    bell_assembly.inertial = Inertial.from_geometry(
        Box((2.40, 1.30, 2.25)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_barrel",
    )
    clapper.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="left_pivot_stub",
    )
    clapper.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="right_pivot_stub",
    )
    clapper.visual(
        Cylinder(radius=0.014, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, -0.328)),
        material=iron,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.693)),
        material=dark_iron,
        name="clapper_bob",
    )
    clapper.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.90)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, -0.44)),
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_assembly,
        origin=Origin(xyz=(0.0, 0.0, 16.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.9,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell_assembly,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.89)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=2.4,
            lower=-0.14,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell_assembly = object_model.get_part("bell_assembly")
    clapper = object_model.get_part("clapper")
    bell_swing = object_model.get_articulation("tower_to_bell")
    clapper_swing = object_model.get_articulation("bell_to_clapper")

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

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulation_clearance_sweep",
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=16,
        name="supported_pose_sweep",
    )

    ctx.check(
        "bell_swing_axis_is_horizontal_x",
        tuple(round(value, 3) for value in bell_swing.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis bell swing, got {bell_swing.axis}",
    )
    ctx.check(
        "clapper_swing_axis_is_horizontal_x",
        tuple(round(value, 3) for value in clapper_swing.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis clapper swing, got {clapper_swing.axis}",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    if tower_aabb is not None:
        tower_size = tuple(tower_aabb[1][axis] - tower_aabb[0][axis] for axis in range(3))
        ctx.check(
            "campanile_realistic_height",
            16.5 <= tower_size[2] <= 18.0,
            f"tower height should read as a real campanile, got {tower_size[2]:.3f} m",
        )
        ctx.check(
            "campanile_square_plan",
            abs(tower_size[0] - tower_size[1]) <= 0.15 and 5.0 <= tower_size[0] <= 5.6,
            f"expected square campanile footprint around 5.4 m, got {tower_size[0]:.3f} x {tower_size[1]:.3f} m",
        )

    bell_shell_aabb = ctx.part_element_world_aabb(bell_assembly, elem="bell_shell")
    if bell_shell_aabb is not None:
        bell_dims = tuple(bell_shell_aabb[1][axis] - bell_shell_aabb[0][axis] for axis in range(3))
        ctx.check(
            "bell_shell_realistic_size",
            1.10 <= bell_dims[0] <= 1.35 and 1.05 <= bell_dims[2] <= 1.30,
            f"bell shell should read as a large cast-iron bell, got {bell_dims}",
        )

    with ctx.pose({bell_swing: 0.0, clapper_swing: 0.0}):
        ctx.expect_contact(
            bell_assembly,
            tower,
            elem_a="left_journal",
            elem_b="left_bearing_block",
            contact_tol=1e-5,
            name="left_bell_bearing_contact",
        )
        ctx.expect_contact(
            bell_assembly,
            tower,
            elem_a="right_journal",
            elem_b="right_bearing_block",
            contact_tol=1e-5,
            name="right_bell_bearing_contact",
        )
        ctx.expect_contact(
            clapper,
            bell_assembly,
            elem_a="left_pivot_stub",
            elem_b="left_clapper_hanger",
            contact_tol=1e-5,
            name="left_clapper_pivot_contact",
        )
        ctx.expect_contact(
            clapper,
            bell_assembly,
            elem_a="right_pivot_stub",
            elem_b="right_clapper_hanger",
            contact_tol=1e-5,
            name="right_clapper_pivot_contact",
        )
        ctx.expect_origin_distance(
            bell_assembly,
            tower,
            axes="xy",
            max_dist=0.001,
            name="bell_centered_in_belfry",
        )
        ctx.expect_gap(
            bell_assembly,
            tower,
            axis="z",
            positive_elem="bell_shell",
            negative_elem="belfry_floor",
            min_gap=0.75,
            max_gap=1.8,
            name="bell_above_belfry_floor",
        )
        ctx.expect_gap(
            tower,
            bell_assembly,
            axis="z",
            positive_elem="ceiling_slab",
            negative_elem="yoke_beam",
            min_gap=0.20,
            max_gap=0.45,
            name="yoke_below_ceiling_slab",
        )

    bell_limits = bell_swing.motion_limits
    if bell_limits is not None and bell_limits.lower is not None and bell_limits.upper is not None:
        with ctx.pose({bell_swing: bell_limits.lower, clapper_swing: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="bell_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="bell_lower_pose_no_floating")
            ctx.expect_contact(
                bell_assembly,
                tower,
                elem_a="left_journal",
                elem_b="left_bearing_block",
                contact_tol=1e-5,
                name="bell_lower_left_bearing_contact",
            )
        with ctx.pose({bell_swing: bell_limits.upper, clapper_swing: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="bell_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="bell_upper_pose_no_floating")
            ctx.expect_contact(
                bell_assembly,
                tower,
                elem_a="right_journal",
                elem_b="right_bearing_block",
                contact_tol=1e-5,
                name="bell_upper_right_bearing_contact",
            )

    clapper_limits = clapper_swing.motion_limits
    if clapper_limits is not None and clapper_limits.lower is not None and clapper_limits.upper is not None:
        with ctx.pose({bell_swing: 0.0, clapper_swing: clapper_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="clapper_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="clapper_lower_pose_no_floating")
            ctx.expect_contact(
                clapper,
                bell_assembly,
                elem_a="left_pivot_stub",
                elem_b="left_clapper_hanger",
                contact_tol=1e-5,
                name="clapper_lower_left_contact",
            )
        with ctx.pose({bell_swing: 0.0, clapper_swing: clapper_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="clapper_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="clapper_upper_pose_no_floating")
            ctx.expect_contact(
                clapper,
                bell_assembly,
                elem_a="right_pivot_stub",
                elem_b="right_clapper_hanger",
                contact_tol=1e-5,
                name="clapper_upper_right_contact",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
