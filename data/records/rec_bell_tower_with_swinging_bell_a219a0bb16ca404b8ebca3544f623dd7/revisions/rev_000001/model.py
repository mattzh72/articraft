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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


Vec3 = tuple[float, float, float]


def _midpoint(a: Vec3, b: Vec3) -> Vec3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(a: Vec3, b: Vec3) -> Vec3:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_timber(
    part,
    a: Vec3,
    b: Vec3,
    *,
    width: float,
    material,
    name: str | None = None,
    depth: float | None = None,
) -> None:
    part.visual(
        Box((width, depth if depth is not None else width, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nordic_stave_bell_tower")

    timber = model.material("timber", rgba=(0.40, 0.27, 0.18, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.46, 0.33, 0.24, 1.0))
    dark_shingle = model.material("dark_shingle", rgba=(0.16, 0.13, 0.11, 1.0))
    iron = model.material("iron", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.46, 0.47, 0.50, 1.0))

    post_half = 0.72
    post_size = 0.14
    base_size = 1.82
    base_height = 0.16
    wall_height = 1.88
    wall_top_z = base_height + wall_height
    waist_rail_z = 2.56
    eave_z = 3.05
    ridge_timber_z = 3.52
    bell_pivot_z = ridge_timber_z - 0.12
    roof_ridge_z = 4.02
    roof_half_run = 0.98
    roof_pitch = math.atan2(roof_ridge_z - eave_z, roof_half_run)
    roof_slope_len = math.hypot(roof_ridge_z - eave_z, roof_half_run)

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((2.30, 2.30, 4.20)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
    )
    tower.visual(
        Box((base_size, base_size, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=weathered_timber,
        name="base_plinth",
    )

    for y_sign in (-1.0, 1.0):
        tower.visual(
            Box((1.56, 0.16, 0.16)),
            origin=Origin(xyz=(0.0, y_sign * post_half, 0.22)),
            material=weathered_timber,
        )
    for x_sign in (-1.0, 1.0):
        tower.visual(
            Box((0.16, 1.56, 0.16)),
            origin=Origin(xyz=(x_sign * post_half, 0.0, 0.22)),
            material=weathered_timber,
        )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_timber(
                tower,
                (x_sign * post_half, y_sign * post_half, base_height),
                (x_sign * post_half, y_sign * post_half, eave_z),
                width=post_size,
                material=timber,
                name=f"corner_post_{'pos' if x_sign > 0 else 'neg'}x_{'pos' if y_sign > 0 else 'neg'}y",
            )

    wall_center_z = base_height + wall_height * 0.5
    wall_span = 2.0 * (post_half - post_size * 0.5)
    tower.visual(
        Box((wall_span, 0.08, wall_height)),
        origin=Origin(xyz=(0.0, post_half, wall_center_z)),
        material=weathered_timber,
        name="front_shaft_wall",
    )
    tower.visual(
        Box((wall_span, 0.08, wall_height)),
        origin=Origin(xyz=(0.0, -post_half, wall_center_z)),
        material=weathered_timber,
        name="rear_shaft_wall",
    )
    tower.visual(
        Box((0.08, wall_span, wall_height)),
        origin=Origin(xyz=(post_half, 0.0, wall_center_z)),
        material=weathered_timber,
        name="right_shaft_wall",
    )
    tower.visual(
        Box((0.08, wall_span, wall_height)),
        origin=Origin(xyz=(-post_half, 0.0, wall_center_z)),
        material=weathered_timber,
        name="left_shaft_wall",
    )

    for y_sign in (-1.0, 1.0):
        tower.visual(
            Box((1.58, 0.12, 0.12)),
            origin=Origin(xyz=(0.0, y_sign * post_half, wall_top_z + 0.06)),
            material=timber,
        )
        tower.visual(
            Box((1.44, 0.10, 0.10)),
            origin=Origin(xyz=(0.0, y_sign * post_half, waist_rail_z)),
            material=timber,
        )
        tower.visual(
            Box((1.62, 0.14, 0.14)),
            origin=Origin(xyz=(0.0, y_sign * post_half, eave_z)),
            material=timber,
        )
    for x_sign in (-1.0, 1.0):
        tower.visual(
            Box((0.12, 1.58, 0.12)),
            origin=Origin(xyz=(x_sign * post_half, 0.0, wall_top_z + 0.06)),
            material=timber,
        )
        tower.visual(
            Box((0.10, 1.44, 0.10)),
            origin=Origin(xyz=(x_sign * post_half, 0.0, waist_rail_z)),
            material=timber,
        )
        tower.visual(
            Box((0.14, 1.62, 0.14)),
            origin=Origin(xyz=(x_sign * post_half, 0.0, eave_z)),
            material=timber,
        )

    brace_low_z = wall_top_z + 0.08
    for y_sign in (-1.0, 1.0):
        face_y = y_sign * post_half
        _add_timber(
            tower,
            (-0.48, face_y, brace_low_z),
            (0.0, face_y, waist_rail_z),
            width=0.06,
            material=timber,
        )
        _add_timber(
            tower,
            (0.48, face_y, brace_low_z),
            (0.0, face_y, waist_rail_z),
            width=0.06,
            material=timber,
        )
    for x_sign in (-1.0, 1.0):
        face_x = x_sign * post_half
        _add_timber(
            tower,
            (face_x, -0.48, brace_low_z),
            (face_x, 0.0, waist_rail_z),
            width=0.06,
            material=timber,
        )
        _add_timber(
            tower,
            (face_x, 0.48, brace_low_z),
            (face_x, 0.0, waist_rail_z),
            width=0.06,
            material=timber,
        )

    tower.visual(
        Box((0.12, 1.72, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, ridge_timber_z)),
        material=timber,
        name="ridge_timber",
    )
    tower.visual(
        Box((0.08, 0.05, 0.24)),
        origin=Origin(xyz=(0.0, 0.245, bell_pivot_z)),
        material=steel,
        name="bearing_block_pos_y",
    )
    tower.visual(
        Box((0.08, 0.05, 0.24)),
        origin=Origin(xyz=(0.0, -0.245, bell_pivot_z)),
        material=steel,
        name="bearing_block_neg_y",
    )

    for y_sign in (-1.0, 1.0):
        y = y_sign * 0.84
        _add_timber(
            tower,
            (0.0, y, ridge_timber_z),
            (roof_half_run, y, eave_z),
            width=0.08,
            material=timber,
        )
        _add_timber(
            tower,
            (0.0, y, ridge_timber_z),
            (-roof_half_run, y, eave_z),
            width=0.08,
            material=timber,
        )
        _add_timber(
            tower,
            (0.0, y, eave_z),
            (0.0, y, ridge_timber_z),
            width=0.08,
            material=timber,
        )

    tower.visual(
        Box((roof_slope_len, 1.94, 0.06)),
        origin=Origin(
            xyz=(-roof_half_run * 0.5, 0.0, (eave_z + roof_ridge_z) * 0.5),
            rpy=(0.0, -roof_pitch, 0.0),
        ),
        material=dark_shingle,
        name="roof_left",
    )
    tower.visual(
        Box((roof_slope_len, 1.94, 0.06)),
        origin=Origin(
            xyz=(roof_half_run * 0.5, 0.0, (eave_z + roof_ridge_z) * 0.5),
            rpy=(0.0, roof_pitch, 0.0),
        ),
        material=dark_shingle,
        name="roof_right",
    )
    tower.visual(
        Box((0.10, 1.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, roof_ridge_z + 0.02)),
        material=dark_shingle,
        name="ridge_cap",
    )

    for side_sign in (-1.0, 1.0):
        panel_angle = -roof_pitch if side_sign < 0.0 else roof_pitch
        side_name = "left" if side_sign < 0.0 else "right"
        for index in range(5):
            t = 0.16 + 0.15 * index
            tower.visual(
                Box((0.028, 1.84, 0.012)),
                origin=Origin(
                    xyz=(
                        side_sign * roof_half_run * (1.0 - t),
                        0.0,
                        eave_z + (roof_ridge_z - eave_z) * t + 0.010,
                    ),
                    rpy=(0.0, panel_angle, 0.0),
                ),
                material=dark_shingle,
                name=f"{side_name}_shingle_course_{index}",
            )

    bell_outer_profile = [
        (0.025, -0.31),
        (0.070, -0.33),
        (0.145, -0.38),
        (0.235, -0.49),
        (0.305, -0.63),
        (0.338, -0.78),
        (0.350, -0.90),
        (0.340, -0.94),
    ]
    bell_inner_profile = [
        (0.000, -0.34),
        (0.050, -0.36),
        (0.125, -0.41),
        (0.205, -0.51),
        (0.275, -0.64),
        (0.308, -0.79),
        (0.320, -0.91),
    ]
    bell_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            bell_outer_profile,
            bell_inner_profile,
            segments=72,
        ),
        "bell_shell",
    )

    bell_assembly = model.part("bell_assembly")
    bell_assembly.inertial = Inertial.from_geometry(
        Box((0.82, 0.42, 1.00)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
    )
    bell_assembly.visual(
        Box((0.12, 0.34, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=timber,
        name="yoke_head",
    )
    bell_assembly.visual(
        Cylinder(radius=0.026, length=0.05),
        origin=Origin(xyz=(0.0, 0.195, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_stub_pos_y",
    )
    bell_assembly.visual(
        Cylinder(radius=0.026, length=0.05),
        origin=Origin(xyz=(0.0, -0.195, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_stub_neg_y",
    )
    bell_assembly.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
    )
    bell_assembly.visual(
        Box((0.08, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=timber,
    )
    bell_assembly.visual(
        Box((0.10, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
        material=timber,
        name="headstock_beam",
    )
    bell_assembly.visual(
        Box((0.12, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.32)),
        material=timber,
        name="bell_crown_block",
    )

    for y_sign in (-1.0, 1.0):
        y = y_sign * 0.11
        _add_timber(
            bell_assembly,
            (0.0, y, -0.02),
            (0.18, y, -0.42),
            width=0.06,
            material=timber,
        )
        _add_timber(
            bell_assembly,
            (0.0, y, -0.02),
            (-0.18, y, -0.42),
            width=0.06,
            material=timber,
        )
        _add_timber(
            bell_assembly,
            (-0.18, y, -0.42),
            (0.18, y, -0.42),
            width=0.05,
            material=timber,
        )

    bell_assembly.visual(
        Box((0.02, 0.06, 0.14)),
        origin=Origin(xyz=(0.07, 0.0, -0.41)),
        material=steel,
    )
    bell_assembly.visual(
        Box((0.02, 0.06, 0.14)),
        origin=Origin(xyz=(-0.07, 0.0, -0.41)),
        material=steel,
    )
    bell_assembly.visual(
        bell_shell_mesh,
        material=iron,
        name="bell_shell",
    )
    bell_assembly.visual(
        Box((0.04, 0.025, 0.05)),
        origin=Origin(xyz=(0.0, 0.0725, -0.41)),
        material=steel,
        name="clapper_bearing_pos_y",
    )
    bell_assembly.visual(
        Box((0.04, 0.025, 0.05)),
        origin=Origin(xyz=(0.0, -0.0725, -0.41)),
        material=steel,
        name="clapper_bearing_neg_y",
    )

    clapper = model.part("clapper")
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.62),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
    )
    clapper.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_eye",
    )
    clapper.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=steel,
        name="clapper_rod",
    )
    clapper.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=iron,
        name="clapper_neck",
    )
    clapper.visual(
        Sphere(radius=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
        material=iron,
        name="clapper_bob",
    )

    model.articulation(
        "tower_to_bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_assembly,
        origin=Origin(xyz=(0.0, 0.0, bell_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=1.5,
            lower=-0.38,
            upper=0.38,
        ),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell_assembly,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.41)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=3.0,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell_assembly = object_model.get_part("bell_assembly")
    clapper = object_model.get_part("clapper")
    bell_swing = object_model.get_articulation("tower_to_bell_swing")
    clapper_pin = object_model.get_articulation("bell_to_clapper")

    tower_bearing_pos = tower.get_visual("bearing_block_pos_y")
    tower_bearing_neg = tower.get_visual("bearing_block_neg_y")
    bell_pivot_pos = bell_assembly.get_visual("pivot_stub_pos_y")
    bell_pivot_neg = bell_assembly.get_visual("pivot_stub_neg_y")
    bell_shell = bell_assembly.get_visual("bell_shell")
    clapper_bearing_pos = bell_assembly.get_visual("clapper_bearing_pos_y")
    clapper_bearing_neg = bell_assembly.get_visual("clapper_bearing_neg_y")
    clapper_eye = clapper.get_visual("pivot_eye")
    clapper_bob = clapper.get_visual("clapper_bob")

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
        bell_assembly,
        tower,
        elem_a=bell_pivot_pos,
        elem_b=tower_bearing_pos,
        contact_tol=0.002,
        name="positive-y bell pivot bears on the ridge support",
    )
    ctx.expect_contact(
        bell_assembly,
        tower,
        elem_a=bell_pivot_neg,
        elem_b=tower_bearing_neg,
        contact_tol=0.002,
        name="negative-y bell pivot bears on the ridge support",
    )
    ctx.expect_contact(
        clapper,
        bell_assembly,
        elem_a=clapper_eye,
        elem_b=clapper_bearing_pos,
        contact_tol=0.002,
        name="clapper pin bears on positive-y bell lug",
    )
    ctx.expect_contact(
        clapper,
        bell_assembly,
        elem_a=clapper_eye,
        elem_b=clapper_bearing_neg,
        contact_tol=0.002,
        name="clapper pin bears on negative-y bell lug",
    )
    ctx.expect_within(
        clapper,
        bell_assembly,
        axes="xy",
        inner_elem=clapper_bob,
        outer_elem=bell_shell,
        margin=0.02,
        name="clapper bob stays within the bell mouth footprint",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    rest_bell_center = _aabb_center(ctx.part_world_aabb(bell_assembly))
    with ctx.pose({bell_swing: bell_swing.motion_limits.upper}):
        swung_bell_center = _aabb_center(ctx.part_world_aabb(bell_assembly))
    ctx.check(
        "bell assembly swings sideways under the ridge timber",
        rest_bell_center is not None
        and swung_bell_center is not None
        and abs(swung_bell_center[0] - rest_bell_center[0]) > 0.12
        and swung_bell_center[2] < rest_bell_center[2] - 0.008,
        details=f"rest_center={rest_bell_center}, swung_center={swung_bell_center}",
    )

    rest_clapper_center = _aabb_center(ctx.part_world_aabb(clapper))
    with ctx.pose({clapper_pin: clapper_pin.motion_limits.upper * 0.75}):
        swung_clapper_center = _aabb_center(ctx.part_world_aabb(clapper))
    ctx.check(
        "clapper swings independently inside the bell",
        rest_clapper_center is not None
        and swung_clapper_center is not None
        and abs(swung_clapper_center[0] - rest_clapper_center[0]) > 0.05,
        details=f"rest_center={rest_clapper_center}, swung_center={swung_clapper_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
