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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_searchlight_tower")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def add_bolt_circle(
        part,
        prefix: str,
        *,
        radius: float,
        z: float,
        count: int,
        head_radius: float,
        head_height: float,
        material,
    ) -> None:
        for index in range(count):
            angle = (2.0 * pi * index) / count
            part.visual(
                Cylinder(radius=head_radius, length=head_height),
                origin=Origin(
                    xyz=(
                        radius * cos(angle),
                        radius * sin(angle),
                        z + 0.5 * head_height,
                    )
                ),
                material=material,
                name=f"{prefix}_{index:02d}",
            )

    def add_side_bolt_grid(
        part,
        prefix: str,
        *,
        y: float,
        xs: tuple[float, ...],
        zs: tuple[float, ...],
        head_radius: float,
        head_length: float,
        material,
    ) -> None:
        side_sign = 1.0 if y >= 0.0 else -1.0
        center_y = y + side_sign * (0.5 * head_length)
        for xi, x in enumerate(xs):
            for zi, z in enumerate(zs):
                part.visual(
                    Cylinder(radius=head_radius, length=head_length),
                    origin=Origin(
                        xyz=(x, center_y, z),
                        rpy=(pi / 2.0, 0.0, 0.0),
                    ),
                    material=material,
                    name=f"{prefix}_{xi}_{zi}",
                )

    def polar_xy(radius: float, angle_deg: float) -> tuple[float, float]:
        angle = radians(angle_deg)
        return radius * cos(angle), radius * sin(angle)

    tower_gray = model.material("tower_gray", rgba=(0.41, 0.44, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    galvanized = model.material("galvanized", rgba=(0.73, 0.75, 0.77, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.83, 0.68, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.82, 0.89, 0.55))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))

    tower_base = model.part("tower_base")
    tower_base.inertial = Inertial.from_geometry(
        Box((1.60, 1.40, 2.98)),
        mass=880.0,
        origin=Origin(xyz=(0.0, 0.0, 1.49)),
    )
    tower_base.visual(
        Box((1.60, 1.40, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="foundation_skid",
    )
    tower_base.visual(
        Box((0.86, 0.82, 0.84)),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=tower_gray,
        name="drive_cabinet",
    )
    tower_base.visual(
        Box((0.40, 0.32, 1.72)),
        origin=Origin(xyz=(0.0, 0.0, 1.80)),
        material=tower_gray,
        name="mast_column",
    )
    tower_base.visual(
        Box((0.46, 0.05, 0.95)),
        origin=Origin(xyz=(0.0, 0.185, 1.55)),
        material=dark_steel,
        name="front_doubler_plate",
    )
    tower_base.visual(
        Box((0.46, 0.05, 0.95)),
        origin=Origin(xyz=(0.0, -0.185, 1.55)),
        material=dark_steel,
        name="rear_doubler_plate",
    )
    tower_base.visual(
        Box((0.66, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.70)),
        material=dark_steel,
        name="mast_head_plate",
    )
    tower_base.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.83)),
        material=tower_gray,
        name="pan_bearing_housing",
    )
    tower_base.visual(
        Box((0.12, 0.28, 0.12)),
        origin=Origin(xyz=(0.30, 0.0, 2.86)),
        material=dark_steel,
        name="pan_lockout_shoe",
    )
    add_bolt_circle(
        tower_base,
        "head_plate_bolt",
        radius=0.30,
        z=2.74,
        count=8,
        head_radius=0.018,
        head_height=0.018,
        material=galvanized,
    )

    for angle_deg, visual_name in ((150.0, "pan_stop_pos"), (-150.0, "pan_stop_neg")):
        px, py = polar_xy(0.31, angle_deg)
        tower_base.visual(
            Box((0.08, 0.05, 0.12)),
            origin=Origin(xyz=(px, py, 2.88), rpy=(0.0, 0.0, radians(angle_deg))),
            material=dark_steel,
            name=visual_name,
        )

    brace_specs = (
        ("tower_brace_fl", 0.30, 0.22),
        ("tower_brace_fr", 0.30, -0.22),
        ("tower_brace_rl", -0.30, 0.22),
        ("tower_brace_rr", -0.30, -0.22),
    )
    for mesh_name, x_sign, y_sign in brace_specs:
        brace_mesh = save_mesh(
            mesh_name,
            tube_from_spline_points(
                [
                    (x_sign * 1.55, y_sign * 1.45, 0.02),
                    (x_sign * 1.10, y_sign * 1.22, 0.26),
                    (x_sign * 0.70, y_sign * 0.80, 0.82),
                    (x_sign * 0.30, y_sign * 0.22, 1.56),
                ],
                radius=0.045,
                samples_per_segment=12,
                radial_segments=16,
            ),
        )
        tower_base.visual(brace_mesh, material=tower_gray, name=mesh_name)

    pan_yoke = model.part("pan_yoke")
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.82, 0.92, 1.04)),
        mass=185.0,
        origin=Origin(xyz=(0.16, 0.0, 0.52)),
    )
    pan_yoke.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="turntable_drum",
    )
    pan_yoke.visual(
        Cylinder(radius=0.35, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=tower_gray,
        name="turntable_flange",
    )
    pan_yoke.visual(
        Box((0.20, 0.24, 0.28)),
        origin=Origin(xyz=(-0.09, 0.0, 0.24)),
        material=tower_gray,
        name="pan_pedestal",
    )
    pan_yoke.visual(
        Box((0.16, 0.82, 0.10)),
        origin=Origin(xyz=(-0.16, 0.0, 0.43)),
        material=dark_steel,
        name="rear_spine_box",
    )
    pan_yoke.visual(
        Box((0.54, 0.06, 1.02)),
        origin=Origin(xyz=(0.12, 0.425, 0.51)),
        material=tower_gray,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.54, 0.06, 1.02)),
        origin=Origin(xyz=(0.12, -0.425, 0.51)),
        material=tower_gray,
        name="right_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.12, 0.92, 0.12)),
        origin=Origin(xyz=(-0.22, 0.0, 0.94)),
        material=dark_steel,
        name="top_tie_bar",
    )
    pan_yoke.visual(
        Box((0.12, 0.05, 0.28)),
        origin=Origin(xyz=(-0.10, 0.455, 0.48)),
        material=dark_steel,
        name="tilt_lockout_plate",
    )
    pan_yoke.visual(
        Box((0.06, 0.05, 0.10)),
        origin=Origin(xyz=(0.18, 0.0, 0.08)),
        material=dark_steel,
        name="pan_stop_dog",
    )
    pan_yoke.visual(
        Box((0.18, 0.10, 0.04)),
        origin=Origin(xyz=(0.08, 0.0, 0.08)),
        material=dark_steel,
        name="pan_stop_dog_base",
    )
    pan_yoke.visual(
        Box((0.18, 0.84, 0.10)),
        origin=Origin(xyz=(-0.16, 0.0, 0.49)),
        material=dark_steel,
        name="lower_cross_tie",
    )
    pan_yoke.visual(
        Box((0.12, 0.10, 0.60)),
        origin=Origin(xyz=(-0.16, 0.41, 0.64)),
        material=dark_steel,
        name="left_load_web",
    )
    pan_yoke.visual(
        Box((0.12, 0.10, 0.60)),
        origin=Origin(xyz=(-0.16, -0.41, 0.64)),
        material=dark_steel,
        name="right_load_web",
    )

    pan_yoke.visual(
        Box((0.12, 0.08, 0.18)),
        origin=Origin(xyz=(0.22, 0.425, 0.82)),
        material=dark_steel,
        name="left_bearing_block",
    )
    pan_yoke.visual(
        Box((0.12, 0.08, 0.18)),
        origin=Origin(xyz=(0.22, -0.425, 0.82)),
        material=dark_steel,
        name="right_bearing_block",
    )
    pan_yoke.visual(
        Box((0.14, 0.04, 0.24)),
        origin=Origin(xyz=(0.22, 0.485, 0.82)),
        material=galvanized,
        name="left_bearing_cap",
    )
    pan_yoke.visual(
        Box((0.14, 0.04, 0.24)),
        origin=Origin(xyz=(0.22, -0.485, 0.82)),
        material=galvanized,
        name="right_bearing_cap",
    )
    add_side_bolt_grid(
        pan_yoke,
        "left_cap_bolt",
        y=0.505,
        xs=(0.18, 0.26),
        zs=(0.74, 0.90),
        head_radius=0.012,
        head_length=0.018,
        material=galvanized,
    )
    add_side_bolt_grid(
        pan_yoke,
        "right_cap_bolt",
        y=-0.505,
        xs=(0.18, 0.26),
        zs=(0.74, 0.90),
        head_radius=0.012,
        head_length=0.018,
        material=galvanized,
    )
    pan_yoke.visual(
        Box((0.06, 0.08, 0.36)),
        origin=Origin(xyz=(0.45, -0.425, 0.82)),
        material=dark_steel,
        name="tilt_stop_support",
    )
    pan_yoke.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(xyz=(0.412, -0.40, 0.962), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="upper_tilt_stop_block",
    )
    pan_yoke.visual(
        Cylinder(radius=0.028, length=0.08),
        origin=Origin(xyz=(0.412, -0.40, 0.678), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lower_tilt_stop_block",
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.inertial = Inertial.from_geometry(
        Box((1.05, 0.70, 0.72)),
        mass=96.0,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )
    spotlight_head.visual(
        Cylinder(radius=0.07, length=0.77),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    spotlight_head.visual(
        Cylinder(radius=0.31, length=0.74),
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="lamp_barrel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.325, length=0.014),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="cooling_ring_0",
    )
    spotlight_head.visual(
        Cylinder(radius=0.325, length=0.014),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="cooling_ring_1",
    )
    spotlight_head.visual(
        Cylinder(radius=0.325, length=0.014),
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="cooling_ring_2",
    )
    spotlight_head.visual(
        Cylinder(radius=0.35, length=0.08),
        origin=Origin(xyz=(0.66, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.28, length=0.01),
        origin=Origin(xyz=(0.705, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_window",
    )
    spotlight_head.visual(
        Box((0.24, 0.34, 0.32)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0)),
        material=tower_gray,
        name="rear_ballast_box",
    )
    spotlight_head.visual(
        Box((0.16, 0.46, 0.18)),
        origin=Origin(xyz=(-0.03, 0.0, -0.17)),
        material=dark_steel,
        name="lower_reinforcement_saddle",
    )
    spotlight_head.visual(
        Box((0.38, 0.20, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, 0.31)),
        material=dark_steel,
        name="top_service_hatch",
    )
    spotlight_head.visual(
        Box((0.26, 0.03, 0.20)),
        origin=Origin(xyz=(0.04, 0.31, -0.10)),
        material=dark_steel,
        name="left_side_stiffener",
    )
    spotlight_head.visual(
        Box((0.26, 0.03, 0.20)),
        origin=Origin(xyz=(0.04, -0.31, -0.10)),
        material=dark_steel,
        name="right_side_stiffener",
    )
    spotlight_head.visual(
        Box((0.20, 0.03, 0.26)),
        origin=Origin(xyz=(0.12, 0.33, 0.02)),
        material=galvanized,
        name="tilt_lock_plate_head",
    )
    spotlight_head.visual(
        Cylinder(radius=0.012, length=0.10),
        origin=Origin(xyz=(0.21, 0.345, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
        name="tilt_lock_handle",
    )
    spotlight_head.visual(
        Box((0.08, 0.05, 0.12)),
        origin=Origin(xyz=(0.16, -0.28, -0.08)),
        material=dark_steel,
        name="stop_pin_bracket",
    )
    spotlight_head.visual(
        Cylinder(radius=0.025, length=0.06),
        origin=Origin(xyz=(0.20, -0.33, -0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="tilt_stop_pin",
    )

    guard_ring = save_mesh(
        "searchlight_front_guard_ring",
        TorusGeometry(radius=0.31, tube=0.014, radial_segments=14, tubular_segments=48),
    )
    spotlight_head.visual(
        guard_ring,
        origin=Origin(xyz=(0.73, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="front_guard_ring",
    )
    for index, (gy, gz) in enumerate(
        (
            (0.219, 0.219),
            (0.219, -0.219),
            (-0.219, 0.219),
            (-0.219, -0.219),
        )
    ):
        spotlight_head.visual(
            Cylinder(radius=0.012, length=0.08),
            origin=Origin(xyz=(0.69, gy, gz), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"guard_standoff_{index}",
        )
    spotlight_head.visual(
        Cylinder(radius=0.010, length=0.62),
        origin=Origin(xyz=(0.73, 0.0, 0.0)),
        material=galvanized,
        name="guard_bar_center_v",
    )
    spotlight_head.visual(
        Cylinder(radius=0.009, length=0.572),
        origin=Origin(xyz=(0.73, 0.12, 0.0)),
        material=galvanized,
        name="guard_bar_left_v",
    )
    spotlight_head.visual(
        Cylinder(radius=0.009, length=0.572),
        origin=Origin(xyz=(0.73, -0.12, 0.0)),
        material=galvanized,
        name="guard_bar_right_v",
    )
    spotlight_head.visual(
        Cylinder(radius=0.010, length=0.62),
        origin=Origin(xyz=(0.73, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="guard_bar_center_h",
    )

    for index in range(8):
        angle = (2.0 * pi * index) / 8.0
        spotlight_head.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(
                xyz=(
                    0.715,
                    0.24 * cos(angle),
                    0.24 * sin(angle),
                ),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=galvanized,
            name=f"bezel_bolt_{index:02d}",
        )

    model.articulation(
        "mast_pan",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=950.0,
            velocity=0.60,
            lower=-radians(150.0),
            upper=radians(150.0),
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_head,
        origin=Origin(xyz=(0.22, 0.0, 0.82)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=780.0,
            velocity=0.75,
            lower=radians(-20.0),
            upper=radians(63.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_base = object_model.get_part("tower_base")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_head = object_model.get_part("spotlight_head")
    pan = object_model.get_articulation("mast_pan")
    tilt = object_model.get_articulation("yoke_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="left_bearing_block",
        reason="Trunnion shaft passes through the left bored bearing housing; the housing is represented as a solid external block.",
    )
    ctx.allow_overlap(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="right_bearing_block",
        reason="Trunnion shaft passes through the right bored bearing housing; the housing is represented as a solid external block.",
    )

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

    ctx.check(
        "pan_axis_vertical",
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        f"Expected pan axis (0, 0, 1), got {pan.axis}",
    )
    ctx.check(
        "tilt_axis_pitch",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        f"Expected tilt axis (0, -1, 0), got {tilt.axis}",
    )
    ctx.check(
        "pan_limits_have_overtravel_margin",
        pan.motion_limits is not None
        and pan.motion_limits.lower is not None
        and pan.motion_limits.upper is not None
        and pan.motion_limits.lower <= -2.5
        and pan.motion_limits.upper >= 2.5,
        f"Unexpected pan limits: {pan.motion_limits}",
    )
    ctx.check(
        "tilt_limits_match_worklight_range",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 1.0,
        f"Unexpected tilt limits: {tilt.motion_limits}",
    )

    ctx.expect_contact(
        pan_yoke,
        tower_base,
        elem_a="turntable_drum",
        elem_b="pan_bearing_housing",
        name="pan_turntable_seated_on_bearing_housing",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="left_bearing_block",
        name="left_trunnion_supported_by_bearing_block",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="right_bearing_block",
        name="right_trunnion_supported_by_bearing_block",
    )
    ctx.expect_within(
        spotlight_head,
        pan_yoke,
        axes="y",
        margin=0.02,
        name="head_stays_within_yoke_span",
    )
    ctx.expect_gap(
        spotlight_head,
        tower_base,
        axis="z",
        min_gap=0.45,
        positive_elem="rear_ballast_box",
        negative_elem="pan_bearing_housing",
        name="rear_ballast_box_clears_mast_head",
    )

    with ctx.pose({tilt: 0.0}):
        rest_guard_top = ctx.part_element_world_aabb(spotlight_head, elem="front_guard_ring")[1][2]
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_guard_top = ctx.part_element_world_aabb(spotlight_head, elem="front_guard_ring")[1][2]
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a="tilt_stop_pin",
            elem_b="upper_tilt_stop_block",
            name="upper_tilt_stop_contacts_hard_stop",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_tilt_limit")
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a="tilt_stop_pin",
            elem_b="lower_tilt_stop_block",
            name="lower_tilt_stop_contacts_hard_stop",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_tilt_limit")

    ctx.check(
        "tilt_limit_raises_front_guard",
        raised_guard_top > rest_guard_top + 0.20,
        f"Front guard top only moved from {rest_guard_top:.3f} m to {raised_guard_top:.3f} m",
    )

    with ctx.pose({pan: pan.motion_limits.upper, tilt: 0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_pan_limit")
    with ctx.pose({pan: pan.motion_limits.lower, tilt: 0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_pan_limit")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
