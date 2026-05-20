from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _mirror_profile_vertical(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for x, y in reversed(profile)]


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _frame_outer_profile() -> list[tuple[float, float]]:
    half_length = 0.091
    shoulder_x = 0.056
    top_z = 0.074

    return [
        (-half_length, 0.000),
        (half_length, 0.000),
        (half_length, 0.026),
        (shoulder_x, top_z),
        (-shoulder_x, top_z),
        (-half_length, 0.026),
    ]


def _build_side_frame_mesh(*, mirror_for_right: bool) -> object:
    outer = _frame_outer_profile()
    lightening_window = _offset_profile(
        rounded_rect_profile(0.086, 0.020, 0.006, corner_segments=5),
        dy=0.023,
    )
    bearing_hole = _circle_profile(0.0, 0.049, 0.008, segments=20)

    if mirror_for_right:
        outer = _mirror_profile_vertical(outer)
        lightening_window = _mirror_profile_vertical(lightening_window)
        bearing_hole = _mirror_profile_vertical(bearing_hole)
        geometry = ExtrudeWithHolesGeometry(
            outer,
            [lightening_window, bearing_hole],
            height=0.012,
            center=False,
        ).rotate_x(-math.pi / 2.0)
    else:
        geometry = ExtrudeWithHolesGeometry(
            outer,
            [lightening_window, bearing_hole],
            height=0.012,
            center=False,
        ).rotate_x(math.pi / 2.0)

    return mesh_from_geometry(
        geometry,
        "right_side_frame" if mirror_for_right else "left_side_frame",
    )


def _build_wheel_cage_mesh() -> object:
    rim_major_radius = 0.044
    rim_tube_radius = 0.004
    half_track = 0.046
    hub_radius = 0.018
    hub_length = 0.092
    spoke_length = 0.030
    spoke_width = 0.082
    spoke_thickness = 0.010
    paddle_radius = 0.045
    paddle_width = 0.090
    paddle_radial_depth = 0.018
    paddle_tangential_thickness = 0.026

    cage = TorusGeometry(
        radius=rim_major_radius,
        tube=rim_tube_radius,
        radial_segments=16,
        tubular_segments=56,
    ).rotate_x(math.pi / 2.0).translate(0.0, -half_track, 0.0)
    cage.merge(
        TorusGeometry(
            radius=rim_major_radius,
            tube=rim_tube_radius,
            radial_segments=16,
            tubular_segments=56,
        ).rotate_x(math.pi / 2.0).translate(0.0, half_track, 0.0)
    )
    cage.merge(
        CylinderGeometry(hub_radius, hub_length, radial_segments=28, closed=True).rotate_x(
            math.pi / 2.0
        )
    )

    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0
        spoke = BoxGeometry((spoke_length, spoke_width, spoke_thickness))
        spoke.translate(hub_radius + (spoke_length * 0.5), 0.0, 0.0)
        spoke.rotate_y(angle)
        cage.merge(spoke)

    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        paddle = BoxGeometry(
            (
                paddle_radial_depth,
                paddle_width,
                paddle_tangential_thickness,
            )
        )
        paddle.translate(paddle_radius, 0.0, 0.0)
        paddle.rotate_y(angle)
        cage.merge(paddle)

    return mesh_from_geometry(cage, "undershot_wheel_cage")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_undershot_waterwheel")

    ash = model.material("ash_plywood", rgba=(0.79, 0.70, 0.55, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.68, 0.72, 1.0))
    waterway_gray = model.material("waterway_gray", rgba=(0.52, 0.54, 0.56, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.300, 0.138, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=ash,
        name="tray_floor",
    )
    chassis.visual(
        Box((0.284, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.075, 0.014)),
        material=ash,
        name="left_rail",
    )
    chassis.visual(
        Box((0.284, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.075, 0.014)),
        material=ash,
        name="right_rail",
    )
    chassis.visual(
        Box((0.008, 0.138, 0.028)),
        origin=Origin(xyz=(0.146, 0.0, 0.014)),
        material=ash,
        name="front_wall",
    )
    chassis.visual(
        Box((0.008, 0.138, 0.038)),
        origin=Origin(xyz=(-0.146, 0.0, 0.019)),
        material=ash,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(-0.116, 0.0, 0.014)),
        material=waterway_gray,
        name="inlet_plenum",
    )
    chassis.visual(
        Box((0.036, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.066, 0.048)),
        material=dark_metal,
        name="left_frame_stop",
    )
    chassis.visual(
        Box((0.036, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, -0.066, 0.048)),
        material=dark_metal,
        name="right_frame_stop",
    )
    chassis.visual(
        Box((0.240, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=waterway_gray,
        name="flow_strip",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.300, 0.162, 0.052)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    left_frame = model.part("left_frame")
    left_frame.visual(
        _build_side_frame_mesh(mirror_for_right=False),
        material=dark_metal,
        name="frame_panel",
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((0.182, 0.012, 0.074)),
        mass=0.24,
        origin=Origin(xyz=(0.0, -0.006, 0.037)),
    )

    right_frame = model.part("right_frame")
    right_frame.visual(
        _build_side_frame_mesh(mirror_for_right=True),
        material=dark_metal,
        name="frame_panel",
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((0.182, 0.012, 0.074)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.006, 0.037)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.008, length=0.023),
        origin=Origin(xyz=(0.0, 0.0575, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_journal",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.023),
        origin=Origin(xyz=(0.0, -0.0575, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_journal",
    )
    wheel.visual(
        _build_wheel_cage_mesh(),
        material=steel,
        name="wheel_cage",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.138),
        mass=0.42,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "left_frame_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=left_frame,
        origin=Origin(xyz=(0.0, 0.081, 0.028)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "right_frame_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=right_frame,
        origin=Origin(xyz=(0.0, -0.081, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    wheel = object_model.get_part("wheel")
    left_fold = object_model.get_articulation("left_frame_fold")
    right_fold = object_model.get_articulation("right_frame_fold")
    wheel_spin = object_model.get_articulation("wheel_spin")

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

    ctx.check(
        "wheel_spin_joint_is_continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        "The paddle wheel must spin continuously around its centered axle.",
    )
    ctx.check(
        "wheel_spin_axis_runs_side_to_side",
        tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"Expected wheel axis (0, 1, 0), got {wheel_spin.axis}.",
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        ctx.expect_origin_gap(
            left_frame,
            wheel,
            axis="y",
            min_gap=0.080,
            max_gap=0.082,
            name="left_support_is_centered_from_wheel",
        )
        ctx.expect_origin_gap(
            wheel,
            right_frame,
            axis="y",
            min_gap=0.080,
            max_gap=0.082,
            name="right_support_is_centered_from_wheel",
        )
        ctx.expect_contact(
            wheel,
            left_frame,
            elem_a="left_journal",
            elem_b="frame_panel",
            name="axle_is_supported_by_left_bearing_slot",
        )
        ctx.expect_contact(
            wheel,
            right_frame,
            elem_a="right_journal",
            elem_b="frame_panel",
            name="axle_is_supported_by_right_bearing_slot",
        )
        ctx.expect_gap(
            wheel,
            chassis,
            axis="z",
            min_gap=0.014,
            max_gap=0.028,
            positive_elem="wheel_cage",
            negative_elem="tray_floor",
            name="undershot_wheel_clears_tray_floor",
        )
        ctx.expect_contact(
            left_frame,
            chassis,
            elem_a="frame_panel",
            elem_b="left_frame_stop",
            name="left_frame_deploys_against_stop",
        )
        ctx.expect_contact(
            right_frame,
            chassis,
            elem_a="frame_panel",
            elem_b="right_frame_stop",
            name="right_frame_deploys_against_stop",
        )

    with ctx.pose({left_fold: 1.20, right_fold: 1.20}):
        ctx.expect_gap(
            left_frame,
            wheel,
            axis="y",
            min_gap=0.020,
            positive_elem="frame_panel",
            negative_elem="wheel_cage",
            name="left_frame_folds_clear_of_wheel_cage",
        )
        ctx.expect_gap(
            wheel,
            right_frame,
            axis="y",
            min_gap=0.020,
            positive_elem="wheel_cage",
            negative_elem="frame_panel",
            name="right_frame_folds_clear_of_wheel_cage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
