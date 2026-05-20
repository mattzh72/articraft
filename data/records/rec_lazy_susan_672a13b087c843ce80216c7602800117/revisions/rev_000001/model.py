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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


CABINET_HALF_SPAN = 0.40
CABINET_HEIGHT = 0.76
PANEL_THICKNESS = 0.024
WALL_THICKNESS = 0.018

SHAFT_CENTER_XY = (-0.165, -0.165)
SHAFT_RADIUS = 0.023
BEARING_HOLE_RADIUS = 0.032
BEARING_COLLAR_RADIUS = 0.042
BEARING_COLLAR_THICKNESS = 0.008

SHELF_RADIUS = 0.205
SHELF_THICKNESS = 0.022
SHELF_HOLE_RADIUS = 0.026
LOWER_SHELF_Z = 0.235
UPPER_SHELF_Z = 0.490
HUB_RADIUS = 0.048
HUB_LENGTH = 0.060

RAIL_PATH_RADIUS = 0.176
RAIL_TUBE_RADIUS = 0.004
RAIL_START_ANGLE = math.radians(105.0)
RAIL_END_ANGLE = math.radians(345.0)
RAIL_CENTER_OFFSET = 0.034
STANCHION_RADIUS = 0.0035
STANCHION_LENGTH = 0.028
STANCHION_CENTER_OFFSET = 0.023


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _arc_points_3d(
    radius: float,
    *,
    z: float,
    start_angle: float,
    end_angle: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(start_angle + (end_angle - start_angle) * index / samples),
            radius * math.sin(start_angle + (end_angle - start_angle) * index / samples),
            z,
        )
        for index in range(samples + 1)
    ]


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_lazy_susan")

    cabinet_wood = model.material("cabinet_wood", rgba=(0.72, 0.63, 0.54, 1.0))
    shelf_finish = model.material("shelf_finish", rgba=(0.90, 0.87, 0.79, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.84, 1.0))

    cabinet_profile = [
        (-CABINET_HALF_SPAN, -CABINET_HALF_SPAN),
        (CABINET_HALF_SPAN, -CABINET_HALF_SPAN),
        (-CABINET_HALF_SPAN, CABINET_HALF_SPAN),
    ]
    bearing_hole_profile = _circle_profile(
        BEARING_HOLE_RADIUS,
        center=SHAFT_CENTER_XY,
    )
    plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            cabinet_profile,
            [bearing_hole_profile],
            PANEL_THICKNESS,
            center=True,
        ),
        "corner_lazy_susan_plate",
    )

    shelf_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(SHELF_RADIUS, segments=72),
            [_circle_profile(SHELF_HOLE_RADIUS, center=(0.0, 0.0), segments=40)],
            SHELF_THICKNESS,
            center=True,
        ),
        "lazy_susan_round_shelf",
    )
    guard_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _arc_points_3d(
                RAIL_PATH_RADIUS,
                z=0.0,
                start_angle=RAIL_START_ANGLE,
                end_angle=RAIL_END_ANGLE,
                samples=20,
            ),
            radius=RAIL_TUBE_RADIUS,
            samples_per_segment=10,
            radial_segments=18,
            cap_ends=True,
        ),
        "lazy_susan_guard_rail",
    )

    cabinet_frame = model.part("cabinet_frame")
    cabinet_frame.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PANEL_THICKNESS / 2.0)),
        material=cabinet_wood,
        name="base_plate",
    )
    cabinet_frame.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - PANEL_THICKNESS / 2.0)),
        material=cabinet_wood,
        name="top_support",
    )
    cabinet_frame.visual(
        Box((WALL_THICKNESS, CABINET_HALF_SPAN * 2.0, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_HALF_SPAN - WALL_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_wood,
        name="left_wall",
    )
    cabinet_frame.visual(
        Box((CABINET_HALF_SPAN * 2.0, WALL_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CABINET_HALF_SPAN - WALL_THICKNESS / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_wood,
        name="right_wall",
    )
    cabinet_frame.inertial = Inertial.from_geometry(
        Box(
            (
                CABINET_HALF_SPAN * 2.0 + WALL_THICKNESS,
                CABINET_HALF_SPAN * 2.0 + WALL_THICKNESS,
                CABINET_HEIGHT,
            )
        ),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    rotating_assembly = model.part("rotating_assembly")
    shaft_length = CABINET_HEIGHT - (2.0 * PANEL_THICKNESS)
    rotating_assembly.visual(
        Cylinder(radius=SHAFT_RADIUS, length=shaft_length),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=brushed_steel,
        name="center_shaft",
    )
    rotating_assembly.visual(
        Cylinder(radius=BEARING_COLLAR_RADIUS, length=BEARING_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, PANEL_THICKNESS + BEARING_COLLAR_THICKNESS / 2.0)
        ),
        material=brushed_steel,
        name="lower_bearing_collar",
    )
    rotating_assembly.visual(
        Cylinder(radius=BEARING_COLLAR_RADIUS, length=BEARING_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CABINET_HEIGHT - PANEL_THICKNESS - BEARING_COLLAR_THICKNESS / 2.0,
            )
        ),
        material=brushed_steel,
        name="upper_bearing_collar",
    )
    rotating_assembly.visual(
        shelf_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SHELF_Z)),
        material=shelf_finish,
        name="lower_shelf",
    )
    rotating_assembly.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, LOWER_SHELF_Z)),
        material=brushed_steel,
        name="lower_hub",
    )
    rotating_assembly.visual(
        guard_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SHELF_Z + RAIL_CENTER_OFFSET)),
        material=chrome,
        name="lower_guard_rail",
    )
    for index, angle in enumerate((math.radians(135.0), math.radians(210.0), math.radians(300.0))):
        px, py = _polar_xy(RAIL_PATH_RADIUS, angle)
        rotating_assembly.visual(
            Cylinder(radius=STANCHION_RADIUS, length=STANCHION_LENGTH),
            origin=Origin(xyz=(px, py, LOWER_SHELF_Z + STANCHION_CENTER_OFFSET)),
            material=chrome,
            name=f"lower_rail_post_{index}",
        )

    rotating_assembly.visual(
        shelf_mesh,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SHELF_Z)),
        material=shelf_finish,
        name="upper_shelf",
    )
    rotating_assembly.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, UPPER_SHELF_Z)),
        material=brushed_steel,
        name="upper_hub",
    )
    rotating_assembly.visual(
        guard_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SHELF_Z + RAIL_CENTER_OFFSET)),
        material=chrome,
        name="upper_guard_rail",
    )
    for index, angle in enumerate((math.radians(135.0), math.radians(210.0), math.radians(300.0))):
        px, py = _polar_xy(RAIL_PATH_RADIUS, angle)
        rotating_assembly.visual(
            Cylinder(radius=STANCHION_RADIUS, length=STANCHION_LENGTH),
            origin=Origin(xyz=(px, py, UPPER_SHELF_Z + STANCHION_CENTER_OFFSET)),
            material=chrome,
            name=f"upper_rail_post_{index}",
        )

    knob_x, knob_y = _polar_xy(SHELF_RADIUS - 0.026, math.radians(45.0))
    rotating_assembly.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(
            xyz=(
                knob_x,
                knob_y,
                UPPER_SHELF_Z + SHELF_THICKNESS / 2.0 + 0.011,
            )
        ),
        material=chrome,
        name="front_pull_knob",
    )
    rotating_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=SHELF_RADIUS, length=CABINET_HEIGHT),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    model.articulation(
        "shelf_rotation",
        ArticulationType.CONTINUOUS,
        parent=cabinet_frame,
        child=rotating_assembly,
        origin=Origin(xyz=(SHAFT_CENTER_XY[0], SHAFT_CENTER_XY[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_frame = object_model.get_part("cabinet_frame")
    rotating_assembly = object_model.get_part("rotating_assembly")
    shelf_rotation = object_model.get_articulation("shelf_rotation")

    cabinet_frame.get_visual("base_plate")
    cabinet_frame.get_visual("top_support")
    cabinet_frame.get_visual("left_wall")
    cabinet_frame.get_visual("right_wall")
    rotating_assembly.get_visual("center_shaft")
    rotating_assembly.get_visual("lower_shelf")
    rotating_assembly.get_visual("upper_shelf")
    rotating_assembly.get_visual("lower_guard_rail")
    rotating_assembly.get_visual("upper_guard_rail")
    rotating_assembly.get_visual("front_pull_knob")

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

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_poses_no_floating")
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="sampled_rotation_clearance",
    )

    ctx.check(
        "rotation_axis_vertical",
        tuple(shelf_rotation.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical rotation axis, got {shelf_rotation.axis!r}",
    )
    limits = shelf_rotation.motion_limits
    ctx.check(
        "rotation_is_continuous",
        shelf_rotation.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        "Lazy Susan shelf assembly should spin continuously about the center shaft.",
    )

    ctx.expect_contact(
        rotating_assembly,
        cabinet_frame,
        name="shaft_assembly_supported_by_base_and_top",
    )
    ctx.expect_gap(
        rotating_assembly,
        cabinet_frame,
        axis="z",
        positive_elem="lower_shelf",
        negative_elem="base_plate",
        min_gap=0.18,
        max_gap=0.24,
        name="lower_shelf_clear_of_base_plate",
    )
    ctx.expect_gap(
        cabinet_frame,
        rotating_assembly,
        axis="z",
        positive_elem="top_support",
        negative_elem="upper_shelf",
        min_gap=0.17,
        max_gap=0.24,
        name="upper_shelf_clear_of_top_support",
    )
    ctx.expect_overlap(
        rotating_assembly,
        cabinet_frame,
        axes="xy",
        elem_a="lower_shelf",
        elem_b="base_plate",
        min_overlap=0.38,
        name="lower_shelf_over_base_footprint",
    )
    ctx.expect_overlap(
        rotating_assembly,
        cabinet_frame,
        axes="xy",
        elem_a="upper_shelf",
        elem_b="top_support",
        min_overlap=0.38,
        name="upper_shelf_over_top_footprint",
    )

    lower_shelf_aabb = ctx.part_element_world_aabb(rotating_assembly, elem="lower_shelf")
    upper_shelf_aabb = ctx.part_element_world_aabb(rotating_assembly, elem="upper_shelf")
    if lower_shelf_aabb is not None and upper_shelf_aabb is not None:
        lower_diameter = lower_shelf_aabb[1][0] - lower_shelf_aabb[0][0]
        upper_diameter = upper_shelf_aabb[1][0] - upper_shelf_aabb[0][0]
        shelf_spacing = upper_shelf_aabb[0][2] - lower_shelf_aabb[1][2]
        ctx.check(
            "round_shelf_diameter_realistic",
            0.39 <= lower_diameter <= 0.43 and 0.39 <= upper_diameter <= 0.43,
            (
                "Expected both shelves to be about 0.41 m in diameter; "
                f"got {lower_diameter:.3f} m and {upper_diameter:.3f} m."
            ),
        )
        ctx.check(
            "two_shelf_vertical_spacing_realistic",
            0.20 <= shelf_spacing <= 0.30,
            f"Expected shelf spacing between 0.20 m and 0.30 m, got {shelf_spacing:.3f} m.",
        )

    knob_rest_aabb = ctx.part_element_world_aabb(rotating_assembly, elem="front_pull_knob")
    with ctx.pose({shelf_rotation: math.pi / 2.0}):
        ctx.expect_contact(
            rotating_assembly,
            cabinet_frame,
            name="quarter_turn_pose_still_supported",
        )
        knob_turn_aabb = ctx.part_element_world_aabb(rotating_assembly, elem="front_pull_knob")
        if knob_rest_aabb is not None and knob_turn_aabb is not None:
            rest_center_x = (knob_rest_aabb[0][0] + knob_rest_aabb[1][0]) / 2.0
            rest_center_y = (knob_rest_aabb[0][1] + knob_rest_aabb[1][1]) / 2.0
            turn_center_x = (knob_turn_aabb[0][0] + knob_turn_aabb[1][0]) / 2.0
            turn_center_y = (knob_turn_aabb[0][1] + knob_turn_aabb[1][1]) / 2.0
            xy_shift = math.hypot(turn_center_x - rest_center_x, turn_center_y - rest_center_y)
            ctx.check(
                "rotation_moves_shelf_features",
                xy_shift >= 0.16,
                f"Expected shelf pull knob to move at least 0.16 m in XY, got {xy_shift:.3f} m.",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
