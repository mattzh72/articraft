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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _curved_strip_loop(
    *,
    center_xy: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    z: float,
    segments: int = 20,
) -> list[tuple[float, float, float]]:
    cx, cy = center_xy
    start = math.radians(start_deg)
    end = math.radians(end_deg)

    outer: list[tuple[float, float, float]] = []
    inner: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        outer.append(
            (
                cx + outer_radius * math.cos(angle),
                cy + outer_radius * math.sin(angle),
                z,
            )
        )
    for index in range(segments, -1, -1):
        t = index / segments
        angle = start + (end - start) * t
        inner.append(
            (
                cx + inner_radius * math.cos(angle),
                cy + inner_radius * math.sin(angle),
                z,
            )
        )
    return outer + inner


def _curved_panel_mesh(
    name: str,
    *,
    center_xy: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    z0: float,
    z1: float,
    segments: int = 20,
):
    return mesh_from_geometry(
        section_loft(
            [
                _curved_strip_loop(
                    center_xy=center_xy,
                    outer_radius=outer_radius,
                    inner_radius=inner_radius,
                    start_deg=start_deg,
                    end_deg=end_deg,
                    z=z0,
                    segments=segments,
                ),
                _curved_strip_loop(
                    center_xy=center_xy,
                    outer_radius=outer_radius,
                    inner_radius=inner_radius,
                    start_deg=start_deg,
                    end_deg=end_deg,
                    z=z1,
                    segments=segments,
                ),
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_glass_elevator")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.20, 0.22, 0.24, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.70, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.72, 0.90, 0.98, 0.28))
    glass_door = model.material("glass_door", rgba=(0.76, 0.92, 1.0, 0.24))
    floor_finish = model.material("floor_finish", rgba=(0.36, 0.38, 0.40, 1.0))

    tower_frame = model.part("tower_frame")
    tower_frame.visual(
        Box((1.90, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, -0.05, 0.06)),
        material=dark_frame,
        name="base_podium",
    )
    tower_frame.visual(
        Box((0.44, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, -0.72, 0.04)),
        material=dark_frame,
        name="rear_base_bridge",
    )
    tower_frame.visual(
        Box((0.38, 0.18, 4.50)),
        origin=Origin(xyz=(0.0, -0.91, 2.37)),
        material=dark_frame,
        name="rear_column",
    )
    tower_frame.visual(
        Box((0.38, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, -0.91, 0.07)),
        material=dark_frame,
        name="rear_column_foot",
    )
    tower_frame.visual(
        Box((0.18, 0.10, 4.35)),
        origin=Origin(xyz=(0.0, -0.77, 2.295)),
        material=brushed_aluminum,
        name="guide_rail",
    )
    tower_frame.visual(
        Box((0.72, 0.30, 0.22)),
        origin=Origin(xyz=(0.0, -0.84, 4.73)),
        material=dark_frame,
        name="header_cap",
    )
    tower_frame.visual(
        Box((0.44, 0.24, 0.26)),
        origin=Origin(xyz=(0.0, -0.84, 4.57)),
        material=dark_frame,
        name="head_bridge",
    )
    tower_frame.inertial = Inertial.from_geometry(
        Box((1.90, 1.20, 4.84)),
        mass=950.0,
        origin=Origin(xyz=(0.0, -0.05, 2.42)),
    )

    car_shell = model.part("car_shell")
    car_shell.visual(
        Cylinder(radius=0.72, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_frame,
        name="floor_pan",
    )
    car_shell.visual(
        Cylinder(radius=0.66, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=floor_finish,
        name="floor_finish",
    )
    car_shell.visual(
        Cylinder(radius=0.72, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.17)),
        material=dark_frame,
        name="roof_pan",
    )
    car_shell.visual(
        Box((0.26, 0.56, 2.04)),
        origin=Origin(xyz=(0.0, -0.44, 1.11)),
        material=dark_frame,
        name="rear_spine",
    )
    car_shell.visual(
        Box((0.08, 0.16, 2.04)),
        origin=Origin(xyz=(-0.40, 0.58, 1.11)),
        material=dark_frame,
        name="left_jamb",
    )
    car_shell.visual(
        Box((0.08, 0.16, 2.04)),
        origin=Origin(xyz=(0.40, 0.58, 1.11)),
        material=dark_frame,
        name="right_jamb",
    )
    car_shell.visual(
        Box((0.96, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.74, 2.09)),
        material=dark_frame,
        name="top_track",
    )
    car_shell.visual(
        Box((0.96, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.74, 0.115)),
        material=dark_frame,
        name="sill_track",
    )
    car_shell.visual(
        Box((0.06, 0.10, 0.92)),
        origin=Origin(xyz=(-0.39, 0.06, 0.55)),
        material=stainless,
        name="left_handrail_post",
    )
    car_shell.visual(
        Box((0.06, 0.10, 0.92)),
        origin=Origin(xyz=(0.39, 0.06, 0.55)),
        material=stainless,
        name="right_handrail_post",
    )
    car_shell.visual(
        _curved_panel_mesh(
            "glass_right_front",
            center_xy=(0.0, 0.0),
            outer_radius=0.72,
            inner_radius=0.696,
            start_deg=20.0,
            end_deg=60.0,
            z0=0.09,
            z1=2.13,
            segments=18,
        ),
        material=glass_clear,
        name="glass_right_front",
    )
    car_shell.visual(
        _curved_panel_mesh(
            "glass_right_rear",
            center_xy=(0.0, 0.0),
            outer_radius=0.72,
            inner_radius=0.696,
            start_deg=-48.0,
            end_deg=20.0,
            z0=0.09,
            z1=2.13,
            segments=20,
        ),
        material=glass_clear,
        name="glass_right_rear",
    )
    car_shell.visual(
        _curved_panel_mesh(
            "glass_left_front",
            center_xy=(0.0, 0.0),
            outer_radius=0.72,
            inner_radius=0.696,
            start_deg=120.0,
            end_deg=160.0,
            z0=0.09,
            z1=2.13,
            segments=18,
        ),
        material=glass_clear,
        name="glass_left_front",
    )
    car_shell.visual(
        _curved_panel_mesh(
            "glass_left_rear",
            center_xy=(0.0, 0.0),
            outer_radius=0.72,
            inner_radius=0.696,
            start_deg=160.0,
            end_deg=228.0,
            z0=0.09,
            z1=2.13,
            segments=20,
        ),
        material=glass_clear,
        name="glass_left_rear",
    )
    car_shell.inertial = Inertial.from_geometry(
        Box((1.48, 1.52, 2.21)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
    )

    sliding_door = model.part("sliding_door")
    sliding_door.visual(
        _curved_panel_mesh(
            "curved_door_panel",
            center_xy=(0.0, -0.92),
            outer_radius=0.95,
            inner_radius=0.924,
            start_deg=68.0,
            end_deg=112.0,
            z0=-0.96,
            z1=0.94,
            segments=18,
        ),
        material=glass_door,
        name="door_glass",
    )
    sliding_door.visual(
        Box((0.18, 0.08, 0.10)),
        origin=Origin(xyz=(-0.20, 0.0, 0.90)),
        material=stainless,
        name="door_hanger",
    )
    sliding_door.visual(
        Box((0.10, 0.06, 0.05)),
        origin=Origin(xyz=(-0.20, 0.0, -0.935)),
        material=stainless,
        name="door_guide",
    )
    sliding_door.inertial = Inertial.from_geometry(
        Box((0.72, 0.10, 1.95)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    handrail_bar = model.part("handrail_bar")
    handrail_bar.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(-0.32, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="left_pin",
    )
    handrail_bar.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.32, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="right_pin",
    )
    handrail_bar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.28, 0.0, 0.0),
                    (-0.24, -0.03, 0.0),
                    (-0.20, -0.08, -0.008),
                    (-0.10, -0.14, -0.010),
                    (0.0, -0.18, -0.012),
                    (0.10, -0.14, -0.010),
                    (0.20, -0.08, -0.008),
                    (0.24, -0.03, 0.0),
                    (0.28, 0.0, 0.0),
                ],
                radius=0.016,
                samples_per_segment=16,
                radial_segments=20,
            ),
            "bowed_handrail_segmented_v4",
        ),
        material=stainless,
        name="bowed_rail",
    )
    handrail_bar.inertial = Inertial.from_geometry(
        Box((0.74, 0.22, 0.05)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.08, 0.0)),
    )

    model.articulation(
        "tower_to_car",
        ArticulationType.PRISMATIC,
        parent=tower_frame,
        child=car_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15000.0,
            velocity=1.5,
            lower=0.0,
            upper=1.8,
        ),
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car_shell,
        child=sliding_door,
        origin=Origin(xyz=(0.0, 0.74, 1.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=-0.48,
            upper=0.0,
        ),
    )
    model.articulation(
        "car_to_handrail",
        ArticulationType.REVOLUTE,
        parent=car_shell,
        child=handrail_bar,
        origin=Origin(xyz=(0.0, 0.06, 0.98)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-1.10,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_frame = object_model.get_part("tower_frame")
    car_shell = object_model.get_part("car_shell")
    sliding_door = object_model.get_part("sliding_door")
    handrail_bar = object_model.get_part("handrail_bar")

    tower_to_car = object_model.get_articulation("tower_to_car")
    car_to_door = object_model.get_articulation("car_to_door")
    car_to_handrail = object_model.get_articulation("car_to_handrail")

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
        "lift_axis_is_vertical",
        tower_to_car.axis == (0.0, 0.0, 1.0),
        f"axis={tower_to_car.axis}",
    )
    ctx.check(
        "door_axis_is_lateral",
        car_to_door.axis == (1.0, 0.0, 0.0),
        f"axis={car_to_door.axis}",
    )
    ctx.check(
        "handrail_axis_is_lateral",
        car_to_handrail.axis == (1.0, 0.0, 0.0),
        f"axis={car_to_handrail.axis}",
    )

    ctx.expect_gap(
        car_shell,
        tower_frame,
        axis="y",
        positive_elem="rear_spine",
        negative_elem="guide_rail",
        max_gap=0.0,
        max_penetration=0.0,
        name="car_spine_seated_on_guide_rail",
    )
    ctx.expect_overlap(
        car_shell,
        tower_frame,
        axes="x",
        elem_a="rear_spine",
        elem_b="guide_rail",
        min_overlap=0.16,
        name="car_spine_aligned_with_guide_rail",
    )
    ctx.expect_contact(
        handrail_bar,
        car_shell,
        elem_a="left_pin",
        elem_b="left_handrail_post",
        name="handrail_left_pin_contact",
    )
    ctx.expect_contact(
        handrail_bar,
        car_shell,
        elem_a="right_pin",
        elem_b="right_handrail_post",
        name="handrail_right_pin_contact",
    )

    with ctx.pose({car_to_door: 0.0}):
        ctx.expect_gap(
            car_shell,
            sliding_door,
            axis="z",
            positive_elem="top_track",
            negative_elem="door_hanger",
            max_gap=0.0,
            max_penetration=0.0,
            name="door_closed_top_track_contact",
        )
        ctx.expect_gap(
            sliding_door,
            car_shell,
            axis="z",
            positive_elem="door_guide",
            negative_elem="sill_track",
            max_gap=0.0001,
            max_penetration=0.0,
            name="door_closed_bottom_guide_contact",
        )

    lift_limits = tower_to_car.motion_limits
    if lift_limits is not None and lift_limits.lower is not None and lift_limits.upper is not None:
        with ctx.pose({tower_to_car: lift_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tower_to_car_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tower_to_car_lower_no_floating")
            ctx.expect_gap(
                car_shell,
                tower_frame,
                axis="y",
                positive_elem="rear_spine",
                negative_elem="guide_rail",
                max_gap=0.0,
                max_penetration=0.0,
                name="tower_to_car_lower_guide_contact",
            )
        with ctx.pose({tower_to_car: lift_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tower_to_car_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tower_to_car_upper_no_floating")
            ctx.expect_gap(
                car_shell,
                tower_frame,
                axis="y",
                positive_elem="rear_spine",
                negative_elem="guide_rail",
                max_gap=0.0,
                max_penetration=0.0,
                name="tower_to_car_upper_guide_contact",
            )

    door_limits = car_to_door.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({car_to_door: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="car_to_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="car_to_door_lower_no_floating")
            ctx.expect_gap(
                car_shell,
                sliding_door,
                axis="z",
                positive_elem="top_track",
                negative_elem="door_hanger",
                max_gap=0.0,
                max_penetration=0.0,
                name="door_open_top_track_contact",
            )
            ctx.expect_gap(
                sliding_door,
                car_shell,
                axis="z",
                positive_elem="door_guide",
                negative_elem="sill_track",
                max_gap=0.0001,
                max_penetration=0.0,
                name="door_open_bottom_guide_contact",
            )
        with ctx.pose({car_to_door: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="car_to_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="car_to_door_upper_no_floating")

    handrail_limits = car_to_handrail.motion_limits
    if handrail_limits is not None and handrail_limits.lower is not None and handrail_limits.upper is not None:
        with ctx.pose({car_to_handrail: handrail_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="car_to_handrail_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="car_to_handrail_lower_no_floating")
            ctx.expect_contact(
                handrail_bar,
                car_shell,
                elem_a="left_pin",
                elem_b="left_handrail_post",
                name="handrail_lower_left_pin_contact",
            )
            ctx.expect_contact(
                handrail_bar,
                car_shell,
                elem_a="right_pin",
                elem_b="right_handrail_post",
                name="handrail_lower_right_pin_contact",
            )
        with ctx.pose({car_to_handrail: handrail_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="car_to_handrail_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="car_to_handrail_upper_no_floating")
            ctx.expect_contact(
                handrail_bar,
                car_shell,
                elem_a="left_pin",
                elem_b="left_handrail_post",
                name="handrail_upper_left_pin_contact",
            )
            ctx.expect_contact(
                handrail_bar,
                car_shell,
                elem_a="right_pin",
                elem_b="right_handrail_post",
                name="handrail_upper_right_pin_contact",
            )

    car_rest = ctx.part_world_position(car_shell)
    with ctx.pose({tower_to_car: tower_to_car.motion_limits.upper}):
        car_high = ctx.part_world_position(car_shell)
    if car_rest is None or car_high is None:
        ctx.fail("car_lift_pose_positions_available", "Could not resolve car world positions.")
    else:
        ctx.check(
            "car_travel_range_realistic",
            1.70 <= (car_high[2] - car_rest[2]) <= 1.90,
            f"travel={car_high[2] - car_rest[2]:.3f} m",
        )

    door_rest = ctx.part_world_position(sliding_door)
    with ctx.pose({car_to_door: car_to_door.motion_limits.lower}):
        door_open = ctx.part_world_position(sliding_door)
    if door_rest is None or door_open is None:
        ctx.fail("door_pose_positions_available", "Could not resolve door world positions.")
    else:
        ctx.check(
            "door_slides_openward",
            door_open[0] < door_rest[0] - 0.40,
            f"rest_x={door_rest[0]:.3f}, open_x={door_open[0]:.3f}",
        )

    rail_rest_aabb = ctx.part_element_world_aabb(handrail_bar, elem="bowed_rail")
    with ctx.pose({car_to_handrail: car_to_handrail.motion_limits.lower}):
        rail_raised_aabb = ctx.part_element_world_aabb(handrail_bar, elem="bowed_rail")
    if rail_rest_aabb is None or rail_raised_aabb is None:
        ctx.fail("handrail_pose_aabbs_available", "Could not resolve handrail AABBs.")
    else:
        ctx.check(
            "handrail_rotates_upward",
            rail_raised_aabb[1][2] > rail_rest_aabb[1][2] + 0.10,
            (
                f"rest_max_z={rail_rest_aabb[1][2]:.3f}, "
                f"raised_max_z={rail_raised_aabb[1][2]:.3f}"
            ),
        )

    car_aabb = ctx.part_world_aabb(car_shell)
    if car_aabb is None:
        ctx.fail("car_aabb_available", "Could not resolve car AABB.")
    else:
        car_dx = car_aabb[1][0] - car_aabb[0][0]
        car_dy = car_aabb[1][1] - car_aabb[0][1]
        car_dz = car_aabb[1][2] - car_aabb[0][2]
        ctx.check(
            "car_scale_reads_as_passenger_elevator",
            1.35 <= car_dx <= 1.60 and 1.35 <= car_dy <= 1.60 and 2.15 <= car_dz <= 2.35,
            f"dims=({car_dx:.3f}, {car_dy:.3f}, {car_dz:.3f})",
        )

    tower_aabb = ctx.part_world_aabb(tower_frame)
    if tower_aabb is None:
        ctx.fail("tower_aabb_available", "Could not resolve tower AABB.")
    else:
        tower_height = tower_aabb[1][2] - tower_aabb[0][2]
        ctx.check(
            "tower_height_reads_as_elevator_mast",
            tower_height >= 4.75,
            f"height={tower_height:.3f}",
        )

    with ctx.pose(
        {
            tower_to_car: tower_to_car.motion_limits.upper,
            car_to_door: car_to_door.motion_limits.lower,
            car_to_handrail: car_to_handrail.motion_limits.lower,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_operating_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_operating_pose_no_floating")
        ctx.expect_gap(
            car_shell,
            tower_frame,
            axis="y",
            positive_elem="rear_spine",
            negative_elem="guide_rail",
            max_gap=0.0,
            max_penetration=0.0,
            name="combined_pose_guide_contact",
        )
        ctx.expect_gap(
            car_shell,
            sliding_door,
            axis="z",
            positive_elem="top_track",
            negative_elem="door_hanger",
            max_gap=0.0,
            max_penetration=0.0,
            name="combined_pose_top_track_contact",
        )
        ctx.expect_contact(
            handrail_bar,
            car_shell,
            elem_a="left_pin",
            elem_b="left_handrail_post",
            name="combined_pose_left_pin_contact",
        )
        ctx.expect_contact(
            handrail_bar,
            car_shell,
            elem_a="right_pin",
            elem_b="right_handrail_post",
            name="combined_pose_right_pin_contact",
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
