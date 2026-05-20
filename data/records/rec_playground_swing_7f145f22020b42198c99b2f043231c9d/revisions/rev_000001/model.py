from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_playground_swing")

    support_metal = model.material("support_metal", rgba=(0.22, 0.28, 0.34, 1.0))
    hanger_metal = model.material("hanger_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    seat_rubber = model.material("seat_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    beam_size = (0.12, 1.80, 0.10)
    beam_center_z = 2.10
    beam_bottom_z = beam_center_z - beam_size[2] / 2.0

    frame_mount_y = 0.80
    frame_foot_x = 0.72
    frame_drop = beam_bottom_z
    leg_radius = 0.035
    apex_cap_size = (0.24, 0.10, 0.12)
    leg_top_drop = apex_cap_size[2]
    leg_vertical_span = frame_drop - leg_top_drop
    leg_length = (frame_foot_x**2 + leg_vertical_span**2) ** 0.5
    leg_pitch = atan2(frame_foot_x, leg_vertical_span)
    leg_center_z = -(leg_top_drop + leg_vertical_span / 2.0)

    pivot_spacing_y = 0.18
    mount_top_size = (0.10, 0.08, 0.016)
    mount_ear_size = (0.03, 0.008, 0.048)
    pivot_drop = 0.040
    trunnion_radius = 0.017
    trunnion_length = 0.05
    mount_ear_y = trunnion_length / 2.0 + mount_ear_size[1] / 2.0
    hanger_link_size = (0.018, 0.022, 1.34)
    hanger_bracket_size = (0.032, 0.05, 0.05)
    hanger_link_center_z = -trunnion_radius - hanger_link_size[2] / 2.0
    hanger_bracket_center_z = (
        -trunnion_radius - hanger_link_size[2] - hanger_bracket_size[2] / 2.0
    )
    seat_mount_origin_z = hanger_bracket_center_z - hanger_bracket_size[2] / 2.0

    seat_depth = 0.36
    seat_half_width = 0.18
    seat_plate_thickness = 0.016
    seat_mount_radius = 0.012
    seat_mount_length = 0.05
    seat_edge_radius = 0.018
    seat_mount_center_z = -seat_mount_radius
    seat_plate_center_z = -2.0 * seat_mount_radius - seat_plate_thickness / 2.0
    seat_edge_center_z = seat_plate_center_z - seat_plate_thickness / 2.0 - seat_edge_radius
    seat_edge_x = seat_depth / 2.0 - 0.03

    crossbeam = model.part("crossbeam")
    crossbeam.visual(
        Box(beam_size),
        origin=Origin(xyz=(0.0, 0.0, beam_center_z)),
        material=support_metal,
        name="beam",
    )

    def add_side_frame(name: str) -> None:
        frame = model.part(name)
        frame.visual(
            Box(apex_cap_size),
            origin=Origin(xyz=(0.0, 0.0, -apex_cap_size[2] / 2.0)),
            material=support_metal,
            name="apex_cap",
        )
        frame.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(
                xyz=(frame_foot_x / 2.0, 0.0, leg_center_z),
                rpy=(0.0, -leg_pitch, 0.0),
            ),
            material=support_metal,
            name="front_leg",
        )
        frame.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(
                xyz=(-frame_foot_x / 2.0, 0.0, leg_center_z),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=support_metal,
            name="rear_leg",
        )

    add_side_frame("left_frame")
    add_side_frame("right_frame")

    model.articulation(
        "crossbeam_to_left_frame",
        ArticulationType.FIXED,
        parent=crossbeam,
        child="left_frame",
        origin=Origin(xyz=(0.0, -frame_mount_y, beam_bottom_z)),
    )
    model.articulation(
        "crossbeam_to_right_frame",
        ArticulationType.FIXED,
        parent=crossbeam,
        child="right_frame",
        origin=Origin(xyz=(0.0, frame_mount_y, beam_bottom_z)),
    )

    def add_pivot_mount(name: str) -> None:
        mount = model.part(name)
        mount.visual(
            Box(mount_top_size),
            origin=Origin(xyz=(0.0, 0.0, -mount_top_size[2] / 2.0)),
            material=support_metal,
            name="top_plate",
        )
        mount.visual(
            Box(mount_ear_size),
            origin=Origin(xyz=(0.0, -mount_ear_y, -pivot_drop)),
            material=support_metal,
            name="left_ear",
        )
        mount.visual(
            Box(mount_ear_size),
            origin=Origin(xyz=(0.0, mount_ear_y, -pivot_drop)),
            material=support_metal,
            name="right_ear",
        )

    def add_hanger(name: str) -> None:
        hanger = model.part(name)
        hanger.visual(
            Cylinder(radius=trunnion_radius, length=trunnion_length),
            origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
            material=hanger_metal,
            name="top_trunnion",
        )
        hanger.visual(
            Box(hanger_link_size),
            origin=Origin(xyz=(0.0, 0.0, hanger_link_center_z)),
            material=hanger_metal,
            name="link_bar",
        )
        hanger.visual(
            Box(hanger_bracket_size),
            origin=Origin(xyz=(0.0, 0.0, hanger_bracket_center_z)),
            material=hanger_metal,
            name="seat_bracket",
        )

    add_pivot_mount("left_pivot_mount")
    add_pivot_mount("right_pivot_mount")
    add_hanger("left_hanger")
    add_hanger("right_hanger")

    model.articulation(
        "crossbeam_to_left_pivot_mount",
        ArticulationType.FIXED,
        parent=crossbeam,
        child="left_pivot_mount",
        origin=Origin(xyz=(0.0, -pivot_spacing_y, beam_bottom_z)),
    )
    model.articulation(
        "crossbeam_to_right_pivot_mount",
        ArticulationType.FIXED,
        parent=crossbeam,
        child="right_pivot_mount",
        origin=Origin(xyz=(0.0, pivot_spacing_y, beam_bottom_z)),
    )

    swing_limits = MotionLimits(
        effort=60.0,
        velocity=2.5,
        lower=-0.55,
        upper=0.55,
    )
    model.articulation(
        "left_top_pivot",
        ArticulationType.REVOLUTE,
        parent="left_pivot_mount",
        child="left_hanger",
        origin=Origin(xyz=(0.0, 0.0, -pivot_drop)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "right_top_pivot",
        ArticulationType.REVOLUTE,
        parent="right_pivot_mount",
        child="right_hanger",
        origin=Origin(xyz=(0.0, 0.0, -pivot_drop)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.55,
        ),
    )

    left_seat = model.part("left_seat_half")
    left_seat.visual(
        Cylinder(radius=seat_mount_radius, length=seat_mount_length),
        origin=Origin(
            xyz=(0.0, seat_mount_length / 2.0, seat_mount_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="mount_roller",
    )
    left_seat.visual(
        Box((seat_depth, seat_half_width, seat_plate_thickness)),
        origin=Origin(xyz=(0.0, seat_half_width / 2.0, seat_plate_center_z)),
        material=seat_rubber,
        name="belt_plate",
    )
    left_seat.visual(
        Cylinder(radius=seat_edge_radius, length=seat_half_width),
        origin=Origin(
            xyz=(seat_edge_x, seat_half_width / 2.0, seat_edge_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="front_roll",
    )
    left_seat.visual(
        Cylinder(radius=seat_edge_radius, length=seat_half_width),
        origin=Origin(
            xyz=(-seat_edge_x, seat_half_width / 2.0, seat_edge_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="rear_roll",
    )

    right_seat = model.part("right_seat_half")
    right_seat.visual(
        Cylinder(radius=seat_mount_radius, length=seat_mount_length),
        origin=Origin(
            xyz=(0.0, -seat_mount_length / 2.0, seat_mount_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="mount_roller",
    )
    right_seat.visual(
        Box((seat_depth, seat_half_width, seat_plate_thickness)),
        origin=Origin(xyz=(0.0, -seat_half_width / 2.0, seat_plate_center_z)),
        material=seat_rubber,
        name="belt_plate",
    )
    right_seat.visual(
        Cylinder(radius=seat_edge_radius, length=seat_half_width),
        origin=Origin(
            xyz=(seat_edge_x, -seat_half_width / 2.0, seat_edge_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="front_roll",
    )
    right_seat.visual(
        Cylinder(radius=seat_edge_radius, length=seat_half_width),
        origin=Origin(
            xyz=(-seat_edge_x, -seat_half_width / 2.0, seat_edge_center_z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=seat_rubber,
        name="rear_roll",
    )

    model.articulation(
        "left_hanger_to_left_seat",
        ArticulationType.FIXED,
        parent="left_hanger",
        child=left_seat,
        origin=Origin(xyz=(0.0, 0.0, seat_mount_origin_z)),
    )
    model.articulation(
        "right_hanger_to_right_seat",
        ArticulationType.FIXED,
        parent="right_hanger",
        child=right_seat,
        origin=Origin(xyz=(0.0, 0.0, seat_mount_origin_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crossbeam = object_model.get_part("crossbeam")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    left_pivot_mount = object_model.get_part("left_pivot_mount")
    right_pivot_mount = object_model.get_part("right_pivot_mount")
    left_hanger = object_model.get_part("left_hanger")
    right_hanger = object_model.get_part("right_hanger")
    left_seat = object_model.get_part("left_seat_half")
    right_seat = object_model.get_part("right_seat_half")
    left_top_pivot = object_model.get_articulation("left_top_pivot")
    right_top_pivot = object_model.get_articulation("right_top_pivot")

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

    ctx.expect_contact(left_frame, crossbeam, name="left_frame_contacts_crossbeam")
    ctx.expect_contact(right_frame, crossbeam, name="right_frame_contacts_crossbeam")
    ctx.expect_contact(left_pivot_mount, crossbeam, name="left_mount_contacts_crossbeam")
    ctx.expect_contact(right_pivot_mount, crossbeam, name="right_mount_contacts_crossbeam")
    ctx.expect_contact(left_pivot_mount, left_hanger, name="left_mount_captures_hanger")
    ctx.expect_contact(right_pivot_mount, right_hanger, name="right_mount_captures_hanger")
    ctx.expect_within(
        left_hanger,
        left_pivot_mount,
        axes="xy",
        margin=0.0,
        inner_elem="top_trunnion",
        name="left_hanger_trunnion_within_mount",
    )
    ctx.expect_within(
        right_hanger,
        right_pivot_mount,
        axes="xy",
        margin=0.0,
        inner_elem="top_trunnion",
        name="right_hanger_trunnion_within_mount",
    )
    ctx.expect_origin_gap(
        left_pivot_mount,
        left_hanger,
        axis="z",
        min_gap=0.03,
        max_gap=0.05,
        name="left_hanger_pivot_below_mount",
    )
    ctx.expect_origin_gap(
        right_pivot_mount,
        right_hanger,
        axis="z",
        min_gap=0.03,
        max_gap=0.05,
        name="right_hanger_pivot_below_mount",
    )
    ctx.expect_contact(left_hanger, left_seat, name="left_hanger_supports_seat")
    ctx.expect_contact(right_hanger, right_seat, name="right_hanger_supports_seat")
    ctx.expect_contact(left_seat, right_seat, name="seat_halves_form_one_seat")
    ctx.expect_gap(
        crossbeam,
        left_seat,
        axis="z",
        min_gap=1.35,
        max_gap=1.60,
        name="seat_hangs_well_below_crossbeam",
    )
    ctx.expect_origin_gap(
        right_frame,
        left_frame,
        axis="y",
        min_gap=1.50,
        max_gap=1.70,
        name="a_frame_side_spacing",
    )

    for pivot in (left_top_pivot, right_top_pivot):
        ctx.check(
            f"{pivot.name}_is_revolute",
            pivot.articulation_type == ArticulationType.REVOLUTE,
            details=f"{pivot.name} should be revolute",
        )
        ctx.check(
            f"{pivot.name}_axis_is_y",
            tuple(pivot.axis) == (0.0, 1.0, 0.0),
            details=f"{pivot.name} axis is {pivot.axis}, expected (0, 1, 0)",
        )
        limits = pivot.motion_limits
        ctx.check(
            f"{pivot.name}_limits_present",
            limits is not None and limits.lower == -0.55 and limits.upper == 0.55,
            details=f"{pivot.name} limits should be symmetric ±0.55 rad",
        )

    rest_pos = ctx.part_world_position(left_seat)
    upper = left_top_pivot.motion_limits.upper if left_top_pivot.motion_limits else None
    lower = left_top_pivot.motion_limits.lower if left_top_pivot.motion_limits else None

    if upper is not None:
        with ctx.pose({left_top_pivot: upper, right_top_pivot: upper}):
            posed = ctx.part_world_position(left_seat)
            ctx.check(
                "seat_swings_to_negative_x",
                rest_pos is not None
                and posed is not None
                and posed[0] < rest_pos[0] - 0.45
                and abs(posed[1] - rest_pos[1]) < 0.02
                and posed[2] > rest_pos[2] + 0.18,
                details=f"rest={rest_pos}, posed={posed}",
            )
            ctx.expect_contact(left_seat, right_seat, name="seat_halves_stay_joined_forward")
            ctx.fail_if_parts_overlap_in_current_pose(name="top_pivots_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="top_pivots_upper_no_floating")

    if lower is not None:
        with ctx.pose({left_top_pivot: lower, right_top_pivot: lower}):
            posed = ctx.part_world_position(left_seat)
            ctx.check(
                "seat_swings_to_positive_x",
                rest_pos is not None
                and posed is not None
                and posed[0] > rest_pos[0] + 0.45
                and abs(posed[1] - rest_pos[1]) < 0.02
                and posed[2] > rest_pos[2] + 0.18,
                details=f"rest={rest_pos}, posed={posed}",
            )
            ctx.expect_contact(left_seat, right_seat, name="seat_halves_stay_joined_back")
            ctx.fail_if_parts_overlap_in_current_pose(name="top_pivots_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="top_pivots_lower_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
