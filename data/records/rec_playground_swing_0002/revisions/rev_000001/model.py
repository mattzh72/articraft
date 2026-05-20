from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    pitch = math.atan2(horizontal, dz)
    midpoint = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)), length


def _add_segment_visual(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _aabb_center_x(ctx: TestContext, part, elem) -> float | None:
    bounds = ctx.part_element_world_aabb(part, elem=elem)
    if bounds is None:
        return None
    lower, upper = bounds
    return 0.5 * (lower[0] + upper[0])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_swing", assets=ASSETS)

    support_green = model.material("support_green", rgba=(0.22, 0.42, 0.27, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.66, 0.68, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.18, 0.18, 0.20, 1.0))
    seat_black = model.material("seat_black", rgba=(0.12, 0.12, 0.13, 1.0))

    beam_z = 2.20
    beam_half_y = 0.76
    side_frame_y = 0.64
    leg_foot_x = 0.78
    leg_radius = 0.04
    beam_radius = 0.05
    spreader_z = 0.72
    pivot_z = 2.12
    pivot_half_y = 0.17
    pin_length = 0.04

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((1.70, 1.60, beam_z)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, beam_z / 2.0)),
    )

    _add_segment_visual(
        support,
        name="crossbeam",
        start=(0.0, -beam_half_y, beam_z),
        end=(0.0, beam_half_y, beam_z),
        radius=beam_radius,
        material=support_green,
    )

    left_apex = (0.0, -side_frame_y, beam_z)
    right_apex = (0.0, side_frame_y, beam_z)
    left_front_foot = (leg_foot_x, -side_frame_y, 0.0)
    left_rear_foot = (-leg_foot_x, -side_frame_y, 0.0)
    right_front_foot = (leg_foot_x, side_frame_y, 0.0)
    right_rear_foot = (-leg_foot_x, side_frame_y, 0.0)

    _add_segment_visual(
        support,
        name="left_front_leg",
        start=left_front_foot,
        end=left_apex,
        radius=leg_radius,
        material=support_green,
    )
    _add_segment_visual(
        support,
        name="left_rear_leg",
        start=left_rear_foot,
        end=left_apex,
        radius=leg_radius,
        material=support_green,
    )
    _add_segment_visual(
        support,
        name="right_front_leg",
        start=right_front_foot,
        end=right_apex,
        radius=leg_radius,
        material=support_green,
    )
    _add_segment_visual(
        support,
        name="right_rear_leg",
        start=right_rear_foot,
        end=right_apex,
        radius=leg_radius,
        material=support_green,
    )

    spreader_half_x = leg_foot_x * (1.0 - spreader_z / beam_z)
    _add_segment_visual(
        support,
        name="left_spreader",
        start=(-spreader_half_x, -side_frame_y, spreader_z),
        end=(spreader_half_x, -side_frame_y, spreader_z),
        radius=0.025,
        material=support_green,
    )
    _add_segment_visual(
        support,
        name="right_spreader",
        start=(-spreader_half_x, side_frame_y, spreader_z),
        end=(spreader_half_x, side_frame_y, spreader_z),
        radius=0.025,
        material=support_green,
    )

    plate_size = (0.028, 0.006, 0.065)
    plate_center_offset = 0.017
    plate_center_z = pivot_z + (plate_size[2] / 2.0) - 0.03
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        pivot_y = side_sign * pivot_half_y
        for plate_name, plate_sign in (("inner", -1.0), ("outer", 1.0)):
            support.visual(
                Box(plate_size),
                origin=Origin(
                    xyz=(
                        0.0,
                        pivot_y + plate_sign * plate_center_offset,
                        plate_center_z,
                    )
                ),
                material=support_green,
                name=f"{side_name}_{plate_name}_hanger_plate",
            )
        _add_segment_visual(
            support,
            name=f"{side_name}_pivot_pin",
            start=(0.0, pivot_y - pin_length / 2.0, pivot_z),
            end=(0.0, pivot_y + pin_length / 2.0, pivot_z),
            radius=0.0045,
            material=hardware_gray,
        )

    seat_center_z = 0.60
    seat_size = (0.18, 0.46, 0.04)
    seat_top_relative_z = seat_center_z - pivot_z + seat_size[2] / 2.0

    swing_seat = model.part("swing_seat")
    swing_seat.inertial = Inertial.from_geometry(
        Box((seat_size[0], 0.38, pivot_z - seat_center_z + 0.04)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (seat_center_z - pivot_z))),
    )

    swing_seat.visual(
        Box(seat_size),
        origin=Origin(xyz=(0.0, 0.0, seat_center_z - pivot_z)),
        material=seat_black,
        name="seat_panel",
    )

    hub_radius = 0.012
    hub_length = 0.022
    rod_radius = 0.009
    rod_top_attach_z = -0.002
    rod_bottom_embed = 0.008

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        pivot_y = side_sign * pivot_half_y
        _add_segment_visual(
            swing_seat,
            name=f"{side_name}_pivot_hub",
            start=(0.0, pivot_y - hub_length / 2.0, 0.0),
            end=(0.0, pivot_y + hub_length / 2.0, 0.0),
            radius=hub_radius,
            material=dark_hardware,
        )
        _add_segment_visual(
            swing_seat,
            name=f"{side_name}_hanger_rod",
            start=(0.0, pivot_y, rod_top_attach_z),
            end=(0.0, pivot_y, seat_top_relative_z - rod_bottom_embed),
            radius=rod_radius,
            material=hardware_gray,
        )

    model.articulation(
        "seat_swing",
        ArticulationType.REVOLUTE,
        parent=support,
        child=swing_seat,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.5,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support_frame")
    swing_seat = object_model.get_part("swing_seat")
    swing_joint = object_model.get_articulation("seat_swing")

    crossbeam = support.get_visual("crossbeam")
    left_pin = support.get_visual("left_pivot_pin")
    right_pin = support.get_visual("right_pivot_pin")
    seat_panel = swing_seat.get_visual("seat_panel")
    left_hub = swing_seat.get_visual("left_pivot_hub")
    right_hub = swing_seat.get_visual("right_pivot_hub")
    left_rod = swing_seat.get_visual("left_hanger_rod")
    right_rod = swing_seat.get_visual("right_hanger_rod")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        support,
        swing_seat,
        elem_a=left_pin,
        elem_b=left_hub,
        reason="Left top pivot is represented as a coaxial pin-and-hub overlap around the revolute axis.",
    )
    ctx.allow_overlap(
        support,
        swing_seat,
        elem_a=right_pin,
        elem_b=right_hub,
        reason="Right top pivot is represented as a coaxial pin-and-hub overlap around the revolute axis.",
    )
    ctx.allow_overlap(
        support,
        swing_seat,
        elem_a=left_pin,
        elem_b=left_rod,
        reason="The simplified left hanger rod includes its top eye in the same cylinder, so the support pin intentionally penetrates the rod volume at the hinge.",
    )
    ctx.allow_overlap(
        support,
        swing_seat,
        elem_a=right_pin,
        elem_b=right_rod,
        reason="The simplified right hanger rod includes its top eye in the same cylinder, so the support pin intentionally penetrates the rod volume at the hinge.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        swing_seat,
        support,
        axes="y",
        inner_elem=seat_panel,
        outer_elem=crossbeam,
        margin=0.0,
        name="seat_stays_between_side_frames_laterally",
    )
    ctx.expect_gap(
        support,
        swing_seat,
        axis="z",
        positive_elem=crossbeam,
        negative_elem=seat_panel,
        min_gap=1.40,
        max_gap=1.65,
        name="seat_hangs_well_below_crossbeam_at_rest",
    )
    ctx.expect_overlap(
        support,
        swing_seat,
        axes="yz",
        elem_a=left_pin,
        elem_b=left_hub,
        min_overlap=0.008,
        name="left_top_pivot_hub_is_centered_on_pin",
    )
    ctx.expect_overlap(
        support,
        swing_seat,
        axes="yz",
        elem_a=right_pin,
        elem_b=right_hub,
        min_overlap=0.008,
        name="right_top_pivot_hub_is_centered_on_pin",
    )

    axis_ok = tuple(round(v, 6) for v in swing_joint.axis) == (0.0, 1.0, 0.0)
    ctx.check("swing_axis_runs_along_crossbeam", axis_ok, f"axis={swing_joint.axis!r}")

    rest_center_x = _aabb_center_x(ctx, swing_seat, seat_panel)
    ctx.check(
        "seat_is_centered_at_rest",
        rest_center_x is not None and abs(rest_center_x) <= 0.02,
        f"seat center x at rest was {rest_center_x!r}",
    )

    with ctx.pose({swing_joint: math.pi / 4.0}):
        ctx.expect_within(
            swing_seat,
            support,
            axes="y",
            inner_elem=seat_panel,
            outer_elem=crossbeam,
            margin=0.0,
            name="seat_remains_between_side_frames_at_forward_swing",
        )
        ctx.expect_gap(
            support,
            swing_seat,
            axis="z",
            positive_elem=crossbeam,
            negative_elem=seat_panel,
            min_gap=0.95,
            max_gap=1.20,
            name="seat_remains_below_crossbeam_at_forward_swing",
        )
        forward_center_x = _aabb_center_x(ctx, swing_seat, seat_panel)

    with ctx.pose({swing_joint: -math.pi / 4.0}):
        ctx.expect_within(
            swing_seat,
            support,
            axes="y",
            inner_elem=seat_panel,
            outer_elem=crossbeam,
            margin=0.0,
            name="seat_remains_between_side_frames_at_back_swing",
        )
        ctx.expect_gap(
            support,
            swing_seat,
            axis="z",
            positive_elem=crossbeam,
            negative_elem=seat_panel,
            min_gap=0.95,
            max_gap=1.20,
            name="seat_remains_below_crossbeam_at_back_swing",
        )
        backward_center_x = _aabb_center_x(ctx, swing_seat, seat_panel)

    swing_motion_ok = (
        forward_center_x is not None
        and backward_center_x is not None
        and abs(forward_center_x) >= 0.9
        and abs(backward_center_x) >= 0.9
        and forward_center_x * backward_center_x < 0.0
    )
    ctx.check(
        "seat_swings_in_opposite_directions_at_joint_limits",
        swing_motion_ok,
        f"forward_center_x={forward_center_x!r}, backward_center_x={backward_center_x!r}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
