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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_sliding_security_gate")

    concrete = model.material("concrete", rgba=(0.56, 0.58, 0.60, 1.0))
    powder_coat = model.material("powder_coat", rgba=(0.19, 0.21, 0.23, 1.0))
    galvanized = model.material("galvanized", rgba=(0.73, 0.75, 0.77, 1.0))
    seal_black = model.material("seal_black", rgba=(0.09, 0.10, 0.11, 1.0))

    support = model.part("support_frame")
    support.visual(
        Box((3.22, 0.30, 0.12)),
        origin=Origin(xyz=(-0.12, -0.03, 0.06)),
        material=concrete,
        name="foundation_beam",
    )
    support.visual(
        Box((3.08, 0.09, 0.02)),
        origin=Origin(xyz=(-0.12, 0.0, 0.13)),
        material=galvanized,
        name="track_floor",
    )
    support.visual(
        Box((3.08, 0.02, 0.05)),
        origin=Origin(xyz=(-0.12, 0.035, 0.155)),
        material=galvanized,
        name="track_wall_left",
    )
    support.visual(
        Box((3.08, 0.02, 0.05)),
        origin=Origin(xyz=(-0.12, -0.035, 0.155)),
        material=galvanized,
        name="track_wall_right",
    )
    support.visual(
        Box((0.14, 0.12, 1.80)),
        origin=Origin(xyz=(-1.58, -0.10, 1.02)),
        material=powder_coat,
        name="pocket_post",
    )
    support.visual(
        Box((0.16, 0.12, 1.80)),
        origin=Origin(xyz=(-0.10, -0.10, 1.02)),
        material=powder_coat,
        name="guide_post",
    )
    support.visual(
        Box((0.14, 0.12, 1.80)),
        origin=Origin(xyz=(1.35, -0.10, 1.02)),
        material=powder_coat,
        name="receiver_post",
    )
    support.visual(
        Box((3.10, 0.18, 0.12)),
        origin=Origin(xyz=(-0.115, -0.03, 1.98)),
        material=powder_coat,
        name="top_beam",
    )
    support.visual(
        Box((3.14, 0.24, 0.03)),
        origin=Origin(xyz=(-0.115, -0.03, 2.055)),
        material=galvanized,
        name="hood_cap",
    )
    support.visual(
        Box((3.14, 0.06, 0.08)),
        origin=Origin(xyz=(-0.115, 0.09, 1.88)),
        material=powder_coat,
        name="hood_lip_front",
    )
    support.visual(
        Box((3.14, 0.06, 0.08)),
        origin=Origin(xyz=(-0.115, -0.15, 1.88)),
        material=powder_coat,
        name="hood_lip_rear",
    )
    support.visual(
        Box((0.03, 0.05, 0.16)),
        origin=Origin(xyz=(1.295, -0.015, 1.13)),
        material=galvanized,
        name="receiver_keeper",
    )
    support.visual(
        Box((0.03, 0.05, 0.16)),
        origin=Origin(xyz=(-1.325, 0.0, 0.22)),
        material=galvanized,
        name="open_end_stop_post",
    )
    support.visual(
        Box((0.03, 0.05, 0.16)),
        origin=Origin(xyz=(-1.325, 0.0, 0.38)),
        material=galvanized,
        name="open_end_stop",
    )
    support.inertial = Inertial.from_geometry(
        Box((3.22, 0.30, 2.07)),
        mass=220.0,
        origin=Origin(xyz=(-0.12, -0.03, 1.035)),
    )

    gate = model.part("gate_leaf")
    gate.visual(
        Box((1.30, 0.06, 0.08)),
        origin=Origin(xyz=(0.65, 0.0, 0.11)),
        material=powder_coat,
        name="bottom_runner",
    )
    gate.visual(
        Box((1.30, 0.06, 0.08)),
        origin=Origin(xyz=(0.65, 0.0, 1.67)),
        material=powder_coat,
        name="top_runner",
    )
    gate.visual(
        Box((0.08, 0.06, 1.48)),
        origin=Origin(xyz=(0.04, 0.0, 0.89)),
        material=powder_coat,
        name="left_stile",
    )
    gate.visual(
        Box((0.08, 0.06, 1.48)),
        origin=Origin(xyz=(1.26, 0.0, 0.89)),
        material=powder_coat,
        name="right_stile",
    )
    gate.visual(
        Box((1.14, 0.04, 0.05)),
        origin=Origin(xyz=(0.65, 0.0, 0.89)),
        material=powder_coat,
        name="mid_rail",
    )
    for index, x_pos in enumerate((0.20, 0.38, 0.56, 0.74, 0.92, 1.10), start=1):
        gate.visual(
            Box((0.03, 0.02, 1.48)),
            origin=Origin(xyz=(x_pos, 0.0, 0.89)),
            material=powder_coat,
            name=f"infill_bar_{index}",
        )
    gate.visual(
        Box((1.32, 0.08, 0.02)),
        origin=Origin(xyz=(0.65, 0.0, 1.72)),
        material=seal_black,
        name="weather_cap",
    )
    gate.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.24, 0.0, 0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_wheel",
    )
    gate.visual(
        Box((0.05, 0.024, 0.014)),
        origin=Origin(xyz=(0.24, 0.0, 0.063)),
        material=galvanized,
        name="left_axle_block",
    )
    gate.visual(
        Box((0.05, 0.024, 0.07)),
        origin=Origin(xyz=(0.24, 0.0, 0.105)),
        material=powder_coat,
        name="left_hanger",
    )
    gate.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(1.06, 0.0, 0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_wheel",
    )
    gate.visual(
        Box((0.05, 0.024, 0.014)),
        origin=Origin(xyz=(1.06, 0.0, 0.063)),
        material=galvanized,
        name="right_axle_block",
    )
    gate.visual(
        Box((0.05, 0.024, 0.07)),
        origin=Origin(xyz=(1.06, 0.0, 0.105)),
        material=powder_coat,
        name="right_hanger",
    )
    gate.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(1.245, 0.0, 0.99)),
        material=powder_coat,
        name="latch_housing",
    )
    gate.visual(
        Box((0.03, 0.02, 0.08)),
        origin=Origin(xyz=(1.285, 0.0, 0.99)),
        material=galvanized,
        name="latch_bolt",
    )
    gate.visual(
        Box((0.03, 0.05, 0.12)),
        origin=Origin(xyz=(0.015, 0.0, 0.24)),
        material=galvanized,
        name="open_stop_pad",
    )
    gate.inertial = Inertial.from_geometry(
        Box((1.32, 0.08, 1.74)),
        mass=68.0,
        origin=Origin(xyz=(0.66, 0.0, 0.87)),
    )

    model.articulation(
        "support_to_gate",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate,
        origin=Origin(xyz=(-0.02, 0.0, 0.14)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("support_to_gate")

    track_floor = support.get_visual("track_floor")
    track_wall_left = support.get_visual("track_wall_left")
    track_wall_right = support.get_visual("track_wall_right")
    top_beam = support.get_visual("top_beam")
    hood_cap = support.get_visual("hood_cap")
    receiver_keeper = support.get_visual("receiver_keeper")
    open_end_stop = support.get_visual("open_end_stop")

    left_wheel = gate.get_visual("left_wheel")
    right_wheel = gate.get_visual("right_wheel")
    weather_cap = gate.get_visual("weather_cap")
    latch_bolt = gate.get_visual("latch_bolt")
    open_stop_pad = gate.get_visual("open_stop_pad")

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

    closed_origin_x = ctx.part_world_position(gate)[0]
    with ctx.pose({slide: slide.motion_limits.upper}):
        open_origin_x = ctx.part_world_position(gate)[0]
    ctx.check(
        "gate_opens_leftward",
        open_origin_x < closed_origin_x - 1.0,
        details=(
            "Gate leaf did not retract into the pocket as expected: "
            f"closed_x={closed_origin_x:.3f}, open_x={open_origin_x:.3f}"
        ),
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(gate, support, elem_a=left_wheel, elem_b=track_floor, name="left_wheel_on_track_closed")
        ctx.expect_contact(gate, support, elem_a=right_wheel, elem_b=track_floor, name="right_wheel_on_track_closed")
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            positive_elem=track_wall_left,
            negative_elem=left_wheel,
            min_gap=0.017,
            max_gap=0.0175,
            name="left_wheel_clear_of_left_wall_closed",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            positive_elem=left_wheel,
            negative_elem=track_wall_right,
            min_gap=0.017,
            max_gap=0.0175,
            name="left_wheel_clear_of_right_wall_closed",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="z",
            positive_elem=top_beam,
            negative_elem=weather_cap,
            min_gap=0.045,
            max_gap=0.055,
            name="weather_cap_tucked_under_hood_closed",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            elem_a=weather_cap,
            elem_b=hood_cap,
            min_overlap=1.30,
            name="hood_overhang_covers_gate_closed",
        )
        ctx.expect_contact(gate, support, elem_a=latch_bolt, elem_b=receiver_keeper, name="latch_seats_in_keeper_closed")

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_contact(gate, support, elem_a=left_wheel, elem_b=track_floor, name="left_wheel_on_track_open")
        ctx.expect_contact(gate, support, elem_a=right_wheel, elem_b=track_floor, name="right_wheel_on_track_open")
        ctx.expect_gap(
            support,
            gate,
            axis="z",
            positive_elem=top_beam,
            negative_elem=weather_cap,
            min_gap=0.045,
            max_gap=0.055,
            name="weather_cap_tucked_under_hood_open",
        )
        ctx.expect_overlap(
            gate,
            support,
            axes="x",
            elem_a=weather_cap,
            elem_b=hood_cap,
            min_overlap=1.30,
            name="hood_overhang_covers_gate_open",
        )
        ctx.expect_contact(gate, support, elem_a=open_stop_pad, elem_b=open_end_stop, name="open_stop_catches_end_stop")
        ctx.expect_gap(
            support,
            gate,
            axis="x",
            positive_elem=receiver_keeper,
            negative_elem=latch_bolt,
            min_gap=1.34,
            max_gap=1.39,
            name="latch_clears_receiver_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
