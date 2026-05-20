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
    model = ArticulatedObject(name="retrofit_sliding_security_gate")

    painted_frame = model.material("painted_frame", rgba=(0.24, 0.31, 0.28, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.14, 0.14, 0.15, 1.0))
    hatch_paint = model.material("hatch_paint", rgba=(0.71, 0.39, 0.15, 1.0))
    stop_red = model.material("stop_red", rgba=(0.50, 0.11, 0.10, 1.0))

    def add_bolt_head(part, xyz, *, axis: str = "z", radius: float = 0.006, length: float = 0.004) -> None:
        rpy = (0.0, 0.0, 0.0)
        if axis == "x":
            rpy = (0.0, math.pi / 2.0, 0.0)
        elif axis == "y":
            rpy = (math.pi / 2.0, 0.0, 0.0)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=dark_hardware,
        )

    support_frame = model.part("support_frame")

    # Anchor posts and base adapters.
    support_frame.visual(
        Box((0.14, 0.16, 2.38)),
        origin=Origin(xyz=(-1.15, 0.0, 1.19)),
        material=painted_frame,
        name="left_post",
    )
    support_frame.visual(
        Box((0.14, 0.14, 2.28)),
        origin=Origin(xyz=(2.05, 0.0, 1.14)),
        material=painted_frame,
        name="right_post",
    )
    support_frame.visual(
        Box((0.24, 0.22, 0.02)),
        origin=Origin(xyz=(-1.15, 0.0, 0.01)),
        material=galvanized,
        name="left_base_plate",
    )
    support_frame.visual(
        Box((0.22, 0.20, 0.02)),
        origin=Origin(xyz=(2.05, 0.0, 0.01)),
        material=galvanized,
        name="right_base_plate",
    )
    for x0, z0 in (
        (-1.21, -0.05),
        (-1.09, -0.05),
        (-1.21, 0.05),
        (-1.09, 0.05),
        (1.99, -0.045),
        (2.11, -0.045),
        (1.99, 0.045),
        (2.11, 0.045),
    ):
        add_bolt_head(support_frame, (x0, z0, 0.02))

    # Upper sliding channel.
    support_frame.visual(
        Box((3.34, 0.12, 0.02)),
        origin=Origin(xyz=(0.45, 0.0, 2.24)),
        material=galvanized,
        name="track_cap",
    )
    support_frame.visual(
        Box((3.34, 0.018, 0.08)),
        origin=Origin(xyz=(0.45, 0.051, 2.19)),
        material=galvanized,
        name="track_front_lip",
    )
    support_frame.visual(
        Box((3.34, 0.018, 0.08)),
        origin=Origin(xyz=(0.45, -0.051, 2.19)),
        material=galvanized,
        name="track_back_lip",
    )
    support_frame.visual(
        Box((0.03, 0.06, 0.09)),
        origin=Origin(xyz=(-0.80, 0.0, 2.185)),
        material=stop_red,
        name="left_end_stop",
    )
    support_frame.visual(
        Box((0.03, 0.06, 0.09)),
        origin=Origin(xyz=(1.81, 0.0, 2.185)),
        material=stop_red,
        name="right_end_stop",
    )

    # Lower guide channel to keep the leaf captive and supported.
    support_frame.visual(
        Box((3.30, 0.10, 0.03)),
        origin=Origin(xyz=(0.45, 0.0, 0.015)),
        material=galvanized,
        name="guide_floor",
    )
    support_frame.visual(
        Box((3.30, 0.015, 0.08)),
        origin=Origin(xyz=(0.45, 0.0425, 0.055)),
        material=galvanized,
        name="guide_front_wall",
    )
    support_frame.visual(
        Box((3.30, 0.015, 0.08)),
        origin=Origin(xyz=(0.45, -0.0425, 0.055)),
        material=galvanized,
        name="guide_back_wall",
    )

    # Legacy retrofit cabinet and latch-side reinforcements.
    support_frame.visual(
        Box((0.26, 0.10, 0.46)),
        origin=Origin(xyz=(-1.05, 0.13, 1.00)),
        material=painted_frame,
        name="service_cabinet",
    )
    support_frame.visual(
        Box((0.22, 0.008, 0.34)),
        origin=Origin(xyz=(-1.04, 0.183, 1.00)),
        material=hatch_paint,
        name="cabinet_hatch",
    )
    support_frame.visual(
        Box((0.018, 0.014, 0.34)),
        origin=Origin(xyz=(-1.148, 0.177, 1.00)),
        material=dark_hardware,
        name="cabinet_hinge_strip",
    )
    for x0 in (-1.12, -1.05, -0.98):
        for z0 in (0.88, 1.12):
            add_bolt_head(support_frame, (x0, 0.188, z0), axis="y", radius=0.0055, length=0.006)

    support_frame.visual(
        Box((0.20, 0.012, 0.22)),
        origin=Origin(xyz=(1.95, 0.076, 1.08)),
        material=galvanized,
        name="receiver_doubler",
    )
    support_frame.visual(
        Box((0.012, 0.07, 0.20)),
        origin=Origin(xyz=(1.971, 0.0, 1.08)),
        material=dark_hardware,
        name="receiver_back_plate",
    )
    support_frame.visual(
        Box((0.06, 0.012, 0.20)),
        origin=Origin(xyz=(1.995, 0.029, 1.08)),
        material=dark_hardware,
        name="receiver_front_cheek",
    )
    support_frame.visual(
        Box((0.06, 0.012, 0.20)),
        origin=Origin(xyz=(1.995, -0.029, 1.08)),
        material=dark_hardware,
        name="receiver_back_cheek",
    )
    for z0 in (0.98, 1.18):
        add_bolt_head(support_frame, (1.95, 0.082, z0), axis="y", radius=0.005, length=0.006)

    support_frame.visual(
        Box((0.19, 0.02, 0.22)),
        origin=Origin(xyz=(-1.06, 0.07, 2.14)),
        material=galvanized,
        name="left_track_adapter",
    )
    support_frame.visual(
        Box((0.18, 0.02, 0.18)),
        origin=Origin(xyz=(1.98, 0.06, 2.13)),
        material=galvanized,
        name="right_track_adapter",
    )

    support_frame.inertial = Inertial.from_geometry(
        Box((3.48, 0.22, 2.40)),
        mass=240.0,
        origin=Origin(xyz=(0.43, 0.0, 1.20)),
    )

    gate_leaf = model.part("gate_leaf")

    gate_leaf.visual(
        Box((0.07, 0.055, 1.93)),
        origin=Origin(xyz=(0.035, 0.0, 0.965)),
        material=painted_frame,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.07, 0.055, 1.93)),
        origin=Origin(xyz=(1.665, 0.0, 0.965)),
        material=painted_frame,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((1.70, 0.055, 0.07)),
        origin=Origin(xyz=(0.85, 0.0, 0.035)),
        material=painted_frame,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((1.70, 0.055, 0.07)),
        origin=Origin(xyz=(0.85, 0.0, 1.895)),
        material=painted_frame,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.56, 0.045, 0.05)),
        origin=Origin(xyz=(0.85, 0.0, 0.78)),
        material=painted_frame,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((1.54, 0.012, 0.68)),
        origin=Origin(xyz=(0.85, 0.0, 0.39)),
        material=galvanized,
        name="lower_sheet",
    )
    gate_leaf.visual(
        Box((1.93, 0.018, 0.04)),
        origin=Origin(
            xyz=(0.85, 0.0, 0.87),
            rpy=(0.0, math.atan2(1.26, 1.46), 0.0),
        ),
        material=galvanized,
        name="diagonal_brace",
    )
    for bar_x in (0.31, 0.58, 0.85, 1.12, 1.39):
        gate_leaf.visual(
            Box((0.038, 0.018, 1.06)),
            origin=Origin(xyz=(bar_x, 0.0, 1.332)),
            material=galvanized,
        )

    gate_leaf.visual(
        Box((0.32, 0.012, 0.36)),
        origin=Origin(xyz=(0.38, 0.009, 0.40)),
        material=hatch_paint,
        name="inspection_hatch",
    )
    gate_leaf.visual(
        Box((0.018, 0.016, 0.36)),
        origin=Origin(xyz=(0.228, 0.012, 0.40)),
        material=dark_hardware,
        name="inspection_hinge",
    )
    for x0 in (0.27, 0.49):
        for z0 in (0.25, 0.55):
            add_bolt_head(gate_leaf, (x0, 0.016, z0), axis="y", radius=0.005, length=0.006)

    gate_leaf.visual(
        Box((0.085, 0.022, 0.06)),
        origin=Origin(xyz=(1.7425, 0.0, 1.08)),
        material=dark_hardware,
        name="latch_tongue",
    )
    gate_leaf.visual(
        Box((0.13, 0.012, 0.22)),
        origin=Origin(xyz=(1.61, 0.024, 1.08)),
        material=galvanized,
        name="latch_adapter",
    )

    for carriage_x, wheel_name, bumper_name in (
        (0.20, "left_top_wheel", "left_bumper"),
        (1.50, "right_top_wheel", "right_bumper"),
    ):
        gate_leaf.visual(
            Box((0.16, 0.012, 0.06)),
            origin=Origin(xyz=(carriage_x, 0.0, 1.935)),
            material=galvanized,
        )
        gate_leaf.visual(
            Box((0.05, 0.008, 0.18)),
            origin=Origin(xyz=(carriage_x, 0.024, 2.01)),
            material=galvanized,
        )
        gate_leaf.visual(
            Box((0.05, 0.008, 0.18)),
            origin=Origin(xyz=(carriage_x, -0.024, 2.01)),
            material=galvanized,
        )
        gate_leaf.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(xyz=(carriage_x, 0.0, 2.065), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
        )
        gate_leaf.visual(
            Cylinder(radius=0.045, length=0.035),
            origin=Origin(xyz=(carriage_x, 0.0, 2.065), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
            name=wheel_name,
        )
        for z0 in (1.95, 2.06):
            add_bolt_head(gate_leaf, (carriage_x - 0.03, 0.031, z0), axis="y", radius=0.005, length=0.006)
            add_bolt_head(gate_leaf, (carriage_x + 0.03, 0.031, z0), axis="y", radius=0.005, length=0.006)
        bumper_x = carriage_x - 0.09 if carriage_x < 0.5 else carriage_x + 0.09
        gate_leaf.visual(
            Box((0.11, 0.014, 0.03)),
            origin=Origin(
                xyz=((carriage_x + bumper_x) / 2.0, 0.024, 2.085),
            ),
            material=galvanized,
        )
        gate_leaf.visual(
            Box((0.04, 0.014, 0.05)),
            origin=Origin(
                xyz=(bumper_x, 0.024, 2.055),
            ),
            material=galvanized,
        )
        gate_leaf.visual(
            Box((0.05, 0.02, 0.04)),
            origin=Origin(xyz=(bumper_x, 0.024, 2.08)),
            material=dark_hardware,
            name=bumper_name,
        )

    for shoe_x, shoe_name in ((0.23, "left_guide_shoe"), (1.47, "right_guide_shoe")):
        gate_leaf.visual(
            Box((0.10, 0.04, 0.09)),
            origin=Origin(xyz=(shoe_x, 0.0, -0.045)),
            material=dark_hardware,
            name=shoe_name,
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.79, 0.07, 2.12)),
        mass=95.0,
        origin=Origin(xyz=(0.895, 0.0, 0.98)),
    )

    model.articulation(
        "support_to_gate",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.18, 0.0, 0.12)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.35,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("support_to_gate")

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

    support_visuals = {visual.name for visual in support_frame.visuals if visual.name}
    gate_visuals = {visual.name for visual in gate_leaf.visuals if visual.name}
    ctx.check(
        "retrofit_gate_details_present",
        {
            "track_cap",
            "guide_floor",
            "cabinet_hatch",
            "receiver_back_plate",
            "left_end_stop",
            "right_end_stop",
        }.issubset(support_visuals)
        and {
            "inspection_hatch",
            "latch_tongue",
            "left_top_wheel",
            "right_top_wheel",
            "left_guide_shoe",
            "right_guide_shoe",
            "left_bumper",
            "right_bumper",
        }.issubset(gate_visuals),
        "Expected retrofit cues, latch hardware, and tracked support visuals are missing.",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="z",
            positive_elem="track_cap",
            negative_elem="left_top_wheel",
            min_gap=0.0,
            max_gap=0.001,
            name="left_roller_runs_under_track",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="z",
            positive_elem="left_guide_shoe",
            negative_elem="guide_floor",
            min_gap=0.0,
            max_gap=0.001,
            name="left_guide_shoe_seats_on_track_floor",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="y",
            positive_elem="track_front_lip",
            negative_elem="left_top_wheel",
            min_gap=0.015,
            max_gap=0.03,
            name="top_roller_front_capture_clearance",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="y",
            positive_elem="left_top_wheel",
            negative_elem="track_back_lip",
            min_gap=0.015,
            max_gap=0.03,
            name="top_roller_rear_capture_clearance",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="y",
            positive_elem="guide_front_wall",
            negative_elem="left_guide_shoe",
            min_gap=0.01,
            max_gap=0.02,
            name="lower_guide_front_clearance",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="y",
            positive_elem="left_guide_shoe",
            negative_elem="guide_back_wall",
            min_gap=0.01,
            max_gap=0.02,
            name="lower_guide_rear_clearance",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="x",
            positive_elem="receiver_back_plate",
            negative_elem="latch_tongue",
            min_gap=0.0,
            max_gap=0.001,
            name="closed_latch_seats_in_receiver",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="x",
            positive_elem="right_end_stop",
            negative_elem="right_bumper",
            min_gap=0.0,
            max_gap=0.001,
            name="closed_travel_meets_right_stop",
        )

    upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    if upper is None:
        ctx.fail("slide_has_upper_travel_limit", "Sliding gate is missing its opening travel limit.")
    else:
        with ctx.pose({slide: upper}):
            ctx.expect_gap(
                gate_leaf,
                support_frame,
                axis="z",
                positive_elem="left_guide_shoe",
                negative_elem="guide_floor",
                min_gap=0.0,
                max_gap=0.001,
                name="open_pose_keeps_lower_guide_supported",
            )
            ctx.expect_gap(
                gate_leaf,
                support_frame,
                axis="x",
                positive_elem="left_bumper",
                negative_elem="left_end_stop",
                max_penetration=1e-5,
                max_gap=0.001,
                name="open_travel_meets_left_stop",
            )

        with ctx.pose({slide: 0.0}):
            closed_pos = ctx.part_world_position(gate_leaf)
        with ctx.pose({slide: upper}):
            open_pos = ctx.part_world_position(gate_leaf)

        moved_left = (
            closed_pos is not None
            and open_pos is not None
            and open_pos[0] < closed_pos[0] - 1.0
            and abs(open_pos[1] - closed_pos[1]) < 1e-6
            and abs(open_pos[2] - closed_pos[2]) < 1e-6
        )
        ctx.check(
            "gate_leaf_slides_left_without_lift",
            moved_left,
            f"Expected leftward prismatic travel without lift, got closed={closed_pos}, open={open_pos}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
