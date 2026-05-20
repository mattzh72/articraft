from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    polymer = model.material("roller_polymer", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")

    frame.visual(
        Box((5.60, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="base_beam",
    )
    frame.visual(
        Box((5.60, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.15)),
        material=dark_steel,
        name="header_beam",
    )

    for name, x_pos, width in (
        ("left_outer_post", -2.75, 0.10),
        ("left_open_stop", -2.53, 0.06),
        ("right_latch_jamb", 2.695, 0.03),
        ("right_outer_post", 2.75, 0.10),
    ):
        frame.visual(
            Box((width, 0.10, 2.10)),
            origin=Origin(xyz=(x_pos, 0.0, 1.15)),
            material=dark_steel,
            name=name,
        )

    frame.visual(
        Box((5.10, 0.05, 0.03)),
        origin=Origin(xyz=(0.05, 0.0, 0.115)),
        material=galvanized,
        name="track_rail",
    )

    frame.visual(
        Box((5.10, 0.10, 0.02)),
        origin=Origin(xyz=(0.05, 0.0, 2.09)),
        material=dark_steel,
        name="guide_channel_top",
    )
    frame.visual(
        Box((5.10, 0.015, 0.08)),
        origin=Origin(xyz=(0.05, -0.0425, 2.04)),
        material=dark_steel,
        name="guide_channel_left_web",
    )
    frame.visual(
        Box((5.10, 0.015, 0.08)),
        origin=Origin(xyz=(0.05, 0.0425, 2.04)),
        material=dark_steel,
        name="guide_channel_right_web",
    )

    frame.visual(
        Box((0.02, 0.06, 0.26)),
        origin=Origin(xyz=(2.67, 0.0, 1.11)),
        material=dark_steel,
        name="receiver_backing",
    )
    frame.visual(
        Box((0.05, 0.012, 0.20)),
        origin=Origin(xyz=(2.645, -0.021, 1.11)),
        material=dark_steel,
        name="receiver_finger_front",
    )
    frame.visual(
        Box((0.05, 0.012, 0.20)),
        origin=Origin(xyz=(2.645, 0.021, 1.11)),
        material=dark_steel,
        name="receiver_finger_back",
    )

    frame.inertial = Inertial.from_geometry(
        Box((5.60, 0.18, 2.20)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    leaf = model.part("leaf")

    leaf.visual(
        Box((2.55, 0.05, 0.08)),
        origin=Origin(xyz=(1.275, 0.0, 0.24)),
        material=galvanized,
        name="bottom_rail",
    )
    leaf.visual(
        Box((2.55, 0.05, 0.08)),
        origin=Origin(xyz=(1.275, 0.0, 1.94)),
        material=galvanized,
        name="top_rail",
    )
    leaf.visual(
        Box((0.08, 0.05, 1.70)),
        origin=Origin(xyz=(0.04, 0.0, 1.11)),
        material=galvanized,
        name="left_stile",
    )
    leaf.visual(
        Box((0.08, 0.05, 1.70)),
        origin=Origin(xyz=(2.51, 0.0, 1.11)),
        material=galvanized,
        name="right_stile",
    )
    leaf.visual(
        Box((2.39, 0.03, 0.05)),
        origin=Origin(xyz=(1.275, 0.0, 1.11)),
        material=galvanized,
        name="mid_rail",
    )

    picket_x_positions = (0.22, 0.50, 0.78, 1.06, 1.34, 1.62, 1.90, 2.18, 2.36)
    for index, x_pos in enumerate(picket_x_positions, start=1):
        leaf.visual(
            Box((0.03, 0.02, 1.64)),
            origin=Origin(xyz=(x_pos, 0.0, 1.09)),
            material=dark_steel,
            name=f"picket_{index}",
        )

    roller_x_positions = (0.35, 2.20)
    roller_names = ("roller_front", "roller_rear")
    bracket_names = ("roller_front_bracket", "roller_rear_bracket")
    guide_names = ("guide_shoe_front", "guide_shoe_rear")
    guide_stem_names = ("guide_stem_front", "guide_stem_rear")
    for x_pos, wheel_name, bracket_name, guide_name, guide_stem_name in zip(
        roller_x_positions,
        roller_names,
        bracket_names,
        guide_names,
        guide_stem_names,
    ):
        leaf.visual(
            Box((0.07, 0.008, 0.12)),
            origin=Origin(xyz=(x_pos, -0.021, 0.225)),
            material=dark_steel,
            name=f"{bracket_name}_left_plate",
        )
        leaf.visual(
            Box((0.07, 0.008, 0.12)),
            origin=Origin(xyz=(x_pos, 0.021, 0.225)),
            material=dark_steel,
            name=f"{bracket_name}_right_plate",
        )
        leaf.visual(
            Cylinder(radius=0.006, length=0.046),
            origin=Origin(xyz=(x_pos, 0.0, 0.165), rpy=(1.57079632679, 0.0, 0.0)),
            material=dark_steel,
            name=f"{wheel_name}_axle",
        )
        leaf.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(xyz=(x_pos, 0.0, 0.165), rpy=(1.57079632679, 0.0, 0.0)),
            material=polymer,
            name=wheel_name,
        )
        leaf.visual(
            Box((0.07, 0.025, 0.05)),
            origin=Origin(xyz=(x_pos, 0.0, 2.00)),
            material=polymer,
            name=guide_stem_name,
        )
        leaf.visual(
            Box((0.10, 0.055, 0.03)),
            origin=Origin(xyz=(x_pos, 0.0, 2.04)),
            material=polymer,
            name=guide_name,
        )

    leaf.visual(
        Box((0.06, 0.018, 0.20)),
        origin=Origin(xyz=(2.58, 0.0, 1.11)),
        material=dark_steel,
        name="latch_tab",
    )

    leaf.inertial = Inertial.from_geometry(
        Box((2.55, 0.10, 2.05)),
        mass=95.0,
        origin=Origin(xyz=(1.275, 0.0, 1.03)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        # The leaf is authored from its left edge toward +X.
        # Negative X motion stores the gate into the left pocket.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.50,
            lower=0.0,
            upper=2.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("leaf")
    slide = object_model.get_articulation("frame_to_leaf")

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

    ctx.warn_if_articulation_overlaps(max_pose_samples=10)

    closed_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 2.55}):
        open_pos = ctx.part_world_position(leaf)

    if closed_pos is not None and open_pos is not None:
        ctx.check(
            "leaf opens toward storage pocket",
            open_pos[0] < closed_pos[0] - 2.4,
            details=f"closed_x={closed_pos[0]:.3f}, open_x={open_pos[0]:.3f}",
        )
    else:
        ctx.fail("leaf opens toward storage pocket", "could not resolve leaf world positions")

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(leaf, frame, elem_a="roller_front", elem_b="track_rail")
        ctx.expect_contact(leaf, frame, elem_a="roller_rear", elem_b="track_rail")
        ctx.expect_contact(leaf, frame, elem_a="latch_tab", elem_b="receiver_backing")
        ctx.expect_overlap(
            leaf,
            frame,
            axes="yz",
            elem_a="latch_tab",
            elem_b="receiver_backing",
            min_overlap=0.015,
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="guide_channel_top",
            negative_elem="guide_shoe_front",
            min_gap=0.015,
            max_gap=0.05,
        )

    with ctx.pose({slide: 2.55}):
        ctx.expect_contact(leaf, frame, elem_a="roller_front", elem_b="track_rail")
        ctx.expect_contact(leaf, frame, elem_a="roller_rear", elem_b="track_rail")
        ctx.expect_contact(leaf, frame, elem_a="left_stile", elem_b="left_open_stop")
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="guide_channel_top",
            negative_elem="guide_shoe_front",
            min_gap=0.015,
            max_gap=0.05,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
