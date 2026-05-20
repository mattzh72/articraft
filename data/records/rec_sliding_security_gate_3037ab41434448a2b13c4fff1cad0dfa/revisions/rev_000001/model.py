from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    powder_coat = model.material("powder_coat", rgba=(0.16, 0.18, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    support_frame = model.part("support_frame")

    # Fixed frame: two outer posts plus a center jamb, all tied together by a
    # header beam and a threshold beam so the support reads as one rigid frame.
    post_size = (0.10, 0.14, 2.20)
    for name, x_pos in (
        ("left_post", -1.10),
        ("center_post", 0.00),
        ("right_post", 1.10),
    ):
        support_frame.visual(
            Box(post_size),
            origin=Origin(xyz=(x_pos, 0.0, 1.10)),
            material=powder_coat,
            name=name,
        )

    support_frame.visual(
        Box((2.30, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.15)),
        material=powder_coat,
        name="header_beam",
    )
    support_frame.visual(
        Box((2.30, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=powder_coat,
        name="threshold_beam",
    )

    # Top capture track: a deep U-channel under the header with thick side
    # cheeks so the rolling carriages visibly remain captured.
    support_frame.visual(
        Box((2.18, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 2.075)),
        material=galvanized,
        name="track_cap",
    )
    support_frame.visual(
        Box((2.18, 0.016, 0.10)),
        origin=Origin(xyz=(0.0, 0.042, 2.015)),
        material=galvanized,
        name="track_front_cheek",
    )
    support_frame.visual(
        Box((2.18, 0.016, 0.10)),
        origin=Origin(xyz=(0.0, -0.042, 2.015)),
        material=galvanized,
        name="track_back_cheek",
    )

    # A continuous bottom guide slot keeps the leaf aligned while it translates
    # between bays.
    support_frame.visual(
        Box((2.18, 0.014, 0.06)),
        origin=Origin(xyz=(0.0, 0.022, 0.11)),
        material=galvanized,
        name="guide_front_cheek",
    )
    support_frame.visual(
        Box((2.18, 0.014, 0.06)),
        origin=Origin(xyz=(0.0, -0.022, 0.11)),
        material=galvanized,
        name="guide_back_cheek",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((2.30, 0.18, 2.20)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    gate_leaf = model.part("gate_leaf")

    # The gate leaf closes the left bay at q=0 and slides right into the
    # storage bay as q increases.
    gate_leaf.visual(
        Box((0.05, 0.045, 1.74)),
        origin=Origin(xyz=(-0.445, 0.0, 1.07)),
        material=powder_coat,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.05, 0.045, 1.74)),
        origin=Origin(xyz=(0.445, 0.0, 1.07)),
        material=powder_coat,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((0.94, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.915)),
        material=powder_coat,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.94, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=powder_coat,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.84, 0.040, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
        material=powder_coat,
        name="mid_rail",
    )

    for index, x_pos in enumerate((-0.32, -0.22, -0.12, -0.02, 0.08, 0.18, 0.28, 0.38), start=1):
        gate_leaf.visual(
            Box((0.028, 0.020, 1.58)),
            origin=Origin(xyz=(x_pos, 0.0, 1.07)),
            material=powder_coat,
            name=f"picket_{index}",
        )

    # Hanger straps connect the leaf frame to the top carriages; the carriages
    # remain physically inside the track channel rather than floating nearby.
    gate_leaf.visual(
        Box((0.022, 0.012, 0.09)),
        origin=Origin(xyz=(-0.23, 0.0, 1.985)),
        material=galvanized,
        name="left_hanger_strap",
    )
    gate_leaf.visual(
        Box((0.022, 0.012, 0.09)),
        origin=Origin(xyz=(0.23, 0.0, 1.985)),
        material=galvanized,
        name="right_hanger_strap",
    )
    gate_leaf.visual(
        Box((0.10, 0.026, 0.038)),
        origin=Origin(xyz=(-0.23, 0.0, 2.046)),
        material=galvanized,
        name="front_carriage_left",
    )
    gate_leaf.visual(
        Box((0.10, 0.026, 0.038)),
        origin=Origin(xyz=(0.23, 0.0, 2.046)),
        material=galvanized,
        name="front_carriage_right",
    )

    gate_leaf.visual(
        Box((0.78, 0.016, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=rubber,
        name="bottom_tongue",
    )
    gate_leaf.visual(
        Box((0.05, 0.07, 0.18)),
        origin=Origin(xyz=(0.445, 0.0, 1.05)),
        material=galvanized,
        name="latch_box",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((0.94, 0.07, 2.06)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.35,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support_frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    closed_x = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: slide.motion_limits.upper}):
        open_x = ctx.part_world_position(gate_leaf)

        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="y",
            positive_elem="track_front_cheek",
            negative_elem="front_carriage_left",
            min_gap=0.020,
            max_gap=0.040,
            name="left carriage clears front track cheek when open",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="y",
            positive_elem="front_carriage_left",
            negative_elem="track_back_cheek",
            min_gap=0.020,
            max_gap=0.040,
            name="left carriage clears back track cheek when open",
        )
        ctx.expect_contact(
            support_frame,
            gate_leaf,
            elem_a="track_cap",
            elem_b="front_carriage_left",
            name="left carriage rides on the track cap",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="y",
            positive_elem="guide_front_cheek",
            negative_elem="bottom_tongue",
            min_gap=0.006,
            max_gap=0.020,
            name="bottom tongue clears front guide cheek when open",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="y",
            positive_elem="bottom_tongue",
            negative_elem="guide_back_cheek",
            min_gap=0.006,
            max_gap=0.020,
            name="bottom tongue clears back guide cheek when open",
        )

    ctx.check(
        "gate leaf translates into storage bay",
        closed_x is not None and open_x is not None and open_x[0] > closed_x[0] + 1.0,
        details=f"closed_position={closed_x}, open_position={open_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
