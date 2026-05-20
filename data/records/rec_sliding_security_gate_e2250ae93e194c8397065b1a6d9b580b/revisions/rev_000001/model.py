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


LEAF_TRAVEL = 0.94


def _add_bar_frame(
    part,
    *,
    width: float,
    height: float,
    depth: float,
    stile: float,
    rail: float,
    material,
    name_prefix: str,
) -> None:
    half_w = width * 0.5
    half_h = height * 0.5
    part.visual(
        Box((stile, depth, height)),
        origin=Origin(xyz=(-half_w + stile * 0.5, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_left_stile",
    )
    part.visual(
        Box((stile, depth, height)),
        origin=Origin(xyz=(half_w - stile * 0.5, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_right_stile",
    )
    part.visual(
        Box((width, depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, half_h - rail * 0.5)),
        material=material,
        name=f"{name_prefix}_top_rail",
    )
    part.visual(
        Box((width, depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + rail * 0.5)),
        material=material,
        name=f"{name_prefix}_bottom_rail",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_sliding_security_gate")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.20, 0.22, 1.0))
    stainless = model.material("stainless", rgba=(0.69, 0.70, 0.72, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.21, 0.39, 0.68, 1.0))
    scale_white = model.material("scale_white", rgba=(0.83, 0.84, 0.85, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.80, 0.41, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.14, 2.18)),
        origin=Origin(xyz=(-1.22, 0.0, 1.09)),
        material=powder_coat,
        name="stack_post",
    )
    frame.visual(
        Box((0.12, 0.14, 2.18)),
        origin=Origin(xyz=(-0.30, 0.0, 1.09)),
        material=powder_coat,
        name="guide_jamb",
    )
    frame.visual(
        Box((0.12, 0.14, 2.18)),
        origin=Origin(xyz=(1.255, 0.0, 1.09)),
        material=powder_coat,
        name="receiver_post",
    )
    frame.visual(
        Box((2.56, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 2.09)),
        material=powder_coat,
        name="head_beam",
    )
    frame.visual(
        Box((2.56, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=powder_coat,
        name="sill_base",
    )

    frame.visual(
        Box((2.34, 0.016, 0.08)),
        origin=Origin(xyz=(0.0, -0.016, 1.96)),
        material=stainless,
        name="track_lip_rear",
    )
    frame.visual(
        Box((2.34, 0.016, 0.08)),
        origin=Origin(xyz=(0.0, 0.066, 1.96)),
        material=stainless,
        name="track_lip_front",
    )
    frame.visual(
        Box((0.03, 0.082, 0.04)),
        origin=Origin(xyz=(-1.02, 0.025, 1.98)),
        material=warning_orange,
        name="open_stop",
    )
    frame.visual(
        Box((0.03, 0.082, 0.04)),
        origin=Origin(xyz=(0.98, 0.025, 1.98)),
        material=warning_orange,
        name="closed_stop",
    )

    frame.visual(
        Box((2.12, 0.038, 0.012)),
        origin=Origin(xyz=(-0.02, -0.029, 0.106)),
        material=stainless,
        name="guide_channel_floor",
    )
    frame.visual(
        Box((2.12, 0.038, 0.012)),
        origin=Origin(xyz=(-0.02, -0.029, 0.144)),
        material=stainless,
        name="guide_channel_roof",
    )
    frame.visual(
        Box((2.12, 0.012, 0.050)),
        origin=Origin(xyz=(-0.02, -0.043, 0.125)),
        material=stainless,
        name="guide_channel_back",
    )

    frame.visual(
        Box((0.045, 0.052, 0.026)),
        origin=Origin(xyz=(1.1825, 0.025, 1.086)),
        material=stainless,
        name="receiver_jaw_top",
    )
    frame.visual(
        Box((0.045, 0.052, 0.026)),
        origin=Origin(xyz=(1.1825, 0.025, 0.914)),
        material=stainless,
        name="receiver_jaw_bottom",
    )
    frame.visual(
        Box((0.014, 0.052, 0.198)),
        origin=Origin(xyz=(1.198, 0.025, 1.00)),
        material=stainless,
        name="receiver_back",
    )
    frame.visual(
        Box((0.035, 0.010, 0.24)),
        origin=Origin(xyz=(1.1725, 0.035, 1.38)),
        material=datum_blue,
        name="receiver_datum_pad",
    )
    frame.visual(
        Box((0.030, 0.010, 0.080)),
        origin=Origin(xyz=(1.190, 0.035, 1.38)),
        material=stainless,
        name="receiver_datum_bridge",
    )

    frame.visual(
        Box((0.82, 0.006, 0.06)),
        origin=Origin(xyz=(-0.10, 0.093, 1.90)),
        material=scale_white,
        name="travel_scale",
    )
    for x_bracket in (-0.46, 0.26):
        frame.visual(
            Box((0.016, 0.006, 0.07)),
            origin=Origin(xyz=(x_bracket, 0.093, 1.965)),
            material=stainless,
        )
    for x_mark, mark_h in (
        (-0.46, 0.028),
        (-0.34, 0.040),
        (-0.22, 0.028),
        (-0.10, 0.050),
        (0.02, 0.028),
        (0.14, 0.040),
        (0.26, 0.028),
    ):
        frame.visual(
            Box((0.008, 0.006, mark_h)),
            origin=Origin(xyz=(x_mark, 0.096, 1.89)),
            material=warning_orange,
        )

    frame.inertial = Inertial.from_geometry(
        Box((2.56, 0.18, 2.18)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    leaf = model.part("gate_leaf")
    _add_bar_frame(
        leaf,
        width=1.36,
        height=1.74,
        depth=0.05,
        stile=0.07,
        rail=0.08,
        material=powder_coat,
        name_prefix="leaf",
    )
    leaf.visual(
        Box((1.06, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=powder_coat,
        name="leaf_mid_rail",
    )
    for x_bar in (-0.47, -0.31, -0.15, 0.01, 0.17, 0.33, 0.49):
        leaf.visual(
            Box((0.022, 0.022, 1.58)),
            origin=Origin(xyz=(x_bar, 0.0, 0.0)),
            material=stainless,
        )

    for x_carriage in (-0.40, 0.40):
        side = "left" if x_carriage < 0.0 else "right"
        leaf.visual(
            Box((0.05, 0.018, 0.11)),
            origin=Origin(xyz=(x_carriage, 0.0, 0.925)),
            material=stainless,
            name=f"{side}_carriage_strap",
        )
        leaf.visual(
            Box((0.08, 0.024, 0.025)),
            origin=Origin(xyz=(x_carriage, 0.0, 0.985)),
            material=stainless,
        )
        leaf.visual(
            Cylinder(radius=0.028, length=0.026),
            origin=Origin(xyz=(x_carriage, 0.0, 0.992), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=stainless,
            name=f"{side}_roller",
        )
        leaf.visual(
            Box((0.11, 0.018, 0.018)),
            origin=Origin(xyz=(x_carriage, 0.016, 0.885)),
            material=datum_blue,
            name=f"{side}_adjust_block",
        )
        leaf.visual(
            Cylinder(radius=0.006, length=0.080),
            origin=Origin(xyz=(x_carriage, 0.0, 0.925)),
            material=warning_orange,
        )

    leaf.visual(
        Box((0.090, 0.024, 0.030)),
        origin=Origin(xyz=(-0.470, 0.0, 0.985)),
        material=warning_orange,
        name="open_strike",
    )
    leaf.visual(
        Box((0.090, 0.024, 0.030)),
        origin=Origin(xyz=(0.470, 0.0, 0.985)),
        material=warning_orange,
        name="closed_strike",
    )

    leaf.visual(
        Box((0.06, 0.070, 0.008)),
        origin=Origin(xyz=(0.40, 0.044, 0.920)),
        material=scale_white,
        name="pointer_arm",
    )
    leaf.visual(
        Box((0.030, 0.010, 0.040)),
        origin=Origin(xyz=(0.40, 0.075, 0.920)),
        material=warning_orange,
        name="travel_pointer",
    )

    leaf.visual(
        Box((1.10, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.045, -0.855)),
        material=stainless,
        name="guide_tongue",
    )
    leaf.visual(
        Box((1.10, 0.011, 0.020)),
        origin=Origin(xyz=(0.0, -0.0305, -0.855)),
        material=stainless,
        name="guide_tongue_bridge",
    )

    leaf.visual(
        Box((0.06, 0.024, 0.14)),
        origin=Origin(xyz=(0.705, 0.0, 0.020)),
        material=stainless,
        name="latch_tongue",
    )
    leaf.visual(
        Box((0.040, 0.010, 0.24)),
        origin=Origin(xyz=(0.645, 0.030, 0.40)),
        material=datum_blue,
        name="leaf_datum_pad",
    )

    leaf.inertial = Inertial.from_geometry(
        Box((1.36, 0.08, 1.95)),
        mass=55.0,
        origin=Origin(),
    )

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.45, 0.025, 0.98)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=LEAF_TRAVEL,
        ),
    )

    return model
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("leaf_slide")

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

    ctx.expect_contact(leaf, frame, elem_a="left_roller", elem_b="head_beam")
    ctx.expect_contact(leaf, frame, elem_a="right_roller", elem_b="head_beam")

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="head_beam",
            negative_elem="leaf_top_rail",
            min_gap=0.14,
            max_gap=0.16,
            name="closed_head_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="leaf_bottom_rail",
            negative_elem="sill_base",
            min_gap=0.01,
            max_gap=0.02,
            name="closed_sill_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            positive_elem="leaf_left_stile",
            negative_elem="guide_jamb",
            min_gap=0.009,
            max_gap=0.011,
            name="closed_pocket_side_gap",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="receiver_back",
            negative_elem="latch_tongue",
            min_gap=0.005,
            max_gap=0.007,
            name="closed_latch_back_gap",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="receiver_jaw_top",
            negative_elem="latch_tongue",
            min_gap=0.0025,
            max_gap=0.0035,
            name="closed_latch_top_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="latch_tongue",
            negative_elem="receiver_jaw_bottom",
            min_gap=0.0025,
            max_gap=0.0035,
            name="closed_latch_bottom_gap",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="closed_strike",
            elem_b="closed_stop",
            name="closed_end_stop_seats",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="guide_channel_roof",
            negative_elem="guide_tongue",
            min_gap=0.0025,
            max_gap=0.0035,
            name="closed_upper_guide_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="guide_tongue",
            negative_elem="guide_channel_floor",
            min_gap=0.0025,
            max_gap=0.0035,
            name="closed_lower_guide_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="y",
            positive_elem="guide_tongue",
            negative_elem="guide_channel_back",
            min_gap=0.008,
            max_gap=0.0085,
            name="closed_back_guide_gap",
        )

    with ctx.pose({slide: LEAF_TRAVEL}):
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="open_strike",
            elem_b="open_stop",
            name="open_end_stop_seats",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="receiver_post",
            negative_elem="leaf_right_stile",
            min_gap=0.96,
            name="opening_clearance_when_open",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="guide_channel_roof",
            negative_elem="guide_tongue",
            min_gap=0.0025,
            max_gap=0.0035,
            name="open_upper_guide_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="guide_tongue",
            negative_elem="guide_channel_floor",
            min_gap=0.0025,
            max_gap=0.0035,
            name="open_lower_guide_gap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="y",
            positive_elem="guide_tongue",
            negative_elem="guide_channel_back",
            min_gap=0.008,
            max_gap=0.0085,
            name="open_back_guide_gap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
