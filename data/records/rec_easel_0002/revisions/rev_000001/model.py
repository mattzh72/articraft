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
    model = ArticulatedObject(name="studio_a_frame_easel")

    oak = model.material("oak", rgba=(0.71, 0.57, 0.37, 1.0))
    walnut = model.material("walnut", rgba=(0.47, 0.33, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.18, 0.17, 0.16, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.110, 0.060, 0.050)),
        origin=Origin(xyz=(-0.290, 0.000, 0.025)),
        material=oak,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.110, 0.060, 0.050)),
        origin=Origin(xyz=(0.290, 0.000, 0.025)),
        material=oak,
        name="right_front_foot",
    )
    front_frame.visual(
        Box((0.055, 0.032, 1.490)),
        origin=Origin(xyz=(-0.185, 0.000, 0.790), rpy=(0.0, 0.140, 0.0)),
        material=oak,
        name="left_front_leg",
    )
    front_frame.visual(
        Box((0.055, 0.032, 1.490)),
        origin=Origin(xyz=(0.185, 0.000, 0.790), rpy=(0.0, -0.140, 0.0)),
        material=oak,
        name="right_front_leg",
    )
    front_frame.visual(
        Box((0.260, 0.060, 0.080)),
        origin=Origin(xyz=(0.000, -0.006, 1.580)),
        material=walnut,
        name="head_block",
    )
    front_frame.visual(
        Box((0.600, 0.034, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.330)),
        material=walnut,
        name="lower_stretcher",
    )
    front_frame.visual(
        Box((0.390, 0.030, 0.042)),
        origin=Origin(xyz=(0.000, 0.000, 0.950)),
        material=walnut,
        name="upper_stretcher",
    )
    front_frame.visual(
        Box((0.082, 0.022, 1.420)),
        origin=Origin(xyz=(0.000, -0.006, 0.870)),
        material=oak,
        name="mast_back",
    )
    front_frame.visual(
        Box((0.016, 0.022, 1.180)),
        origin=Origin(xyz=(-0.031, 0.016, 0.890)),
        material=walnut,
        name="slot_rail_left",
    )
    front_frame.visual(
        Box((0.016, 0.022, 1.180)),
        origin=Origin(xyz=(0.031, 0.016, 0.890)),
        material=walnut,
        name="slot_rail_right",
    )
    front_frame.visual(
        Box((0.240, 0.026, 0.034)),
        origin=Origin(xyz=(0.000, -0.029, 1.595)),
        material=steel,
        name="front_hinge_cleat",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.700, 0.120, 1.640)),
        mass=8.2,
        origin=Origin(xyz=(0.000, 0.000, 0.820)),
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.013, length=0.220),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_hinge_sleeve",
    )
    rear_leg.visual(
        Box((0.095, 0.034, 0.090)),
        origin=Origin(xyz=(0.000, -0.030, -0.042)),
        material=oak,
        name="rear_head_block",
    )
    rear_leg.visual(
        Box((0.040, 0.028, 1.680)),
        origin=Origin(xyz=(0.000, -0.380, -0.764), rpy=(-0.430, 0.0, 0.0)),
        material=oak,
        name="rear_strut",
    )
    rear_leg.visual(
        Box((0.110, 0.120, 0.074)),
        origin=Origin(xyz=(0.000, -0.720, -1.558)),
        material=rubber,
        name="rear_foot",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.160, 0.860, 1.720)),
        mass=2.9,
        origin=Origin(xyz=(0.000, -0.360, -0.800)),
    )

    support_ledge = model.part("support_ledge")
    support_ledge.visual(
        Box((0.034, 0.016, 0.140)),
        origin=Origin(xyz=(0.000, 0.013, 0.065)),
        material=steel,
        name="ledge_slider",
    )
    support_ledge.visual(
        Box((0.032, 0.032, 0.100)),
        origin=Origin(xyz=(0.000, 0.037, 0.050)),
        material=oak,
        name="ledge_web",
    )
    support_ledge.visual(
        Box((0.540, 0.060, 0.022)),
        origin=Origin(xyz=(0.000, 0.083, 0.011)),
        material=oak,
        name="ledge_shelf",
    )
    support_ledge.visual(
        Box((0.540, 0.018, 0.038)),
        origin=Origin(xyz=(0.000, 0.122, 0.030)),
        material=walnut,
        name="ledge_lip",
    )
    support_ledge.inertial = Inertial.from_geometry(
        Box((0.560, 0.120, 0.190)),
        mass=1.4,
        origin=Origin(xyz=(0.000, 0.060, 0.060)),
    )

    top_clamp = model.part("top_clamp")
    top_clamp.visual(
        Box((0.034, 0.016, 0.160)),
        origin=Origin(xyz=(0.000, 0.013, 0.080)),
        material=steel,
        name="clamp_slider",
    )
    top_clamp.visual(
        Box((0.032, 0.028, 0.075)),
        origin=Origin(xyz=(0.000, 0.035, 0.018)),
        material=oak,
        name="clamp_body",
    )
    top_clamp.visual(
        Box((0.220, 0.034, 0.022)),
        origin=Origin(xyz=(0.000, 0.061, -0.004)),
        material=walnut,
        name="clamp_bar",
    )
    top_clamp.visual(
        Box((0.068, 0.020, 0.058)),
        origin=Origin(xyz=(0.000, 0.051, -0.044)),
        material=oak,
        name="clamp_pad",
    )
    top_clamp.inertial = Inertial.from_geometry(
        Box((0.240, 0.100, 0.240)),
        mass=0.9,
        origin=Origin(xyz=(0.000, 0.042, 0.010)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.000, -0.055, 1.595)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.300,
            upper=0.180,
        ),
    )
    model.articulation(
        "ledge_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=support_ledge,
        origin=Origin(xyz=(0.000, 0.000, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.20,
            lower=0.0,
            upper=0.380,
        ),
    )
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=top_clamp,
        origin=Origin(xyz=(0.000, 0.000, 1.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.16,
            lower=0.0,
            upper=0.300,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    front_frame = object_model.get_part("front_frame")
    rear_leg = object_model.get_part("rear_leg")
    support_ledge = object_model.get_part("support_ledge")
    top_clamp = object_model.get_part("top_clamp")

    rear_hinge = object_model.get_articulation("rear_hinge")
    ledge_slide = object_model.get_articulation("ledge_slide")
    clamp_slide = object_model.get_articulation("clamp_slide")

    lower_stretcher = front_frame.get_visual("lower_stretcher")
    head_block = front_frame.get_visual("head_block")
    mast_back = front_frame.get_visual("mast_back")
    slot_rail_left = front_frame.get_visual("slot_rail_left")
    slot_rail_right = front_frame.get_visual("slot_rail_right")
    front_hinge_cleat = front_frame.get_visual("front_hinge_cleat")
    rear_hinge_sleeve = rear_leg.get_visual("rear_hinge_sleeve")
    rear_strut = rear_leg.get_visual("rear_strut")
    rear_foot = rear_leg.get_visual("rear_foot")
    ledge_slider = support_ledge.get_visual("ledge_slider")
    ledge_shelf = support_ledge.get_visual("ledge_shelf")
    clamp_slider = top_clamp.get_visual("clamp_slider")
    clamp_pad = top_clamp.get_visual("clamp_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "rear_hinge_is_revolute",
        rear_hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Rear leg should swing on a top revolute hinge.",
    )
    ctx.check(
        "ledge_slide_is_prismatic",
        ledge_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Canvas support should move on a vertical prismatic slot.",
    )
    ctx.check(
        "clamp_slide_is_prismatic",
        clamp_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Top clamp should adjust vertically along the mast.",
    )
    ctx.check(
        "rear_hinge_axis_runs_across_width",
        tuple(rear_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected rear hinge axis (1, 0, 0), got {rear_hinge.axis!r}.",
    )
    ctx.check(
        "slot_axes_vertical",
        tuple(ledge_slide.axis) == (0.0, 0.0, 1.0)
        and tuple(clamp_slide.axis) == (0.0, 0.0, 1.0),
        details="Both mast-mounted carriages should travel vertically.",
    )

    ctx.expect_contact(
        front_frame,
        rear_leg,
        elem_a=front_hinge_cleat,
        elem_b=rear_hinge_sleeve,
        name="rear_hinge_cleat_seated",
    )
    ctx.expect_contact(
        support_ledge,
        front_frame,
        elem_a=ledge_slider,
        elem_b=mast_back,
        name="ledge_slider_contacts_mast",
    )
    ctx.expect_contact(
        top_clamp,
        front_frame,
        elem_a=clamp_slider,
        elem_b=mast_back,
        name="clamp_slider_contacts_mast",
    )
    ctx.expect_within(
        support_ledge,
        front_frame,
        axes="x",
        inner_elem=ledge_slider,
        outer_elem=mast_back,
        name="ledge_slider_within_mast_width",
    )
    ctx.expect_within(
        top_clamp,
        front_frame,
        axes="x",
        inner_elem=clamp_slider,
        outer_elem=mast_back,
        name="clamp_slider_within_mast_width",
    )
    ctx.expect_overlap(
        support_ledge,
        front_frame,
        axes="z",
        elem_a=ledge_slider,
        elem_b=slot_rail_left,
        min_overlap=0.100,
        name="ledge_slider_engages_slot_height",
    )
    ctx.expect_overlap(
        top_clamp,
        front_frame,
        axes="z",
        elem_a=clamp_slider,
        elem_b=slot_rail_right,
        min_overlap=0.100,
        name="clamp_slider_engages_slot_height",
    )
    ctx.expect_gap(
        support_ledge,
        front_frame,
        axis="z",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem=ledge_shelf,
        negative_elem=lower_stretcher,
        name="ledge_resting_just_above_lower_stretcher",
    )
    ctx.expect_gap(
        top_clamp,
        support_ledge,
        axis="z",
        min_gap=0.550,
        positive_elem=clamp_pad,
        negative_elem=ledge_shelf,
        name="canvas_opening_between_ledge_and_clamp",
    )

    rear_limits = rear_hinge.motion_limits
    ledge_limits = ledge_slide.motion_limits
    clamp_limits = clamp_slide.motion_limits
    assert rear_limits is not None
    assert ledge_limits is not None
    assert clamp_limits is not None
    assert rear_limits.lower is not None and rear_limits.upper is not None
    assert ledge_limits.lower is not None and ledge_limits.upper is not None
    assert clamp_limits.lower is not None and clamp_limits.upper is not None

    rear_foot_rest = ctx.part_element_world_aabb(rear_leg, elem=rear_foot)
    ledge_rest = ctx.part_world_aabb(support_ledge)
    clamp_rest = ctx.part_element_world_aabb(top_clamp, elem=clamp_pad)
    assert rear_foot_rest is not None
    assert ledge_rest is not None
    assert clamp_rest is not None

    with ctx.pose({rear_hinge: rear_limits.lower}):
        rear_foot_open = ctx.part_element_world_aabb(rear_leg, elem=rear_foot)
        assert rear_foot_open is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_open_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_open_no_floating")
        ctx.expect_contact(
            front_frame,
            rear_leg,
            elem_a=front_hinge_cleat,
            elem_b=rear_hinge_sleeve,
            name="rear_hinge_open_contact",
        )
        ctx.check(
            "rear_leg_swings_farther_back_when_opened",
            rear_foot_open[0][1] < rear_foot_rest[0][1] - 0.180,
            details="Opening the easel should move the rear foot farther behind the front frame.",
        )

    with ctx.pose({rear_hinge: rear_limits.upper}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_leg, elem=rear_foot)
        assert rear_foot_folded is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_folded_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_folded_no_floating")
        ctx.expect_contact(
            front_frame,
            rear_leg,
            elem_a=front_hinge_cleat,
            elem_b=rear_hinge_sleeve,
            name="rear_hinge_folded_contact",
        )
        ctx.check(
            "rear_leg_folds_forward_from_open_pose",
            rear_foot_folded[0][1] > rear_foot_rest[0][1] + 0.100,
            details="Folding should bring the rear foot closer to the front frame.",
        )

    with ctx.pose({ledge_slide: ledge_limits.upper}):
        ledge_high = ctx.part_world_aabb(support_ledge)
        assert ledge_high is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="ledge_high_no_overlap")
        ctx.fail_if_isolated_parts(name="ledge_high_no_floating")
        ctx.expect_contact(
            support_ledge,
            front_frame,
            elem_a=ledge_slider,
            elem_b=mast_back,
            name="ledge_high_contact",
        )
        ctx.expect_gap(
            top_clamp,
            support_ledge,
            axis="z",
            min_gap=0.080,
            positive_elem=clamp_pad,
            negative_elem=ledge_shelf,
            name="ledge_high_still_below_clamp",
        )
        ctx.check(
            "ledge_rises_substantially",
            ledge_high[0][2] > ledge_rest[0][2] + 0.340,
            details="The canvas support ledge should travel most of the mast height.",
        )

    with ctx.pose({clamp_slide: clamp_limits.upper}):
        clamp_high = ctx.part_element_world_aabb(top_clamp, elem=clamp_pad)
        assert clamp_high is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="clamp_high_no_overlap")
        ctx.fail_if_isolated_parts(name="clamp_high_no_floating")
        ctx.expect_contact(
            top_clamp,
            front_frame,
            elem_a=clamp_slider,
            elem_b=mast_back,
            name="clamp_high_contact",
        )
        ctx.expect_gap(
            front_frame,
            top_clamp,
            axis="z",
            min_gap=0.010,
            positive_elem=head_block,
            negative_elem=clamp_slider,
            name="clamp_clears_head_block_at_top",
        )
        ctx.check(
            "clamp_rises_substantially",
            clamp_high[0][2] > clamp_rest[0][2] + 0.250,
            details="The top clamp should slide enough to grip canvases of different heights.",
        )

    with ctx.pose({ledge_slide: ledge_limits.upper, clamp_slide: clamp_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both_sliders_high_no_overlap")
        ctx.fail_if_isolated_parts(name="both_sliders_high_no_floating")
        ctx.expect_gap(
            top_clamp,
            support_ledge,
            axis="z",
            min_gap=0.140,
            positive_elem=clamp_pad,
            negative_elem=ledge_shelf,
            name="tall_canvas_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
