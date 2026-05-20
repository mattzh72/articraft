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
    model = ArticulatedObject(name="jaw_in_jaw_drill_vise")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.36, 0.29, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.12, 1.0))

    base_body = model.part("base_body")
    base_body.visual(
        Box((0.13, 0.26, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="base_plate",
    )
    base_body.visual(
        Box((0.020, 0.180, 0.010)),
        origin=Origin(xyz=(-0.030, -0.012, 0.019)),
        material=machined_steel,
        name="left_rail",
    )
    base_body.visual(
        Box((0.020, 0.180, 0.010)),
        origin=Origin(xyz=(0.030, -0.012, 0.019)),
        material=machined_steel,
        name="right_rail",
    )
    base_body.visual(
        Box((0.050, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.109, 0.005)),
        material=cast_iron,
        name="screw_support_base",
    )
    base_body.visual(
        Box((0.010, 0.020, 0.028)),
        origin=Origin(xyz=(-0.011, -0.109, 0.028)),
        material=cast_iron,
        name="screw_support_left_cheek",
    )
    base_body.visual(
        Box((0.010, 0.020, 0.028)),
        origin=Origin(xyz=(0.011, -0.109, 0.028)),
        material=cast_iron,
        name="screw_support_right_cheek",
    )
    base_body.visual(
        Box((0.032, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.109, 0.045)),
        material=cast_iron,
        name="screw_support_bridge",
    )
    base_body.inertial = Inertial.from_geometry(
        Box((0.13, 0.26, 0.040)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    fixed_rear_jaw = model.part("fixed_rear_jaw")
    fixed_rear_jaw.visual(
        Box((0.084, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="jaw_seat",
    )
    fixed_rear_jaw.visual(
        Box((0.102, 0.030, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=cast_iron,
        name="jaw_block",
    )
    fixed_rear_jaw.visual(
        Box((0.090, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.014, 0.023)),
        material=cast_iron,
        name="rear_web",
    )
    fixed_rear_jaw.visual(
        Box((0.090, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.018, 0.029)),
        material=dark_steel,
        name="jaw_face",
    )
    fixed_rear_jaw.inertial = Inertial.from_geometry(
        Box((0.102, 0.040, 0.052)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    primary_front_jaw = model.part("primary_front_jaw")
    primary_front_jaw.visual(
        Box((0.022, 0.036, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, 0.007)),
        material=machined_steel,
        name="left_carriage_pad",
    )
    primary_front_jaw.visual(
        Box((0.022, 0.036, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, 0.007)),
        material=machined_steel,
        name="right_carriage_pad",
    )
    primary_front_jaw.visual(
        Box((0.070, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cast_iron,
        name="bridge",
    )
    primary_front_jaw.visual(
        Box((0.100, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.004, 0.036)),
        material=cast_iron,
        name="jaw_block",
    )
    primary_front_jaw.visual(
        Box((0.090, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.034)),
        material=dark_steel,
        name="moving_jaw_face",
    )
    primary_front_jaw.visual(
        Box((0.034, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.018, 0.008)),
        material=cast_iron,
        name="nut_housing",
    )
    primary_front_jaw.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.0, -0.008, 0.061)),
        material=machined_steel,
        name="slide_post",
    )
    primary_front_jaw.inertial = Inertial.from_geometry(
        Box((0.10, 0.050, 0.080)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    auxiliary_top_jaw = model.part("auxiliary_top_jaw")
    auxiliary_top_jaw.visual(
        Box((0.008, 0.010, 0.028)),
        origin=Origin(xyz=(-0.011, 0.0, 0.014)),
        material=machined_steel,
        name="left_runner",
    )
    auxiliary_top_jaw.visual(
        Box((0.008, 0.010, 0.028)),
        origin=Origin(xyz=(0.011, 0.0, 0.014)),
        material=machined_steel,
        name="right_runner",
    )
    auxiliary_top_jaw.visual(
        Box((0.046, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.014, 0.033)),
        material=dark_steel,
        name="clamp_body",
    )
    auxiliary_top_jaw.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.014, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="clamp_shoe",
    )
    auxiliary_top_jaw.inertial = Inertial.from_geometry(
        Box((0.050, 0.030, 0.044)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.010, 0.022)),
    )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        Cylinder(radius=0.006, length=0.198),
        origin=Origin(xyz=(0.0, 0.079, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="screw_shaft",
    )
    leadscrew.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="thrust_collar",
    )
    leadscrew.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    leadscrew.inertial = Inertial.from_geometry(
        Box((0.040, 0.200, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
    )

    handle_bar = model.part("handle_bar")
    handle_bar.visual(
        Cylinder(radius=0.0045, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="cross_bar",
    )
    handle_bar.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(-0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="left_knob",
    )
    handle_bar.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="right_knob",
    )
    handle_bar.inertial = Inertial.from_geometry(
        Box((0.090, 0.020, 0.020)),
        mass=0.12,
    )

    model.articulation(
        "base_to_fixed_rear_jaw",
        ArticulationType.FIXED,
        parent=base_body,
        child=fixed_rear_jaw,
        origin=Origin(xyz=(0.0, 0.100, 0.014)),
    )
    model.articulation(
        "primary_slide",
        ArticulationType.PRISMATIC,
        parent=base_body,
        child=primary_front_jaw,
        origin=Origin(xyz=(0.0, -0.005, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.06,
            lower=0.0,
            upper=0.058,
        ),
    )
    model.articulation(
        "auxiliary_slide",
        ArticulationType.PRISMATIC,
        parent=primary_front_jaw,
        child=auxiliary_top_jaw,
        origin=Origin(xyz=(0.0, -0.008, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.03,
            lower=0.0,
            upper=0.016,
        ),
    )
    model.articulation(
        "leadscrew_spin",
        ArticulationType.CONTINUOUS,
        parent=base_body,
        child=leadscrew,
        origin=Origin(xyz=(0.0, -0.119, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=8.0),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=leadscrew,
        child=handle_bar,
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=-0.018,
            upper=0.018,
        ),
    )

    return model


def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    base_body = object_model.get_part("base_body")
    fixed_rear_jaw = object_model.get_part("fixed_rear_jaw")
    primary_front_jaw = object_model.get_part("primary_front_jaw")
    auxiliary_top_jaw = object_model.get_part("auxiliary_top_jaw")
    leadscrew = object_model.get_part("leadscrew")
    handle_bar = object_model.get_part("handle_bar")

    primary_slide = object_model.get_articulation("primary_slide")
    auxiliary_slide = object_model.get_articulation("auxiliary_slide")
    leadscrew_spin = object_model.get_articulation("leadscrew_spin")
    handle_slide = object_model.get_articulation("handle_slide")

    left_rail = base_body.get_visual("left_rail")
    right_rail = base_body.get_visual("right_rail")
    fixed_face = fixed_rear_jaw.get_visual("jaw_face")
    left_pad = primary_front_jaw.get_visual("left_carriage_pad")
    right_pad = primary_front_jaw.get_visual("right_carriage_pad")
    moving_face = primary_front_jaw.get_visual("moving_jaw_face")
    moving_body = primary_front_jaw.get_visual("jaw_block")
    nut_housing = primary_front_jaw.get_visual("nut_housing")
    slide_post = primary_front_jaw.get_visual("slide_post")
    left_runner = auxiliary_top_jaw.get_visual("left_runner")
    right_runner = auxiliary_top_jaw.get_visual("right_runner")
    clamp_shoe = auxiliary_top_jaw.get_visual("clamp_shoe")
    screw_shaft = leadscrew.get_visual("screw_shaft")
    handle_hub = leadscrew.get_visual("handle_hub")
    cross_bar = handle_bar.get_visual("cross_bar")

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
    ctx.allow_overlap(
        leadscrew,
        primary_front_jaw,
        elem_a=screw_shaft,
        elem_b=nut_housing,
        reason="The rotating leadscrew runs through an unmodeled threaded nut bore in the moving jaw.",
    )
    ctx.allow_overlap(
        leadscrew,
        handle_bar,
        elem_a=handle_hub,
        elem_b=cross_bar,
        reason="The T-bar passes through a cross-drilled hub that is not cut out explicitly.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(fixed_rear_jaw, base_body, name="fixed_jaw_mounted_to_base")
    ctx.expect_contact(leadscrew, base_body, name="leadscrew_supported_in_front_bearing")
    ctx.expect_contact(
        primary_front_jaw,
        base_body,
        elem_a=left_pad,
        elem_b=left_rail,
        name="left_pad_contacts_left_rail",
    )
    ctx.expect_contact(
        primary_front_jaw,
        base_body,
        elem_a=right_pad,
        elem_b=right_rail,
        name="right_pad_contacts_right_rail",
    )
    ctx.expect_gap(
        primary_front_jaw,
        base_body,
        axis="z",
        positive_elem=left_pad,
        negative_elem=left_rail,
        max_gap=0.0,
        max_penetration=0.0,
        name="left_pad_sits_flat_on_left_rail",
    )
    ctx.expect_gap(
        primary_front_jaw,
        base_body,
        axis="z",
        positive_elem=right_pad,
        negative_elem=right_rail,
        max_gap=0.0,
        max_penetration=0.0,
        name="right_pad_sits_flat_on_right_rail",
    )
    ctx.expect_contact(
        auxiliary_top_jaw,
        primary_front_jaw,
        elem_a=left_runner,
        elem_b=slide_post,
        name="aux_left_runner_captures_slide_post",
    )
    ctx.expect_contact(
        auxiliary_top_jaw,
        primary_front_jaw,
        elem_a=right_runner,
        elem_b=slide_post,
        name="aux_right_runner_captures_slide_post",
    )
    ctx.expect_contact(handle_bar, leadscrew, name="t_bar_passes_through_hub")
    ctx.expect_overlap(
        fixed_rear_jaw,
        primary_front_jaw,
        axes="x",
        elem_a=fixed_face,
        elem_b=moving_face,
        min_overlap=0.085,
        name="jaw_faces_align_across_width",
    )

    with ctx.pose({primary_slide: 0.0}):
        ctx.expect_gap(
            fixed_rear_jaw,
            primary_front_jaw,
            axis="y",
            positive_elem=fixed_face,
            negative_elem=moving_face,
            min_gap=0.058,
            max_gap=0.064,
            name="main_jaws_open_gap",
        )

    with ctx.pose({primary_slide: 0.058}):
        ctx.expect_gap(
            fixed_rear_jaw,
            primary_front_jaw,
            axis="y",
            positive_elem=fixed_face,
            negative_elem=moving_face,
            min_gap=0.002,
            max_gap=0.006,
            name="main_jaws_nearly_closed_gap",
        )

    with ctx.pose({auxiliary_slide: 0.0}):
        ctx.expect_gap(
            auxiliary_top_jaw,
            primary_front_jaw,
            axis="z",
            positive_elem=clamp_shoe,
            negative_elem=moving_body,
            min_gap=0.010,
            max_gap=0.014,
            name="aux_jaw_low_position_gap_above_main_jaw",
        )

    with ctx.pose({auxiliary_slide: 0.016}):
        ctx.expect_gap(
            auxiliary_top_jaw,
            primary_front_jaw,
            axis="z",
            positive_elem=clamp_shoe,
            negative_elem=moving_body,
            min_gap=0.026,
            max_gap=0.030,
            name="aux_jaw_open_position_gap_above_main_jaw",
        )

    ctx.expect_within(
        auxiliary_top_jaw,
        primary_front_jaw,
        axes="x",
        margin=0.0,
        name="auxiliary_jaw_within_primary_width",
    )

    primary_rest = ctx.part_world_position(primary_front_jaw)
    with ctx.pose({primary_slide: 0.058}):
        primary_closed = ctx.part_world_position(primary_front_jaw)
    primary_ok = (
        primary_rest is not None
        and primary_closed is not None
        and primary_closed[1] - primary_rest[1] > 0.055
        and abs(primary_closed[2] - primary_rest[2]) < 1e-6
    )
    ctx.check(
        "primary_jaw_translates_along_y",
        primary_ok,
        details=f"rest={primary_rest}, closed={primary_closed}",
    )

    aux_rest = ctx.part_world_position(auxiliary_top_jaw)
    with ctx.pose({auxiliary_slide: 0.016}):
        aux_open = ctx.part_world_position(auxiliary_top_jaw)
    aux_ok = (
        aux_rest is not None
        and aux_open is not None
        and aux_open[2] - aux_rest[2] > 0.015
        and abs(aux_open[1] - aux_rest[1]) < 1e-6
    )
    ctx.check(
        "auxiliary_jaw_translates_vertically",
        aux_ok,
        details=f"rest={aux_rest}, open={aux_open}",
    )

    with ctx.pose({handle_slide: -0.018}):
        handle_left = ctx.part_world_position(handle_bar)
    with ctx.pose({handle_slide: 0.018}):
        handle_right = ctx.part_world_position(handle_bar)
    handle_ok = (
        handle_left is not None
        and handle_right is not None
        and handle_right[0] - handle_left[0] > 0.035
        and abs(handle_right[1] - handle_left[1]) < 1e-6
    )
    ctx.check(
        "t_bar_slides_side_to_side",
        handle_ok,
        details=f"left={handle_left}, right={handle_right}",
    )

    handle_rest_aabb = ctx.part_world_aabb(handle_bar)
    with ctx.pose({leadscrew_spin: math.pi / 2.0}):
        handle_rot_aabb = ctx.part_world_aabb(handle_bar)
    rotate_ok = (
        handle_rest_aabb is not None
        and handle_rot_aabb is not None
        and _extent(handle_rest_aabb, 0) > _extent(handle_rest_aabb, 2) + 0.040
        and _extent(handle_rot_aabb, 2) > _extent(handle_rot_aabb, 0) + 0.040
    )
    ctx.check(
        "leadscrew_rotation_turns_t_bar_orientation",
        rotate_ok,
        details=f"rest_aabb={handle_rest_aabb}, rot_aabb={handle_rot_aabb}",
    )

    for joint, label in (
        (primary_slide, "primary_slide"),
        (auxiliary_slide, "auxiliary_slide"),
        (handle_slide, "handle_slide"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")

    with ctx.pose({primary_slide: 0.058, auxiliary_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_vise_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="closed_vise_pose_no_floating")

    with ctx.pose({leadscrew_spin: math.pi / 2.0, handle_slide: 0.018}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_handle_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_handle_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
