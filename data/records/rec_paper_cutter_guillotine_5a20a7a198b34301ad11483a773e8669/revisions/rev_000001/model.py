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
    model = ArticulatedObject(name="desktop_guillotine_paper_cutter")

    base_grey = model.material("base_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_red = model.material("safety_red", rgba=(0.72, 0.12, 0.12, 1.0))
    handle_orange = model.material("handle_orange", rgba=(0.86, 0.39, 0.14, 1.0))

    bed_width = 0.48
    bed_depth = 0.34
    plinth_width = 0.42
    plinth_depth = 0.27
    plinth_height = 0.036
    bed_thickness = 0.014
    bed_top_z = plinth_height + bed_thickness

    hinge_x = bed_width * 0.5 - 0.018
    hinge_y = bed_depth * 0.5 - 0.045
    hinge_z = bed_top_z + 0.018
    arm_tip_x = -bed_width * 0.5 + 0.050
    arm_tip_y = -bed_depth * 0.5 + 0.024
    cut_angle = math.atan2(arm_tip_y - hinge_y, arm_tip_x - hinge_x)
    cut_dx = math.cos(cut_angle)
    cut_dy = math.sin(cut_angle)
    normal_dx = -math.sin(cut_angle)
    normal_dy = math.cos(cut_angle)

    def offset_from_hinge(along: float, normal: float = 0.0) -> tuple[float, float]:
        return (
            hinge_x + cut_dx * along + normal_dx * normal,
            hinge_y + cut_dy * along + normal_dy * normal,
        )

    blade_strip_along = 0.212
    blade_strip_normal = -0.011
    hold_down_along = 0.270
    hold_down_normal = -0.050

    cut_line_center_x, cut_line_center_y = offset_from_hinge(blade_strip_along, blade_strip_normal)
    hold_down_center_x, hold_down_center_y = offset_from_hinge(hold_down_along, hold_down_normal)
    left_guide_x, left_guide_y = offset_from_hinge(hold_down_along - 0.055, hold_down_normal)
    right_guide_x, right_guide_y = offset_from_hinge(hold_down_along + 0.055, hold_down_normal)

    base = model.part("base")
    base.visual(
        Box((plinth_width, plinth_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=dark_metal,
        name="base_plinth",
    )
    base.visual(
        Box((bed_width, bed_depth, bed_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + bed_thickness * 0.5)),
        material=base_grey,
        name="bed_plate",
    )
    base.visual(
        Box((0.34, 0.016, 0.024)),
        origin=Origin(xyz=(-0.058, bed_depth * 0.5 - 0.008, bed_top_z + 0.012)),
        material=steel,
        name="rear_fence",
    )
    base.visual(
        Box((0.41, 0.006, 0.002)),
        origin=Origin(
            xyz=(cut_line_center_x, cut_line_center_y, bed_top_z + 0.001),
            rpy=(0.0, 0.0, cut_angle),
        ),
        material=safety_red,
        name="cut_line_strip",
    )
    base.visual(
        Box((0.38, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, -bed_depth * 0.5 + 0.018, bed_top_z + 0.001)),
        material=steel,
        name="front_scale_strip",
    )
    base.visual(
        Box((0.040, 0.024, 0.006)),
        origin=Origin(xyz=(hinge_x, hinge_y, bed_top_z + 0.002)),
        material=dark_metal,
        name="pivot_bracket_base",
    )
    base.visual(
        Box((0.005, 0.024, 0.034)),
        origin=Origin(xyz=(hinge_x - 0.022, hinge_y + 0.011, bed_top_z + 0.017)),
        material=dark_metal,
        name="left_bracket_cheek",
    )
    base.visual(
        Box((0.005, 0.024, 0.034)),
        origin=Origin(xyz=(hinge_x + 0.022, hinge_y + 0.011, bed_top_z + 0.017)),
        material=dark_metal,
        name="right_bracket_cheek",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.039),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )

    for guide_name, guide_x, guide_y in (
        ("left", left_guide_x, left_guide_y),
        ("right", right_guide_x, right_guide_y),
    ):
        base.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(guide_x, guide_y, bed_top_z + 0.020)),
            material=steel,
            name=f"{guide_name}_guide_rod",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(guide_x, guide_y, bed_top_z + 0.005)),
            material=dark_metal,
            name=f"{guide_name}_guide_base",
        )

    base.inertial = Inertial.from_geometry(
        Box((bed_width, bed_depth, 0.094)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_eye",
    )
    blade_arm.visual(
        Box((0.390, 0.028, 0.014)),
        origin=Origin(xyz=(0.211, 0.0, 0.0)),
        material=dark_metal,
        name="arm_beam",
    )
    blade_arm.visual(
        Box((0.420, 0.006, 0.008)),
        origin=Origin(xyz=(0.228, blade_strip_normal, -0.010)),
        material=steel,
        name="blade_strip",
    )
    blade_arm.visual(
        Box((0.080, 0.024, 0.020)),
        origin=Origin(xyz=(0.396, 0.0, 0.017)),
        material=dark_metal,
        name="handle_mount",
    )
    blade_arm.visual(
        Box((0.082, 0.030, 0.028)),
        origin=Origin(xyz=(0.462, 0.0, 0.030)),
        material=handle_orange,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.540, 0.060, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.270, 0.0, 0.004)),
    )

    hold_down_bar = model.part("hold_down_bar")
    hold_down_bar.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=steel,
        name="left_slider_sleeve",
    )
    hold_down_bar.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=steel,
        name="right_slider_sleeve",
    )
    hold_down_bar.visual(
        Box((0.150, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, -0.014)),
        material=dark_metal,
        name="clamp_bar",
    )
    hold_down_bar.visual(
        Box((0.144, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, -0.021)),
        material=rubber,
        name="clamp_pad",
    )
    hold_down_bar.visual(
        Box((0.038, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_metal,
        name="bar_handle",
    )
    hold_down_bar.inertial = Inertial.from_geometry(
        Box((0.200, 0.030, 0.040)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.006, -0.004)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, 0.0, cut_angle)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "base_to_hold_down_bar",
        ArticulationType.PRISMATIC,
        parent=base,
        child=hold_down_bar,
        origin=Origin(
            xyz=(hold_down_center_x, hold_down_center_y, bed_top_z + 0.035),
            rpy=(0.0, 0.0, cut_angle),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    hold_down_bar = object_model.get_part("hold_down_bar")

    blade_hinge = object_model.get_articulation("base_to_blade_arm")
    hold_down_slide = object_model.get_articulation("base_to_hold_down_bar")

    bed_plate = base.get_visual("bed_plate")
    rear_fence = base.get_visual("rear_fence")
    cut_line_strip = base.get_visual("cut_line_strip")
    pivot_pin = base.get_visual("pivot_pin")
    pivot_bracket_base = base.get_visual("pivot_bracket_base")
    left_bracket_cheek = base.get_visual("left_bracket_cheek")
    left_guide_rod = base.get_visual("left_guide_rod")
    right_guide_rod = base.get_visual("right_guide_rod")
    left_guide_base = base.get_visual("left_guide_base")
    right_guide_base = base.get_visual("right_guide_base")

    pivot_eye = blade_arm.get_visual("pivot_eye")
    arm_beam = blade_arm.get_visual("arm_beam")
    blade_strip = blade_arm.get_visual("blade_strip")
    handle_grip = blade_arm.get_visual("handle_grip")

    left_slider_sleeve = hold_down_bar.get_visual("left_slider_sleeve")
    right_slider_sleeve = hold_down_bar.get_visual("right_slider_sleeve")
    clamp_bar = hold_down_bar.get_visual("clamp_bar")
    clamp_pad = hold_down_bar.get_visual("clamp_pad")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        blade_arm,
        elem_a=pivot_pin,
        elem_b=pivot_eye,
        reason="The rear-corner hinge is represented as a solid pivot pin passing through the blade arm pivot eye.",
    )
    ctx.allow_overlap(
        base,
        blade_arm,
        elem_a=pivot_bracket_base,
        elem_b=pivot_eye,
        reason="The pivot eye nests within the rear hinge bracket pocket during blade rotation.",
    )
    ctx.allow_overlap(
        base,
        blade_arm,
        elem_a=pivot_bracket_base,
        elem_b=blade_strip,
        reason="The sharpened blade strip enters the hinge pocket at the rear end of the cutter.",
    )
    ctx.allow_overlap(
        base,
        blade_arm,
        elem_a=pivot_pin,
        elem_b=arm_beam,
        reason="The blade arm's rear knuckle collar is simplified into the arm beam and rotates around the pivot pin.",
    )
    ctx.allow_overlap(
        base,
        blade_arm,
        elem_a=left_bracket_cheek,
        elem_b=blade_strip,
        reason="The rear blade heel runs inside the left hinge cheek near the pivot.",
    )
    for rod, sleeve, side in (
        (left_guide_rod, left_slider_sleeve, "left"),
        (right_guide_rod, right_slider_sleeve, "right"),
    ):
        ctx.allow_overlap(
            base,
            hold_down_bar,
            elem_a=rod,
            elem_b=sleeve,
            reason=f"The {side} hold-down sleeve slides concentrically around its guide rod.",
        )
        ctx.allow_overlap(
            base,
            hold_down_bar,
            elem_a=rod,
            elem_b=clamp_bar,
            reason=f"The {side} guide rod passes through the hold-down crossbar end boss.",
        )
        ctx.allow_overlap(
            base,
            hold_down_bar,
            elem_a=rod,
            elem_b=clamp_pad,
            reason=f"The {side} guide rod continues through the underside pad carrier at the sleeve end.",
        )
    ctx.allow_overlap(
        base,
        hold_down_bar,
        elem_a=left_guide_base,
        elem_b=left_slider_sleeve,
        reason="At full clamp travel the left sleeve closes down onto its guide base as a hard stop.",
    )
    ctx.allow_overlap(
        base,
        hold_down_bar,
        elem_a=right_guide_base,
        elem_b=right_slider_sleeve,
        reason="At full clamp travel the right sleeve closes down onto its guide base as a hard stop.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base_aabb = ctx.part_world_aabb(base)
    bed_aabb = ctx.part_element_world_aabb(base, elem=bed_plate)
    fence_aabb = ctx.part_element_world_aabb(base, elem=rear_fence)
    handle_rest_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)
    clamp_rest_aabb = ctx.part_element_world_aabb(hold_down_bar, elem=clamp_bar)

    if base_aabb is not None:
        base_width = base_aabb[1][0] - base_aabb[0][0]
        base_depth = base_aabb[1][1] - base_aabb[0][1]
        base_height = base_aabb[1][2] - base_aabb[0][2]
        ctx.check(
            "base_width_realistic",
            0.44 <= base_width <= 0.52,
            f"Expected desktop cutter width near 0.48 m, got {base_width:.3f} m.",
        )
        ctx.check(
            "base_depth_realistic",
            0.30 <= base_depth <= 0.38,
            f"Expected desktop cutter depth near 0.34 m, got {base_depth:.3f} m.",
        )
        ctx.check(
            "base_height_realistic",
            0.05 <= base_height <= 0.11,
            f"Expected low desktop cutter height, got {base_height:.3f} m.",
        )

    blade_limits = blade_hinge.motion_limits
    hold_down_limits = hold_down_slide.motion_limits
    if blade_limits is not None and blade_limits.lower is not None and blade_limits.upper is not None:
        ctx.check(
            "blade_hinge_range_realistic",
            0.9 <= blade_limits.upper <= 1.5 and abs(blade_limits.lower) < 1e-9,
            f"Blade arm range should open roughly 50°-85°, got lower={blade_limits.lower}, upper={blade_limits.upper}.",
        )
    if hold_down_limits is not None and hold_down_limits.lower is not None and hold_down_limits.upper is not None:
        ctx.check(
            "hold_down_travel_realistic",
            0.008 <= hold_down_limits.upper <= 0.020 and abs(hold_down_limits.lower) < 1e-9,
            f"Hold-down bar travel should be short, got lower={hold_down_limits.lower}, upper={hold_down_limits.upper}.",
        )

    ctx.check(
        "blade_hinge_axis_horizontal",
        blade_hinge.axis == (0.0, -1.0, 0.0),
        f"Expected blade hinge axis (0, -1, 0), got {blade_hinge.axis}.",
    )
    ctx.check(
        "hold_down_axis_vertical",
        hold_down_slide.axis == (0.0, 0.0, -1.0),
        f"Expected hold-down axis (0, 0, -1), got {hold_down_slide.axis}.",
    )

    if bed_aabb is not None and fence_aabb is not None:
        fence_bottom_gap = fence_aabb[0][2] - bed_aabb[1][2]
        rear_offset = bed_aabb[1][1] - fence_aabb[1][1]
        ctx.check(
            "rear_fence_seated_on_bed",
            -1e-6 <= fence_bottom_gap <= 0.001,
            f"Rear fence should sit on the bed, got z gap {fence_bottom_gap:.6f} m.",
        )
        ctx.check(
            "rear_fence_on_back_edge",
            -1e-6 <= rear_offset <= 0.012,
            f"Rear fence should live on the back edge, got rear offset {rear_offset:.3f} m.",
        )

    with ctx.pose({blade_hinge: 0.0, hold_down_slide: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem=blade_strip,
            negative_elem=cut_line_strip,
            min_gap=0.0,
            max_gap=0.0025,
            name="blade_closed_near_cut_line",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            elem_a=blade_strip,
            elem_b=cut_line_strip,
            min_overlap=0.14,
            name="blade_aligned_with_cut_line",
        )
        ctx.expect_within(
            base,
            hold_down_bar,
            axes="xy",
            inner_elem=left_guide_rod,
            outer_elem=left_slider_sleeve,
            margin=0.004,
            name="left_slider_concentric_on_guide",
        )
        ctx.expect_within(
            base,
            hold_down_bar,
            axes="xy",
            inner_elem=right_guide_rod,
            outer_elem=right_slider_sleeve,
            margin=0.004,
            name="right_slider_concentric_on_guide",
        )
        ctx.expect_gap(
            hold_down_bar,
            base,
            axis="z",
            positive_elem=clamp_bar,
            negative_elem=bed_plate,
            min_gap=0.014,
            max_gap=0.018,
            name="hold_down_bar_raised_clearance",
        )
        ctx.expect_within(
            hold_down_bar,
            base,
            axes="xy",
            inner_elem=clamp_bar,
            outer_elem=bed_plate,
            margin=0.02,
            name="hold_down_bar_over_bed",
        )

    if handle_rest_aabb is not None and blade_limits is not None and blade_limits.upper is not None:
        with ctx.pose({blade_hinge: blade_limits.upper}):
            handle_open_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)
            ctx.fail_if_parts_overlap_in_current_pose(name="blade_arm_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="blade_arm_upper_no_floating")
            ctx.expect_gap(
                blade_arm,
                base,
                axis="z",
                positive_elem=blade_strip,
                negative_elem=bed_plate,
                min_gap=0.025,
                name="blade_arm_open_clear_of_bed",
            )
            if handle_open_aabb is not None:
                ctx.check(
                    "blade_arm_rotates_upward",
                    handle_open_aabb[0][2] > handle_rest_aabb[1][2] + 0.25,
                    (
                        "Expected the handle to rise substantially when the blade opens; "
                        f"rest top z={handle_rest_aabb[1][2]:.3f}, open bottom z={handle_open_aabb[0][2]:.3f}."
                    ),
                )

    if clamp_rest_aabb is not None and hold_down_limits is not None and hold_down_limits.upper is not None:
        with ctx.pose({hold_down_slide: hold_down_limits.upper}):
            clamp_low_aabb = ctx.part_element_world_aabb(hold_down_bar, elem=clamp_bar)
            ctx.fail_if_parts_overlap_in_current_pose(name="hold_down_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hold_down_upper_no_floating")
            ctx.expect_within(
                base,
                hold_down_bar,
                axes="xy",
                inner_elem=left_guide_rod,
                outer_elem=left_slider_sleeve,
                margin=0.004,
                name="left_slider_concentric_at_low_pose",
            )
            ctx.expect_within(
                base,
                hold_down_bar,
                axes="xy",
                inner_elem=right_guide_rod,
                outer_elem=right_slider_sleeve,
                margin=0.004,
                name="right_slider_concentric_at_low_pose",
            )
            ctx.expect_gap(
                hold_down_bar,
                base,
                axis="z",
                positive_elem=clamp_bar,
                negative_elem=bed_plate,
                min_gap=0.002,
                max_gap=0.0065,
                name="hold_down_bar_lowered_near_bed",
            )
            if clamp_low_aabb is not None:
                ctx.check(
                    "hold_down_bar_moves_downward",
                    clamp_low_aabb[0][2] < clamp_rest_aabb[0][2] - 0.009,
                    (
                        "Expected the hold-down bar to descend on its guides; "
                        f"rest bottom z={clamp_rest_aabb[0][2]:.3f}, lowered bottom z={clamp_low_aabb[0][2]:.3f}."
                    ),
                )

    if (
        blade_limits is not None
        and blade_limits.upper is not None
        and hold_down_limits is not None
        and hold_down_limits.upper is not None
    ):
        with ctx.pose({blade_hinge: blade_limits.upper, hold_down_slide: hold_down_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="all_articulations_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="all_articulations_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
