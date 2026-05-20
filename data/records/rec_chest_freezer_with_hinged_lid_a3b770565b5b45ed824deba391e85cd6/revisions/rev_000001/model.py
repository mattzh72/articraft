from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_chest_freezer")

    freezer_white = model.material("freezer_white", rgba=(0.95, 0.96, 0.97, 1.0))
    liner_white = model.material("liner_white", rgba=(0.91, 0.93, 0.95, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))

    body_width = 1.60
    body_depth = 0.76
    body_height = 0.82
    wall = 0.045
    base_height = 0.060
    floor_thickness = 0.055

    hinge_y = -0.385
    hinge_z = 0.828
    hinge_x_offset = 0.56

    lid_depth = 0.760
    lid_top_thickness = 0.022
    lid_side_height = 0.070

    stay_pivot_x = 0.720
    stay_pivot_y = -0.280
    stay_pivot_z = 0.470
    stay_length = 0.440
    stay_open_angle = 1.50

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=108.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    upper_shell_height = body_height - base_height
    shell_center_z = base_height + upper_shell_height * 0.5
    inner_width = body_width - 2.0 * wall
    inner_depth = body_depth - 2.0 * wall
    rear_wall_height = upper_shell_height - 0.050
    rear_wall_center_z = base_height + rear_wall_height * 0.5

    body.visual(
        Box((body_width, body_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=dark_plastic,
        name="base_plinth",
    )
    body.visual(
        Box((body_width, wall, upper_shell_height)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - wall * 0.5, shell_center_z)),
        material=freezer_white,
        name="front_wall",
    )
    body.visual(
        Box((body_width, wall, rear_wall_height)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall * 0.5, rear_wall_center_z)),
        material=freezer_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall, inner_depth, upper_shell_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, 0.0, shell_center_z)),
        material=freezer_white,
        name="left_wall",
    )
    body.visual(
        Box((wall, inner_depth, upper_shell_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, 0.0, shell_center_z)),
        material=freezer_white,
        name="right_wall",
    )
    body.visual(
        Box((inner_width, inner_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_height + floor_thickness * 0.5)),
        material=liner_white,
        name="cavity_floor",
    )
    body.visual(
        Box((0.30, 0.22, 0.24)),
        origin=Origin(xyz=(0.49, -0.11, base_height + 0.12)),
        material=liner_white,
        name="compressor_hump",
    )
    body.visual(
        Box((body_width - 0.12, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.350, body_height + 0.004)),
        material=gasket_gray,
        name="front_rim_gasket",
    )
    body.visual(
        Box((0.030, body_depth - 0.18, 0.008)),
        origin=Origin(xyz=(-0.748, -0.005, body_height + 0.004)),
        material=gasket_gray,
        name="left_rim_gasket",
    )
    body.visual(
        Box((0.030, body_depth - 0.18, 0.008)),
        origin=Origin(xyz=(0.748, -0.005, body_height + 0.004)),
        material=gasket_gray,
        name="right_rim_gasket",
    )
    body.visual(
        Box((0.48, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.375, 0.50)),
        material=charcoal,
        name="front_kick_panel",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        hinge_pin_x = side_sign * 0.660
        body.visual(
            Cylinder(radius=0.016, length=0.180),
            origin=Origin(xyz=(hinge_pin_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_gray,
            name=f"{side_name}_hinge_pin",
        )
        body.visual(
            Box((0.018, 0.032, 0.070)),
            origin=Origin(
                xyz=(hinge_pin_x + side_sign * 0.087, hinge_y, hinge_z - 0.0235)
            ),
            material=hinge_gray,
            name=f"{side_name}_hinge_side_support",
        )
        body.visual(
            Box((0.090, 0.025, 0.110)),
            origin=Origin(
                xyz=(side_sign * 0.752, stay_pivot_y - 0.0275, stay_pivot_z)
            ),
            material=hinge_gray,
            name=f"{side_name}_stay_bracket",
        )
        body.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(
                xyz=(side_sign * stay_pivot_x, stay_pivot_y, stay_pivot_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_gray,
            name=f"{side_name}_stay_pin",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((body_width + 0.02, lid_depth + 0.02, 0.085)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.375, 0.040)),
    )
    lid.visual(
        Box((body_width + 0.02, lid_depth + 0.02, lid_top_thickness)),
        origin=Origin(xyz=(0.0, 0.375, 0.056)),
        material=freezer_white,
        name="top_skin",
    )
    lid.visual(
        Box((body_width + 0.01, 0.035, lid_side_height)),
        origin=Origin(xyz=(0.0, 0.7425, 0.032)),
        material=freezer_white,
        name="front_skirt",
    )
    lid.visual(
        Box((0.035, 0.710, lid_side_height)),
        origin=Origin(xyz=(-0.7925, 0.375, 0.032)),
        material=freezer_white,
        name="left_skirt",
    )
    lid.visual(
        Box((0.035, 0.710, lid_side_height)),
        origin=Origin(xyz=(0.7925, 0.375, 0.032)),
        material=freezer_white,
        name="right_skirt",
    )
    lid.visual(
        Box((1.55, 0.70, 0.040)),
        origin=Origin(xyz=(0.0, 0.375, 0.031)),
        material=liner_white,
        name="insulation_core",
    )
    lid.visual(
        Box((1.47, 0.63, 0.012)),
        origin=Origin(xyz=(0.0, 0.375, 0.010)),
        material=liner_white,
        name="inner_liner",
    )
    lid.visual(
        Box((1.48, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.735, 0.005)),
        material=gasket_gray,
        name="front_gasket",
    )
    lid.visual(
        Box((0.46, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.755, 0.026)),
        material=dark_plastic,
        name="front_handle",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        hinge_x = side_sign * hinge_x_offset
        stay_stop_x = side_sign * 0.686
        lid.visual(
            Cylinder(radius=0.020, length=0.260),
            origin=Origin(xyz=(hinge_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_gray,
            name=f"{side_name}_hinge_sleeve",
        )
        lid.visual(
            Box((0.180, 0.010, 0.050)),
            origin=Origin(xyz=(hinge_x, 0.025, 0.025)),
            material=hinge_gray,
            name=f"{side_name}_hinge_lid_leaf",
        )
        lid.visual(
            Cylinder(radius=0.006, length=0.110),
            origin=Origin(xyz=(stay_stop_x, 0.136, -0.035)),
            material=liner_white,
            name=f"{side_name}_stay_stop_post",
        )
        lid.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=(stay_stop_x, 0.136, -0.090)),
            material=gasket_gray,
            name=f"{side_name}_stay_stop_bumper",
        )

    def add_stay_arm(name: str, side_sign: float) -> None:
        arm = model.part(name)
        x_shift = -side_sign * 0.034
        stay_bar_length = 0.414
        arm.inertial = Inertial.from_geometry(
            Box((0.070, stay_length, 0.040)),
            mass=2.2,
            origin=Origin(
                xyz=(
                    x_shift * 0.7,
                    0.205,
                    0.0,
                )
            ),
        )
        arm.visual(
            Cylinder(radius=0.017, length=0.060),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_gray,
            name="pivot_collar",
        )
        arm.visual(
            Box((0.026, stay_bar_length, 0.022)),
            origin=Origin(
                xyz=(
                    x_shift,
                    stay_bar_length * 0.5,
                    0.0,
                ),
            ),
            material=hinge_gray,
            name="stay_bar",
        )
        arm.visual(
            Sphere(radius=0.014),
            origin=Origin(
                xyz=(
                    x_shift,
                    0.426,
                    0.0,
                )
            ),
            material=charcoal,
            name="stay_head",
        )

    add_stay_arm("left_stay_arm", -1.0)
    add_stay_arm("right_stay_arm", 1.0)

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.18),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        model.articulation(
            f"{side_name}_stay_pivot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=f"{side_name}_stay_arm",
            origin=Origin(xyz=(side_sign * stay_pivot_x, stay_pivot_y, stay_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.2,
                lower=0.0,
                upper=stay_open_angle,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_stay_arm = object_model.get_part("left_stay_arm")
    right_stay_arm = object_model.get_part("right_stay_arm")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_stay_pivot = object_model.get_articulation("left_stay_pivot")
    right_stay_pivot = object_model.get_articulation("right_stay_pivot")

    body_front_rim_gasket = body.get_visual("front_rim_gasket")
    lid_front_gasket = lid.get_visual("front_gasket")
    lid_front_handle = lid.get_visual("front_handle")
    left_hinge_pin = body.get_visual("left_hinge_pin")
    right_hinge_pin = body.get_visual("right_hinge_pin")
    left_hinge_sleeve = lid.get_visual("left_hinge_sleeve")
    right_hinge_sleeve = lid.get_visual("right_hinge_sleeve")
    left_stay_pin = body.get_visual("left_stay_pin")
    right_stay_pin = body.get_visual("right_stay_pin")
    left_stay_collar = left_stay_arm.get_visual("pivot_collar")
    right_stay_collar = right_stay_arm.get_visual("pivot_collar")
    left_stay_head = left_stay_arm.get_visual("stay_head")
    right_stay_head = right_stay_arm.get_visual("stay_head")
    left_stay_stop = lid.get_visual("left_stay_stop_bumper")
    right_stay_stop = lid.get_visual("right_stay_stop_bumper")

    ctx.allow_overlap(
        body,
        lid,
        elem_a=left_hinge_pin,
        elem_b=left_hinge_sleeve,
        reason="Left barrel hinge sleeve intentionally nests over its hinge pin.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a=right_hinge_pin,
        elem_b=right_hinge_sleeve,
        reason="Right barrel hinge sleeve intentionally nests over its hinge pin.",
    )
    ctx.allow_overlap(
        body,
        left_stay_arm,
        elem_a=left_stay_pin,
        elem_b=left_stay_collar,
        reason="Left stay arm collar intentionally rotates around its side pivot pin.",
    )
    ctx.allow_overlap(
        body,
        right_stay_arm,
        elem_a=right_stay_pin,
        elem_b=right_stay_collar,
        reason="Right stay arm collar intentionally rotates around its side pivot pin.",
    )

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

    parts_present = all(
        part is not None for part in (body, lid, left_stay_arm, right_stay_arm)
    )
    ctx.check("freezer_parts_present", parts_present, "Body, lid, and both stay arms must exist.")

    ctx.check(
        "lid_hinge_axis_runs_along_width",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected lid hinge axis (1, 0, 0), got {lid_hinge.axis!r}.",
    )
    ctx.check(
        "stay_arm_axes_run_along_width",
        tuple(left_stay_pivot.axis) == (1.0, 0.0, 0.0)
        and tuple(right_stay_pivot.axis) == (1.0, 0.0, 0.0),
        "Both stay arms should pivot on side-mounted axes parallel to the freezer width.",
    )

    lid_limits = lid_hinge.motion_limits
    left_limits = left_stay_pivot.motion_limits
    right_limits = right_stay_pivot.motion_limits
    ctx.check(
        "motion_limits_match_freezer_hardware",
        lid_limits is not None
        and left_limits is not None
        and right_limits is not None
        and lid_limits.lower == 0.0
        and left_limits.lower == 0.0
        and right_limits.lower == 0.0
        and 1.0 <= lid_limits.upper <= 1.25
        and 1.40 <= left_limits.upper <= 1.55
        and 1.40 <= right_limits.upper <= 1.55,
        "The lid and stay arms should open through realistic bounded ranges.",
    )

    with ctx.pose({lid_hinge: 0.0, left_stay_pivot: 0.0, right_stay_pivot: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_front_gasket,
            negative_elem=body_front_rim_gasket,
            max_gap=0.003,
            max_penetration=0.0,
            name="lid_closes_on_front_rim_gasket",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            min_overlap=1.45,
            name="lid_spans_freezer_width",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            min_overlap=0.68,
            name="lid_spans_freezer_depth",
        )

    if lid_limits is not None and left_limits is not None and right_limits is not None:
        with ctx.pose(
            {
                lid_hinge: lid_limits.upper,
                left_stay_pivot: left_limits.upper,
                right_stay_pivot: right_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="freezer_open_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="freezer_open_pose_no_floating")
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem=lid_front_handle,
                negative_elem=body_front_rim_gasket,
                min_gap=0.28,
                name="front_handle_lifts_clear_when_open",
            )
            ctx.expect_overlap(
                left_stay_arm,
                lid,
                axes="x",
                elem_a=left_stay_head,
                elem_b=left_stay_stop,
                min_overlap=0.020,
                name="left_stay_lines_up_with_lid_stop",
            )
            ctx.expect_overlap(
                right_stay_arm,
                lid,
                axes="x",
                elem_a=right_stay_head,
                elem_b=right_stay_stop,
                min_overlap=0.020,
                name="right_stay_lines_up_with_lid_stop",
            )
            ctx.expect_contact(
                left_stay_arm,
                lid,
                elem_a=left_stay_head,
                elem_b=left_stay_stop,
                contact_tol=0.004,
                name="left_stay_supports_open_lid",
            )
            ctx.expect_contact(
                right_stay_arm,
                lid,
                elem_a=right_stay_head,
                elem_b=right_stay_stop,
                contact_tol=0.004,
                name="right_stay_supports_open_lid",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
