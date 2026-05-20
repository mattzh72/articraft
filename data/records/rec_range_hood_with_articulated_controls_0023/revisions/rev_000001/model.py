from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless_steel", rgba=(0.80, 0.81, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.78, 0.18, 0.14, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.16
    shell_thickness = 0.006

    chimney_width = 0.30
    chimney_depth = 0.26
    chimney_height = 0.72
    chimney_thickness = 0.005
    chimney_center_y = -0.11

    knob_radius = 0.028
    knob_depth = 0.020
    knob_z = 0.085
    knob_x = 0.072

    bezel_width = 0.036
    bezel_height = 0.026
    bezel_depth = 0.006
    bezel_bar_x = 0.004
    bezel_bar_z = 0.004
    button_width = 0.028
    button_height = 0.018
    button_depth = 0.004
    button_travel = 0.002
    upper_button_z = 0.123
    lower_button_z = 0.047

    def front_face_origin(x_pos: float, z_pos: float) -> Origin:
        return Origin(xyz=(x_pos, canopy_depth / 2.0, z_pos))

    def top_face_origin(x_pos: float, y_pos: float) -> Origin:
        return Origin(xyz=(x_pos, y_pos, canopy_height))

    canopy = model.part("canopy")
    canopy.visual(
        Box((canopy_width, canopy_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_thickness / 2.0)),
        material=stainless,
        name="top_panel",
    )
    canopy.visual(
        Box((canopy_width, shell_thickness, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                canopy_depth / 2.0 - shell_thickness / 2.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="front_panel",
    )
    canopy.visual(
        Box((canopy_width, shell_thickness, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -canopy_depth / 2.0 + shell_thickness / 2.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="rear_panel",
    )
    canopy.visual(
        Box((shell_thickness, canopy_depth - 2.0 * shell_thickness, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                canopy_width / 2.0 - shell_thickness / 2.0,
                0.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="right_panel",
    )
    canopy.visual(
        Box((shell_thickness, canopy_depth - 2.0 * shell_thickness, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                -canopy_width / 2.0 + shell_thickness / 2.0,
                0.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="left_panel",
    )
    canopy.visual(
        Box((0.888, 0.34, 0.010)),
        origin=Origin(xyz=(0.0, -0.01, 0.015)),
        material=dark_trim,
        name="grease_filter",
    )
    canopy.visual(
        Box((0.16, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, canopy_depth / 2.0 - 0.010, canopy_height - 0.050)),
        material=dark_trim,
        name="control_backing_strip",
    )

    chimney = model.part("chimney")
    chimney.visual(
        Box((chimney_width, chimney_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_depth / 2.0 - chimney_thickness / 2.0,
                chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="front_wall",
    )
    chimney.visual(
        Box((chimney_width, chimney_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                -chimney_depth / 2.0 + chimney_thickness / 2.0,
                chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="rear_wall",
    )
    chimney.visual(
        Box((chimney_thickness, chimney_depth - 2.0 * chimney_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width / 2.0 - chimney_thickness / 2.0,
                0.0,
                chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="right_wall",
    )
    chimney.visual(
        Box((chimney_thickness, chimney_depth - 2.0 * chimney_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width / 2.0 + chimney_thickness / 2.0,
                0.0,
                chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="left_wall",
    )

    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=top_face_origin(0.0, chimney_center_y),
    )

    upper_bezel = model.part("upper_button_bezel")
    upper_bezel.visual(
        Box((bezel_width, bezel_depth, bezel_bar_z)),
        origin=Origin(xyz=(0.0, bezel_depth / 2.0, bezel_height / 2.0 - bezel_bar_z / 2.0)),
        material=dark_trim,
        name="top_bar",
    )
    upper_bezel.visual(
        Box((bezel_width, bezel_depth, bezel_bar_z)),
        origin=Origin(xyz=(0.0, bezel_depth / 2.0, -bezel_height / 2.0 + bezel_bar_z / 2.0)),
        material=dark_trim,
        name="bottom_bar",
    )
    upper_bezel.visual(
        Box((bezel_bar_x, bezel_depth, bezel_height - 2.0 * bezel_bar_z)),
        origin=Origin(xyz=(bezel_width / 2.0 - bezel_bar_x / 2.0, bezel_depth / 2.0, 0.0)),
        material=dark_trim,
        name="right_bar",
    )
    upper_bezel.visual(
        Box((bezel_bar_x, bezel_depth, bezel_height - 2.0 * bezel_bar_z)),
        origin=Origin(xyz=(-bezel_width / 2.0 + bezel_bar_x / 2.0, bezel_depth / 2.0, 0.0)),
        material=dark_trim,
        name="left_bar",
    )

    lower_bezel = model.part("lower_button_bezel")
    lower_bezel.visual(
        Box((bezel_width, bezel_depth, bezel_bar_z)),
        origin=Origin(xyz=(0.0, bezel_depth / 2.0, bezel_height / 2.0 - bezel_bar_z / 2.0)),
        material=dark_trim,
        name="top_bar",
    )
    lower_bezel.visual(
        Box((bezel_width, bezel_depth, bezel_bar_z)),
        origin=Origin(xyz=(0.0, bezel_depth / 2.0, -bezel_height / 2.0 + bezel_bar_z / 2.0)),
        material=dark_trim,
        name="bottom_bar",
    )
    lower_bezel.visual(
        Box((bezel_bar_x, bezel_depth, bezel_height - 2.0 * bezel_bar_z)),
        origin=Origin(xyz=(bezel_width / 2.0 - bezel_bar_x / 2.0, bezel_depth / 2.0, 0.0)),
        material=dark_trim,
        name="right_bar",
    )
    lower_bezel.visual(
        Box((bezel_bar_x, bezel_depth, bezel_height - 2.0 * bezel_bar_z)),
        origin=Origin(xyz=(-bezel_width / 2.0 + bezel_bar_x / 2.0, bezel_depth / 2.0, 0.0)),
        material=dark_trim,
        name="left_bar",
    )

    model.articulation(
        "canopy_to_upper_button_bezel",
        ArticulationType.FIXED,
        parent=canopy,
        child=upper_bezel,
        origin=front_face_origin(0.0, upper_button_z),
    )
    model.articulation(
        "canopy_to_lower_button_bezel",
        ArticulationType.FIXED,
        parent=canopy,
        child=lower_bezel,
        origin=front_face_origin(0.0, lower_button_z),
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(xyz=(0.0, knob_depth / 2.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    left_knob.visual(
        Box((0.005, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, knob_depth - 0.002, knob_radius - 0.010)),
        material=indicator_red,
        name="indicator",
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(xyz=(0.0, knob_depth / 2.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    right_knob.visual(
        Box((0.005, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, knob_depth - 0.002, knob_radius - 0.010)),
        material=indicator_red,
        name="indicator",
    )

    model.articulation(
        "left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=left_knob,
        origin=front_face_origin(-knob_x, knob_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    model.articulation(
        "right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=right_knob,
        origin=front_face_origin(knob_x, knob_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    upper_button = model.part("upper_button")
    upper_button.visual(
        Box((button_width, button_depth, button_height)),
        origin=Origin(xyz=(0.0, button_depth / 2.0 + button_travel, 0.0)),
        material=dark_trim,
        name="button_cap",
    )

    lower_button = model.part("lower_button")
    lower_button.visual(
        Box((button_width, button_depth, button_height)),
        origin=Origin(xyz=(0.0, button_depth / 2.0 + button_travel, 0.0)),
        material=dark_trim,
        name="button_cap",
    )

    model.articulation(
        "upper_button_press",
        ArticulationType.PRISMATIC,
        parent=upper_bezel,
        child=upper_button,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=button_travel),
    )
    model.articulation(
        "lower_button_press",
        ArticulationType.PRISMATIC,
        parent=lower_bezel,
        child=lower_button,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=button_travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    button_travel = 0.002
    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney")
    upper_bezel = object_model.get_part("upper_button_bezel")
    lower_bezel = object_model.get_part("lower_button_bezel")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")

    left_knob_spin = object_model.get_articulation("left_knob_spin")
    right_knob_spin = object_model.get_articulation("right_knob_spin")
    upper_button_press = object_model.get_articulation("upper_button_press")
    lower_button_press = object_model.get_articulation("lower_button_press")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    ctx.expect_contact(chimney, canopy, name="chimney_seated_on_canopy")
    ctx.expect_within(chimney, canopy, axes="xy", margin=0.0, name="chimney_within_canopy_footprint")

    ctx.expect_contact(left_knob, canopy, name="left_knob_touches_canopy")
    ctx.expect_contact(right_knob, canopy, name="right_knob_touches_canopy")
    ctx.expect_gap(left_knob, canopy, axis="y", max_gap=0.001, max_penetration=0.0, name="left_knob_front_mount_gap")
    ctx.expect_gap(right_knob, canopy, axis="y", max_gap=0.001, max_penetration=0.0, name="right_knob_front_mount_gap")

    ctx.expect_contact(upper_bezel, canopy, name="upper_bezel_contacts_canopy")
    ctx.expect_contact(lower_bezel, canopy, name="lower_bezel_contacts_canopy")
    ctx.expect_within(upper_button, upper_bezel, axes="xz", margin=0.001, name="upper_button_within_bezel")
    ctx.expect_within(lower_button, lower_bezel, axes="xz", margin=0.001, name="lower_button_within_bezel")

    ctx.expect_origin_distance(left_knob, right_knob, axes="x", min_dist=0.13, max_dist=0.15, name="knob_spacing")
    ctx.expect_origin_distance(upper_button, lower_button, axes="x", min_dist=0.0, max_dist=0.001, name="button_centerline_alignment")
    ctx.expect_origin_distance(upper_button, lower_button, axes="z", min_dist=0.07, max_dist=0.08, name="button_vertical_spacing")

    with ctx.pose({left_knob_spin: 1.5707963267948966, right_knob_spin: -1.0471975511965976}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")
        ctx.expect_contact(left_knob, canopy, name="left_knob_rotated_contact")
        ctx.expect_contact(right_knob, canopy, name="right_knob_rotated_contact")

    upper_limits = upper_button_press.motion_limits
    if upper_limits is not None and upper_limits.lower is not None and upper_limits.upper is not None:
        with ctx.pose({upper_button_press: upper_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="upper_button_rest_no_overlap")
            ctx.fail_if_isolated_parts(name="upper_button_rest_no_floating")
            ctx.expect_gap(
                upper_button,
                canopy,
                axis="y",
                min_gap=button_travel,
                max_gap=button_travel + 0.0005,
                name="upper_button_rest_proud",
            )
            ctx.expect_within(
                upper_button,
                upper_bezel,
                axes="xz",
                margin=0.001,
                name="upper_button_rest_within_bezel",
            )
        with ctx.pose({upper_button_press: upper_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="upper_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="upper_button_pressed_no_floating")
            ctx.expect_gap(
                upper_button,
                canopy,
                axis="y",
                min_gap=0.0,
                max_gap=0.0005,
                name="upper_button_pressed_flush",
            )
            ctx.expect_within(
                upper_button,
                upper_bezel,
                axes="xz",
                margin=0.001,
                name="upper_button_pressed_within_bezel",
            )

    lower_limits = lower_button_press.motion_limits
    if lower_limits is not None and lower_limits.lower is not None and lower_limits.upper is not None:
        with ctx.pose({lower_button_press: lower_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lower_button_rest_no_overlap")
            ctx.fail_if_isolated_parts(name="lower_button_rest_no_floating")
            ctx.expect_gap(
                lower_button,
                canopy,
                axis="y",
                min_gap=button_travel,
                max_gap=button_travel + 0.0005,
                name="lower_button_rest_proud",
            )
            ctx.expect_within(
                lower_button,
                lower_bezel,
                axes="xz",
                margin=0.001,
                name="lower_button_rest_within_bezel",
            )
        with ctx.pose({lower_button_press: lower_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lower_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="lower_button_pressed_no_floating")
            ctx.expect_gap(
                lower_button,
                canopy,
                axis="y",
                min_gap=0.0,
                max_gap=0.0005,
                name="lower_button_pressed_flush",
            )
            ctx.expect_within(
                lower_button,
                lower_bezel,
                axes="xz",
                margin=0.001,
                name="lower_button_pressed_within_bezel",
            )

    canopy_aabb = ctx.part_world_aabb(canopy)
    chimney_aabb = ctx.part_world_aabb(chimney)
    if canopy_aabb is not None and chimney_aabb is not None:
        canopy_width = canopy_aabb[1][0] - canopy_aabb[0][0]
        canopy_depth = canopy_aabb[1][1] - canopy_aabb[0][1]
        overall_height = chimney_aabb[1][2]
        ctx.check(
            "canopy_width_realistic",
            0.85 <= canopy_width <= 0.95,
            details=f"measured canopy width {canopy_width:.3f} m",
        )
        ctx.check(
            "canopy_depth_realistic",
            0.46 <= canopy_depth <= 0.52,
            details=f"measured canopy depth {canopy_depth:.3f} m",
        )
        ctx.check(
            "overall_height_realistic",
            0.85 <= overall_height <= 0.95,
            details=f"measured hood height {overall_height:.3f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
