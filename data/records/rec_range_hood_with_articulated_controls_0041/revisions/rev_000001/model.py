from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_style_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    control_panel = model.material("control_panel", rgba=(0.15, 0.16, 0.17, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    marker = model.material("marker", rgba=(0.86, 0.87, 0.88, 1.0))

    body = model.part("hood_body")

    # Lower canopy shell: 900 mm wide, 500 mm deep, 180 mm tall, open underneath.
    _add_box(
        body,
        name="top_panel",
        size=(0.90, 0.50, 0.012),
        xyz=(0.0, 0.0, 0.174),
        material=stainless,
    )
    _add_box(
        body,
        name="left_side",
        size=(0.012, 0.50, 0.168),
        xyz=(-0.444, 0.0, 0.084),
        material=stainless,
    )
    _add_box(
        body,
        name="right_side",
        size=(0.012, 0.50, 0.168),
        xyz=(0.444, 0.0, 0.084),
        material=stainless,
    )
    _add_box(
        body,
        name="back_panel",
        size=(0.90, 0.012, 0.168),
        xyz=(0.0, -0.244, 0.084),
        material=stainless,
    )

    # Front fascia frame with a recessed control pocket.
    _add_box(
        body,
        name="front_left_strip",
        size=(0.28, 0.012, 0.168),
        xyz=(-0.31, 0.244, 0.084),
        material=stainless,
    )
    _add_box(
        body,
        name="front_right_strip",
        size=(0.28, 0.012, 0.168),
        xyz=(0.31, 0.244, 0.084),
        material=stainless,
    )
    _add_box(
        body,
        name="front_top_strip",
        size=(0.34, 0.012, 0.034),
        xyz=(0.0, 0.244, 0.151),
        material=stainless,
    )
    _add_box(
        body,
        name="front_bottom_strip",
        size=(0.34, 0.012, 0.034),
        xyz=(0.0, 0.244, 0.017),
        material=stainless,
    )
    _add_box(
        body,
        name="control_backplate",
        size=(0.34, 0.006, 0.10),
        xyz=(0.0, 0.225, 0.084),
        material=control_panel,
    )
    _add_box(
        body,
        name="control_pocket_left",
        size=(0.008, 0.010, 0.10),
        xyz=(-0.166, 0.233, 0.084),
        material=control_panel,
    )
    _add_box(
        body,
        name="control_pocket_right",
        size=(0.008, 0.010, 0.10),
        xyz=(0.166, 0.233, 0.084),
        material=control_panel,
    )
    _add_box(
        body,
        name="control_pocket_top",
        size=(0.34, 0.010, 0.006),
        xyz=(0.0, 0.233, 0.131),
        material=control_panel,
    )
    _add_box(
        body,
        name="control_pocket_bottom",
        size=(0.34, 0.010, 0.006),
        xyz=(0.0, 0.233, 0.037),
        material=control_panel,
    )

    # Three button guide channels inside the recessed control pocket.
    button_centers = {
        "left_button": (-0.055, 0.057),
        "center_button": (0.0, 0.057),
        "right_button": (0.055, 0.057),
    }
    for button_name, (button_x, button_z) in button_centers.items():
        _add_box(
            body,
            name=f"{button_name}_left_guide",
            size=(0.004, 0.022, 0.022),
            xyz=(button_x - 0.022, 0.239, button_z),
            material=filter_gray,
        )
        _add_box(
            body,
            name=f"{button_name}_right_guide",
            size=(0.004, 0.022, 0.022),
            xyz=(button_x + 0.022, 0.239, button_z),
            material=filter_gray,
        )
        _add_box(
            body,
            name=f"{button_name}_top_guide",
            size=(0.048, 0.022, 0.002),
            xyz=(button_x, 0.239, button_z + 0.010),
            material=filter_gray,
        )
        _add_box(
            body,
            name=f"{button_name}_bottom_guide",
            size=(0.048, 0.022, 0.002),
            xyz=(button_x, 0.239, button_z - 0.010),
            material=filter_gray,
        )

    # A shallow underside filter cassette keeps the hood from reading as a solid block
    # while remaining attached to the canopy shell.
    _add_box(
        body,
        name="underside_filter_cassette",
        size=(0.84, 0.42, 0.008),
        xyz=(0.0, 0.0, 0.004),
        material=filter_gray,
    )
    _add_box(
        body,
        name="underside_filter_left_rail",
        size=(0.018, 0.42, 0.008),
        xyz=(-0.429, 0.0, 0.004),
        material=filter_gray,
    )
    _add_box(
        body,
        name="underside_filter_right_rail",
        size=(0.018, 0.42, 0.008),
        xyz=(0.429, 0.0, 0.004),
        material=filter_gray,
    )

    # Rectangular chimney stack above the canopy, positioned toward the wall side.
    chimney_center_y = -0.12
    chimney_center_z = 0.538
    chimney_height = 0.724
    _add_box(
        body,
        name="chimney_left",
        size=(0.008, 0.26, chimney_height),
        xyz=(-0.156, chimney_center_y, chimney_center_z),
        material=stainless,
    )
    _add_box(
        body,
        name="chimney_right",
        size=(0.008, 0.26, chimney_height),
        xyz=(0.156, chimney_center_y, chimney_center_z),
        material=stainless,
    )
    _add_box(
        body,
        name="chimney_back",
        size=(0.32, 0.008, chimney_height),
        xyz=(0.0, -0.246, chimney_center_z),
        material=stainless,
    )
    _add_box(
        body,
        name="chimney_front",
        size=(0.32, 0.008, chimney_height),
        xyz=(0.0, 0.006, chimney_center_z),
        material=stainless,
    )

    knob_specs = (
        ("left_knob", -0.042),
        ("right_knob", 0.042),
    )
    for knob_name, knob_x in knob_specs:
        knob = model.part(knob_name)
        _add_cylinder(
            knob,
            name="knob_body",
            radius=0.024,
            length=0.028,
            xyz=(0.0, 0.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
            material=matte_black,
        )
        _add_box(
            knob,
            name="knob_marker",
            size=(0.004, 0.002, 0.014),
            xyz=(0.0, 0.015, 0.013),
            material=marker,
        )
        model.articulation(
            f"{knob_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, 0.242, 0.103)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    for button_name, (button_x, button_z) in button_centers.items():
        button = model.part(button_name)
        _add_box(
            button,
            name="button_cap",
            size=(0.040, 0.012, 0.018),
            xyz=(0.0, 0.0, 0.0),
            material=matte_black,
        )
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, 0.240, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.08,
                lower=0.0,
                upper=0.006,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("hood_body")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    left_button = object_model.get_part("left_button")
    center_button = object_model.get_part("center_button")
    right_button = object_model.get_part("right_button")

    left_knob_spin = object_model.get_articulation("left_knob_spin")
    right_knob_spin = object_model.get_articulation("right_knob_spin")
    left_button_press = object_model.get_articulation("left_button_press")
    center_button_press = object_model.get_articulation("center_button_press")
    right_button_press = object_model.get_articulation("right_button_press")

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

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("hood_body_aabb_present", "hood_body AABB was not available")
    else:
        body_min, body_max = body_aabb
        width = body_max[0] - body_min[0]
        depth = body_max[1] - body_min[1]
        height = body_max[2] - body_min[2]
        ctx.check(
            "range_hood_realistic_size",
            0.88 <= width <= 0.92 and 0.49 <= depth <= 0.51 and 0.89 <= height <= 0.91,
            f"width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
        )

    ctx.check(
        "control_joint_types_and_axes",
        left_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_knob_spin.axis == (0.0, 1.0, 0.0)
        and right_knob_spin.axis == (0.0, 1.0, 0.0)
        and left_button_press.articulation_type == ArticulationType.PRISMATIC
        and center_button_press.articulation_type == ArticulationType.PRISMATIC
        and right_button_press.articulation_type == ArticulationType.PRISMATIC
        and left_button_press.axis == (0.0, -1.0, 0.0)
        and center_button_press.axis == (0.0, -1.0, 0.0)
        and right_button_press.axis == (0.0, -1.0, 0.0),
        (
            f"knob axes=({left_knob_spin.axis}, {right_knob_spin.axis}), "
            f"button axes=({left_button_press.axis}, {center_button_press.axis}, {right_button_press.axis})"
        ),
    )

    ctx.expect_contact(
        left_knob,
        body,
        elem_a="knob_body",
        elem_b="control_backplate",
        name="left_knob_touching_control_panel",
    )
    ctx.expect_contact(
        right_knob,
        body,
        elem_a="knob_body",
        elem_b="control_backplate",
        name="right_knob_touching_control_panel",
    )
    ctx.expect_within(
        left_knob,
        body,
        axes="xz",
        outer_elem="control_backplate",
        margin=0.0,
        name="left_knob_within_control_panel",
    )
    ctx.expect_within(
        right_knob,
        body,
        axes="xz",
        outer_elem="control_backplate",
        margin=0.0,
        name="right_knob_within_control_panel",
    )

    for button_part in (left_button, center_button, right_button):
        ctx.expect_contact(
            button_part,
            body,
            name=f"{button_part.name}_guided_contact_at_rest",
        )
        ctx.expect_gap(
            button_part,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="control_backplate",
            min_gap=0.0055,
            max_gap=0.0065,
            name=f"{button_part.name}_rest_standoff_from_backplate",
        )
        ctx.expect_within(
            button_part,
            body,
            axes="xz",
            outer_elem="control_backplate",
            margin=0.0,
            name=f"{button_part.name}_within_control_panel",
        )

    ctx.expect_origin_distance(
        left_knob,
        right_knob,
        axes="x",
        min_dist=0.08,
        max_dist=0.09,
        name="knob_pair_spacing",
    )
    ctx.expect_origin_distance(
        left_knob,
        right_knob,
        axes="z",
        max_dist=0.001,
        name="knob_pair_level",
    )
    ctx.expect_origin_distance(
        left_button,
        center_button,
        axes="x",
        min_dist=0.05,
        max_dist=0.06,
        name="left_to_center_button_spacing",
    )
    ctx.expect_origin_distance(
        center_button,
        right_button,
        axes="x",
        min_dist=0.05,
        max_dist=0.06,
        name="center_to_right_button_spacing",
    )
    ctx.expect_origin_distance(
        left_button,
        center_button,
        axes="z",
        max_dist=0.001,
        name="button_row_level_left_center",
    )
    ctx.expect_origin_distance(
        center_button,
        right_button,
        axes="z",
        max_dist=0.001,
        name="button_row_level_center_right",
    )
    ctx.expect_origin_gap(
        left_knob,
        left_button,
        axis="z",
        min_gap=0.045,
        max_gap=0.055,
        name="buttons_below_knobs",
    )
    ctx.expect_origin_distance(
        center_button,
        body,
        axes="x",
        max_dist=0.001,
        name="center_button_is_centered",
    )

    with ctx.pose({left_knob_spin: pi / 2.0, right_knob_spin: pi}):
        ctx.expect_contact(
            left_knob,
            body,
            elem_a="knob_body",
            elem_b="control_backplate",
            name="left_knob_contact_when_rotated",
        )
        ctx.expect_contact(
            right_knob,
            body,
            elem_a="knob_body",
            elem_b="control_backplate",
            name="right_knob_contact_when_rotated",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")

    button_joints = (left_button_press, center_button_press, right_button_press)
    for button_joint in button_joints:
        limits = button_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_lower_no_floating")
            with ctx.pose({button_joint: limits.upper}):
                button_part = object_model.get_part(button_joint.child)
                ctx.expect_contact(
                    button_part,
                    body,
                    name=f"{button_joint.name}_pressed_contact",
                )
                ctx.expect_gap(
                    button_part,
                    body,
                    axis="y",
                    positive_elem="button_cap",
                    negative_elem="control_backplate",
                    max_gap=0.0005,
                    max_penetration=1e-6,
                    name=f"{button_joint.name}_pressed_against_backplate",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_upper_no_floating")

    with ctx.pose(
        {
            left_button_press: left_button_press.motion_limits.upper,  # type: ignore[union-attr]
            center_button_press: center_button_press.motion_limits.upper,  # type: ignore[union-attr]
            right_button_press: right_button_press.motion_limits.upper,  # type: ignore[union-attr]
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="all_buttons_pressed_no_floating")

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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
