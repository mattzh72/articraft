from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _rounded_xz_section(
    width: float,
    height: float,
    radius: float,
    y_pos: float,
) -> list[tuple[float, float, float]]:
    return [(x, y_pos, z) for x, z in rounded_rect_profile(width, height, radius)]


def _build_curved_front_strip_mesh():
    return section_loft(
        [
            _rounded_xz_section(0.46, 0.024, 0.006, 0.000),
            _rounded_xz_section(0.44, 0.028, 0.007, 0.008),
            _rounded_xz_section(0.42, 0.024, 0.006, 0.016),
        ]
    )


def _build_guide_ring_mesh(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    *,
    filename: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, depth)],
            [(inner_radius, 0.0), (inner_radius, depth)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path(filename),
    )


def _add_y_axis_cylinder(
    part,
    radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.83, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.52
    canopy_height = 0.10
    shell_thickness = 0.010

    hood_canopy = model.part("hood_canopy")
    hood_canopy.visual(
        Box((canopy_width, canopy_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_thickness * 0.5)),
        material=stainless,
        name="top_panel",
    )
    hood_canopy.visual(
        Box((shell_thickness, canopy_depth, canopy_height)),
        origin=Origin(
            xyz=(-canopy_width * 0.5 + shell_thickness * 0.5, 0.0, canopy_height * 0.5)
        ),
        material=stainless,
        name="left_side_panel",
    )
    hood_canopy.visual(
        Box((shell_thickness, canopy_depth, canopy_height)),
        origin=Origin(
            xyz=(canopy_width * 0.5 - shell_thickness * 0.5, 0.0, canopy_height * 0.5)
        ),
        material=stainless,
        name="right_side_panel",
    )
    hood_canopy.visual(
        Box((canopy_width - 2.0 * shell_thickness, shell_thickness, canopy_height)),
        origin=Origin(
            xyz=(
                0.0,
                -canopy_depth * 0.5 + shell_thickness * 0.5,
                canopy_height * 0.5,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    hood_canopy.visual(
        Box((canopy_width - 2.0 * shell_thickness, shell_thickness, canopy_height)),
        origin=Origin(
            xyz=(
                0.0,
                canopy_depth * 0.5 - shell_thickness * 0.5,
                canopy_height * 0.5,
            )
        ),
        material=stainless,
        name="front_panel",
    )
    hood_canopy.visual(
        Box((0.34, 0.18, 0.006)),
        origin=Origin(xyz=(0.0, -0.15, canopy_height - 0.013)),
        material=brushed_steel,
        name="chimney_mount_pad",
    )
    hood_canopy.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, canopy_height * 0.5)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_width = 0.30
    chimney_depth = 0.24
    chimney_height = 0.80
    chimney_shell = 0.008
    chimney_cover.visual(
        Box((chimney_width, chimney_shell, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_depth * 0.5 - chimney_shell * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_front",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_shell, chimney_height)),
        origin=Origin(xyz=(0.0, -chimney_depth * 0.5 + chimney_shell * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_back",
    )
    chimney_cover.visual(
        Box((chimney_shell, chimney_depth - 2.0 * chimney_shell, chimney_height)),
        origin=Origin(xyz=(-chimney_width * 0.5 + chimney_shell * 0.5, 0.0, chimney_height * 0.5)),
        material=stainless,
        name="chimney_left",
    )
    chimney_cover.visual(
        Box((chimney_shell, chimney_depth - 2.0 * chimney_shell, chimney_height)),
        origin=Origin(xyz=(chimney_width * 0.5 - chimney_shell * 0.5, 0.0, chimney_height * 0.5)),
        material=stainless,
        name="chimney_right",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_depth, chimney_shell)),
        origin=Origin(xyz=(0.0, 0.0, chimney_height - chimney_shell * 0.5)),
        material=stainless,
        name="chimney_cap",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, chimney_height * 0.5)),
    )

    front_strip = model.part("front_strip")
    front_strip.visual(
        mesh_from_geometry(
            _build_curved_front_strip_mesh(),
            ASSETS.mesh_path("range_hood_curved_front_strip.obj"),
        ),
        material=brushed_steel,
        name="curved_strip_shell",
    )
    front_strip.inertial = Inertial.from_geometry(
        Box((0.46, 0.016, 0.028)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
    )

    left_control_panel = model.part("left_control_panel")
    left_control_panel.visual(
        Box((0.19, 0.004, 0.074)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=brushed_steel,
        name="knob_panel_plate",
    )
    left_control_panel.visual(
        Box((0.070, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=charcoal,
        name="knob_cluster_bridge",
    )
    left_control_panel.inertial = Inertial.from_geometry(
        Box((0.19, 0.004, 0.074)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
    )

    right_button_panel = model.part("right_button_panel")
    right_button_panel.visual(
        Box((0.090, 0.004, 0.086)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=brushed_steel,
        name="button_panel_plate",
    )
    right_button_panel.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=charcoal,
        name="button_panel_spine",
    )
    right_button_panel.inertial = Inertial.from_geometry(
        Box((0.090, 0.004, 0.086)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
    )

    model.articulation(
        "canopy_to_chimney_cover",
        ArticulationType.FIXED,
        parent=hood_canopy,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, -0.14, canopy_height)),
    )
    model.articulation(
        "canopy_to_front_strip",
        ArticulationType.FIXED,
        parent=hood_canopy,
        child=front_strip,
        origin=Origin(xyz=(0.0, canopy_depth * 0.5, 0.082)),
    )
    model.articulation(
        "canopy_to_left_control_panel",
        ArticulationType.FIXED,
        parent=hood_canopy,
        child=left_control_panel,
        origin=Origin(xyz=(-0.275, canopy_depth * 0.5, 0.028)),
    )
    model.articulation(
        "canopy_to_right_button_panel",
        ArticulationType.FIXED,
        parent=hood_canopy,
        child=right_button_panel,
        origin=Origin(xyz=(0.290, canopy_depth * 0.5, 0.028)),
    )

    for knob_name, knob_x, knob_z in (
        ("knob_top", 0.0, 0.018),
        ("knob_lower_left", -0.028, -0.018),
        ("knob_lower_right", 0.028, -0.018),
    ):
        knob_part = model.part(knob_name)
        _add_y_axis_cylinder(
            knob_part,
            radius=0.0085,
            length=0.018,
            center=(0.0, 0.009, 0.0),
            material=soft_black,
            name="knob_shaft",
        )
        _add_y_axis_cylinder(
            knob_part,
            radius=0.014,
            length=0.010,
            center=(0.0, 0.023, 0.0),
            material=graphite,
            name="knob_body",
        )
        _add_y_axis_cylinder(
            knob_part,
            radius=0.015,
            length=0.004,
            center=(0.0, 0.030, 0.0),
            material=graphite,
            name="knob_face",
        )
        knob_part.visual(
            Box((0.003, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, 0.031, 0.010)),
            material=stainless,
            name="knob_indicator",
        )
        knob_part.inertial = Inertial.from_geometry(
            Box((0.030, 0.036, 0.030)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.014, 0.0)),
        )
        model.articulation(
            f"left_panel_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=left_control_panel,
            child=knob_part,
            origin=Origin(xyz=(knob_x, 0.004, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=6.0),
        )

    for button_name, button_z in (("button_upper", 0.022), ("button_lower", -0.022)):
        button_part = model.part(button_name)
        _add_y_axis_cylinder(
            button_part,
            radius=0.010,
            length=0.018,
            center=(0.0, 0.009, 0.0),
            material=soft_black,
            name="button_stem",
        )
        _add_y_axis_cylinder(
            button_part,
            radius=0.0135,
            length=0.006,
            center=(0.0, 0.021, 0.0),
            material=graphite,
            name="button_cap",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((0.027, 0.028, 0.027)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.012, 0.0)),
        )
        model.articulation(
            f"right_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=right_button_panel,
            child=button_part,
            origin=Origin(xyz=(0.0, 0.004, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.04,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_canopy = object_model.get_part("hood_canopy")
    chimney_cover = object_model.get_part("chimney_cover")
    front_strip = object_model.get_part("front_strip")
    left_control_panel = object_model.get_part("left_control_panel")
    right_button_panel = object_model.get_part("right_button_panel")
    knob_top = object_model.get_part("knob_top")
    knob_lower_left = object_model.get_part("knob_lower_left")
    knob_lower_right = object_model.get_part("knob_lower_right")
    button_upper = object_model.get_part("button_upper")
    button_lower = object_model.get_part("button_lower")

    knob_top_joint = object_model.get_articulation("left_panel_to_knob_top")
    knob_lower_left_joint = object_model.get_articulation("left_panel_to_knob_lower_left")
    knob_lower_right_joint = object_model.get_articulation("left_panel_to_knob_lower_right")
    button_upper_joint = object_model.get_articulation("right_panel_to_button_upper")
    button_lower_joint = object_model.get_articulation("right_panel_to_button_lower")

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
    ctx.allow_overlap(
        button_upper,
        right_button_panel,
        reason="Upper push-button plunger retracts into the faceplate guide bore.",
        elem_a="button_stem",
    )
    ctx.allow_overlap(
        button_lower,
        right_button_panel,
        reason="Lower push-button plunger retracts into the faceplate guide bore.",
        elem_a="button_stem",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(chimney_cover, hood_canopy)
    ctx.expect_contact(front_strip, hood_canopy)
    ctx.expect_contact(left_control_panel, hood_canopy)
    ctx.expect_contact(right_button_panel, hood_canopy)
    ctx.expect_overlap(front_strip, hood_canopy, axes="x", min_overlap=0.20)

    for knob in (knob_top, knob_lower_left, knob_lower_right):
        ctx.expect_contact(knob, left_control_panel, name=f"{knob.name}_mounted")
    for button in (button_upper, button_lower):
        ctx.expect_contact(button, right_button_panel, name=f"{button.name}_mounted")

    canopy_pos = ctx.part_world_position(hood_canopy)
    chimney_pos = ctx.part_world_position(chimney_cover)
    knob_top_pos = ctx.part_world_position(knob_top)
    knob_lower_left_pos = ctx.part_world_position(knob_lower_left)
    knob_lower_right_pos = ctx.part_world_position(knob_lower_right)
    button_upper_pos = ctx.part_world_position(button_upper)
    button_lower_pos = ctx.part_world_position(button_lower)

    assert canopy_pos is not None
    assert chimney_pos is not None
    assert knob_top_pos is not None
    assert knob_lower_left_pos is not None
    assert knob_lower_right_pos is not None
    assert button_upper_pos is not None
    assert button_lower_pos is not None

    ctx.check(
        "chimney_is_centered_over_canopy",
        abs(chimney_pos[0] - canopy_pos[0]) < 0.005 and chimney_pos[2] > canopy_pos[2] + 0.09,
        details=f"canopy={canopy_pos}, chimney={chimney_pos}",
    )
    ctx.check(
        "left_knobs_form_triangular_cluster",
        (
            knob_top_pos[2] > knob_lower_left_pos[2] + 0.025
            and knob_top_pos[2] > knob_lower_right_pos[2] + 0.025
            and knob_lower_left_pos[0] < knob_top_pos[0] < knob_lower_right_pos[0]
        ),
        details=(
            f"top={knob_top_pos}, lower_left={knob_lower_left_pos}, "
            f"lower_right={knob_lower_right_pos}"
        ),
    )
    ctx.check(
        "right_buttons_stack_vertically",
        abs(button_upper_pos[0] - button_lower_pos[0]) < 0.002
        and button_upper_pos[2] > button_lower_pos[2] + 0.035,
        details=f"upper={button_upper_pos}, lower={button_lower_pos}",
    )
    ctx.check(
        "knob_joints_are_continuous_about_front_axis",
        knob_top_joint.joint_type == ArticulationType.CONTINUOUS
        and knob_lower_left_joint.joint_type == ArticulationType.CONTINUOUS
        and knob_lower_right_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(knob_top_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(knob_lower_left_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(knob_lower_right_joint.axis) == (0.0, 1.0, 0.0),
        details=(
            f"axes={[knob_top_joint.axis, knob_lower_left_joint.axis, knob_lower_right_joint.axis]}, "
            f"types={[knob_top_joint.joint_type, knob_lower_left_joint.joint_type, knob_lower_right_joint.joint_type]}"
        ),
    )
    ctx.check(
        "button_joints_slide_inward",
        button_upper_joint.joint_type == ArticulationType.PRISMATIC
        and button_lower_joint.joint_type == ArticulationType.PRISMATIC
        and tuple(button_upper_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(button_lower_joint.axis) == (0.0, -1.0, 0.0),
        details=(
            f"axes={[button_upper_joint.axis, button_lower_joint.axis]}, "
            f"types={[button_upper_joint.joint_type, button_lower_joint.joint_type]}"
        ),
    )

    with ctx.pose(
        {
            knob_top_joint: 1.1,
            knob_lower_left_joint: -1.8,
            knob_lower_right_joint: 2.2,
        }
    ):
        ctx.expect_contact(knob_top, left_control_panel, name="knob_top_rotated_contact")
        ctx.expect_contact(
            knob_lower_left,
            left_control_panel,
            name="knob_lower_left_rotated_contact",
        )
        ctx.expect_contact(
            knob_lower_right,
            left_control_panel,
            name="knob_lower_right_rotated_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")

    upper_button_rest = ctx.part_world_position(button_upper)
    lower_button_rest = ctx.part_world_position(button_lower)
    assert upper_button_rest is not None
    assert lower_button_rest is not None

    button_upper_limits = button_upper_joint.motion_limits
    button_lower_limits = button_lower_joint.motion_limits
    assert button_upper_limits is not None
    assert button_lower_limits is not None
    assert button_upper_limits.upper is not None
    assert button_lower_limits.upper is not None

    with ctx.pose(
        {
            button_upper_joint: button_upper_limits.upper,
            button_lower_joint: button_lower_limits.upper,
        }
    ):
        upper_button_pressed = ctx.part_world_position(button_upper)
        lower_button_pressed = ctx.part_world_position(button_lower)
        assert upper_button_pressed is not None
        assert lower_button_pressed is not None
        ctx.check(
            "buttons_move_inward_when_pressed",
            upper_button_pressed[1] < upper_button_rest[1] - 0.003
            and lower_button_pressed[1] < lower_button_rest[1] - 0.003,
            details=(
                f"upper_rest={upper_button_rest}, upper_pressed={upper_button_pressed}, "
                f"lower_rest={lower_button_rest}, lower_pressed={lower_button_pressed}"
            ),
        )
        ctx.expect_contact(button_upper, right_button_panel, name="button_upper_pressed_contact")
        ctx.expect_contact(button_lower, right_button_panel, name="button_lower_pressed_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="pressed_buttons_no_overlap")
        ctx.fail_if_isolated_parts(name="pressed_buttons_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
