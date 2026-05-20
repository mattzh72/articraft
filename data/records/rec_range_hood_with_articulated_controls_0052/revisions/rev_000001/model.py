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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    enamel = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    fascia = model.material("fascia_beige", rgba=(0.85, 0.84, 0.80, 1.0))
    charcoal = model.material("charcoal_control", rgba=(0.15, 0.15, 0.16, 1.0))
    marker = model.material("control_marker", rgba=(0.78, 0.23, 0.18, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.75, 0.77, 0.79, 1.0))
    lens_white = model.material("lens_white", rgba=(0.94, 0.94, 0.92, 0.9))

    hood_width = 0.76
    hood_depth = 0.50
    hood_height = 0.156
    shell_thickness = 0.012
    strip_height = 0.078
    lower_lip_height = 0.020
    side_height = hood_height - shell_thickness
    front_y = -hood_depth * 0.5 + shell_thickness * 0.5
    top_z = hood_height - shell_thickness * 0.5
    strip_center_z = hood_height - shell_thickness - strip_height * 0.5
    control_z = strip_center_z

    knob_shaft_radius = 0.0135
    button_shaft_radius = 0.0075
    strip_inner_width = hood_width - 2.0 * shell_thickness
    knob_aperture = knob_shaft_radius * 2.0
    button_aperture = button_shaft_radius * 2.0
    middle_band_height = knob_aperture
    strip_bar_height = (strip_height - middle_band_height) * 0.5
    button_x_positions = {
        "left_outer": -0.125,
        "left_inner": -0.075,
        "right_inner": 0.075,
        "right_outer": 0.125,
    }

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((hood_width, hood_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=enamel,
        name="top_panel",
    )
    hood_body.visual(
        Box((shell_thickness, hood_depth, side_height)),
        origin=Origin(xyz=(-hood_width * 0.5 + shell_thickness * 0.5, 0.0, side_height * 0.5)),
        material=enamel,
        name="left_side",
    )
    hood_body.visual(
        Box((shell_thickness, hood_depth, side_height)),
        origin=Origin(xyz=(hood_width * 0.5 - shell_thickness * 0.5, 0.0, side_height * 0.5)),
        material=enamel,
        name="right_side",
    )
    hood_body.visual(
        Box((hood_width - 2.0 * shell_thickness, shell_thickness, side_height)),
        origin=Origin(xyz=(0.0, hood_depth * 0.5 - shell_thickness * 0.5, side_height * 0.5)),
        material=enamel,
        name="back_panel",
    )
    hood_body.visual(
        Box((strip_inner_width, shell_thickness, lower_lip_height)),
        origin=Origin(xyz=(0.0, front_y, lower_lip_height * 0.5)),
        material=enamel,
        name="front_lower_lip",
    )
    hood_body.visual(
        Box((strip_inner_width, shell_thickness, strip_bar_height)),
        origin=Origin(
            xyz=(0.0, front_y, strip_center_z + (middle_band_height + strip_bar_height) * 0.5)
        ),
        material=fascia,
        name="front_strip_top",
    )
    hood_body.visual(
        Box((strip_inner_width, shell_thickness, strip_bar_height)),
        origin=Origin(
            xyz=(0.0, front_y, strip_center_z - (middle_band_height + strip_bar_height) * 0.5)
        ),
        material=fascia,
        name="front_strip_bottom",
    )

    control_openings = [
        ("left_outer", button_x_positions["left_outer"], button_aperture),
        ("left_inner", button_x_positions["left_inner"], button_aperture),
        ("center", 0.0, knob_aperture),
        ("right_inner", button_x_positions["right_inner"], button_aperture),
        ("right_outer", button_x_positions["right_outer"], button_aperture),
    ]
    half_strip = strip_inner_width * 0.5
    opening_edges = [(-half_strip, control_openings[0][1] - control_openings[0][2] * 0.5)]
    for (_, center_x, aperture), (_, next_center_x, next_aperture) in zip(control_openings, control_openings[1:]):
        opening_edges.append((center_x + aperture * 0.5, next_center_x - next_aperture * 0.5))
    opening_edges.append((control_openings[-1][1] + control_openings[-1][2] * 0.5, half_strip))

    segment_names = (
        "outer_left_strip",
        "between_left_outer_and_left_inner",
        "between_left_inner_and_knob",
        "between_knob_and_right_inner",
        "between_right_inner_and_right_outer",
        "outer_right_strip",
    )
    for visual_name, (start_x, end_x) in zip(segment_names, opening_edges):
        width = end_x - start_x
        hood_body.visual(
            Box((width, shell_thickness, middle_band_height)),
            origin=Origin(
                xyz=((start_x + end_x) * 0.5, front_y, strip_center_z)
            ),
            material=fascia,
            name=visual_name,
        )

    filter_thickness = 0.004
    filter_gap = 0.016
    filter_width = (hood_width - 2.0 * shell_thickness - filter_gap) * 0.5
    filter_depth = hood_depth - 0.120
    filter_center_y = 0.015
    filter_z = hood_height - shell_thickness - filter_thickness * 0.5
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(
            xyz=(
                -(filter_width + filter_gap) * 0.5,
                filter_center_y,
                filter_z,
            )
        ),
        material=filter_metal,
        name="left_filter",
    )
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(
            xyz=(
                (filter_width + filter_gap) * 0.5,
                filter_center_y,
                filter_z,
            )
        ),
        material=filter_metal,
        name="right_filter",
    )
    rib_depth = filter_depth - 0.040
    rib_thickness = 0.0016
    rib_z = filter_z - filter_thickness * 0.5 - rib_thickness * 0.5
    rib_offsets = (-0.105, -0.035, 0.035, 0.105)
    left_filter_center_x = -(filter_width + filter_gap) * 0.5
    right_filter_center_x = (filter_width + filter_gap) * 0.5
    for side_name, filter_center_x in (
        ("left", left_filter_center_x),
        ("right", right_filter_center_x),
    ):
        for rib_index, rib_offset in enumerate(rib_offsets):
            hood_body.visual(
                Box((0.008, rib_depth, rib_thickness)),
                origin=Origin(
                    xyz=(
                        filter_center_x + rib_offset,
                        filter_center_y,
                        rib_z,
                    )
                ),
                material=filter_metal,
                name=f"{side_name}_filter_rib_{rib_index}",
            )

    hood_body.visual(
        Box((0.170, 0.048, 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                -0.206,
                hood_height - shell_thickness - 0.002,
            )
        ),
        material=lens_white,
        name="light_lens",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((hood_width, hood_depth, hood_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, hood_height * 0.5)),
    )

    knob = model.part("central_knob")
    knob.visual(
        Cylinder(radius=0.031, length=0.020),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="knob_grip",
    )
    knob.visual(
        Cylinder(radius=knob_shaft_radius, length=0.026),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="knob_shaft",
    )
    knob.visual(
        Box((0.006, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.031, 0.018)),
        material=marker,
        name="knob_marker",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.070)),
        mass=0.11,
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
    )
    model.articulation(
        "hood_to_central_knob",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child=knob,
        origin=Origin(xyz=(0.0, front_y, control_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    for name, center_x in button_x_positions.items():
        button = model.part(f"{name}_button")
        button.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=charcoal,
            name="button_head",
        )
        button.visual(
            Cylinder(radius=button_shaft_radius, length=0.022),
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=charcoal,
            name="button_shaft",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.026, 0.026, 0.026)),
            mass=0.024,
            origin=Origin(xyz=(0.0, -0.008, 0.0)),
        )
        model.articulation(
            f"hood_to_{name}_button",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(center_x, front_y, control_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    hood_body = object_model.get_part("hood_body")
    knob = object_model.get_part("central_knob")
    knob_grip = knob.get_visual("knob_grip")
    knob_joint = object_model.get_articulation("hood_to_central_knob")

    button_parts = {
        name: object_model.get_part(f"{name}_button")
        for name in ("left_outer", "left_inner", "right_inner", "right_outer")
    }
    button_joints = {
        name: object_model.get_articulation(f"hood_to_{name}_button")
        for name in ("left_outer", "left_inner", "right_inner", "right_outer")
    }

    hood_aabb = ctx.part_world_aabb(hood_body)
    if hood_aabb is None:
        ctx.fail("hood_body_aabb", "Could not resolve hood body bounds.")
    else:
        hood_width = hood_aabb[1][0] - hood_aabb[0][0]
        hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
        hood_height = hood_aabb[1][2] - hood_aabb[0][2]
        ctx.check(
            "hood_body_realistic_size",
            0.72 <= hood_width <= 0.80
            and 0.46 <= hood_depth <= 0.54
            and 0.15 <= hood_height <= 0.17,
            f"Observed hood dims {(hood_width, hood_depth, hood_height)}.",
        )

    def _axis_ok(joint_obj, expected: tuple[float, float, float]) -> bool:
        return all(abs(float(a) - float(b)) <= 1e-9 for a, b in zip(joint_obj.axis, expected))

    ctx.check(
        "central_knob_joint_type",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"Expected continuous knob, got {knob_joint.articulation_type}.",
    )
    ctx.check(
        "central_knob_axis",
        _axis_ok(knob_joint, (0.0, 1.0, 0.0)),
        f"Expected knob axis (0, 1, 0), got {knob_joint.axis}.",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "central_knob_unbounded_rotation",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        "Continuous knob should not expose lower or upper position bounds.",
    )
    ctx.expect_contact(knob, hood_body, name="central_knob_contact")
    ctx.expect_overlap(
        knob,
        hood_body,
        axes="xz",
        min_overlap=0.055,
        name="central_knob_on_front_strip",
    )
    ctx.expect_gap(
        hood_body,
        knob,
        axis="y",
        min_gap=0.003,
        max_gap=0.006,
        negative_elem=knob_grip,
        name="central_knob_face_gap",
    )
    with ctx.pose({knob_joint: math.pi * 0.5}):
        ctx.expect_contact(knob, hood_body, name="central_knob_contact_rotated")
        ctx.fail_if_parts_overlap_in_current_pose(name="central_knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="central_knob_rotated_no_floating")

    button_positions: dict[str, tuple[float, float, float]] = {}
    for name, button in button_parts.items():
        button_head = button.get_visual("button_head")
        joint = button_joints[name]
        limits = joint.motion_limits
        ctx.check(
            f"{name}_button_joint_type",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"Expected prismatic button joint, got {joint.articulation_type}.",
        )
        ctx.check(
            f"{name}_button_axis",
            _axis_ok(joint, (0.0, 1.0, 0.0)),
            f"Expected button axis (0, 1, 0), got {joint.axis}.",
        )
        ctx.expect_contact(button, hood_body, name=f"{name}_button_contact_rest")
        ctx.expect_overlap(
            button,
            hood_body,
            axes="xz",
            min_overlap=0.018,
            name=f"{name}_button_on_front_strip",
        )
        ctx.expect_gap(
            hood_body,
            button,
            axis="y",
            min_gap=0.004,
            max_gap=0.010,
            negative_elem=button_head,
            name=f"{name}_button_face_gap_rest",
        )
        button_position = ctx.part_world_position(button)
        if button_position is not None:
            button_positions[name] = button_position
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_button_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_button_lower_no_floating")
                ctx.expect_contact(button, hood_body, name=f"{name}_button_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_button_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_button_upper_no_floating")
                ctx.expect_contact(button, hood_body, name=f"{name}_button_upper_contact")
                ctx.expect_gap(
                    hood_body,
                    button,
                    axis="y",
                    min_gap=0.0005,
                    max_gap=0.006,
                    negative_elem=button_head,
                    name=f"{name}_button_face_gap_pressed",
                )
                pressed_position = ctx.part_world_position(button)
                if button_position is not None and pressed_position is not None:
                    ctx.check(
                        f"{name}_button_moves_inward",
                        pressed_position[1] > button_position[1] + 0.0035,
                        f"Expected {name} button to move inward along +y. Rest={button_position}, pressed={pressed_position}.",
                    )

    if all(name in button_positions for name in ("left_outer", "left_inner", "right_inner", "right_outer")):
        left_outer = button_positions["left_outer"]
        left_inner = button_positions["left_inner"]
        right_inner = button_positions["right_inner"]
        right_outer = button_positions["right_outer"]
        ctx.check(
            "button_layout_symmetric_outer_pair",
            abs(left_outer[0] + right_outer[0]) <= 1e-6
            and abs(left_outer[1] - right_outer[1]) <= 1e-6
            and abs(left_outer[2] - right_outer[2]) <= 1e-6,
            f"Outer button positions are not mirrored: {left_outer} vs {right_outer}.",
        )
        ctx.check(
            "button_layout_symmetric_inner_pair",
            abs(left_inner[0] + right_inner[0]) <= 1e-6
            and abs(left_inner[1] - right_inner[1]) <= 1e-6
            and abs(left_inner[2] - right_inner[2]) <= 1e-6,
            f"Inner button positions are not mirrored: {left_inner} vs {right_inner}.",
        )
        ctx.check(
            "five_control_order",
            left_outer[0] < left_inner[0] < 0.0 < right_inner[0] < right_outer[0],
            f"Unexpected control ordering from button positions {button_positions}.",
        )

    knob_position = ctx.part_world_position(knob)
    if knob_position is not None:
        ctx.check(
            "central_knob_centered",
            abs(knob_position[0]) <= 1e-6,
            f"Expected central knob on hood centerline, got {knob_position}.",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="control_pose_overlap_sweep")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
