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
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _button_cap_mesh():
    profile = rounded_rect_profile(0.016, 0.014, 0.0026, corner_segments=8)
    geom = ExtrudeGeometry.centered(profile, 0.016, cap=True, closed=True).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("range_hood_button_cap.obj"))


def _knob_body_mesh():
    profile = [
        (0.0, 0.000),
        (0.024, 0.000),
        (0.024, 0.002),
        (0.0225, 0.004),
        (0.0215, 0.010),
        (0.0210, 0.020),
        (0.0185, 0.026),
        (0.0, 0.026),
    ]
    geom = LatheGeometry(profile, segments=48).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("range_hood_knob_body.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    steel_shadow = model.material("steel_shadow", rgba=(0.60, 0.63, 0.66, 1.0))
    control_black = model.material("control_black", rgba=(0.11, 0.11, 0.12, 1.0))
    control_grey = model.material("control_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    indicator = model.material("indicator", rgba=(0.76, 0.77, 0.79, 1.0))

    hood_width = 0.90
    front_y = 0.250
    front_lip_thickness = 0.012
    front_inner_y = front_y - front_lip_thickness * 0.5
    front_outer_y = front_y + front_lip_thickness * 0.5
    top_z = 0.260
    button_cap_mesh = _button_cap_mesh()
    knob_body_mesh = _knob_body_mesh()

    body = model.part("hood_body")
    body.visual(
        Box((hood_width, 0.012, 0.220)),
        origin=Origin(xyz=(0.0, -0.244, 0.150)),
        material=stainless,
        name="canopy_back",
    )
    body.visual(
        Box((hood_width, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, -0.160, top_z - 0.006)),
        material=stainless,
        name="canopy_top",
    )
    body.visual(
        Box((0.012, 0.380, 0.100)),
        origin=Origin(xyz=(-0.444, 0.060, 0.150), rpy=(-0.42, 0.0, 0.0)),
        material=stainless,
        name="canopy_left_side",
    )
    body.visual(
        Box((0.012, 0.380, 0.100)),
        origin=Origin(xyz=(0.444, 0.060, 0.150), rpy=(-0.42, 0.0, 0.0)),
        material=stainless,
        name="canopy_right_side",
    )
    body.visual(
        Box((hood_width, front_lip_thickness, 0.027)),
        origin=Origin(xyz=(0.0, front_y, 0.0915)),
        material=stainless,
        name="front_top_strip",
    )
    body.visual(
        Box((0.132, front_lip_thickness, 0.014)),
        origin=Origin(xyz=(-0.384, front_y, 0.071)),
        material=steel_shadow,
        name="front_button_left_guard",
    )
    body.visual(
        Box((0.013, front_lip_thickness, 0.014)),
        origin=Origin(xyz=(-0.2955, front_y, 0.071)),
        material=steel_shadow,
        name="front_button_separator_1",
    )
    body.visual(
        Box((0.013, front_lip_thickness, 0.014)),
        origin=Origin(xyz=(-0.2665, front_y, 0.071)),
        material=steel_shadow,
        name="front_button_separator_2",
    )
    body.visual(
        Box((0.694, front_lip_thickness, 0.014)),
        origin=Origin(xyz=(0.103, front_y, 0.071)),
        material=steel_shadow,
        name="front_right_band",
    )
    body.visual(
        Box((0.320, 0.012, 0.780)),
        origin=Origin(xyz=(0.0, -0.244, 0.650)),
        material=stainless,
        name="chimney_back",
    )
    body.visual(
        Box((0.320, 0.012, 0.780)),
        origin=Origin(xyz=(0.0, -0.036, 0.650)),
        material=stainless,
        name="chimney_front",
    )
    body.visual(
        Box((0.012, 0.220, 0.780)),
        origin=Origin(xyz=(-0.154, -0.140, 0.650)),
        material=stainless,
        name="chimney_left",
    )
    body.visual(
        Box((0.012, 0.220, 0.780)),
        origin=Origin(xyz=(0.154, -0.140, 0.650)),
        material=stainless,
        name="chimney_right",
    )
    body.visual(
        Box((0.320, 0.220, 0.012)),
        origin=Origin(xyz=(0.0, -0.140, 1.034)),
        material=steel_shadow,
        name="chimney_cap",
    )

    button_centers = (-0.310, -0.281, -0.252)
    for index, center_x in enumerate(button_centers, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            button_cap_mesh,
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
            material=control_grey,
            name="button_cap",
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(center_x, front_inner_y, 0.071)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.050,
                lower=0.0,
                upper=0.004,
            ),
        )

    knob = model.part("control_knob")
    knob.visual(
        knob_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=control_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.024, length=0.003),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_grey,
        name="knob_bezel",
    )
    knob.visual(
        Box((0.006, 0.006, 0.014)),
        origin=Origin(xyz=(0.013, 0.023, 0.0)),
        material=indicator,
        name="knob_indicator",
    )
    model.articulation(
        "hood_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.285, front_outer_y, 0.071)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("hood_body")
    buttons = [
        object_model.get_part("button_1"),
        object_model.get_part("button_2"),
        object_model.get_part("button_3"),
    ]
    button_joints = [
        object_model.get_articulation("hood_to_button_1"),
        object_model.get_articulation("hood_to_button_2"),
        object_model.get_articulation("hood_to_button_3"),
    ]
    knob = object_model.get_part("control_knob")
    knob_joint = object_model.get_articulation("hood_to_knob")
    chimney_front = body.get_visual("chimney_front")
    canopy_top = body.get_visual("canopy_top")

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
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="controls_supported_across_sampled_poses")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="controls_clear_in_sampled_poses")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("hood_body_aabb", "hood_body returned no world AABB")
    else:
        body_width = body_aabb[1][0] - body_aabb[0][0]
        body_depth = body_aabb[1][1] - body_aabb[0][1]
        body_height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check("hood_width_realistic", 0.84 <= body_width <= 0.96, f"width={body_width:.3f} m")
        ctx.check("hood_depth_realistic", 0.46 <= body_depth <= 0.56, f"depth={body_depth:.3f} m")
        ctx.check("hood_height_realistic", 0.95 <= body_height <= 1.08, f"height={body_height:.3f} m")

    chimney_aabb = ctx.part_element_world_aabb(body, elem=chimney_front)
    canopy_aabb = ctx.part_element_world_aabb(body, elem=canopy_top)
    if chimney_aabb is None or canopy_aabb is None:
        ctx.fail("hood_named_visual_aabbs", "chimney_front or canopy_top visual AABB missing")
    else:
        chimney_center_x = 0.5 * (chimney_aabb[0][0] + chimney_aabb[1][0])
        canopy_center_x = 0.5 * (canopy_aabb[0][0] + canopy_aabb[1][0])
        chimney_width = chimney_aabb[1][0] - chimney_aabb[0][0]
        ctx.check(
            "chimney_centered_on_canopy",
            abs(chimney_center_x - canopy_center_x) <= 0.003,
            f"chimney_center_x={chimney_center_x:.4f}, canopy_center_x={canopy_center_x:.4f}",
        )
        ctx.check(
            "chimney_narrower_than_canopy",
            chimney_width <= 0.40,
            f"chimney_width={chimney_width:.3f} m",
        )

    button_positions = [ctx.part_world_position(button) for button in buttons]
    knob_position = ctx.part_world_position(knob)
    if any(position is None for position in button_positions) or knob_position is None:
        ctx.fail("control_positions_exist", "one or more control part positions are unavailable")
    else:
        left_ok = all(position[0] < -0.20 for position in button_positions)
        row_y = max(abs(position[1] - button_positions[0][1]) for position in button_positions[1:]) <= 1e-6
        row_z = max(abs(position[2] - button_positions[0][2]) for position in button_positions[1:]) <= 1e-6
        spacing_1 = button_positions[1][0] - button_positions[0][0]
        spacing_2 = button_positions[2][0] - button_positions[1][0]
        ctx.check(
            "buttons_form_left_row",
            left_ok and row_y and row_z and abs(spacing_1 - spacing_2) <= 0.002,
            (
                f"xs={[round(position[0], 4) for position in button_positions]}, "
                f"spacing=({spacing_1:.4f}, {spacing_2:.4f})"
            ),
        )
        ctx.check(
            "knob_isolated_on_right",
            knob_position[0] > 0.20 and (knob_position[0] - button_positions[2][0]) >= 0.48,
            f"knob_x={knob_position[0]:.4f}, right_button_x={button_positions[2][0]:.4f}",
        )

    for index, (button, joint) in enumerate(zip(buttons, button_joints, strict=True), start=1):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index}_joint_type",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"type={joint.articulation_type}",
        )
        ctx.check(
            f"button_{index}_joint_axis",
            tuple(round(value, 6) for value in joint.axis) == (0.0, -1.0, 0.0),
            f"axis={joint.axis}",
        )
        ctx.check(
            f"button_{index}_travel_range",
            limits is not None and limits.lower == 0.0 and abs(limits.upper - 0.004) <= 1e-9,
            f"limits={limits}",
        )
        ctx.expect_contact(button, body, name=f"button_{index}_rest_contact")

        rest_position = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper if limits is not None else 0.004}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"button_{index}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"button_{index}_upper_no_floating")
            ctx.expect_contact(button, body, name=f"button_{index}_upper_contact")
            pressed_position = ctx.part_world_position(button)
        if rest_position is None or pressed_position is None:
            ctx.fail(f"button_{index}_pose_positions", "missing rest or pressed part world position")
        else:
            ctx.check(
                f"button_{index}_moves_inward",
                abs(pressed_position[0] - rest_position[0]) <= 1e-6
                and abs(pressed_position[2] - rest_position[2]) <= 1e-6
                and pressed_position[1] < rest_position[1] - 0.0035,
                f"rest={rest_position}, pressed={pressed_position}",
            )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_joint_type",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "knob_joint_axis",
        tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
        f"axis={knob_joint.axis}",
    )
    ctx.check(
        "knob_continuous_limits",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        f"limits={knob_limits}",
    )
    ctx.expect_contact(knob, body, name="knob_rest_contact")

    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 1.7}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_pose_no_floating")
        ctx.expect_contact(knob, body, name="knob_pose_contact")
        knob_turned = ctx.part_world_position(knob)
    if knob_rest is None or knob_turned is None:
        ctx.fail("knob_pose_positions", "missing rest or turned knob position")
    else:
        ctx.check(
            "knob_rotates_in_place",
            max(abs(knob_rest[i] - knob_turned[i]) for i in range(3)) <= 1e-6,
            f"rest={knob_rest}, turned={knob_turned}",
        )

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
