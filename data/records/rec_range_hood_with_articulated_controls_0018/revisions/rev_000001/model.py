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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_side_panel_mesh():
    side_profile = [
        (0.000, 0.000),
        (0.000, 0.300),
        (0.200, 0.300),
        (0.500, 0.090),
        (0.500, 0.000),
    ]
    return _save_mesh(
        "range_hood_side_panel.obj",
        ExtrudeGeometry(side_profile, 0.018).rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.31, 0.33, 0.35, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    filter_black = model.material("filter_black", rgba=(0.18, 0.19, 0.20, 1.0))
    indicator_silver = model.material("indicator_silver", rgba=(0.85, 0.86, 0.87, 1.0))

    hood_width = 0.900
    hood_depth = 0.500
    back_height = 0.300
    strip_height = 0.090
    panel_thickness = 0.018
    rear_roof_depth = 0.200
    slope_drop = back_height - strip_height
    slope_run = hood_depth - rear_roof_depth
    front_roof_angle = -math.atan2(slope_drop, slope_run)
    front_roof_length = math.hypot(slope_run, slope_drop)

    chimney_width = 0.300
    chimney_depth = 0.220
    chimney_height = 0.620
    chimney_panel = 0.016

    hood_body = model.part("hood_body")
    side_panel_mesh = _build_side_panel_mesh()

    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(-0.441, 0.000, 0.000)),
        material=stainless,
        name="left_side_panel",
    )
    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(0.441, 0.000, 0.000)),
        material=stainless,
        name="right_side_panel",
    )
    hood_body.visual(
        Box((hood_width, panel_thickness, back_height)),
        origin=Origin(xyz=(0.000, panel_thickness / 2.0, back_height / 2.0)),
        material=stainless,
        name="back_panel",
    )
    hood_body.visual(
        Box((hood_width, rear_roof_depth, panel_thickness)),
        origin=Origin(
            xyz=(0.000, rear_roof_depth / 2.0, back_height - (panel_thickness / 2.0))
        ),
        material=stainless,
        name="rear_roof",
    )
    hood_body.visual(
        Box((hood_width, front_roof_length, panel_thickness)),
        origin=Origin(
            xyz=(0.000, 0.350, 0.195),
            rpy=(front_roof_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="front_roof",
    )
    hood_body.visual(
        Box((hood_width, panel_thickness, strip_height)),
        origin=Origin(xyz=(0.000, hood_depth - (panel_thickness / 2.0), strip_height / 2.0)),
        material=brushed_steel,
        name="front_strip",
    )

    hood_body.visual(
        Box((hood_width, 0.035, 0.025)),
        origin=Origin(xyz=(0.000, 0.0175, 0.0125)),
        material=shadow_steel,
        name="rear_filter_rail",
    )
    hood_body.visual(
        Box((hood_width, 0.035, 0.025)),
        origin=Origin(xyz=(0.000, 0.4825, 0.0125)),
        material=shadow_steel,
        name="front_filter_rail",
    )
    hood_body.visual(
        Box((0.035, 0.430, 0.025)),
        origin=Origin(xyz=(-0.4325, 0.250, 0.0125)),
        material=shadow_steel,
        name="left_filter_rail",
    )
    hood_body.visual(
        Box((0.035, 0.430, 0.025)),
        origin=Origin(xyz=(0.4325, 0.250, 0.0125)),
        material=shadow_steel,
        name="right_filter_rail",
    )
    hood_body.visual(
        Box((0.030, 0.430, 0.025)),
        origin=Origin(xyz=(0.000, 0.250, 0.0125)),
        material=shadow_steel,
        name="center_filter_rail",
    )
    hood_body.visual(
        Box((0.400, 0.430, 0.008)),
        origin=Origin(xyz=(-0.215, 0.250, 0.017)),
        material=filter_black,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.400, 0.430, 0.008)),
        origin=Origin(xyz=(0.215, 0.250, 0.017)),
        material=filter_black,
        name="right_filter",
    )

    hood_body.visual(
        Box((chimney_width + 0.004, chimney_panel, chimney_height)),
        origin=Origin(
            xyz=(
                0.000,
                chimney_depth - (chimney_panel / 2.0),
                back_height + (chimney_height / 2.0),
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width + 0.004, chimney_panel, chimney_height)),
        origin=Origin(
            xyz=(0.000, chimney_panel / 2.0, back_height + (chimney_height / 2.0))
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_panel, chimney_depth + 0.004, chimney_height)),
        origin=Origin(
            xyz=(
                -(chimney_width / 2.0) + (chimney_panel / 2.0),
                chimney_depth / 2.0,
                back_height + (chimney_height / 2.0),
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((chimney_panel, chimney_depth + 0.004, chimney_height)),
        origin=Origin(
            xyz=(
                (chimney_width / 2.0) - (chimney_panel / 2.0),
                chimney_depth / 2.0,
                back_height + (chimney_height / 2.0),
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((hood_width, hood_depth, back_height + chimney_height)),
        mass=26.0,
        origin=Origin(xyz=(0.000, hood_depth / 2.0, (back_height + chimney_height) / 2.0)),
    )

    knob_specs = [
        ("knob_left", "hood_to_knob_left", 0.292, 0.015),
        ("knob_center", "hood_to_knob_center", 0.344, 0.018),
        ("knob_right", "hood_to_knob_right", 0.400, 0.021),
    ]
    knob_depth = 0.030

    for part_name, joint_name, x_pos, radius in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=radius, length=knob_depth),
            origin=Origin(
                xyz=(0.000, knob_depth / 2.0, 0.000),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=radius * 0.78, length=0.006),
            origin=Origin(
                xyz=(0.000, knob_depth - 0.003, 0.000),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=shadow_steel,
            name="knob_face",
        )
        knob.visual(
            Box((radius * 0.28, 0.004, 0.003)),
            origin=Origin(xyz=(0.000, knob_depth - 0.001, radius * 0.58)),
            material=indicator_silver,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((radius * 2.0, knob_depth, radius * 2.0)),
            mass=0.08,
            origin=Origin(xyz=(0.000, knob_depth / 2.0, 0.000)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, hood_depth, strip_height / 2.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=9.0),
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
    knob_left = object_model.get_part("knob_left")
    knob_center = object_model.get_part("knob_center")
    knob_right = object_model.get_part("knob_right")

    front_strip = hood_body.get_visual("front_strip")
    chimney_front = hood_body.get_visual("chimney_front")

    left_joint = object_model.get_articulation("hood_to_knob_left")
    center_joint = object_model.get_articulation("hood_to_knob_center")
    right_joint = object_model.get_articulation("hood_to_knob_right")

    ctx.check(
        "only_three_articulations",
        len(object_model.articulations) == 3,
        details=f"Expected exactly 3 articulations, found {len(object_model.articulations)}.",
    )
    ctx.check(
        "single_root_body",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "hood_body",
        details="The range hood should be one rooted body with the three knobs attached to it.",
    )

    for joint in (left_joint, center_joint, right_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_type",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} should be CONTINUOUS.",
        )
        ctx.check(
            f"{joint.name}_front_to_back_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name} axis should be (0, 1, 0), found {joint.axis}.",
        )
        ctx.check(
            f"{joint.name}_unbounded_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} should be continuous with no finite lower/upper limits.",
        )

    for knob in (knob_left, knob_center, knob_right):
        ctx.expect_contact(
            knob,
            hood_body,
            elem_b=front_strip,
            name=f"{knob.name}_touches_front_strip",
        )
        ctx.expect_within(
            knob,
            hood_body,
            axes="xz",
            outer_elem=front_strip,
            name=f"{knob.name}_sits_on_control_band",
        )

    body_aabb = ctx.part_world_aabb(hood_body)
    ctx.check("hood_body_aabb_available", body_aabb is not None, details="Failed to measure hood body.")
    if body_aabb is not None:
        hood_size = (
            body_aabb[1][0] - body_aabb[0][0],
            body_aabb[1][1] - body_aabb[0][1],
            body_aabb[1][2] - body_aabb[0][2],
        )
        ctx.check(
            "realistic_range_hood_size",
            0.88 <= hood_size[0] <= 0.92
            and 0.49 <= hood_size[1] <= 0.52
            and 0.90 <= hood_size[2] <= 0.93,
            details=f"Unexpected hood size {hood_size}.",
        )

    chimney_aabb = ctx.part_element_world_aabb(hood_body, elem=chimney_front)
    ctx.check(
        "chimney_front_present",
        chimney_aabb is not None,
        details="Failed to measure the chimney front panel.",
    )
    if chimney_aabb is not None:
        ctx.check(
            "chimney_above_canopy",
            chimney_aabb[0][2] >= 0.299,
            details=f"Chimney starts too low: {chimney_aabb[0][2]:.4f} m.",
        )

    knob_positions = []
    knob_widths = []
    for knob in (knob_left, knob_center, knob_right):
        pos = ctx.part_world_position(knob)
        aabb = ctx.part_world_aabb(knob)
        ctx.check(f"{knob.name}_position_available", pos is not None, details="Missing knob world position.")
        ctx.check(f"{knob.name}_aabb_available", aabb is not None, details="Missing knob world bounds.")
        if pos is not None:
            knob_positions.append(pos)
        if aabb is not None:
            knob_widths.append(aabb[1][0] - aabb[0][0])

    if len(knob_positions) == 3:
        x_positions = [pos[0] for pos in knob_positions]
        spacings = [x_positions[1] - x_positions[0], x_positions[2] - x_positions[1]]
        ctx.check(
            "knobs_clustered_at_far_right",
            x_positions[0] > 0.26 and x_positions[2] > 0.37,
            details=f"Knob x positions should sit at the far right, got {x_positions}.",
        )
        ctx.check(
            "knobs_left_to_right_ordered",
            x_positions[0] < x_positions[1] < x_positions[2],
            details=f"Knob positions are not ordered left-to-right: {x_positions}.",
        )
        ctx.check(
            "knobs_close_together",
            0.04 <= spacings[0] <= 0.065 and 0.04 <= spacings[1] <= 0.07,
            details=f"Unexpected knob spacing {spacings}.",
        )

    if len(knob_widths) == 3:
        ctx.check(
            "knob_diameters_increase_left_to_right",
            knob_widths[0] < knob_widths[1] < knob_widths[2],
            details=f"Knob widths should increase left-to-right, got {knob_widths}.",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="all_poses_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="knob_rotation_clearance")

    with ctx.pose(
        {
            left_joint: 0.8,
            center_joint: 2.2,
            right_joint: -1.4,
        }
    ):
        for knob in (knob_left, knob_center, knob_right):
            ctx.expect_contact(
                knob,
                hood_body,
                elem_b=front_strip,
                name=f"{knob.name}_pose_contact",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="posed_knobs_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
