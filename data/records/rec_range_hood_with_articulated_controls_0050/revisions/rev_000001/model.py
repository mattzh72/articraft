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

HOOD_WIDTH = 0.76
HOOD_DEPTH = 0.50
HOOD_HEIGHT = 0.14
SHELL_THICKNESS = 0.015
SIDE_HEIGHT = HOOD_HEIGHT - SHELL_THICKNESS
INNER_WIDTH = HOOD_WIDTH - 2.0 * SHELL_THICKNESS
INNER_DEPTH = HOOD_DEPTH - 2.0 * SHELL_THICKNESS
FRONT_WALL_Y = HOOD_DEPTH * 0.5 - SHELL_THICKNESS * 0.5
REAR_WALL_Y = -FRONT_WALL_Y
TOP_PANEL_Z = HOOD_HEIGHT - SHELL_THICKNESS * 0.5
FRONT_OUTER_Y = HOOD_DEPTH * 0.5

BUTTON_HOLE = 0.012
BUTTON_CENTER_OFFSET = 0.011
BUTTON_LOWER_Z = 0.061
BUTTON_UPPER_Z = 0.083
BUTTON_TRAVEL = 0.003
BUTTON_CAP_RADIUS = 0.009
BUTTON_CAP_LENGTH = 0.004
BUTTON_STEM_LENGTH = 0.025


def _add_box_span(
    part,
    *,
    x0: float,
    x1: float,
    y_center: float,
    y_size: float,
    z0: float,
    z1: float,
    material,
    name: str,
) -> None:
    part.visual(
        Box((x1 - x0, y_size, z1 - z0)),
        origin=Origin(xyz=((x0 + x1) * 0.5, y_center, (z0 + z1) * 0.5)),
        material=material,
        name=name,
    )


def _add_filter_panel(
    part,
    *,
    x_center: float,
    width: float,
    depth: float,
    thickness: float,
    frame_material,
    slat_material,
    name_prefix: str,
    slat_count: int = 6,
) -> None:
    frame_width = 0.018
    frame_depth = 0.018
    z_center = thickness * 0.5

    part.visual(
        Box((width, frame_depth, thickness)),
        origin=Origin(xyz=(x_center, depth * 0.5 - frame_depth * 0.5, z_center)),
        material=frame_material,
        name=f"{name_prefix}_front_rail",
    )
    part.visual(
        Box((width, frame_depth, thickness)),
        origin=Origin(xyz=(x_center, -(depth * 0.5 - frame_depth * 0.5), z_center)),
        material=frame_material,
        name=f"{name_prefix}_rear_rail",
    )
    part.visual(
        Box((frame_width, depth - 2.0 * frame_depth, thickness)),
        origin=Origin(xyz=(x_center - (width * 0.5 - frame_width * 0.5), 0.0, z_center)),
        material=frame_material,
        name=f"{name_prefix}_outer_rail",
    )
    part.visual(
        Box((frame_width, depth - 2.0 * frame_depth, thickness)),
        origin=Origin(xyz=(x_center + (width * 0.5 - frame_width * 0.5), 0.0, z_center)),
        material=frame_material,
        name=f"{name_prefix}_inner_rail",
    )

    inner_clear_width = width - 2.0 * frame_width
    slat_width = 0.016
    for index in range(slat_count):
        t = (index + 1) / (slat_count + 1)
        slat_x = x_center - inner_clear_width * 0.5 + inner_clear_width * t
        part.visual(
            Box((slat_width, depth - 2.0 * frame_depth, thickness)),
            origin=Origin(xyz=(slat_x, 0.0, z_center)),
            material=slat_material,
            name=f"{name_prefix}_slat_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    stainless = model.material("stainless_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    shadow_metal = model.material("shadow_metal", rgba=(0.48, 0.50, 0.53, 1.0))
    filter_charcoal = model.material("filter_charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    button_black = model.material("button_black", rgba=(0.11, 0.11, 0.12, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((HOOD_WIDTH, HOOD_DEPTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PANEL_Z)),
        material=stainless,
        name="top_panel",
    )
    hood_body.visual(
        Box((SHELL_THICKNESS, HOOD_DEPTH, SIDE_HEIGHT)),
        origin=Origin(
            xyz=(-(HOOD_WIDTH * 0.5 - SHELL_THICKNESS * 0.5), 0.0, SIDE_HEIGHT * 0.5)
        ),
        material=stainless,
        name="left_side",
    )
    hood_body.visual(
        Box((SHELL_THICKNESS, HOOD_DEPTH, SIDE_HEIGHT)),
        origin=Origin(
            xyz=((HOOD_WIDTH * 0.5 - SHELL_THICKNESS * 0.5), 0.0, SIDE_HEIGHT * 0.5)
        ),
        material=stainless,
        name="right_side",
    )
    hood_body.visual(
        Box((INNER_WIDTH, SHELL_THICKNESS, SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, REAR_WALL_Y, SIDE_HEIGHT * 0.5)),
        material=stainless,
        name="rear_wall",
    )

    x_left_outer = -INNER_WIDTH * 0.5
    x_left_hole_min = -BUTTON_CENTER_OFFSET - BUTTON_HOLE * 0.5
    x_left_hole_max = -BUTTON_CENTER_OFFSET + BUTTON_HOLE * 0.5
    x_right_hole_min = BUTTON_CENTER_OFFSET - BUTTON_HOLE * 0.5
    x_right_hole_max = BUTTON_CENTER_OFFSET + BUTTON_HOLE * 0.5
    x_right_outer = INNER_WIDTH * 0.5

    z_lower_hole_min = BUTTON_LOWER_Z - BUTTON_HOLE * 0.5
    z_lower_hole_max = BUTTON_LOWER_Z + BUTTON_HOLE * 0.5
    z_upper_hole_min = BUTTON_UPPER_Z - BUTTON_HOLE * 0.5
    z_upper_hole_max = BUTTON_UPPER_Z + BUTTON_HOLE * 0.5

    _add_box_span(
        hood_body,
        x0=x_left_outer,
        x1=x_right_outer,
        y_center=FRONT_WALL_Y,
        y_size=SHELL_THICKNESS,
        z0=0.0,
        z1=z_lower_hole_min,
        material=stainless,
        name="front_bottom_band",
    )
    _add_box_span(
        hood_body,
        x0=x_left_outer,
        x1=x_right_outer,
        y_center=FRONT_WALL_Y,
        y_size=SHELL_THICKNESS,
        z0=z_upper_hole_max,
        z1=SIDE_HEIGHT,
        material=stainless,
        name="front_top_band",
    )
    _add_box_span(
        hood_body,
        x0=x_left_outer,
        x1=x_right_outer,
        y_center=FRONT_WALL_Y,
        y_size=SHELL_THICKNESS,
        z0=z_lower_hole_max,
        z1=z_upper_hole_min,
        material=shadow_metal,
        name="front_middle_band",
    )
    for row_name, row_min, row_max in (
        ("lower", z_lower_hole_min, z_lower_hole_max),
        ("upper", z_upper_hole_min, z_upper_hole_max),
    ):
        _add_box_span(
            hood_body,
            x0=x_left_outer,
            x1=x_left_hole_min,
            y_center=FRONT_WALL_Y,
            y_size=SHELL_THICKNESS,
            z0=row_min,
            z1=row_max,
            material=stainless,
            name=f"front_{row_name}_left_band",
        )
        _add_box_span(
            hood_body,
            x0=x_left_hole_max,
            x1=x_right_hole_min,
            y_center=FRONT_WALL_Y,
            y_size=SHELL_THICKNESS,
            z0=row_min,
            z1=row_max,
            material=shadow_metal,
            name=f"front_{row_name}_center_band",
        )
        _add_box_span(
            hood_body,
            x0=x_right_hole_max,
            x1=x_right_outer,
            y_center=FRONT_WALL_Y,
            y_size=SHELL_THICKNESS,
            z0=row_min,
            z1=row_max,
            material=stainless,
            name=f"front_{row_name}_right_band",
        )

    filter_thickness = 0.008
    filter_center_bar = 0.012
    filter_width = (INNER_WIDTH - filter_center_bar) * 0.5
    filter_depth = INNER_DEPTH
    _add_filter_panel(
        hood_body,
        x_center=-(filter_center_bar * 0.5 + filter_width * 0.5),
        width=filter_width,
        depth=filter_depth,
        thickness=filter_thickness,
        frame_material=shadow_metal,
        slat_material=filter_charcoal,
        name_prefix="left_filter",
    )
    hood_body.visual(
        Box((filter_center_bar, filter_depth, filter_thickness)),
        origin=Origin(xyz=(0.0, 0.0, filter_thickness * 0.5)),
        material=shadow_metal,
        name="center_filter_bar",
    )
    _add_filter_panel(
        hood_body,
        x_center=(filter_center_bar * 0.5 + filter_width * 0.5),
        width=filter_width,
        depth=filter_depth,
        thickness=filter_thickness,
        frame_material=shadow_metal,
        slat_material=filter_charcoal,
        name_prefix="right_filter",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((HOOD_WIDTH, HOOD_DEPTH, HOOD_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, HOOD_HEIGHT * 0.5)),
    )

    button_positions = (
        ("button_upper_left", -BUTTON_CENTER_OFFSET, BUTTON_UPPER_Z),
        ("button_upper_right", BUTTON_CENTER_OFFSET, BUTTON_UPPER_Z),
        ("button_lower_left", -BUTTON_CENTER_OFFSET, BUTTON_LOWER_Z),
        ("button_lower_right", BUTTON_CENTER_OFFSET, BUTTON_LOWER_Z),
    )
    for part_name, button_x, button_z in button_positions:
        button_part = model.part(part_name)
        button_part.visual(
            Box((BUTTON_HOLE, BUTTON_STEM_LENGTH, BUTTON_HOLE)),
            origin=Origin(xyz=(0.0, BUTTON_TRAVEL - BUTTON_STEM_LENGTH * 0.5, 0.0)),
            material=button_black,
            name="stem",
        )
        button_part.visual(
            Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_LENGTH),
            origin=Origin(
                xyz=(0.0, BUTTON_TRAVEL + BUTTON_CAP_LENGTH * 0.5, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=button_black,
            name="cap",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((BUTTON_CAP_RADIUS * 2.0, BUTTON_STEM_LENGTH + BUTTON_CAP_LENGTH, BUTTON_CAP_RADIUS * 2.0)),
            mass=0.03,
            origin=Origin(
                xyz=(
                    0.0,
                    BUTTON_TRAVEL + (BUTTON_CAP_LENGTH - BUTTON_STEM_LENGTH) * 0.5,
                    0.0,
                )
            ),
        )
        model.articulation(
            f"hood_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button_part,
            origin=Origin(xyz=(button_x, FRONT_OUTER_Y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    button_names = (
        "button_upper_left",
        "button_upper_right",
        "button_lower_left",
        "button_lower_right",
    )
    buttons = {name: object_model.get_part(name) for name in button_names}
    button_joints = {
        name: object_model.get_articulation(f"hood_to_{name}") for name in button_names
    }

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

    hood_aabb = ctx.part_world_aabb(hood_body)
    if hood_aabb is None:
        ctx.fail("hood_body_aabb_exists", "hood body world AABB could not be resolved")
    else:
        hood_min, hood_max = hood_aabb
        hood_size = (
            hood_max[0] - hood_min[0],
            hood_max[1] - hood_min[1],
            hood_max[2] - hood_min[2],
        )
        ctx.check(
            "hood_width_realistic",
            0.70 <= hood_size[0] <= 0.90,
            f"expected width in [0.70, 0.90], got {hood_size[0]:.4f}",
        )
        ctx.check(
            "hood_depth_realistic",
            0.42 <= hood_size[1] <= 0.58,
            f"expected depth in [0.42, 0.58], got {hood_size[1]:.4f}",
        )
        ctx.check(
            "hood_height_low_profile",
            0.10 <= hood_size[2] <= 0.18,
            f"expected height in [0.10, 0.18], got {hood_size[2]:.4f}",
        )

    ctx.check(
        "only_four_button_articulations",
        len(getattr(object_model, "articulations", ())) == 4,
        f"expected exactly 4 articulations, got {len(getattr(object_model, 'articulations', ()))}",
    )

    top_left = ctx.part_world_position(buttons["button_upper_left"])
    top_right = ctx.part_world_position(buttons["button_upper_right"])
    bottom_left = ctx.part_world_position(buttons["button_lower_left"])
    bottom_right = ctx.part_world_position(buttons["button_lower_right"])
    if None not in (top_left, top_right, bottom_left, bottom_right):
        cluster_center_x = (top_left[0] + top_right[0] + bottom_left[0] + bottom_right[0]) * 0.25
        cluster_center_z = (top_left[2] + top_right[2] + bottom_left[2] + bottom_right[2]) * 0.25
        horizontal_spacing = top_right[0] - top_left[0]
        vertical_spacing = top_left[2] - bottom_left[2]
        ctx.check(
            "button_cluster_front_centered",
            abs(cluster_center_x) <= 0.002,
            f"expected button cluster centered on x=0, got {cluster_center_x:.4f}",
        )
        ctx.check(
            "button_cluster_height_centered_on_face",
            0.065 <= cluster_center_z <= 0.080,
            f"expected button cluster center z in [0.065, 0.080], got {cluster_center_z:.4f}",
        )
        ctx.check(
            "button_cluster_compact_width",
            0.018 <= horizontal_spacing <= 0.028,
            f"expected compact horizontal spacing in [0.018, 0.028], got {horizontal_spacing:.4f}",
        )
        ctx.check(
            "button_cluster_compact_height",
            0.018 <= vertical_spacing <= 0.028,
            f"expected compact vertical spacing in [0.018, 0.028], got {vertical_spacing:.4f}",
        )
    else:
        ctx.fail("button_cluster_positions_exist", "one or more button world positions could not be resolved")

    for name, button in buttons.items():
        joint = button_joints[name]
        limits = joint.motion_limits
        ctx.check(
            f"{name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"expected PRISMATIC articulation, got {joint.articulation_type}",
        )
        ctx.check(
            f"{name}_axis_points_into_hood",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"expected axis (0, -1, 0), got {joint.axis}",
        )
        ctx.check(
            f"{name}_travel_is_short",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - 0.0) <= 1e-9
            and abs(limits.upper - BUTTON_TRAVEL) <= 1e-9,
            f"expected travel [0.0, {BUTTON_TRAVEL:.4f}], got {limits}",
        )

        ctx.expect_contact(hood_body, button, name=f"{name}_contact_at_rest")
        button_aabb = ctx.part_world_aabb(button)
        hood_aabb = ctx.part_world_aabb(hood_body)
        if button_aabb is not None and hood_aabb is not None:
            proud = button_aabb[1][1] - hood_aabb[1][1]
            ctx.check(
                f"{name}_rests_proud_of_front_face",
                0.005 <= proud <= 0.0085,
                f"expected proud amount in [0.0050, 0.0085], got {proud:.4f}",
            )

        if limits is not None and limits.upper is not None and limits.lower is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_lower_no_floating")
                ctx.expect_contact(hood_body, button, name=f"{name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_upper_no_floating")
                ctx.expect_contact(hood_body, button, name=f"{name}_upper_contact")
                pressed_button_aabb = ctx.part_world_aabb(button)
                pressed_hood_aabb = ctx.part_world_aabb(hood_body)
                if pressed_button_aabb is not None and pressed_hood_aabb is not None:
                    pressed_proud = pressed_button_aabb[1][1] - pressed_hood_aabb[1][1]
                    ctx.check(
                        f"{name}_pressed_still_visible",
                        0.001 <= pressed_proud <= 0.0055,
                        f"expected pressed proud amount in [0.0010, 0.0055], got {pressed_proud:.4f}",
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
