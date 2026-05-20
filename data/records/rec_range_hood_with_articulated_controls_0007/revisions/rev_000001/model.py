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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _rectangle_profile(
    width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)
) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _trapezoid_profile(
    bottom_width: float,
    top_width: float,
    height: float,
) -> list[tuple[float, float]]:
    half_bottom = bottom_width * 0.5
    half_top = top_width * 0.5
    half_height = height * 0.5
    return [
        (-half_bottom, -half_height),
        (half_bottom, -half_height),
        (half_top, half_height),
        (-half_top, half_height),
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    stainless_dark = model.material("stainless_dark", rgba=(0.60, 0.62, 0.66, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    button_black = model.material("button_black", rgba=(0.09, 0.10, 0.11, 1.0))

    canopy_width_bottom = 0.90
    canopy_depth_bottom = 0.50
    canopy_width_top = 0.38
    canopy_depth_top = 0.30
    canopy_top_center_y = -0.03
    canopy_height = 0.24
    shell_thickness = 0.012

    front_edge_y = canopy_depth_bottom * 0.5
    bottom_front_y = 0.20
    bottom_back_y = -canopy_depth_bottom * 0.5
    top_front_y = canopy_top_center_y + canopy_depth_top * 0.5
    top_back_y = canopy_top_center_y - canopy_depth_top * 0.5

    front_slant = math.hypot(bottom_front_y - top_front_y, canopy_height)
    back_slant = math.hypot(top_back_y - bottom_back_y, canopy_height)
    side_slant = math.hypot(
        (canopy_width_bottom - canopy_width_top) * 0.5,
        canopy_height,
    )
    side_tilt = math.atan2(
        (canopy_width_bottom - canopy_width_top) * 0.5,
        canopy_height,
    )

    hood_body = model.part("hood_body")

    front_panel_mesh = _save_mesh(
        "canopy_front_panel.obj",
        ExtrudeGeometry(
            _trapezoid_profile(canopy_width_bottom, canopy_width_top, front_slant),
            shell_thickness,
        ),
    )
    hood_body.visual(
        front_panel_mesh,
        origin=Origin(
            xyz=(0.0, 0.5 * (bottom_front_y + top_front_y), 0.5 * canopy_height),
            rpy=(
                math.pi * 0.5 + math.atan2(bottom_front_y - top_front_y, canopy_height),
                0.0,
                0.0,
            ),
        ),
        material=stainless,
        name="canopy_front",
    )

    back_panel_mesh = _save_mesh(
        "canopy_back_panel.obj",
        ExtrudeGeometry(
            _trapezoid_profile(canopy_width_bottom, canopy_width_top, back_slant),
            shell_thickness,
        ),
    )
    hood_body.visual(
        back_panel_mesh,
        origin=Origin(
            xyz=(0.0, 0.5 * (bottom_back_y + top_back_y), 0.5 * canopy_height),
            rpy=(
                math.pi * 0.5 + math.atan2(bottom_back_y - top_back_y, canopy_height),
                0.0,
                0.0,
            ),
        ),
        material=stainless,
        name="canopy_back",
    )

    side_panel_depth = 0.48
    side_panel_center_y = canopy_top_center_y * 0.5
    side_panel_center_x = 0.25 * (canopy_width_bottom + canopy_width_top)
    hood_body.visual(
        Box((shell_thickness, side_panel_depth, side_slant)),
        origin=Origin(
            xyz=(side_panel_center_x, side_panel_center_y, 0.5 * canopy_height),
            rpy=(0.0, -side_tilt, 0.0),
        ),
        material=stainless,
        name="canopy_right",
    )
    hood_body.visual(
        Box((shell_thickness, side_panel_depth, side_slant)),
        origin=Origin(
            xyz=(-side_panel_center_x, side_panel_center_y, 0.5 * canopy_height),
            rpy=(0.0, side_tilt, 0.0),
        ),
        material=stainless,
        name="canopy_left",
    )

    deck_outer_width = 0.44
    deck_outer_depth = 0.32
    deck_hole_width = 0.22
    deck_hole_depth = 0.16
    deck_thickness = 0.012
    deck_mesh = _save_mesh(
        "canopy_top_deck.obj",
        ExtrudeWithHolesGeometry(
            _rectangle_profile(deck_outer_width, deck_outer_depth),
            [_rectangle_profile(deck_hole_width, deck_hole_depth)],
            deck_thickness,
        ),
    )
    hood_body.visual(
        deck_mesh,
        origin=Origin(
            xyz=(0.0, canopy_top_center_y, canopy_height + 0.5 * deck_thickness),
        ),
        material=stainless,
        name="top_deck",
    )

    hood_body.visual(
        Box((0.33, 0.427, 0.005)),
        origin=Origin(xyz=(-0.18, -0.0215, 0.018)),
        material=charcoal,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.33, 0.427, 0.005)),
        origin=Origin(xyz=(0.18, -0.0215, 0.018)),
        material=charcoal,
        name="right_filter",
    )

    button_frame_width = 0.84
    button_frame_thickness = 0.006
    button_opening_height = 0.014
    button_rail_height = 0.006
    button_side_rail_width = 0.006
    button_width = 0.026
    button_depth = 0.010
    button_height = 0.014
    button_travel = 0.005
    button_x_positions = (-0.31, -0.155, 0.0, 0.155, 0.31)
    button_panel_center_y = front_edge_y - 0.5 * button_frame_thickness
    button_panel_center_z = 0.028

    hood_body.visual(
        Box((button_frame_width, button_frame_thickness, button_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                button_panel_center_y,
                button_panel_center_z + 0.5 * (button_opening_height + button_rail_height),
            )
        ),
        material=stainless_dark,
        name="control_top_rail",
    )
    hood_body.visual(
        Box((button_frame_width, button_frame_thickness, button_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                button_panel_center_y,
                button_panel_center_z - 0.5 * (button_opening_height + button_rail_height),
            )
        ),
        material=stainless_dark,
        name="control_bottom_rail",
    )
    hood_body.visual(
        Box(
            (
                button_side_rail_width,
                button_frame_thickness,
                button_opening_height + 2.0 * button_rail_height,
            )
        ),
        origin=Origin(
            xyz=(
                0.5 * (button_frame_width - button_side_rail_width),
                button_panel_center_y,
                button_panel_center_z,
            )
        ),
        material=stainless_dark,
        name="control_right_rail",
    )
    hood_body.visual(
        Box(
            (
                button_side_rail_width,
                button_frame_thickness,
                button_opening_height + 2.0 * button_rail_height,
            )
        ),
        origin=Origin(
            xyz=(
                -0.5 * (button_frame_width - button_side_rail_width),
                button_panel_center_y,
                button_panel_center_z,
            )
        ),
        material=stainless_dark,
        name="control_left_rail",
    )
    hood_body.visual(
        Box((0.012, 0.046, button_opening_height + 2.0 * button_rail_height)),
        origin=Origin(
            xyz=(
                0.408,
                0.223,
                button_panel_center_z,
            )
        ),
        material=stainless_dark,
        name="control_right_bridge",
    )
    hood_body.visual(
        Box((0.012, 0.046, button_opening_height + 2.0 * button_rail_height)),
        origin=Origin(
            xyz=(
                -0.408,
                0.223,
                button_panel_center_z,
            )
        ),
        material=stainless_dark,
        name="control_left_bridge",
    )

    chimney_width = 0.32
    chimney_depth = 0.27
    chimney_height = 0.60
    chimney_wall = 0.010
    chimney_center_y = -0.03
    chimney_base_z = canopy_height + deck_thickness
    chimney_center_z = chimney_base_z + 0.5 * chimney_height

    hood_body.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + 0.5 * chimney_depth - 0.5 * chimney_wall,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - 0.5 * chimney_depth + 0.5 * chimney_wall,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_wall, chimney_depth - chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(0.5 * chimney_width - 0.5 * chimney_wall, chimney_center_y, chimney_center_z)
        ),
        material=stainless,
        name="chimney_right",
    )
    hood_body.visual(
        Box((chimney_wall, chimney_depth - chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(-0.5 * chimney_width + 0.5 * chimney_wall, chimney_center_y, chimney_center_z)
        ),
        material=stainless,
        name="chimney_left",
    )

    hood_body.inertial = Inertial.from_geometry(
        Box((0.90, 0.50, canopy_height + deck_thickness + chimney_height)),
        mass=18.0,
        origin=Origin(
            xyz=(0.0, 0.0, 0.5 * (canopy_height + deck_thickness + chimney_height))
        ),
    )

    button_rest_y = front_edge_y - 0.001
    for index, button_x in enumerate(button_x_positions, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((button_width, button_depth, button_height)),
            material=button_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_width, button_depth, button_height)),
            mass=0.03,
            origin=Origin(),
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(button_x, button_rest_y, button_panel_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    joints = [object_model.get_articulation(f"hood_to_button_{index}") for index in range(1, 6)]

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

    ctx.check(
        "only_five_articulations",
        len(getattr(object_model, "articulations", ())) == 5,
        "The range hood should only articulate its five push-buttons.",
    )

    body_aabb = ctx.part_world_aabb(hood_body)
    assert body_aabb is not None
    body_width = body_aabb[1][0] - body_aabb[0][0]
    body_depth = body_aabb[1][1] - body_aabb[0][1]
    body_height = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "range_hood_width_realistic",
        0.84 <= body_width <= 0.95,
        f"Expected approximately 0.9 m width, got {body_width:.3f} m.",
    )
    ctx.check(
        "range_hood_depth_realistic",
        0.48 <= body_depth <= 0.55,
        f"Expected approximately 0.5 m depth, got {body_depth:.3f} m.",
    )
    ctx.check(
        "range_hood_height_realistic",
        0.82 <= body_height <= 0.88,
        f"Expected approximately 0.85 m total height, got {body_height:.3f} m.",
    )

    all_pressed_pose: dict[object, float] = {}
    for index, (button, joint) in enumerate(zip(buttons, joints), start=1):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.upper is not None
        all_pressed_pose[joint] = limits.upper

        ctx.check(
            f"button_{index}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            "Each control must be a short-travel push-button prismatic joint.",
        )
        ctx.check(
            f"button_{index}_axis_faces_inward",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"Expected inward axis (0, -1, 0), got {joint.axis}.",
        )
        ctx.check(
            f"button_{index}_travel_short",
            0.003 <= limits.upper <= 0.007,
            f"Expected short button travel, got {limits.upper}.",
        )

        ctx.expect_contact(button, hood_body, name=f"button_{index}_mounted_at_rest")
        ctx.expect_within(
            button,
            hood_body,
            axes="xz",
            inner_elem="button_cap",
            name=f"button_{index}_within_button_panel",
        )

        rest_position = ctx.part_world_position(button)
        assert rest_position is not None

        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.expect_contact(button, hood_body, name=f"button_{index}_mounted_pressed")
            ctx.expect_within(
                button,
                hood_body,
                axes="xz",
                inner_elem="button_cap",
                name=f"button_{index}_pressed_within_button_panel",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"button_{index}_pressed_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"button_{index}_pressed_no_floating")
            ctx.check(
                f"button_{index}_moves_inward_only",
                pressed_position[1] < rest_position[1] - 0.004
                and abs(pressed_position[0] - rest_position[0]) <= 1e-6
                and abs(pressed_position[2] - rest_position[2]) <= 1e-6,
                (
                    "Buttons should move only inward along the front-panel normal. "
                    f"Rest={rest_position}, pressed={pressed_position}."
                ),
            )

    with ctx.pose(all_pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="all_buttons_pressed_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
