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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.20
TOP_DECK_WIDTH = 0.36
TOP_DECK_DEPTH = 0.22
TOP_DECK_CENTER_Y = -0.02
TOP_DECK_THICKNESS = 0.012
SHELL_THICKNESS = 0.012

BUTTON_ROW_Z = 0.075
FRONT_PANEL_Y = CANOPY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5
CONTROL_PANEL_WIDTH = 0.34
CONTROL_PANEL_HEIGHT = 0.05
CONTROL_PANEL_THICKNESS = SHELL_THICKNESS
CONTROL_PANEL_CENTER_Y = FRONT_PANEL_Y
CONTROL_PANEL_CENTER_Z = BUTTON_ROW_Z
FRONT_LOWER_LIP_HEIGHT = CONTROL_PANEL_CENTER_Z - CONTROL_PANEL_HEIGHT * 0.5
FRONT_UPPER_LIP_HEIGHT = 0.11 - (CONTROL_PANEL_CENTER_Z + CONTROL_PANEL_HEIGHT * 0.5)
FRONT_SIDE_FILLER_WIDTH = (CANOPY_WIDTH - CONTROL_PANEL_WIDTH) * 0.5

CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.24
CHIMNEY_HEIGHT = 0.62
CHIMNEY_SHELL_THICKNESS = 0.003

BUTTON_HOLE_RADIUS = 0.009
BUTTON_RADIUS = 0.007
BUTTON_FACE_LENGTH = 0.004
BUTTON_STEM_RADIUS = 0.0045
BUTTON_STEM_LENGTH = CONTROL_PANEL_THICKNESS
BUTTON_PLUNGER_LENGTH = 0.022
BUTTON_PLUNGER_SIZE = 0.024
BUTTON_TRAVEL = 0.008
BUTTON_X_POSITIONS = (-0.105, -0.035, 0.035, 0.105)


def _rectangle_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_control_panel_mesh():
    panel = ExtrudeWithHolesGeometry(
        _rectangle_profile(CONTROL_PANEL_WIDTH, CONTROL_PANEL_HEIGHT),
        [_circle_profile(BUTTON_HOLE_RADIUS, center=(button_x, 0.0)) for button_x in BUTTON_X_POSITIONS],
        height=CONTROL_PANEL_THICKNESS,
        center=True,
    )
    panel.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(panel, ASSETS.mesh_path("range_hood_control_panel.obj"))


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple(max_corner[index] - min_corner[index] for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_style_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_trim = model.material("brushed_trim", rgba=(0.66, 0.68, 0.71, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.10, 0.11, 1.0))

    canopy = model.part("canopy")
    canopy.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
    )
    canopy.visual(
        Box((TOP_DECK_WIDTH, TOP_DECK_DEPTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(0.0, TOP_DECK_CENTER_Y, CANOPY_HEIGHT - TOP_DECK_THICKNESS * 0.5)
        ),
        material=stainless,
        name="top_deck",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, CANOPY_HEIGHT - TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -CANOPY_DEPTH * 0.5 + SHELL_THICKNESS * 0.5,
                (CANOPY_HEIGHT - TOP_DECK_THICKNESS) * 0.5,
            )
        ),
        material=stainless,
        name="back_shell",
    )

    side_top_x = TOP_DECK_WIDTH * 0.5 + 0.002
    side_bottom_x = CANOPY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5
    side_rise = CANOPY_HEIGHT - TOP_DECK_THICKNESS
    side_length = math.hypot(side_bottom_x - side_top_x, side_rise)
    side_angle = math.atan2(side_bottom_x - side_top_x, side_rise)
    side_center_x = (side_top_x + side_bottom_x) * 0.5
    side_center_z = side_rise * 0.5
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, side_length)),
        origin=Origin(
            xyz=(side_center_x, 0.0, side_center_z),
            rpy=(0.0, -side_angle, 0.0),
        ),
        material=stainless,
        name="left_flare",
    )
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, side_length)),
        origin=Origin(
            xyz=(-side_center_x, 0.0, side_center_z),
            rpy=(0.0, side_angle, 0.0),
        ),
        material=stainless,
        name="right_flare",
    )

    front_lip_height = 0.11
    front_lip_center_y = FRONT_PANEL_Y
    front_lip_center_z = front_lip_height * 0.5
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, FRONT_LOWER_LIP_HEIGHT)),
        origin=Origin(xyz=(0.0, front_lip_center_y, FRONT_LOWER_LIP_HEIGHT * 0.5)),
        material=stainless,
        name="front_lower_lip",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, FRONT_UPPER_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                front_lip_center_y,
                CONTROL_PANEL_CENTER_Z + CONTROL_PANEL_HEIGHT * 0.5 + FRONT_UPPER_LIP_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_upper_lip",
    )
    canopy.visual(
        Box((FRONT_SIDE_FILLER_WIDTH, SHELL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                -CONTROL_PANEL_WIDTH * 0.5 - FRONT_SIDE_FILLER_WIDTH * 0.5,
                front_lip_center_y,
                CONTROL_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="front_left_filler",
    )
    canopy.visual(
        Box((FRONT_SIDE_FILLER_WIDTH, SHELL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_WIDTH * 0.5 + FRONT_SIDE_FILLER_WIDTH * 0.5,
                front_lip_center_y,
                CONTROL_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="front_right_filler",
    )

    front_top_y = TOP_DECK_CENTER_Y + TOP_DECK_DEPTH * 0.5
    front_top_z = CANOPY_HEIGHT - TOP_DECK_THICKNESS * 0.5
    front_bottom_y = front_lip_center_y - SHELL_THICKNESS * 0.5
    front_bottom_z = front_lip_height
    front_slope_length = math.hypot(
        front_bottom_y - front_top_y,
        front_top_z - front_bottom_z,
    )
    front_slope_angle = math.atan2(
        front_bottom_y - front_top_y,
        front_top_z - front_bottom_z,
    )
    canopy.visual(
        Box((0.76, SHELL_THICKNESS, front_slope_length)),
        origin=Origin(
            xyz=(
                0.0,
                (front_top_y + front_bottom_y) * 0.5,
                (front_top_z + front_bottom_z) * 0.5,
            ),
            rpy=(front_slope_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="front_slope",
    )

    control_panel_mesh = _build_control_panel_mesh()
    canopy.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.0, CONTROL_PANEL_CENTER_Y, BUTTON_ROW_Z)),
        material=brushed_trim,
        name="control_panel",
    )

    chimney_cover = model.part("chimney_cover")
    chimney_cover.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, CHIMNEY_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT * 0.5)),
    )
    chimney_cover.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_SHELL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CHIMNEY_DEPTH * 0.5 - CHIMNEY_SHELL_THICKNESS * 0.5, CHIMNEY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="front_shell",
    )
    chimney_cover.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_SHELL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CHIMNEY_DEPTH * 0.5 + CHIMNEY_SHELL_THICKNESS * 0.5, CHIMNEY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="rear_shell",
    )
    chimney_cover.visual(
        Box((CHIMNEY_SHELL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(CHIMNEY_WIDTH * 0.5 - CHIMNEY_SHELL_THICKNESS * 0.5, 0.0, CHIMNEY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="left_shell",
    )
    chimney_cover.visual(
        Box((CHIMNEY_SHELL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(-CHIMNEY_WIDTH * 0.5 + CHIMNEY_SHELL_THICKNESS * 0.5, 0.0, CHIMNEY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="right_shell",
    )
    chimney_cover.visual(
        Box((0.34, 0.26, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT - 0.004)),
        material=brushed_trim,
        name="top_cap",
    )

    model.articulation(
        "canopy_to_chimney_cover",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, TOP_DECK_CENTER_Y, CANOPY_HEIGHT)),
    )

    button_visual_rpy = (-math.pi / 2.0, 0.0, 0.0)
    button_front_face_y = CONTROL_PANEL_CENTER_Y + CONTROL_PANEL_THICKNESS * 0.5

    for index, button_x in enumerate(BUTTON_X_POSITIONS, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_PLUNGER_LENGTH - BUTTON_FACE_LENGTH),
            origin=Origin(
                xyz=(0.0, -(BUTTON_PLUNGER_LENGTH - BUTTON_FACE_LENGTH) * 0.5, 0.0),
                rpy=button_visual_rpy,
            ),
            material=button_black,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_FACE_LENGTH),
            origin=Origin(
                xyz=(0.0, BUTTON_FACE_LENGTH * 0.5, 0.0),
                rpy=button_visual_rpy,
            ),
            material=button_black,
            name="button_face",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_RADIUS * 2.0, BUTTON_PLUNGER_LENGTH, BUTTON_RADIUS * 2.0)),
            mass=0.05,
            origin=Origin(xyz=(0.0, -0.5 * (BUTTON_PLUNGER_LENGTH - BUTTON_FACE_LENGTH) + BUTTON_FACE_LENGTH * 0.5, 0.0)),
        )
        model.articulation(
            f"canopy_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(button_x, button_front_face_y, BUTTON_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy")
    chimney_cover = object_model.get_part("chimney_cover")
    control_panel = canopy.get_visual("control_panel")
    top_deck = canopy.get_visual("top_deck")
    chimney_mount = object_model.get_articulation("canopy_to_chimney_cover")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 5)]
    button_joints = [
        object_model.get_articulation(f"canopy_to_button_{index}") for index in range(1, 5)
    ]
    for button in buttons:
        ctx.allow_isolated_part(
            button,
            reason=(
                "Captive push-button plunger rides inside the concealed switch body "
                "with running clearance to the panel bushing, so support is not "
                "represented as literal contact geometry."
            ),
        )

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

    canopy_size = _aabb_size(ctx.part_world_aabb(canopy))
    chimney_size = _aabb_size(ctx.part_world_aabb(chimney_cover))
    ctx.check(
        "canopy_realistic_size",
        canopy_size is not None
        and 0.88 <= canopy_size[0] <= 0.92
        and 0.49 <= canopy_size[1] <= 0.51
        and 0.19 <= canopy_size[2] <= 0.21,
        f"unexpected canopy size: {canopy_size}",
    )
    ctx.check(
        "chimney_realistic_size",
        chimney_size is not None
        and 0.31 <= chimney_size[0] <= 0.35
        and 0.23 <= chimney_size[1] <= 0.27
        and 0.61 <= chimney_size[2] <= 0.63,
        f"unexpected chimney size: {chimney_size}",
    )
    ctx.check(
        "chimney_mount_is_fixed",
        chimney_mount.articulation_type == ArticulationType.FIXED,
        f"chimney mount should be fixed, got {chimney_mount.articulation_type}",
    )

    ctx.expect_gap(
        chimney_cover,
        canopy,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem=top_deck,
        name="chimney_seated_on_top_deck",
    )
    ctx.expect_overlap(
        chimney_cover,
        canopy,
        axes="xy",
        min_overlap=0.20,
        elem_b=top_deck,
        name="chimney_aligned_over_top_deck",
    )

    control_panel_aabb = ctx.part_element_world_aabb(canopy, elem=control_panel)
    button_positions = [ctx.part_world_position(button) for button in buttons]
    row_spacing_ok = (
        all(position is not None for position in button_positions)
        and all(
            abs(button_positions[index + 1][0] - button_positions[index][0] - 0.07) <= 1e-4
            for index in range(3)
        )
        and max(abs(position[1] - button_positions[0][1]) for position in button_positions) <= 1e-6
        and max(abs(position[2] - button_positions[0][2]) for position in button_positions) <= 1e-6
    )
    ctx.check(
        "buttons_in_straight_even_row",
        row_spacing_ok,
        f"button positions were {button_positions}",
    )

    for index, (button, joint, button_position) in enumerate(
        zip(buttons, button_joints, button_positions, strict=True),
        start=1,
    ):
        limits = joint.motion_limits
        axis_ok = (
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == BUTTON_TRAVEL
        )
        ctx.check(
            f"button_{index}_joint_definition",
            axis_ok,
            (
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )
        button_face_aabb = ctx.part_element_world_aabb(button, elem="button_face")
        face_proud = (
            button_face_aabb is not None
            and control_panel_aabb is not None
            and button_face_aabb[0][1] >= control_panel_aabb[1][1] - 1e-6
            and button_face_aabb[1][1] >= control_panel_aabb[1][1] + 0.0035
        )
        ctx.check(
            f"button_{index}_rests_proud_of_control_panel",
            face_proud,
            f"button face aabb {button_face_aabb} vs control panel aabb {control_panel_aabb}",
        )
        within_panel = (
            control_panel_aabb is not None
            and button_position is not None
            and control_panel_aabb[0][0] + BUTTON_HOLE_RADIUS < button_position[0] < control_panel_aabb[1][0] - BUTTON_HOLE_RADIUS
            and control_panel_aabb[0][2] + BUTTON_HOLE_RADIUS < button_position[2] < control_panel_aabb[1][2] - BUTTON_HOLE_RADIUS
        )
        ctx.check(
            f"button_{index}_centered_in_control_band",
            within_panel,
            f"button position {button_position} not within control panel bounds {control_panel_aabb}",
        )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"button_{index}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"button_{index}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"button_{index}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"button_{index}_upper_no_floating")
                pressed_position = ctx.part_world_position(button)
                pressed_correctly = (
                    button_position is not None
                    and pressed_position is not None
                    and abs(pressed_position[0] - button_position[0]) <= 1e-6
                    and abs(pressed_position[2] - button_position[2]) <= 1e-6
                    and abs((button_position[1] - pressed_position[1]) - limits.upper) <= 1e-4
                )
                ctx.check(
                    f"button_{index}_presses_normal_to_panel",
                    pressed_correctly,
                    f"rest={button_position}, pressed={pressed_position}, expected travel={limits.upper}",
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
