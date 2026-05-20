from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.392
BODY_DEPTH = 0.278
BASE_FLOOR_THICKNESS = 0.020
UPPER_FRAME_THICKNESS = 0.018
BASE_TOP_Z = BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS

HINGE_BARREL_LENGTH = 0.218
HINGE_BARREL_RADIUS = 0.010
HINGE_AXIS_Y = 0.129
HINGE_AXIS_Z = 0.046

KEY_ROWS = 4
KEY_COLS = 11
KEY_PITCH_X = 0.022
KEY_PITCH_Y = 0.019
KEY_START_Y = 0.079
KEY_TRAVEL = 0.0018
KEYCAP_SIZE = (0.0188, 0.0152, 0.0032)
KEY_STEM_SIZE = (0.0060, 0.0060, 0.0030)
KEY_JOINT_Z = 0.0350

SPACEBAR_SIZE = (0.132, 0.016, 0.0032)
SPACEBAR_Y = 0.001

BUTTON_TRAVEL = 0.0016
BUTTON_SIZE = (0.052, 0.010, 0.0040)
BUTTON_STEM_SIZE = (0.010, 0.006, 0.0030)
BUTTON_JOINT_Z = 0.0355
LEFT_BUTTON_X = -0.030
RIGHT_BUTTON_X = 0.030
BUTTON_Y = -0.104
MOUNT_PLATE_THICKNESS = 0.0012


def _key_name(row: int, col: int) -> str:
    return f"key_r{row}_c{col}"


def _key_joint_name(row: int, col: int) -> str:
    return f"base_to_key_r{row}_c{col}"


def _key_center_x(col: int) -> float:
    return (col - (KEY_COLS - 1) / 2.0) * KEY_PITCH_X


def _key_center_y(row: int) -> float:
    return KEY_START_Y - row * KEY_PITCH_Y


def _add_keycap(
    model: ArticulatedObject,
    base,
    *,
    part_name: str,
    joint_name: str,
    center_x: float,
    center_y: float,
    cap_size: tuple[float, float, float],
    cap_material,
    mass: float,
) -> None:
    key_part = model.part(part_name)
    key_part.visual(
        Box(cap_size),
        origin=Origin(xyz=(0.0, 0.0, cap_size[2] * 0.5)),
        material=cap_material,
        name="cap",
    )
    key_part.visual(
        Box(KEY_STEM_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -KEY_STEM_SIZE[2] * 0.5)),
        material=cap_material,
        name="stem",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((cap_size[0], cap_size[1], cap_size[2] + KEY_STEM_SIZE[2])),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
    )
    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=base,
        child=key_part,
        origin=Origin(xyz=(center_x, center_y, KEY_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_workstation_laptop")

    chassis_metal = model.material("chassis_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    lid_metal = model.material("lid_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    accent_black = model.material("accent_black", rgba=(0.10, 0.11, 0.12, 1.0))
    keyboard_black = model.material("keyboard_black", rgba=(0.12, 0.13, 0.14, 1.0))
    keycap_grey = model.material("keycap_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    glass_black = model.material("glass_black", rgba=(0.07, 0.08, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.12, 0.18, 0.60))
    hinge_metal = model.material("hinge_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    button_black = model.material("button_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BASE_FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_FLOOR_THICKNESS * 0.5)),
        material=chassis_metal,
        name="bottom_pan",
    )
    base.visual(
        Box((0.018, BODY_DEPTH, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(-0.187, 0.0, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="left_side_rail",
    )
    base.visual(
        Box((0.018, BODY_DEPTH, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.187, 0.0, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="right_side_rail",
    )
    base.visual(
        Box((0.356, 0.040, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.119, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="rear_rail",
    )
    base.visual(
        Box((0.332, 0.014, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.009, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="keyboard_front_bridge",
    )
    base.visual(
        Box((0.120, 0.066, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.049, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="touchpad_support",
    )
    base.visual(
        Box((0.104, 0.106, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(-0.101, -0.086, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="left_palmrest",
    )
    base.visual(
        Box((0.104, 0.106, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.101, -0.086, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="right_palmrest",
    )
    base.visual(
        Box((0.010, 0.046, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.108, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="button_center_divider",
    )
    base.visual(
        Box((0.132, 0.014, UPPER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.131, BASE_FLOOR_THICKNESS + UPPER_FRAME_THICKNESS * 0.5)),
        material=chassis_metal,
        name="front_center_rail",
    )
    base.visual(
        Box((0.312, 0.118, 0.001)),
        origin=Origin(xyz=(0.0, 0.046, BASE_FLOOR_THICKNESS + 0.0005)),
        material=accent_black,
        name="keyboard_well_floor",
    )
    base.visual(
        Box((0.126, 0.036, 0.001)),
        origin=Origin(xyz=(0.0, -0.104, BASE_FLOOR_THICKNESS + 0.0005)),
        material=accent_black,
        name="button_well_floor",
    )
    base.visual(
        Box((0.110, 0.070, 0.001)),
        origin=Origin(xyz=(0.0, -0.049, BASE_TOP_Z + 0.0005)),
        material=glass_black,
        name="touchpad_panel",
    )
    base.visual(
        Box((0.252, 0.084, MOUNT_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.050, KEY_JOINT_Z - MOUNT_PLATE_THICKNESS * 0.5)),
        material=keyboard_black,
        name="keyboard_mount_plate",
    )
    base.visual(
        Box((0.150, 0.018, MOUNT_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, SPACEBAR_Y, KEY_JOINT_Z - MOUNT_PLATE_THICKNESS * 0.5)),
        material=keyboard_black,
        name="spacebar_mount_plate",
    )
    base.visual(
        Box((0.056, 0.012, MOUNT_PLATE_THICKNESS)),
        origin=Origin(xyz=(LEFT_BUTTON_X, BUTTON_Y, BUTTON_JOINT_Z - MOUNT_PLATE_THICKNESS * 0.5)),
        material=button_black,
        name="left_button_mount_plate",
    )
    base.visual(
        Box((0.056, 0.012, MOUNT_PLATE_THICKNESS)),
        origin=Origin(xyz=(RIGHT_BUTTON_X, BUTTON_Y, BUTTON_JOINT_Z - MOUNT_PLATE_THICKNESS * 0.5)),
        material=button_black,
        name="right_button_mount_plate",
    )
    base.visual(
        Box((0.222, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y - 0.002, 0.029)),
        material=hinge_metal,
        name="hinge_spine",
    )
    base.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="hinge_barrel",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.145, -0.105),
            (0.145, -0.105),
            (-0.145, 0.100),
            (0.145, 0.100),
        )
    ):
        base.visual(
            Box((0.042, 0.012, 0.002)),
            origin=Origin(xyz=(foot_x, foot_y, 0.001)),
            material=accent_black,
            name=f"foot_{foot_index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, 0.048)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    lid = model.part("display_lid")
    lid.visual(
        Box((0.378, 0.246, 0.012)),
        origin=Origin(xyz=(0.0, -0.137, 0.006)),
        material=lid_metal,
        name="lid_panel",
    )
    lid.visual(
        Box((0.336, 0.204, 0.002)),
        origin=Origin(xyz=(0.0, -0.137, 0.001)),
        material=screen_glass,
        name="screen_panel",
    )
    lid.visual(
        Box((0.202, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -0.012, 0.014)),
        material=lid_metal,
        name="hinge_cowl",
    )
    lid.visual(
        Box((0.194, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.018, 0.008)),
        material=lid_metal,
        name="hinge_understrap",
    )
    lid.visual(
        Box((0.070, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.002)),
        material=accent_black,
        name="lid_hinge_inner_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.378, 0.246, 0.014)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.125, 0.008)),
    )

    model.articulation(
        "base_to_display_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.4,
            lower=0.0,
            upper=2.08,
        ),
    )

    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            _add_keycap(
                model,
                base,
                part_name=_key_name(row, col),
                joint_name=_key_joint_name(row, col),
                center_x=_key_center_x(col),
                center_y=_key_center_y(row),
                cap_size=KEYCAP_SIZE,
                cap_material=keycap_grey,
                mass=0.008,
            )

    _add_keycap(
        model,
        base,
        part_name="spacebar",
        joint_name="base_to_spacebar",
        center_x=0.0,
        center_y=SPACEBAR_Y,
        cap_size=SPACEBAR_SIZE,
        cap_material=keyboard_black,
        mass=0.015,
    )

    left_button = model.part("left_touchpad_button")
    left_button.visual(
        Box(BUTTON_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] * 0.5)),
        material=button_black,
        name="button_cap",
    )
    left_button.visual(
        Box(BUTTON_STEM_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_STEM_SIZE[2] * 0.5)),
        material=button_black,
        name="button_stem",
    )
    left_button.inertial = Inertial.from_geometry(
        Box((BUTTON_SIZE[0], BUTTON_SIZE[1], BUTTON_SIZE[2] + BUTTON_STEM_SIZE[2])),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
    )
    model.articulation(
        "base_to_left_touchpad_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_button,
        origin=Origin(xyz=(LEFT_BUTTON_X, BUTTON_Y, BUTTON_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    right_button = model.part("right_touchpad_button")
    right_button.visual(
        Box(BUTTON_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] * 0.5)),
        material=button_black,
        name="button_cap",
    )
    right_button.visual(
        Box(BUTTON_STEM_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_STEM_SIZE[2] * 0.5)),
        material=button_black,
        name="button_stem",
    )
    right_button.inertial = Inertial.from_geometry(
        Box((BUTTON_SIZE[0], BUTTON_SIZE[1], BUTTON_SIZE[2] + BUTTON_STEM_SIZE[2])),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
    )
    model.articulation(
        "base_to_right_touchpad_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_button,
        origin=Origin(xyz=(RIGHT_BUTTON_X, BUTTON_Y, BUTTON_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    lid = object_model.get_part("display_lid")
    hinge = object_model.get_articulation("base_to_display_lid")
    spacebar = object_model.get_part("spacebar")
    spacebar_joint = object_model.get_articulation("base_to_spacebar")
    sample_key = object_model.get_part(_key_name(1, 5))
    sample_key_joint = object_model.get_articulation(_key_joint_name(1, 5))
    left_button = object_model.get_part("left_touchpad_button")
    left_button_joint = object_model.get_articulation("base_to_left_touchpad_button")
    right_button = object_model.get_part("right_touchpad_button")
    right_button_joint = object_model.get_articulation("base_to_right_touchpad_button")

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

    expected_part_names = ["base", "display_lid", "spacebar", "left_touchpad_button", "right_touchpad_button"]
    expected_part_names.extend(_key_name(row, col) for row in range(KEY_ROWS) for col in range(KEY_COLS))
    for part_name in expected_part_names:
        object_model.get_part(part_name)

    key_layout_ok = True
    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            key_part = object_model.get_part(_key_name(row, col))
            key_joint = object_model.get_articulation(_key_joint_name(row, col))
            if key_joint.articulation_type != ArticulationType.PRISMATIC:
                key_layout_ok = False
            if tuple(key_joint.axis) != (0.0, 0.0, -1.0):
                key_layout_ok = False
            limits = key_joint.motion_limits
            if limits is None or limits.lower != 0.0 or limits.upper != KEY_TRAVEL:
                key_layout_ok = False
            key_pos = ctx.part_world_position(key_part)
            if key_pos is None:
                key_layout_ok = False
            else:
                if not (-0.125 <= key_pos[0] <= 0.125 and 0.010 <= key_pos[1] <= 0.090):
                    key_layout_ok = False
            ctx.expect_overlap(
                key_part,
                base,
                axes="xy",
                elem_b="keyboard_well_floor",
                min_overlap=0.010,
                name=f"{key_part.name}_within_keyboard_deck",
            )
            ctx.expect_contact(
                key_part,
                base,
                elem_a="cap",
                elem_b="keyboard_mount_plate",
                contact_tol=0.0002,
                name=f"{key_part.name}_rests_on_keyboard_mount_plate",
            )
    ctx.check("keyboard_key_grid_and_travel", key_layout_ok, "Keyboard keycaps are misplaced or use incorrect prismatic travel.")

    ctx.expect_contact(
        spacebar,
        base,
        elem_a="cap",
        elem_b="spacebar_mount_plate",
        contact_tol=0.0002,
        name="spacebar_rests_on_mount_plate",
    )

    for button_part, button_joint in (
        (left_button, left_button_joint),
        (right_button, right_button_joint),
    ):
        limits = button_joint.motion_limits
        button_ok = (
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(button_joint.axis) == (0.0, 0.0, -1.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == BUTTON_TRAVEL
        )
        ctx.check(
            f"{button_part.name}_joint_definition",
            button_ok,
            "Touchpad button should be a short vertical prismatic plunger.",
        )
        ctx.expect_overlap(
            button_part,
            base,
            axes="xy",
            elem_b="button_well_floor",
            min_overlap=0.008,
            name=f"{button_part.name}_within_button_bar_well",
        )
        ctx.expect_contact(
            button_part,
            base,
            elem_a="button_cap",
            elem_b="left_button_mount_plate" if button_part.name == "left_touchpad_button" else "right_button_mount_plate",
            contact_tol=0.0002,
            name=f"{button_part.name}_rests_on_mount_plate",
        )

    ctx.expect_overlap(lid, base, axes="x", elem_a="lid_panel", elem_b="bottom_pan", min_overlap=0.330)
    ctx.expect_overlap(lid, base, axes="y", elem_a="lid_panel", elem_b="bottom_pan", min_overlap=0.220)
    ctx.expect_overlap(lid, base, axes="x", elem_a="hinge_cowl", elem_b="hinge_barrel", min_overlap=0.180)
    ctx.expect_contact(
        lid,
        base,
        elem_a="hinge_cowl",
        elem_b="hinge_barrel",
        contact_tol=0.0012,
        name="closed_hinge_cowl_tracks_barrel",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="screen_panel",
        negative_elem="touchpad_panel",
        min_gap=0.004,
        max_gap=0.020,
        name="closed_screen_clears_touchpad_deck",
    )

    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    open_screen = None
    with ctx.pose({hinge: math.radians(110)}):
        open_screen = ctx.part_element_world_aabb(lid, elem="screen_panel")
        ctx.expect_contact(
            lid,
            base,
            elem_a="hinge_cowl",
            elem_b="hinge_barrel",
            contact_tol=0.0012,
            name="open_hinge_cowl_tracks_barrel",
        )
    assert closed_lid_panel is not None
    assert open_screen is not None
    assert open_screen[1][2] > closed_lid_panel[1][2] + 0.160

    sample_key_rest = ctx.part_world_position(sample_key)
    spacebar_rest = ctx.part_world_position(spacebar)
    left_button_rest = ctx.part_world_position(left_button)
    right_button_rest = ctx.part_world_position(right_button)
    assert sample_key_rest is not None
    assert spacebar_rest is not None
    assert left_button_rest is not None
    assert right_button_rest is not None
    with ctx.pose(
        {
            sample_key_joint: KEY_TRAVEL,
            spacebar_joint: KEY_TRAVEL,
            left_button_joint: BUTTON_TRAVEL,
            right_button_joint: BUTTON_TRAVEL,
        }
    ):
        sample_key_pressed = ctx.part_world_position(sample_key)
        spacebar_pressed = ctx.part_world_position(spacebar)
        left_button_pressed = ctx.part_world_position(left_button)
        right_button_pressed = ctx.part_world_position(right_button)
        assert sample_key_pressed is not None
        assert spacebar_pressed is not None
        assert left_button_pressed is not None
        assert right_button_pressed is not None
        assert sample_key_pressed[2] < sample_key_rest[2] - 0.0012
        assert spacebar_pressed[2] < spacebar_rest[2] - 0.0012
        assert left_button_pressed[2] < left_button_rest[2] - 0.0010
        assert right_button_pressed[2] < right_button_rest[2] - 0.0010

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
