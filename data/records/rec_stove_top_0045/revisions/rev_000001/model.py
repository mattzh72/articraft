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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.60
BODY_DEPTH = 0.62
BODY_HEIGHT = 0.90
SIDE_THICKNESS = 0.02
BACK_THICKNESS = 0.02
TOP_THICKNESS = 0.02
FRONT_FRAME_DEPTH = 0.04

BUTTON_X_CENTERS = (-0.20, -0.10, 0.0, 0.10, 0.20)
BUTTON_WIDTH = 0.048
BUTTON_HEIGHT = 0.020
BUTTON_CAP_DEPTH = 0.012
BUTTON_STEM_DEPTH = 0.018
BUTTON_TRAVEL = 0.012
BUTTON_Z = 0.81
CONTROL_OPENING_HALF_WIDTH = 0.28

BURNER_POSITIONS = (
    (-0.15, -0.16),
    (0.15, -0.16),
    (-0.15, 0.14),
    (0.15, 0.14),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_stove", assets=ASSETS)

    enamel = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_black = model.material("burner_black", rgba=(0.10, 0.10, 0.11, 1.0))
    oven_black = model.material("oven_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_silver = model.material("button_silver", rgba=(0.80, 0.82, 0.84, 1.0))

    cabinet = model.part("cabinet")

    def add_box(part, name, size, xyz, material):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    # Main freestanding body shell and front frame.
    add_box(
        cabinet,
        "left_side",
        (SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT),
        (-(BODY_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0),
        enamel,
    )
    add_box(
        cabinet,
        "right_side",
        (SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT),
        ((BODY_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0),
        enamel,
    )
    add_box(
        cabinet,
        "back_panel",
        (BODY_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, BODY_HEIGHT),
        (0.0, BODY_DEPTH / 2.0 - BACK_THICKNESS / 2.0, BODY_HEIGHT / 2.0),
        enamel,
    )
    add_box(
        cabinet,
        "top_deck",
        (BODY_WIDTH, BODY_DEPTH, TOP_THICKNESS),
        (0.0, 0.0, BODY_HEIGHT - TOP_THICKNESS / 2.0),
        dark_trim,
    )
    add_box(
        cabinet,
        "floor_panel",
        (BODY_WIDTH - 2.0 * SIDE_THICKNESS, BODY_DEPTH - 0.02, 0.02),
        (0.0, -0.01, 0.01),
        enamel,
    )
    add_box(
        cabinet,
        "front_left_stile",
        (0.03, FRONT_FRAME_DEPTH, 0.76),
        (-0.285, -(BODY_DEPTH / 2.0 - FRONT_FRAME_DEPTH / 2.0), 0.38),
        enamel,
    )
    add_box(
        cabinet,
        "front_right_stile",
        (0.03, FRONT_FRAME_DEPTH, 0.76),
        (0.285, -(BODY_DEPTH / 2.0 - FRONT_FRAME_DEPTH / 2.0), 0.38),
        enamel,
    )
    add_box(
        cabinet,
        "kick_panel",
        (0.54, FRONT_FRAME_DEPTH, 0.075),
        (0.0, -(BODY_DEPTH / 2.0 - FRONT_FRAME_DEPTH / 2.0), 0.0375),
        enamel,
    )
    add_box(
        cabinet,
        "door_header",
        (0.54, FRONT_FRAME_DEPTH, 0.04),
        (0.0, -(BODY_DEPTH / 2.0 - FRONT_FRAME_DEPTH / 2.0), 0.74),
        enamel,
    )
    add_box(
        cabinet,
        "control_bottom_strip",
        (0.56, 0.03, 0.04),
        (0.0, -(BODY_DEPTH / 2.0 - 0.015), 0.78),
        enamel,
    )
    add_box(
        cabinet,
        "control_top_strip",
        (0.56, 0.03, 0.06),
        (0.0, -(BODY_DEPTH / 2.0 - 0.015), 0.85),
        enamel,
    )

    control_web_intervals = (
        (-CONTROL_OPENING_HALF_WIDTH, -0.224),
        (-0.176, -0.124),
        (-0.076, -0.024),
        (0.024, 0.076),
        (0.124, 0.176),
        (0.224, CONTROL_OPENING_HALF_WIDTH),
    )
    for index, (x0, x1) in enumerate(control_web_intervals, start=1):
        add_box(
            cabinet,
            f"control_web_{index}",
            (x1 - x0, 0.03, BUTTON_HEIGHT),
            ((x0 + x1) / 2.0, -(BODY_DEPTH / 2.0 - 0.015), BUTTON_Z),
            enamel,
        )

    # Lower hinge barrel carried by small brackets above the kick panel.
    cabinet.visual(
        Cylinder(radius=0.006, length=0.40),
        origin=Origin(xyz=(0.0, -0.300, 0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel_center",
    )
    add_box(
        cabinet,
        "hinge_support_left",
        (0.028, 0.012, 0.022),
        (-0.14, -0.300, 0.086),
        dark_trim,
    )
    add_box(
        cabinet,
        "hinge_support_right",
        (0.028, 0.012, 0.022),
        (0.14, -0.300, 0.086),
        dark_trim,
    )

    oven_liner = model.part("oven_liner")
    oven_liner_center = (0.0, 0.02, 0.40)
    liner_width = 0.48
    liner_depth = 0.54
    liner_height = 0.54
    liner_wall = 0.012
    add_box(
        oven_liner,
        "liner_left_wall",
        (liner_wall, liner_depth, liner_height),
        (-(liner_width / 2.0 - liner_wall / 2.0), 0.0, 0.0),
        oven_black,
    )
    add_box(
        oven_liner,
        "liner_right_wall",
        (liner_wall, liner_depth, liner_height),
        ((liner_width / 2.0 - liner_wall / 2.0), 0.0, 0.0),
        oven_black,
    )
    add_box(
        oven_liner,
        "liner_floor",
        (liner_width - 2.0 * liner_wall, liner_depth, liner_wall),
        (0.0, 0.0, -(liner_height / 2.0 - liner_wall / 2.0)),
        oven_black,
    )
    add_box(
        oven_liner,
        "liner_roof",
        (liner_width - 2.0 * liner_wall, liner_depth, liner_wall),
        (0.0, 0.0, (liner_height / 2.0 - liner_wall / 2.0)),
        oven_black,
    )
    add_box(
        oven_liner,
        "liner_back",
        (liner_width - 2.0 * liner_wall, liner_wall, liner_height),
        (0.0, liner_depth / 2.0 - liner_wall / 2.0, 0.0),
        oven_black,
    )
    model.articulation(
        "cabinet_to_oven_liner",
        ArticulationType.FIXED,
        parent=cabinet,
        child=oven_liner,
        origin=Origin(xyz=oven_liner_center),
    )

    oven_door = model.part("oven_door")
    add_box(
        oven_door,
        "door_panel",
        (0.52, 0.022, 0.604),
        (0.0, 0.0, 0.314),
        enamel,
    )
    add_box(
        oven_door,
        "hinge_strap_left",
        (0.03, 0.012, 0.024),
        (-0.225, 0.0, 0.012),
        dark_trim,
    )
    add_box(
        oven_door,
        "hinge_strap_right",
        (0.03, 0.012, 0.024),
        (0.225, 0.0, 0.012),
        dark_trim,
    )
    oven_door.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(-0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_knuckle_left",
    )
    oven_door.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_knuckle_right",
    )
    add_box(
        oven_door,
        "door_handle",
        (0.22, 0.024, 0.022),
        (0.0, -0.023, 0.55),
        dark_trim,
    )
    door_hinge = model.articulation(
        "cabinet_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=oven_door,
        origin=Origin(xyz=(0.0, -0.300, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    for index, (x_pos, y_pos) in enumerate(BURNER_POSITIONS, start=1):
        burner = model.part(f"burner_{index}")
        burner.visual(
            Cylinder(radius=0.085, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=burner_black,
            name="burner_ring",
        )
        burner.visual(
            Cylinder(radius=0.060, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=burner_black,
            name="burner_cap",
        )
        model.articulation(
            f"cabinet_to_burner_{index}",
            ArticulationType.FIXED,
            parent=cabinet,
            child=burner,
            origin=Origin(xyz=(x_pos, y_pos, BODY_HEIGHT)),
        )

    for index, x_pos in enumerate(BUTTON_X_CENTERS, start=1):
        button = model.part(f"button_{index}")
        add_box(
            button,
            "button_cap",
            (BUTTON_WIDTH, BUTTON_CAP_DEPTH, BUTTON_HEIGHT),
            (0.0, -BUTTON_CAP_DEPTH / 2.0, 0.0),
            button_silver,
        )
        add_box(
            button,
            "button_stem",
            (BUTTON_WIDTH, BUTTON_STEM_DEPTH, BUTTON_HEIGHT),
            (0.0, BUTTON_STEM_DEPTH / 2.0, 0.0),
            button_silver,
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, -(BODY_DEPTH / 2.0), BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    oven_liner = object_model.get_part("oven_liner")
    oven_door = object_model.get_part("oven_door")
    burners = [object_model.get_part(f"burner_{index}") for index in range(1, 5)]
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    door_hinge = object_model.get_articulation("cabinet_to_oven_door")
    button_joints = [
        object_model.get_articulation(f"cabinet_to_button_{index}")
        for index in range(1, 6)
    ]

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

    for required_part in (cabinet, oven_liner, oven_door, *burners, *buttons):
        ctx.check(f"{required_part.name}_present", required_part is not None)

    ctx.check(
        "door_hinge_axis_is_horizontal_x",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected oven door axis (1, 0, 0), got {door_hinge.axis}",
    )
    for index, joint in enumerate(button_joints, start=1):
        ctx.check(
            f"button_{index}_axis_is_panel_normal",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"expected button axis (0, 1, 0), got {joint.axis}",
        )

    ctx.expect_contact(oven_liner, cabinet, name="oven_liner_mounted_to_cabinet")
    ctx.expect_contact(oven_door, cabinet, name="oven_door_hinge_connected_at_rest")
    for burner in burners:
        ctx.expect_contact(burner, cabinet, name=f"{burner.name}_mounted_to_cooktop")
    for button in buttons:
        ctx.expect_contact(button, cabinet, name=f"{button.name}_guided_at_rest")

    button_positions = [ctx.part_world_position(button) for button in buttons]
    row_z_ok = all(
        position is not None and abs(position[2] - BUTTON_Z) <= 1e-6
        for position in button_positions
    )
    spacings = [
        button_positions[index + 1][0] - button_positions[index][0]
        for index in range(len(button_positions) - 1)
        if button_positions[index] is not None and button_positions[index + 1] is not None
    ]
    ctx.check(
        "buttons_form_straight_row",
        row_z_ok and all(0.095 <= spacing <= 0.105 for spacing in spacings),
        f"button world positions: {button_positions}",
    )

    door_aabb = ctx.part_world_aabb(oven_door)
    if door_aabb is not None:
        door_width = door_aabb[1][0] - door_aabb[0][0]
        door_height = door_aabb[1][2] - door_aabb[0][2]
        ctx.check(
            "oven_door_dominates_lower_front",
            0.50 <= door_width <= 0.54 and 0.62 <= door_height <= 0.64,
            f"door width={door_width:.3f}, height={door_height:.3f}",
        )
    else:
        ctx.fail("oven_door_aabb_available", "oven door world AABB was unavailable")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(oven_door, cabinet, name="oven_door_connected_closed")
        ctx.expect_overlap(
            oven_door,
            cabinet,
            axes="xz",
            min_overlap=0.50,
            name="oven_door_covers_front_opening",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_closed_no_floating")

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.expect_contact(oven_door, cabinet, name="oven_door_connected_open")
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")

    for index, (button, joint) in enumerate(zip(buttons, button_joints, strict=True), start=1):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"button_{index}_limits_available", "button joint limits were missing")
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(button, cabinet, name=f"button_{index}_connected_unpressed")
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"button_{index}_unpressed_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"button_{index}_unpressed_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(button, cabinet, name=f"button_{index}_connected_pressed")
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"button_{index}_pressed_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"button_{index}_pressed_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
