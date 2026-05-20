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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_microwave")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.74, 0.74, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    black_glass = model.material("black_glass", rgba=(0.10, 0.13, 0.15, 0.42))
    smoked_panel = model.material("smoked_panel", rgba=(0.08, 0.09, 0.10, 0.95))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.78, 0.80, 0.82, 1.0))
    button_grey = model.material("button_grey", rgba=(0.83, 0.84, 0.85, 1.0))

    width = 0.55
    depth = 0.43
    height = 0.33
    wall = 0.012
    front_y = depth * 0.5

    divider_x = 0.111
    control_region_center_x = 0.194

    door_width = 0.366
    door_height = 0.276
    door_thickness = 0.024
    door_center_z = 0.165
    hinge_x = -0.271
    hinge_y = front_y + 0.008
    door_offset_x = 0.010

    panel_width = 0.138
    panel_height = 0.286
    panel_face_thickness = 0.003
    panel_tray_depth = 0.028

    dial_z = 0.072
    button_z = -0.090
    button_width = 0.050
    button_height = 0.018
    button_depth = 0.018

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=body_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall * 0.5)),
        material=body_white,
        name="top_shell",
    )
    housing.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(xyz=(-width * 0.5 + wall * 0.5, 0.0, height * 0.5)),
        material=body_white,
        name="left_shell",
    )
    housing.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(xyz=(width * 0.5 - wall * 0.5, 0.0, height * 0.5)),
        material=body_white,
        name="right_shell",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + wall * 0.5, height * 0.5)),
        material=body_white,
        name="back_shell",
    )
    housing.visual(
        Box((wall, depth - wall, height - 2.0 * wall)),
        origin=Origin(xyz=(divider_x, -wall * 0.5, height * 0.5)),
        material=warm_grey,
        name="cavity_divider",
    )
    housing.visual(
        Box((0.010, 0.020, door_height + 0.010)),
        origin=Origin(xyz=(hinge_x - 0.004, hinge_y - 0.018, door_center_z)),
        material=warm_grey,
        name="hinge_bracket",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(xyz=(hinge_x, hinge_y, door_center_z)),
        material=charcoal,
        name="hinge_barrel_mid",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    control_panel = model.part("control_panel")
    panel_outer = rounded_rect_profile(panel_width, panel_height, 0.012, corner_segments=8)
    button_hole = _translate_profile(
        rounded_rect_profile(button_width, button_height, 0.004, corner_segments=6),
        0.0,
        button_z,
    )
    panel_face_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            panel_outer,
            [button_hole],
            height=panel_face_thickness,
            center=True,
        ),
        "microwave_control_panel_face",
    )
    control_panel.visual(
        panel_face_mesh,
        origin=Origin(xyz=(0.0, -panel_face_thickness * 0.5, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_panel,
        name="panel_face",
    )
    control_panel.visual(
        Box((0.010, panel_tray_depth, panel_height - 0.010)),
        origin=Origin(xyz=(-panel_width * 0.5 + 0.005, -panel_tray_depth * 0.5, 0.0)),
        material=charcoal,
        name="left_flange",
    )
    control_panel.visual(
        Box((0.010, panel_tray_depth, panel_height - 0.010)),
        origin=Origin(xyz=(panel_width * 0.5 - 0.005, -panel_tray_depth * 0.5, 0.0)),
        material=charcoal,
        name="right_flange",
    )
    control_panel.visual(
        Box((panel_width, panel_tray_depth, 0.010)),
        origin=Origin(xyz=(0.0, -panel_tray_depth * 0.5, panel_height * 0.5 - 0.005)),
        material=charcoal,
        name="top_flange",
    )
    control_panel.visual(
        Box((panel_width, panel_tray_depth, 0.010)),
        origin=Origin(xyz=(0.0, -panel_tray_depth * 0.5, -panel_height * 0.5 + 0.005)),
        material=charcoal,
        name="bottom_flange",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_tray_depth, panel_height)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -panel_tray_depth * 0.5, 0.0)),
    )

    door = model.part("door")
    door_outer = rounded_rect_profile(door_width, door_height, 0.012, corner_segments=8)
    door_window = rounded_rect_profile(0.286, 0.214, 0.010, corner_segments=8)
    door_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            door_outer,
            [door_window],
            height=door_thickness,
            center=True,
        ),
        "microwave_door_frame",
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(door_width * 0.5 + door_offset_x, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="door_frame",
    )
    door.visual(
        Box((0.304, 0.004, 0.222)),
        origin=Origin(xyz=(door_width * 0.5 + door_offset_x, 0.002, 0.0)),
        material=black_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.020, 0.014, 0.190)),
        origin=Origin(xyz=(door_offset_x + door_width - 0.032, 0.019, 0.0)),
        material=knob_dark,
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=charcoal,
        name="door_hinge_knuckle_upper",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        material=charcoal,
        name="door_hinge_knuckle_lower",
    )
    door.visual(
        Box((0.012, 0.008, 0.070)),
        origin=Origin(xyz=(0.006, 0.004, 0.087)),
        material=charcoal,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.012, 0.008, 0.070)),
        origin=Origin(xyz=(0.006, 0.004, -0.087)),
        material=charcoal,
        name="lower_hinge_leaf",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.045, door_height)),
        mass=3.5,
        origin=Origin(xyz=(door_width * 0.5 + door_offset_x, 0.010, 0.0)),
    )

    dial = model.part("jog_dial")
    dial.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, 0.019)),
        material=knob_marker,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=0.028),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    button = model.part("push_button")
    button_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(button_width, button_height, 0.004, corner_segments=6),
            button_depth,
            center=True,
        ),
        "microwave_push_button",
    )
    button.visual(
        button_mesh,
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_grey,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((button_width, button_depth, button_height)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
    )

    model.articulation(
        "housing_to_control_panel",
        ArticulationType.FIXED,
        parent=housing,
        child=control_panel,
        origin=Origin(xyz=(control_region_center_x, front_y, height * 0.5)),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "control_panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=12.0,
        ),
    )
    model.articulation(
        "control_panel_to_button",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=button,
        origin=Origin(xyz=(0.0, 0.0, button_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    control_panel = object_model.get_part("control_panel")
    door = object_model.get_part("door")
    dial = object_model.get_part("jog_dial")
    button = object_model.get_part("push_button")

    door_joint = object_model.get_articulation("housing_to_door")
    dial_joint = object_model.get_articulation("control_panel_to_dial")
    button_joint = object_model.get_articulation("control_panel_to_button")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    ctx.check(
        "all primary parts exist",
        all(part is not None for part in (housing, control_panel, door, dial, button)),
        details="Expected housing, control panel, door, jog dial, and push button parts.",
    )
    ctx.check(
        "door hinge axis is vertical",
        tuple(round(value, 6) for value in door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "dial spins on panel-normal axis",
        tuple(round(value, 6) for value in dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={dial_joint.axis}",
    )
    ctx.check(
        "button translates into the panel",
        tuple(round(value, 6) for value in button_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={button_joint.axis}",
    )

    ctx.expect_contact(
        dial,
        control_panel,
        elem_a="dial_body",
        elem_b="panel_face",
        name="dial body seats on the control panel face",
    )
    ctx.expect_contact(
        control_panel,
        housing,
        name="control panel tray is mounted into the housing",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.radians(95.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            control_panel,
            door,
            axis="x",
            min_gap=0.20,
            name="opened door clears the control column laterally",
        )

    closed_max_y = closed_door_aabb[1][1] if closed_door_aabb is not None else None
    open_max_y = open_door_aabb[1][1] if open_door_aabb is not None else None
    ctx.check(
        "door opens outward from the front face",
        closed_max_y is not None and open_max_y is not None and open_max_y > closed_max_y + 0.12,
        details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
    )

    marker_rest = ctx.part_element_world_aabb(dial, elem="dial_marker")
    with ctx.pose({dial_joint: math.pi * 0.5}):
        marker_quarter = ctx.part_element_world_aabb(dial, elem="dial_marker")
    marker_rest_center = _aabb_center(marker_rest)
    marker_quarter_center = _aabb_center(marker_quarter)
    ctx.check(
        "dial marker visibly rotates with the jog dial",
        marker_rest_center is not None
        and marker_quarter_center is not None
        and abs(marker_quarter_center[0] - marker_rest_center[0]) > 0.012,
        details=f"rest={marker_rest_center}, quarter_turn={marker_quarter_center}",
    )

    button_rest = ctx.part_world_aabb(button)
    with ctx.pose({button_joint: 0.012}):
        button_pressed = ctx.part_world_aabb(button)
    button_rest_front = button_rest[1][1] if button_rest is not None else None
    button_pressed_front = button_pressed[1][1] if button_pressed is not None else None
    ctx.check(
        "button travels inward when pressed",
        button_rest_front is not None
        and button_pressed_front is not None
        and button_pressed_front < button_rest_front - 0.009,
        details=f"rest_front={button_rest_front}, pressed_front={button_pressed_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
