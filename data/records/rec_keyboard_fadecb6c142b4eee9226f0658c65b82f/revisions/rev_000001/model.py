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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_WIDTH = 0.276
HOUSING_DEPTH = 0.168
FLOOR_THICKNESS = 0.003
WALL_THICKNESS = 0.006
WALL_HEIGHT = 0.028

KEY_COLS = 5
KEY_ROWS = 4
KEY_POCKET_WIDTH = 0.022
KEY_POCKET_DEPTH = 0.019
KEY_GAP = 0.0018
KEY_BORDER = 0.004
KEY_FRAME_THICKNESS = 0.002
KEY_FRAME_UNDERSIDE_Z = 0.022

KEY_FIELD_WIDTH = KEY_COLS * KEY_POCKET_WIDTH + (KEY_COLS - 1) * KEY_GAP + 2.0 * KEY_BORDER
KEY_FIELD_DEPTH = KEY_ROWS * KEY_POCKET_DEPTH + (KEY_ROWS - 1) * KEY_GAP + 2.0 * KEY_BORDER
KEY_FIELD_CENTER_X = -0.030
KEY_FIELD_CENTER_Y = -0.004
KEY_STEP_X = KEY_POCKET_WIDTH + KEY_GAP
KEY_STEP_Y = KEY_POCKET_DEPTH + KEY_GAP

CONTROL_DECK_TOP_Z = 0.015
TRACKBALL_SOCKET_XYZ = (0.087, -0.010, CONTROL_DECK_TOP_Z)
JOG_POD_XYZ = (0.045, 0.058, 0.019)

TRACKBALL_RADIUS = 0.018
TRACKBALL_CENTER_Z = 0.018
TRACKBALL_PAD_RADIUS = 0.003
TRACKBALL_PAD_RING_RADIUS = 0.0195
TRACKBALL_PAD_Z = TRACKBALL_CENTER_Z - math.sqrt(
    (TRACKBALL_RADIUS + TRACKBALL_PAD_RADIUS) ** 2 - TRACKBALL_PAD_RING_RADIUS**2
)


def _key_part_name(row: int, col: int) -> str:
    return f"key_r{row}_c{col}"


def _key_joint_name(row: int, col: int) -> str:
    return f"housing_to_key_r{row}_c{col}"


def _key_center(row: int, col: int) -> tuple[float, float]:
    x = KEY_FIELD_CENTER_X + (col - (KEY_COLS - 1) / 2.0) * KEY_STEP_X
    y = KEY_FIELD_CENTER_Y + ((KEY_ROWS - 1) / 2.0 - row) * KEY_STEP_Y
    return (x, y)


def _build_trackball_socket_mesh():
    outer_profile = [
        (0.0240, 0.0000),
        (0.0278, 0.0100),
        (0.0268, 0.0200),
        (0.0225, 0.0280),
    ]
    inner_profile = [
        (0.0190, 0.0000),
        (0.0212, 0.0100),
        (0.0205, 0.0180),
        (0.0178, 0.0250),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_keyboard")

    housing_color = model.material("housing_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    deck_color = model.material("deck_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    key_dark = model.material("key_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    key_light = model.material("key_light", rgba=(0.36, 0.38, 0.40, 1.0))
    trackball_red = model.material("trackball_red", rgba=(0.67, 0.08, 0.08, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.58, 0.60, 0.62, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material=housing_color,
        name="floor",
    )

    side_wall_height = WALL_HEIGHT - FLOOR_THICKNESS
    side_wall_z = FLOOR_THICKNESS + side_wall_height / 2.0
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, side_wall_height)),
        origin=Origin(xyz=(-(HOUSING_WIDTH - WALL_THICKNESS) / 2.0, 0.0, side_wall_z)),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, side_wall_height)),
        origin=Origin(xyz=((HOUSING_WIDTH - WALL_THICKNESS) / 2.0, 0.0, side_wall_z)),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_wall_height)),
        origin=Origin(xyz=(0.0, -(HOUSING_DEPTH - WALL_THICKNESS) / 2.0, side_wall_z)),
        material=housing_color,
        name="front_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_wall_height)),
        origin=Origin(xyz=(0.0, (HOUSING_DEPTH - WALL_THICKNESS) / 2.0, side_wall_z)),
        material=housing_color,
        name="back_wall",
    )

    housing.visual(
        Box((0.082, 0.110, 0.004)),
        origin=Origin(xyz=(0.091, -0.006, 0.013)),
        material=deck_color,
        name="control_deck",
    )
    housing.visual(
        Box((0.136, 0.034, 0.004)),
        origin=Origin(xyz=(0.020, 0.061, 0.017)),
        material=deck_color,
        name="rear_shelf",
    )
    housing.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(0.060, -0.042, 0.007)),
        material=deck_color,
        name="control_support_front",
    )
    housing.visual(
        Box((0.024, 0.024, 0.012)),
        origin=Origin(xyz=(0.104, 0.030, 0.009)),
        material=deck_color,
        name="control_support_rear",
    )

    frame_center_z = KEY_FRAME_UNDERSIDE_Z + KEY_FRAME_THICKNESS / 2.0
    field_x_min = KEY_FIELD_CENTER_X - KEY_FIELD_WIDTH / 2.0
    field_y_min = KEY_FIELD_CENTER_Y - KEY_FIELD_DEPTH / 2.0

    housing.visual(
        Box((KEY_BORDER, KEY_FIELD_DEPTH, KEY_FRAME_THICKNESS)),
        origin=Origin(
            xyz=(field_x_min + KEY_BORDER / 2.0, KEY_FIELD_CENTER_Y, frame_center_z)
        ),
        material=deck_color,
        name="key_border_left",
    )
    housing.visual(
        Box((KEY_BORDER, KEY_FIELD_DEPTH, KEY_FRAME_THICKNESS)),
        origin=Origin(
            xyz=(field_x_min + KEY_FIELD_WIDTH - KEY_BORDER / 2.0, KEY_FIELD_CENTER_Y, frame_center_z)
        ),
        material=deck_color,
        name="key_border_right",
    )
    housing.visual(
        Box((KEY_FIELD_WIDTH - 2.0 * KEY_BORDER, KEY_BORDER, KEY_FRAME_THICKNESS)),
        origin=Origin(
            xyz=(KEY_FIELD_CENTER_X, field_y_min + KEY_FIELD_DEPTH - KEY_BORDER / 2.0, frame_center_z)
        ),
        material=deck_color,
        name="key_border_top",
    )
    housing.visual(
        Box((KEY_FIELD_WIDTH - 2.0 * KEY_BORDER, KEY_BORDER, KEY_FRAME_THICKNESS)),
        origin=Origin(
            xyz=(KEY_FIELD_CENTER_X, field_y_min + KEY_BORDER / 2.0, frame_center_z)
        ),
        material=deck_color,
        name="key_border_bottom",
    )

    for divider in range(KEY_COLS - 1):
        divider_x = (
            field_x_min
            + KEY_BORDER
            + KEY_POCKET_WIDTH
            + KEY_GAP / 2.0
            + divider * KEY_STEP_X
        )
        housing.visual(
            Box((KEY_GAP, KEY_FIELD_DEPTH - 2.0 * KEY_BORDER, KEY_FRAME_THICKNESS)),
            origin=Origin(xyz=(divider_x, KEY_FIELD_CENTER_Y, frame_center_z)),
            material=deck_color,
            name=f"key_divider_vertical_{divider}",
        )

    for divider in range(KEY_ROWS - 1):
        divider_y = (
            field_y_min
            + KEY_BORDER
            + KEY_POCKET_DEPTH
            + KEY_GAP / 2.0
            + divider * KEY_STEP_Y
        )
        housing.visual(
            Box((KEY_FIELD_WIDTH - 2.0 * KEY_BORDER, KEY_GAP, KEY_FRAME_THICKNESS)),
            origin=Origin(xyz=(KEY_FIELD_CENTER_X, divider_y, frame_center_z)),
            material=deck_color,
            name=f"key_divider_horizontal_{divider}",
        )

    post_height = KEY_FRAME_UNDERSIDE_Z - FLOOR_THICKNESS
    post_z = FLOOR_THICKNESS + post_height / 2.0
    for index, (post_x, post_y) in enumerate(
        (
            (
                KEY_FIELD_CENTER_X - KEY_FIELD_WIDTH / 2.0 + 0.008,
                KEY_FIELD_CENTER_Y - KEY_FIELD_DEPTH / 2.0 + 0.008,
            ),
            (
                KEY_FIELD_CENTER_X + KEY_FIELD_WIDTH / 2.0 - 0.008,
                KEY_FIELD_CENTER_Y - KEY_FIELD_DEPTH / 2.0 + 0.008,
            ),
            (
                KEY_FIELD_CENTER_X - KEY_FIELD_WIDTH / 2.0 + 0.008,
                KEY_FIELD_CENTER_Y + KEY_FIELD_DEPTH / 2.0 - 0.008,
            ),
            (
                KEY_FIELD_CENTER_X + KEY_FIELD_WIDTH / 2.0 - 0.008,
                KEY_FIELD_CENTER_Y + KEY_FIELD_DEPTH / 2.0 - 0.008,
            ),
        )
    ):
        housing.visual(
            Box((0.012, 0.012, post_height)),
            origin=Origin(xyz=(post_x, post_y, post_z)),
            material=deck_color,
            name=f"key_frame_post_{index}",
        )

    housing.visual(
        Box((0.156, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, -0.074, 0.009)),
        material=trim_silver,
        name="front_trim",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, WALL_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, WALL_HEIGHT / 2.0)),
    )

    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            key_part = model.part(_key_part_name(row, col))
            key_material = key_light if row == 0 else key_dark
            key_part.visual(
                Box((0.0236, 0.0200, 0.0010)),
                origin=Origin(xyz=(0.0, 0.0, -0.0005)),
                material=key_material,
                name="retainer_flange",
            )
            key_part.visual(
                Box((0.0140, 0.0105, 0.0025)),
                origin=Origin(xyz=(0.0, 0.0, 0.00125)),
                material=key_material,
                name="plunger_stem",
            )
            key_part.visual(
                Box((0.0204, 0.0174, 0.0030)),
                origin=Origin(xyz=(0.0, 0.0, 0.0040)),
                material=key_material,
                name="keycap",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((0.0236, 0.0200, 0.0065)),
                mass=0.010,
                origin=Origin(xyz=(0.0, 0.0, 0.00275)),
            )

            key_x, key_y = _key_center(row, col)
            model.articulation(
                _key_joint_name(row, col),
                ArticulationType.PRISMATIC,
                parent=housing,
                child=key_part,
                origin=Origin(xyz=(key_x, key_y, KEY_FRAME_UNDERSIDE_Z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=1.5,
                    velocity=0.08,
                    lower=-0.0022,
                    upper=0.0,
                ),
            )

    socket = model.part("trackball_socket")
    socket.visual(
        mesh_from_geometry(_build_trackball_socket_mesh(), "trackball_socket_shell"),
        material=trim_silver,
        name="socket_shell",
    )
    for index in range(3):
        angle = math.pi / 6.0 + index * (2.0 * math.pi / 3.0)
        pad_x = TRACKBALL_PAD_RING_RADIUS * math.cos(angle)
        pad_y = TRACKBALL_PAD_RING_RADIUS * math.sin(angle)
        post_height_local = TRACKBALL_PAD_Z - TRACKBALL_PAD_RADIUS
        socket.visual(
            Cylinder(radius=0.0015, length=post_height_local),
            origin=Origin(xyz=(pad_x, pad_y, post_height_local / 2.0)),
            material=trim_silver,
            name=f"pad_post_{index}",
        )
        socket.visual(
            Sphere(radius=TRACKBALL_PAD_RADIUS),
            origin=Origin(xyz=(pad_x, pad_y, TRACKBALL_PAD_Z)),
            material=trim_silver,
            name=f"ball_support_{index}",
        )
    socket.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )
    model.articulation(
        "housing_to_trackball_socket",
        ArticulationType.FIXED,
        parent=housing,
        child=socket,
        origin=Origin(xyz=TRACKBALL_SOCKET_XYZ),
    )

    trackball = model.part("trackball")
    trackball.visual(
        Sphere(radius=TRACKBALL_RADIUS),
        material=trackball_red,
        name="ball",
    )
    trackball.inertial = Inertial.from_geometry(
        Sphere(radius=TRACKBALL_RADIUS),
        mass=0.045,
    )
    model.articulation(
        "trackball_socket_to_trackball",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=trackball,
        origin=Origin(xyz=(0.0, 0.0, TRACKBALL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    jog_pod = model.part("jog_wheel_pod")
    jog_pod.visual(
        Box((0.040, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=deck_color,
        name="pod_base",
    )
    jog_pod.visual(
        Box((0.008, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.012)),
        material=deck_color,
        name="pod_front_cheek",
    )
    jog_pod.visual(
        Box((0.008, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.012)),
        material=deck_color,
        name="pod_rear_cheek",
    )
    jog_pod.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="front_axle_pin",
    )
    jog_pod.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="rear_axle_pin",
    )
    jog_pod.inertial = Inertial.from_geometry(
        Box((0.040, 0.028, 0.024)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "housing_to_jog_wheel_pod",
        ArticulationType.FIXED,
        parent=housing,
        child=jog_pod,
        origin=Origin(xyz=JOG_POD_XYZ),
    )

    jog_wheel = model.part("jog_wheel")
    jog_wheel.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    jog_wheel.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="wheel_hub",
    )
    jog_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.012),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "jog_wheel_pod_to_jog_wheel",
        ArticulationType.CONTINUOUS,
        parent=jog_pod,
        child=jog_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    socket = object_model.get_part("trackball_socket")
    trackball = object_model.get_part("trackball")
    jog_pod = object_model.get_part("jog_wheel_pod")
    jog_wheel = object_model.get_part("jog_wheel")
    trackball_joint = object_model.get_articulation("trackball_socket_to_trackball")
    jog_joint = object_model.get_articulation("jog_wheel_pod_to_jog_wheel")
    sample_key = object_model.get_part(_key_part_name(1, 2))
    sample_key_joint = object_model.get_articulation(_key_joint_name(1, 2))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_contact(socket, housing, contact_tol=5e-5, name="trackball_socket_mounted")
    ctx.expect_contact(trackball, socket, contact_tol=5e-5, name="trackball_seated")
    ctx.expect_overlap(trackball, socket, axes="xy", min_overlap=0.030, name="trackball_over_socket")
    ctx.expect_contact(jog_pod, housing, contact_tol=5e-5, name="jog_pod_mounted")
    ctx.expect_contact(jog_wheel, jog_pod, contact_tol=5e-5, name="jog_wheel_supported")

    ctx.check(
        "trackball_joint_is_continuous",
        trackball_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous joint, got {trackball_joint.articulation_type!r}",
    )
    ctx.check(
        "trackball_joint_axis_vertical",
        tuple(trackball_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical axis, got {trackball_joint.axis!r}",
    )
    ctx.check(
        "jog_wheel_joint_is_continuous",
        jog_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous joint, got {jog_joint.articulation_type!r}",
    )
    ctx.check(
        "jog_wheel_axis_front_to_back",
        tuple(jog_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected front-to-back axis, got {jog_joint.axis!r}",
    )

    released_position = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: -0.0022}):
        pressed_position = ctx.part_world_position(sample_key)
    ctx.check(
        "sample_key_moves_down",
        released_position is not None
        and pressed_position is not None
        and pressed_position[2] < released_position[2] - 0.0015,
        details=f"Released={released_position}, pressed={pressed_position}",
    )

    with ctx.pose({trackball_joint: math.pi, jog_joint: math.pi / 2.0}):
        ctx.expect_contact(
            trackball,
            socket,
            contact_tol=5e-5,
            name="trackball_keeps_contact_when_spun",
        )
        ctx.expect_contact(
            jog_wheel,
            jog_pod,
            contact_tol=5e-5,
            name="jog_wheel_keeps_contact_when_turned",
        )

    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            key = object_model.get_part(_key_part_name(row, col))
            key_joint = object_model.get_articulation(_key_joint_name(row, col))
            ctx.expect_contact(
                key,
                housing,
                contact_tol=5e-5,
                name=f"{key.name}_mounted",
            )
            ctx.check(
                f"{key_joint.name}_axis",
                tuple(key_joint.axis) == (0.0, 0.0, 1.0),
                details=f"Expected key axis (0, 0, 1), got {key_joint.axis!r}",
            )
            ctx.check(
                f"{key_joint.name}_limits",
                key_joint.motion_limits is not None
                and key_joint.motion_limits.lower == -0.0022
                and key_joint.motion_limits.upper == 0.0,
                details=f"Unexpected limits {key_joint.motion_limits!r}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
