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


LOWER_WIDTH = 0.054
LOWER_LENGTH = 0.098
LOWER_BODY_HEIGHT = 0.014
LOWER_BACK_THICKNESS = 0.0026
LOWER_SIDE_HEIGHT = 0.0098
LOWER_FASCIA_THICKNESS = 0.0016
LOWER_CENTER_Y = -LOWER_LENGTH / 2.0

UPPER_WIDTH = 0.052
UPPER_LENGTH = 0.091
UPPER_BEZEL_THICKNESS = 0.0014
UPPER_SIDE_HEIGHT = 0.0090
UPPER_BACK_THICKNESS = 0.0022
UPPER_CENTER_Y = UPPER_LENGTH / 2.0

BODY_CORNER_RADIUS = 0.008
INNER_CORNER_RADIUS = 0.0064

HINGE_RADIUS = 0.0032
HINGE_Y = 0.0015
HINGE_Z = 0.0165
HINGE_BARREL_LENGTH = 0.018
HINGE_CENTER_BARREL_LENGTH = LOWER_WIDTH - (2.0 * HINGE_BARREL_LENGTH)

OPEN_RANGE = math.radians(170.0)
UPPER_OPEN_TILT = math.radians(10.0)

NUMERIC_PAD_SIZE = (0.043, 0.053, 0.0020)
NUMERIC_PAD_CENTER_Y = -0.065
NAV_PAD_SIZE = (0.041, 0.030, 0.0020)
NAV_PAD_CENTER_Y = -0.024


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rotate_x_point(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, (y * c) - (z * s), (y * s) + (z * c))


def _upper_origin(
    xyz: tuple[float, float, float],
    *,
    roll_with_body: bool = True,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    rotated_xyz = _rotate_x_point(xyz, UPPER_OPEN_TILT)
    if roll_with_body:
        return Origin(xyz=rotated_xyz, rpy=(UPPER_OPEN_TILT + rpy[0], rpy[1], rpy[2]))
    return Origin(xyz=rotated_xyz, rpy=rpy)


def _rounded_plate_mesh(
    width: float,
    height: float,
    radius: float,
    thickness: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, height, radius),
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def _rounded_frame_mesh(
    outer_width: float,
    outer_height: float,
    outer_radius: float,
    holes: list[list[tuple[float, float]]],
    thickness: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_height, outer_radius),
            holes,
            thickness,
            cap=True,
            center=False,
            closed=True,
        ),
        name,
    )


def _build_numeric_keypad_part(model: ArticulatedObject, material, label_material):
    keypad = model.part("numeric_keypad")
    keypad_mesh = _rounded_plate_mesh(
        NUMERIC_PAD_SIZE[0],
        NUMERIC_PAD_SIZE[1],
        0.004,
        NUMERIC_PAD_SIZE[2],
        "numeric_keypad_base",
    )
    keypad.visual(keypad_mesh, material=material, name="numeric_pad_base")

    key_w = 0.010
    key_h = 0.0085
    key_thickness = 0.0012
    key_z = NUMERIC_PAD_SIZE[2]
    key_mesh = _rounded_plate_mesh(
        key_w,
        key_h,
        0.0020,
        key_thickness,
        "numeric_key_cap",
    )
    legend_mesh = _rounded_plate_mesh(
        key_w * 0.52,
        key_h * 0.20,
        0.00045,
        0.00018,
        "numeric_key_legend",
    )
    x_positions = (-0.013, 0.0, 0.013)
    y_positions = (0.017, 0.005, -0.007, -0.019)
    for row_index, y_pos in enumerate(y_positions):
        for column_index, x_pos in enumerate(x_positions):
            key_index = row_index * 3 + column_index + 1
            keypad.visual(
                key_mesh,
                origin=Origin(xyz=(x_pos, y_pos, key_z)),
                material=label_material,
                name=f"key_{key_index:02d}",
            )
            keypad.visual(
                legend_mesh,
                origin=Origin(xyz=(x_pos, y_pos + (key_h * 0.10), key_z + key_thickness)),
                material=material,
                name=f"key_{key_index:02d}_legend",
            )

    keypad.inertial = Inertial.from_geometry(
        Box(NUMERIC_PAD_SIZE),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, NUMERIC_PAD_SIZE[2] / 2.0)),
    )
    return keypad


def _build_navigation_cluster_part(model: ArticulatedObject, material, accent, action_green, action_red):
    nav = model.part("navigation_cluster")
    nav_mesh = _rounded_plate_mesh(
        NAV_PAD_SIZE[0],
        NAV_PAD_SIZE[1],
        0.004,
        NAV_PAD_SIZE[2],
        "navigation_cluster_base",
    )
    nav.visual(nav_mesh, material=material, name="nav_cluster_base")

    top_z = NAV_PAD_SIZE[2]
    soft_key_mesh = _rounded_plate_mesh(
        0.010,
        0.006,
        0.0015,
        0.0012,
        "soft_key_cap",
    )
    action_key_mesh = _rounded_plate_mesh(
        0.009,
        0.006,
        0.0015,
        0.0012,
        "action_key_cap",
    )
    dpad_ring_mesh = _rounded_frame_mesh(
        0.021,
        0.020,
        0.0045,
        [rounded_rect_profile(0.010, 0.009, 0.0028)],
        0.0010,
        "dpad_ring_cap",
    )
    select_mesh = _rounded_plate_mesh(
        0.0088,
        0.0088,
        0.0028,
        0.0012,
        "select_key_cap",
    )
    nav.visual(
        soft_key_mesh,
        origin=Origin(xyz=(-0.015, 0.009, top_z)),
        material=accent,
        name="soft_left",
    )
    nav.visual(
        soft_key_mesh,
        origin=Origin(xyz=(0.015, 0.009, top_z)),
        material=accent,
        name="soft_right",
    )
    nav.visual(
        action_key_mesh,
        origin=Origin(xyz=(-0.014, -0.008, top_z)),
        material=action_green,
        name="call_key",
    )
    nav.visual(
        action_key_mesh,
        origin=Origin(xyz=(0.014, -0.008, top_z)),
        material=action_red,
        name="end_key",
    )
    nav.visual(
        dpad_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=accent,
        name="dpad_ring",
    )
    nav.visual(
        select_mesh,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=material,
        name="select_button",
    )

    nav.inertial = Inertial.from_geometry(
        Box(NAV_PAD_SIZE),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, NAV_PAD_SIZE[2] / 2.0)),
    )
    return nav


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_flip_phone")

    body_silver = model.material("body_silver", rgba=(0.72, 0.74, 0.78, 1.0))
    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.20, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    keypad_dark = model.material("keypad_dark", rgba=(0.21, 0.23, 0.26, 1.0))
    key_label = model.material("key_label", rgba=(0.76, 0.79, 0.83, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.16, 0.30, 0.38, 0.70))
    speaker_dark = model.material("speaker_dark", rgba=(0.05, 0.05, 0.06, 1.0))
    call_green = model.material("call_green", rgba=(0.16, 0.53, 0.25, 1.0))
    end_red = model.material("end_red", rgba=(0.66, 0.16, 0.16, 1.0))

    lower_outer_mesh = _rounded_plate_mesh(
        LOWER_WIDTH,
        LOWER_LENGTH,
        BODY_CORNER_RADIUS,
        LOWER_BACK_THICKNESS,
        "lower_back_cover",
    )
    lower_side_ring_mesh = _rounded_frame_mesh(
        LOWER_WIDTH,
        LOWER_LENGTH,
        BODY_CORNER_RADIUS,
        [rounded_rect_profile(0.047, 0.091, INNER_CORNER_RADIUS)],
        LOWER_SIDE_HEIGHT,
        "lower_side_ring",
    )
    lower_fascia_mesh = _rounded_frame_mesh(
        0.047,
        0.091,
        INNER_CORNER_RADIUS,
        [
            rounded_rect_profile(NAV_PAD_SIZE[0], NAV_PAD_SIZE[1], 0.004),
            rounded_rect_profile(NUMERIC_PAD_SIZE[0], NUMERIC_PAD_SIZE[1], 0.004),
        ],
        LOWER_FASCIA_THICKNESS,
        "lower_front_fascia",
    )

    upper_back_mesh = _rounded_plate_mesh(
        UPPER_WIDTH,
        UPPER_LENGTH,
        BODY_CORNER_RADIUS,
        UPPER_BACK_THICKNESS,
        "upper_back_cover",
    )
    upper_side_ring_mesh = _rounded_frame_mesh(
        UPPER_WIDTH,
        UPPER_LENGTH,
        BODY_CORNER_RADIUS,
        [rounded_rect_profile(0.046, 0.085, INNER_CORNER_RADIUS)],
        UPPER_SIDE_HEIGHT,
        "upper_side_ring",
    )
    upper_bezel_mesh = _rounded_frame_mesh(
        0.046,
        0.085,
        INNER_CORNER_RADIUS,
        [
            _offset_profile(
                rounded_rect_profile(0.035, 0.032, 0.004),
                dy=0.006,
            )
        ],
        UPPER_BEZEL_THICKNESS,
        "upper_inner_bezel",
    )
    microphone_mesh = _rounded_plate_mesh(
        0.022,
        0.0025,
        0.0011,
        0.0008,
        "microphone_slot_shape",
    )
    earpiece_mesh = _rounded_plate_mesh(
        0.016,
        0.0025,
        0.0011,
        0.0008,
        "earpiece_slot_shape",
    )
    screen_glass_mesh = _rounded_plate_mesh(
        0.033,
        0.030,
        0.0032,
        0.0008,
        "display_glass_shape",
    )

    lower_body = model.part("lower_body")
    lower_body.visual(
        lower_outer_mesh,
        origin=Origin(xyz=(0.0, LOWER_CENTER_Y, 0.0)),
        material=body_silver,
        name="back_cover",
    )
    lower_body.visual(
        lower_side_ring_mesh,
        origin=Origin(xyz=(0.0, LOWER_CENTER_Y, LOWER_BACK_THICKNESS)),
        material=body_silver,
        name="side_ring",
    )
    lower_body.visual(
        lower_fascia_mesh,
        origin=Origin(xyz=(0.0, LOWER_CENTER_Y, LOWER_BACK_THICKNESS + LOWER_SIDE_HEIGHT)),
        material=bezel_dark,
        name="front_fascia",
    )
    lower_body.visual(
        microphone_mesh,
        origin=Origin(xyz=(0.0, -0.089, LOWER_BACK_THICKNESS + LOWER_SIDE_HEIGHT)),
        material=speaker_dark,
        name="microphone_slot",
    )
    lower_body.visual(
        Box((0.019, 0.006, 0.0044)),
        origin=Origin(xyz=(-0.0175, -0.0010, 0.0136)),
        material=body_dark,
        name="left_hinge_bridge",
    )
    lower_body.visual(
        Box((0.019, 0.006, 0.0044)),
        origin=Origin(xyz=(0.0175, -0.0010, 0.0136)),
        material=body_dark,
        name="right_hinge_bridge",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(-(HINGE_CENTER_BARREL_LENGTH + HINGE_BARREL_LENGTH) / 2.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_dark,
        name="left_barrel",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=((HINGE_CENTER_BARREL_LENGTH + HINGE_BARREL_LENGTH) / 2.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_dark,
        name="right_barrel",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((LOWER_WIDTH, LOWER_LENGTH, HINGE_Z)),
        mass=0.085,
        origin=Origin(xyz=(0.0, LOWER_CENTER_Y, HINGE_Z / 2.0)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        upper_back_mesh,
        origin=_upper_origin((0.0, UPPER_CENTER_Y, -0.0127)),
        material=body_silver,
        name="upper_back_cover",
    )
    upper_body.visual(
        upper_side_ring_mesh,
        origin=_upper_origin((0.0, UPPER_CENTER_Y, -0.0105)),
        material=body_silver,
        name="upper_side_ring",
    )
    upper_body.visual(
        upper_bezel_mesh,
        origin=_upper_origin((0.0, UPPER_CENTER_Y, -0.0015)),
        material=bezel_dark,
        name="upper_bezel",
    )
    upper_body.visual(
        Box((0.024, 0.072, 0.0014)),
        origin=_upper_origin((0.0, 0.041, -0.0022)),
        material=body_dark,
        name="display_spine_support",
    )
    upper_body.visual(
        Box((0.0315, 0.0285, 0.0008)),
        origin=_upper_origin((0.0, 0.051, -0.0019)),
        material=body_dark,
        name="display_panel",
    )
    upper_body.visual(
        screen_glass_mesh,
        origin=_upper_origin((0.0, 0.051, -0.0015)),
        material=screen_glass,
        name="screen_glass",
    )
    upper_body.visual(
        Box((0.024, 0.006, 0.0040)),
        origin=_upper_origin((0.0, 0.0045, -0.0016)),
        material=body_dark,
        name="center_hinge_bridge",
    )
    upper_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_CENTER_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="center_barrel",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((UPPER_WIDTH, UPPER_LENGTH, 0.020)),
        mass=0.055,
        origin=_upper_origin((0.0, UPPER_CENTER_Y, -0.0060)),
    )

    earpiece_grille = model.part("earpiece_grille")
    earpiece_grille.visual(
        earpiece_mesh,
        material=speaker_dark,
        name="earpiece_slot",
    )
    earpiece_grille.inertial = Inertial.from_geometry(
        Box((0.016, 0.0025, 0.0012)),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
    )

    navigation_cluster = _build_navigation_cluster_part(
        model,
        keypad_dark,
        key_label,
        call_green,
        end_red,
    )
    numeric_keypad = _build_numeric_keypad_part(model, keypad_dark, key_label)

    model.articulation(
        "lower_to_navigation_cluster",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=navigation_cluster,
        origin=Origin(
            xyz=(
                0.0,
                NAV_PAD_CENTER_Y,
                LOWER_BACK_THICKNESS + LOWER_SIDE_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0012,
        ),
    )
    model.articulation(
        "lower_to_numeric_keypad",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=numeric_keypad,
        origin=Origin(
            xyz=(
                0.0,
                NUMERIC_PAD_CENTER_Y,
                LOWER_BACK_THICKNESS + LOWER_SIDE_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=0.05,
            lower=0.0,
            upper=0.0012,
        ),
    )
    model.articulation(
        "clamshell_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=OPEN_RANGE,
        ),
    )
    model.articulation(
        "upper_to_earpiece_grille",
        ArticulationType.FIXED,
        parent=upper_body,
        child=earpiece_grille,
        origin=_upper_origin((0.0, 0.018, -0.0017)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    earpiece_grille = object_model.get_part("earpiece_grille")
    navigation_cluster = object_model.get_part("navigation_cluster")
    numeric_keypad = object_model.get_part("numeric_keypad")

    clamshell_hinge = object_model.get_articulation("clamshell_hinge")
    nav_press = object_model.get_articulation("lower_to_navigation_cluster")
    keypad_press = object_model.get_articulation("lower_to_numeric_keypad")

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

    ctx.expect_contact(
        upper_body,
        lower_body,
        elem_a="center_barrel",
        elem_b="left_barrel",
        name="hinge_left_barrel_contact",
    )
    ctx.expect_contact(
        upper_body,
        lower_body,
        elem_a="center_barrel",
        elem_b="right_barrel",
        name="hinge_right_barrel_contact",
    )
    ctx.expect_contact(earpiece_grille, upper_body, name="earpiece_grille_mounted")
    ctx.expect_contact(navigation_cluster, lower_body, name="navigation_cluster_mounted")
    ctx.expect_contact(numeric_keypad, lower_body, name="numeric_keypad_mounted")
    ctx.expect_within(
        navigation_cluster,
        lower_body,
        axes="xy",
        margin=0.0005,
        name="navigation_cluster_within_lower_body",
    )
    ctx.expect_within(
        numeric_keypad,
        lower_body,
        axes="xy",
        margin=0.0005,
        name="numeric_keypad_within_lower_body",
    )

    with ctx.pose({clamshell_hinge: 0.0}):
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="y",
            positive_elem="screen_glass",
            negative_elem="front_fascia",
            min_gap=0.010,
            name="phone_open_screen_above_keypad_area",
        )
        ctx.expect_overlap(
            upper_body,
            lower_body,
            axes="x",
            min_overlap=0.045,
            name="phone_halves_share_width",
        )

    hinge_limits = clamshell_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({clamshell_hinge: hinge_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="clamshell_open_no_overlap")
            ctx.fail_if_isolated_parts(name="clamshell_open_no_floating")
        with ctx.pose({clamshell_hinge: hinge_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="clamshell_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="clamshell_closed_no_floating")
            ctx.expect_overlap(
                upper_body,
                lower_body,
                axes="xy",
                min_overlap=0.040,
                name="closed_phone_stacks_compactly",
            )
            ctx.expect_gap(
                upper_body,
                lower_body,
                axis="z",
                positive_elem="screen_glass",
                negative_elem="front_fascia",
                min_gap=0.001,
                max_gap=0.010,
                name="closed_phone_has_small_clamshell_gap",
            )

    for joint, part_name in (
        (nav_press, "navigation_cluster"),
        (keypad_press, "numeric_keypad"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            rest_pos = ctx.part_world_position(part_name)
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{part_name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{part_name}_pressed_no_floating")
                pressed_pos = ctx.part_world_position(part_name)
                moved_down = (
                    rest_pos is not None
                    and pressed_pos is not None
                    and pressed_pos[2] < rest_pos[2] - 0.0008
                )
                ctx.check(
                    f"{part_name}_has_downward_travel",
                    moved_down,
                    details=f"{part_name} should move downward by about 1 mm when pressed.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
