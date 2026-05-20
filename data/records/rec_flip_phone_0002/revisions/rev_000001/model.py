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

LOWER_W = 0.074
LOWER_D = 0.087
LOWER_T = 0.019
LOWER_REAR = 0.012

UPPER_W = 0.064
UPPER_D = 0.074
UPPER_T = 0.017
UPPER_REAR = 0.008

CLOSED_GAP = 0.003
HINGE_OPEN_LIMIT = math.radians(160.0)
X_AXIS_CYLINDER = (0.0, math.pi / 2.0, 0.0)
Y_AXIS_CYLINDER = (-math.pi / 2.0, 0.0, 0.0)


def _add_corner_bumpers(
    part,
    prefix: str,
    *,
    width: float,
    depth: float,
    thickness: float,
    rear_offset: float,
    z_center: float,
    material,
) -> None:
    bumper_w = 0.011
    bumper_d = 0.011
    bumper_t = thickness + 0.001
    x_positions = (
        -(width / 2.0 - bumper_w / 2.0 + 0.0012),
        width / 2.0 - bumper_w / 2.0 + 0.0012,
    )
    y_positions = (
        ("rear", rear_offset + bumper_d / 2.0 - 0.001),
        ("front", rear_offset + depth - bumper_d / 2.0 + 0.001),
    )
    for side, x_pos in (("left", x_positions[0]), ("right", x_positions[1])):
        for end_name, y_pos in y_positions:
            part.visual(
                Box((bumper_w, bumper_d, bumper_t)),
                origin=Origin(xyz=(x_pos, y_pos, z_center)),
                material=material,
                name=f"{prefix}_{end_name}_{side}_bumper",
            )


def _add_grip_strips(
    part,
    prefix: str,
    *,
    width: float,
    depth: float,
    thickness: float,
    rear_offset: float,
    z_center: float,
    material,
) -> None:
    strip_w = 0.0032
    strip_d = depth * 0.56
    strip_t = thickness * 0.82
    rib_d = 0.009
    rib_t = thickness * 0.17
    x_positions = (
        -(width / 2.0 - strip_w / 2.0 + 0.0007),
        width / 2.0 - strip_w / 2.0 + 0.0007,
    )
    rib_y_positions = (
        rear_offset + depth * 0.34,
        rear_offset + depth * 0.50,
        rear_offset + depth * 0.66,
    )
    for side, x_pos in (("left", x_positions[0]), ("right", x_positions[1])):
        part.visual(
            Box((strip_w, strip_d, strip_t)),
            origin=Origin(xyz=(x_pos, rear_offset + depth * 0.50, z_center)),
            material=material,
            name=f"{prefix}_{side}_grip_strip",
        )
        for rib_index, y_pos in enumerate(rib_y_positions, start=1):
            part.visual(
                Box((strip_w * 1.15, rib_d, rib_t)),
                origin=Origin(xyz=(x_pos, y_pos, z_center)),
                material=material,
                name=f"{prefix}_{side}_grip_rib_{rib_index}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_flip_phone", assets=ASSETS)

    shell = model.material("shell_polymer", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber_trim", rgba=(0.07, 0.07, 0.08, 1.0))
    keycap = model.material("keycap", rgba=(0.24, 0.25, 0.27, 1.0))
    screen_mat = model.material("lcd_glass", rgba=(0.08, 0.14, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.84, 1.0))
    lens_black = model.material("lens_black", rgba=(0.03, 0.03, 0.04, 1.0))

    lower = model.part("lower_body")
    lower.visual(
        Box((LOWER_W, LOWER_D, LOWER_T)),
        origin=Origin(xyz=(0.0, LOWER_REAR + LOWER_D / 2.0, -LOWER_T / 2.0)),
        material=shell,
        name="lower_shell",
    )
    lower.visual(
        Box((0.060, 0.064, 0.0012)),
        origin=Origin(xyz=(0.0, LOWER_REAR + 0.053, 0.0005)),
        material=shell_dark,
        name="control_deck",
    )
    lower.visual(
        Box((0.018, 0.009, 0.007)),
        origin=Origin(xyz=(-0.0235, 0.0075, -0.007)),
        material=shell_dark,
        name="left_hinge_support",
    )
    lower.visual(
        Box((0.018, 0.009, 0.007)),
        origin=Origin(xyz=(0.0235, 0.0075, -0.007)),
        material=shell_dark,
        name="right_hinge_support",
    )
    lower.visual(
        Cylinder(radius=0.0061, length=0.021),
        origin=Origin(xyz=(-0.0235, 0.0, 0.0), rpy=X_AXIS_CYLINDER),
        material=shell_dark,
        name="left_barrel",
    )
    lower.visual(
        Cylinder(radius=0.0061, length=0.021),
        origin=Origin(xyz=(0.0235, 0.0, 0.0), rpy=X_AXIS_CYLINDER),
        material=shell_dark,
        name="right_barrel",
    )
    lower.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(-0.0355, 0.0, 0.0), rpy=X_AXIS_CYLINDER),
        material=chrome,
        name="left_pivot_pin",
    )
    lower.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(0.0355, 0.0, 0.0), rpy=X_AXIS_CYLINDER),
        material=chrome,
        name="right_pivot_pin",
    )

    dpad_y = LOWER_REAR + 0.025
    lower.visual(
        Box((0.024, 0.009, 0.0014)),
        origin=Origin(xyz=(0.0, dpad_y, 0.0011)),
        material=keycap,
        name="dpad_horizontal",
    )
    lower.visual(
        Box((0.010, 0.024, 0.0014)),
        origin=Origin(xyz=(0.0, dpad_y, 0.0011)),
        material=keycap,
        name="dpad_vertical",
    )
    lower.visual(
        Cylinder(radius=0.0052, length=0.0017),
        origin=Origin(xyz=(0.0, dpad_y, 0.0013)),
        material=keycap,
        name="dpad_center",
    )
    lower.visual(
        Box((0.013, 0.008, 0.0012)),
        origin=Origin(xyz=(-0.021, LOWER_REAR + 0.026, 0.0010)),
        material=keycap,
        name="left_soft_key",
    )
    lower.visual(
        Box((0.013, 0.008, 0.0012)),
        origin=Origin(xyz=(0.021, LOWER_REAR + 0.026, 0.0010)),
        material=keycap,
        name="right_soft_key",
    )

    button_x_positions = (-0.018, 0.0, 0.018)
    button_y_positions = (
        LOWER_REAR + 0.043,
        LOWER_REAR + 0.056,
        LOWER_REAR + 0.069,
        LOWER_REAR + 0.082,
    )
    button_names = (
        ("key_1", "key_2", "key_3"),
        ("key_4", "key_5", "key_6"),
        ("key_7", "key_8", "key_9"),
        ("key_star", "key_0", "key_hash"),
    )
    for row_y, row_names in zip(button_y_positions, button_names):
        for x_pos, button_name in zip(button_x_positions, row_names):
            lower.visual(
                Box((0.015, 0.0085, 0.0012)),
                origin=Origin(xyz=(x_pos, row_y, 0.0011)),
                material=keycap,
                name=button_name,
            )

    lower.visual(
        Cylinder(radius=0.0032, length=0.013),
        origin=Origin(
            xyz=(0.0, LOWER_REAR + LOWER_D + 0.006, -0.010),
            rpy=Y_AXIS_CYLINDER,
        ),
        material=shell_dark,
        name="antenna_stub",
    )
    _add_corner_bumpers(
        lower,
        "lower",
        width=LOWER_W,
        depth=LOWER_D,
        thickness=LOWER_T,
        rear_offset=LOWER_REAR,
        z_center=-LOWER_T / 2.0,
        material=rubber,
    )
    _add_grip_strips(
        lower,
        "lower",
        width=LOWER_W,
        depth=LOWER_D,
        thickness=LOWER_T,
        rear_offset=LOWER_REAR,
        z_center=-LOWER_T / 2.0,
        material=rubber,
    )
    lower.inertial = Inertial.from_geometry(
        Box((LOWER_W, LOWER_D, LOWER_T)),
        mass=0.18,
        origin=Origin(xyz=(0.0, LOWER_REAR + LOWER_D / 2.0, -LOWER_T / 2.0)),
    )

    upper = model.part("upper_body")
    upper.visual(
        Box((UPPER_W, UPPER_D, UPPER_T)),
        origin=Origin(xyz=(0.0, UPPER_REAR + UPPER_D / 2.0, CLOSED_GAP + UPPER_T / 2.0)),
        material=shell,
        name="upper_shell",
    )
    upper.visual(
        Cylinder(radius=0.0066, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=X_AXIS_CYLINDER),
        material=shell_dark,
        name="upper_center_barrel",
    )
    upper.visual(
        Box((0.020, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.006, 0.005)),
        material=shell_dark,
        name="upper_spine_bridge",
    )

    screen_center_y = UPPER_REAR + 0.038
    screen_w = 0.047
    screen_h = 0.046
    bezel = 0.0045
    bezel_t = 0.0015
    bezel_z = CLOSED_GAP + bezel_t / 2.0
    upper.visual(
        Box((bezel, screen_h + 2.0 * bezel, bezel_t)),
        origin=Origin(xyz=(-(screen_w / 2.0 + bezel / 2.0), screen_center_y, bezel_z)),
        material=shell_dark,
        name="screen_bezel_left",
    )
    upper.visual(
        Box((bezel, screen_h + 2.0 * bezel, bezel_t)),
        origin=Origin(xyz=((screen_w / 2.0 + bezel / 2.0), screen_center_y, bezel_z)),
        material=shell_dark,
        name="screen_bezel_right",
    )
    upper.visual(
        Box((screen_w + 2.0 * bezel, bezel, bezel_t)),
        origin=Origin(xyz=(0.0, screen_center_y - (screen_h / 2.0 + bezel / 2.0), bezel_z)),
        material=shell_dark,
        name="screen_bezel_top",
    )
    upper.visual(
        Box((screen_w + 2.0 * bezel, bezel, bezel_t)),
        origin=Origin(xyz=(0.0, screen_center_y + (screen_h / 2.0 + bezel / 2.0), bezel_z)),
        material=shell_dark,
        name="screen_bezel_bottom",
    )
    upper.visual(
        Box((screen_w, screen_h, 0.0009)),
        origin=Origin(xyz=(0.0, screen_center_y, CLOSED_GAP + 0.00045)),
        material=screen_mat,
        name="lcd_panel",
    )
    upper.visual(
        Box((0.020, 0.0035, 0.0008)),
        origin=Origin(xyz=(0.0, UPPER_REAR + 0.012, CLOSED_GAP + 0.0004)),
        material=shell_dark,
        name="earpiece_slot",
    )
    upper.visual(
        Cylinder(radius=0.0044, length=0.0014),
        origin=Origin(xyz=(0.015, UPPER_REAR + 0.011, CLOSED_GAP + UPPER_T + 0.0007)),
        material=chrome,
        name="rear_camera_ring",
    )
    upper.visual(
        Cylinder(radius=0.0024, length=0.0018),
        origin=Origin(xyz=(0.015, UPPER_REAR + 0.011, CLOSED_GAP + UPPER_T + 0.0009)),
        material=lens_black,
        name="rear_camera_lens",
    )
    _add_corner_bumpers(
        upper,
        "upper",
        width=UPPER_W,
        depth=UPPER_D,
        thickness=UPPER_T,
        rear_offset=UPPER_REAR,
        z_center=CLOSED_GAP + UPPER_T / 2.0,
        material=rubber,
    )
    _add_grip_strips(
        upper,
        "upper",
        width=UPPER_W,
        depth=UPPER_D,
        thickness=UPPER_T,
        rear_offset=UPPER_REAR,
        z_center=CLOSED_GAP + UPPER_T / 2.0,
        material=rubber,
    )
    upper.inertial = Inertial.from_geometry(
        Box((UPPER_W, UPPER_D, UPPER_T)),
        mass=0.14,
        origin=Origin(xyz=(0.0, UPPER_REAR + UPPER_D / 2.0, CLOSED_GAP + UPPER_T / 2.0)),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=HINGE_OPEN_LIMIT,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_body")
    hinge = object_model.get_articulation("flip_hinge")

    lower_shell = lower.get_visual("lower_shell")
    control_deck = lower.get_visual("control_deck")
    antenna = lower.get_visual("antenna_stub")
    key_1 = lower.get_visual("key_1")
    key_5 = lower.get_visual("key_5")
    key_hash = lower.get_visual("key_hash")
    dpad_center = lower.get_visual("dpad_center")
    lower_left_grip = lower.get_visual("lower_left_grip_strip")
    lower_front_left_bumper = lower.get_visual("lower_front_left_bumper")
    left_barrel = lower.get_visual("left_barrel")
    right_barrel = lower.get_visual("right_barrel")
    left_pin = lower.get_visual("left_pivot_pin")
    right_pin = lower.get_visual("right_pivot_pin")
    upper_shell = upper.get_visual("upper_shell")
    upper_barrel = upper.get_visual("upper_center_barrel")
    upper_bridge = upper.get_visual("upper_spine_bridge")
    upper_left_grip = upper.get_visual("upper_left_grip_strip")
    upper_rear_right_bumper = upper.get_visual("upper_rear_right_bumper")
    bezel_left = upper.get_visual("screen_bezel_left")
    bezel_right = upper.get_visual("screen_bezel_right")
    bezel_top = upper.get_visual("screen_bezel_top")
    bezel_bottom = upper.get_visual("screen_bezel_bottom")
    lcd_panel = upper.get_visual("lcd_panel")
    camera_lens = upper.get_visual("rear_camera_lens")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(upper, lower, axes="xy", min_overlap=0.040)
    ctx.expect_origin_distance(upper, lower, axes="x", max_dist=0.003)
    ctx.expect_within(upper, lower, axes="x")
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=upper_shell,
        negative_elem=lower_shell,
        name="closed_shell_gap",
    )
    ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=left_barrel)
    ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=right_barrel)
    ctx.expect_contact(lower, lower, elem_a=left_pin, elem_b=left_barrel)
    ctx.expect_contact(lower, lower, elem_a=right_pin, elem_b=right_barrel)
    ctx.expect_contact(upper, upper, elem_a=upper_barrel, elem_b=upper_bridge)
    ctx.expect_contact(upper, upper, elem_a=upper_bridge, elem_b=upper_shell)
    ctx.expect_contact(lower, lower, elem_a=antenna, elem_b=lower_shell)
    ctx.expect_contact(lower, lower, elem_a=lower_left_grip, elem_b=lower_shell)
    ctx.expect_contact(upper, upper, elem_a=upper_left_grip, elem_b=upper_shell)
    ctx.expect_contact(lower, lower, elem_a=lower_front_left_bumper, elem_b=lower_shell)
    ctx.expect_contact(upper, upper, elem_a=upper_rear_right_bumper, elem_b=upper_shell)
    ctx.expect_contact(lower, lower, elem_a=key_1, elem_b=control_deck)
    ctx.expect_contact(lower, lower, elem_a=key_5, elem_b=control_deck)
    ctx.expect_contact(lower, lower, elem_a=key_hash, elem_b=control_deck)
    ctx.expect_contact(lower, lower, elem_a=dpad_center, elem_b=control_deck)
    ctx.expect_contact(upper, upper, elem_a=bezel_left, elem_b=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=bezel_right, elem_b=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=bezel_top, elem_b=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=bezel_bottom, elem_b=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=lcd_panel, elem_b=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=lcd_panel, elem_b=bezel_left)
    ctx.expect_contact(upper, upper, elem_a=lcd_panel, elem_b=bezel_right)
    ctx.expect_contact(upper, upper, elem_a=lcd_panel, elem_b=bezel_top)
    ctx.expect_contact(upper, upper, elem_a=lcd_panel, elem_b=bezel_bottom)
    ctx.expect_contact(upper, upper, elem_a=camera_lens, elem_b=upper_shell)

    with ctx.pose({hinge: HINGE_OPEN_LIMIT}):
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            min_gap=0.006,
            positive_elem=lower_shell,
            negative_elem=upper_shell,
            name="opened_halves_clear_in_depth",
        )
        ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=left_barrel)
        ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=right_barrel)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
