from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="learning_universal_remote")

    body_plastic = model.material("body_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    panel_smoke = model.material("panel_smoke", rgba=(0.28, 0.31, 0.34, 0.88))
    button_gray = model.material("button_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    button_dark = model.material("button_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.20, 0.42, 0.78, 1.0))
    display_green = model.material("display_green", rgba=(0.42, 0.60, 0.38, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.05, 0.05, 0.06, 1.0))

    small_button_mesh = _save_mesh(
        "remote_small_button",
        ExtrudeGeometry(rounded_rect_profile(0.008, 0.010, 0.0018), 0.0016),
    )
    rocker_button_mesh = _save_mesh(
        "remote_rocker_button",
        ExtrudeGeometry(rounded_rect_profile(0.013, 0.016, 0.0024), 0.0018),
    )
    learn_button_mesh = _save_mesh(
        "remote_learn_button",
        ExtrudeGeometry(rounded_rect_profile(0.0085, 0.009, 0.0018), 0.0016),
    )
    slider_plate_mesh = _save_mesh(
        "remote_slider_plate",
        ExtrudeGeometry(rounded_rect_profile(0.046, 0.072, 0.004), 0.0026),
    )
    side_button_mesh = _save_mesh(
        "remote_side_button",
        ExtrudeGeometry(rounded_rect_profile(0.022, 0.008, 0.0022), 0.0032).rotate_y(pi / 2.0),
    )

    body = model.part("body")
    body_shell = superellipse_side_loft(
        [
            (-0.108, 0.000, 0.010, 0.028),
            (-0.078, 0.000, 0.017, 0.048),
            (-0.020, 0.000, 0.021, 0.058),
            (0.036, 0.000, 0.0225, 0.060),
            (0.082, 0.000, 0.0185, 0.054),
            (0.108, 0.000, 0.0105, 0.036),
        ],
        exponents=2.5,
        segments=56,
    )
    body.visual(
        _save_mesh("remote_body_shell", body_shell),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.022, 0.014, 0.0016)),
        origin=Origin(xyz=(0.0, -0.088, 0.0158)),
        material=sensor_black,
        name="ir_window",
    )
    body.visual(
        Box((0.038, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, -0.048, 0.0189)),
        material=button_dark,
        name="display_bezel",
    )
    body.visual(
        Box((0.032, 0.018, 0.0010)),
        origin=Origin(xyz=(0.0, -0.048, 0.0196)),
        material=display_green,
        name="lcd_screen",
    )
    body.visual(
        Box((0.036, 0.052, 0.0013)),
        origin=Origin(xyz=(0.0, -0.001, 0.0192)),
        material=button_dark,
        name="learn_button_tray",
    )
    body.visual(
        Box((0.004, 0.100, 0.0024)),
        origin=Origin(xyz=(-0.019, 0.002, 0.0200)),
        material=button_dark,
        name="left_rail",
    )
    body.visual(
        Box((0.004, 0.100, 0.0024)),
        origin=Origin(xyz=(0.019, 0.002, 0.0200)),
        material=button_dark,
        name="right_rail",
    )
    body.visual(
        Box((0.028, 0.006, 0.0018)),
        origin=Origin(xyz=(0.0, -0.047, 0.0197)),
        material=button_dark,
        name="rail_front_stop",
    )
    body.visual(
        Box((0.028, 0.008, 0.0018)),
        origin=Origin(xyz=(0.0, 0.051, 0.0197)),
        material=button_dark,
        name="rail_rear_stop",
    )

    for name, x, y in (
        ("learn_button_front_left", -0.0075, -0.010),
        ("learn_button_front_right", 0.0075, -0.010),
        ("learn_button_rear_left", -0.0075, 0.008),
        ("learn_button_rear_right", 0.0075, 0.008),
    ):
        body.visual(
            learn_button_mesh,
            origin=Origin(xyz=(x, y, 0.0198)),
            material=accent_blue,
            name=name,
        )

    body.visual(
        Cylinder(radius=0.0135, length=0.0018),
        origin=Origin(xyz=(0.0, 0.044, 0.0198)),
        material=button_dark,
        name="nav_ring",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.0020),
        origin=Origin(xyz=(0.0, 0.044, 0.0199)),
        material=button_gray,
        name="nav_center",
    )
    body.visual(
        rocker_button_mesh,
        origin=Origin(xyz=(-0.014, 0.068, 0.0197)),
        material=button_gray,
        name="volume_button",
    )
    body.visual(
        rocker_button_mesh,
        origin=Origin(xyz=(0.014, 0.068, 0.0197)),
        material=button_gray,
        name="channel_button",
    )
    body.visual(
        Box((0.044, 0.058, 0.0014)),
        origin=Origin(xyz=(0.0, 0.104, 0.0190)),
        material=button_dark,
        name="keypad_pad",
    )

    keypad_rows = (
        ("key_1", "key_2", "key_3"),
        ("key_4", "key_5", "key_6"),
        ("key_7", "key_8", "key_9"),
        ("star_key", "key_0", "hash_key"),
    )
    row_ys = (0.086, 0.098, 0.110, 0.122)
    col_xs = (-0.015, 0.0, 0.015)
    for row_names, y in zip(keypad_rows, row_ys):
        for key_name, x in zip(row_names, col_xs):
            body.visual(
                small_button_mesh,
                origin=Origin(xyz=(x, y, 0.0197)),
                material=button_gray,
                name=key_name,
            )

    body.visual(
        Box((0.0070, 0.0018, 0.0225)),
        origin=Origin(xyz=(-0.0335, 0.0271, 0.0105)),
        material=button_dark,
        name="side_button_front_guide",
    )
    body.visual(
        Box((0.0070, 0.0018, 0.0225)),
        origin=Origin(xyz=(-0.0335, 0.0369, 0.0105)),
        material=button_dark,
        name="side_button_rear_guide",
    )
    body.visual(
        Box((0.0070, 0.0100, 0.0012)),
        origin=Origin(xyz=(-0.0335, 0.0320, 0.0221)),
        material=button_dark,
        name="side_button_top_guide",
    )
    body.visual(
        Box((0.0070, 0.0100, 0.0010)),
        origin=Origin(xyz=(-0.0335, 0.0320, 0.0000)),
        material=button_dark,
        name="side_button_bottom_guide",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.066, 0.220, 0.024)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    slide_panel = model.part("slide_panel")
    slide_panel.visual(
        slider_plate_mesh,
        material=panel_smoke,
        name="panel_plate",
    )
    slide_panel.visual(
        Box((0.006, 0.064, 0.0010)),
        origin=Origin(xyz=(-0.019, 0.0, -0.0013)),
        material=button_dark,
        name="left_glide",
    )
    slide_panel.visual(
        Box((0.006, 0.064, 0.0010)),
        origin=Origin(xyz=(0.019, 0.0, -0.0013)),
        material=button_dark,
        name="right_glide",
    )
    slide_panel.visual(
        Box((0.018, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.020, 0.0019)),
        material=button_gray,
        name="thumb_rib",
    )
    slide_panel.inertial = Inertial.from_geometry(
        Box((0.048, 0.074, 0.004)),
        mass=0.04,
    )

    model.articulation(
        "body_to_slide_panel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slide_panel,
        origin=Origin(xyz=(0.0, -0.002, 0.0233)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    side_button = model.part("side_button")
    side_button.visual(
        side_button_mesh,
        material=button_gray,
        name="side_button_cap",
    )
    side_button.inertial = Inertial.from_geometry(
        Box((0.0035, 0.024, 0.009)),
        mass=0.005,
    )

    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(-0.0356, 0.032, 0.0105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=0.0011,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    slide_panel = object_model.get_part("slide_panel")
    side_button = object_model.get_part("side_button")
    slide_joint = object_model.get_articulation("body_to_slide_panel")
    side_joint = object_model.get_articulation("body_to_side_button")

    left_rail = body.get_visual("left_rail")
    right_rail = body.get_visual("right_rail")
    rear_learn_button = body.get_visual("learn_button_rear_right")
    side_front_guide = body.get_visual("side_button_front_guide")
    panel_plate = slide_panel.get_visual("panel_plate")
    left_glide = slide_panel.get_visual("left_glide")
    right_glide = slide_panel.get_visual("right_glide")
    side_button_cap = side_button.get_visual("side_button_cap")

    ctx.expect_overlap(
        slide_panel,
        body,
        axes="xy",
        elem_a=panel_plate,
        elem_b=rear_learn_button,
        min_overlap=0.006,
        name="slide panel covers learning button cluster at rest",
    )
    ctx.expect_overlap(
        slide_panel,
        body,
        axes="y",
        elem_a=left_glide,
        elem_b=left_rail,
        min_overlap=0.060,
        name="left glide remains engaged with left rail at rest",
    )
    ctx.expect_overlap(
        slide_panel,
        body,
        axes="y",
        elem_a=right_glide,
        elem_b=right_rail,
        min_overlap=0.060,
        name="right glide remains engaged with right rail at rest",
    )
    ctx.expect_contact(
        side_button,
        body,
        elem_a=side_button_cap,
        elem_b=side_front_guide,
        name="side button is captured by its guide frame at rest",
    )

    slider_rest = ctx.part_world_position(slide_panel)
    button_rest = ctx.part_world_position(side_button)
    slide_upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else None
    side_upper = side_joint.motion_limits.upper if side_joint.motion_limits is not None else None

    with ctx.pose({slide_joint: slide_upper}):
        ctx.expect_gap(
            slide_panel,
            body,
            axis="y",
            positive_elem=panel_plate,
            negative_elem=rear_learn_button,
            min_gap=0.004,
            name="open slide panel clears the learning buttons",
        )
        ctx.expect_overlap(
            slide_panel,
            body,
            axes="y",
            elem_a=left_glide,
            elem_b=left_rail,
            min_overlap=0.030,
            name="left glide retains insertion when the panel is open",
        )
        ctx.expect_overlap(
            slide_panel,
            body,
            axes="y",
            elem_a=right_glide,
            elem_b=right_rail,
            min_overlap=0.030,
            name="right glide retains insertion when the panel is open",
        )
        slider_open = ctx.part_world_position(slide_panel)

    ctx.check(
        "slide panel moves toward the remote tail when opened",
        slider_rest is not None and slider_open is not None and slider_open[1] > slider_rest[1] + 0.045,
        details=f"rest={slider_rest}, open={slider_open}",
    )

    with ctx.pose({side_joint: side_upper}):
        ctx.expect_contact(
            side_button,
            body,
            elem_a=side_button_cap,
            elem_b=side_front_guide,
            name="pressed side button stays guided by the frame",
        )
        button_pressed = ctx.part_world_position(side_button)

    ctx.check(
        "side button presses inward",
        button_rest is not None and button_pressed is not None and button_pressed[0] > button_rest[0] + 0.0008,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
