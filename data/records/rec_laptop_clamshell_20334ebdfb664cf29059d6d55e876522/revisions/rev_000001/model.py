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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_profile(
    center_x: float,
    center_y: float,
    width: float,
    height: float,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (center_x - half_w, center_y - half_h),
        (center_x + half_w, center_y - half_h),
        (center_x + half_w, center_y + half_h),
        (center_x - half_w, center_y + half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_laptop")

    chassis_dark = model.material("chassis_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    deck_dark = model.material("deck_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    key_black = model.material("key_black", rgba=(0.07, 0.08, 0.09, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.14, 0.20, 0.24, 0.92))
    touchpad_grey = model.material("touchpad_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    shutter_grey = model.material("shutter_grey", rgba=(0.42, 0.43, 0.45, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.02, 0.02, 0.03, 1.0))

    base_width = 0.320
    base_depth = 0.220
    side_wall = 0.004
    shell_height = 0.016
    deck_thickness = 0.002
    key_travel = 0.0015
    key_stem = 0.010

    row_y_positions = (0.040, 0.004, -0.032)
    row_x_offsets = (0.000, 0.004, 0.008)
    col_x_positions = (-0.115, -0.069, -0.023, 0.023, 0.069, 0.115)
    key_specs: list[tuple[str, float, float]] = []
    hole_profiles: list[list[tuple[float, float]]] = []
    for row_index, row_y in enumerate(row_y_positions):
        for col_index, col_x in enumerate(col_x_positions):
            key_x = col_x + row_x_offsets[row_index]
            key_name = f"key_r{row_index}_c{col_index}"
            key_specs.append((key_name, key_x, row_y))
            hole_profiles.append(_rect_profile(key_x, row_y, key_stem, key_stem))

    deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.312, 0.212, 0.010, corner_segments=10),
            hole_profiles,
            deck_thickness,
            center=True,
            cap=True,
            closed=True,
        ),
        "business_laptop_keyboard_deck",
    )

    base = model.part("base")
    base.visual(
        Box((base_width, base_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=chassis_dark,
        name="bottom_shell",
    )
    base.visual(
        Box((base_width, side_wall, shell_height)),
        origin=Origin(xyz=(0.0, -(base_depth * 0.5 - side_wall * 0.5), shell_height * 0.5)),
        material=chassis_dark,
        name="front_rim",
    )
    base.visual(
        Box((base_width, side_wall, shell_height)),
        origin=Origin(xyz=(0.0, base_depth * 0.5 - side_wall * 0.5, shell_height * 0.5)),
        material=chassis_dark,
        name="rear_rim",
    )
    base.visual(
        Box((side_wall, base_depth - 2.0 * side_wall, shell_height)),
        origin=Origin(xyz=(-(base_width * 0.5 - side_wall * 0.5), 0.0, shell_height * 0.5)),
        material=chassis_dark,
        name="left_rim",
    )
    base.visual(
        Box((side_wall, base_depth - 2.0 * side_wall, shell_height)),
        origin=Origin(xyz=(base_width * 0.5 - side_wall * 0.5, 0.0, shell_height * 0.5)),
        material=chassis_dark,
        name="right_rim",
    )
    base.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=deck_dark,
        name="deck_panel",
    )
    base.visual(
        Box((0.220, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, 0.108, 0.015)),
        material=chassis_dark,
        name="hinge_support",
    )
    base.visual(
        Box((0.106, 0.070, 0.0008)),
        origin=Origin(xyz=(0.0, -0.068, 0.0164)),
        material=touchpad_grey,
        name="touchpad",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, 0.024)),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    lid_width = 0.318
    lid_height = 0.215
    lid = model.part("lid")
    lid.visual(
        Box((lid_width, 0.002, lid_height)),
        origin=Origin(xyz=(0.0, -0.007, lid_height * 0.5)),
        material=chassis_dark,
        name="back_cover",
    )
    lid.visual(
        Box((0.012, 0.008, lid_height)),
        origin=Origin(xyz=(-(lid_width * 0.5 - 0.006), -0.004, lid_height * 0.5)),
        material=chassis_dark,
        name="left_bezel",
    )
    lid.visual(
        Box((0.012, 0.008, lid_height)),
        origin=Origin(xyz=(lid_width * 0.5 - 0.006, -0.004, lid_height * 0.5)),
        material=chassis_dark,
        name="right_bezel",
    )
    lid.visual(
        Box((0.294, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, lid_height - 0.007)),
        material=chassis_dark,
        name="top_bezel",
    )
    lid.visual(
        Box((0.294, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.004, 0.012)),
        material=chassis_dark,
        name="bottom_bezel",
    )
    lid.visual(
        Box((0.028, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, -0.001, 0.204)),
        material=sensor_black,
        name="camera_strip",
    )
    lid.visual(
        Cylinder(radius=0.003, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=chassis_dark,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.032, 0.006, 0.008)),
        origin=Origin(xyz=(-0.082, -0.001, 0.005)),
        material=chassis_dark,
        name="left_hinge_tab",
    )
    lid.visual(
        Box((0.032, 0.006, 0.008)),
        origin=Origin(xyz=(0.082, -0.001, 0.005)),
        material=chassis_dark,
        name="right_hinge_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, 0.012, lid_height)),
        mass=0.82,
        origin=Origin(xyz=(0.0, -0.004, lid_height * 0.5)),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.294, 0.0015, 0.177)),
        material=screen_glass,
        name="display_panel",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.294, 0.0015, 0.177)),
        mass=0.12,
        origin=Origin(),
    )

    shutter = model.part("webcam_shutter")
    shutter.visual(
        Box((0.018, 0.0012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0006, 0.0)),
        material=shutter_grey,
        name="shutter_plate",
    )
    shutter.visual(
        Box((0.004, 0.003, 0.003)),
        origin=Origin(xyz=(0.0065, 0.0015, 0.0)),
        material=shutter_grey,
        name="shutter_tab",
    )
    shutter.inertial = Inertial.from_geometry(
        Box((0.022, 0.003, 0.008)),
        mass=0.003,
        origin=Origin(xyz=(0.002, 0.0015, 0.0)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.108, 0.019), rpy=(0.35, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=-1.10,
            upper=0.55,
        ),
    )
    model.articulation(
        "lid_to_screen",
        ArticulationType.FIXED,
        parent=lid,
        child=screen,
        origin=Origin(xyz=(0.0, -0.0015, 0.1125)),
    )
    model.articulation(
        "lid_to_webcam_shutter",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=shutter,
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=0.05,
            lower=0.0,
            upper=0.014,
        ),
    )

    for key_name, key_x, key_y in key_specs:
        key = model.part(key_name)
        key.visual(
            Box((0.034, 0.030, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.00325)),
            material=key_black,
            name="keycap",
        )
        key.visual(
            Box((key_stem, key_stem, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            material=key_black,
            name="plunger",
        )
        key.inertial = Inertial.from_geometry(
            Box((0.034, 0.030, 0.013)),
            mass=0.007,
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
        )
        model.articulation(
            f"base_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(key_x, key_y, 0.016)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.04,
                lower=0.0,
                upper=key_travel,
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

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    screen = object_model.get_part("screen")
    shutter = object_model.get_part("webcam_shutter")
    center_key = object_model.get_part("key_r1_c2")

    lid_hinge = object_model.get_articulation("base_to_lid")
    shutter_slide = object_model.get_articulation("lid_to_webcam_shutter")
    key_plunger = object_model.get_articulation("base_to_key_r1_c2")

    ctx.expect_contact(screen, lid, name="screen is mounted against the lid structure")
    ctx.expect_contact(shutter, lid, name="webcam shutter rides directly on the lid bezel")
    ctx.expect_contact(center_key, base, name="keyboard key remains guided by the deck opening")

    rest_screen_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({lid_hinge: 0.35}):
        more_open_screen_aabb = ctx.part_world_aabb(screen)
    ctx.check(
        "lid positive motion opens the display farther",
        rest_screen_aabb is not None
        and more_open_screen_aabb is not None
        and more_open_screen_aabb[1][2] > rest_screen_aabb[1][2] + 0.008,
        details=f"rest={rest_screen_aabb}, more_open={more_open_screen_aabb}",
    )

    rest_shutter_pos = ctx.part_world_position(shutter)
    with ctx.pose({shutter_slide: 0.012}):
        open_shutter_pos = ctx.part_world_position(shutter)
    ctx.check(
        "privacy shutter slides laterally across the top bezel",
        rest_shutter_pos is not None
        and open_shutter_pos is not None
        and open_shutter_pos[0] > rest_shutter_pos[0] + 0.010,
        details=f"rest={rest_shutter_pos}, open={open_shutter_pos}",
    )

    rest_key_pos = ctx.part_world_position(center_key)
    with ctx.pose({key_plunger: key_plunger.motion_limits.upper}):
        pressed_key_pos = ctx.part_world_position(center_key)
        ctx.expect_gap(
            center_key,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.0005,
            positive_elem="keycap",
            negative_elem="deck_panel",
            name="pressed keycap nearly seats into the deck",
        )
    ctx.check(
        "key press drives the key downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.001,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
