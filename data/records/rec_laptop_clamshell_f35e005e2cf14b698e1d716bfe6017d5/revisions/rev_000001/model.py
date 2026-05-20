from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _rotate_x(angle: float, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = xyz
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, (y * c) - (z * s), (y * s) + (z * c))


def _lid_origin(
    closed_xyz: tuple[float, float, float],
    *,
    rest_angle: float,
    rpy: tuple[float, float, float] | None = None,
) -> Origin:
    visual_rpy = (-rest_angle, 0.0, 0.0) if rpy is None else rpy
    return Origin(xyz=_rotate_x(-rest_angle, closed_xyz), rpy=visual_rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ultraportable_laptop")

    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.56, 0.59, 0.63, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    key_black = model.material("key_black", rgba=(0.12, 0.13, 0.14, 1.0))
    screen_black = model.material("screen_black", rgba=(0.05, 0.07, 0.09, 1.0))
    trackpad_grey = model.material("trackpad_grey", rgba=(0.68, 0.70, 0.73, 1.0))

    base_w = 0.302
    base_d = 0.210
    hinge_y = (base_d * 0.5) - 0.010
    hinge_z = 0.0104

    lid_w = 0.296
    lid_h = 0.198
    lid_t = 0.0064
    lid_back_t = 0.0032
    lid_frame_t = lid_t - lid_back_t
    screen_side_bezel = 0.006
    screen_top_bezel = 0.005
    screen_bottom_bezel = 0.011
    screen_w = lid_w - (2.0 * screen_side_bezel)
    screen_h = lid_h - screen_top_bezel - screen_bottom_bezel
    rest_open_angle = math.radians(106.0)

    chassis = model.part("lower_chassis")
    chassis_mesh = mesh_from_geometry(
        superellipse_side_loft(
            [
                (-base_d * 0.5, 0.0, 0.0084, 0.294),
                (-0.020, 0.0, 0.0088, 0.299),
                (0.060, 0.0, 0.0092, 0.301),
                (base_d * 0.5, 0.0, 0.0096, 0.302),
            ],
            exponents=4.2,
            segments=64,
            cap=True,
        ),
        "ultraportable_lower_chassis",
    )
    chassis.visual(chassis_mesh, material=aluminum, name="chassis_body")
    chassis.visual(
        Box((0.224, 0.110, 0.0020)),
        origin=Origin(xyz=(0.0, -0.002, 0.0071)),
        material=graphite,
        name="keyboard_deck",
    )
    chassis.visual(
        Box((0.112, 0.072, 0.0018)),
        origin=Origin(xyz=(0.0, -0.067, 0.0073)),
        material=trackpad_grey,
        name="trackpad",
    )
    chassis.visual(
        Box((0.013, 0.084, 0.0005)),
        origin=Origin(xyz=(-0.118, -0.002, 0.00805)),
        material=dark_aluminum,
        name="left_speaker_strip",
    )
    chassis.visual(
        Box((0.013, 0.084, 0.0005)),
        origin=Origin(xyz=(0.118, -0.002, 0.00805)),
        material=dark_aluminum,
        name="right_speaker_strip",
    )
    chassis.visual(
        Box((0.018, 0.010, 0.005)),
        origin=Origin(xyz=(-0.112, hinge_y - 0.001, 0.0071)),
        material=dark_aluminum,
        name="left_hinge_block",
    )
    chassis.visual(
        Box((0.018, 0.010, 0.005)),
        origin=Origin(xyz=(0.112, hinge_y - 0.001, 0.0071)),
        material=dark_aluminum,
        name="right_hinge_block",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((base_w, base_d, 0.010)),
        mass=1.20,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    lid = model.part("display_lid")
    lid.visual(
        Box((lid_w, lid_h, lid_back_t)),
        origin=_lid_origin((0.0, -lid_h * 0.5, lid_back_t * 0.5), rest_angle=rest_open_angle),
        material=aluminum,
        name="rear_shell",
    )
    lid.visual(
        Box((screen_w, screen_h, 0.0012)),
        origin=_lid_origin(
            (
                0.0,
                -(screen_bottom_bezel + (screen_h * 0.5)),
                lid_back_t + 0.0006,
            ),
            rest_angle=rest_open_angle,
        ),
        material=screen_black,
        name="screen_panel",
    )
    lid.visual(
        Box((screen_side_bezel, lid_h, lid_frame_t)),
        origin=_lid_origin(
            (
                -(lid_w * 0.5) + (screen_side_bezel * 0.5),
                -lid_h * 0.5,
                lid_back_t + (lid_frame_t * 0.5),
            ),
            rest_angle=rest_open_angle,
        ),
        material=graphite,
        name="left_bezel",
    )
    lid.visual(
        Box((screen_side_bezel, lid_h, lid_frame_t)),
        origin=_lid_origin(
            (
                (lid_w * 0.5) - (screen_side_bezel * 0.5),
                -lid_h * 0.5,
                lid_back_t + (lid_frame_t * 0.5),
            ),
            rest_angle=rest_open_angle,
        ),
        material=graphite,
        name="right_bezel",
    )
    lid.visual(
        Box((screen_w, screen_top_bezel, lid_frame_t)),
        origin=_lid_origin(
            (
                0.0,
                -(lid_h - (screen_top_bezel * 0.5)),
                lid_back_t + (lid_frame_t * 0.5),
            ),
            rest_angle=rest_open_angle,
        ),
        material=graphite,
        name="top_bezel",
    )
    lid.visual(
        Box((screen_w, screen_bottom_bezel, lid_frame_t)),
        origin=_lid_origin(
            (
                0.0,
                -(screen_bottom_bezel * 0.5),
                lid_back_t + (lid_frame_t * 0.5),
            ),
            rest_angle=rest_open_angle,
        ),
        material=graphite,
        name="bottom_bezel",
    )
    lid.visual(
        Box((0.014, 0.010, 0.003)),
        origin=_lid_origin((-0.112, -0.005, 0.0015), rest_angle=rest_open_angle),
        material=dark_aluminum,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.014, 0.010, 0.003)),
        origin=_lid_origin((0.112, -0.005, 0.0015), rest_angle=rest_open_angle),
        material=dark_aluminum,
        name="right_hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_h, lid_t)),
        mass=0.62,
        origin=_lid_origin((0.0, -lid_h * 0.5, lid_t * 0.5), rest_angle=rest_open_angle),
    )

    lid_joint = model.articulation(
        "chassis_to_display",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-rest_open_angle,
            upper=math.radians(22.0),
        ),
    )

    key_width = 0.0162
    key_depth = 0.0148
    key_height = 0.0011
    key_plunger_height = 0.0003
    key_origin_z = 0.0084
    key_travel = 0.00085
    key_pitch_x = 0.0190
    row_positions = (0.020, 0.000, -0.020)
    column_count = 10
    leftmost_x = -0.5 * (column_count - 1) * key_pitch_x

    for row_index, key_y in enumerate(row_positions):
        for col_index in range(column_count):
            key_x = leftmost_x + (col_index * key_pitch_x)
            key = model.part(f"key_r{row_index}_c{col_index}")
            key.visual(
                Box((key_width, key_depth, key_height)),
                origin=Origin(xyz=(0.0, 0.0, key_height * 0.5)),
                material=key_black,
                name="keycap",
            )
            key.visual(
                Box((0.0046, 0.0046, key_plunger_height)),
                origin=Origin(xyz=(0.0, 0.0, -0.5 * key_plunger_height)),
                material=graphite,
                name="plunger",
            )
            key.inertial = Inertial.from_geometry(
                Box((key_width, key_depth, key_height)),
                mass=0.0035,
                origin=Origin(xyz=(0.0, 0.0, key_height * 0.5)),
            )
            model.articulation(
                f"chassis_to_key_r{row_index}_c{col_index}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key,
                origin=Origin(xyz=(key_x, key_y, key_origin_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.4,
                    velocity=0.03,
                    lower=0.0,
                    upper=key_travel,
                ),
            )

    spacebar = model.part("spacebar")
    spacebar.visual(
        Box((0.104, 0.013, key_height)),
        origin=Origin(xyz=(0.0, 0.0, key_height * 0.5)),
        material=key_black,
        name="keycap",
    )
    spacebar.visual(
        Box((0.044, 0.0038, key_plunger_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * key_plunger_height)),
        material=graphite,
        name="plunger",
    )
    spacebar.inertial = Inertial.from_geometry(
        Box((0.104, 0.013, key_height)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, key_height * 0.5)),
    )
    model.articulation(
        "chassis_to_spacebar",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=spacebar,
        origin=Origin(xyz=(0.0, -0.043, key_origin_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=0.03,
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

    chassis = object_model.get_part("lower_chassis")
    lid = object_model.get_part("display_lid")
    lid_joint = object_model.get_articulation("chassis_to_display")
    sample_key = object_model.get_part("key_r1_c4")
    sample_key_joint = object_model.get_articulation("chassis_to_key_r1_c4")

    ctx.expect_gap(
        sample_key,
        chassis,
        axis="z",
        positive_elem="keycap",
        negative_elem="keyboard_deck",
        min_gap=0.00025,
        max_gap=0.00035,
        name="resting key sits slightly proud of the keyboard deck",
    )

    with ctx.pose({lid_joint: lid_joint.motion_limits.lower}):
        ctx.expect_overlap(
            lid,
            chassis,
            axes="xy",
            elem_a="rear_shell",
            elem_b="chassis_body",
            min_overlap=0.18,
            name="closed lid covers the lower chassis footprint",
        )
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            positive_elem="rear_shell",
            negative_elem="chassis_body",
            min_gap=0.0004,
            max_gap=0.0020,
            name="closed lid sits just above the lower chassis",
        )

    key_rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: sample_key_joint.motion_limits.upper}):
        key_pressed_pos = ctx.part_world_position(sample_key)

    ctx.check(
        "sample key travels downward when pressed",
        key_rest_pos is not None
        and key_pressed_pos is not None
        and key_pressed_pos[2] < key_rest_pos[2] - 0.0006,
        details=f"rest={key_rest_pos}, pressed={key_pressed_pos}",
    )

    def _y_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    screen_rest = ctx.part_element_world_aabb(lid, elem="screen_panel")
    with ctx.pose({lid_joint: 0.25}):
        screen_more_open = ctx.part_element_world_aabb(lid, elem="screen_panel")

    screen_rest_y = _y_center(screen_rest)
    screen_more_open_y = _y_center(screen_more_open)
    ctx.check(
        "display rotates rearward as the hinge opens",
        screen_rest_y is not None
        and screen_more_open_y is not None
        and screen_more_open_y > screen_rest_y + 0.012,
        details=f"rest_y={screen_rest_y}, more_open_y={screen_more_open_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
