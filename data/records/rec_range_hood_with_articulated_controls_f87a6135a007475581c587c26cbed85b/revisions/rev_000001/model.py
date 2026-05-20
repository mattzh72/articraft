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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_range_hood")

    enamel = model.material("appliance_white", rgba=(0.93, 0.93, 0.91, 1.0))
    trim = model.material("trim_white", rgba=(0.86, 0.86, 0.84, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    lens = model.material("lens_frosted", rgba=(0.97, 0.97, 0.92, 0.78))
    button_plastic = model.material("button_plastic", rgba=(0.80, 0.80, 0.77, 1.0))
    dark_internal = model.material("dark_internal", rgba=(0.18, 0.19, 0.20, 1.0))

    hood_width = 0.60
    hood_depth = 0.48
    hood_height = 0.12
    shell_t = 0.003

    front_y = hood_depth * 0.5 - shell_t * 0.5
    rear_y = -front_y
    side_x = hood_width * 0.5 - shell_t * 0.5

    canopy = model.part("canopy")
    canopy.visual(
        Box((hood_width, hood_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, hood_height - 0.002)),
        material=enamel,
        name="top_panel",
    )
    canopy.visual(
        Box((shell_t, hood_depth, hood_height)),
        origin=Origin(xyz=(-side_x, 0.0, hood_height * 0.5)),
        material=enamel,
        name="left_side_panel",
    )
    canopy.visual(
        Box((shell_t, hood_depth, hood_height)),
        origin=Origin(xyz=(side_x, 0.0, hood_height * 0.5)),
        material=enamel,
        name="right_side_panel",
    )
    canopy.visual(
        Box((hood_width - 0.006, shell_t, hood_height)),
        origin=Origin(xyz=(0.0, rear_y, hood_height * 0.5)),
        material=enamel,
        name="rear_panel",
    )

    canopy.visual(
        Box((hood_width - 0.006, shell_t, 0.020)),
        origin=Origin(xyz=(0.0, front_y, 0.010)),
        material=enamel,
        name="front_bottom_band",
    )
    canopy.visual(
        Box((hood_width - 0.006, shell_t, 0.028)),
        origin=Origin(xyz=(0.0, front_y, 0.034)),
        material=enamel,
        name="front_mid_lower_band",
    )
    canopy.visual(
        Box((hood_width - 0.006, shell_t, 0.042)),
        origin=Origin(xyz=(0.0, front_y, 0.097)),
        material=enamel,
        name="front_upper_band",
    )
    canopy.visual(
        Box((0.355, shell_t, 0.028)),
        origin=Origin(xyz=(-0.1195, front_y, 0.062)),
        material=enamel,
        name="front_button_band_left",
    )
    canopy.visual(
        Box((0.012, shell_t, 0.028)),
        origin=Origin(xyz=(0.100, front_y, 0.062)),
        material=enamel,
        name="front_button_band_center",
    )
    canopy.visual(
        Box((0.152, shell_t, 0.028)),
        origin=Origin(xyz=(0.218, front_y, 0.062)),
        material=enamel,
        name="front_button_band_right",
    )

    canopy.visual(
        Box((hood_width - 0.006, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.222, 0.006)),
        material=trim,
        name="underside_front_lip",
    )
    canopy.visual(
        Box((hood_width - 0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.136, 0.005)),
        material=trim,
        name="light_cover_back_rail",
    )
    canopy.visual(
        Box((0.223, 0.056, 0.010)),
        origin=Origin(xyz=(-0.1855, 0.173, 0.005)),
        material=trim,
        name="light_cover_left_rail",
    )
    canopy.visual(
        Box((0.223, 0.056, 0.010)),
        origin=Origin(xyz=(0.1855, 0.173, 0.005)),
        material=trim,
        name="light_cover_right_rail",
    )
    canopy.visual(
        Box((0.016, 0.008, 0.008)),
        origin=Origin(xyz=(-0.050, 0.202, 0.004)),
        material=trim,
        name="light_cover_left_hinge_lug",
    )
    canopy.visual(
        Box((0.016, 0.008, 0.008)),
        origin=Origin(xyz=(0.050, 0.202, 0.004)),
        material=trim,
        name="light_cover_right_hinge_lug",
    )
    canopy.visual(
        Box((hood_width - 0.012, 0.362, 0.003)),
        origin=Origin(xyz=(0.0, -0.042, 0.0015)),
        material=filter_metal,
        name="underside_filter",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((hood_width, hood_depth, hood_height)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, hood_height * 0.5)),
    )

    cover_width = 0.138
    cover_depth = 0.055
    cover_thickness = 0.004

    light_cover = model.part("light_cover")
    light_cover.visual(
        Box((cover_width, cover_depth, cover_thickness)),
        origin=Origin(xyz=(0.0, -cover_depth * 0.5, -0.004)),
        material=lens,
        name="cover_lens",
    )
    light_cover.visual(
        Box((cover_width, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.003, -0.0005)),
        material=trim,
        name="cover_frame_front",
    )
    light_cover.visual(
        Box((cover_width, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -cover_depth + 0.002, -0.0005)),
        material=trim,
        name="cover_frame_back",
    )
    light_cover.visual(
        Box((0.006, cover_depth - 0.008, 0.006)),
        origin=Origin(xyz=(-cover_width * 0.5 + 0.003, -cover_depth * 0.5, -0.0005)),
        material=trim,
        name="cover_frame_left",
    )
    light_cover.visual(
        Box((0.006, cover_depth - 0.008, 0.006)),
        origin=Origin(xyz=(cover_width * 0.5 - 0.003, -cover_depth * 0.5, -0.0005)),
        material=trim,
        name="cover_frame_right",
    )
    light_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_depth, 0.008)),
        mass=0.20,
        origin=Origin(xyz=(0.0, -cover_depth * 0.5, -0.002)),
    )

    model.articulation(
        "canopy_to_light_cover",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=light_cover,
        origin=Origin(xyz=(0.0, 0.198, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    button_cap_size = (0.024, 0.006, 0.014)
    button_plunger_size = (0.010, 0.018, 0.010)
    button_travel = 0.003

    for part_name, joint_name, button_x in (
        ("fan_button", "canopy_to_fan_button", 0.076),
        ("light_button", "canopy_to_light_button", 0.124),
    ):
        button = model.part(part_name)
        button.visual(
            Box(button_cap_size),
            origin=Origin(xyz=(0.0, button_cap_size[1] * 0.5, 0.0)),
            material=button_plastic,
            name="button_cap",
        )
        button.visual(
            Box(button_plunger_size),
            origin=Origin(xyz=(0.0, -button_plunger_size[1] * 0.5, 0.0)),
            material=dark_internal,
            name="button_plunger",
        )
        button.visual(
            Cylinder(radius=0.004, length=button_plunger_size[1] - 0.002),
            origin=Origin(
                xyz=(0.0, -0.008, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=dark_internal,
            name="button_stem",
        )
        button.visual(
            Box((0.044, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.001, 0.016)),
            material=dark_internal,
            name="button_stop_top",
        )
        button.visual(
            Box((0.044, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.001, -0.016)),
            material=dark_internal,
            name="button_stop_bottom",
        )
        button.visual(
            Box((0.004, 0.002, 0.032)),
            origin=Origin(xyz=(-0.020, -0.001, 0.0)),
            material=dark_internal,
            name="button_stop_left",
        )
        button.visual(
            Box((0.004, 0.002, 0.032)),
            origin=Origin(xyz=(0.020, -0.001, 0.0)),
            material=dark_internal,
            name="button_stop_right",
        )
        button.visual(
            Box((0.044, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.001, 0.0)),
            material=dark_internal,
            name="button_stop_crossbar",
        )
        button.visual(
            Box((0.004, 0.002, 0.032)),
            origin=Origin(xyz=(0.0, -0.001, 0.0)),
            material=dark_internal,
            name="button_stop_spine",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.024, 0.024, 0.016)),
            mass=0.03,
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(button_x, 0.237, 0.062)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=button_travel,
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

    canopy = object_model.get_part("canopy")
    light_cover = object_model.get_part("light_cover")
    fan_button = object_model.get_part("fan_button")
    light_button = object_model.get_part("light_button")

    cover_hinge = object_model.get_articulation("canopy_to_light_cover")
    fan_button_joint = object_model.get_articulation("canopy_to_fan_button")
    light_button_joint = object_model.get_articulation("canopy_to_light_button")

    ctx.expect_overlap(
        light_cover,
        canopy,
        axes="x",
        min_overlap=0.12,
        name="light cover spans a believable portion of the hood underside",
    )
    ctx.expect_origin_distance(
        fan_button,
        light_button,
        axes="x",
        min_dist=0.040,
        max_dist=0.060,
        name="front push buttons sit side by side",
    )

    cover_closed = ctx.part_element_world_aabb(light_cover, elem="cover_lens")
    front_face = ctx.part_element_world_aabb(canopy, elem="front_upper_band")
    fan_cap_rest = ctx.part_element_world_aabb(fan_button, elem="button_cap")
    light_cap_rest = ctx.part_element_world_aabb(light_button, elem="button_cap")

    ctx.check(
        "light cover stores flush under the canopy",
        cover_closed is not None
        and abs(cover_closed[0][2]) <= 0.001
        and cover_closed[1][2] <= 0.0055
        and cover_closed[1][1] < 0.205,
        details=f"cover_closed={cover_closed}",
    )
    ctx.check(
        "buttons protrude slightly from the front face at rest",
        front_face is not None
        and fan_cap_rest is not None
        and light_cap_rest is not None
        and fan_cap_rest[1][1] > front_face[1][1] + 0.0015
        and light_cap_rest[1][1] > front_face[1][1] + 0.0015,
        details=(
            f"front_face={front_face}, fan_cap_rest={fan_cap_rest}, "
            f"light_cap_rest={light_cap_rest}"
        ),
    )

    cover_open = None
    fan_pressed_pos = None
    light_pressed_pos = None
    fan_cap_pressed = None
    with ctx.pose(
        {
            cover_hinge: 1.15,
            fan_button_joint: 0.003,
            light_button_joint: 0.003,
        }
    ):
        cover_open = ctx.part_element_world_aabb(light_cover, elem="cover_lens")
        fan_pressed_pos = ctx.part_world_position(fan_button)
        light_pressed_pos = ctx.part_world_position(light_button)
        fan_cap_pressed = ctx.part_element_world_aabb(fan_button, elem="button_cap")

    fan_rest_pos = ctx.part_world_position(fan_button)
    light_rest_pos = ctx.part_world_position(light_button)

    ctx.check(
        "light cover flips downward when opened",
        cover_closed is not None
        and cover_open is not None
        and cover_open[0][2] < cover_closed[0][2] - 0.035,
        details=f"cover_closed={cover_closed}, cover_open={cover_open}",
    )
    ctx.check(
        "fan button presses inward",
        fan_rest_pos is not None
        and fan_pressed_pos is not None
        and fan_pressed_pos[1] < fan_rest_pos[1] - 0.0025,
        details=f"fan_rest_pos={fan_rest_pos}, fan_pressed_pos={fan_pressed_pos}",
    )
    ctx.check(
        "light button presses inward",
        light_rest_pos is not None
        and light_pressed_pos is not None
        and light_pressed_pos[1] < light_rest_pos[1] - 0.0025,
        details=f"light_rest_pos={light_rest_pos}, light_pressed_pos={light_pressed_pos}",
    )
    ctx.check(
        "pressed button sits nearly flush with the fascia",
        front_face is not None
        and fan_cap_pressed is not None
        and fan_cap_pressed[1][1] <= front_face[1][1] + 0.0006,
        details=f"front_face={front_face}, fan_cap_pressed={fan_cap_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
