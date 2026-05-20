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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_speaker")

    housing_color = model.material("housing_color", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_color = model.material("trim_color", rgba=(0.12, 0.13, 0.14, 1.0))
    grille_color = model.material("grille_color", rgba=(0.09, 0.10, 0.11, 1.0))
    hardware_color = model.material("hardware_color", rgba=(0.56, 0.58, 0.62, 1.0))
    accent_color = model.material("accent_color", rgba=(0.93, 0.50, 0.13, 1.0))

    width = 0.290
    depth = 0.180
    height = 0.220
    wall = 0.012

    pivot_z = height + 0.008
    handle_end_x = width / 2.0 + 0.021
    knob_z = 0.146
    button_x = 0.086
    button_z = 0.047

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=housing_color,
        name="bottom_wall",
    )
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=housing_color,
        name="top_wall",
    )
    housing.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(xyz=(-(width / 2.0) + wall / 2.0, 0.0, height / 2.0)),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(xyz=((width / 2.0) - wall / 2.0, 0.0, height / 2.0)),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -(depth / 2.0) + wall / 2.0, height / 2.0)),
        material=housing_color,
        name="rear_wall",
    )

    grille_open_width = 0.214
    grille_open_height = 0.126
    side_frame_width = ((width - 2.0 * wall) - grille_open_width) / 2.0
    grille_center_z = 0.134

    housing.visual(
        Box((side_frame_width, wall, grille_open_height)),
        origin=Origin(
            xyz=(
                -(grille_open_width / 2.0 + side_frame_width / 2.0),
                (depth / 2.0) - wall / 2.0,
                grille_center_z,
            )
        ),
        material=housing_color,
        name="front_frame_left",
    )
    housing.visual(
        Box((side_frame_width, wall, grille_open_height)),
        origin=Origin(
            xyz=(
                grille_open_width / 2.0 + side_frame_width / 2.0,
                (depth / 2.0) - wall / 2.0,
                grille_center_z,
            )
        ),
        material=housing_color,
        name="front_frame_right",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, 0.012)),
        origin=Origin(xyz=(0.0, (depth / 2.0) - wall / 2.0, 0.202)),
        material=housing_color,
        name="front_frame_top",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, 0.072)),
        origin=Origin(xyz=(0.0, (depth / 2.0) - wall / 2.0, 0.036)),
        material=housing_color,
        name="front_lower_panel",
    )
    housing.visual(
        Box((0.218, 0.006, 0.130)),
        origin=Origin(xyz=(0.0, (depth / 2.0) + 0.003, grille_center_z)),
        material=grille_color,
        name="front_grille",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((0.014, 0.026, 0.018)),
            origin=Origin(
                xyz=(side_sign * ((width / 2.0) + 0.007), 0.0, pivot_z - 0.016)
            ),
            material=housing_color,
            name=f"{side_name}_handle_bracket",
        )
        housing.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(side_sign * ((width / 2.0) + 0.011), 0.0, pivot_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hardware_color,
            name=f"{side_name}_pivot_cap",
        )

    housing.visual(
        Box((0.012, 0.070, 0.070)),
        origin=Origin(xyz=((width / 2.0) + 0.006, 0.0, knob_z)),
        material=trim_color,
        name="knob_pod",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(
            xyz=((width / 2.0) + 0.014, 0.0, knob_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hardware_color,
        name="knob_bezel",
    )

    guide_outer_w = 0.032
    guide_outer_h = 0.026
    guide_wall = 0.003
    guide_len = 0.018
    guide_center_y = (depth / 2.0) + (guide_len / 2.0)
    guide_inner_h = guide_outer_h - 2.0 * guide_wall

    housing.visual(
        Box((guide_outer_w, guide_len, guide_wall)),
        origin=Origin(
            xyz=(button_x, guide_center_y, button_z + (guide_outer_h / 2.0) - (guide_wall / 2.0))
        ),
        material=trim_color,
        name="button_guide_top",
    )
    housing.visual(
        Box((guide_outer_w, guide_len, guide_wall)),
        origin=Origin(
            xyz=(button_x, guide_center_y, button_z - (guide_outer_h / 2.0) + (guide_wall / 2.0))
        ),
        material=trim_color,
        name="button_guide_bottom",
    )
    housing.visual(
        Box((guide_wall, guide_len, guide_inner_h)),
        origin=Origin(
            xyz=(button_x - (guide_outer_w / 2.0) + (guide_wall / 2.0), guide_center_y, button_z)
        ),
        material=trim_color,
        name="button_guide_left",
    )
    housing.visual(
        Box((guide_wall, guide_len, guide_inner_h)),
        origin=Origin(
            xyz=(button_x + (guide_outer_w / 2.0) - (guide_wall / 2.0), guide_center_y, button_z)
        ),
        material=trim_color,
        name="button_guide_right",
    )

    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    handle = model.part("handle")
    handle_pivot_x = (width / 2.0) + 0.018
    handle_profile = tube_from_spline_points(
        [
            (-0.167, -0.013, 0.005),
            (-0.148, -0.034, 0.026),
            (-0.108, -0.056, 0.044),
            (0.000, -0.070, 0.054),
            (0.108, -0.056, 0.044),
            (0.148, -0.034, 0.026),
            (0.167, -0.013, 0.005),
        ],
        radius=0.0073,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        mesh_from_geometry(handle_profile, "speaker_handle_frame"),
        material=trim_color,
        name="handle_frame",
    )
    handle.visual(
        Box((0.112, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.060, 0.045)),
        material=trim_color,
        name="handle_grip",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        handle.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(
                xyz=(side_sign * handle_pivot_x, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hardware_color,
            name=f"{side_name}_handle_knuckle",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.340, 0.090, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.028, 0.020)),
    )

    knob = model.part("volume_knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_color,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_color,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, 0.018)),
        material=accent_color,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.032, 0.060, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    button = model.part("power_button")
    button.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=accent_color,
        name="button_cap",
    )
    button.visual(
        Box((0.026, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=hardware_color,
        name="button_stem",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.020, 0.024, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
    )

    model.articulation(
        "housing_to_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "housing_to_volume_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=((width / 2.0) + 0.016, 0.0, knob_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-2.4,
            upper=0.7,
        ),
    )
    model.articulation(
        "housing_to_power_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(xyz=(button_x, (depth / 2.0) + 0.017, button_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
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
    handle = object_model.get_part("handle")
    knob = object_model.get_part("volume_knob")
    button = object_model.get_part("power_button")

    handle_joint = object_model.get_articulation("housing_to_handle")
    knob_joint = object_model.get_articulation("housing_to_volume_knob")
    button_joint = object_model.get_articulation("housing_to_power_button")

    ctx.check(
        "speaker articulations are correctly typed",
        handle_joint.articulation_type == ArticulationType.REVOLUTE
        and knob_joint.articulation_type == ArticulationType.REVOLUTE
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"handle={handle_joint.articulation_type}, "
            f"knob={knob_joint.articulation_type}, "
            f"button={button_joint.articulation_type}"
        ),
    )
    ctx.check(
        "speaker articulation axes match the real controls",
        tuple(handle_joint.axis) == (-1.0, 0.0, 0.0)
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(button_joint.axis) == (0.0, -1.0, 0.0),
        details=(
            f"handle_axis={handle_joint.axis}, "
            f"knob_axis={knob_joint.axis}, "
            f"button_axis={button_joint.axis}"
        ),
    )

    ctx.expect_gap(
        handle,
        housing,
        axis="z",
        positive_elem="handle_grip",
        negative_elem="top_wall",
        min_gap=0.020,
        name="handle grip clears the speaker top",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        positive_elem="knob_body",
        negative_elem="knob_pod",
        min_gap=0.008,
        max_gap=0.020,
        name="volume knob sits proud of the side pod",
    )
    ctx.expect_gap(
        button,
        housing,
        axis="y",
        positive_elem="button_cap",
        negative_elem="front_lower_panel",
        min_gap=0.010,
        max_gap=0.030,
        name="button cap stands proud of the front panel",
    )

    rest_button_pos = ctx.part_world_position(button)
    rest_handle_box = ctx.part_world_aabb(handle)
    with ctx.pose({button_joint: button_joint.motion_limits.upper}):
        pressed_button_pos = ctx.part_world_position(button)
    with ctx.pose({handle_joint: 1.10}):
        raised_handle_box = ctx.part_world_aabb(handle)

    ctx.check(
        "front button presses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )
    ctx.check(
        "handle swings up into a carry position",
        rest_handle_box is not None
        and raised_handle_box is not None
        and raised_handle_box[1][2] > rest_handle_box[1][2] + 0.020,
        details=f"rest={rest_handle_box}, raised={raised_handle_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
