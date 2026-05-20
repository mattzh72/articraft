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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood_box_fan")

    painted_steel = model.material("painted_steel", rgba=(0.90, 0.91, 0.93, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    filter_aluminum = model.material("filter_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.78, 0.15, 0.08, 1.0))

    hood_width = 0.76
    hood_depth = 0.50
    hood_height = 0.18
    wall_thickness = 0.018
    top_thickness = 0.014

    filter_width = 0.68
    filter_depth = 0.40
    filter_thickness = 0.012
    filter_center_z = 0.010

    housing = model.part("housing")
    housing.visual(
        Box((hood_width, hood_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, hood_height - top_thickness / 2.0)),
        material=painted_steel,
        name="top_panel",
    )
    housing.visual(
        Box((hood_width, wall_thickness, hood_height - top_thickness)),
        origin=Origin(
            xyz=(0.0, -(hood_depth / 2.0) + wall_thickness / 2.0, (hood_height - top_thickness) / 2.0)
        ),
        material=painted_steel,
        name="back_wall",
    )
    side_wall_depth = hood_depth - 2.0 * wall_thickness
    housing.visual(
        Box((wall_thickness, side_wall_depth, hood_height - top_thickness)),
        origin=Origin(
            xyz=(
                -(hood_width / 2.0) + wall_thickness / 2.0,
                0.0,
                (hood_height - top_thickness) / 2.0,
            )
        ),
        material=painted_steel,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, side_wall_depth, hood_height - top_thickness)),
        origin=Origin(
            xyz=(
                (hood_width / 2.0) - wall_thickness / 2.0,
                0.0,
                (hood_height - top_thickness) / 2.0,
            )
        ),
        material=painted_steel,
        name="right_wall",
    )
    front_fascia_height = 0.128
    housing.visual(
        Box((hood_width, wall_thickness, front_fascia_height)),
        origin=Origin(
            xyz=(
                0.0,
                (hood_depth / 2.0) - wall_thickness / 2.0,
                hood_height - front_fascia_height / 2.0,
            )
        ),
        material=painted_steel,
        name="front_fascia",
    )

    rail_cap_size = (0.024, 0.44, 0.010)
    rail_fin_size = (0.012, 0.44, 0.024)
    rail_center_x = 0.346
    rail_center_y = 0.010
    housing.visual(
        Box(rail_cap_size),
        origin=Origin(xyz=(-rail_center_x, rail_center_y, 0.030)),
        material=dark_trim,
        name="left_rail_cap",
    )
    housing.visual(
        Box(rail_fin_size),
        origin=Origin(xyz=(-0.356, rail_center_y, 0.020)),
        material=dark_trim,
        name="left_rail_fin",
    )
    housing.visual(
        Box(rail_cap_size),
        origin=Origin(xyz=(rail_center_x, rail_center_y, 0.030)),
        material=dark_trim,
        name="right_rail_cap",
    )
    housing.visual(
        Box(rail_fin_size),
        origin=Origin(xyz=(0.356, rail_center_y, 0.020)),
        material=dark_trim,
        name="right_rail_fin",
    )

    housing.visual(
        Cylinder(radius=0.072, length=0.066),
        origin=Origin(xyz=(0.0, -0.015, 0.133)),
        material=dark_trim,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, -0.015, 0.086)),
        material=dark_trim,
        name="motor_shaft",
    )

    grease_filter = model.part("grease_filter")
    frame_bar = 0.022
    grease_filter.visual(
        Box((filter_width, frame_bar, filter_thickness)),
        origin=Origin(xyz=(0.0, (filter_depth / 2.0) - frame_bar / 2.0, 0.0)),
        material=filter_aluminum,
        name="front_frame",
    )
    grease_filter.visual(
        Box((filter_width, frame_bar, filter_thickness)),
        origin=Origin(xyz=(0.0, -(filter_depth / 2.0) + frame_bar / 2.0, 0.0)),
        material=filter_aluminum,
        name="rear_frame",
    )
    grease_filter.visual(
        Box((frame_bar, filter_depth, filter_thickness)),
        origin=Origin(xyz=(-(filter_width / 2.0) + frame_bar / 2.0, 0.0, 0.0)),
        material=filter_aluminum,
        name="left_frame",
    )
    grease_filter.visual(
        Box((frame_bar, filter_depth, filter_thickness)),
        origin=Origin(xyz=((filter_width / 2.0) - frame_bar / 2.0, 0.0, 0.0)),
        material=filter_aluminum,
        name="right_frame",
    )

    slat_depth = 0.012
    slat_height = 0.006
    slat_width = filter_width - 0.040
    slat_centers_y = (-0.132, -0.078, -0.024, 0.030, 0.084, 0.138)
    for idx, center_y in enumerate(slat_centers_y, start=1):
        grease_filter.visual(
            Box((slat_width, slat_depth, slat_height)),
            origin=Origin(xyz=(0.0, center_y, 0.0)),
            material=filter_aluminum,
            name=f"mesh_slat_{idx}",
        )

    grease_filter.visual(
        Box((0.012, 0.34, 0.008)),
        origin=Origin(xyz=(-0.346, 0.0, 0.010)),
        material=dark_trim,
        name="left_runner",
    )
    grease_filter.visual(
        Box((0.012, 0.34, 0.008)),
        origin=Origin(xyz=(0.346, 0.0, 0.010)),
        material=dark_trim,
        name="right_runner",
    )
    grease_filter.visual(
        Box((0.12, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.208, 0.0)),
        material=dark_trim,
        name="pull_tab",
    )

    fan_blade = model.part("fan_blade")
    fan_blade.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="hub",
    )
    blade_origin_radius = 0.105
    blade_size = (0.172, 0.038, 0.006)
    blade_pitch = math.radians(17.0)
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), start=1):
        fan_blade.visual(
            Box(blade_size),
            origin=Origin(xyz=(blade_origin_radius, 0.0, 0.0), rpy=(blade_pitch, 0.0, yaw)),
            material=matte_black,
            name="blade_primary" if idx == 1 else f"blade_{idx}",
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.008, 0.004, 0.014)),
        origin=Origin(xyz=(0.013, 0.026, 0.0)),
        material=indicator_red,
        name="knob_indicator",
    )

    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=grease_filter,
        origin=Origin(xyz=(0.0, 0.0, filter_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.25, lower=0.0, upper=0.18),
    )
    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_blade,
        origin=Origin(xyz=(0.0, -0.015, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.285, hood_depth / 2.0, 0.116)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
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
    grease_filter = object_model.get_part("grease_filter")
    fan_blade = object_model.get_part("fan_blade")
    speed_knob = object_model.get_part("speed_knob")

    filter_slide = object_model.get_articulation("housing_to_filter")
    fan_spin = object_model.get_articulation("housing_to_fan")
    knob_spin = object_model.get_articulation("housing_to_knob")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.check("housing part exists", housing is not None)
    ctx.check("grease filter part exists", grease_filter is not None)
    ctx.check("fan blade part exists", fan_blade is not None)
    ctx.check("speed knob part exists", speed_knob is not None)

    with ctx.pose({filter_slide: 0.0}):
        ctx.expect_overlap(
            grease_filter,
            housing,
            axes="xy",
            min_overlap=0.38,
            name="closed grease filter sits under the hood footprint",
        )
        ctx.expect_within(
            grease_filter,
            housing,
            axes="x",
            margin=0.03,
            name="closed grease filter stays between the side walls",
        )
        filter_rest_pos = ctx.part_world_position(grease_filter)

    with ctx.pose({filter_slide: filter_slide.motion_limits.upper}):
        ctx.expect_within(
            grease_filter,
            housing,
            axes="x",
            margin=0.03,
            name="extended grease filter remains guided laterally by the rails",
        )
        ctx.expect_overlap(
            grease_filter,
            housing,
            axes="y",
            min_overlap=0.25,
            name="extended grease filter keeps retained insertion in the hood",
        )
        filter_open_pos = ctx.part_world_position(grease_filter)

    ctx.check(
        "grease filter slides out toward the front",
        filter_rest_pos is not None
        and filter_open_pos is not None
        and filter_open_pos[1] > filter_rest_pos[1] + 0.12,
        details=f"rest={filter_rest_pos}, open={filter_open_pos}",
    )

    ctx.check(
        "fan articulation is continuous around vertical axis",
        fan_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(fan_spin.axis) == (0.0, 0.0, 1.0)
        and fan_spin.motion_limits is not None
        and fan_spin.motion_limits.lower is None
        and fan_spin.motion_limits.upper is None,
        details=f"type={fan_spin.joint_type}, axis={fan_spin.axis}, limits={fan_spin.motion_limits}",
    )
    ctx.check(
        "speed knob articulation is continuous around its spindle axis",
        knob_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (0.0, 1.0, 0.0)
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=f"type={knob_spin.joint_type}, axis={knob_spin.axis}, limits={knob_spin.motion_limits}",
    )

    with ctx.pose({fan_spin: 0.0}):
        fan_blade_rest_center = aabb_center(ctx.part_element_world_aabb(fan_blade, elem="blade_primary"))
    with ctx.pose({fan_spin: math.pi / 2.0}):
        fan_blade_rotated_center = aabb_center(ctx.part_element_world_aabb(fan_blade, elem="blade_primary"))
    fan_motion = None
    if fan_blade_rest_center is not None and fan_blade_rotated_center is not None:
        fan_motion = math.dist(fan_blade_rest_center[:2], fan_blade_rotated_center[:2])
    ctx.check(
        "fan blade visibly rotates inside the housing",
        fan_motion is not None
        and fan_motion > 0.12
        and abs(fan_blade_rotated_center[2] - fan_blade_rest_center[2]) < 1e-6,
        details=f"rest={fan_blade_rest_center}, rotated={fan_blade_rotated_center}, motion={fan_motion}",
    )

    with ctx.pose({knob_spin: 0.0}):
        knob_rest_center = aabb_center(ctx.part_element_world_aabb(speed_knob, elem="knob_indicator"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        knob_rotated_center = aabb_center(ctx.part_element_world_aabb(speed_knob, elem="knob_indicator"))
    knob_motion = None
    if knob_rest_center is not None and knob_rotated_center is not None:
        knob_motion = math.dist(knob_rest_center, knob_rotated_center)
    ctx.check(
        "speed control knob visibly turns",
        knob_motion is not None
        and knob_motion > 0.015
        and abs(knob_rest_center[1] - knob_rotated_center[1]) < 1e-6,
        details=f"rest={knob_rest_center}, rotated={knob_rotated_center}, motion={knob_motion}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
