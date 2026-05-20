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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_extraction_ceiling_fan")

    housing_color = model.material("housing_white", rgba=(0.94, 0.95, 0.93, 1.0))
    trim_color = model.material("trim_white", rgba=(0.98, 0.98, 0.97, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    knob_color = model.material("knob_gray", rgba=(0.70, 0.71, 0.73, 1.0))
    hinge_color = model.material("hinge_gray", rgba=(0.62, 0.63, 0.66, 1.0))

    outer_size = 0.24
    body_depth = 0.086
    top_thickness = 0.004
    wall_thickness = 0.020
    trim_thickness = 0.008
    opening_size = 0.160
    inner_body_size = outer_size - 2.0 * wall_thickness

    housing = model.part("housing")
    housing.visual(
        Box((outer_size, outer_size, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -top_thickness / 2.0)),
        material=housing_color,
        name="top_plate",
    )

    wall_height = body_depth - top_thickness - trim_thickness
    wall_z = -(top_thickness + wall_height / 2.0)
    housing.visual(
        Box((wall_thickness, outer_size, wall_height)),
        origin=Origin(xyz=(-outer_size / 2.0 + wall_thickness / 2.0, 0.0, wall_z)),
        material=housing_color,
        name="side_wall_left",
    )
    housing.visual(
        Box((wall_thickness, outer_size, wall_height)),
        origin=Origin(xyz=(outer_size / 2.0 - wall_thickness / 2.0, 0.0, wall_z)),
        material=housing_color,
        name="side_wall_right",
    )
    housing.visual(
        Box((inner_body_size, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, outer_size / 2.0 - wall_thickness / 2.0, wall_z)),
        material=housing_color,
        name="side_wall_top",
    )
    housing.visual(
        Box((inner_body_size, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -outer_size / 2.0 + wall_thickness / 2.0, wall_z)),
        material=housing_color,
        name="side_wall_bottom",
    )

    trim_z = -(body_depth - trim_thickness / 2.0)
    trim_band = (outer_size - opening_size) / 2.0
    housing.visual(
        Box((trim_band, outer_size, trim_thickness)),
        origin=Origin(xyz=(-opening_size / 2.0 - trim_band / 2.0, 0.0, trim_z)),
        material=trim_color,
        name="front_trim_left",
    )
    housing.visual(
        Box((trim_band, outer_size, trim_thickness)),
        origin=Origin(xyz=(opening_size / 2.0 + trim_band / 2.0, 0.0, trim_z)),
        material=trim_color,
        name="front_trim_right",
    )
    housing.visual(
        Box((opening_size, trim_band, trim_thickness)),
        origin=Origin(xyz=(0.0, opening_size / 2.0 + trim_band / 2.0, trim_z)),
        material=trim_color,
        name="front_trim_top",
    )
    housing.visual(
        Box((opening_size, trim_band, trim_thickness)),
        origin=Origin(xyz=(0.0, -opening_size / 2.0 - trim_band / 2.0, trim_z)),
        material=trim_color,
        name="front_trim_bottom",
    )

    motor_radius = 0.028
    motor_length = 0.028
    housing.visual(
        Cylinder(radius=motor_radius, length=motor_length),
        origin=Origin(xyz=(0.0, 0.0, -(top_thickness + motor_length / 2.0))),
        material=dark_plastic,
        name="motor_can",
    )

    hinge_axis_x = -(outer_size / 2.0 + 0.006)
    hinge_axis_z = -(body_depth - trim_thickness)
    hinge_radius = 0.004
    bracket_x = -0.118
    bracket_size = (0.016, 0.028, 0.012)
    hinge_segment_length = 0.026
    hinge_segment_offset = 0.057

    housing.visual(
        Box(bracket_size),
        origin=Origin(xyz=(bracket_x, hinge_segment_offset, hinge_axis_z)),
        material=housing_color,
        name="hinge_bracket_upper",
    )
    housing.visual(
        Box(bracket_size),
        origin=Origin(xyz=(bracket_x, -hinge_segment_offset, hinge_axis_z)),
        material=housing_color,
        name="hinge_bracket_lower",
    )
    housing.visual(
        Cylinder(radius=hinge_radius, length=hinge_segment_length),
        origin=Origin(
            xyz=(hinge_axis_x, hinge_segment_offset, hinge_axis_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_color,
        name="hinge_barrel_upper",
    )
    housing.visual(
        Cylinder(radius=hinge_radius, length=hinge_segment_length),
        origin=Origin(
            xyz=(hinge_axis_x, -hinge_segment_offset, hinge_axis_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_color,
        name="hinge_barrel_lower",
    )

    knob_mount = (0.095, -0.075, -body_depth)
    housing.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(knob_mount[0], knob_mount[1], knob_mount[2] + 0.002)),
        material=trim_color,
        name="knob_bezel",
    )

    cover = model.part("cover_panel")
    cover_t = 0.007
    cover_w = opening_size
    cover_x0 = (-opening_size / 2.0) - hinge_axis_x
    rail_w = 0.012
    hinge_stile_w = 0.014
    free_stile_w = 0.012
    cover_z = -cover_t / 2.0

    cover.visual(
        Box((hinge_stile_w, cover_w, cover_t)),
        origin=Origin(xyz=(cover_x0 + hinge_stile_w / 2.0, 0.0, cover_z)),
        material=trim_color,
        name="cover_hinge_stile",
    )
    cover.visual(
        Box((free_stile_w, cover_w, cover_t)),
        origin=Origin(xyz=(cover_x0 + cover_w - free_stile_w / 2.0, 0.0, cover_z)),
        material=trim_color,
        name="cover_free_stile",
    )
    cover.visual(
        Box((cover_w, rail_w, cover_t)),
        origin=Origin(
            xyz=(cover_x0 + cover_w / 2.0, cover_w / 2.0 - rail_w / 2.0, cover_z)
        ),
        material=trim_color,
        name="cover_top_rail",
    )
    cover.visual(
        Box((cover_w, rail_w, cover_t)),
        origin=Origin(
            xyz=(cover_x0 + cover_w / 2.0, -cover_w / 2.0 + rail_w / 2.0, cover_z)
        ),
        material=trim_color,
        name="cover_bottom_rail",
    )
    for idx, y_pos in enumerate((-0.045, -0.015, 0.015, 0.045)):
        cover.visual(
            Box((0.150, 0.010, 0.0035)),
            origin=Origin(xyz=(cover_x0 + 0.080, y_pos, -0.0035)),
            material=trim_color,
            name=f"cover_louver_{idx}",
        )
    cover.visual(
        Cylinder(radius=hinge_radius, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_color,
        name="cover_barrel",
    )
    cover.visual(
        Box((cover_x0 + 0.010, 0.078, 0.004)),
        origin=Origin(xyz=((cover_x0 + 0.010) / 2.0, 0.0, 0.002)),
        material=hinge_color,
        name="cover_hinge_leaf",
    )

    fan = model.part("fan_blade")
    fan.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(),
        material=dark_plastic,
        name="hub",
    )
    blade_radius = 0.042
    for idx, yaw in enumerate((0.25, 1.50, 2.76, 4.01, 5.27)):
        fan.visual(
            Box((0.070, 0.017, 0.0035)),
            origin=Origin(xyz=(blade_radius, 0.0, 0.0), rpy=(0.0, 0.18, yaw)),
            material=dark_plastic,
            name=f"blade_{idx}",
        )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_plastic,
        name="knob_stem",
    )
    knob.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=knob_color,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=knob_color,
        name="knob_rim",
    )
    knob.visual(
        Box((0.010, 0.0025, 0.0025)),
        origin=Origin(xyz=(0.007, 0.0, -0.024)),
        material=dark_plastic,
        name="knob_pointer",
    )

    model.articulation(
        "housing_to_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan,
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=knob_mount),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    cover = object_model.get_part("cover_panel")
    fan = object_model.get_part("fan_blade")
    knob = object_model.get_part("speed_knob")

    cover_hinge = object_model.get_articulation("housing_to_cover")
    fan_spin = object_model.get_articulation("housing_to_fan")
    knob_spin = object_model.get_articulation("housing_to_knob")

    ctx.expect_gap(
        housing,
        fan,
        axis="z",
        positive_elem="motor_can",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.0005,
        name="fan hub seats directly beneath the motor can",
    )
    ctx.expect_gap(
        housing,
        knob,
        axis="z",
        positive_elem="knob_bezel",
        negative_elem="knob_stem",
        min_gap=0.0,
        max_gap=0.0005,
        name="speed knob stem meets the bezel without penetration",
    )

    closed_cover = ctx.part_element_world_aabb(cover, elem="cover_free_stile")
    closed_blade = ctx.part_element_world_aabb(fan, elem="blade_0")
    closed_pointer = ctx.part_element_world_aabb(knob, elem="knob_pointer")

    with ctx.pose({cover_hinge: 1.35, fan_spin: 1.1, knob_spin: 1.2}):
        open_cover = ctx.part_element_world_aabb(cover, elem="cover_free_stile")
        spun_blade = ctx.part_element_world_aabb(fan, elem="blade_0")
        turned_pointer = ctx.part_element_world_aabb(knob, elem="knob_pointer")

        ctx.expect_gap(
            housing,
            cover,
            axis="z",
            positive_elem="motor_can",
            negative_elem="cover_free_stile",
            min_gap=0.020,
            name="open cover hangs below the fan opening",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return (
            (low[0] + high[0]) / 2.0,
            (low[1] + high[1]) / 2.0,
            (low[2] + high[2]) / 2.0,
        )

    closed_cover_center = aabb_center(closed_cover)
    open_cover_center = aabb_center(open_cover)
    closed_blade_center = aabb_center(closed_blade)
    spun_blade_center = aabb_center(spun_blade)
    closed_pointer_center = aabb_center(closed_pointer)
    turned_pointer_center = aabb_center(turned_pointer)

    ctx.check(
        "cover free edge swings downward on the side hinge",
        closed_cover_center is not None
        and open_cover_center is not None
        and open_cover_center[2] < closed_cover_center[2] - 0.040
        and open_cover_center[0] < closed_cover_center[0] - 0.025,
        details=f"closed={closed_cover_center}, open={open_cover_center}",
    )
    ctx.check(
        "fan blade sweeps around its central axis",
        closed_blade_center is not None
        and spun_blade_center is not None
        and abs(spun_blade_center[0] - closed_blade_center[0]) > 0.010
        and abs(spun_blade_center[1] - closed_blade_center[1]) > 0.010,
        details=f"closed={closed_blade_center}, spun={spun_blade_center}",
    )
    ctx.check(
        "speed knob pointer rotates around the knob axis",
        closed_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[0] - closed_pointer_center[0]) > 0.004
        and abs(turned_pointer_center[1] - closed_pointer_center[1]) > 0.004,
        details=f"closed={closed_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
