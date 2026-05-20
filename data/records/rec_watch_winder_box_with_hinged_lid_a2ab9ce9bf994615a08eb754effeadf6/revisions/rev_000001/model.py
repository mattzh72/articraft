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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.33, 0.20, 0.12, 1.0))
    liner = model.material("liner", rgba=(0.78, 0.72, 0.62, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.44, 0.50, 0.54, 0.30))
    hinge_metal = model.material("hinge_metal", rgba=(0.70, 0.63, 0.49, 1.0))

    outer_width = 0.22
    outer_depth = 0.17
    body_height = 0.12
    floor_thickness = 0.018
    left_wall = 0.075
    right_wall = 0.035
    end_wall = 0.012
    hinge_radius = 0.005
    hinge_axis_y = -(outer_depth * 0.5 + hinge_radius + 0.003)
    hinge_axis_z = body_height + 0.003

    body = model.part("body")
    body.visual(
        Box((outer_width, outer_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=walnut,
        name="base_floor",
    )
    body.visual(
        Box((left_wall, outer_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                -outer_width * 0.5 + left_wall * 0.5,
                0.0,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=walnut,
        name="left_mass_wall",
    )
    body.visual(
        Box((right_wall, outer_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                outer_width * 0.5 - right_wall * 0.5,
                0.0,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=walnut,
        name="right_side_wall",
    )
    body.visual(
        Box((outer_width, end_wall, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -outer_depth * 0.5 + end_wall * 0.5,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((outer_width, end_wall, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                outer_depth * 0.5 - end_wall * 0.5,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((outer_width - left_wall - right_wall, outer_depth - 2.0 * end_wall, 0.008)),
        origin=Origin(
            xyz=(
                0.02,
                0.0,
                floor_thickness + 0.004,
            )
        ),
        material=liner,
        name="inner_floor_pad",
    )
    body.visual(
        Box((0.034, 0.090, 0.074)),
        origin=Origin(xyz=(-0.060, -0.004, 0.055)),
        material=walnut,
        name="motor_mass_block",
    )
    body.visual(
        Box((0.028, 0.040, 0.034)),
        origin=Origin(xyz=(-0.038, 0.0, 0.035)),
        material=walnut,
        name="drive_tower",
    )
    body.visual(
        Box((0.018, 0.024, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, 0.054)),
        material=walnut,
        name="drive_riser",
    )
    body.visual(
        Box((0.026, 0.024, 0.018)),
        origin=Origin(xyz=(-0.021, 0.0, 0.070)),
        material=hinge_metal,
        name="spindle_arm",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(-0.006, 0.0, 0.070),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="spindle_pin",
    )
    body.visual(
        Box((0.040, 0.008, 0.010)),
        origin=Origin(xyz=(-0.070, -0.089, body_height + 0.001)),
        material=hinge_metal,
        name="left_hinge_leaf",
    )
    body.visual(
        Box((0.040, 0.008, 0.010)),
        origin=Origin(xyz=(0.070, -0.089, body_height + 0.001)),
        material=hinge_metal,
        name="right_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.04),
        origin=Origin(
            xyz=(-0.07, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.04),
        origin=Origin(
            xyz=(0.07, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, body_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid_outer = rounded_rect_profile(0.216, 0.166, 0.018, corner_segments=8)
    lid_window = [(x + 0.020, y) for x, y in rounded_rect_profile(0.126, 0.098, 0.014, corner_segments=8)]
    lid_frame = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            lid_outer,
            [lid_window],
            height=0.018,
            center=True,
        ),
        "lid_frame",
    )
    lid.visual(
        lid_frame,
        origin=Origin(xyz=(0.0, 0.095, 0.006)),
        material=walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((0.132, 0.104, 0.004)),
        origin=Origin(xyz=(0.020, 0.095, 0.0005)),
        material=smoked_glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.090, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.008, 0.004)),
        material=hinge_metal,
        name="hinge_strap",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.09),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="center_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.216, 0.166, 0.022)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.095, 0.006)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(
            xyz=(0.024, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="hub_shell",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.064),
        origin=Origin(
            xyz=(0.050, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=liner,
        name="pillow_roll",
    )
    cradle.visual(
        Box((0.012, 0.020, 0.010)),
        origin=Origin(xyz=(0.076, 0.0, 0.034)),
        material=walnut,
        name="retention_pad",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.070)),
        mass=0.28,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(-0.010, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        )

    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.010,
            max_penetration=0.012,
            name="closed lid stays close to the body top envelope",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.12,
            name="closed lid covers the box plan",
        )
        ctx.expect_within(
            cradle,
            body,
            axes="yz",
            margin=0.002,
            name="cradle fits inside the body cavity profile",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.2}):
        opened_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward",
        closed_aabb is not None and opened_aabb is not None and opened_aabb[1][2] > closed_aabb[1][2] + 0.05,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    with ctx.pose({cradle_spin: 0.0}):
        rest_pad = aabb_center(ctx.part_element_world_aabb(cradle, elem="retention_pad"))
    with ctx.pose({cradle_spin: math.pi * 0.5}):
        quarter_turn_pad = aabb_center(ctx.part_element_world_aabb(cradle, elem="retention_pad"))

    ctx.check(
        "cradle rotates about the spindle",
        rest_pad is not None
        and quarter_turn_pad is not None
        and quarter_turn_pad[1] < rest_pad[1] - 0.02
        and quarter_turn_pad[2] < rest_pad[2] - 0.02,
        details=f"rest={rest_pad}, quarter_turn={quarter_turn_pad}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        glass_center = aabb_center(ctx.part_element_world_aabb(lid, elem="glass_panel"))
    ctx.check(
        "window opening is offset toward the lighter side",
        glass_center is not None and glass_center[0] > 0.015,
        details=f"glass_center={glass_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
