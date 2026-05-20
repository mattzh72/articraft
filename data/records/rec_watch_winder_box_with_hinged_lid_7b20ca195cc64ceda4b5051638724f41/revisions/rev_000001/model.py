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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.28, 0.17, 0.10, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.18, 0.11, 0.07, 1.0))
    champagne = model.material("champagne", rgba=(0.82, 0.74, 0.60, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.62, 0.36, 1.0))

    body_width = 0.28
    body_depth = 0.19
    body_height = 0.11
    wall = 0.012
    floor = 0.010
    lid_width = 0.286
    lid_depth = 0.192
    lid_thickness = 0.020
    hinge_radius = 0.005
    hinge_y = -body_depth * 0.5 - hinge_radius

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )
    body.visual(
        Box((body_width, body_depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor * 0.5)),
        material=walnut,
        name="outer_floor",
    )
    body.visual(
        Box((body_width, wall, body_height - floor)),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 - wall * 0.5, floor + (body_height - floor) * 0.5)
        ),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((body_width, wall, body_height - floor)),
        origin=Origin(
            xyz=(0.0, -body_depth * 0.5 + wall * 0.5, floor + (body_height - floor) * 0.5)
        ),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((wall, body_depth - 2.0 * wall, body_height - floor)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall * 0.5, 0.0, floor + (body_height - floor) * 0.5)
        ),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_depth - 2.0 * wall, body_height - floor)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall * 0.5, 0.0, floor + (body_height - floor) * 0.5)
        ),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall, body_depth - 2.0 * wall, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, floor + 0.002)),
        material=champagne,
        name="liner_floor",
    )
    body.visual(
        Box((body_width - 0.010, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.009, 0.028)),
        material=walnut_dark,
        name="front_apron",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.070),
        origin=Origin(
            xyz=(-0.095, hinge_y, body_height - hinge_radius),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=brass,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.070),
        origin=Origin(
            xyz=(0.095, hinge_y, body_height - hinge_radius),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=brass,
        name="right_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.074, 0.060), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="drive_boss",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_thickness)),
        mass=0.9,
        origin=Origin(xyz=(0.0, lid_depth * 0.5, hinge_radius + lid_thickness * 0.5)),
    )
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_depth * 0.5, hinge_radius + lid_thickness * 0.5)),
        material=walnut,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_width - 0.030, lid_depth - 0.038, 0.008)),
        origin=Origin(xyz=(0.0, lid_depth * 0.5, 0.001)),
        material=champagne,
        name="lid_liner",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=brass,
        name="center_hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, body_height - hinge_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    cradle = model.part("cradle")
    cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.090)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )
    cradle.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="rotor_backplate",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=walnut_dark,
        name="spindle_collar",
    )
    cradle.visual(
        Box((0.032, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, 0.037, 0.0)),
        material=walnut_dark,
        name="pillow_bridge",
    )
    cradle.visual(
        Box((0.076, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.072, 0.0)),
        material=champagne,
        name="watch_pillow",
    )
    cradle.visual(
        Box((0.084, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, 0.104, 0.0)),
        material=walnut_dark,
        name="retainer_pad",
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.065, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=1e-6,
            name="closed lid seats on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="outer_floor",
            min_overlap=0.18,
            name="lid panel spans most of the body footprint",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="rotor_backplate",
            elem_b="drive_boss",
            name="cradle is mounted on the rear drive spindle",
        )

    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_joint: 1.1}):
        open_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_panel is not None
        and open_lid_panel is not None
        and open_lid_panel[1][2] > closed_lid_panel[1][2] + 0.12,
        details=f"closed={closed_lid_panel}, open={open_lid_panel}",
    )

    closed_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    with ctx.pose({cradle_joint: pi * 0.5}):
        quarter_turn_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    ctx.check(
        "cradle spins about its spindle axis",
        closed_pillow is not None
        and quarter_turn_pillow is not None
        and (quarter_turn_pillow[1][2] - quarter_turn_pillow[0][2])
        > (closed_pillow[1][2] - closed_pillow[0][2]) + 0.010
        and (quarter_turn_pillow[1][0] - quarter_turn_pillow[0][0])
        < (closed_pillow[1][0] - closed_pillow[0][0]) - 0.010,
        details=f"closed={closed_pillow}, quarter_turn={quarter_turn_pillow}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
