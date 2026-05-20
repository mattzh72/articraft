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
    ExtrudeGeometry,
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

    body_depth = 0.19
    body_width = 0.15
    body_height = 0.12
    wall = 0.012
    floor = 0.015
    lid_height = 0.018

    walnut = model.material("walnut", rgba=(0.29, 0.18, 0.10, 1.0))
    lining = model.material("lining", rgba=(0.57, 0.47, 0.34, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.55, 0.66, 0.72, 0.30))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.71, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_depth, body_width, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor * 0.5)),
        material=walnut,
        name="base_floor",
    )
    body.visual(
        Box((body_depth, wall, body_height - floor + 0.002)),
        origin=Origin(
            xyz=(0.0, 0.5 * (body_width - wall), body_height - 0.5 * (body_height - floor + 0.002))
        ),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((body_depth, wall, body_height - floor + 0.002)),
        origin=Origin(
            xyz=(0.0, -0.5 * (body_width - wall), body_height - 0.5 * (body_height - floor + 0.002))
        ),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall, body_height - floor + 0.002)),
        origin=Origin(
            xyz=(
                -0.5 * (body_depth - wall),
                0.0,
                body_height - 0.5 * (body_height - floor + 0.002),
            )
        ),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((0.020, body_width - 2.0 * wall, 0.048)),
        origin=Origin(xyz=(0.5 * body_depth - 0.010, 0.0, 0.024)),
        material=walnut,
        name="front_lip",
    )
    body.visual(
        Box((body_depth - 2.0 * wall, body_width - 2.0 * wall, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, floor + 0.002)),
        material=lining,
        name="inner_floor_lining",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(-0.5 * body_depth + wall + 0.009, 0.0, 0.074),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_metal,
        name="motor_housing",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(
            xyz=(-0.5 * body_depth + wall + 0.024, 0.0, 0.074),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_metal,
        name="spindle_support",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((body_depth, wall, lid_height)),
        origin=Origin(xyz=(0.5 * body_depth, 0.5 * (body_width - wall), 0.5 * lid_height)),
        material=walnut,
        name="left_frame_rail",
    )
    lid.visual(
        Box((body_depth, wall, lid_height)),
        origin=Origin(xyz=(0.5 * body_depth, -0.5 * (body_width - wall), 0.5 * lid_height)),
        material=walnut,
        name="right_frame_rail",
    )
    lid.visual(
        Box((wall, body_width, lid_height)),
        origin=Origin(xyz=(0.5 * wall, 0.0, 0.5 * lid_height)),
        material=walnut,
        name="rear_frame_rail",
    )
    lid.visual(
        Box((wall, body_width, lid_height)),
        origin=Origin(xyz=(body_depth - 0.5 * wall, 0.0, 0.5 * lid_height)),
        material=walnut,
        name="front_frame_rail",
    )
    lid.visual(
        Box((body_depth - 0.020, body_width - 0.020, 0.003)),
        origin=Origin(xyz=(0.5 * body_depth, 0.0, 0.011)),
        material=smoked_glass,
        name="glass_panel",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(
            xyz=(body_depth - 0.006, 0.0, 0.009),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_metal,
        name="lid_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, lid_height)),
        mass=0.55,
        origin=Origin(xyz=(0.5 * body_depth, 0.0, 0.5 * lid_height)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_metal,
        name="spindle_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_metal,
        name="spindle_collar",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_metal,
        name="backing_disc",
    )
    pillow_geom = ExtrudeGeometry.centered(rounded_rect_profile(0.064, 0.086, 0.017), 0.054)
    pillow_geom.rotate_y(math.pi * 0.5)
    cradle.visual(
        mesh_from_geometry(pillow_geom, "watch_cushion"),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=lining,
        name="watch_cushion",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.086, 0.086)),
        mass=0.30,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.5 * body_depth, 0.0, body_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(-0.5 * body_depth + wall + 0.030, 0.0, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cradle = object_model.get_part("cradle")
    lid = object_model.get_part("lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            name="lid seats onto the box rim when closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.12,
            name="lid covers the box opening footprint",
        )

    with ctx.pose({lid_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    with ctx.pose({cradle_spin: 0.0}):
        ctx.expect_overlap(
            cradle,
            body,
            axes="yz",
            min_overlap=0.05,
            name="cradle sits within the watch chamber footprint",
        )
        ctx.expect_gap(
            cradle,
            body,
            axis="z",
            min_gap=0.010,
            negative_elem="inner_floor_lining",
            name="cradle stays above the lined floor",
        )

    rest_cushion = ctx.part_element_world_aabb(cradle, elem="watch_cushion")
    with ctx.pose({cradle_spin: math.pi * 0.5}):
        quarter_turn_cushion = ctx.part_element_world_aabb(cradle, elem="watch_cushion")

    rest_y = None if rest_cushion is None else rest_cushion[1][1] - rest_cushion[0][1]
    rest_z = None if rest_cushion is None else rest_cushion[1][2] - rest_cushion[0][2]
    quarter_y = (
        None if quarter_turn_cushion is None else quarter_turn_cushion[1][1] - quarter_turn_cushion[0][1]
    )
    quarter_z = (
        None if quarter_turn_cushion is None else quarter_turn_cushion[1][2] - quarter_turn_cushion[0][2]
    )
    ctx.check(
        "cradle visibly rotates about its spindle",
        rest_y is not None
        and rest_z is not None
        and quarter_y is not None
        and quarter_z is not None
        and abs(rest_y - quarter_y) > 0.015
        and abs(rest_z - quarter_z) > 0.015,
        details=(
            f"rest_cushion={rest_cushion}, quarter_turn_cushion={quarter_turn_cushion}, "
            f"rest_y={rest_y}, rest_z={rest_z}, quarter_y={quarter_y}, quarter_z={quarter_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
