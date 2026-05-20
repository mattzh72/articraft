from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHELF_RADIUS = 0.285
SHELF_HOLE_RADIUS = 0.058
LOWER_Z = 0.290
UPPER_Z = 0.590


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    center_z: float = 0.0,
) -> cq.Workplane:
    """CadQuery annular cylinder centered on the local Z axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, center_z - height / 2.0))
    )


def _shelf_wood_shape() -> cq.Workplane:
    """One connected wooden tray: annular plate, raised lip, inner boss and ribs."""
    plate_thickness = 0.022
    rim_height = 0.038
    rim_width = 0.026

    body = _annular_cylinder(
        SHELF_RADIUS,
        SHELF_HOLE_RADIUS,
        plate_thickness,
        center_z=0.0,
    )

    outer_lip = _annular_cylinder(
        SHELF_RADIUS,
        SHELF_RADIUS - rim_width,
        rim_height,
        center_z=plate_thickness / 2.0 + rim_height / 2.0,
    )
    body = body.union(outer_lip)

    inner_boss = _annular_cylinder(
        0.095,
        SHELF_HOLE_RADIUS,
        0.016,
        center_z=plate_thickness / 2.0 + 0.008,
    )
    body = body.union(inner_boss)

    # Six shallow underside ribs make the shelf read as a carried tray rather
    # than a flat disk.  Each rib overlaps the plate slightly so the exported
    # mesh is one supported piece.
    rib_length = SHELF_RADIUS - SHELF_HOLE_RADIUS - 0.075
    rib_center_x = SHELF_HOLE_RADIUS + rib_length / 2.0 + 0.020
    for angle in range(0, 360, 60):
        rib = (
            cq.Workplane("XY")
            .box(rib_length, 0.018, 0.016)
            .translate(
                (
                    rib_center_x,
                    0.0,
                    -plate_thickness / 2.0 - 0.006,
                )
            )
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.union(rib)

    return body


def _bearing_collar_shape() -> cq.Workplane:
    """Metal bearing collar with a true central clearance hole for the fixed post."""
    inner_radius = 0.024
    sleeve_outer = 0.064
    race_outer = 0.076

    sleeve = _annular_cylinder(sleeve_outer, inner_radius, 0.080, center_z=0.0)
    lower_race = _annular_cylinder(race_outer, inner_radius, 0.012, center_z=-0.046)
    upper_race = _annular_cylinder(race_outer, inner_radius, 0.012, center_z=0.046)
    return sleeve.union(lower_race).union(upper_race)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_lazy_susan")

    wood = model.material("warm_bamboo", rgba=(0.82, 0.55, 0.28, 1.0))
    dark_wood = model.material("end_grain", rgba=(0.55, 0.34, 0.16, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.72, 0.68, 1.0))
    dark_steel = model.material("shadowed_bearing", rgba=(0.18, 0.18, 0.17, 1.0))
    brass = model.material("brass_marker", rgba=(0.95, 0.70, 0.24, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.023, 0.020, 1.0))

    post = model.part("central_post")
    post.visual(
        Cylinder(radius=0.090, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_disk",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=brushed_steel,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.035, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.807)),
        material=brushed_steel,
        name="top_cap",
    )
    post.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.828)),
        material=brushed_steel,
        name="rounded_knob",
    )

    # Fixed thrust washers bracket each rotating collar without being part of
    # the rotating shelf.  Small clearance gaps keep the bearing stack legible.
    post.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, LOWER_Z - 0.059)),
        material=dark_steel,
        name="lower_washer_below",
    )
    post.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, LOWER_Z + 0.059)),
        material=dark_steel,
        name="lower_washer_above",
    )
    post.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, UPPER_Z - 0.059)),
        material=dark_steel,
        name="upper_washer_below",
    )
    post.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, UPPER_Z + 0.059)),
        material=dark_steel,
        name="upper_washer_above",
    )

    # Four low rubber feet are partly embedded in the metal base so they read as
    # attached pads, not loose islands.
    for idx, angle in enumerate((45, 135, 225, 315)):
        rad = math.radians(angle)
        post.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(
                xyz=(0.062 * math.cos(rad), 0.062 * math.sin(rad), 0.004)
            ),
            material=rubber,
            name=f"foot_{idx}",
        )

    shelf_wood = _shelf_wood_shape()
    collar = _bearing_collar_shape()

    lower_shelf = model.part("lower_shelf")
    lower_shelf.visual(
        mesh_from_cadquery(shelf_wood, "lower_wood_tray"),
        material=wood,
        name="lower_wood_tray",
    )
    lower_shelf.visual(
        mesh_from_cadquery(collar, "lower_bearing_collar"),
        material=brushed_steel,
        name="lower_bearing_collar",
    )
    lower_shelf.visual(
        Cylinder(radius=SHELF_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_wood,
        name="lower_underside_shadow",
    )
    lower_shelf.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(SHELF_RADIUS - 0.018, 0.0, 0.049)),
        material=brass,
        name="lower_index",
    )

    upper_shelf = model.part("upper_shelf")
    upper_shelf.visual(
        mesh_from_cadquery(shelf_wood, "upper_wood_tray"),
        material=wood,
        name="upper_wood_tray",
    )
    upper_shelf.visual(
        mesh_from_cadquery(collar, "upper_bearing_collar"),
        material=brushed_steel,
        name="upper_bearing_collar",
    )
    upper_shelf.visual(
        Cylinder(radius=SHELF_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_wood,
        name="upper_underside_shadow",
    )
    upper_shelf.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(SHELF_RADIUS - 0.018, 0.0, 0.049)),
        material=brass,
        name="upper_index",
    )

    lower_shelf = model.get_part("lower_shelf")
    upper_shelf = model.get_part("upper_shelf")

    model.articulation(
        "post_to_lower_shelf",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=lower_shelf,
        origin=Origin(xyz=(0.0, 0.0, LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "post_to_upper_shelf",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=upper_shelf,
        origin=Origin(xyz=(0.0, 0.0, UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("central_post")
    lower = object_model.get_part("lower_shelf")
    upper = object_model.get_part("upper_shelf")
    lower_joint = object_model.get_articulation("post_to_lower_shelf")
    upper_joint = object_model.get_articulation("post_to_upper_shelf")

    ctx.check(
        "two independent rotating shelf joints",
        lower_joint.child == "lower_shelf"
        and upper_joint.child == "upper_shelf"
        and lower_joint.parent == "central_post"
        and upper_joint.parent == "central_post"
        and lower_joint.name != upper_joint.name,
    )
    ctx.check(
        "both axes are vertical",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
    )
    ctx.expect_origin_distance(
        lower,
        post,
        axes="xy",
        max_dist=0.001,
        name="lower collar is concentric with post",
    )
    ctx.expect_origin_distance(
        upper,
        post,
        axes="xy",
        max_dist=0.001,
        name="upper collar is concentric with post",
    )
    ctx.expect_origin_gap(
        upper,
        lower,
        axis="z",
        min_gap=0.280,
        max_gap=0.320,
        name="shelf tiers are vertically separated",
    )
    ctx.expect_gap(
        lower,
        post,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="lower_bearing_collar",
        negative_elem="lower_washer_below",
        name="lower collar rides just above its washer",
    )
    ctx.expect_gap(
        upper,
        post,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="upper_bearing_collar",
        negative_elem="upper_washer_below",
        name="upper collar rides just above its washer",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))

    with ctx.pose({lower_joint: math.pi / 2.0, upper_joint: 0.0}):
        lower_index = _center_from_aabb(
            ctx.part_element_world_aabb(lower, elem="lower_index")
        )
        upper_index = _center_from_aabb(
            ctx.part_element_world_aabb(upper, elem="upper_index")
        )
        ctx.check(
            "lower shelf rotates while upper shelf stays put",
            lower_index is not None
            and upper_index is not None
            and lower_index[1] > 0.230
            and abs(upper_index[1]) < 0.030
            and upper_index[0] > 0.250,
            details=f"lower_index={lower_index}, upper_index={upper_index}",
        )

    with ctx.pose({lower_joint: 0.0, upper_joint: -math.pi / 2.0}):
        lower_index = _center_from_aabb(
            ctx.part_element_world_aabb(lower, elem="lower_index")
        )
        upper_index = _center_from_aabb(
            ctx.part_element_world_aabb(upper, elem="upper_index")
        )
        ctx.check(
            "upper shelf rotates while lower shelf stays put",
            lower_index is not None
            and upper_index is not None
            and abs(lower_index[1]) < 0.030
            and lower_index[0] > 0.250
            and upper_index[1] < -0.230,
            details=f"lower_index={lower_index}, upper_index={upper_index}",
        )

    return ctx.report()


object_model = build_object_model()
