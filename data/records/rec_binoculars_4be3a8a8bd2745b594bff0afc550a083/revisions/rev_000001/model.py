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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MAT_ARMOR = Material("matte_black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
MAT_PANEL = Material("pebbled_charcoal_grip", rgba=(0.025, 0.026, 0.025, 1.0))
MAT_METAL = Material("satin_black_anodized_metal", rgba=(0.015, 0.015, 0.014, 1.0))
MAT_EDGE = Material("dry_graphite_edges", rgba=(0.080, 0.082, 0.078, 1.0))
MAT_GLASS = Material("green_blue_coated_glass", rgba=(0.04, 0.22, 0.25, 0.62))
MAT_MARK = Material("engraved_white_mark", rgba=(0.82, 0.84, 0.78, 1.0))


def _tube_x(outer_radius: float, inner_radius: float, length: float):
    """Hollow optical tube whose local extrusion axis is +X."""
    return cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(length)


def _add_barrel_module(
    part,
    *,
    side: int,
    barrel_y: float,
    main_mesh_name: str,
    objective_mesh_name: str,
    eyecup_mesh_name: str,
) -> None:
    """Add one precision optical barrel with armor, lenses, and rubber grip."""
    # Main hollow rubber-armored barrel, open through the optical axis.
    part.visual(
        mesh_from_cadquery(_tube_x(0.029, 0.020, 0.122), main_mesh_name, tolerance=0.0007),
        origin=Origin(xyz=(-0.061, barrel_y, 0.0)),
        material=MAT_ARMOR,
        name="main_tube",
    )

    # Front objective bell with a larger annular retaining ring.
    part.visual(
        mesh_from_cadquery(_tube_x(0.036, 0.024, 0.033), objective_mesh_name, tolerance=0.0007),
        origin=Origin(xyz=(0.058, barrel_y, 0.0)),
        material=MAT_METAL,
        name="objective_ring",
    )
    part.visual(
        Cylinder(radius=0.0248, length=0.0026),
        origin=Origin(xyz=(0.0922, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=MAT_GLASS,
        name="objective_glass",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0545, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=MAT_EDGE,
        name="objective_shoulder",
    )

    # Rear eyepiece cup and ocular glass.
    part.visual(
        mesh_from_cadquery(_tube_x(0.026, 0.016, 0.030), eyecup_mesh_name, tolerance=0.0007),
        origin=Origin(xyz=(-0.090, barrel_y, 0.0)),
        material=MAT_ARMOR,
        name="eyecup",
    )
    part.visual(
        Cylinder(radius=0.0168, length=0.0022),
        origin=Origin(xyz=(-0.0908, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=MAT_GLASS,
        name="ocular_glass",
    )

    # Raised side grip pad and individual ribs.  They are slightly buried into
    # the tube/pad so the molded rubber reads as one connected armored shell.
    outer_y = barrel_y + side * 0.0310
    part.visual(
        Box((0.066, 0.0055, 0.030)),
        origin=Origin(xyz=(-0.002, outer_y, 0.0)),
        material=MAT_PANEL,
        name="grip_panel",
    )
    for i, x in enumerate((-0.031, -0.020, -0.009, 0.002, 0.013, 0.024)):
        part.visual(
            Box((0.0035, 0.0080, 0.032)),
            origin=Origin(xyz=(x, outer_y + side * 0.0020, 0.0)),
            material=MAT_EDGE,
            name=f"grip_rib_{i}",
        )

    # Fine protective lip rings at the two ends of the body tube.
    for i, x in enumerate((-0.055, 0.049)):
        part.visual(
            Cylinder(radius=0.0305, length=0.006),
            origin=Origin(xyz=(x, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=MAT_EDGE,
            name=f"armor_lip_{i}",
        )


def _add_focus_wheel(part) -> None:
    part.visual(
        Cylinder(radius=0.0140, length=0.032),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=MAT_EDGE,
        name="wheel_body",
    )
    for i in range(18):
        theta = 2.0 * math.pi * i / 18.0
        part.visual(
            Box((0.0028, 0.0340, 0.0042)),
            origin=Origin(
                xyz=(0.0142 * math.cos(theta), 0.0, 0.0142 * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=MAT_PANEL,
            name=f"wheel_knurl_{i}",
        )
    part.visual(
        Box((0.004, 0.030, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=MAT_MARK,
        name="index_mark",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_binoculars")

    left_barrel = model.part("left_barrel")
    right_barrel = model.part("right_barrel")
    focus_wheel = model.part("focus_wheel")

    # The part frames for both optical halves lie on the longitudinal hinge
    # axis.  The optical axes are parallel to +X and offset laterally in +/-Y.
    _add_barrel_module(
        left_barrel,
        side=-1,
        barrel_y=-0.044,
        main_mesh_name="left_hollow_barrel",
        objective_mesh_name="left_objective_annulus",
        eyecup_mesh_name="left_eyecup_annulus",
    )
    _add_barrel_module(
        right_barrel,
        side=1,
        barrel_y=0.044,
        main_mesh_name="right_hollow_barrel",
        objective_mesh_name="right_objective_annulus",
        eyecup_mesh_name="right_eyecup_annulus",
    )

    # Alternating exposed hinge knuckles on the shared IPD hinge axis.  The left
    # half carries the front/rear knuckles; the right half carries the center
    # knuckle, so the joint reads as a real interleaved hinge without broad
    # current-pose overlap.
    for i, x in enumerate((-0.052, 0.052)):
        left_barrel.visual(
            Cylinder(radius=0.0110, length=0.038),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=MAT_METAL,
            name=f"hinge_knuckle_{i}",
        )
        left_barrel.visual(
            Box((0.050, 0.016, 0.011)),
            origin=Origin(xyz=(x, -0.013, 0.0)),
            material=MAT_METAL,
            name=f"bridge_web_{i}",
        )
        left_barrel.visual(
            Box((0.032, 0.010, 0.007)),
            origin=Origin(xyz=(x, -0.024, 0.020)),
            material=MAT_EDGE,
            name=f"upper_bridge_rib_{i}",
        )
    right_barrel.visual(
        Cylinder(radius=0.0106, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=MAT_METAL,
        name="hinge_knuckle",
    )
    right_barrel.visual(
        Box((0.056, 0.016, 0.011)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=MAT_METAL,
        name="bridge_web",
    )
    right_barrel.visual(
        Box((0.034, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, 0.024, 0.020)),
        material=MAT_EDGE,
        name="upper_bridge_rib",
    )

    # Small fixed saddle plates that visually cradle the focusing wheel above
    # the central bridge without intersecting the rotating wheel.
    left_barrel.visual(
        Box((0.014, 0.004, 0.024)),
        origin=Origin(xyz=(-0.013, -0.019, 0.023)),
        material=MAT_METAL,
        name="focus_yoke",
    )

    _add_focus_wheel(focus_wheel)

    model.articulation(
        "interpupillary_hinge",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=right_barrel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.28, upper=0.28),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )
    model.articulation(
        "focus_axis",
        ArticulationType.CONTINUOUS,
        parent=left_barrel,
        child=focus_wheel,
        origin=Origin(xyz=(-0.013, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left = object_model.get_part("left_barrel")
    right = object_model.get_part("right_barrel")
    focus = object_model.get_part("focus_wheel")
    hinge = object_model.get_articulation("interpupillary_hinge")
    focus_axis = object_model.get_articulation("focus_axis")

    ctx.expect_gap(
        right,
        left,
        axis="y",
        positive_elem="main_tube",
        negative_elem="main_tube",
        min_gap=0.020,
        name="parallel barrels have realistic interpupillary separation",
    )
    ctx.expect_overlap(
        right,
        left,
        axes="x",
        elem_a="main_tube",
        elem_b="main_tube",
        min_overlap=0.110,
        name="left and right optical barrels run in parallel",
    )
    ctx.expect_gap(
        focus,
        left,
        axis="z",
        positive_elem="wheel_body",
        negative_elem="main_tube",
        min_gap=0.0,
        max_gap=0.006,
        name="focus wheel is closely cradled above the armor",
    )

    rest_aabb = ctx.part_element_world_aabb(right, elem="main_tube")
    with ctx.pose({hinge: 0.28}):
        folded_aabb = ctx.part_element_world_aabb(right, elem="main_tube")
    if rest_aabb is not None and folded_aabb is not None:
        rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
        rest_center_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
        folded_center_y = 0.5 * (folded_aabb[0][1] + folded_aabb[1][1])
        folded_center_z = 0.5 * (folded_aabb[0][2] + folded_aabb[1][2])
        ctx.check(
            "interpupillary hinge folds the right optical half inward",
            folded_center_y < rest_center_y - 0.001
            and folded_center_z > rest_center_z + 0.010,
            details=f"rest_yz=({rest_center_y:.4f}, {rest_center_z:.4f}), "
            f"folded_yz=({folded_center_y:.4f}, {folded_center_z:.4f})",
        )
    else:
        ctx.fail("interpupillary hinge pose can be measured", "missing right main_tube AABB")

    mark_rest = ctx.part_element_world_aabb(focus, elem="index_mark")
    with ctx.pose({focus_axis: 1.0}):
        mark_rotated = ctx.part_element_world_aabb(focus, elem="index_mark")
    if mark_rest is not None and mark_rotated is not None:
        rest_x = 0.5 * (mark_rest[0][0] + mark_rest[1][0])
        rotated_x = 0.5 * (mark_rotated[0][0] + mark_rotated[1][0])
        ctx.check(
            "focus wheel visibly rotates about its transverse axle",
            abs(rotated_x - rest_x) > 0.006,
            details=f"rest_x={rest_x:.4f}, rotated_x={rotated_x:.4f}",
        )
    else:
        ctx.fail("focus wheel marker pose can be measured", "missing index_mark AABB")

    return ctx.report()


object_model = build_object_model()
