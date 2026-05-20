from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    ConeGeometry,
)


def _head_casting() -> cq.Workplane:
    """One continuous head casting set behind the exposed quill sleeve."""
    main = cq.Workplane("XY").box(0.40, 0.34, 0.25).translate((0.07, 0.0, 1.425))
    belt_cover = cq.Workplane("XY").box(0.44, 0.29, 0.11).translate((0.10, 0.0, 1.60))
    return main.union(belt_cover)


def _table_plate() -> cq.Workplane:
    """Machined drill table with a through hole under the spindle."""
    plate = cq.Workplane("XY").box(0.44, 0.34, 0.04).translate((0.08, 0.0, 0.08))
    center_hole = cq.Workplane("XY").circle(0.026).extrude(0.08).translate((0.08, 0.0, 0.04))
    return plate.cut(center_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    cast = model.material("cast_iron", rgba=(0.18, 0.20, 0.21, 1.0))
    dark = model.material("black_oxide", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    handwheel = model.material("red_handle", rgba=(0.65, 0.05, 0.035, 1.0))
    slot_black = model.material("slot_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    frame = model.part("frame")
    frame.visual(Box((0.70, 0.45, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=cast, name="base_plate")
    frame.visual(Box((0.25, 0.19, 0.03)), origin=Origin(xyz=(0.0, 0.0, 0.095)), material=cast, name="column_foot")
    for y in (-0.13, 0.13):
        frame.visual(Box((0.38, 0.035, 0.006)), origin=Origin(xyz=(0.12, y, 0.083)), material=slot_black, name=f"base_slot_{y:+.2f}")
    frame.visual(Cylinder(radius=0.035, length=1.46), origin=Origin(xyz=(0.0, 0.0, 0.82)), material=dark, name="round_column")
    frame.visual(mesh_from_cadquery(_head_casting(), "head_casting"), material=cast, name="head_casting")
    frame.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(xyz=(0.31, -0.195, 1.44), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast,
        name="feed_boss",
    )
    frame.visual(Box((0.018, 0.115, 0.220)), origin=Origin(xyz=(0.272, 0.0, 1.405)), material=cast, name="quill_guide")

    table_carriage = model.part("table_carriage")
    table_carriage.visual(Box((0.060, 0.170, 0.140)), origin=Origin(xyz=(0.064, 0.0, 0.0)), material=cast, name="front_collar")
    for y, suffix in ((0.062, "0"), (-0.062, "1")):
        table_carriage.visual(Box((0.140, 0.040, 0.140)), origin=Origin(xyz=(0.0, y, 0.0)), material=cast, name=f"side_collar_{suffix}")
    table_carriage.visual(Box((0.12, 0.12, 0.065)), origin=Origin(xyz=(0.130, 0.0, -0.012)), material=cast, name="front_arm")
    table_carriage.visual(Box((0.17, 0.23, 0.03)), origin=Origin(xyz=(0.155, 0.0, -0.080)), material=cast, name="yoke_web")
    for y, suffix in ((0.105, "0"), (-0.105, "1")):
        table_carriage.visual(Box((0.085, 0.040, 0.120)), origin=Origin(xyz=(0.235, y, -0.020)), material=cast, name=f"hinge_lug_{suffix}")
    table_carriage.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.116, 0.0, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="lock_screw",
    )

    table = model.part("table")
    table.visual(mesh_from_cadquery(_table_plate(), "table_plate"), material=cast, name="table_plate")
    table.visual(Cylinder(radius=0.030, length=0.172), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name="hinge_barrel")
    table.visual(Box((0.115, 0.120, 0.070)), origin=Origin(xyz=(0.045, 0.0, 0.035)), material=cast, name="tilt_web")
    for y, suffix in ((-0.085, "0"), (0.085, "1")):
        table.visual(Box((0.30, 0.018, 0.004)), origin=Origin(xyz=(0.08, y, 0.1005)), material=slot_black, name=f"t_slot_{suffix}")

    quill = model.part("quill")
    quill.visual(Cylinder(radius=0.030, length=0.245), origin=Origin(xyz=(0.0, 0.0, -0.145)), material=steel, name="quill_sleeve")
    quill.visual(Cylinder(radius=0.017, length=0.130), origin=Origin(xyz=(0.0, 0.0, -0.310)), material=steel, name="spindle")
    quill.visual(Cylinder(radius=0.032, length=0.080), origin=Origin(xyz=(0.0, 0.0, -0.385)), material=dark, name="chuck")
    quill.visual(Cylinder(radius=0.006, length=0.185), origin=Origin(xyz=(0.0, 0.0, -0.515)), material=steel, name="drill_bit")
    quill.visual(mesh_from_geometry(ConeGeometry(0.010, 0.035, radial_segments=24), "drill_tip"), origin=Origin(xyz=(0.0, 0.0, -0.610)), material=steel, name="drill_tip")

    feed_lever = model.part("feed_lever")
    feed_lever.visual(Cylinder(radius=0.034, length=0.045), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=dark, name="lever_hub")
    for idx, angle in enumerate((0.0, 2.10, -2.10)):
        length = 0.18
        cx = 0.5 * length * cos(angle)
        cz = -0.5 * length * sin(angle)
        ex = length * cos(angle)
        ez = -length * sin(angle)
        feed_lever.visual(
            Box((length, 0.018, 0.018)),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, angle, 0.0)),
            material=steel,
            name=f"feed_spoke_{idx}",
        )
        feed_lever.visual(Sphere(radius=0.025), origin=Origin(xyz=(ex, 0.0, ez)), material=handwheel, name=f"feed_ball_{idx}")

    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.12, lower=-0.18, upper=0.28),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=quill,
        origin=Origin(xyz=(0.31, 0.0, 1.52)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.16),
    )
    model.articulation(
        "feed_handle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=feed_lever,
        origin=Origin(xyz=(0.31, -0.240, 1.44)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.55, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")
    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    quill_feed = object_model.get_articulation("quill_feed")

    ctx.expect_origin_distance(carriage, frame, axes="xy", max_dist=0.001, name="table carriage stays centered on column")
    ctx.expect_within(quill, table, axes="xy", inner_elem="drill_bit", outer_elem="table_plate", margin=0.005, name="drill bit aligns over table hole footprint")

    lift_rest = ctx.part_world_position(carriage)
    with ctx.pose({table_lift: 0.20}):
        lifted = ctx.part_world_position(carriage)
    ctx.check(
        "table lift raises carriage",
        lift_rest is not None and lifted is not None and lifted[2] > lift_rest[2] + 0.18,
        details=f"rest={lift_rest}, lifted={lifted}",
    )

    flat_aabb = ctx.part_element_world_aabb(table, elem="table_plate")
    with ctx.pose({table_tilt: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(table, elem="table_plate")
    ctx.check(
        "table tilt lifts front edge",
        flat_aabb is not None and tilted_aabb is not None and tilted_aabb[1][2] > flat_aabb[1][2] + 0.05,
        details=f"flat={flat_aabb}, tilted={tilted_aabb}",
    )

    quill_rest = ctx.part_world_position(quill)
    with ctx.pose({quill_feed: 0.14}):
        quill_down = ctx.part_world_position(quill)
    ctx.check(
        "quill feed drops spindle",
        quill_rest is not None and quill_down is not None and quill_down[2] < quill_rest[2] - 0.12,
        details=f"rest={quill_rest}, down={quill_down}",
    )

    return ctx.report()


object_model = build_object_model()
