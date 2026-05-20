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
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _dish_shell(radius: float = 0.56, depth: float = 0.22, thickness: float = 0.018) -> cq.Workplane:
    """Thin parabolic reflector, axis on local Y, open toward -Y."""
    samples = 18
    inner = []
    outer = []
    for i in range(samples + 1):
        r = radius * i / samples
        y = depth * (1.0 - (r / radius) ** 2)
        inner.append((r, y))
        outer.append((r, y + thickness))

    profile = [outer[0], *outer[1:], *reversed(inner)]
    shell = (
        cq.Workplane("XY")
        .moveTo(*profile[0])
        .polyline(profile[1:])
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )

    rim = (
        cq.Workplane("XY")
        .moveTo(radius, thickness * 0.55)
        .circle(0.020)
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    return shell.union(rim)


def _rear_frame(width: float = 0.50, height: float = 0.36, thickness: float = 0.025) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, height, thickness)
    window = cq.Workplane("XY").box(0.36, 0.24, thickness * 3.0)
    return outer.cut(window)


def _cylinder_between(start: tuple[float, float, float], end: tuple[float, float, float], radius: float) -> cq.Workplane:
    s = cq.Vector(*start)
    e = cq.Vector(*end)
    direction = e - s
    solid = cq.Solid.makeCylinder(radius, direction.Length, s, direction.normalized())
    return cq.Workplane("XY").add(solid)


def _feed_support() -> cq.Workplane:
    feed_tip = (0.0, -0.40, 0.0)
    starts = (
        (0.470, 0.030, 0.285),
        (-0.470, 0.030, 0.285),
    )
    shape = _cylinder_between(starts[0], feed_tip, 0.010)
    for start in starts[1:]:
        shape = shape.union(_cylinder_between(start, feed_tip, 0.010))
    feed_horn = _cylinder_between((0.0, -0.470, 0.0), (0.0, -0.365, 0.0), 0.038)
    return shape.union(feed_horn)


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracking_dish")

    steel = model.material("dark_powder_coat", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bearing = model.material("brushed_bearing_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    reflector_mat = model.material("warm_white_reflector", rgba=(0.86, 0.88, 0.84, 1.0))
    frame_mat = model.material("rear_cast_frame", rgba=(0.18, 0.22, 0.24, 1.0))
    cover_mat = model.material("dark_service_cover", rgba=(0.07, 0.09, 0.10, 1.0))
    accent = model.material("safety_orange_release", rgba=(0.95, 0.36, 0.08, 1.0))

    base = model.part("base")
    base.visual(Cylinder(radius=0.58, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=steel, name="base_slab")
    base.visual(Cylinder(radius=0.34, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.145)), material=bearing, name="fixed_bearing_ring")
    for i in range(8):
        angle = i * math.tau / 8.0
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.43 * math.cos(angle), 0.43 * math.sin(angle), 0.127)),
            material=bearing,
            name=f"bolt_{i}",
        )
    for i, (x, y) in enumerate(((0.42, 0.42), (-0.42, 0.42), (-0.42, -0.42), (0.42, -0.42))):
        base.visual(
            Cylinder(radius=0.060, length=0.026),
            origin=Origin(xyz=(x, y, 0.013)),
            material=black,
            name=f"foot_{i}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(Cylinder(radius=0.285, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=steel, name="turntable")
    pedestal.visual(Cylinder(radius=0.090, length=0.590), origin=Origin(xyz=(0.0, 0.0, 0.335)), material=steel, name="azimuth_column")
    pedestal.visual(Box((1.26, 0.58, 0.075)), origin=Origin(xyz=(0.0, -0.200, 0.165)), material=steel, name="crosshead")
    pedestal.visual(Box((0.085, 0.165, 0.660)), origin=Origin(xyz=(0.645, -0.450, 0.505)), material=steel, name="yoke_0")
    pedestal.visual(Box((0.085, 0.165, 0.660)), origin=Origin(xyz=(-0.645, -0.450, 0.505)), material=steel, name="yoke_1")
    pedestal.visual(
        Cylinder(radius=0.095, length=0.070),
        origin=Origin(xyz=(0.638, -0.450, 0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="bearing_0",
    )
    pedestal.visual(
        Cylinder(radius=0.095, length=0.070),
        origin=Origin(xyz=(-0.638, -0.450, 0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="bearing_1",
    )
    pedestal.visual(Box((0.12, 0.080, 0.250)), origin=Origin(xyz=(0.0, 0.110, 0.395)), material=frame_mat, name="rear_cable_box")

    dish = model.part("dish")
    dish.visual(
        mesh_from_cadquery(_dish_shell(), "parabolic_reflector", tolerance=0.002, angular_tolerance=0.08),
        material=reflector_mat,
        name="reflector",
    )
    dish.visual(
        Cylinder(radius=0.072, length=0.058),
        origin=Origin(xyz=(0.576, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="trunnion_0",
    )
    dish.visual(
        Cylinder(radius=0.072, length=0.058),
        origin=Origin(xyz=(-0.576, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="trunnion_1",
    )
    dish.visual(
        Cylinder(radius=0.135, length=0.070),
        origin=Origin(xyz=(0.0, 0.245, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="rear_boss",
    )
    dish.visual(
        mesh_from_cadquery(_feed_support(), "feed_support", tolerance=0.0015, angular_tolerance=0.08),
        material=bearing,
        name="feed_support",
    )
    dish.visual(
        mesh_from_cadquery(_rear_frame(), "rear_support_frame", tolerance=0.001, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.326, -0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="rear_frame",
    )
    dish.visual(Box((0.036, 0.185, 0.040)), origin=Origin(xyz=(0.220, 0.255, -0.170)), material=frame_mat, name="frame_strut_0")
    dish.visual(Box((0.036, 0.185, 0.040)), origin=Origin(xyz=(-0.220, 0.255, -0.170)), material=frame_mat, name="frame_strut_1")
    dish.visual(Box((0.032, 0.024, 0.250)), origin=Origin(xyz=(-0.176, 0.346, -0.170)), material=bearing, name="cover_hinge_mount")

    rear_cover = model.part("rear_cover")
    rear_cover.visual(Box((0.300, 0.014, 0.190)), origin=Origin(xyz=(0.150, 0.0, 0.0)), material=cover_mat, name="cover_panel")
    rear_cover.visual(Cylinder(radius=0.012, length=0.225), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=bearing, name="hinge_barrel")
    rear_cover.visual(Box((0.030, 0.020, 0.150)), origin=Origin(xyz=(0.020, -0.002, 0.0)), material=bearing, name="hinge_leaf")
    rear_cover.visual(Cylinder(radius=0.016, length=0.007), origin=Origin(xyz=(0.265, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=accent, name="release_knob")

    model.articulation(
        "base_to_pedestal",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8),
    )
    model.articulation(
        "pedestal_to_dish",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=dish,
        origin=Origin(xyz=(0.0, -0.450, 0.820)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=-0.35, upper=0.85),
    )
    model.articulation(
        "dish_to_rear_cover",
        ArticulationType.REVOLUTE,
        parent=dish,
        child=rear_cover,
        origin=Origin(xyz=(-0.150, 0.346, -0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    dish = object_model.get_part("dish")
    cover = object_model.get_part("rear_cover")
    azimuth = object_model.get_articulation("base_to_pedestal")
    elevation = object_model.get_articulation("pedestal_to_dish")
    cover_hinge = object_model.get_articulation("dish_to_rear_cover")

    ctx.allow_overlap(
        dish,
        cover,
        elem_a="cover_hinge_mount",
        elem_b="hinge_barrel",
        reason="The small rear-cover hinge barrel is intentionally captured in the frame-side hinge mount.",
    )
    ctx.allow_overlap(
        dish,
        pedestal,
        elem_a="trunnion_0",
        elem_b="bearing_0",
        reason="The right elevation trunnion is intentionally seated inside the yoke bearing.",
    )
    ctx.allow_overlap(
        dish,
        pedestal,
        elem_a="trunnion_1",
        elem_b="bearing_1",
        reason="The left elevation trunnion is intentionally seated inside the yoke bearing.",
    )

    ctx.expect_gap(pedestal, base, axis="z", positive_elem="turntable", negative_elem="fixed_bearing_ring", max_gap=0.0015, max_penetration=0.0, name="turntable sits on fixed bearing")
    ctx.expect_gap(pedestal, dish, axis="x", positive_elem="bearing_0", negative_elem="trunnion_0", max_gap=0.006, max_penetration=0.004, name="right trunnion seats at yoke bearing")
    ctx.expect_gap(dish, pedestal, axis="x", positive_elem="trunnion_1", negative_elem="bearing_1", max_gap=0.006, max_penetration=0.004, name="left trunnion seats at yoke bearing")
    ctx.expect_gap(cover, dish, axis="x", positive_elem="hinge_barrel", negative_elem="cover_hinge_mount", max_penetration=0.004, name="cover hinge barrel is locally captured")
    ctx.expect_within(cover, dish, axes="xz", inner_elem="cover_panel", outer_elem="rear_frame", margin=0.006, name="cover panel fits inside rear frame cutout")

    ctx.check("azimuth joint is vertical", tuple(azimuth.axis) == (0.0, 0.0, 1.0), details=f"axis={azimuth.axis}")
    ctx.check("elevation joint is horizontal", abs(elevation.axis[0]) == 1.0 and elevation.axis[1] == 0.0 and elevation.axis[2] == 0.0, details=f"axis={elevation.axis}")

    rest_cable = _aabb_center(ctx.part_element_world_aabb(pedestal, elem="rear_cable_box"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_cable = _aabb_center(ctx.part_element_world_aabb(pedestal, elem="rear_cable_box"))
    ctx.check(
        "azimuth rotates cable box around base",
        turned_cable[0] < rest_cable[0] - 0.07,
        details=f"rest={rest_cable}, turned={turned_cable}",
    )

    rest_frame = _aabb_center(ctx.part_element_world_aabb(dish, elem="rear_frame"))
    with ctx.pose({elevation: 0.75}):
        raised_frame = _aabb_center(ctx.part_element_world_aabb(dish, elem="rear_frame"))
    ctx.check(
        "elevation tilts the rear frame",
        abs(raised_frame[2] - rest_frame[2]) > 0.05,
        details=f"rest={rest_frame}, raised={raised_frame}",
    )

    closed_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_panel"))
    with ctx.pose({cover_hinge: 1.20}):
        open_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_panel"))
    ctx.check(
        "rear cover opens outward on hinge",
        open_cover[1] > closed_cover[1] + 0.10,
        details=f"closed={closed_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()
