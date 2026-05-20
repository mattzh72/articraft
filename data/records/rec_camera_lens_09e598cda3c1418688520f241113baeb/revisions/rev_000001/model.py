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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _x_cylinder_origin(x: float) -> Origin:
    return Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0))


def _cq_ring_x(inner_radius: float, outer_radius: float, length: float):
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length * 0.5, both=True)
    bore = cq.Workplane("YZ").circle(inner_radius).extrude((length + 0.006) * 0.5, both=True)
    return outer.cut(bore)


def _focus_ring_mesh():
    ring = _cq_ring_x(0.068, 0.073, 0.065)
    for i in range(13):
        x = -0.030 + i * 0.005
        rib = _cq_ring_x(0.068, 0.076, 0.0017).translate((x, 0.0, 0.0))
        ring = ring.union(rib)
    return ring


def _tripod_collar_mesh():
    collar = _cq_ring_x(0.069, 0.086, 0.056)
    stem = cq.Workplane("XY").box(0.045, 0.050, 0.092).translate((0.0, 0.0, -0.115))
    foot = cq.Workplane("XY").box(0.135, 0.052, 0.020).translate((-0.014, 0.0, -0.162))
    clamp_lug = cq.Workplane("XY").box(0.044, 0.036, 0.038).translate((0.0, 0.095, 0.014))
    return collar.union(stem).union(foot).union(clamp_lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_telephoto_prime")

    white = Material("warm_white", color=(0.91, 0.91, 0.84, 1.0))
    black = Material("matte_black", color=(0.005, 0.005, 0.004, 1.0))
    rubber = Material("ribbed_black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    glass = Material("dark_blue_glass", color=(0.015, 0.035, 0.050, 0.72))
    metal = Material("brushed_metal", color=(0.62, 0.64, 0.66, 1.0))
    red = Material("red_alignment_dot", color=(0.85, 0.04, 0.025, 1.0))

    barrel = model.part("lens_barrel")
    barrel.visual(
        _x_cylinder(0.061, 0.460),
        origin=_x_cylinder_origin(0.010),
        material=white,
        name="inner_barrel",
    )
    barrel.visual(
        _x_cylinder(0.066, 0.095),
        origin=_x_cylinder_origin(-0.153),
        material=white,
        name="rear_barrel",
    )
    barrel.visual(
        _x_cylinder(0.066, 0.108),
        origin=_x_cylinder_origin(0.030),
        material=white,
        name="middle_barrel",
    )
    barrel.visual(
        _x_cylinder(0.080, 0.108),
        origin=_x_cylinder_origin(0.218),
        material=white,
        name="front_barrel",
    )
    barrel.visual(
        _x_cylinder(0.052, 0.036),
        origin=_x_cylinder_origin(-0.228),
        material=black,
        name="rear_mount",
    )
    barrel.visual(
        _x_cylinder(0.056, 0.010),
        origin=_x_cylinder_origin(-0.205),
        material=metal,
        name="bayonet_ring",
    )
    barrel.visual(
        _x_cylinder(0.076, 0.018),
        origin=_x_cylinder_origin(0.276),
        material=black,
        name="front_filter_ring",
    )
    barrel.visual(
        _x_cylinder(0.061, 0.004),
        origin=_x_cylinder_origin(0.282),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        Sphere(0.0045),
        origin=Origin(xyz=(-0.174, -0.063, 0.020)),
        material=red,
        name="mount_dot",
    )
    barrel.visual(
        Box((0.052, 0.004, 0.018)),
        origin=Origin(xyz=(0.012, -0.0640, 0.020)),
        material=black,
        name="distance_window",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_focus_ring_mesh(), "ribbed_focus_ring", tolerance=0.0007),
        material=rubber,
        name="rubber_grip",
    )
    focus_ring.visual(
        Box((0.038, 0.006, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.076)),
        material=white,
        name="focus_index",
    )
    focus_ring.visual(
        Sphere(0.005),
        origin=Origin(xyz=(0.000, 0.000, 0.0645)),
        material=black,
        name="focus_bearing_pad",
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        mesh_from_cadquery(_tripod_collar_mesh(), "tripod_collar_body", tolerance=0.0007),
        material=white,
        name="collar_body",
    )
    tripod_collar.visual(
        Box((0.050, 0.007, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.087)),
        material=black,
        name="split_mark",
    )
    tripod_collar.visual(
        _x_cylinder(0.018, 0.028),
        origin=Origin(xyz=(0.000, 0.118, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="lock_knob",
    )
    tripod_collar.visual(
        Box((0.108, 0.039, 0.006)),
        origin=Origin(xyz=(-0.016, 0.000, -0.173)),
        material=black,
        name="rubber_foot_pad",
    )
    tripod_collar.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(-0.018, 0.0, -0.176)),
        material=metal,
        name="tripod_socket",
    )
    tripod_collar.visual(
        Sphere(0.005),
        origin=Origin(xyz=(0.000, 0.000, 0.0645)),
        material=black,
        name="collar_bearing_pad",
    )

    model.articulation(
        "barrel_to_collar",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=tripod_collar,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.4, upper=1.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("lens_barrel")
    collar = object_model.get_part("tripod_collar")
    focus_ring = object_model.get_part("focus_ring")
    collar_joint = object_model.get_articulation("barrel_to_collar")
    focus_joint = object_model.get_articulation("barrel_to_focus_ring")

    ctx.allow_overlap(
        focus_ring,
        barrel,
        elem_a="focus_bearing_pad",
        elem_b="inner_barrel",
        reason="A small hidden bearing pad is lightly preloaded against the lens barrel to support the rotating focus sleeve.",
    )
    ctx.allow_overlap(
        collar,
        barrel,
        elem_a="collar_bearing_pad",
        elem_b="inner_barrel",
        reason="A small hidden bearing pad represents the captured rotating tripod collar riding on the barrel.",
    )

    ctx.expect_origin_distance(
        collar,
        barrel,
        axes="yz",
        max_dist=0.0001,
        name="tripod collar rotates concentrically around lens axis",
    )
    ctx.expect_origin_distance(
        focus_ring,
        barrel,
        axes="yz",
        max_dist=0.0001,
        name="focus ring rotates concentrically around lens axis",
    )
    ctx.expect_overlap(
        collar,
        barrel,
        axes="x",
        min_overlap=0.050,
        elem_a="collar_body",
        elem_b="inner_barrel",
        name="collar sleeve surrounds the barrel along its width",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="x",
        min_overlap=0.060,
        elem_a="rubber_grip",
        elem_b="inner_barrel",
        name="focus ring surrounds the front barrel section",
    )
    ctx.expect_gap(
        focus_ring,
        barrel,
        axis="z",
        positive_elem="focus_bearing_pad",
        negative_elem="inner_barrel",
        max_penetration=0.003,
        name="focus bearing pad is only lightly preloaded",
    )
    ctx.expect_gap(
        collar,
        barrel,
        axis="z",
        positive_elem="collar_bearing_pad",
        negative_elem="inner_barrel",
        max_penetration=0.003,
        name="collar bearing pad is only lightly preloaded",
    )

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_foot = elem_center(collar, "rubber_foot_pad")
    rest_index = elem_center(focus_ring, "focus_index")
    with ctx.pose({collar_joint: math.pi / 2.0, focus_joint: 0.9}):
        turned_foot = elem_center(collar, "rubber_foot_pad")
        turned_index = elem_center(focus_ring, "focus_index")

    ctx.check(
        "tripod foot swings around lens axis",
        rest_foot is not None
        and turned_foot is not None
        and rest_foot[2] < -0.16
        and turned_foot[1] > 0.15,
        details=f"rest_foot={rest_foot}, turned_foot={turned_foot}",
    )
    ctx.check(
        "focus index turns with focus ring",
        rest_index is not None
        and turned_index is not None
        and rest_index[2] > 0.070
        and turned_index[1] < -0.050,
        details=f"rest_index={rest_index}, turned_index={turned_index}",
    )

    return ctx.report()


object_model = build_object_model()
