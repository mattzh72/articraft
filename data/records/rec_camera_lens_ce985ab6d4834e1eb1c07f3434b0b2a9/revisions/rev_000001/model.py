from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder(radius: float, x_min: float, x_max: float):
    """CadQuery cylinder whose axis is the lens X axis."""

    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(x_max - x_min)
        .translate((x_min, 0.0, 0.0))
    )


def _hollow_x_tube(
    *,
    length: float,
    inner_radius: float,
    outer_radius: float,
    x_center: float = 0.0,
):
    x_min = x_center - length * 0.5
    x_max = x_center + length * 0.5
    return _x_cylinder(outer_radius, x_min, x_max).cut(
        _x_cylinder(inner_radius, x_min - 0.006, x_max + 0.006)
    )


def _build_barrel_mesh():
    """One connected, hollow, stepped telephoto barrel shell."""

    segments = [
        (-0.455, -0.365, 0.052),
        (-0.370, -0.315, 0.061),
        (-0.325, -0.160, 0.060),
        (-0.165, -0.105, 0.071),
        (-0.110, 0.125, 0.074),
        (0.120, 0.225, 0.083),
        (0.220, 0.425, 0.103),
        (0.398, 0.466, 0.111),
    ]
    body = _x_cylinder(segments[0][2], segments[0][0], segments[0][1])
    for x_min, x_max, radius in segments[1:]:
        body = body.union(_x_cylinder(radius, x_min, x_max))

    # The long central optical bore and the wider front glass recess keep the
    # lens from reading as a solid rod.
    body = body.cut(_x_cylinder(0.032, -0.470, 0.475))
    body = body.cut(_x_cylinder(0.086, 0.300, 0.480))
    body = body.cut(_x_cylinder(0.040, -0.475, -0.410))
    return body


def _build_grip_ring_mesh(
    *,
    length: float,
    inner_radius: float,
    core_radius: float,
    rib_height: float,
    rib_count: int,
):
    """Rubber control ring with raised axial ribs and hollow clearance."""

    x_min = -length * 0.5
    x_max = length * 0.5
    ring = _x_cylinder(core_radius, x_min, x_max)
    ring = ring.union(_x_cylinder(core_radius + 0.003, x_min, x_min + 0.009))
    ring = ring.union(_x_cylinder(core_radius + 0.003, x_max - 0.009, x_max))

    for index in range(rib_count):
        angle_deg = index * 360.0 / rib_count
        rib = (
            cq.Workplane("XY")
            .box(length * 0.83, 0.0036, rib_height)
            .translate((0.0, 0.0, core_radius + rib_height * 0.5 - 0.0007))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        ring = ring.union(rib)

    return ring.cut(_x_cylinder(inner_radius, x_min - 0.010, x_max + 0.010))


def _build_tripod_collar_mesh():
    """Rotating tripod collar with an integrated downward foot."""

    length = 0.074
    x_min = -length * 0.5
    x_max = length * 0.5
    collar = _x_cylinder(0.091, x_min, x_max)
    collar = collar.union(_x_cylinder(0.097, x_min, x_min + 0.010))
    collar = collar.union(_x_cylinder(0.097, x_max - 0.010, x_max))

    # Clamp ears at camera-right and an Arca-style foot below the barrel.
    ears = (
        cq.Workplane("XY")
        .box(0.050, 0.030, 0.034)
        .translate((0.0, 0.090, 0.000))
    )
    collar = collar.union(ears)
    neck = (
        cq.Workplane("XY")
        .box(0.046, 0.052, 0.072)
        .translate((0.0, 0.0, -0.125))
    )
    foot_plate = (
        cq.Workplane("XY")
        .box(0.180, 0.064, 0.018)
        .translate((0.0, 0.0, -0.166))
    )
    collar = collar.union(neck).union(foot_plate)

    # Keep the collar a true clearance ring around the barrel.
    return collar.cut(_x_cylinder(0.074, x_min - 0.010, x_max + 0.010))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.035, 0.036, 0.038, 1.0))
    graphite = model.material("graphite_metal", rgba=(0.12, 0.125, 0.13, 1.0))
    glass = model.material("coated_glass", rgba=(0.18, 0.32, 0.40, 0.48))
    deep_glass = model.material("rear_glass", rgba=(0.03, 0.04, 0.05, 0.70))
    white = model.material("engraving_white", rgba=(0.86, 0.88, 0.84, 1.0))
    red = model.material("mount_red", rgba=(0.70, 0.04, 0.03, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel_mesh(), "stepped_lens_barrel"),
        material=satin_black,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.087, length=0.008),
        origin=Origin(xyz=(0.302, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(-0.444, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deep_glass,
        name="rear_element",
    )
    barrel.visual(
        Box((0.006, 0.022, 0.0024)),
        origin=Origin(xyz=(-0.145, 0.0, 0.0718)),
        material=white,
        name="zoom_index",
    )
    barrel.visual(
        Box((0.006, 0.026, 0.0024)),
        origin=Origin(xyz=(0.070, 0.0, 0.0748)),
        material=white,
        name="focus_index",
    )
    barrel.visual(
        Sphere(radius=0.0045),
        origin=Origin(xyz=(-0.392, -0.039, 0.039)),
        material=red,
        name="mount_dot",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_cadquery(
            _build_grip_ring_mesh(
                length=0.145,
                inner_radius=0.0600,
                core_radius=0.078,
                rib_height=0.006,
                rib_count=36,
            ),
            "zoom_ring_ribbed",
        ),
        material=rubber,
        name="zoom_grip",
    )
    for mark_x, mark_y, mark_z, mark_len in (
        (-0.042, 0.0, 0.084, 0.018),
        (-0.006, 0.0, 0.084, 0.012),
        (0.030, 0.0, 0.084, 0.016),
    ):
        zoom_ring.visual(
            Box((0.003, mark_len, 0.002)),
            origin=Origin(xyz=(mark_x, mark_y, mark_z)),
            material=white,
            name=f"zoom_mark_{mark_x:.3f}".replace("-", "m").replace(".", "_"),
        )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            _build_grip_ring_mesh(
                length=0.112,
                inner_radius=0.0830,
                core_radius=0.096,
                rib_height=0.005,
                rib_count=44,
            ),
            "focus_ring_ribbed",
        ),
        material=rubber,
        name="focus_grip",
    )
    for index, mark_x in enumerate((-0.032, 0.000, 0.032)):
        focus_ring.visual(
            Box((0.003, 0.020 - index * 0.004, 0.002)),
            origin=Origin(xyz=(mark_x, 0.0, 0.101)),
            material=white,
            name=f"focus_mark_{index}",
        )

    collar = model.part("tripod_collar")
    collar.visual(
        mesh_from_cadquery(_build_tripod_collar_mesh(), "tripod_collar_foot"),
        material=graphite,
        name="collar_foot",
    )
    collar.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.107, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="clamp_screw",
    )
    collar.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.1765)),
        material=satin_black,
        name="tripod_socket",
    )

    model.articulation(
        "barrel_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(-0.245, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.7,
            velocity=3.5,
            lower=math.radians(-38.0),
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.45,
            velocity=4.0,
            lower=math.radians(-105.0),
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "barrel_to_tripod_collar",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=collar,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    collar = object_model.get_part("tripod_collar")

    zoom_joint = object_model.get_articulation("barrel_to_zoom_ring")
    focus_joint = object_model.get_articulation("barrel_to_focus_ring")
    collar_joint = object_model.get_articulation("barrel_to_tripod_collar")

    for ring, ring_elem, label in (
        (zoom_ring, "zoom_grip", "zoom"),
        (focus_ring, "focus_grip", "focus"),
        (collar, "collar_foot", "tripod collar"),
    ):
        ctx.allow_overlap(
            barrel,
            ring,
            elem_a="barrel_shell",
            elem_b=ring_elem,
            reason=(
                f"The {label} rotating sleeve is intentionally modeled as a "
                "seated bearing fit on the barrel proxy so it is captured and "
                "not floating."
            ),
        )
        ctx.expect_contact(
            ring,
            barrel,
            elem_a=ring_elem,
            elem_b="barrel_shell",
            name=f"{label} sleeve is seated on barrel",
        )

    ctx.check(
        "zoom and focus are limited revolute rings",
        zoom_joint.articulation_type == ArticulationType.REVOLUTE
        and focus_joint.articulation_type == ArticulationType.REVOLUTE
        and zoom_joint.motion_limits is not None
        and focus_joint.motion_limits is not None
        and zoom_joint.motion_limits.lower < zoom_joint.motion_limits.upper
        and focus_joint.motion_limits.lower < focus_joint.motion_limits.upper,
    )
    ctx.check(
        "tripod collar spins continuously",
        collar_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in collar_joint.axis) == (1.0, 0.0, 0.0),
    )

    for ring, name in ((zoom_ring, "zoom"), (focus_ring, "focus"), (collar, "collar")):
        ctx.expect_origin_distance(
            ring,
            barrel,
            axes="yz",
            max_dist=0.001,
            name=f"{name} ring is coaxial with barrel",
        )
        ctx.expect_overlap(
            ring,
            barrel,
            axes="x",
            min_overlap=0.050,
            name=f"{name} ring wraps an axial barrel section",
        )

    with ctx.pose({collar_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(collar, elem="collar_foot")
    with ctx.pose({collar_joint: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(collar, elem="collar_foot")

    ctx.check(
        "tripod foot swings around barrel",
        rest_aabb is not None
        and turned_aabb is not None
        and rest_aabb[0][2] < -0.14
        and turned_aabb[1][1] > 0.14,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
