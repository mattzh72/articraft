from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
)


def _annular_y_mesh(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    """A circular annulus extruded along local Y."""
    outer = CylinderGeometry(radius=outer_radius, height=depth, radial_segments=segments)
    cutter = CylinderGeometry(radius=inner_radius, height=depth + 0.008, radial_segments=segments)
    ring = boolean_difference(outer, cutter)
    return ring.rotate_x(math.pi / 2.0)


def _cylinder_y_mesh(radius: float, depth: float, *, segments: int = 72) -> MeshGeometry:
    return CylinderGeometry(radius=radius, height=depth, radial_segments=segments).rotate_x(
        math.pi / 2.0
    )


def _torus_y_mesh(radius: float, tube: float, *, radial_segments: int = 72) -> MeshGeometry:
    return TorusGeometry(
        radius=radius,
        tube=tube,
        radial_segments=18,
        tubular_segments=radial_segments,
    ).rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_porthole_hatch")

    painted_steel = model.material("painted_steel", rgba=(0.10, 0.30, 0.42, 1.0))
    worn_edge = model.material("worn_edge", rgba=(0.50, 0.56, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.34, 0.36, 0.37, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    brass = model.material("oiled_bronze", rgba=(0.68, 0.49, 0.22, 1.0))

    ring_frame = model.part("ring_frame")
    ring_frame.visual(
        mesh_from_geometry(_annular_y_mesh(0.445, 0.310, 0.100), "ring_body"),
        material=painted_steel,
        name="ring_body",
    )
    ring_frame.visual(
        mesh_from_geometry(_torus_y_mesh(0.327, 0.010), "inner_rounded_lip"),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=worn_edge,
        name="inner_lip",
    )
    ring_frame.visual(
        mesh_from_geometry(_torus_y_mesh(0.438, 0.011), "outer_rounded_lip"),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material=worn_edge,
        name="outer_lip",
    )

    # Bolt heads are slightly seated into the front of the annular frame.
    for index in range(12):
        angle = index * math.tau / 12.0
        radius = 0.382
        ring_frame.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(
                xyz=(radius * math.cos(angle), -0.057, radius * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_steel,
            name=f"bolt_{index}",
        )

    # Fixed hinge-side hardware: a side leaf, bridge lugs, and two stationary knuckles.
    ring_frame.visual(
        Box((0.075, 0.022, 0.470)),
        origin=Origin(xyz=(-0.405, -0.038, 0.0)),
        material=hinge_steel,
        name="fixed_hinge_leaf",
    )
    for z_pos in (-0.145, 0.145):
        ring_frame.visual(
            Box((0.052, 0.038, 0.105)),
            origin=Origin(xyz=(-0.430, -0.067, z_pos)),
            material=hinge_steel,
            name=f"hinge_bridge_{z_pos:+.2f}",
        )
        ring_frame.visual(
            Cylinder(radius=0.027, length=0.122),
            origin=Origin(xyz=(-0.430, -0.0775, z_pos)),
            material=hinge_steel,
            name=f"fixed_knuckle_{z_pos:+.2f}",
        )

    # Opposite-side latch keeper bolted to the ring, reaching forward toward the dog.
    ring_frame.visual(
        Box((0.094, 0.074, 0.082)),
        origin=Origin(xyz=(0.435, -0.085, 0.0)),
        material=hinge_steel,
        name="latch_keeper",
    )
    ring_frame.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.430, -0.105, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="keeper_bolt_upper",
    )
    ring_frame.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.430, -0.105, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="keeper_bolt_lower",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_cylinder_y_mesh(0.300, 0.045), "lid_disk"),
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        material=painted_steel,
        name="lid_disk",
    )
    lid.visual(
        mesh_from_geometry(_annular_y_mesh(0.282, 0.238, 0.014), "raised_lid_rim"),
        origin=Origin(xyz=(0.430, -0.028, 0.0)),
        material=worn_edge,
        name="raised_rim",
    )
    lid.visual(
        mesh_from_geometry(_annular_y_mesh(0.292, 0.252, 0.011), "back_gasket"),
        origin=Origin(xyz=(0.430, 0.025, 0.0)),
        material=rubber,
        name="gasket",
    )
    lid.visual(
        Box((0.400, 0.018, 0.036)),
        origin=Origin(xyz=(0.430, -0.031, 0.0)),
        material=worn_edge,
        name="horizontal_rib",
    )
    lid.visual(
        Box((0.036, 0.018, 0.455)),
        origin=Origin(xyz=(0.430, -0.032, 0.0)),
        material=worn_edge,
        name="vertical_rib",
    )
    lid.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.430, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="center_boss",
    )
    lid.visual(
        Box((0.180, 0.020, 0.135)),
        origin=Origin(xyz=(0.090, -0.028, 0.0)),
        material=hinge_steel,
        name="moving_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="moving_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.680, -0.0325, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="handle_bearing",
    )

    clamp_handle = model.part("clamp_handle")
    clamp_handle.visual(
        Cylinder(radius=0.043, length=0.025),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    clamp_handle.visual(
        Box((0.040, 0.030, 0.255)),
        origin=Origin(xyz=(0.0, -0.010, -0.125)),
        material=dark_steel,
        name="grip",
    )
    clamp_handle.visual(
        Box((0.150, 0.020, 0.036)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=brass,
        name="dog_tongue",
    )
    clamp_handle.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, -0.010, -0.265)),
        material=dark_steel,
        name="rounded_grip_end",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=ring_frame,
        child=lid,
        origin=Origin(xyz=(-0.430, -0.0775, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.1, lower=0.0, upper=1.55),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=clamp_handle,
        origin=Origin(xyz=(0.680, -0.055, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.15, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ring = object_model.get_part("ring_frame")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("clamp_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_gap(
        ring,
        lid,
        axis="y",
        max_gap=0.007,
        max_penetration=0.0,
        positive_elem="ring_body",
        negative_elem="lid_disk",
        name="closed lid sits just proud of the frame",
    )
    ctx.expect_gap(
        lid,
        handle,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="handle_bearing",
        negative_elem="pivot_hub",
        name="clamp hub seats against its bearing",
    )
    ctx.expect_overlap(
        lid,
        ring,
        axes="xz",
        min_overlap=0.55,
        elem_a="lid_disk",
        elem_b="ring_body",
        name="round lid spans the porthole opening",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    if closed_aabb is not None and open_aabb is not None:
        closed_min_y = closed_aabb[0][1]
        open_min_y = open_aabb[0][1]
        ctx.check(
            "lid hinge swings the hatch outward",
            open_min_y < closed_min_y - 0.20,
            details=f"closed_min_y={closed_min_y}, open_min_y={open_min_y}",
        )
    else:
        ctx.fail("lid hinge swings the hatch outward", "Could not measure lid disk AABB.")

    def _center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    handle_rest = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_pivot: 0.75}):
        handle_turned = ctx.part_element_world_aabb(handle, elem="grip")
    rest_x = _center_x(handle_rest)
    turned_x = _center_x(handle_turned)
    ctx.check(
        "dog latch handle rotates on its short pivot",
        rest_x is not None and turned_x is not None and turned_x < rest_x - 0.055,
        details=f"rest_x={rest_x}, turned_x={turned_x}",
    )

    return ctx.report()


object_model = build_object_model()
