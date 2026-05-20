from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder primitives are local-Z aligned; rotate them onto the truck width axis."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_grab_handle")

    paint = model.material("deep_blue_paint", rgba=(0.02, 0.12, 0.28, 1.0))
    bedliner = model.material("sprayed_black_bedliner", rgba=(0.015, 0.017, 0.018, 1.0))
    pocket_shadow = model.material("recess_shadow", rgba=(0.003, 0.003, 0.004, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    bare_steel = model.material("worn_pivot_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    rubber = model.material("dark_rubber_stops", rgba=(0.01, 0.01, 0.01, 1.0))

    bed_edge = model.part("bed_edge")
    bed_edge.visual(
        Box((1.95, 0.11, 0.09)),
        origin=Origin(xyz=(0.0, 0.125, -0.055)),
        material=dark_steel,
        name="lower_sill",
    )
    bed_edge.visual(
        Box((1.95, 0.045, 0.23)),
        origin=Origin(xyz=(0.0, 0.185, 0.075)),
        material=bedliner,
        name="bed_lip",
    )
    bed_edge.visual(
        Box((1.86, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.030)),
        material=bare_steel,
        name="hinge_backing_strip",
    )
    bed_edge.visual(
        Box((1.86, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, 0.105, 0.005)),
        material=dark_steel,
        name="backing_web",
    )

    for x, bracket_name, hinge_name, pad_name in (
        (-0.80, "hinge_bracket_0", "bed_hinge_0", "bumper_pad_0"),
        (0.80, "hinge_bracket_1", "bed_hinge_1", "bumper_pad_1"),
    ):
        bed_edge.visual(
            Box((0.19, 0.105, 0.034)),
            origin=Origin(xyz=(x, 0.070, 0.000)),
            material=dark_steel,
            name=bracket_name,
        )
        bed_edge.visual(
            Cylinder(radius=0.023, length=0.120),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=bare_steel,
            name=hinge_name,
        )
        bed_edge.visual(
            Box((0.13, 0.018, 0.055)),
            origin=Origin(xyz=(x, 0.054, 0.010)),
            material=rubber,
            name=pad_name,
        )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.82, 0.074, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=paint,
        name="gate_shell",
    )
    tailgate.visual(
        Box((1.86, 0.090, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.748)),
        material=paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((1.86, 0.090, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=paint,
        name="bottom_cap",
    )
    for i, x in enumerate((-0.925, 0.925)):
        tailgate.visual(
            Box((0.050, 0.088, 0.730)),
            origin=Origin(xyz=(x, 0.0, 0.390)),
            material=paint,
            name=f"side_cap_{i}",
        )

    tailgate.visual(
        Box((1.66, 0.010, 0.610)),
        origin=Origin(xyz=(0.0, 0.040, 0.395)),
        material=bedliner,
        name="inner_liner",
    )
    for i, x in enumerate((-0.58, 0.58)):
        tailgate.visual(
            Box((0.055, 0.018, 0.540)),
            origin=Origin(xyz=(x, 0.050, 0.390)),
            material=bedliner,
            name=f"inner_rib_{i}",
        )
    tailgate.visual(
        Box((1.38, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.050, 0.205)),
        material=bedliner,
        name="lower_inner_rib",
    )

    tailgate.visual(
        Box((0.760, 0.006, 0.250)),
        origin=Origin(xyz=(0.0, 0.045, 0.585)),
        material=pocket_shadow,
        name="handle_pocket",
    )
    tailgate.visual(
        Box((0.840, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.052, 0.728)),
        material=bedliner,
        name="pocket_upper_rim",
    )
    tailgate.visual(
        Box((0.840, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.052, 0.442)),
        material=bedliner,
        name="pocket_lower_rim",
    )
    for i, x in enumerate((-0.410, 0.410)):
        tailgate.visual(
            Box((0.030, 0.014, 0.286)),
            origin=Origin(xyz=(x, 0.052, 0.585)),
            material=bedliner,
            name=f"pocket_side_rim_{i}",
        )

    for socket_x, bridge_x, socket_name, bridge_name in (
        (-0.3625, -0.385, "pivot_socket_0", "socket_bridge_0"),
        (0.3625, 0.385, "pivot_socket_1", "socket_bridge_1"),
    ):
        tailgate.visual(
            Box((0.045, 0.030, 0.060)),
            origin=Origin(xyz=(socket_x, 0.065, 0.665)),
            material=bare_steel,
            name=socket_name,
        )
        tailgate.visual(
            Box((0.030, 0.026, 0.055)),
            origin=Origin(xyz=(bridge_x, 0.052, 0.665)),
            material=bedliner,
            name=bridge_name,
        )

    for x, gate_hinge_name, leaf_name in (
        (-0.64, "gate_hinge_0", "hinge_leaf_0"),
        (0.64, "gate_hinge_1", "hinge_leaf_1"),
    ):
        tailgate.visual(
            Cylinder(radius=0.022, length=0.200),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=bare_steel,
            name=gate_hinge_name,
        )
        tailgate.visual(
            Box((0.205, 0.048, 0.060)),
            origin=Origin(xyz=(x, -0.006, 0.035)),
            material=paint,
            name=leaf_name,
        )

    handle = model.part("grab_handle")
    handle_tube = wire_from_points(
        [
            (-0.290, 0.0, 0.000),
            (-0.290, 0.0, -0.170),
            (0.290, 0.0, -0.170),
            (0.290, 0.0, 0.000),
        ],
        radius=0.012,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.035,
        corner_segments=10,
    )
    handle.visual(
        mesh_from_geometry(handle_tube, "grab_tube"),
        material=bare_steel,
        name="grab_tube",
    )
    for x, pin_name in (
        (-0.315, "pivot_pin_0"),
        (0.315, "pivot_pin_1"),
    ):
        handle.visual(
            Cylinder(radius=0.014, length=0.050),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=bare_steel,
            name=pin_name,
        )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_edge,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.9, lower=0.0, upper=math.radians(92.0)),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=handle,
        origin=Origin(xyz=(0.0, 0.065, 0.665)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=0.0, upper=math.radians(72.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_edge = object_model.get_part("bed_edge")
    tailgate = object_model.get_part("tailgate")
    handle = object_model.get_part("grab_handle")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_contact(
        tailgate,
        bed_edge,
        elem_a="gate_hinge_0",
        elem_b="bed_hinge_0",
        contact_tol=0.0015,
        name="lower hinge knuckles meet on one side",
    )
    ctx.expect_contact(
        tailgate,
        bed_edge,
        elem_a="gate_hinge_1",
        elem_b="bed_hinge_1",
        contact_tol=0.0015,
        name="lower hinge knuckles meet on other side",
    )
    ctx.expect_contact(
        handle,
        tailgate,
        elem_a="pivot_pin_0",
        elem_b="pivot_socket_0",
        contact_tol=0.0015,
        name="grab handle has one side pivot",
    )
    ctx.expect_contact(
        handle,
        tailgate,
        elem_a="pivot_pin_1",
        elem_b="pivot_socket_1",
        contact_tol=0.0015,
        name="grab handle has other side pivot",
    )
    ctx.expect_within(
        handle,
        tailgate,
        axes="xz",
        inner_elem="grab_tube",
        outer_elem="handle_pocket",
        margin=0.006,
        name="folded handle sits inside the recessed pocket outline",
    )
    ctx.expect_gap(
        handle,
        tailgate,
        axis="y",
        positive_elem="grab_tube",
        negative_elem="handle_pocket",
        min_gap=0.001,
        max_gap=0.025,
        name="folded handle is just proud of the pocket floor",
    )

    def coord(vec, index: int) -> float:
        if hasattr(vec, "x"):
            return (vec.x, vec.y, vec.z)[index]
        return vec[index]

    def aabb_axis(aabb, end: int, index: int) -> float | None:
        if aabb is None:
            return None
        return coord(aabb[end], index)

    closed_gate = ctx.part_world_aabb(tailgate)
    with ctx.pose({tailgate_hinge: math.radians(78.0)}):
        lowered_gate = ctx.part_world_aabb(tailgate)
    ctx.check(
        "tailgate rotates downward from the lower bed edge",
        closed_gate is not None
        and lowered_gate is not None
        and aabb_axis(lowered_gate, 1, 2) < aabb_axis(closed_gate, 1, 2) - 0.28
        and aabb_axis(lowered_gate, 0, 1) < aabb_axis(closed_gate, 0, 1) - 0.55,
        details=f"closed={closed_gate}, lowered={lowered_gate}",
    )

    folded_handle = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: math.radians(62.0)}):
        raised_handle = ctx.part_world_aabb(handle)
    ctx.check(
        "grab handle folds outward from the upper inner face",
        folded_handle is not None
        and raised_handle is not None
        and aabb_axis(raised_handle, 1, 1) > aabb_axis(folded_handle, 1, 1) + 0.11,
        details=f"folded={folded_handle}, folded_out={raised_handle}",
    )

    return ctx.report()


object_model = build_object_model()
