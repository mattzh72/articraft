from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Return a cylinder descriptor plus a local rotation that lays it on +X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="picket_garden_gate")

    white_paint = model.material("weathered_white_paint", rgba=(0.86, 0.88, 0.82, 1.0))
    post_wood = model.material("oiled_post_wood", rgba=(0.46, 0.31, 0.18, 1.0))
    dark_iron = model.material("blackened_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    galvanized = model.material("galvanized_wear", rgba=(0.50, 0.52, 0.50, 1.0))
    stone = model.material("flagstone", rgba=(0.45, 0.43, 0.38, 1.0))

    picket_cap_mesh = mesh_from_geometry(
        ConeGeometry(radius=0.050, height=0.130, radial_segments=4),
        "picket_cap",
    )
    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.055, tube=0.006, radial_segments=16, tubular_segments=48),
        "ring_latch_loop",
    )
    guide_loop_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.015, tube=0.0058, radial_segments=12, tubular_segments=36),
        "drop_bolt_guide_loop",
    )

    posts = model.part("posts")
    posts.visual(
        Box((1.42, 0.92, 0.035)),
        origin=Origin(xyz=(0.55, 0.34, 0.0175)),
        material=stone,
        name="stone_pad",
    )
    posts.visual(
        Box((0.11, 0.11, 1.45)),
        origin=Origin(xyz=(-0.06, 0.0, 0.725)),
        material=post_wood,
        name="hinge_post",
    )
    posts.visual(
        Box((0.11, 0.11, 1.45)),
        origin=Origin(xyz=(1.14, 0.0, 0.725)),
        material=post_wood,
        name="latch_post",
    )
    posts.visual(
        Box((0.15, 0.15, 0.045)),
        origin=Origin(xyz=(-0.06, 0.0, 1.472)),
        material=post_wood,
        name="hinge_post_cap",
    )
    posts.visual(
        Box((0.15, 0.15, 0.045)),
        origin=Origin(xyz=(1.14, 0.0, 1.472)),
        material=post_wood,
        name="latch_post_cap",
    )
    posts.visual(
        Box((0.040, 0.065, 0.36)),
        origin=Origin(xyz=(1.065, -0.075, 0.73)),
        material=dark_iron,
        name="latch_stop",
    )
    posts.visual(
        Box((0.045, 0.030, 0.12)),
        origin=Origin(xyz=(1.086, -0.075, 0.80)),
        material=dark_iron,
        name="keeper_staple",
    )
    for index, z_center in enumerate((0.50, 1.04)):
        posts.visual(
            Cylinder(radius=0.020, length=0.18),
            origin=Origin(xyz=(0.0, -0.075, z_center)),
            material=dark_iron,
            name=f"hinge_pin_{index}",
        )
        posts.visual(
            Box((0.045, 0.018, 0.22)),
            origin=Origin(xyz=(-0.030, -0.055, z_center)),
            material=dark_iron,
            name=f"post_hinge_leaf_{index}",
        )
    posts.visual(
        Box((0.10, 0.10, 0.012)),
        origin=Origin(xyz=(0.35, 0.665, 0.041)),
        material=galvanized,
        name="open_bolt_socket",
    )
    posts.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.35, 0.665, 0.052)),
        material=dark_iron,
        name="socket_hole_rim",
    )

    leaf = model.part("leaf")
    # Rectangular leaf frame.
    leaf.visual(
        Box((0.070, 0.050, 1.08)),
        origin=Origin(xyz=(0.060, 0.0, 0.68)),
        material=white_paint,
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.070, 0.050, 1.08)),
        origin=Origin(xyz=(1.000, 0.0, 0.68)),
        material=white_paint,
        name="latch_stile",
    )
    leaf.visual(
        Box((0.940, 0.055, 0.100)),
        origin=Origin(xyz=(0.530, 0.0, 0.300)),
        material=white_paint,
        name="lower_rail",
    )
    leaf.visual(
        Box((0.940, 0.055, 0.075)),
        origin=Origin(xyz=(0.530, 0.0, 0.630)),
        material=white_paint,
        name="middle_rail",
    )
    leaf.visual(
        Box((0.940, 0.055, 0.100)),
        origin=Origin(xyz=(0.530, 0.0, 0.980)),
        material=white_paint,
        name="upper_rail",
    )
    leaf.visual(
        Box((1.060, 0.040, 0.050)),
        origin=Origin(xyz=(0.535, 0.006, 0.690), rpy=(0.0, -0.66, 0.0)),
        material=white_paint,
        name="diagonal_brace",
    )
    for index, x_pos in enumerate((0.17, 0.32, 0.47, 0.62, 0.77, 0.92)):
        leaf.visual(
            Box((0.070, 0.035, 0.960)),
            origin=Origin(xyz=(x_pos, -0.002, 0.700)),
            material=white_paint,
            name=f"picket_{index}",
        )
        leaf.visual(
            picket_cap_mesh,
            origin=Origin(xyz=(x_pos, -0.002, 1.245), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=white_paint,
            name=f"picket_cap_{index}",
        )
    # Moving-leaf hinge straps mounted to the hinge stile.
    for index, z_center in enumerate((0.50, 1.04)):
        leaf.visual(
            Box((0.225, 0.014, 0.058)),
            origin=Origin(xyz=(0.135, -0.020, z_center)),
            material=dark_iron,
            name=f"gate_hinge_strap_{index}",
        )
        leaf.visual(
            Box((0.035, 0.030, 0.13)),
            origin=Origin(xyz=(0.010, 0.000, z_center)),
            material=dark_iron,
            name=f"gate_hinge_knuckle_{index}",
        )
    # Fixed latch backplate and the guide hardware for the sliding drop bolt.
    leaf.visual(
        Box((0.135, 0.012, 0.165)),
        origin=Origin(xyz=(0.910, -0.030, 0.780)),
        material=dark_iron,
        name="ring_backplate",
    )
    leaf.visual(
        Box((0.055, 0.010, 0.560)),
        origin=Origin(xyz=(0.820, -0.029, 0.340)),
        material=dark_iron,
        name="bolt_backplate",
    )
    for name, z_center in (("lower", 0.235), ("upper", 0.455)):
        leaf.visual(
            Box((0.010, 0.032, 0.040)),
            origin=Origin(xyz=(0.796, -0.048, z_center)),
            material=galvanized,
            name=f"bolt_guide_{name}_side_0",
        )
        leaf.visual(
            Box((0.010, 0.032, 0.040)),
            origin=Origin(xyz=(0.844, -0.048, z_center)),
            material=galvanized,
            name=f"bolt_guide_{name}_side_1",
        )
        leaf.visual(
            Box((0.058, 0.010, 0.032)),
            origin=Origin(xyz=(0.820, -0.066, z_center)),
            material=galvanized,
            name=f"bolt_guide_{name}_front",
        )
        leaf.visual(
            Box((0.012, 0.006, 0.018)),
            origin=Origin(xyz=(0.820, -0.0605, z_center)),
            material=galvanized,
            name=f"bolt_guide_{name}_clip",
        )

    ring_latch = model.part("ring_latch")
    ring_latch.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="ring_loop",
    )
    ring_latch.visual(
        Cylinder(radius=0.015, length=0.025),
        origin=Origin(xyz=(0.0, -0.0125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="pivot_boss",
    )
    ring_latch.visual(
        Box((0.014, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, -0.027, 0.036)),
        material=dark_iron,
        name="ring_spoke",
    )
    ring_latch.visual(
        Box((0.150, 0.016, 0.022)),
        origin=Origin(xyz=(0.075, -0.025, 0.0)),
        material=dark_iron,
        name="latch_bar",
    )

    drop_bolt = model.part("drop_bolt")
    drop_bolt.visual(
        Cylinder(radius=0.010, length=0.580),
        origin=Origin(),
        material=galvanized,
        name="bolt_rod",
    )
    drop_bolt.visual(
        Box((0.078, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, 0.210)),
        material=galvanized,
        name="bolt_grip",
    )
    drop_bolt.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=galvanized,
        name="bolt_tip",
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(0.0, -0.075, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.20),
    )
    model.articulation(
        "ring_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=ring_latch,
        origin=Origin(xyz=(0.910, -0.036, 0.780)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "bolt_slide",
        ArticulationType.PRISMATIC,
        parent=leaf,
        child=drop_bolt,
        origin=Origin(xyz=(0.820, -0.048, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    ring_latch = object_model.get_part("ring_latch")
    drop_bolt = object_model.get_part("drop_bolt")
    gate_hinge = object_model.get_articulation("gate_hinge")
    ring_pivot = object_model.get_articulation("ring_pivot")
    bolt_slide = object_model.get_articulation("bolt_slide")

    for index in (0, 1):
        ctx.allow_overlap(
            leaf,
            posts,
            elem_a=f"gate_hinge_knuckle_{index}",
            elem_b=f"hinge_pin_{index}",
            reason="The gate-side hinge knuckle intentionally captures the fixed hinge pin on the post.",
        )
        ctx.expect_overlap(
            leaf,
            posts,
            axes="z",
            elem_a=f"gate_hinge_knuckle_{index}",
            elem_b=f"hinge_pin_{index}",
            min_overlap=0.10,
            name=f"hinge knuckle {index} surrounds pin vertically",
        )
        ctx.expect_overlap(
            leaf,
            posts,
            axes="xy",
            elem_a=f"gate_hinge_knuckle_{index}",
            elem_b=f"hinge_pin_{index}",
            min_overlap=0.010,
            name=f"hinge knuckle {index} captures pin in plan",
        )

    ctx.expect_gap(
        posts,
        leaf,
        axis="x",
        positive_elem="latch_stop",
        negative_elem="latch_stile",
        min_gap=0.004,
        max_gap=0.025,
        name="closed leaf seats just before latch stop",
    )
    ctx.expect_overlap(
        posts,
        leaf,
        axes="yz",
        elem_a="latch_stop",
        elem_b="latch_stile",
        min_overlap=0.030,
        name="latch stop aligns with closing stile",
    )
    ctx.expect_contact(
        ring_latch,
        leaf,
        elem_a="pivot_boss",
        elem_b="ring_backplate",
        contact_tol=0.002,
        name="ring latch pivot is mounted to its backplate",
    )

    def _center_axis(aabb, axis_index: int):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    closed_latch_stile = ctx.part_element_world_aabb(leaf, elem="latch_stile")
    with ctx.pose({gate_hinge: 1.20}):
        open_latch_stile = ctx.part_element_world_aabb(leaf, elem="latch_stile")
    closed_y = _center_axis(closed_latch_stile, 1)
    open_y = _center_axis(open_latch_stile, 1)
    ctx.check(
        "gate leaf swings outward on vertical hinge",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.55,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    closed_latch_bar = ctx.part_element_world_aabb(ring_latch, elem="latch_bar")
    with ctx.pose({ring_pivot: 0.85}):
        turned_latch_bar = ctx.part_element_world_aabb(ring_latch, elem="latch_bar")
    closed_z = _center_axis(closed_latch_bar, 2)
    turned_z = _center_axis(turned_latch_bar, 2)
    ctx.check(
        "ring latch rotates its latch bar",
        closed_z is not None and turned_z is not None and abs(turned_z - closed_z) > 0.035,
        details=f"closed_z={closed_z}, turned_z={turned_z}",
    )

    for guide_name in ("lower", "upper"):
        ctx.expect_gap(
            drop_bolt,
            leaf,
            axis="x",
            positive_elem="bolt_rod",
            negative_elem=f"bolt_guide_{guide_name}_side_0",
            min_gap=0.004,
            max_gap=0.016,
            name=f"bolt rod clears {guide_name} left side",
        )
        ctx.expect_gap(
            leaf,
            drop_bolt,
            axis="x",
            positive_elem=f"bolt_guide_{guide_name}_side_1",
            negative_elem="bolt_rod",
            min_gap=0.004,
            max_gap=0.016,
            name=f"bolt rod clears {guide_name} right side",
        )
        ctx.expect_overlap(
            drop_bolt,
            leaf,
            axes="z",
            elem_a="bolt_rod",
            elem_b=f"bolt_guide_{guide_name}_side_0",
            min_overlap=0.020,
            name=f"bolt remains through {guide_name} guide",
        )

    rest_rod = ctx.part_element_world_aabb(drop_bolt, elem="bolt_rod")
    with ctx.pose({bolt_slide: 0.18}):
        raised_rod = ctx.part_element_world_aabb(drop_bolt, elem="bolt_rod")
        for guide_name in ("lower", "upper"):
            ctx.expect_overlap(
                drop_bolt,
                leaf,
                axes="z",
                elem_a="bolt_rod",
                elem_b=f"bolt_guide_{guide_name}_side_0",
                min_overlap=0.020,
                name=f"raised bolt remains through {guide_name} guide",
            )
    rest_bottom = rest_rod[0][2] if rest_rod is not None else None
    raised_bottom = raised_rod[0][2] if raised_rod is not None else None
    ctx.check(
        "drop bolt slides upward in its guides",
        rest_bottom is not None and raised_bottom is not None and raised_bottom > rest_bottom + 0.15,
        details=f"rest_bottom={rest_bottom}, raised_bottom={raised_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
