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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_cylinder(outer_radius: float, inner_radius: float, length: float, name: str):
    """CadQuery washer tube, centered on local Z, used for hinge/caster sleeves."""
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tubular_side_yard_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.46, 0.50, 0.48, 1.0))
    dark_green = model.material("green_powder_coat", rgba=(0.05, 0.19, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    zinc = model.material("zinc_hardware", rgba=(0.66, 0.66, 0.60, 1.0))

    # Fixed hinge post.  Its frame is the vertical hinge axis at ground level.
    post = model.part("post")
    post.visual(
        Box((0.30, 0.24, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, -0.04)),
        material=concrete,
        name="concrete_pad",
    )
    post.visual(
        Box((0.22, 0.16, 0.018)),
        origin=Origin(xyz=(-0.10, 0.0, 0.009)),
        material=zinc,
        name="base_plate",
    )
    post.visual(
        Cylinder(radius=0.045, length=1.75),
        origin=Origin(xyz=(-0.10, 0.0, 0.875)),
        material=galvanized,
        name="post_tube",
    )

    hinge_levels = (0.55, 1.25)
    for i, z in enumerate(hinge_levels):
        post.visual(
            Cylinder(radius=0.014, length=0.34),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=zinc,
            name=f"hinge_pin_{i}",
        )
        # Clevis plates welded from the post to the protruding hinge pin ends.
        for suffix, dz in (("lower", -0.185), ("upper", 0.185)):
            post.visual(
                Box((0.12, 0.050, 0.040)),
                origin=Origin(xyz=(-0.050, 0.0, z + dz)),
                material=zinc,
                name=f"{suffix}_clevis_{i}",
            )
        post.visual(
            Box((0.080, 0.035, 0.060)),
            origin=Origin(xyz=(-0.075, 0.0, z)),
            material=zinc,
            name=f"hinge_arm_{i}",
        )

    # The gate leaf is authored in the hinge frame; at q=0 it extends along +X.
    leaf = model.part("leaf")
    tube_radius = 0.025
    x_hinge = 0.10
    x_free = 2.35
    x_mid = 0.5 * (x_hinge + x_free)
    rail_length = x_free - x_hinge
    z_bottom = 0.40
    z_top = 1.42
    z_mid = 0.5 * (z_bottom + z_top)
    stile_length = z_top - z_bottom

    for name, z in (
        ("bottom_tube", z_bottom),
        ("rail_0", 0.74),
        ("rail_1", 1.08),
        ("top_tube", z_top),
    ):
        leaf.visual(
            Cylinder(radius=tube_radius, length=rail_length),
            origin=Origin(xyz=(x_mid, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_green,
            name=name,
        )

    for name, x in (("hinge_stile", x_hinge), ("free_stile", x_free)):
        leaf.visual(
            Cylinder(radius=tube_radius, length=stile_length),
            origin=Origin(xyz=(x, 0.0, z_mid)),
            material=dark_green,
            name=name,
        )

    # Slender welded infill pickets make the leaf read as a real side-yard gate.
    for i, x in enumerate((0.55, 1.00, 1.45, 1.90)):
        leaf.visual(
            Cylinder(radius=0.012, length=stile_length),
            origin=Origin(xyz=(x, 0.0, z_mid)),
            material=dark_green,
            name=f"picket_{i}",
        )

    sleeve_meshes = (
        _hollow_cylinder(0.035, 0.018, 0.22, "lower_hinge_sleeve"),
        _hollow_cylinder(0.035, 0.018, 0.22, "upper_hinge_sleeve"),
    )
    for i, (z, sleeve_mesh) in enumerate(zip(hinge_levels, sleeve_meshes)):
        leaf.visual(
            sleeve_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_green,
            name=f"hinge_sleeve_{i}",
        )
        leaf.visual(
            Box((0.080, 0.030, 0.050)),
            origin=Origin(xyz=(0.060, 0.0, z)),
            material=dark_green,
            name=f"sleeve_strap_{i}",
        )

    # Caster mounting saddle welded under the free lower corner.
    caster_x = 2.30
    caster_z = 0.305
    for i, y in enumerate((-0.045, 0.045)):
        leaf.visual(
            Box((0.16, 0.030, 0.018)),
            origin=Origin(xyz=(caster_x, y, 0.345)),
            material=dark_green,
            name=f"caster_saddle_{i}",
        )
        leaf.visual(
            Box((0.040, 0.030, 0.070)),
            origin=Origin(xyz=(caster_x, y * 0.78, 0.370)),
            material=dark_green,
            name=f"caster_tab_{i}",
        )
    leaf.visual(
        _hollow_cylinder(0.032, 0.019, 0.080, "caster_collar"),
        origin=Origin(xyz=(caster_x, 0.0, caster_z)),
        material=dark_green,
        name="caster_collar",
    )

    # Swiveling caster fork under the free end.
    fork = model.part("wheel_fork")
    fork.visual(
        Cylinder(radius=0.0195, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=zinc,
        name="swivel_stem",
    )
    fork.visual(
        Box((0.105, 0.112, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=zinc,
        name="fork_crown",
    )
    for plate_name, cap_name, y in (
        ("fork_plate_0", "axle_cap_0", -0.040),
        ("fork_plate_1", "axle_cap_1", 0.040),
    ):
        fork.visual(
            Box((0.052, 0.012, 0.210)),
            origin=Origin(xyz=(0.0, y, -0.205)),
            material=zinc,
            name=plate_name,
        )
        fork.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.0, y * 1.17, -0.210), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=cap_name,
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.095, length=0.045),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="axle_sleeve",
    )

    model.articulation(
        "post_to_leaf",
        ArticulationType.REVOLUTE,
        parent=post,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "leaf_to_wheel_fork",
        ArticulationType.CONTINUOUS,
        parent=leaf,
        child=fork,
        origin=Origin(xyz=(caster_x, 0.0, caster_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    leaf = object_model.get_part("leaf")
    fork = object_model.get_part("wheel_fork")
    wheel = object_model.get_part("wheel")
    leaf_hinge = object_model.get_articulation("post_to_leaf")
    caster = object_model.get_articulation("leaf_to_wheel_fork")
    wheel_spin = object_model.get_articulation("fork_to_wheel")

    ctx.check(
        "leaf hinge is vertical revolute",
        leaf_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(leaf_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={leaf_hinge.articulation_type}, axis={leaf_hinge.axis}",
    )
    ctx.check(
        "caster and wheel spin continuously",
        caster.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(caster.axis) == (0.0, 0.0, 1.0)
        and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"caster={caster.articulation_type}/{caster.axis}, wheel={wheel_spin.articulation_type}/{wheel_spin.axis}",
    )

    ctx.allow_overlap(
        leaf,
        fork,
        elem_a="caster_collar",
        elem_b="swivel_stem",
        reason="The caster swivel stem is intentionally captured inside the welded collar bearing.",
    )
    for plate_name in ("fork_plate_0", "fork_plate_1"):
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a=plate_name,
            elem_b="axle_sleeve",
            reason="The wheel axle sleeve is intentionally captured through the fork plate bore.",
        )

    # Hinge barrels are retained on the fixed pins without modeling the sleeve as a solid plug.
    for i in (0, 1):
        ctx.expect_within(
            post,
            leaf,
            axes="xy",
            inner_elem=f"hinge_pin_{i}",
            outer_elem=f"hinge_sleeve_{i}",
            margin=0.002,
            name=f"hinge pin {i} sits inside sleeve",
        )
        ctx.expect_overlap(
            post,
            leaf,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b=f"hinge_sleeve_{i}",
            min_overlap=0.20,
            name=f"hinge pin {i} is vertically captured",
        )

    ctx.expect_within(
        fork,
        leaf,
        axes="xy",
        inner_elem="swivel_stem",
        outer_elem="caster_collar",
        margin=0.002,
        name="caster stem sits inside collar",
    )
    ctx.expect_overlap(
        fork,
        leaf,
        axes="z",
        elem_a="swivel_stem",
        elem_b="caster_collar",
        min_overlap=0.07,
        name="caster stem passes through collar",
    )
    ctx.expect_gap(
        fork,
        wheel,
        axis="y",
        positive_elem="fork_plate_1",
        negative_elem="tire",
        min_gap=0.006,
        name="wheel clears positive fork plate",
    )
    ctx.expect_gap(
        wheel,
        fork,
        axis="y",
        positive_elem="tire",
        negative_elem="fork_plate_0",
        min_gap=0.006,
        name="wheel clears negative fork plate",
    )
    ctx.expect_gap(
        fork,
        wheel,
        axis="y",
        positive_elem="fork_plate_1",
        negative_elem="axle_sleeve",
        max_penetration=0.002,
        name="axle sleeve seats in positive fork plate",
    )
    ctx.expect_gap(
        wheel,
        fork,
        axis="y",
        positive_elem="axle_sleeve",
        negative_elem="fork_plate_0",
        max_penetration=0.002,
        name="axle sleeve seats in negative fork plate",
    )

    tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    ctx.check(
        "support wheel rests at ground height",
        tire_aabb is not None and abs(tire_aabb[0][2]) < 0.004,
        details=f"tire_aabb={tire_aabb}",
    )

    closed_free_aabb = ctx.part_element_world_aabb(leaf, elem="free_stile")
    with ctx.pose({leaf_hinge: 1.2}):
        open_free_aabb = ctx.part_element_world_aabb(leaf, elem="free_stile")
    ctx.check(
        "gate leaf swings outward from the post",
        closed_free_aabb is not None
        and open_free_aabb is not None
        and open_free_aabb[0][1] > closed_free_aabb[1][1] + 1.0,
        details=f"closed={closed_free_aabb}, open={open_free_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
