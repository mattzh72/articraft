from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_gate")

    wood = model.material("weathered_cedar", rgba=(0.55, 0.36, 0.18, 1.0))
    darker_wood = model.material("end_grain_cedar", rgba=(0.38, 0.23, 0.11, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    worn_steel = model.material("worn_galvanized_steel", rgba=(0.42, 0.43, 0.40, 1.0))
    gravel = model.material("gravel_and_soil", rgba=(0.28, 0.24, 0.19, 1.0))
    leaf = model.material("garden_green", rgba=(0.13, 0.33, 0.12, 1.0))

    fence = model.part("fence")

    # Ground strip and fixed fence run.  The posts touch this strip so the
    # static assembly reads as one planted exterior fixture.
    fence.visual(
        Box((3.8, 0.54, 0.04)),
        origin=Origin(xyz=(0.45, 0.0, -0.02)),
        material=gravel,
        name="gravel_strip",
    )
    fence.visual(
        Box((0.16, 0.16, 1.65)),
        origin=Origin(xyz=(-0.10, 0.0, 0.825)),
        material=wood,
        name="hinge_post",
    )
    fence.visual(
        Box((0.18, 0.18, 0.045)),
        origin=Origin(xyz=(-0.10, 0.0, 1.6725)),
        material=darker_wood,
        name="hinge_post_cap",
    )
    fence.visual(
        Box((0.16, 0.16, 1.65)),
        origin=Origin(xyz=(1.15, 0.0, 0.825)),
        material=wood,
        name="latch_post",
    )
    fence.visual(
        Box((0.18, 0.18, 0.045)),
        origin=Origin(xyz=(1.15, 0.0, 1.6725)),
        material=darker_wood,
        name="latch_post_cap",
    )

    for prefix, center, length in (
        ("left", -0.78, 1.20),
        ("right", 1.765, 1.07),
    ):
        for z, rail_name in ((0.49, "lower_rail"), (1.06, "upper_rail")):
            fence.visual(
                Box((length, 0.055, 0.095)),
                origin=Origin(xyz=(center, 0.0, z)),
                material=wood,
                name=f"{prefix}_{rail_name}",
            )

    for index, x in enumerate([-1.30, -1.14, -0.98, -0.82, -0.66, -0.50, -0.34]):
        fence.visual(
            Box((0.058, 0.045, 1.12)),
            origin=Origin(xyz=(x, 0.0, 0.56)),
            material=wood,
            name=f"left_picket_{index}",
        )
        fence.visual(
            Box((0.070, 0.047, 0.055)),
            origin=Origin(xyz=(x, 0.0, 1.147)),
            material=darker_wood,
            name=f"left_picket_cap_{index}",
        )

    for index, x in enumerate([1.32, 1.48, 1.64, 1.80, 1.96, 2.12, 2.28]):
        fence.visual(
            Box((0.058, 0.045, 1.12)),
            origin=Origin(xyz=(x, 0.0, 0.56)),
            material=wood,
            name=f"right_picket_{index}",
        )
        fence.visual(
            Box((0.070, 0.047, 0.055)),
            origin=Origin(xyz=(x, 0.0, 1.147)),
            material=darker_wood,
            name=f"right_picket_cap_{index}",
        )

    # Small low shrubs are tied into the ground strip to give garden character.
    for index, x in enumerate([-1.18, -1.02, 1.92, 2.10]):
        fence.visual(
            Cylinder(radius=0.014, length=0.16),
            origin=Origin(xyz=(x, -0.20, 0.08)),
            material=leaf,
            name=f"shrub_stem_{index}",
        )
        fence.visual(
            Sphere(radius=0.075),
            origin=Origin(xyz=(x, -0.20, 0.17)),
            material=leaf,
            name=f"shrub_clump_{index}",
        )

    # Fixed half of three exposed strap hinges: alternating knuckles and short
    # leaves keep the hinge readable without locking the moving gate solid.
    hinge_centers = (0.35, 0.75, 1.15)
    for i, zc in enumerate(hinge_centers):
        for j, dz in enumerate((-0.055, 0.055)):
            z = zc + dz
            fence.visual(
                Cylinder(radius=0.018, length=0.045),
                origin=Origin(xyz=(0.0, -0.086, z)),
                material=black_iron,
                name=f"fixed_knuckle_{i}_{j}",
            )
            fence.visual(
                Box((0.080, 0.014, 0.045)),
                origin=Origin(xyz=(-0.055, -0.085, z)),
                material=black_iron,
                name=f"fixed_hinge_leaf_{i}_{j}",
            )
            for k, sx in enumerate((-0.075, -0.036)):
                fence.visual(
                    Cylinder(radius=0.0065, length=0.006),
                    origin=Origin(xyz=(sx, -0.095, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=worn_steel,
                    name=f"fixed_hinge_screw_{i}_{j}_{k}",
                )

    # Keeper hardware on the latch post.  The latch tongue passes between the
    # jaws in the closed pose, with visible clearance rather than collision.
    fence.visual(
        Box((0.012, 0.014, 0.22)),
        origin=Origin(xyz=(1.064, -0.086, 0.84)),
        material=black_iron,
        name="strike_plate",
    )
    for name, z in (("lower_keeper", 0.805), ("upper_keeper", 0.875)):
        fence.visual(
            Box((0.065, 0.026, 0.018)),
            origin=Origin(xyz=(1.0425, -0.103, z)),
            material=black_iron,
            name=name,
        )
    for z in (0.76, 0.92):
        fence.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(1.058, -0.096, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"strike_screw_{z:.2f}",
        )

    gate = model.part("gate")

    gate_y = 0.036
    # Timber frame: hinge and latch stiles, rails, pickets, and a diagonal brace.
    gate.visual(
        Box((0.09, 0.055, 1.24)),
        origin=Origin(xyz=(0.085, gate_y, 0.72)),
        material=wood,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.09, 0.055, 1.24)),
        origin=Origin(xyz=(0.985, gate_y, 0.72)),
        material=wood,
        name="latch_stile",
    )
    gate.visual(
        Box((0.99, 0.055, 0.09)),
        origin=Origin(xyz=(0.535, gate_y, 0.17)),
        material=wood,
        name="bottom_rail",
    )
    gate.visual(
        Box((0.99, 0.055, 0.075)),
        origin=Origin(xyz=(0.535, gate_y, 0.72)),
        material=wood,
        name="middle_rail",
    )
    gate.visual(
        Box((0.99, 0.055, 0.09)),
        origin=Origin(xyz=(0.535, gate_y, 1.28)),
        material=wood,
        name="top_rail",
    )
    for index, x in enumerate([0.24, 0.36, 0.48, 0.60, 0.72, 0.84]):
        gate.visual(
            Box((0.055, 0.036, 1.08)),
            origin=Origin(xyz=(x, gate_y + 0.004, 0.68)),
            material=wood,
            name=f"gate_picket_{index}",
        )
        gate.visual(
            Box((0.064, 0.038, 0.050)),
            origin=Origin(xyz=(x, gate_y + 0.004, 1.245)),
            material=darker_wood,
            name=f"gate_picket_cap_{index}",
        )

    brace_angle = math.atan2(0.94, 0.78)
    gate.visual(
        Box((math.hypot(0.78, 0.94), 0.045, 0.070)),
        origin=Origin(xyz=(0.54, 0.070, 0.71), rpy=(0.0, -brace_angle, 0.0)),
        material=darker_wood,
        name="diagonal_brace",
    )

    # Moving half of the hinge: center knuckles with strap leaves bolted into
    # the gate stile.
    for i, zc in enumerate(hinge_centers):
        gate.visual(
            Cylinder(radius=0.018, length=0.065),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=black_iron,
            name=f"swing_knuckle_{i}",
        )
        gate.visual(
            Box((0.205, 0.014, 0.055)),
            origin=Origin(xyz=(0.1175, 0.002, zc)),
            material=black_iron,
            name=f"swing_hinge_leaf_{i}",
        )
        for k, sx in enumerate((0.060, 0.145)):
            gate.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(sx, -0.008, zc), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=worn_steel,
                name=f"swing_hinge_screw_{i}_{k}",
            )

    # Fixed backplate for the handle spindle.
    gate.visual(
        Box((0.070, 0.011, 0.115)),
        origin=Origin(xyz=(0.93, 0.003, 0.84)),
        material=black_iron,
        name="latch_backplate",
    )
    gate.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.93, -0.002, 0.84), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="latch_bushing",
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="handle_spindle",
    )
    latch_handle.visual(
        Box((0.28, 0.012, 0.035)),
        origin=Origin(xyz=(-0.14, -0.021, 0.0)),
        material=black_iron,
        name="handle_lever",
    )
    latch_handle.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.305, -0.021, 0.0)),
        material=black_iron,
        name="handle_grip",
    )
    latch_handle.visual(
        Box((0.135, 0.012, 0.026)),
        origin=Origin(xyz=(0.0675, -0.021, 0.0)),
        material=black_iron,
        name="latch_tongue",
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=fence,
        child=gate,
        origin=Origin(xyz=(0.0, -0.086, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=gate,
        child=latch_handle,
        origin=Origin(xyz=(0.93, -0.0025, 0.84)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.18, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fence = object_model.get_part("fence")
    gate = object_model.get_part("gate")
    latch_handle = object_model.get_part("latch_handle")
    gate_hinge = object_model.get_articulation("gate_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    # The closed gate hangs above the ground and remains centered in the opening.
    ctx.expect_gap(
        gate,
        fence,
        axis="z",
        min_gap=0.08,
        positive_elem="bottom_rail",
        negative_elem="gravel_strip",
        name="gate clears the garden path",
    )
    ctx.expect_gap(
        fence,
        gate,
        axis="x",
        min_gap=0.03,
        positive_elem="latch_post",
        negative_elem="latch_stile",
        name="closed gate clears latch post",
    )
    ctx.expect_gap(
        fence,
        latch_handle,
        axis="x",
        min_gap=0.002,
        positive_elem="latch_post",
        negative_elem="latch_tongue",
        name="latch tongue stops before the post",
    )

    closed_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    closed_grip_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_grip")
    with ctx.pose({gate_hinge: 1.25}):
        open_gate_aabb = ctx.part_element_world_aabb(gate, elem="latch_stile")
    with ctx.pose({handle_pivot: 0.55}):
        raised_grip_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_grip")

    ctx.check(
        "gate swings outward",
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and open_gate_aabb[0][1] > closed_gate_aabb[0][1] + 0.45,
        details=f"closed={closed_gate_aabb}, open={open_gate_aabb}",
    )
    ctx.check(
        "latch handle lifts",
        closed_grip_aabb is not None
        and raised_grip_aabb is not None
        and raised_grip_aabb[1][2] > closed_grip_aabb[1][2] + 0.10,
        details=f"closed={closed_grip_aabb}, raised={raised_grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
