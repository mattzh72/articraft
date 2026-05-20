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


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a cylinder whose local +Z axis spans p0 -> p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length rail {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    rail_mat = model.material("warm_white_powder_coat", rgba=(0.93, 0.93, 0.88, 1.0))
    bracket_mat = model.material("chunky_blue_bracket", rgba=(0.12, 0.34, 0.55, 1.0))
    joint_mat = model.material("brushed_steel_hinge", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber_mat = model.material("black_rubber_feet", rgba=(0.02, 0.02, 0.018, 1.0))

    main_r = 0.022
    hanger_r = 0.014

    central = model.part("central_frame")
    z_top = 0.92

    # Central rectangular top rack.
    _cylinder_between(central, "front_rail", (-0.36, -0.36, z_top), (0.36, -0.36, z_top), main_r, rail_mat)
    _cylinder_between(central, "rear_rail", (-0.36, 0.36, z_top), (0.36, 0.36, z_top), main_r, rail_mat)
    _cylinder_between(central, "side_rail_0", (-0.36, -0.36, z_top), (-0.36, 0.36, z_top), main_r, rail_mat)
    _cylinder_between(central, "side_rail_1", (0.36, -0.36, z_top), (0.36, 0.36, z_top), main_r, rail_mat)
    for i, x in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        _cylinder_between(
            central,
            f"center_hanging_rail_{i}",
            (x, -0.385, z_top),
            (x, 0.385, z_top),
            hanger_r,
            rail_mat,
        )

    # Chunky top bracket/spine that visually carries the three folding frames.
    central.visual(
        Box((0.20, 0.82, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=bracket_mat,
        name="top_bracket",
    )
    for i, y in enumerate((-0.26, 0.26)):
        central.visual(
            Box((0.06, 0.14, 0.08)),
            origin=Origin(xyz=(-0.39, y, 0.895)),
            material=bracket_mat,
            name=f"wing_hinge_block_0_{i}",
        )
        central.visual(
            Box((0.06, 0.14, 0.08)),
            origin=Origin(xyz=(0.39, y, 0.895)),
            material=bracket_mat,
            name=f"wing_hinge_block_1_{i}",
        )
    for i, y in enumerate((-0.32, 0.32)):
        central.visual(
            Box((0.16, 0.075, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.8275)),
            material=bracket_mat,
            name=f"lower_hinge_saddle_{i}",
        )

    def add_wing(part, sign):
        # Local frame origin is the hinge axis; the frame extends outward in +/-X.
        inner = sign * 0.040
        outer = sign * 0.68
        xs = [sign * v for v in (0.14, 0.25, 0.36, 0.47, 0.58)]
        _cylinder_between(part, "hinge_rail", (inner, -0.36, 0.0), (inner, 0.36, 0.0), 0.020, joint_mat)
        _cylinder_between(part, "outer_rail", (outer, -0.36, 0.0), (outer, 0.36, 0.0), 0.026, rail_mat)
        _cylinder_between(part, "front_rail", (inner, -0.36, 0.0), (outer, -0.36, 0.0), main_r, rail_mat)
        _cylinder_between(part, "rear_rail", (inner, 0.36, 0.0), (outer, 0.36, 0.0), main_r, rail_mat)
        for i, x in enumerate(xs):
            _cylinder_between(
                part,
                f"hanging_rail_{i}",
                (x, -0.385, 0.0),
                (x, 0.385, 0.0),
                hanger_r,
                rail_mat,
            )
        for i, y in enumerate((-0.36, 0.36)):
            part.visual(
                Sphere(radius=0.034),
                origin=Origin(xyz=(outer, y, 0.0)),
                material=rubber_mat,
                name=f"outer_corner_cap_{i}",
            )

    wing_0 = model.part("wing_0")
    add_wing(wing_0, 1.0)
    wing_1 = model.part("wing_1")
    add_wing(wing_1, -1.0)

    lower = model.part("lower_support")
    # Fold-down lower support frame: broad stance, thick rails, and a lower drying span.
    _cylinder_between(lower, "support_hinge_rail", (0.0, -0.36, 0.0), (0.0, 0.36, 0.0), 0.025, joint_mat)
    for i, y in enumerate((-0.32, 0.32)):
        _cylinder_between(lower, f"leg_0_{i}", (0.0, y, 0.0), (-0.32, y * 1.32, -0.73), main_r, rail_mat)
        _cylinder_between(lower, f"leg_1_{i}", (0.0, y, 0.0), (0.32, y * 1.32, -0.73), main_r, rail_mat)
    _cylinder_between(lower, "lower_front_rail", (-0.34, -0.42, -0.73), (0.34, -0.42, -0.73), main_r, rail_mat)
    _cylinder_between(lower, "lower_rear_rail", (-0.34, 0.42, -0.73), (0.34, 0.42, -0.73), main_r, rail_mat)
    _cylinder_between(lower, "lower_side_rail_0", (-0.34, -0.42, -0.73), (-0.34, 0.42, -0.73), main_r, rail_mat)
    _cylinder_between(lower, "lower_side_rail_1", (0.34, -0.42, -0.73), (0.34, 0.42, -0.73), main_r, rail_mat)
    for i, x in enumerate((-0.20, 0.0, 0.20)):
        _cylinder_between(
            lower,
            f"lower_hanging_rail_{i}",
            (x, -0.42, -0.708),
            (x, 0.42, -0.708),
            hanger_r,
            rail_mat,
        )
    for i, y in enumerate((-0.46, 0.46)):
        lower.visual(
            Box((0.86, 0.12, 0.05)),
            origin=Origin(xyz=(0.0, y, -0.755)),
            material=rubber_mat,
            name=f"broad_foot_{i}",
        )

    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.40, 0.0, z_top)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.10, upper=1.15),
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(-0.40, 0.0, z_top)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.10, upper=1.15),
    )
    model.articulation(
        "central_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lower = object_model.get_part("lower_support")
    wing_0_joint = object_model.get_articulation("central_to_wing_0")
    wing_1_joint = object_model.get_articulation("central_to_wing_1")
    lower_joint = object_model.get_articulation("central_to_lower_support")

    ctx.expect_gap(
        wing_0,
        central,
        axis="x",
        max_gap=0.004,
        max_penetration=0.00001,
        positive_elem="hinge_rail",
        negative_elem="wing_hinge_block_1_0",
        name="positive wing hinge rail bears on the bracket",
    )
    ctx.expect_gap(
        central,
        wing_1,
        axis="x",
        max_gap=0.004,
        max_penetration=0.00001,
        positive_elem="wing_hinge_block_0_0",
        negative_elem="hinge_rail",
        name="negative wing hinge rail bears on the bracket",
    )
    ctx.expect_gap(
        central,
        lower,
        axis="z",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="top_bracket",
        negative_elem="support_hinge_rail",
        name="lower folding hinge is tucked below top bracket",
    )
    ctx.expect_overlap(wing_0, central, axes="y", min_overlap=0.55, name="wing_0 shares the central drying span")
    ctx.expect_overlap(wing_1, central, axes="y", min_overlap=0.55, name="wing_1 shares the central drying span")

    rest_w0 = ctx.part_element_world_aabb(wing_0, elem="outer_rail")
    with ctx.pose({wing_0_joint: 0.75}):
        lifted_w0 = ctx.part_element_world_aabb(wing_0, elem="outer_rail")
    ctx.check(
        "wing_0 folds upward about its side hinge",
        rest_w0 is not None
        and lifted_w0 is not None
        and lifted_w0[1][2] > rest_w0[1][2] + 0.20,
        details=f"rest={rest_w0}, lifted={lifted_w0}",
    )

    rest_w1 = ctx.part_element_world_aabb(wing_1, elem="outer_rail")
    with ctx.pose({wing_1_joint: 0.75}):
        lifted_w1 = ctx.part_element_world_aabb(wing_1, elem="outer_rail")
    ctx.check(
        "wing_1 folds upward about its side hinge",
        rest_w1 is not None
        and lifted_w1 is not None
        and lifted_w1[1][2] > rest_w1[1][2] + 0.20,
        details=f"rest={rest_w1}, lifted={lifted_w1}",
    )

    rest_lower = ctx.part_element_world_aabb(lower, elem="broad_foot_0")
    with ctx.pose({lower_joint: 0.80}):
        folded_lower = ctx.part_element_world_aabb(lower, elem="broad_foot_0")
    ctx.check(
        "lower support frame folds upward on its own hinge",
        rest_lower is not None
        and folded_lower is not None
        and ((folded_lower[0][2] + folded_lower[1][2]) / 2.0)
        > ((rest_lower[0][2] + rest_lower[1][2]) / 2.0) + 0.15,
        details=f"rest={rest_lower}, folded={folded_lower}",
    )

    return ctx.report()


object_model = build_object_model()
