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
)


HUB_X = 0.160


def _add_bearing_stage(column, *, prefix: str, z: float, metal, bronze, yellow) -> None:
    """Add a split bearing bracket and its side supports to the rigid column."""
    rail_x = 0.095
    for zsign, zname in ((1.0, "upper"), (-1.0, "lower")):
        rail_z = z + zsign * 0.042
        for ysign, yname in ((1.0, "pos"), (-1.0, "neg")):
            column.visual(
                Box((0.172, 0.018, 0.022)),
                origin=Origin(xyz=(rail_x, ysign * 0.036, rail_z)),
                material=metal,
                name=f"{prefix}_{zname}_rail_{yname}",
            )
            column.visual(
                Box((0.050, 0.022, 0.026)),
                origin=Origin(xyz=(HUB_X, ysign * 0.025, rail_z)),
                material=bronze,
                name=f"{prefix}_{zname}_bushing_{yname}",
            )

        # A rear bridge ties the two side cheeks together without filling the
        # rotating journal clearance at the hub axis.
        column.visual(
            Box((0.028, 0.092, 0.022)),
            origin=Origin(xyz=(HUB_X - 0.055, 0.0, rail_z)),
            material=metal,
            name=f"{prefix}_{zname}_bridge",
        )

    # A vertical web from the column into the lower bracket makes the stage read
    # as a machined side support rather than a floating bearing block.
    column.visual(
        Box((0.112, 0.018, 0.058)),
        origin=Origin(xyz=(0.074, 0.0, z - 0.073)),
        material=metal,
        name=f"{prefix}_web",
    )

    # Paired stop lugs sit on the lower rail below the branch sweep.  They are
    # deliberately just under the arm plane, leaving visible running clearance.
    for ysign, yname in ((1.0, "pos"), (-1.0, "neg")):
        column.visual(
            Box((0.026, 0.020, 0.026)),
            origin=Origin(xyz=(HUB_X + 0.030, ysign * 0.053, z - 0.028)),
            material=yellow,
            name=f"{prefix}_stop_{yname}",
        )


def _add_journal(branch, *, prefix: str, steel, dark) -> None:
    branch.visual(
        Cylinder(radius=0.014, length=0.096),
        origin=Origin(),
        material=steel,
        name=f"{prefix}_journal",
    )
    for zsign, zname in ((1.0, "upper"), (-1.0, "lower")):
        branch.visual(
            Cylinder(radius=0.024, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, zsign * 0.023)),
            material=dark,
            name=f"{prefix}_{zname}_collar",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metrology_fixture_tree")

    cast_iron = model.material("ground_cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    black_oxide = model.material("black_oxide_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    bright_steel = model.material("turned_steel", rgba=(0.72, 0.74, 0.70, 1.0))
    bronze = model.material("oiled_bronze", rgba=(0.64, 0.46, 0.22, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    yellow = model.material("stop_yellow", rgba=(0.95, 0.68, 0.10, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.360, 0.270, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=0.046, length=1.120),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=black_oxide,
        name="center_column",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=cast_iron,
        name="foot_boss",
    )
    column.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 1.166)),
        material=bright_steel,
        name="top_cap",
    )
    # Rear key rail and front datum rib make the column look like grounded
    # industrial fixture hardware instead of a plain pipe.
    column.visual(
        Box((0.030, 0.024, 0.960)),
        origin=Origin(xyz=(-0.037, 0.0, 0.590)),
        material=cast_iron,
        name="rear_key_rail",
    )
    column.visual(
        Box((0.020, 0.018, 0.920)),
        origin=Origin(xyz=(0.034, 0.0, 0.600)),
        material=bright_steel,
        name="front_datum_rib",
    )
    for ysign, yname in ((1.0, "pos"), (-1.0, "neg")):
        for x in (-0.120, 0.120):
            column.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, ysign * 0.088, 0.033)),
                material=bright_steel,
                name=f"base_bolt_{yname}_{'front' if x > 0 else 'rear'}",
            )

    base_z = 0.270
    mid_z = 0.615
    top_z = 0.930
    _add_bearing_stage(column, prefix="base", z=base_z, metal=cast_iron, bronze=bronze, yellow=yellow)
    _add_bearing_stage(column, prefix="mid", z=mid_z, metal=cast_iron, bronze=bronze, yellow=yellow)
    _add_bearing_stage(column, prefix="top", z=top_z, metal=cast_iron, bronze=bronze, yellow=yellow)

    base_branch = model.part("base_branch")
    _add_journal(base_branch, prefix="base", steel=bright_steel, dark=black_oxide)
    base_branch.visual(
        Box((0.145, 0.022, 0.022)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=black_oxide,
        name="base_arm",
    )
    base_branch.visual(
        Cylinder(radius=0.041, length=0.018),
        origin=Origin(xyz=(0.164, 0.0, -0.004)),
        material=rubber,
        name="round_pad",
    )
    base_branch.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="pad_stem",
    )

    mid_branch = model.part("mid_branch")
    _add_journal(mid_branch, prefix="mid", steel=bright_steel, dark=black_oxide)
    mid_branch.visual(
        Box((0.272, 0.024, 0.024)),
        origin=Origin(xyz=(0.148, 0.0, 0.0)),
        material=black_oxide,
        name="mid_arm",
    )
    mid_branch.visual(
        Box((0.072, 0.078, 0.018)),
        origin=Origin(xyz=(0.306, 0.0, -0.018)),
        material=bright_steel,
        name="rect_pad",
    )
    mid_branch.visual(
        Box((0.038, 0.032, 0.028)),
        origin=Origin(xyz=(0.270, 0.0, -0.006)),
        material=black_oxide,
        name="pad_neck",
    )

    top_branch = model.part("top_branch")
    _add_journal(top_branch, prefix="top", steel=bright_steel, dark=black_oxide)
    top_branch.visual(
        Box((0.090, 0.022, 0.022)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=black_oxide,
        name="top_root_arm",
    )
    angle = math.radians(26.0)
    diag_len = 0.230
    diag_start_x = 0.096
    diag_cx = diag_start_x + 0.5 * diag_len * math.cos(angle)
    diag_cz = 0.5 * diag_len * math.sin(angle)
    diag_end_x = diag_start_x + diag_len * math.cos(angle)
    diag_end_z = diag_len * math.sin(angle)
    top_branch.visual(
        Box((diag_len, 0.022, 0.022)),
        origin=Origin(xyz=(diag_cx, 0.0, diag_cz), rpy=(0.0, -angle, 0.0)),
        material=black_oxide,
        name="angled_arm",
    )
    # Open fork made from three connected machined bars.  The fork tines are
    # the third terminal style, distinct from the round and rectangular pads.
    top_branch.visual(
        Box((0.024, 0.078, 0.022)),
        origin=Origin(xyz=(diag_end_x + 0.012, 0.0, diag_end_z)),
        material=bright_steel,
        name="fork_bridge",
    )
    for ysign, yname in ((1.0, "pos"), (-1.0, "neg")):
        top_branch.visual(
            Box((0.082, 0.016, 0.022)),
            origin=Origin(xyz=(diag_end_x + 0.060, ysign * 0.031, diag_end_z)),
            material=bright_steel,
            name=f"fork_tine_{yname}",
        )
        top_branch.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(
                xyz=(diag_end_x + 0.092, ysign * 0.031, diag_end_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=f"fork_tip_{yname}",
        )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=base_branch,
        origin=Origin(xyz=(HUB_X, 0.0, base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "mid_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=mid_branch,
        origin=Origin(xyz=(HUB_X, 0.0, mid_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=36.0, velocity=1.1, lower=-0.36, upper=0.48),
    )
    model.articulation(
        "top_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=top_branch,
        origin=Origin(xyz=(HUB_X, 0.0, top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.0, lower=-0.34, upper=0.38),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    base_branch = object_model.get_part("base_branch")
    mid_branch = object_model.get_part("mid_branch")
    top_branch = object_model.get_part("top_branch")
    base_pivot = object_model.get_articulation("base_pivot")
    mid_pivot = object_model.get_articulation("mid_pivot")
    top_pivot = object_model.get_articulation("top_pivot")

    ctx.check(
        "three supported revolute pivots",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    # Resting arm bars run between split upper/lower bearing rails, not through
    # them, and the stop lugs sit just under the arm sweep.
    for prefix, branch, arm in (
        ("base", base_branch, "base_arm"),
        ("mid", mid_branch, "mid_arm"),
        ("top", top_branch, "top_root_arm"),
    ):
        ctx.expect_gap(
            branch,
            column,
            axis="z",
            positive_elem=arm,
            negative_elem=f"{prefix}_lower_rail_pos",
            min_gap=0.015,
            name=f"{prefix} arm clears lower rail",
        )
        ctx.expect_gap(
            column,
            branch,
            axis="z",
            positive_elem=f"{prefix}_upper_rail_pos",
            negative_elem=arm,
            min_gap=0.015,
            name=f"{prefix} arm clears upper rail",
        )
        ctx.expect_gap(
            branch,
            column,
            axis="z",
            positive_elem=arm,
            negative_elem=f"{prefix}_stop_pos",
            min_gap=0.002,
            max_gap=0.008,
            name=f"{prefix} stop lug has running clearance",
        )

    # The three branches remain vertically staggered as separate fixture levels.
    ctx.expect_gap(mid_branch, base_branch, axis="z", min_gap=0.180, name="mid branch above base branch")
    ctx.expect_gap(top_branch, mid_branch, axis="z", min_gap=0.130, name="top branch above mid branch")

    # Joint limits should swing the work ends side-to-side while the hub origins
    # stay fixed in their supported bearing housings.
    for joint, branch, elem, lower, upper in (
        (base_pivot, base_branch, "round_pad", -0.42, 0.42),
        (mid_pivot, mid_branch, "rect_pad", -0.36, 0.48),
        (top_pivot, top_branch, "fork_bridge", -0.34, 0.38),
    ):
        with ctx.pose({joint: lower}):
            lower_aabb = ctx.part_element_world_aabb(branch, elem=elem)
        with ctx.pose({joint: upper}):
            upper_aabb = ctx.part_element_world_aabb(branch, elem=elem)
        lower_y = None if lower_aabb is None else 0.5 * (lower_aabb[0][1] + lower_aabb[1][1])
        upper_y = None if upper_aabb is None else 0.5 * (upper_aabb[0][1] + upper_aabb[1][1])
        ctx.check(
            f"{joint.name} sweeps terminal clear",
            lower_y is not None and upper_y is not None and upper_y > lower_y + 0.080,
            details=f"lower_y={lower_y}, upper_y={upper_y}",
        )

    return ctx.report()


object_model = build_object_model()
