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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_three_branch_rotary_bracket")

    satin_steel = Material("satin_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    dark_bearing = Material("dark_bearing", rgba=(0.08, 0.085, 0.09, 1.0))
    blue_arm = Material("blue_powdercoat", rgba=(0.08, 0.20, 0.34, 1.0))
    black_pad = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.018, 0.180, 0.480)),
        origin=Origin(xyz=(-0.009, 0.0, 0.240)),
        material=satin_steel,
        name="backplate",
    )
    wall_mount.visual(
        Box((0.070, 0.060, 0.390)),
        origin=Origin(xyz=(0.035, 0.0, 0.240)),
        material=satin_steel,
        name="standoff_spine",
    )

    # Slightly proud screw heads make the plate read as wall-backed hardware.
    for index, (y_pos, z_pos) in enumerate(
        ((-0.055, 0.055), (0.055, 0.055), (-0.055, 0.425), (0.055, 0.425))
    ):
        wall_mount.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(
                xyz=(0.0025, y_pos, z_pos),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_bearing,
            name=f"screw_head_{index}",
        )

    station_z = (0.115, 0.240, 0.365)
    hinge_x = 0.120

    for index, z_pos in enumerate(station_z):
        # Each fixed knuckle is a forked bracket projecting from the spine.
        wall_mount.visual(
            Box((0.030, 0.078, 0.058)),
            origin=Origin(xyz=(0.078, 0.0, z_pos)),
            material=satin_steel,
            name=f"yoke_{index}_bridge",
        )
        for side_name, y_pos in (("neg", -0.025), ("pos", 0.025)):
            wall_mount.visual(
                Box((0.070, 0.012, 0.072)),
                origin=Origin(xyz=(hinge_x, y_pos, z_pos)),
                material=satin_steel,
                name=f"yoke_{index}_cheek_{side_name}",
            )
            wall_mount.visual(
                Cylinder(radius=0.016, length=0.008),
                origin=Origin(
                    xyz=(hinge_x, y_pos * 1.30, z_pos),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_bearing,
                name=f"yoke_{index}_cap_{side_name}",
            )

        branch = model.part(f"branch_{index}")
        branch.visual(
            Cylinder(radius=0.021, length=0.038),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_bearing,
            name="hub",
        )
        branch.visual(
            Box((0.130, 0.026, 0.016)),
            origin=Origin(xyz=(0.085, 0.0, 0.0)),
            material=blue_arm,
            name="arm_web",
        )
        branch.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue_arm,
            name="end_boss",
        )
        branch.visual(
            Box((0.010, 0.036, 0.026)),
            origin=Origin(xyz=(0.184, 0.0, 0.0)),
            material=black_pad,
            name="end_pad",
        )

        model.articulation(
            f"spine_to_branch_{index}",
            ArticulationType.REVOLUTE,
            parent=wall_mount,
            child=branch,
            origin=Origin(xyz=(hinge_x, 0.0, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=2.0,
                lower=-0.85,
                upper=0.85,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    branches = [object_model.get_part(f"branch_{index}") for index in range(3)]
    joints = [object_model.get_articulation(f"spine_to_branch_{index}") for index in range(3)]

    ctx.check(
        "three separate revolute branch joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "branch axes are parallel",
        all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    origins = [j.origin.xyz for j in joints]
    vertical_gaps = [origins[i + 1][2] - origins[i][2] for i in range(2)]
    ctx.check(
        "knuckles have clear vertical offsets",
        all(gap > 0.10 for gap in vertical_gaps),
        details=f"vertical_gaps={vertical_gaps}",
    )

    for index, branch in enumerate(branches):
        ctx.expect_gap(
            wall_mount,
            branch,
            axis="y",
            positive_elem=f"yoke_{index}_cheek_pos",
            negative_elem="hub",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"branch_{index} positive cheek clearance",
        )
        ctx.expect_gap(
            branch,
            wall_mount,
            axis="y",
            positive_elem="hub",
            negative_elem=f"yoke_{index}_cheek_neg",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"branch_{index} negative cheek clearance",
        )
        ctx.expect_overlap(
            branch,
            wall_mount,
            axes="xz",
            elem_a="hub",
            elem_b=f"yoke_{index}_cheek_pos",
            min_overlap=0.035,
            name=f"branch_{index} hub is captured by yoke profile",
        )

    rest_pad_aabb = ctx.part_element_world_aabb(branches[1], elem="end_pad")
    with ctx.pose({joints[1]: 0.60}):
        moved_pad_aabb = ctx.part_element_world_aabb(branches[1], elem="end_pad")

    rest_pad_z = None if rest_pad_aabb is None else (rest_pad_aabb[0][2] + rest_pad_aabb[1][2]) * 0.5
    moved_pad_z = None if moved_pad_aabb is None else (moved_pad_aabb[0][2] + moved_pad_aabb[1][2]) * 0.5
    ctx.check(
        "middle branch rotates about its supported axis",
        rest_pad_z is not None and moved_pad_z is not None and moved_pad_z < rest_pad_z - 0.070,
        details=f"rest_pad_z={rest_pad_z}, moved_pad_z={moved_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()
