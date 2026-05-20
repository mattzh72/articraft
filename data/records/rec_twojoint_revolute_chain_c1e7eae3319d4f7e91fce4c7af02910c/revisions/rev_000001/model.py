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
    model = ArticulatedObject(name="ladder_frame_revolute_chain")

    steel = model.material("dark_blued_steel", rgba=(0.18, 0.23, 0.28, 1.0))
    edge_steel = model.material("machined_edge_steel", rgba=(0.55, 0.60, 0.63, 1.0))
    brass = model.material("oil_bronze_bushings", rgba=(0.78, 0.55, 0.22, 1.0))
    black = model.material("black_stop_pads", rgba=(0.02, 0.02, 0.018, 1.0))

    axis_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    root = model.part("root_bracket")
    # Grounded first rigid link: a bolted base and a fixed outer clevis/side-plate run.
    root.visual(
        Box((0.42, 0.20, 0.025)),
        origin=Origin(xyz=(-0.21, 0.0, -0.145)),
        material=edge_steel,
        name="ground_foot",
    )
    for y in (-0.070, 0.070):
        root.visual(
            Box((0.32, 0.016, 0.120)),
            origin=Origin(xyz=(-0.18, y, -0.073)),
            material=steel,
            name=f"upright_{'n' if y < 0 else 'p'}",
        )
        root.visual(
            Box((0.36, 0.016, 0.060)),
            origin=Origin(xyz=(-0.18, y, 0.0)),
            material=steel,
            name=f"side_plate_{'n' if y < 0 else 'p'}",
        )
        root.visual(
            Cylinder(radius=0.030, length=0.042),
            origin=Origin(xyz=(0.0, math.copysign(0.062, y), 0.0), rpy=axis_y.rpy),
            material=brass,
            name=f"root_bushing_{'n' if y < 0 else 'p'}",
        )
    root.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(-0.335, 0.0, 0.0), rpy=axis_y.rpy),
        material=edge_steel,
        name="rear_tube",
    )
    for x in (-0.32, -0.10):
        root.visual(
            Box((0.040, 0.205, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.127)),
            material=edge_steel,
            name=f"base_rib_{abs(int(x * 100)):02d}",
        )

    middle = model.part("middle_link")
    middle_len = 0.44
    for y in (-0.034, 0.034):
        middle.visual(
            Box((middle_len, 0.014, 0.048)),
            origin=Origin(xyz=(middle_len / 2.0, y, 0.0)),
            material=steel,
            name=f"side_plate_{'n' if y < 0 else 'p'}",
        )
        middle.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=axis_y.rpy),
            material=brass,
            name=f"prox_bushing_{'n' if y < 0 else 'p'}",
        )
        middle.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(middle_len, y, 0.0), rpy=axis_y.rpy),
            material=brass,
            name=f"dist_bushing_{'n' if y < 0 else 'p'}",
        )
    for x, label in ((0.0, "prox_tube"), (middle_len, "dist_tube")):
        middle.visual(
            Cylinder(radius=0.016, length=0.082),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=axis_y.rpy),
            material=edge_steel,
            name=label,
        )
    for x in (0.155, 0.285):
        middle.visual(
            Box((0.030, 0.086, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=edge_steel,
            name=f"ladder_rung_{int(x * 1000)}",
        )

    end = model.part("end_link")
    end_len = 0.34
    for y in (-0.064, 0.064):
        end.visual(
            Box((0.320, 0.014, 0.044)),
            origin=Origin(xyz=(0.160, y, 0.0)),
            material=steel,
            name=f"side_plate_{'n' if y < 0 else 'p'}",
        )
        end.visual(
            Cylinder(radius=0.026, length=0.033),
            origin=Origin(xyz=(0.0, math.copysign(0.0575, y), 0.0), rpy=axis_y.rpy),
            material=brass,
            name=f"pivot_bushing_{'n' if y < 0 else 'p'}",
        )
    end.visual(
        Box((0.070, 0.150, 0.055)),
        origin=Origin(xyz=(end_len, 0.0, 0.0)),
        material=steel,
        name="tip_tab",
    )
    end.visual(
        Cylinder(radius=0.028, length=0.150),
        origin=Origin(xyz=(end_len + 0.035, 0.0, 0.0), rpy=axis_y.rpy),
        material=edge_steel,
        name="rounded_tip",
    )
    end.visual(
        Cylinder(radius=0.011, length=0.154),
        origin=Origin(xyz=(end_len + 0.035, 0.0, 0.0), rpy=axis_y.rpy),
        material=black,
        name="tab_hole_shadow",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=root,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "middle_pivot",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=end,
        origin=Origin(xyz=(middle_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_bracket")
    middle = object_model.get_part("middle_link")
    end = object_model.get_part("end_link")
    root_pivot = object_model.get_articulation("root_pivot")
    middle_pivot = object_model.get_articulation("middle_pivot")

    ctx.check(
        "two serial revolute joints",
        len(object_model.articulations) == 2
        and root_pivot.articulation_type == ArticulationType.REVOLUTE
        and middle_pivot.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "parallel pivot axes",
        tuple(root_pivot.axis) == (0.0, -1.0, 0.0)
        and tuple(middle_pivot.axis) == (0.0, -1.0, 0.0),
    )
    ctx.expect_origin_distance(
        root,
        middle,
        axes="xy",
        max_dist=0.001,
        name="middle link pivots at bracket nose",
    )
    ctx.expect_origin_distance(
        middle,
        end,
        axes="x",
        min_dist=0.439,
        max_dist=0.441,
        name="end link is carried at middle distal pivot",
    )

    rest_tip = ctx.part_element_world_aabb(end, elem="rounded_tip")
    with ctx.pose({root_pivot: 0.65, middle_pivot: 0.55}):
        raised_tip = ctx.part_element_world_aabb(end, elem="rounded_tip")
    ctx.check(
        "chain sweeps in the xz plane",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[1][2] > rest_tip[1][2] + 0.15,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
