from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


def _wedge_weight_geometry() -> MeshGeometry:
    """A small trapezoidal-prism sliding weight, broad at the lower end."""
    bottom_half = 0.032
    top_half = 0.018
    half_height = 0.030
    half_depth = 0.010

    geom = MeshGeometry()
    verts = [
        (-bottom_half, -half_depth, -half_height),
        (bottom_half, -half_depth, -half_height),
        (top_half, -half_depth, half_height),
        (-top_half, -half_depth, half_height),
        (-bottom_half, half_depth, -half_height),
        (bottom_half, half_depth, -half_height),
        (top_half, half_depth, half_height),
        (-top_half, half_depth, half_height),
    ]
    for v in verts:
        geom.add_vertex(*v)

    for face in (
        (0, 1, 2),
        (0, 2, 3),
        (5, 4, 7),
        (5, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_style_metronome")

    polished_wood = model.material("polished_wood", rgba=(0.34, 0.17, 0.075, 1.0))
    dark_wood = model.material("dark_endgrain", rgba=(0.18, 0.08, 0.035, 1.0))
    brass = model.material("warm_brass", rgba=(0.88, 0.63, 0.25, 1.0))
    black = model.material("blackened_steel", rgba=(0.025, 0.023, 0.020, 1.0))

    # Object frame: +Z up, +X across the front, +Y toward the back.
    body_w = 0.220
    body_d = 0.130
    body_h = 0.460
    wall_t = 0.012
    foot_h = 0.035
    body_bottom = foot_h
    body_top = body_bottom + body_h
    slot_w = 0.160
    slot_bottom = body_bottom + 0.035
    slot_top = body_top - 0.065
    front_y = -body_d / 2.0 + wall_t / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((0.290, 0.180, foot_h)),
        origin=Origin(xyz=(0.0, 0.0, foot_h / 2.0)),
        material=dark_wood,
        name="low_foot",
    )
    housing.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall_t / 2.0, 0.0, body_bottom + body_h / 2.0)),
        material=polished_wood,
        name="left_side_panel",
    )
    housing.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall_t / 2.0, 0.0, body_bottom + body_h / 2.0)),
        material=polished_wood,
        name="right_side_panel",
    )
    housing.visual(
        Box((body_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall_t / 2.0, body_bottom + body_h / 2.0)),
        material=polished_wood,
        name="back_panel",
    )
    housing.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall_t / 2.0)),
        material=dark_wood,
        name="bottom_cap",
    )
    housing.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall_t / 2.0)),
        material=dark_wood,
        name="top_cap",
    )

    side_rail_w = (body_w - slot_w) / 2.0
    housing.visual(
        Box((side_rail_w, wall_t, body_h)),
        origin=Origin(
            xyz=(
                -slot_w / 2.0 - side_rail_w / 2.0,
                front_y,
                body_bottom + body_h / 2.0,
            )
        ),
        material=polished_wood,
        name="front_rail_0",
    )
    housing.visual(
        Box((side_rail_w, wall_t, body_h)),
        origin=Origin(
            xyz=(
                slot_w / 2.0 + side_rail_w / 2.0,
                front_y,
                body_bottom + body_h / 2.0,
            )
        ),
        material=polished_wood,
        name="front_rail_1",
    )
    housing.visual(
        Box((slot_w, wall_t, slot_bottom - body_bottom)),
        origin=Origin(xyz=(0.0, front_y, (body_bottom + slot_bottom) / 2.0)),
        material=polished_wood,
        name="front_lower_bridge",
    )
    housing.visual(
        Box((slot_w, wall_t, body_top - slot_top)),
        origin=Origin(xyz=(0.0, front_y, (slot_top + body_top) / 2.0)),
        material=polished_wood,
        name="front_upper_bridge",
    )

    # Small brass tempo ticks seated on the right front rail, emphasizing scale.
    for i, z in enumerate([0.165, 0.205, 0.245, 0.285, 0.325, 0.365]):
        housing.visual(
            Box((0.018 if i % 2 == 0 else 0.012, 0.002, 0.003)),
            origin=Origin(xyz=(slot_w / 2.0 + 0.013, -body_d / 2.0 - 0.001, z)),
            material=brass,
            name=f"tempo_tick_{i}",
        )

    pivot_z = body_top - 0.070
    pivot_y = -body_d / 2.0 - 0.003
    housing.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(
            xyz=(0.0, -body_d / 2.0 + 0.003, pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="pivot_bearing",
    )

    pendulum = model.part("pendulum")
    rod_len = 0.340
    pendulum.visual(
        Cylinder(radius=0.0030, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, -rod_len / 2.0 - 0.010)),
        material=black,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-0.16, upper=0.16),
    )

    weight = model.part("weight")
    weight.visual(
        mesh_from_geometry(_wedge_weight_geometry(), "wedge_weight"),
        material=brass,
        name="wedge_weight",
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.20, lower=0.0, upper=0.160),
    )

    winding_key = model.part("winding_key")
    stem_len = 0.012
    winding_key.visual(
        Cylinder(radius=0.006, length=stem_len),
        origin=Origin(xyz=(-stem_len / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="key_stem",
    )
    winding_key.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="round_key",
    )
    winding_key.visual(
        Box((0.003, 0.006, 0.040)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
        material=dark_wood,
        name="key_grip_ridge",
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(-body_w / 2.0, 0.0, body_bottom + body_h / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    winding_key = object_model.get_part("winding_key")
    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_slide = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    ctx.allow_overlap(
        pendulum,
        weight,
        elem_a="rod",
        elem_b="wedge_weight",
        reason="The brass sliding weight is intentionally modeled as a captured sleeve around the slim pendulum rod.",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        elem_a="wedge_weight",
        elem_b="rod",
        min_overlap=0.050,
        name="sliding weight stays threaded on the rod",
    )
    ctx.expect_origin_distance(
        weight,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="weight slider is centered on pendulum rod axis",
    )

    ctx.expect_contact(
        pendulum,
        housing,
        elem_a="pivot_hub",
        elem_b="pivot_bearing",
        contact_tol=0.0015,
        name="pendulum hub seats on upper pivot bearing",
    )
    ctx.expect_contact(
        winding_key,
        housing,
        elem_a="key_stem",
        elem_b="left_side_panel",
        contact_tol=0.0015,
        name="winding key stem is mounted on left side face",
    )

    rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    if rod_aabb is not None:
        (rod_min, rod_max) = rod_aabb
        ctx.check(
            "resting rod lies inside front vertical slot",
            rod_min[0] > -0.080
            and rod_max[0] < 0.080
            and rod_min[2] > 0.065
            and rod_max[2] < 0.435,
            details=f"rod_aabb={rod_aabb}",
        )

    with ctx.pose({pendulum_joint: 0.16, weight_slide: 0.16, key_joint: math.pi / 2.0}):
        moved_weight = ctx.part_world_position(weight)
        moved_key = ctx.part_world_position(winding_key)
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            elem_a="wedge_weight",
            elem_b="rod",
            min_overlap=0.045,
            name="lowered weight remains captured on rod",
        )
        ctx.check(
            "weight slides downward along pendulum",
            moved_weight is not None and moved_weight[2] < 0.250,
            details=f"moved_weight={moved_weight}",
        )
        ctx.check(
            "side winding key supports continuous rotation pose",
            moved_key is not None,
            details=f"moved_key={moved_key}",
        )

    return ctx.report()


object_model = build_object_model()
