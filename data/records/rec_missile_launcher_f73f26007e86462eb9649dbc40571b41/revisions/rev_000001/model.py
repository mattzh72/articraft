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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coastal_defense_launcher")

    paint = model.material("weathered_olive_drab", rgba=(0.22, 0.29, 0.20, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.017, 0.016, 1.0))
    steel = model.material("dark_gunmetal", rgba=(0.18, 0.19, 0.19, 1.0))
    concrete = model.material("salt_stained_concrete", rgba=(0.48, 0.50, 0.47, 1.0))
    warning = model.material("faded_yellow", rgba=(0.80, 0.65, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Box((2.20, 1.80, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="concrete_pad",
    )
    base.visual(
        Cylinder(radius=0.82, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=steel,
        name="armored_plinth",
    )
    base.visual(
        Cylinder(radius=0.72, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=dark,
        name="lower_race",
    )
    base.visual(
        Box((0.28, 0.06, 0.018)),
        origin=Origin(xyz=(0.78, 0.0, 0.189)),
        material=warning,
        name="yaw_index_mark",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.68, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="upper_race",
    )
    pedestal.visual(
        Cylinder(radius=0.31, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=paint,
        name="rotating_column",
    )
    pedestal.visual(
        Box((0.95, 1.08, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, 0.60)),
        material=paint,
        name="trunnion_deck",
    )
    pedestal.visual(
        Box((0.64, 0.12, 0.82)),
        origin=Origin(xyz=(0.08, 0.40, 1.04)),
        material=paint,
        name="side_cheek_pos",
    )
    pedestal.visual(
        Box((0.64, 0.12, 0.82)),
        origin=Origin(xyz=(0.08, -0.40, 1.04)),
        material=paint,
        name="side_cheek_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.0, 0.40, 1.10), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="bearing_boss_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.0, -0.40, 1.10), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="bearing_boss_neg",
    )
    pedestal.visual(
        Box((0.18, 0.92, 0.14)),
        origin=Origin(xyz=(-0.20, 0.0, 1.42)),
        material=paint,
        name="cheek_tie_bar",
    )
    pedestal.visual(
        Box((0.12, 0.10, 0.55)),
        origin=Origin(xyz=(0.37, 0.40, 0.93), rpy=(0.0, math.radians(18.0), 0.0)),
        material=steel,
        name="front_gusset_pos",
    )
    pedestal.visual(
        Box((0.12, 0.10, 0.55)),
        origin=Origin(xyz=(0.37, -0.40, 0.93), rpy=(0.0, math.radians(18.0), 0.0)),
        material=steel,
        name="front_gusset_neg",
    )

    pod = model.part("pod")
    pod_shell = (
        cq.Workplane("XY")
        .box(1.70, 0.48, 0.42)
        .edges()
        .fillet(0.035)
    )
    pod.visual(
        mesh_from_cadquery(pod_shell, "rounded_pod_shell", tolerance=0.002),
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
        material=paint,
        name="pod_shell",
    )
    pod.visual(
        Box((0.035, 0.42, 0.34)),
        origin=Origin(xyz=(1.285, 0.0, 0.0)),
        material=dark,
        name="front_plate",
    )
    pod.visual(
        Box((0.030, 0.40, 0.32)),
        origin=Origin(xyz=(-0.435, 0.0, 0.0)),
        material=steel,
        name="rear_cap",
    )
    for index, (yy, zz) in enumerate(((-0.11, -0.085), (0.11, -0.085), (-0.11, 0.085), (0.11, 0.085))):
        pod.visual(
            Cylinder(radius=0.070, length=0.035),
            origin=Origin(xyz=(1.318, yy, zz), rpy=(0.0, math.pi / 2, 0.0)),
            material=dark,
            name=f"tube_cover_{index}",
        )
    pod.visual(
        Box((0.72, 0.50, 0.045)),
        origin=Origin(xyz=(0.36, 0.0, 0.232)),
        material=steel,
        name="top_access_panel",
    )
    pod.visual(
        Box((0.055, 0.51, 0.46)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=steel,
        name="rear_band",
    )
    pod.visual(
        Box((0.055, 0.51, 0.46)),
        origin=Origin(xyz=(0.84, 0.0, 0.0)),
        material=steel,
        name="front_band",
    )
    pod.visual(
        Cylinder(radius=0.105, length=0.10),
        origin=Origin(xyz=(0.0, 0.29, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="trunnion_pin_pos",
    )
    pod.visual(
        Cylinder(radius=0.105, length=0.10),
        origin=Origin(xyz=(0.0, -0.29, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="trunnion_pin_neg",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pedestal_to_pod",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=pod,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.25, lower=-0.08, upper=0.78),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    pod = object_model.get_part("pod")
    yaw = object_model.get_articulation("base_to_pedestal")
    pitch = object_model.get_articulation("pedestal_to_pod")

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw race is seated on base",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.55,
        name="yaw race is concentric with base",
    )
    ctx.expect_gap(
        pedestal,
        pod,
        axis="y",
        positive_elem="side_cheek_pos",
        negative_elem="trunnion_pin_pos",
        max_gap=0.002,
        max_penetration=0.0,
        name="positive trunnion pin reaches cheek",
    )
    ctx.expect_gap(
        pod,
        pedestal,
        axis="y",
        positive_elem="trunnion_pin_neg",
        negative_elem="side_cheek_neg",
        max_gap=0.002,
        max_penetration=0.0,
        name="negative trunnion pin reaches cheek",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        low, high = bounds
        return tuple((float(low[i]) + float(high[i])) * 0.5 for i in range(3))

    rest_front = _aabb_center(ctx.part_element_world_aabb(pod, elem="front_plate"))
    with ctx.pose({pitch: 0.78}):
        elevated_front = _aabb_center(ctx.part_element_world_aabb(pod, elem="front_plate"))
    ctx.check(
        "pod elevates about trunnion",
        rest_front is not None
        and elevated_front is not None
        and elevated_front[2] > rest_front[2] + 0.45,
        details=f"rest={rest_front}, elevated={elevated_front}",
    )

    with ctx.pose({yaw: 0.75}):
        yawed_front = _aabb_center(ctx.part_element_world_aabb(pod, elem="front_plate"))
    ctx.check(
        "pedestal yaws the launcher pod",
        rest_front is not None
        and yawed_front is not None
        and abs(yawed_front[1]) > 0.45
        and abs(yawed_front[2] - rest_front[2]) < 0.02,
        details=f"rest={rest_front}, yawed={yawed_front}",
    )

    return ctx.report()


object_model = build_object_model()
