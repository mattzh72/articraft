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


def _radial_origin(radius: float, z: float, yaw: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(yaw), radius * math.sin(yaw), z),
        rpy=(0.0, 0.0, yaw),
    )


def _local_radial_origin(x: float, y: float, z: float, yaw: float) -> Origin:
    return Origin(
        xyz=(x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z),
        rpy=(0.0, 0.0, yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_three_branch_fixture")

    dark_steel = Material("dark_burnished_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    warm_tab = Material("warm_anodized_tabs", rgba=(0.92, 0.60, 0.22, 1.0))
    output_blue = Material("blue_output_faces", rgba=(0.08, 0.23, 0.70, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.130, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_steel,
        name="squat_base",
    )
    pedestal.visual(
        Cylinder(radius=0.118, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=rubber,
        name="base_foot_pad",
    )
    pedestal.visual(
        Cylinder(radius=0.016, length=0.362),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=satin_steel,
        name="central_post",
    )

    levels = (0.120, 0.220, 0.320)
    yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    hinge_radius = 0.056

    for i, (level, yaw) in enumerate(zip(levels, yaws)):
        pedestal.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=_radial_origin(hinge_radius, level - 0.009, yaw),
            material=satin_steel,
            name=f"support_{i}_plate",
        )
        pedestal.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=_radial_origin(hinge_radius, level + 0.009, yaw),
            material=dark_steel,
            name=f"support_{i}_keeper",
        )
        pedestal.visual(
            Box((0.050, 0.006, 0.036)),
            origin=_local_radial_origin(hinge_radius, 0.0254, level, yaw),
            material=dark_steel,
            name=f"support_{i}_cheek_0",
        )
        pedestal.visual(
            Box((0.050, 0.006, 0.036)),
            origin=_local_radial_origin(hinge_radius, -0.0254, level, yaw),
            material=dark_steel,
            name=f"support_{i}_cheek_1",
        )
        pedestal.visual(
            Box((0.094, 0.046, 0.007)),
            origin=_radial_origin(0.037, level - 0.015, yaw),
            material=dark_steel,
            name=f"support_{i}_shelf",
        )
        pedestal.visual(
            Box((0.082, 0.008, 0.033)),
            origin=_radial_origin(0.034, level - 0.034, yaw),
            material=dark_steel,
            name=f"support_{i}_web",
        )

    for i, (level, yaw) in enumerate(zip(levels, yaws)):
        tab = model.part(f"tab_{i}")
        tab.visual(
            Cylinder(radius=0.022, length=0.012),
            material=warm_tab,
            name="hub",
        )
        tab.visual(
            Box((0.108, 0.024, 0.010)),
            origin=Origin(xyz=(0.076, 0.0, 0.0)),
            material=warm_tab,
            name="tab_arm",
        )
        tab.visual(
            Box((0.020, 0.046, 0.032)),
            origin=Origin(xyz=(0.139, 0.0, 0.0)),
            material=satin_steel,
            name="output_block",
        )
        tab.visual(
            Box((0.004, 0.040, 0.026)),
            origin=Origin(xyz=(0.1505, 0.0, 0.0)),
            material=output_blue,
            name="output_face",
        )
        model.articulation(
            f"tab_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=tab,
            origin=_radial_origin(hinge_radius, level, yaw),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.85, upper=0.85),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    tabs = [object_model.get_part(f"tab_{i}") for i in range(3)]
    pivots = [object_model.get_articulation(f"tab_{i}_pivot") for i in range(3)]

    ctx.expect_origin_gap(
        tabs[1],
        tabs[0],
        axis="z",
        min_gap=0.095,
        max_gap=0.105,
        name="middle tab is stacked above lower tab",
    )
    ctx.expect_origin_gap(
        tabs[2],
        tabs[1],
        axis="z",
        min_gap=0.095,
        max_gap=0.105,
        name="upper tab is stacked above middle tab",
    )

    for i, tab in enumerate(tabs):
        ctx.expect_gap(
            tab,
            pedestal,
            axis="z",
            positive_elem="hub",
            negative_elem=f"support_{i}_plate",
            max_gap=0.002,
            max_penetration=0.0001,
            name=f"tab_{i} hub rides just above its support plate",
        )
        ctx.expect_overlap(
            tab,
            pedestal,
            axes="xy",
            elem_a="hub",
            elem_b=f"support_{i}_plate",
            min_overlap=0.040,
            name=f"tab_{i} hub is centered on its post support",
        )

    def element_center(part, elem: str):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[j] + hi[j]) * 0.5 for j in range(3))

    rest_face = element_center(tabs[0], "output_face")
    with ctx.pose({pivots[0]: 0.75}):
        swept_face = element_center(tabs[0], "output_face")

    ctx.check(
        "lower output face sweeps horizontally about the post",
        rest_face is not None
        and swept_face is not None
        and swept_face[1] > rest_face[1] + 0.070
        and swept_face[0] < rest_face[0] - 0.025,
        details=f"rest={rest_face}, swept={swept_face}",
    )

    return ctx.report()


object_model = build_object_model()
