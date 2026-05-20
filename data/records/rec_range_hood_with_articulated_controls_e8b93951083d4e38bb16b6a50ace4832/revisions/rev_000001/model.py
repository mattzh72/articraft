from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood")

    shell = model.material("shell", rgba=(0.93, 0.94, 0.95, 1.0))
    fascia = model.material("fascia", rgba=(0.83, 0.85, 0.87, 1.0))
    underside = model.material("underside", rgba=(0.18, 0.19, 0.20, 1.0))
    diffuser = model.material("diffuser", rgba=(0.92, 0.93, 0.89, 1.0))
    control = model.material("control", rgba=(0.14, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.72, 0.74, 0.76, 1.0))

    body_width = 0.76
    body_depth = 0.50
    body_height = 0.085
    front_y = -body_depth / 2.0

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, body_height)),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
        material=shell,
        name="shell",
    )
    body.visual(
        Box((body_width, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, -0.238, 0.035)),
        material=fascia,
        name="control_strip",
    )
    body.visual(
        Box((body_width, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.225, 0.005)),
        material=fascia,
        name="front_lip",
    )
    body.visual(
        Box((0.60, 0.315, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.002)),
        material=underside,
        name="intake_panel",
    )
    body.visual(
        Box((0.012, 0.138, 0.006)),
        origin=Origin(xyz=(-0.296, -0.166, -0.003)),
        material=accent,
        name="guide_0",
    )
    body.visual(
        Box((0.012, 0.138, 0.006)),
        origin=Origin(xyz=(0.296, -0.166, -0.003)),
        material=accent,
        name="guide_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.232, -0.249, 0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="knob_bezel",
    )
    body.visual(
        Box((0.034, 0.004, 0.042)),
        origin=Origin(xyz=(0.176, -0.248, 0.034)),
        material=accent,
        name="switch_bezel",
    )

    light_shield = model.part("light_shield")
    light_shield.visual(
        Box((0.62, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=fascia,
        name="trim",
    )
    light_shield.visual(
        Box((0.564, 0.074, 0.006)),
        origin=Origin(xyz=(0.0, 0.038, -0.003)),
        material=diffuser,
        name="lens",
    )
    light_shield.visual(
        Box((0.548, 0.110, 0.004)),
        origin=Origin(xyz=(0.0, 0.091, -0.002)),
        material=underside,
        name="tongue",
    )
    light_shield.visual(
        Box((0.016, 0.110, 0.006)),
        origin=Origin(xyz=(-0.272, 0.081, -0.003)),
        material=accent,
        name="runner_0",
    )
    light_shield.visual(
        Box((0.016, 0.110, 0.006)),
        origin=Origin(xyz=(0.272, 0.081, -0.003)),
        material=accent,
        name="runner_1",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.018,
                body_style="skirted",
                top_diameter=0.025,
                base_diameter=0.032,
                crown_radius=0.0015,
                edge_radius=0.0008,
            ),
            "hood_knob",
        ),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control,
        name="cap",
    )
    knob.visual(
        Box((0.004, 0.002, 0.011)),
        origin=Origin(xyz=(0.0, -0.017, 0.007)),
        material=fascia,
        name="pointer",
    )

    rocker = model.part("rocker")
    rocker.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control,
        name="barrel",
    )
    rocker.visual(
        Box((0.025, 0.009, 0.034)),
        origin=Origin(xyz=(0.0, -0.0045, -0.017)),
        material=control,
        name="paddle",
    )

    model.articulation(
        "body_to_light_shield",
        ArticulationType.PRISMATIC,
        parent=body,
        child=light_shield,
        origin=Origin(xyz=(0.0, front_y - 0.007, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=0.085),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.232, front_y, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "body_to_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(0.176, front_y - 0.0012, 0.053)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-0.18, upper=0.18),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    light_shield = object_model.get_part("light_shield")
    knob = object_model.get_part("knob")
    rocker = object_model.get_part("rocker")

    shield_slide = object_model.get_articulation("body_to_light_shield")
    knob_turn = object_model.get_articulation("body_to_knob")
    rocker_hinge = object_model.get_articulation("body_to_rocker")

    ctx.expect_gap(
        body,
        light_shield,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="shell",
        name="light shield tucks directly under the hood body",
    )
    ctx.expect_overlap(
        light_shield,
        body,
        axes="x",
        min_overlap=0.58,
        name="light shield spans the intake width",
    )

    shield_rest = ctx.part_world_position(light_shield)
    with ctx.pose({shield_slide: 0.085}):
        ctx.expect_overlap(
            light_shield,
            body,
            axes="y",
            min_overlap=0.050,
            name="extended light shield keeps retained insertion on the short rail",
        )
        shield_open = ctx.part_world_position(light_shield)
    ctx.check(
        "light shield slides outward from the front lip",
        shield_rest is not None
        and shield_open is not None
        and shield_open[1] < shield_rest[1] - 0.060,
        details=f"rest={shield_rest}, open={shield_open}",
    )

    rocker_rest = _aabb_center(ctx.part_element_world_aabb(rocker, elem="paddle"))
    with ctx.pose({rocker_hinge: 0.18}):
        rocker_tipped = _aabb_center(ctx.part_element_world_aabb(rocker, elem="paddle"))
    ctx.check(
        "rocker switch tips outward on its hinge",
        rocker_rest is not None
        and rocker_tipped is not None
        and rocker_tipped[1] < rocker_rest[1] - 0.002,
        details=f"rest={rocker_rest}, tipped={rocker_tipped}",
    )

    pointer_rest = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    with ctx.pose({knob_turn: math.pi / 2.0}):
        pointer_quarter = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    ctx.check(
        "knob turns continuously around its shaft",
        pointer_rest is not None
        and pointer_quarter is not None
        and pointer_quarter[0] < pointer_rest[0] - 0.004,
        details=f"rest={pointer_rest}, quarter_turn={pointer_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
