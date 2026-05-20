from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_knob_part(
    model: ArticulatedObject,
    *,
    name: str,
    knob_mesh,
    knob_material: str,
    shaft_material: str,
) -> object:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shaft_material,
        name="shaft",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=knob_material,
        name="knob_cap",
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood")

    hood_white = model.material("hood_white", rgba=(0.94, 0.94, 0.92, 1.0))
    trim_white = model.material("trim_white", rgba=(0.90, 0.90, 0.88, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    shaft_dark = model.material("shaft_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    switch_ivory = model.material("switch_ivory", rgba=(0.89, 0.88, 0.84, 1.0))

    hood = model.part("hood")
    hood.visual(
        Box((0.760, 0.340, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=hood_white,
        name="top_panel",
    )
    hood.visual(
        Box((0.016, 0.340, 0.108)),
        origin=Origin(xyz=(-0.372, 0.0, -0.054)),
        material=hood_white,
        name="left_wall",
    )
    hood.visual(
        Box((0.016, 0.340, 0.108)),
        origin=Origin(xyz=(0.372, 0.0, -0.054)),
        material=hood_white,
        name="right_wall",
    )
    hood.visual(
        Box((0.728, 0.016, 0.108)),
        origin=Origin(xyz=(0.0, -0.162, -0.054)),
        material=hood_white,
        name="rear_wall",
    )
    hood.visual(
        Box((0.728, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, 0.163, -0.0275)),
        material=hood_white,
        name="front_wall",
    )
    hood.visual(
        Box((0.728, 0.055, 0.078)),
        origin=Origin(xyz=(0.0, 0.1425, -0.092)),
        material=trim_white,
        name="control_strip",
    )
    hood.visual(
        Box((0.022, 0.250, 0.014)),
        origin=Origin(xyz=(-0.353, -0.005, -0.101)),
        material=trim_white,
        name="left_frame",
    )
    hood.visual(
        Box((0.022, 0.250, 0.014)),
        origin=Origin(xyz=(0.353, -0.005, -0.101)),
        material=trim_white,
        name="right_frame",
    )
    hood.visual(
        Box((0.680, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, -0.143, -0.101)),
        material=trim_white,
        name="rear_frame",
    )
    hood.visual(
        Box((0.680, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.105, -0.101)),
        material=trim_white,
        name="front_frame",
    )
    hood.visual(
        Box((0.688, 0.228, 0.004)),
        origin=Origin(xyz=(0.0, -0.019, -0.099)),
        material=filter_dark,
        name="filter_panel",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.018,
            body_style="skirted",
            top_diameter=0.035,
            skirt=KnobSkirt(0.050, 0.004, flare=0.06),
            grip=KnobGrip(style="fluted", count=20, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "range_hood_knob",
    )

    left_knob = _add_knob_part(
        model,
        name="left_knob",
        knob_mesh=knob_mesh,
        knob_material="control_black",
        shaft_material="shaft_dark",
    )
    right_knob = _add_knob_part(
        model,
        name="right_knob",
        knob_mesh=knob_mesh,
        knob_material="control_black",
        shaft_material="shaft_dark",
    )

    light_switch = model.part("light_switch")
    light_switch.visual(
        Cylinder(radius=0.002, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_dark,
        name="hinge_barrel",
    )
    light_switch.visual(
        Box((0.040, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.004, -0.010)),
        material=switch_ivory,
        name="switch_face",
    )

    model.articulation(
        "hood_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=left_knob,
        origin=Origin(xyz=(-0.255, 0.170, -0.089), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )
    model.articulation(
        "hood_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=right_knob,
        origin=Origin(xyz=(0.255, 0.170, -0.089), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )
    model.articulation(
        "hood_to_light_switch",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=light_switch,
        origin=Origin(xyz=(0.0, 0.172, -0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=4.0,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    light_switch = object_model.get_part("light_switch")
    left_knob_joint = object_model.get_articulation("hood_to_left_knob")
    right_knob_joint = object_model.get_articulation("hood_to_right_knob")
    switch_joint = object_model.get_articulation("hood_to_light_switch")

    ctx.check(
        "left knob is continuous",
        left_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_knob_joint.motion_limits is not None
        and left_knob_joint.motion_limits.lower is None
        and left_knob_joint.motion_limits.upper is None,
        details=f"joint={left_knob_joint.articulation_type}, limits={left_knob_joint.motion_limits!r}",
    )
    ctx.check(
        "right knob is continuous",
        right_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_joint.motion_limits is not None
        and right_knob_joint.motion_limits.lower is None
        and right_knob_joint.motion_limits.upper is None,
        details=f"joint={right_knob_joint.articulation_type}, limits={right_knob_joint.motion_limits!r}",
    )
    ctx.check(
        "light switch is bounded hinge",
        switch_joint.articulation_type == ArticulationType.REVOLUTE
        and switch_joint.motion_limits is not None
        and switch_joint.motion_limits.lower == 0.0
        and switch_joint.motion_limits.upper is not None
        and 0.20 <= switch_joint.motion_limits.upper <= 0.35,
        details=f"joint={switch_joint.articulation_type}, limits={switch_joint.motion_limits!r}",
    )

    ctx.expect_gap(
        left_knob,
        hood,
        axis="y",
        max_gap=0.002,
        max_penetration=1e-6,
        name="left knob seats on front strip",
    )
    ctx.expect_gap(
        right_knob,
        hood,
        axis="y",
        max_gap=0.002,
        max_penetration=1e-6,
        name="right knob seats on front strip",
    )
    ctx.expect_gap(
        light_switch,
        hood,
        axis="y",
        max_gap=0.002,
        max_penetration=1e-6,
        name="light switch sits on front strip",
    )
    ctx.expect_origin_gap(
        light_switch,
        left_knob,
        axis="x",
        min_gap=0.20,
        name="left knob stays left of center switch",
    )
    ctx.expect_origin_gap(
        right_knob,
        light_switch,
        axis="x",
        min_gap=0.20,
        name="right knob stays right of center switch",
    )
    ctx.expect_origin_distance(
        light_switch,
        hood,
        axes="x",
        max_dist=0.005,
        name="light switch remains centered on hood",
    )

    rest_aabb = ctx.part_element_world_aabb(light_switch, elem="switch_face")
    with ctx.pose({switch_joint: 0.24}):
        tipped_aabb = ctx.part_element_world_aabb(light_switch, elem="switch_face")

    ctx.check(
        "light switch tips outward",
        rest_aabb is not None
        and tipped_aabb is not None
        and tipped_aabb[1][1] > rest_aabb[1][1] + 0.003,
        details=f"rest={rest_aabb!r}, tipped={tipped_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
