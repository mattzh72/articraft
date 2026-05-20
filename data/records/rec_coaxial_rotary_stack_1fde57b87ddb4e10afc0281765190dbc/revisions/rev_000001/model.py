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
    mesh_from_cadquery,
)
import cadquery as cq


def _annular_stage(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    tab_length: float = 0.0,
    tab_width: float = 0.0,
    boss_count: int = 0,
    boss_radius: float = 0.0,
    boss_height: float = 0.0,
    boss_circle_radius: float = 0.0,
) -> cq.Workplane:
    """Return a centered annular disk with optional integral radial tab and bosses."""

    body = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )

    if tab_length > 0.0 and tab_width > 0.0:
        tab = (
            cq.Workplane("XY")
            .box(tab_length, tab_width, height)
            .translate((outer_radius + tab_length / 2.0 - 0.008, 0.0, 0.0))
        )
        body = body.union(tab)

    if boss_count > 0 and boss_radius > 0.0 and boss_height > 0.0:
        for index in range(boss_count):
            angle = 2.0 * math.pi * index / boss_count
            x = boss_circle_radius * math.cos(angle)
            y = boss_circle_radius * math.sin(angle)
            boss = (
                cq.Workplane("XY")
                .circle(boss_radius)
                .extrude(boss_height)
                .translate((x, y, height / 2.0 - 0.001))
            )
            body = body.union(boss)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_stack")

    dark_metal = Material("dark_burnished_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    base_blue = Material("blue_anodized_turntable", rgba=(0.05, 0.18, 0.42, 1.0))
    middle_orange = Material("orange_middle_ring", rgba=(0.95, 0.42, 0.08, 1.0))
    top_silver = Material("machined_top_flange", rgba=(0.72, 0.72, 0.68, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_metal,
        name="floor_plinth",
    )
    shaft.visual(
        Cylinder(radius=0.026, length=0.39),
        origin=Origin(xyz=(0.0, 0.0, 0.212)),
        material=satin_steel,
        name="vertical_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.040, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    shaft.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=dark_metal,
        name="middle_spacer_collar",
    )
    shaft.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=dark_metal,
        name="upper_spacer_collar",
    )
    shaft.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
        material=dark_metal,
        name="shaft_cap",
    )

    base_stage = model.part("base_turntable")
    base_stage.visual(
        mesh_from_cadquery(
            _annular_stage(
                0.170,
                0.045,
                0.048,
                tab_length=0.060,
                tab_width=0.038,
                boss_count=6,
                boss_radius=0.008,
                boss_height=0.007,
                boss_circle_radius=0.130,
            ),
            "base_turntable_plate",
        ),
        material=base_blue,
        name="turntable_plate",
    )
    base_stage.visual(
        mesh_from_cadquery(_annular_stage(0.050, 0.030, 0.030), "base_bearing_sleeve"),
        material=black_rubber,
        name="base_bearing_sleeve",
    )

    middle_stage = model.part("middle_ring")
    middle_stage.visual(
        mesh_from_cadquery(
            _annular_stage(
                0.122,
                0.043,
                0.040,
                tab_length=0.045,
                tab_width=0.028,
                boss_count=4,
                boss_radius=0.0065,
                boss_height=0.006,
                boss_circle_radius=0.092,
            ),
            "middle_ring_body",
        ),
        material=middle_orange,
        name="ring_body",
    )
    middle_stage.visual(
        mesh_from_cadquery(_annular_stage(0.047, 0.030, 0.046), "middle_bearing_sleeve"),
        material=black_rubber,
        name="middle_bearing_sleeve",
    )

    top_stage = model.part("top_flange")
    top_stage.visual(
        mesh_from_cadquery(
            _annular_stage(
                0.086,
                0.040,
                0.032,
                tab_length=0.032,
                tab_width=0.022,
                boss_count=6,
                boss_radius=0.0045,
                boss_height=0.005,
                boss_circle_radius=0.065,
            ),
            "top_flange_body",
        ),
        material=top_silver,
        name="flange_body",
    )
    top_stage.visual(
        mesh_from_cadquery(_annular_stage(0.044, 0.030, 0.038), "top_bearing_sleeve"),
        material=black_rubber,
        name="top_bearing_sleeve",
    )
    top_stage.visual(
        Box((0.018, 0.010, 0.009)),
        origin=Origin(xyz=(0.074, 0.0, 0.020)),
        material=dark_metal,
        name="flange_index_mark",
    )

    model.articulation(
        "shaft_to_base",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=base_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "shaft_to_middle",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-2.094, upper=2.094),
    )
    model.articulation(
        "shaft_to_top",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-2.094, upper=2.094),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("shaft")
    base = object_model.get_part("base_turntable")
    middle = object_model.get_part("middle_ring")
    top = object_model.get_part("top_flange")
    base_joint = object_model.get_articulation("shaft_to_base")
    middle_joint = object_model.get_articulation("shaft_to_middle")
    top_joint = object_model.get_articulation("shaft_to_top")

    ctx.check(
        "three coaxial rotary stages",
        len(object_model.articulations) == 3
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in object_model.articulations),
        details="Expected base, middle, and top joints all rotating about the vertical shaft axis.",
    )
    ctx.check(
        "base stage is continuous",
        base_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"base joint type={base_joint.articulation_type}",
    )
    ctx.check(
        "upper stages limited about 120 degrees each way",
        middle_joint.motion_limits is not None
        and top_joint.motion_limits is not None
        and abs(middle_joint.motion_limits.lower + 2.094) < 0.01
        and abs(middle_joint.motion_limits.upper - 2.094) < 0.01
        and abs(top_joint.motion_limits.lower + 2.094) < 0.01
        and abs(top_joint.motion_limits.upper - 2.094) < 0.01,
        details="Middle and top stage limits should be approximately +/-120 degrees.",
    )
    ctx.expect_overlap(base, shaft, axes="xy", elem_a="base_bearing_sleeve", elem_b="vertical_shaft")
    ctx.expect_overlap(middle, shaft, axes="xy", elem_a="middle_bearing_sleeve", elem_b="vertical_shaft")
    ctx.expect_overlap(top, shaft, axes="xy", elem_a="top_bearing_sleeve", elem_b="vertical_shaft")
    ctx.expect_gap(middle, base, axis="z", min_gap=0.010, name="middle stage clears base stage vertically")
    ctx.expect_gap(top, middle, axis="z", min_gap=0.010, name="top stage clears middle stage vertically")

    with ctx.pose({middle_joint: 2.094, top_joint: -2.094}):
        ctx.expect_gap(middle, base, axis="z", min_gap=0.010, name="limited rotations preserve lower clearance")
        ctx.expect_gap(top, middle, axis="z", min_gap=0.010, name="limited rotations preserve upper clearance")

    return ctx.report()


object_model = build_object_model()
