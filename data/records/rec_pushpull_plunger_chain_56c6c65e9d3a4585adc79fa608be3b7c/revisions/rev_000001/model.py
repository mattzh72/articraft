from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


CYL_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _x_shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
):
    """Build a revolved hollow sleeve whose length runs along local +X."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _x_lathe_mesh(profile: list[tuple[float, float]], name: str):
    """Build a solid stepped round part whose length runs along local +X."""
    geom = LatheGeometry(profile, segments=72, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_stage_plunger_actuator")

    model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("dark_oxide", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    model.material("polished_rod", rgba=(0.86, 0.88, 0.84, 1.0))
    model.material("bronze_bushing", rgba=(0.72, 0.48, 0.20, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("bolt_black", rgba=(0.02, 0.02, 0.018, 1.0))

    rear_bracket = model.part("rear_bracket")
    rear_bracket.visual(
        Box((0.72, 0.42, 0.052)),
        origin=Origin(xyz=(0.25, 0.0, -0.241)),
        material="painted_steel",
        name="ground_foot",
    )
    rear_bracket.visual(
        Box((0.060, 0.36, 0.36)),
        origin=Origin(xyz=(-0.075, 0.0, -0.061)),
        material="painted_steel",
        name="rear_wall",
    )
    for y in (-0.155, 0.155):
        rear_bracket.visual(
            Box((0.26, 0.040, 0.26)),
            origin=Origin(xyz=(0.025, y, -0.086)),
            material="painted_steel",
            name=f"side_gusset_{0 if y < 0 else 1}",
        )
    for x in (-0.02, 0.20, 0.42, 0.62):
        for y in (-0.155, 0.155):
            rear_bracket.visual(
                Cylinder(radius=0.017, length=0.016),
                origin=Origin(xyz=(x, y, -0.207)),
                material="bolt_black",
                name=f"anchor_bolt_{x}_{y}",
            )

    large_sleeve = model.part("large_sleeve")
    large_sleeve.visual(
        _x_shell_mesh(
            [
                (0.138, 0.000),
                (0.138, 0.045),
                (0.108, 0.060),
                (0.108, 0.480),
                (0.126, 0.492),
                (0.126, 0.560),
                (0.108, 0.575),
                (0.108, 0.640),
            ],
            [(0.080, 0.000), (0.080, 0.640)],
            "large_sleeve_shell",
        ),
        material="dark_oxide",
        name="guide_shell",
    )
    large_sleeve.visual(
        _x_shell_mesh(
            [(0.098, 0.004), (0.098, 0.048)],
            [(0.080, 0.004), (0.080, 0.048)],
            "large_rear_bushing",
        ),
        material="bronze_bushing",
        name="rear_bushing",
    )
    large_sleeve.visual(
        _x_shell_mesh(
            [(0.100, 0.585), (0.100, 0.635)],
            [(0.080, 0.585), (0.080, 0.635)],
            "large_front_bushing",
        ),
        material="bronze_bushing",
        name="front_bushing",
    )
    large_sleeve.visual(
        Box((0.19, 0.16, 0.110)),
        origin=Origin(xyz=(0.145, 0.0, -0.160)),
        material="dark_oxide",
        name="rear_foot",
    )
    large_sleeve.visual(
        Box((0.18, 0.15, 0.110)),
        origin=Origin(xyz=(0.530, 0.0, -0.160)),
        material="dark_oxide",
        name="front_foot",
    )
    for x in (-0.006, 0.570):
        for y, z in ((0.104, 0.058), (-0.104, 0.058), (0.104, -0.058), (-0.104, -0.058)):
            large_sleeve.visual(
                Cylinder(radius=0.012, length=0.020),
                origin=Origin(xyz=(x, y, z), rpy=CYL_X.rpy),
                material="bolt_black",
                name=f"flange_bolt_{x}_{y}_{z}",
            )

    middle_sleeve = model.part("middle_sleeve")
    middle_sleeve.visual(
        _x_shell_mesh(
            [
                (0.080, -0.520),
                (0.080, -0.465),
                (0.058, -0.450),
                (0.058, 0.150),
                (0.082, 0.170),
                (0.082, 0.250),
                (0.058, 0.270),
                (0.058, 0.330),
            ],
            [(0.043, -0.520), (0.043, 0.330)],
            "middle_sleeve_shell",
        ),
        material="brushed_steel",
        name="telescoping_shell",
    )
    middle_sleeve.visual(
        _x_shell_mesh(
            [(0.055, -0.500), (0.055, -0.440)],
            [(0.043, -0.500), (0.043, -0.440)],
            "middle_rear_bushing",
        ),
        material="bronze_bushing",
        name="rear_bushing",
    )
    middle_sleeve.visual(
        _x_shell_mesh(
            [(0.056, 0.270), (0.056, 0.326)],
            [(0.043, 0.270), (0.043, 0.326)],
            "middle_front_bushing",
        ),
        material="bronze_bushing",
        name="front_bushing",
    )
    middle_sleeve.visual(
        _x_shell_mesh(
            [(0.088, 0.190), (0.088, 0.232)],
            [(0.058, 0.190), (0.058, 0.232)],
            "middle_stop_collar",
        ),
        material="dark_oxide",
        name="stop_collar",
    )

    output_rod = model.part("output_rod")
    output_rod.visual(
        _x_lathe_mesh(
            [
                (0.0, -0.455),
                (0.043, -0.455),
                (0.043, -0.365),
                (0.030, -0.355),
                (0.030, 0.070),
                (0.049, 0.085),
                (0.049, 0.145),
                (0.030, 0.160),
                (0.030, 0.510),
                (0.022, 0.530),
                (0.022, 0.660),
                (0.030, 0.675),
                (0.030, 0.720),
                (0.0, 0.720),
            ],
            "stepped_output_rod",
        ),
        material="polished_rod",
        name="stepped_shaft",
    )
    output_rod.visual(
        _x_lathe_mesh(
            [
                (0.0, 0.705),
                (0.052, 0.705),
                (0.052, 0.755),
                (0.030, 0.785),
                (0.030, 0.845),
                (0.0, 0.845),
            ],
            "output_nose_cap",
        ),
        material="dark_oxide",
        name="nose_cap",
    )
    for x in (0.552, 0.588, 0.624):
        output_rod.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=CYL_X.rpy),
            material="dark_oxide",
            name=f"thread_crest_{x}",
        )

    model.articulation(
        "bracket_to_large_sleeve",
        ArticulationType.FIXED,
        parent=rear_bracket,
        child=large_sleeve,
        origin=Origin(),
    )
    model.articulation(
        "large_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=large_sleeve,
        child=middle_sleeve,
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2600.0, velocity=0.20, lower=0.0, upper=0.320),
        motion_properties=MotionProperties(damping=80.0, friction=40.0),
    )
    model.articulation(
        "middle_to_output_slide",
        ArticulationType.PRISMATIC,
        parent=middle_sleeve,
        child=output_rod,
        origin=Origin(xyz=(0.330, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.24, lower=0.0, upper=0.300),
        motion_properties=MotionProperties(damping=55.0, friction=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_bracket = object_model.get_part("rear_bracket")
    large_sleeve = object_model.get_part("large_sleeve")
    middle_sleeve = object_model.get_part("middle_sleeve")
    output_rod = object_model.get_part("output_rod")
    large_slide = object_model.get_articulation("large_to_middle_slide")
    output_slide = object_model.get_articulation("middle_to_output_slide")

    ctx.allow_overlap(
        large_sleeve,
        middle_sleeve,
        elem_a="guide_shell",
        elem_b="telescoping_shell",
        reason=(
            "The middle sleeve is intentionally modeled as a retained sliding member "
            "inside the large guide sleeve bore."
        ),
    )
    ctx.allow_overlap(
        middle_sleeve,
        output_rod,
        elem_a="telescoping_shell",
        elem_b="stepped_shaft",
        reason=(
            "The stepped output rod is intentionally captured as a sliding member "
            "inside the second sleeve guide bore."
        ),
    )

    ctx.expect_contact(
        rear_bracket,
        large_sleeve,
        elem_a="ground_foot",
        elem_b="rear_foot",
        contact_tol=0.002,
        name="rear guide sleeve is seated on grounded base",
    )
    ctx.expect_contact(
        rear_bracket,
        large_sleeve,
        elem_a="ground_foot",
        elem_b="front_foot",
        contact_tol=0.002,
        name="front guide sleeve foot is seated on grounded base",
    )

    ctx.expect_overlap(
        middle_sleeve,
        large_sleeve,
        axes="x",
        elem_a="telescoping_shell",
        elem_b="guide_shell",
        min_overlap=0.50,
        name="middle sleeve has deep retained insertion at rest",
    )
    ctx.expect_within(
        middle_sleeve,
        large_sleeve,
        axes="yz",
        inner_elem="rear_bushing",
        outer_elem="guide_shell",
        margin=0.002,
        name="middle rear bearing fits within large guide envelope",
    )
    ctx.expect_overlap(
        output_rod,
        middle_sleeve,
        axes="x",
        elem_a="stepped_shaft",
        elem_b="telescoping_shell",
        min_overlap=0.40,
        name="output rod has retained insertion at rest",
    )
    ctx.expect_within(
        output_rod,
        middle_sleeve,
        axes="yz",
        inner_elem="stepped_shaft",
        outer_elem="telescoping_shell",
        margin=0.002,
        name="output shaft stays within middle sleeve guide envelope",
    )

    middle_rest = ctx.part_world_position(middle_sleeve)
    output_rest = ctx.part_world_position(output_rod)
    with ctx.pose({large_slide: 0.320, output_slide: 0.300}):
        ctx.expect_overlap(
            middle_sleeve,
            large_sleeve,
            axes="x",
            elem_a="telescoping_shell",
            elem_b="guide_shell",
            min_overlap=0.18,
            name="extended middle sleeve remains captured in large guide",
        )
        ctx.expect_overlap(
            output_rod,
            middle_sleeve,
            axes="x",
            elem_a="stepped_shaft",
            elem_b="telescoping_shell",
            min_overlap=0.11,
            name="extended output rod remains captured in middle sleeve",
        )
        ctx.expect_within(
            output_rod,
            middle_sleeve,
            axes="yz",
            inner_elem="stepped_shaft",
            outer_elem="telescoping_shell",
            margin=0.002,
            name="extended output shaft remains coaxial in guide envelope",
        )
        middle_extended = ctx.part_world_position(middle_sleeve)
        output_extended = ctx.part_world_position(output_rod)

    ctx.check(
        "large-to-middle prismatic joint extends forward",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.30,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "middle-to-output prismatic joint extends forward",
        output_rest is not None
        and output_extended is not None
        and output_extended[0] > output_rest[0] + 0.60,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    return ctx.report()


object_model = build_object_model()
