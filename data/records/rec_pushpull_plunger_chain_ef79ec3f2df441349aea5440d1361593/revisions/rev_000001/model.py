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


def _annular_tube(
    *,
    start_x: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
) -> cq.Workplane:
    """Return an open-ended tube along +X with a real through-bore."""
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    cutter = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length + 0.020)
        .translate((-0.010, 0.0, 0.0))
    )
    return outer.cut(cutter).translate((start_x, 0.0, 0.0))


def _sleeve_mesh(
    name: str,
    *,
    start_x: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
    collars: tuple[tuple[float, float, float], ...] = (),
):
    shell = _annular_tube(
        start_x=start_x,
        length=length,
        outer_radius=outer_radius,
        inner_radius=inner_radius,
    )
    for collar_start, collar_length, collar_radius in collars:
        shell = shell.union(
            _annular_tube(
                start_x=collar_start,
                length=collar_length,
                outer_radius=collar_radius,
                inner_radius=inner_radius,
            )
        )
    return mesh_from_cadquery(shell, name, tolerance=0.001, angular_tolerance=0.08)


def _rear_bracket_mesh():
    base = (
        cq.Workplane("XY")
        .box(0.300, 0.44, 0.040)
        .translate((-0.150, 0.0, -0.170))
    )
    upright = (
        cq.Workplane("XY")
        .box(0.055, 0.360, 0.360)
        .translate((-0.0275, 0.0, 0.000))
    )
    bearing_ring = _annular_tube(
        start_x=-0.055,
        length=0.055,
        outer_radius=0.120,
        inner_radius=0.067,
    )

    bracket = base.union(upright).union(bearing_ring)

    # Clearance through the upright so the rear of the guide sleeve reads as a
    # true bracketed support instead of a solid wall.
    bore_cutter = (
        cq.Workplane("YZ")
        .circle(0.067)
        .extrude(0.130)
        .translate((-0.090, 0.0, 0.0))
    )
    bracket = bracket.cut(bore_cutter)

    # Two welded side gussets rising from the foot to the bearing ring.
    web_profile = [(-0.055, -0.150), (-0.010, -0.150), (-0.055, 0.070)]
    for y_center in (-0.115, 0.115):
        web = (
            cq.Workplane("XZ")
            .polyline(web_profile)
            .close()
            .extrude(0.030)
            .translate((0.0, y_center - 0.015, 0.0))
        )
        bracket = bracket.union(web)

    return mesh_from_cadquery(bracket, "rear_bracket", tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_stage_plunger_actuator")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.21, 0.24, 1.0))
    guide_blue = model.material("guide_blue", rgba=(0.08, 0.22, 0.36, 1.0))
    sleeve_blue = model.material("sleeve_blue", rgba=(0.10, 0.32, 0.48, 1.0))
    polished_rod = model.material("polished_rod", rgba=(0.72, 0.75, 0.76, 1.0))
    dark_seal = model.material("dark_seal", rgba=(0.02, 0.025, 0.03, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.40, 0.42, 0.42, 1.0))

    rear_bracket = model.part("rear_bracket")
    rear_bracket.visual(_rear_bracket_mesh(), material=painted_steel, name="bracket_frame")
    for bolt_index, (x, y) in enumerate(
        ((-0.235, -0.160), (-0.235, 0.160), (-0.065, -0.160), (-0.065, 0.160))
    ):
        rear_bracket.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, -0.143)),
            material=bolt_steel,
            name=f"mount_bolt_{bolt_index}",
        )

    large_sleeve = model.part("large_sleeve")
    large_sleeve.visual(
        _sleeve_mesh(
            "large_guide_sleeve",
            start_x=0.000,
            length=0.720,
            outer_radius=0.090,
            inner_radius=0.060,
            collars=((0.000, 0.060, 0.110), (0.660, 0.060, 0.105)),
        ),
        material=guide_blue,
        name="guide_shell",
    )
    large_sleeve.visual(
        _sleeve_mesh(
            "large_front_seal",
            start_x=0.718,
            length=0.014,
            outer_radius=0.092,
            inner_radius=0.060,
        ),
        material=dark_seal,
        name="front_seal",
    )

    second_sleeve = model.part("second_sleeve")
    second_sleeve.visual(
        _sleeve_mesh(
            "second_sliding_sleeve",
            start_x=-0.600,
            length=0.900,
            outer_radius=0.060,
            inner_radius=0.022,
            collars=((0.240, 0.060, 0.064),),
        ),
        material=sleeve_blue,
        name="sleeve_shell",
    )
    second_sleeve.visual(
        _sleeve_mesh(
            "second_front_seal",
            start_x=0.298,
            length=0.014,
            outer_radius=0.050,
            inner_radius=0.022,
        ),
        material=dark_seal,
        name="rod_seal",
    )

    output_rod = model.part("output_rod")
    output_rod.visual(
        Cylinder(radius=0.022, length=0.700),
        origin=Origin(xyz=(-0.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="rear_shank",
    )
    output_rod.visual(
        Cylinder(radius=0.033, length=0.060),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="shoulder_stop",
    )
    output_rod.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="front_nose",
    )
    output_rod.visual(
        Box((0.030, 0.060, 0.060)),
        origin=Origin(xyz=(0.595, 0.0, 0.0)),
        material=painted_steel,
        name="end_pad",
    )

    model.articulation(
        "bracket_to_large_sleeve",
        ArticulationType.FIXED,
        parent=rear_bracket,
        child=large_sleeve,
        origin=Origin(),
    )

    model.articulation(
        "large_to_second_sleeve",
        ArticulationType.PRISMATIC,
        parent=large_sleeve,
        child=second_sleeve,
        origin=Origin(xyz=(0.720, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.32, lower=0.0, upper=0.380),
    )

    model.articulation(
        "second_sleeve_to_output",
        ArticulationType.PRISMATIC,
        parent=second_sleeve,
        child=output_rod,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.40, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    large = object_model.get_part("large_sleeve")
    second = object_model.get_part("second_sleeve")
    output = object_model.get_part("output_rod")
    large_slide = object_model.get_articulation("large_to_second_sleeve")
    output_slide = object_model.get_articulation("second_sleeve_to_output")

    ctx.allow_overlap(
        large,
        second,
        elem_a="guide_shell",
        elem_b="sleeve_shell",
        reason=(
            "The second sleeve is modeled as a zero-clearance captured slider "
            "inside the large guide sleeve so the telescoping guide reads "
            "physically supported instead of floating."
        ),
    )
    ctx.allow_overlap(
        output,
        second,
        elem_a="rear_shank",
        elem_b="sleeve_shell",
        reason=(
            "The output shank is a zero-clearance sliding fit through the "
            "second sleeve bore, representing a guided industrial push rod."
        ),
    )
    ctx.allow_overlap(
        large,
        second,
        elem_a="front_seal",
        elem_b="sleeve_shell",
        reason=(
            "The dark front seal is intentionally compressed against the "
            "sliding second sleeve at the guide-sleeve mouth."
        ),
    )
    ctx.allow_overlap(
        output,
        second,
        elem_a="rear_shank",
        elem_b="rod_seal",
        reason=(
            "The small rod seal intentionally presses on the output shank as "
            "it exits the second sleeve."
        ),
    )

    ctx.expect_origin_distance(
        second,
        large,
        axes="yz",
        max_dist=0.0005,
        name="large and second sleeve centerlines coincide",
    )
    ctx.expect_within(
        second,
        large,
        axes="yz",
        elem_a="sleeve_shell",
        elem_b="guide_shell",
        margin=0.001,
        name="second sleeve body is radially captured by guide sleeve",
    )
    ctx.expect_origin_distance(
        output,
        second,
        axes="yz",
        max_dist=0.0005,
        name="output rod shares sleeve centerline",
    )
    ctx.expect_within(
        output,
        second,
        axes="yz",
        elem_a="rear_shank",
        elem_b="sleeve_shell",
        margin=0.001,
        name="output shank is radially captured by second sleeve",
    )
    ctx.expect_overlap(
        second,
        large,
        axes="x",
        elem_a="sleeve_shell",
        elem_b="guide_shell",
        min_overlap=0.50,
        name="second sleeve retained in large guide at rest",
    )
    ctx.expect_overlap(
        second,
        large,
        axes="x",
        elem_a="sleeve_shell",
        elem_b="front_seal",
        min_overlap=0.010,
        name="large guide seal bears on second sleeve",
    )
    ctx.expect_overlap(
        output,
        second,
        axes="x",
        elem_a="rear_shank",
        elem_b="sleeve_shell",
        min_overlap=0.55,
        name="output rear shank retained in second sleeve at rest",
    )
    ctx.expect_overlap(
        output,
        second,
        axes="x",
        elem_a="rear_shank",
        elem_b="rod_seal",
        min_overlap=0.010,
        name="second sleeve seal bears on output shank",
    )

    rest_output = ctx.part_world_position(output)
    with ctx.pose({large_slide: 0.380, output_slide: 0.320}):
        ctx.expect_origin_distance(
            second,
            large,
            axes="yz",
            max_dist=0.0005,
            name="extended second sleeve remains coaxial",
        )
        ctx.expect_origin_distance(
            output,
            second,
            axes="yz",
            max_dist=0.0005,
            name="extended output rod remains coaxial",
        )
        ctx.expect_overlap(
            second,
            large,
            axes="x",
            elem_a="sleeve_shell",
            elem_b="guide_shell",
            min_overlap=0.20,
            name="extended second sleeve remains inserted in guide",
        )
        ctx.expect_overlap(
            output,
            second,
            axes="x",
            elem_a="rear_shank",
            elem_b="sleeve_shell",
            min_overlap=0.25,
            name="extended output rod remains inserted in second sleeve",
        )
        extended_output = ctx.part_world_position(output)

    ctx.check(
        "positive prismatic motion extends the actuator",
        rest_output is not None
        and extended_output is not None
        and extended_output[0] > rest_output[0] + 0.60,
        details=f"rest={rest_output}, extended={extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
