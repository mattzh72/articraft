from __future__ import annotations

import math

import cadquery as cq

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


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _platen_body() -> cq.Workplane:
    """One connected sliding carriage body with two vertical guide bores."""
    column_x = 0.18
    column_y = 0.055
    sleeve_outer = 0.057
    bore_radius = 0.026
    height = 0.240

    body = _box_solid((0.460, 0.052, height), (0.0, 0.132, 0.0))

    for x in (-column_x, column_x):
        sleeve = (
            cq.Workplane("XY")
            .circle(sleeve_outer)
            .extrude(height / 2.0, both=True)
            .translate((x, column_y, 0.0))
        )
        bridge = _box_solid((0.096, 0.078, 0.150), (x, 0.096, 0.0))
        side_boss = _box_solid((0.118, 0.034, 0.205), (x, 0.075, 0.0))
        body = body.union(sleeve).union(bridge).union(side_boss)

    # Re-cut the guide bores through the whole union so the visible body reads
    # as a captured carriage sliding on two fixed columns, not as solid blocks.
    for x in (-column_x, column_x):
        bore = (
            cq.Workplane("XY")
            .circle(bore_radius)
            .extrude((height + 0.040) / 2.0, both=True)
            .translate((x, column_y, 0.0))
        )
        body = body.cut(bore)

    # Shallow front reliefs make the platen face read as a machined plate.
    front_relief = _box_solid((0.345, 0.012, 0.030), (0.0, 0.161, 0.065))
    lower_relief = _box_solid((0.345, 0.012, 0.030), (0.0, 0.161, -0.065))
    body = body.cut(front_relief).cut(lower_relief)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_dual_column_carriage")

    frame_paint = model.material("warm_grey_powdercoat", rgba=(0.46, 0.47, 0.46, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.05, 0.055, 0.060, 1.0))
    polished_steel = model.material("polished_guide_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    platen_paint = model.material("safety_orange_platen", rgba=(0.96, 0.38, 0.10, 1.0))
    bolt_black = model.material("black_fasteners", rgba=(0.015, 0.014, 0.013, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((0.620, 0.022, 1.120)),
        origin=Origin(xyz=(0.0, -0.040, 0.560)),
        material=frame_paint,
        name="wall_plate",
    )
    back_frame.visual(
        Box((0.560, 0.070, 0.075)),
        origin=Origin(xyz=(0.0, -0.002, 1.040)),
        material=dark_steel,
        name="top_crossbar",
    )
    back_frame.visual(
        Box((0.560, 0.070, 0.075)),
        origin=Origin(xyz=(0.0, -0.002, 0.080)),
        material=dark_steel,
        name="bottom_crossbar",
    )
    for x, name in ((-0.285, "side_stile_0"), (0.285, "side_stile_1")):
        back_frame.visual(
            Box((0.050, 0.060, 1.000)),
            origin=Origin(xyz=(x, -0.005, 0.560)),
            material=dark_steel,
            name=name,
        )

    column_x = 0.180
    column_y = 0.055
    for x, idx, guide_name in (
        (-column_x, 0, "guide_column_0"),
        (column_x, 1, "guide_column_1"),
    ):
        back_frame.visual(
            Box((0.122, 0.125, 0.095)),
            origin=Origin(xyz=(x, 0.048, 0.125)),
            material=dark_steel,
            name=f"lower_column_block_{idx}",
        )
        back_frame.visual(
            Box((0.122, 0.125, 0.095)),
            origin=Origin(xyz=(x, 0.048, 0.995)),
            material=dark_steel,
            name=f"upper_column_block_{idx}",
        )
        back_frame.visual(
            Cylinder(radius=0.026, length=0.880),
            origin=Origin(xyz=(x, column_y, 0.560)),
            material=polished_steel,
            name=guide_name,
        )
        for z, label in ((0.188, "lower_stop"), (0.918, "upper_stop")):
            back_frame.visual(
                Cylinder(radius=0.038, length=0.028),
                origin=Origin(xyz=(x, column_y, z)),
                material=dark_steel,
                name=f"{label}_{idx}",
            )

    # Wall-plate screw heads and crossbar bolts are rigid details on the frame.
    for x in (-0.245, 0.245):
        for z in (0.205, 0.915):
            back_frame.visual(
                Cylinder(radius=0.018, length=0.009),
                origin=Origin(xyz=(x, -0.025, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"wall_screw_{x:+.3f}_{z:.3f}",
            )
    for x in (-column_x, column_x):
        for z in (0.125, 0.995):
            back_frame.visual(
                Cylinder(radius=0.015, length=0.010),
                origin=Origin(xyz=(x - 0.038, 0.113, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"block_bolt_a_{x:+.3f}_{z:.3f}",
            )
            back_frame.visual(
                Cylinder(radius=0.015, length=0.010),
                origin=Origin(xyz=(x + 0.038, 0.113, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"block_bolt_b_{x:+.3f}_{z:.3f}",
            )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(_platen_body(), "platen_body", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(),
        material=platen_paint,
        name="platen_body",
    )
    # Low-profile fasteners on the moving face make the sliding body read as one
    # assembled platen rather than two independent bushings.
    for x in (-0.145, 0.145):
        for z in (-0.075, 0.075):
            platen.visual(
                Cylinder(radius=0.016, length=0.010),
                origin=Origin(xyz=(x, 0.160, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"face_bolt_{x:+.3f}_{z:+.3f}",
            )

    model.articulation(
        "platen_slide",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.22, lower=0.0, upper=0.400),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("back_frame")
    platen = object_model.get_part("platen")
    slide = object_model.get_articulation("platen_slide")

    ctx.allow_overlap(
        frame,
        platen,
        elem_a="guide_column_0",
        elem_b="platen_body",
        reason="The fixed guide column is intentionally captured inside the platen bushing bore as the bearing contact proxy.",
    )
    ctx.allow_overlap(
        frame,
        platen,
        elem_a="guide_column_1",
        elem_b="platen_body",
        reason="The second guide column is intentionally captured inside the matching platen bushing bore as the bearing contact proxy.",
    )

    ctx.expect_within(
        frame,
        platen,
        axes="xy",
        inner_elem="guide_column_0",
        outer_elem="platen_body",
        margin=0.001,
        name="first guide runs through platen bushing",
    )
    ctx.expect_within(
        frame,
        platen,
        axes="xy",
        inner_elem="guide_column_1",
        outer_elem="platen_body",
        margin=0.001,
        name="second guide runs through platen bushing",
    )
    ctx.expect_overlap(
        platen,
        frame,
        axes="x",
        elem_a="platen_body",
        elem_b="guide_column_0",
        min_overlap=0.030,
        name="platen wraps first guide in x projection",
    )
    ctx.expect_overlap(
        platen,
        frame,
        axes="x",
        elem_a="platen_body",
        elem_b="guide_column_1",
        min_overlap=0.030,
        name="platen wraps second guide in x projection",
    )
    ctx.expect_overlap(
        platen,
        frame,
        axes="z",
        elem_a="platen_body",
        elem_b="guide_column_0",
        min_overlap=0.150,
        name="low platen remains engaged on guides",
    )

    rest_pos = ctx.part_world_position(platen)
    with ctx.pose({slide: 0.400}):
        ctx.expect_overlap(
            platen,
            frame,
            axes="z",
            elem_a="platen_body",
            elem_b="guide_column_1",
            min_overlap=0.150,
            name="raised platen remains engaged on guides",
        )
        raised_pos = ctx.part_world_position(platen)

    ctx.check(
        "platen slide is vertical",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.35,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
