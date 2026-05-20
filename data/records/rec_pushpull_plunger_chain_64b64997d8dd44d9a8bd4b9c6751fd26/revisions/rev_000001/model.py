from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_Z = 0.20


def _circle_profile(radius: float, segments: int = 64):
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Annular guide sleeve mesh, authored on local Z before X-axis placement."""
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            length,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain")

    frame_mat = Material("powder_coated_frame", rgba=(0.10, 0.14, 0.18, 1.0))
    rail_mat = Material("machined_steel_rail", rgba=(0.42, 0.44, 0.45, 1.0))
    sleeve_mat = Material("black_oxide_sleeve", rgba=(0.015, 0.017, 0.018, 1.0))
    rod_mat = Material("polished_plunger_rod", rgba=(0.72, 0.75, 0.76, 1.0))
    bronze_mat = Material("bronze_bushing", rgba=(0.78, 0.50, 0.20, 1.0))
    handle_mat = Material("red_push_pad", rgba=(0.72, 0.05, 0.035, 1.0))
    pin_mat = Material("dark_hardened_pin", rgba=(0.06, 0.065, 0.07, 1.0))

    rod_radius = 0.0165
    sleeve_outer = 0.045
    sleeve_inner = 0.026
    sleeve_len = 0.24
    sleeve_rpy = (0.0, math.pi / 2.0, 0.0)

    frame = model.part("frame")
    frame.visual(
        Box((1.42, 0.30, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 0.0175)),
        material=frame_mat,
        name="base_plate",
    )
    frame.visual(
        Box((1.34, 0.070, 0.040)),
        origin=Origin(xyz=(0.06, 0.0, 0.055)),
        material=rail_mat,
        name="center_rail",
    )
    for y, name in ((-0.115, "side_rail_0"), (0.115, "side_rail_1")):
        frame.visual(
            Box((1.28, 0.030, 0.030)),
            origin=Origin(xyz=(0.07, y, 0.050)),
            material=rail_mat,
            name=name,
        )

    frame.visual(
        _tube_mesh(sleeve_outer, sleeve_inner, sleeve_len, "fixed_guide_sleeve_mesh"),
        origin=Origin(xyz=(-0.45, 0.0, AXIS_Z), rpy=sleeve_rpy),
        material=sleeve_mat,
        name="guide_sleeve",
    )
    for x, name in ((-0.565, "rear_bushing"), (-0.335, "front_bushing")):
        frame.visual(
            _tube_mesh(0.047, sleeve_inner, 0.024, f"fixed_{name}_mesh"),
            origin=Origin(xyz=(x, 0.0, AXIS_Z), rpy=sleeve_rpy),
            material=bronze_mat,
            name=name,
        )
    frame.visual(
        Box((0.23, 0.095, 0.083)),
        origin=Origin(xyz=(-0.45, 0.0, 0.115)),
        material=frame_mat,
        name="sleeve_pedestal",
    )
    frame.visual(
        Box((0.205, 0.125, 0.014)),
        origin=Origin(xyz=(-0.45, 0.0, AXIS_Z + sleeve_outer + 0.004)),
        material=frame_mat,
        name="top_clamp",
    )
    for i, (x, y) in enumerate(
        (
            (-0.525, -0.050),
            (-0.525, 0.050),
            (-0.375, -0.050),
            (-0.375, 0.050),
        )
    ):
        frame.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x, y, AXIS_Z + sleeve_outer + 0.014)),
            material=pin_mat,
            name=f"clamp_bolt_{i}",
        )

    def add_carried_sleeve_plunger(part, *, mesh_prefix: str, include_handle: bool = False) -> None:
        part.visual(
            Cylinder(radius=rod_radius, length=0.45),
            origin=Origin(xyz=(0.165, 0.0, 0.0), rpy=sleeve_rpy),
            material=rod_mat,
            name="rod",
        )
        part.visual(
            Cylinder(radius=0.026, length=0.045),
            origin=Origin(xyz=(0.4025, 0.0, 0.0), rpy=sleeve_rpy),
            material=pin_mat,
            name="front_collar",
        )
        part.visual(
            _tube_mesh(sleeve_outer, sleeve_inner, sleeve_len, f"{mesh_prefix}_guide_sleeve_mesh"),
            origin=Origin(xyz=(0.56, 0.0, 0.0), rpy=sleeve_rpy),
            material=sleeve_mat,
            name="guide_sleeve",
        )
        for x, name in ((0.445, "rear_bushing"), (0.675, "front_bushing")):
            part.visual(
                _tube_mesh(0.047, sleeve_inner, 0.024, f"{mesh_prefix}_{name}_mesh"),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=sleeve_rpy),
                material=bronze_mat,
                name=name,
            )
        part.visual(
            Box((0.130, 0.030, 0.020)),
            origin=Origin(xyz=(0.455, 0.0, -0.050)),
            material=pin_mat,
            name="under_bridge",
        )
        part.visual(
            Box((0.040, 0.034, 0.037)),
            origin=Origin(xyz=(0.392, 0.0, -0.035)),
            material=pin_mat,
            name="collar_drop",
        )
        part.visual(
            Box((0.112, 0.044, 0.065)),
            origin=Origin(xyz=(0.56, 0.0, -0.075)),
            material=frame_mat,
            name="sleeve_drop",
        )
        part.visual(
            Box((0.175, 0.115, 0.018)),
            origin=Origin(xyz=(0.56, 0.0, -0.110)),
            material=rail_mat,
            name="slide_shoe",
        )
        if include_handle:
            part.visual(
                Cylinder(radius=0.035, length=0.045),
                origin=Origin(xyz=(-0.0825, 0.0, 0.0), rpy=sleeve_rpy),
                material=handle_mat,
                name="push_pad",
            )

    plunger_0 = model.part("plunger_0")
    add_carried_sleeve_plunger(plunger_0, mesh_prefix="plunger_0", include_handle=True)

    plunger_1 = model.part("plunger_1")
    add_carried_sleeve_plunger(plunger_1, mesh_prefix="plunger_1")

    plunger_2 = model.part("plunger_2")
    plunger_2.visual(
        Cylinder(radius=rod_radius, length=0.62),
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=sleeve_rpy),
        material=rod_mat,
        name="rod",
    )
    plunger_2.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.578, 0.0, 0.0), rpy=sleeve_rpy),
        material=pin_mat,
        name="end_collar",
    )
    plunger_2.visual(
        Box((0.064, 0.083, 0.058)),
        origin=Origin(xyz=(0.615, 0.0, 0.0)),
        material=pin_mat,
        name="clevis_bridge",
    )
    for y, name in ((-0.042, "clevis_ear_0"), (0.042, "clevis_ear_1")):
        plunger_2.visual(
            Box((0.120, 0.018, 0.070)),
            origin=Origin(xyz=(0.685, y, 0.0)),
            material=pin_mat,
            name=name,
        )
    plunger_2.visual(
        Cylinder(radius=0.009, length=0.112),
        origin=Origin(xyz=(0.700, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rod_mat,
        name="clevis_pin",
    )

    model.articulation(
        "frame_to_plunger_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=plunger_0,
        origin=Origin(xyz=(-0.56, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.22, lower=0.0, upper=0.16),
    )
    model.articulation(
        "plunger_0_to_1",
        ArticulationType.PRISMATIC,
        parent=plunger_0,
        child=plunger_1,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=620.0, velocity=0.20, lower=0.0, upper=0.14),
    )
    model.articulation(
        "plunger_1_to_2",
        ArticulationType.PRISMATIC,
        parent=plunger_1,
        child=plunger_2,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    plunger_0 = object_model.get_part("plunger_0")
    plunger_1 = object_model.get_part("plunger_1")
    plunger_2 = object_model.get_part("plunger_2")
    slide_0 = object_model.get_articulation("frame_to_plunger_0")
    slide_1 = object_model.get_articulation("plunger_0_to_1")
    slide_2 = object_model.get_articulation("plunger_1_to_2")

    ctx.allow_overlap(
        frame,
        plunger_0,
        elem_a="guide_sleeve",
        elem_b="rod",
        reason="The first plunger rod is intentionally captured inside the fixed guide sleeve/bushing fit.",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.allow_overlap(
            frame,
            plunger_0,
            elem_a=bushing_name,
            elem_b="rod",
            reason="The fixed bronze bushing is intentionally represented as a captured sliding fit around the rod.",
        )
    ctx.allow_overlap(
        plunger_0,
        plunger_1,
        elem_a="guide_sleeve",
        elem_b="rod",
        reason="The second plunger rod intentionally slides through the guide sleeve carried by the first stage.",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.allow_overlap(
            plunger_0,
            plunger_1,
            elem_a=bushing_name,
            elem_b="rod",
            reason="The carried bronze bushing is intentionally represented as a captured sliding fit around the next rod.",
        )
    ctx.allow_overlap(
        plunger_1,
        plunger_2,
        elem_a="guide_sleeve",
        elem_b="rod",
        reason="The output rod intentionally slides through the guide sleeve carried by the second stage.",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.allow_overlap(
            plunger_1,
            plunger_2,
            elem_a=bushing_name,
            elem_b="rod",
            reason="The final bronze bushing is intentionally represented as a captured sliding fit around the output rod.",
        )

    ctx.check(
        "three serial prismatic slides",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in (slide_0, slide_1, slide_2)),
        details="The actuation chain should be three axial prismatic stages.",
    )

    ctx.expect_within(
        plunger_0,
        frame,
        axes="yz",
        inner_elem="rod",
        outer_elem="guide_sleeve",
        name="first rod centered in fixed guide",
    )
    ctx.expect_overlap(
        plunger_0,
        frame,
        axes="x",
        elem_a="rod",
        elem_b="guide_sleeve",
        min_overlap=0.20,
        name="first rod retained in fixed guide",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.expect_overlap(
            plunger_0,
            frame,
            axes="x",
            elem_a="rod",
            elem_b=bushing_name,
            min_overlap=0.016,
            name=f"first rod passes through {bushing_name}",
        )
    ctx.expect_within(
        plunger_1,
        plunger_0,
        axes="yz",
        inner_elem="rod",
        outer_elem="guide_sleeve",
        name="second rod centered in carried guide",
    )
    ctx.expect_overlap(
        plunger_1,
        plunger_0,
        axes="x",
        elem_a="rod",
        elem_b="guide_sleeve",
        min_overlap=0.20,
        name="second rod retained in carried guide",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.expect_overlap(
            plunger_1,
            plunger_0,
            axes="x",
            elem_a="rod",
            elem_b=bushing_name,
            min_overlap=0.016,
            name=f"second rod passes through {bushing_name}",
        )
    ctx.expect_within(
        plunger_2,
        plunger_1,
        axes="yz",
        inner_elem="rod",
        outer_elem="guide_sleeve",
        name="output rod centered in final guide",
    )
    ctx.expect_overlap(
        plunger_2,
        plunger_1,
        axes="x",
        elem_a="rod",
        elem_b="guide_sleeve",
        min_overlap=0.20,
        name="output rod retained in final guide",
    )
    for bushing_name in ("rear_bushing", "front_bushing"):
        ctx.expect_overlap(
            plunger_2,
            plunger_1,
            axes="x",
            elem_a="rod",
            elem_b=bushing_name,
            min_overlap=0.016,
            name=f"output rod passes through {bushing_name}",
        )
    ctx.expect_gap(
        plunger_0,
        frame,
        axis="z",
        positive_elem="slide_shoe",
        negative_elem="center_rail",
        min_gap=0.004,
        max_gap=0.008,
        name="moving shoe rides just above rail",
    )

    rest_output = ctx.part_world_position(plunger_2)
    with ctx.pose({slide_0: 0.16, slide_1: 0.14, slide_2: 0.12}):
        ctx.expect_overlap(
            plunger_0,
            frame,
            axes="x",
            elem_a="rod",
            elem_b="guide_sleeve",
            min_overlap=0.12,
            name="extended first rod remains inserted",
        )
        ctx.expect_overlap(
            plunger_1,
            plunger_0,
            axes="x",
            elem_a="rod",
            elem_b="guide_sleeve",
            min_overlap=0.09,
            name="extended second rod remains inserted",
        )
        ctx.expect_overlap(
            plunger_2,
            plunger_1,
            axes="x",
            elem_a="rod",
            elem_b="guide_sleeve",
            min_overlap=0.09,
            name="extended output rod remains inserted",
        )
        extended_output = ctx.part_world_position(plunger_2)

    ctx.check(
        "output clevis advances along axis",
        rest_output is not None
        and extended_output is not None
        and extended_output[0] > rest_output[0] + 0.38,
        details=f"rest={rest_output}, extended={extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
