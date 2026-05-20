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
    model = ArticulatedObject(name="side_frame_dual_column_carriage")

    painted_frame = model.material("painted_frame", rgba=(0.18, 0.19, 0.20, 1.0))
    black_hardware = model.material("black_hardware", rgba=(0.03, 0.035, 0.04, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.22, 0.44, 1.0))
    label_dark = model.material("label_dark", rgba=(0.01, 0.015, 0.02, 1.0))

    frame = model.part("side_frame")
    frame.visual(
        Box((0.86, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_frame,
        name="base_plate",
    )
    frame.visual(
        Box((0.86, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=painted_frame,
        name="top_crosshead",
    )
    frame.visual(
        Box((0.82, 0.035, 1.09)),
        origin=Origin(xyz=(0.0, -0.115, 0.62)),
        material=painted_frame,
        name="rear_web",
    )
    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        x = sign * 0.34
        frame.visual(
            Cylinder(radius=0.024, length=1.08),
            origin=Origin(xyz=(x, 0.0, 0.62)),
            material=ground_steel,
            name=f"guide_column_{suffix}",
        )
        frame.visual(
            Box((0.13, 0.095, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=black_hardware,
            name=f"lower_column_block_{suffix}",
        )
        frame.visual(
            Box((0.13, 0.095, 0.055)),
            origin=Origin(xyz=(x, 0.0, 1.135)),
            material=black_hardware,
            name=f"upper_column_block_{suffix}",
        )

    sleeve_shape = (
        cq.Workplane("XY")
        .circle(0.046)
        .circle(0.031)
        .extrude(0.115, both=True)
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Box((0.54, 0.045, 0.30)),
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
        material=carriage_blue,
        name="plate_panel",
    )
    faceplate.visual(
        Box((0.40, 0.010, 0.18)),
        origin=Origin(xyz=(0.0, 0.102, 0.0)),
        material=label_dark,
        name="front_recess",
    )
    for sx in (-0.18, 0.18):
        for sz in (-0.105, 0.105):
            faceplate.visual(
                Cylinder(radius=0.012, length=0.014),
                origin=Origin(xyz=(sx, 0.101, sz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=black_hardware,
                name=f"face_bolt_{'n' if sx < 0.0 else 'p'}_{'l' if sz < 0.0 else 'u'}",
            )

    sleeve_mesh = mesh_from_cadquery(sleeve_shape, "linear_bearing_sleeve")
    for sign, side in ((-1.0, "0"), (1.0, "1")):
        x = sign * 0.34
        lug_x = sign * 0.3075
        for z, level in ((-0.085, "lower"), (0.085, "upper")):
            faceplate.visual(
                sleeve_mesh,
                origin=Origin(xyz=(x, 0.0, z)),
                material=black_hardware,
                name=f"{level}_sleeve_{side}",
            )
            faceplate.visual(
                Box((0.095, 0.035, 0.040)),
                origin=Origin(xyz=(lug_x, 0.060, z)),
                material=carriage_blue,
                name=f"{level}_lug_{side}",
            )
            faceplate.visual(
                Box((0.014, 0.010, 0.052)),
                origin=Origin(xyz=(x, 0.029, z)),
                material=black_hardware,
                name=f"{level}_bearing_shoe_{side}",
            )

    model.articulation(
        "frame_to_faceplate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=800.0, velocity=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    faceplate = object_model.get_part("faceplate")
    slide = object_model.get_articulation("frame_to_faceplate")

    ctx.check(
        "single vertical guide joint",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )

    ctx.expect_gap(
        faceplate,
        frame,
        axis="z",
        positive_elem="plate_panel",
        negative_elem="base_plate",
        min_gap=0.20,
        name="faceplate clears the lower frame",
    )
    ctx.expect_gap(
        frame,
        faceplate,
        axis="z",
        positive_elem="top_crosshead",
        negative_elem="plate_panel",
        min_gap=0.50,
        name="faceplate has upward travel room",
    )
    for suffix in ("0", "1"):
        ctx.expect_within(
            frame,
            faceplate,
            axes="xy",
            inner_elem=f"guide_column_{suffix}",
            outer_elem=f"lower_sleeve_{suffix}",
            margin=0.001,
            name=f"lower sleeve {suffix} surrounds its column",
        )
        ctx.expect_overlap(
            frame,
            faceplate,
            axes="z",
            elem_a=f"guide_column_{suffix}",
            elem_b=f"lower_sleeve_{suffix}",
            min_overlap=0.08,
            name=f"lower sleeve {suffix} is engaged on the guide",
        )
        ctx.expect_contact(
            faceplate,
            frame,
            elem_a=f"lower_bearing_shoe_{suffix}",
            elem_b=f"guide_column_{suffix}",
            contact_tol=0.0001,
            name=f"bearing shoe {suffix} supports the carriage on the column",
        )

    rest_pos = ctx.part_world_position(faceplate)
    with ctx.pose({slide: 0.42}):
        ctx.expect_gap(
            frame,
            faceplate,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="plate_panel",
            min_gap=0.08,
            name="raised faceplate remains below the top crosshead",
        )
        for suffix in ("0", "1"):
            ctx.expect_within(
                frame,
                faceplate,
                axes="xy",
                inner_elem=f"guide_column_{suffix}",
                outer_elem=f"upper_sleeve_{suffix}",
                margin=0.001,
                name=f"raised upper sleeve {suffix} remains on the column axis",
            )
            ctx.expect_overlap(
                frame,
                faceplate,
                axes="z",
                elem_a=f"guide_column_{suffix}",
                elem_b=f"upper_sleeve_{suffix}",
                min_overlap=0.08,
                name=f"raised upper sleeve {suffix} remains engaged",
            )
        raised_pos = ctx.part_world_position(faceplate)

    ctx.check(
        "prismatic joint lifts the faceplate",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.40,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
