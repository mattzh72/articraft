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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supported_tilt_head")

    dark_cast = Material("dark_cast_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    machined = Material("brushed_machined_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    blue = Material("blue_powder_coat", rgba=(0.05, 0.18, 0.36, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    pivot_z = 0.420
    tower_y = 0.285
    tower_width = 0.075

    cap_shape = (
        cq.Workplane("XY")
        .circle(0.082)
        .circle(0.047)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.008))
    )
    cap_mesh = mesh_from_cadquery(cap_shape, "trunnion_cap")

    frame = model.part("frame")
    frame.visual(
        Box((0.76, 0.70, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="base_plate",
    )

    for side, y in (("pos", tower_y), ("neg", -tower_y)):
        sign = 1.0 if y > 0.0 else -1.0
        inner_y = sign * (tower_y - tower_width / 2.0)
        outer_y = sign * (tower_y + tower_width / 2.0)

        frame.visual(
            Box((0.18, tower_width, 0.50)),
            origin=Origin(xyz=(0.0, y, 0.285)),
            material=dark_cast,
            name=f"tower_{side}",
        )
        frame.visual(
            Box((0.28, 0.13, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=dark_cast,
            name=f"foot_clamp_{side}",
        )

        for x in (-0.075, 0.075):
            frame.visual(
                Box((0.036, 0.045, 0.39)),
                origin=Origin(xyz=(x, y, 0.265)),
                material=dark_cast,
                name=f"cheek_rib_{side}_{'front' if x > 0 else 'rear'}",
            )
            frame.visual(
                Box((0.055, 0.105, 0.034)),
                origin=Origin(xyz=(x, y, 0.095)),
                material=dark_cast,
                name=f"rib_foot_{side}_{'front' if x > 0 else 'rear'}",
            )

        for z in (pivot_z - 0.074, pivot_z + 0.074):
            frame.visual(
                Box((0.150, 0.018, 0.034)),
                origin=Origin(xyz=(0.0, inner_y + sign * 0.004, z)),
                material=machined,
                name=f"clamp_face_{side}_{'upper' if z > pivot_z else 'lower'}",
            )

        frame.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, inner_y + sign * 0.004, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"inner_cap_{side}",
        )
        frame.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, outer_y - sign * 0.004, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"outer_cap_{side}",
        )

        for x in (-0.052, 0.052):
            for z in (pivot_z - 0.052, pivot_z + 0.052):
                frame.visual(
                    Cylinder(radius=0.009, length=0.008),
                    origin=Origin(xyz=(x, inner_y + sign * 0.013, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=machined,
                    name=f"cap_screw_{side}_{x:+.0e}_{z:.2f}",
                )

    for x in (-0.350, 0.350):
        frame.visual(
            Box((0.050, 0.220, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=rubber,
            name=f"travel_bumper_{'front' if x > 0 else 'rear'}",
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.028, length=0.464),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="cross_shaft",
    )
    for side, y in (("pos", 0.202), ("neg", -0.202)):
        cradle.visual(
            Cylinder(radius=0.052, length=0.083),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"trunnion_collar_{side}",
        )
        cradle.visual(
            Box((0.070, 0.026, 0.220)),
            origin=Origin(xyz=(0.0, y, -0.103)),
            material=blue,
            name=f"hanging_cheek_{side}",
        )
        cradle.visual(
            Box((0.125, 0.022, 0.028)),
            origin=Origin(xyz=(0.062, y, -0.060)),
            material=blue,
            name=f"diagonal_rib_{side}_front",
        )
        cradle.visual(
            Box((0.125, 0.022, 0.028)),
            origin=Origin(xyz=(-0.062, y, -0.135)),
            material=blue,
            name=f"diagonal_rib_{side}_rear",
        )

    cradle.visual(
        Box((0.500, 0.330, 0.026)),
        origin=Origin(xyz=(0.060, 0.0, -0.202)),
        material=blue,
        name="floor_plate",
    )
    for y in (-0.177, 0.177):
        cradle.visual(
            Box((0.510, 0.030, 0.105)),
            origin=Origin(xyz=(0.060, y, -0.153)),
            material=blue,
            name=f"side_rail_{'pos' if y > 0 else 'neg'}",
        )
    for x in (-0.195, 0.315):
        cradle.visual(
            Box((0.030, 0.330, 0.095)),
            origin=Origin(xyz=(x, 0.0, -0.158)),
            material=blue,
            name=f"end_rail_{'front' if x > 0 else 'rear'}",
        )
    cradle.visual(
        Box((0.320, 0.250, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, -0.182)),
        material=rubber,
        name="payload_pad",
    )
    for x in (-0.145, 0.255):
        cradle.visual(
            Box((0.040, 0.260, 0.110)),
            origin=Origin(xyz=(x, 0.0, -0.134)),
            material=machined,
            name=f"clamp_bar_{'front' if x > 0 else 'rear'}",
        )

    model.articulation(
        "frame_to_cradle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.50, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    cradle = object_model.get_part("cradle")
    tilt = object_model.get_articulation("frame_to_cradle")

    ctx.expect_gap(
        frame,
        cradle,
        axis="y",
        positive_elem="tower_pos",
        negative_elem="trunnion_collar_pos",
        min_gap=0.002,
        name="positive tower clears cradle trunnion",
    )
    ctx.expect_gap(
        cradle,
        frame,
        axis="y",
        positive_elem="trunnion_collar_neg",
        negative_elem="tower_neg",
        min_gap=0.002,
        name="negative tower clears cradle trunnion",
    )
    ctx.expect_overlap(
        frame,
        cradle,
        axes="xz",
        elem_a="inner_cap_pos",
        elem_b="cross_shaft",
        min_overlap=0.045,
        name="cap visually surrounds the shaft axis",
    )

    for q, label in ((-0.50, "rearward"), (0.50, "forward")):
        with ctx.pose({tilt: q}):
            ctx.expect_gap(
                cradle,
                frame,
                axis="z",
                positive_elem="floor_plate",
                negative_elem="base_plate",
                min_gap=0.045,
                name=f"{label} tilt keeps cradle above base",
            )

            floor_aabb = ctx.part_element_world_aabb(cradle, elem="floor_plate")
            bumper_aabb = ctx.part_element_world_aabb(
                frame,
                elem="travel_bumper_front" if q > 0.0 else "travel_bumper_rear",
            )
            if floor_aabb is not None and bumper_aabb is not None:
                if q > 0.0:
                    gap = bumper_aabb[0][0] - floor_aabb[1][0]
                else:
                    gap = floor_aabb[0][0] - bumper_aabb[1][0]
                ctx.check(
                    f"{label} tilt leaves bumper clearance",
                    gap > 0.010,
                    details=f"computed x gap={gap:.4f}",
                )
            else:
                ctx.fail(f"{label} tilt leaves bumper clearance", "missing AABB for floor or bumper")

    return ctx.report()


object_model = build_object_model()
