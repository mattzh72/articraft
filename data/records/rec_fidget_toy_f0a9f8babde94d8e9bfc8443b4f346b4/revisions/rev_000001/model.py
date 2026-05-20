from __future__ import annotations

from math import pi

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


LINK_COUNT = 12
LINK_PITCH = 0.028
PLATE_THICKNESS = 0.0085
HINGE_RADIUS = 0.0048
CENTER_BARREL_LENGTH = 0.0100
OUTER_BARREL_LENGTH = 0.0046
OUTER_BARREL_Y = 0.0095


def _link_plate_shape() -> cq.Workplane:
    """A single molded trapezoid link body with a rear tongue and front fork."""
    core = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.0065, -0.0073),
                (0.0205, -0.0108),
                (0.0235, -0.0094),
                (0.0235, 0.0094),
                (0.0205, 0.0108),
                (0.0065, 0.0073),
            ]
        )
        .close()
        .extrude(PLATE_THICKNESS)
        .translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))
    )

    rear_tongue = (
        cq.Workplane("XY")
        .box(0.0072, 0.0114, PLATE_THICKNESS)
        .translate((0.0056, 0.0, 0.0))
    )
    upper_fork = (
        cq.Workplane("XY")
        .box(0.0078, 0.0043, PLATE_THICKNESS)
        .translate((0.0242, OUTER_BARREL_Y, 0.0))
    )
    lower_fork = (
        cq.Workplane("XY")
        .box(0.0078, 0.0043, PLATE_THICKNESS)
        .translate((0.0242, -OUTER_BARREL_Y, 0.0))
    )

    return core.union(rear_tongue).union(upper_fork).union(lower_fork)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wacky_tracks_snap_chain")

    color_names = (
        "red_plastic",
        "orange_plastic",
        "yellow_plastic",
        "green_plastic",
        "blue_plastic",
        "purple_plastic",
    )
    color_values = (
        (0.94, 0.08, 0.10, 1.0),
        (1.00, 0.45, 0.05, 1.0),
        (0.98, 0.86, 0.08, 1.0),
        (0.06, 0.70, 0.22, 1.0),
        (0.05, 0.28, 0.92, 1.0),
        (0.54, 0.16, 0.78, 1.0),
    )
    for name, rgba in zip(color_names, color_values):
        model.material(name, rgba=rgba)
    model.material("dark_pin", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("shadow_groove", rgba=(0.03, 0.03, 0.035, 1.0))

    plate_mesh = mesh_from_cadquery(_link_plate_shape(), "trapezoid_link_plate")

    segments = []
    for index in range(LINK_COUNT):
        material = color_names[index % len(color_names)]
        segment = model.part(f"segment_{index}")
        segment.visual(plate_mesh, material=material, name="trapezoid_plate")
        segment.visual(
            Cylinder(radius=HINGE_RADIUS, length=CENTER_BARREL_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
            name="rear_barrel",
        )
        for barrel_name, cap_name, y_pos in (
            ("front_barrel_0", "pin_cap_0", OUTER_BARREL_Y),
            ("front_barrel_1", "pin_cap_1", -OUTER_BARREL_Y),
        ):
            segment.visual(
                Cylinder(radius=HINGE_RADIUS, length=OUTER_BARREL_LENGTH),
                origin=Origin(xyz=(LINK_PITCH, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=material,
                name=barrel_name,
            )
            segment.visual(
                Cylinder(radius=0.0032, length=0.0012),
                origin=Origin(
                    xyz=(
                        LINK_PITCH,
                        y_pos + (0.5 * OUTER_BARREL_LENGTH + 0.00045) * (1.0 if y_pos > 0.0 else -1.0),
                        0.0,
                    ),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material="dark_pin",
                name=cap_name,
            )
        segment.visual(
            Box((0.0095, 0.0021, 0.0013)),
            origin=Origin(xyz=(0.0155, 0.0, PLATE_THICKNESS / 2.0 + 0.00045)),
            material=material,
            name="raised_click_rib",
        )
        for groove_index, x_pos in enumerate((0.0108, 0.0202)):
            segment.visual(
                Box((0.0011, 0.0148, 0.00045)),
                origin=Origin(xyz=(x_pos, 0.0, PLATE_THICKNESS / 2.0 + 0.00072)),
                material="shadow_groove",
                name=f"detent_groove_{groove_index}",
            )
        segments.append(segment)

    for index in range(LINK_COUNT - 1):
        model.articulation(
            f"hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=segments[index],
            child=segments[index + 1],
            origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=5.0, lower=-pi / 2.0, upper=pi / 2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "twelve molded links",
        len(object_model.parts) == LINK_COUNT,
        details=f"found {len(object_model.parts)} parts",
    )
    ctx.check(
        "eleven sequential hinge joints",
        len(object_model.articulations) == LINK_COUNT - 1,
        details=f"found {len(object_model.articulations)} articulations",
    )

    for index in range(LINK_COUNT - 1):
        parent = object_model.get_part(f"segment_{index}")
        child = object_model.get_part(f"segment_{index + 1}")
        ctx.expect_origin_gap(
            child,
            parent,
            axis="x",
            min_gap=LINK_PITCH - 0.0005,
            max_gap=LINK_PITCH + 0.0005,
            name=f"hinge_{index} pitch spacing",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="xz",
            min_overlap=0.006,
            elem_a="rear_barrel",
            elem_b="front_barrel_0",
            name=f"hinge_{index} coaxial barrel projection",
        )

    first = object_model.get_part("segment_0")
    second = object_model.get_part("segment_1")
    ctx.expect_gap(
        first,
        second,
        axis="y",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="front_barrel_0",
        negative_elem="rear_barrel",
        name="front fork clears center hinge barrel",
    )

    third = object_model.get_part("segment_2")
    hinge = object_model.get_articulation("hinge_0")
    rest_pos = ctx.part_world_position(third)
    with ctx.pose({hinge: pi / 2.0}):
        bent_pos = ctx.part_world_position(third)
    ctx.check(
        "positive hinge click bends chain upward",
        rest_pos is not None and bent_pos is not None and bent_pos[2] > rest_pos[2] + 0.020,
        details=f"rest={rest_pos}, bent={bent_pos}",
    )

    return ctx.report()


object_model = build_object_model()
