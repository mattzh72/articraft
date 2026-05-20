from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.35
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.022
BASE_CORNER_RADIUS = 0.016

BARREL_CENTER_X = 0.062
BARREL_OUTER_RADIUS = 0.035
BARREL_INNER_RADIUS = 0.0315
BARREL_HEIGHT = 0.36
BARREL_BOTTOM_CAP = 0.012
BARREL_TOP_RING = 0.011
BARREL_LOWER_FLANGE = 0.018
ROD_GUIDE_RADIUS = 0.012
GLAND_HEIGHT = 0.012

YOKE_Z = 0.567
YOKE_LENGTH = 0.182
YOKE_WIDTH = 0.030
YOKE_THICKNESS = 0.020
HANDLE_STEM_RADIUS = 0.014
HANDLE_STEM_HEIGHT = 0.094
GRIP_RADIUS = 0.015
GRIP_LENGTH = 0.28
GRIP_SLEEVE_RADIUS = 0.0185
GRIP_SLEEVE_LENGTH = 0.064
GRIP_CENTER_Z = 0.118

ROD_RADIUS = 0.012
ROD_LENGTH = 0.322

PUMP_STROKE = 0.15


def _base_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    plate = plate.edges("|Z").fillet(BASE_CORNER_RADIUS)

    toe_ribs = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.115, 0.0),
                (0.115, 0.0),
            ]
        )
        .rect(0.065, BASE_WIDTH * 0.78)
        .extrude(0.005)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    center_spine = (
        cq.Workplane("XY")
        .rect(0.12, 0.05)
        .extrude(0.012)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    return plate.union(toe_ribs).union(center_spine)


def _barrel_shape(center_x: float) -> cq.Workplane:
    wall = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(BARREL_OUTER_RADIUS)
        .circle(BARREL_INNER_RADIUS)
        .extrude(BARREL_HEIGHT)
    )

    bottom_disk = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(BARREL_INNER_RADIUS)
        .extrude(BARREL_BOTTOM_CAP)
    )

    lower_flange = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(BARREL_OUTER_RADIUS + 0.006)
        .circle(BARREL_OUTER_RADIUS)
        .extrude(BARREL_LOWER_FLANGE)
    )

    top_ring = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(BARREL_INNER_RADIUS)
        .circle(ROD_GUIDE_RADIUS)
        .extrude(BARREL_TOP_RING)
        .translate((0.0, 0.0, BARREL_HEIGHT - BARREL_TOP_RING))
    )

    gland = (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(ROD_GUIDE_RADIUS + 0.004)
        .circle(ROD_GUIDE_RADIUS)
        .extrude(GLAND_HEIGHT)
        .translate((0.0, 0.0, BARREL_HEIGHT))
    )

    return (
        wall.union(bottom_disk)
        .union(lower_flange)
        .union(top_ring)
        .union(gland)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )


def _gauge_shape() -> cq.Workplane:
    bracket = (
        cq.Workplane("XY")
        .box(
            0.07,
            0.028,
            0.058,
            centered=(True, True, False),
        )
        .translate((0.0, BASE_WIDTH * 0.31, BASE_THICKNESS))
    )

    gauge_body = (
        cq.Workplane("XZ")
        .center(0.0, BASE_THICKNESS + 0.064)
        .circle(0.034)
        .extrude(0.018)
        .translate((0.0, BASE_WIDTH / 2.0 - 0.018, 0.0))
    )

    return bracket.union(gauge_body)


def _handle_bridge_shape() -> cq.Workplane:
    yoke = cq.Workplane("XY").box(YOKE_LENGTH, YOKE_WIDTH, YOKE_THICKNESS)

    stem = (
        cq.Workplane("XY")
        .circle(HANDLE_STEM_RADIUS)
        .extrude(HANDLE_STEM_HEIGHT)
        .translate((0.0, 0.0, YOKE_THICKNESS / 2.0))
    )

    grip = (
        cq.Workplane("YZ")
        .circle(GRIP_RADIUS)
        .extrude(GRIP_LENGTH)
        .translate((-GRIP_LENGTH / 2.0, 0.0, GRIP_CENTER_Z))
    )

    left_sleeve = (
        cq.Workplane("YZ")
        .circle(GRIP_SLEEVE_RADIUS)
        .extrude(GRIP_SLEEVE_LENGTH)
        .translate((-GRIP_LENGTH / 2.0, 0.0, GRIP_CENTER_Z))
    )
    right_sleeve = (
        cq.Workplane("YZ")
        .circle(GRIP_SLEEVE_RADIUS)
        .extrude(GRIP_SLEEVE_LENGTH)
        .translate((GRIP_LENGTH / 2.0 - GRIP_SLEEVE_LENGTH, 0.0, GRIP_CENTER_Z))
    )

    return yoke.union(stem).union(grip).union(left_sleeve).union(right_sleeve)


def _rod_shape(center_x: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .circle(ROD_RADIUS)
        .extrude(ROD_LENGTH)
        .translate((0.0, 0.0, -ROD_LENGTH))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_barrel_floor_pump")

    model.material("base_black", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("gauge_gray", rgba=(0.84, 0.85, 0.86, 1.0))
    model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))

    pump_body = model.part("pump_body")
    pump_body.visual(
        mesh_from_cadquery(_base_plate_shape(), "pump_base_plate"),
        material="base_black",
        name="base_plate",
    )
    pump_body.visual(
        mesh_from_cadquery(_barrel_shape(-BARREL_CENTER_X), "left_barrel_shell"),
        material="steel",
        name="left_barrel",
    )
    pump_body.visual(
        mesh_from_cadquery(_barrel_shape(BARREL_CENTER_X), "right_barrel_shell"),
        material="steel",
        name="right_barrel",
    )
    pump_body.visual(
        mesh_from_cadquery(_gauge_shape(), "front_gauge_housing"),
        material="gauge_gray",
        name="front_gauge",
    )

    handle_assembly = model.part("handle_assembly")
    handle_assembly.visual(
        mesh_from_cadquery(_handle_bridge_shape(), "t_handle_bridge"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="handle_black",
        name="handle_bridge",
    )
    handle_assembly.visual(
        mesh_from_cadquery(_rod_shape(-BARREL_CENTER_X), "left_pump_rod"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="left_rod",
    )
    handle_assembly.visual(
        mesh_from_cadquery(_rod_shape(BARREL_CENTER_X), "right_pump_rod"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="right_rod",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=pump_body,
        child=handle_assembly,
        origin=Origin(xyz=(0.0, 0.0, YOKE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.7,
            lower=-PUMP_STROKE,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pump_body = object_model.get_part("pump_body")
    handle_assembly = object_model.get_part("handle_assembly")
    slide = object_model.get_articulation("body_to_handle")

    ctx.allow_overlap(
        pump_body,
        handle_assembly,
        elem_a="left_barrel",
        elem_b="left_rod",
        reason="The left rod is intentionally represented as sliding inside the hollow left pump barrel.",
    )
    ctx.allow_overlap(
        pump_body,
        handle_assembly,
        elem_a="right_barrel",
        elem_b="right_rod",
        reason="The right rod is intentionally represented as sliding inside the hollow right pump barrel.",
    )

    ctx.expect_within(
        handle_assembly,
        pump_body,
        axes="xy",
        inner_elem="left_rod",
        outer_elem="left_barrel",
        margin=0.0,
        name="left rod stays centered in left barrel",
    )
    ctx.expect_within(
        handle_assembly,
        pump_body,
        axes="xy",
        inner_elem="right_rod",
        outer_elem="right_barrel",
        margin=0.0,
        name="right rod stays centered in right barrel",
    )
    ctx.expect_overlap(
        handle_assembly,
        pump_body,
        axes="z",
        elem_a="left_rod",
        elem_b="left_barrel",
        min_overlap=0.14,
        name="left rod remains inserted at rest",
    )
    ctx.expect_overlap(
        handle_assembly,
        pump_body,
        axes="z",
        elem_a="right_rod",
        elem_b="right_barrel",
        min_overlap=0.14,
        name="right rod remains inserted at rest",
    )
    rest_pos = ctx.part_world_position(handle_assembly)
    with ctx.pose({slide: -PUMP_STROKE}):
        ctx.expect_within(
            handle_assembly,
            pump_body,
            axes="xy",
            inner_elem="left_rod",
            outer_elem="left_barrel",
            margin=0.0,
            name="left rod stays centered when compressed",
        )
        ctx.expect_within(
            handle_assembly,
            pump_body,
            axes="xy",
            inner_elem="right_rod",
            outer_elem="right_barrel",
            margin=0.0,
            name="right rod stays centered when compressed",
        )
        ctx.expect_overlap(
            handle_assembly,
            pump_body,
            axes="z",
            elem_a="left_rod",
            elem_b="left_barrel",
            min_overlap=0.28,
            name="left rod retains deep insertion when compressed",
        )
        ctx.expect_overlap(
            handle_assembly,
            pump_body,
            axes="z",
            elem_a="right_rod",
            elem_b="right_barrel",
            min_overlap=0.28,
            name="right rod retains deep insertion when compressed",
        )
        ctx.expect_gap(
            handle_assembly,
            pump_body,
            axis="z",
            positive_elem="handle_bridge",
            negative_elem="left_barrel",
            min_gap=0.01,
            name="t handle bridge stays above barrel tops at full stroke",
        )
        compressed_pos = ctx.part_world_position(handle_assembly)

    ctx.check(
        "handle assembly translates downward on compression",
        rest_pos is not None
        and compressed_pos is not None
        and compressed_pos[2] < rest_pos[2] - 0.10,
        details=f"rest={rest_pos}, compressed={compressed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
