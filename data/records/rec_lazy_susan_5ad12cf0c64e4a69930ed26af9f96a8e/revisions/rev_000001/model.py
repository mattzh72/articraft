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


def _kidney_blank(radius: float, height: float, *, notch_x: float, notch_radius: float) -> cq.Workplane:
    """A round shelf with a broad concave bite, like cabinet lazy-Susan kidney shelves."""
    shelf = cq.Workplane("XY").circle(radius).extrude(height)
    notch = cq.Workplane("XY").center(notch_x, 0.0).circle(notch_radius).extrude(height)
    return shelf.cut(notch)


def _kidney_shelf_board(z_center: float) -> cq.Workplane:
    board_thickness = 0.035
    board = _kidney_blank(0.390, board_thickness, notch_x=-0.330, notch_radius=0.205)
    axle_hole = cq.Workplane("XY").circle(0.026).extrude(board_thickness)
    return board.cut(axle_hole).translate((0.0, 0.0, z_center - board_thickness / 2.0))


def _kidney_guard_rail(z_center: float) -> cq.Workplane:
    board_thickness = 0.035
    rail_height = 0.052
    rail_overlap = 0.003
    outer = _kidney_blank(0.390, rail_height, notch_x=-0.330, notch_radius=0.205)
    inner = _kidney_blank(0.357, rail_height + 0.006, notch_x=-0.330, notch_radius=0.238)
    rail = outer.cut(inner.translate((0.0, 0.0, -0.003)))
    return rail.translate((0.0, 0.0, z_center + board_thickness / 2.0 - rail_overlap))


def _bearing_ring(z_min: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.068)
        .circle(0.024)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cabinet_insert_lazy_susan")

    maple = model.material("pale_maple", rgba=(0.78, 0.58, 0.36, 1.0))
    plywood = model.material("cabinet_plywood", rgba=(0.70, 0.52, 0.32, 0.55))
    chrome = model.material("brushed_chrome", rgba=(0.62, 0.64, 0.65, 1.0))
    dark = model.material("shadowed_edge", rgba=(0.18, 0.16, 0.13, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.00, 1.00, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=plywood,
        name="floor",
    )
    cabinet.visual(
        Box((1.00, 0.036, 0.800)),
        origin=Origin(xyz=(0.0, -0.518, 0.400)),
        material=plywood,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.036, 1.00, 0.800)),
        origin=Origin(xyz=(-0.518, 0.0, 0.400)),
        material=plywood,
        name="side_wall",
    )
    cabinet.visual(
        Box((1.00, 1.00, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=plywood,
        name="top",
    )
    cabinet.visual(
        Box((0.040, 0.660, 0.060)),
        origin=Origin(xyz=(0.500, 0.080, 0.060)),
        material=dark,
        name="front_stile",
    )
    cabinet.visual(
        mesh_from_cadquery(_bearing_ring(0.034, 0.028), "bottom_bearing"),
        material=chrome,
        name="bottom_bearing",
    )
    cabinet.visual(
        mesh_from_cadquery(_bearing_ring(0.772, 0.030), "top_bearing"),
        material=chrome,
        name="top_bearing",
    )

    carousel = model.part("carousel")
    carousel.visual(
        Cylinder(radius=0.018, length=0.715),
        origin=Origin(xyz=(0.0, 0.0, 0.4125)),
        material=chrome,
        name="shaft",
    )
    carousel.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=chrome,
        name="bottom_washer",
    )
    carousel.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.769)),
        material=chrome,
        name="top_washer",
    )

    lower_z = 0.235
    upper_z = 0.555
    for prefix, z in (("lower", lower_z), ("upper", upper_z)):
        carousel.visual(
            mesh_from_cadquery(_kidney_shelf_board(z), f"{prefix}_shelf_board"),
            material=maple,
            name=f"{prefix}_shelf_board",
        )
        carousel.visual(
            mesh_from_cadquery(_kidney_guard_rail(z), f"{prefix}_shelf_rail"),
            material=chrome,
            name=f"{prefix}_shelf_rail",
        )
        carousel.visual(
            Cylinder(radius=0.052, length=0.082),
            origin=Origin(xyz=(0.0, 0.0, z + 0.018)),
            material=chrome,
            name=f"{prefix}_hub",
        )
        for i, angle in enumerate((math.radians(-58.0), math.radians(18.0), math.radians(92.0))):
            carousel.visual(
                Box((0.325, 0.020, 0.018)),
                origin=Origin(
                    xyz=(math.cos(angle) * 0.190, math.sin(angle) * 0.190, z - 0.020),
                    rpy=(0.0, 0.0, angle),
                ),
                material=chrome,
                name=f"{prefix}_arm_{i}",
            )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=carousel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    carousel = object_model.get_part("carousel")
    spin = object_model.get_articulation("shaft_spin")

    ctx.expect_within(
        carousel,
        cabinet,
        axes="xy",
        margin=0.0,
        name="rotating shelves fit inside the cabinet corner footprint",
    )
    ctx.expect_gap(
        carousel,
        cabinet,
        axis="z",
        positive_elem="lower_shelf_board",
        negative_elem="floor",
        min_gap=0.150,
        name="lower shelf is raised above the cabinet floor",
    )
    ctx.expect_gap(
        carousel,
        carousel,
        axis="z",
        positive_elem="upper_shelf_board",
        negative_elem="lower_shelf_rail",
        min_gap=0.190,
        name="upper and lower kidney shelves are at distinct heights",
    )
    ctx.expect_overlap(
        carousel,
        carousel,
        axes="xy",
        elem_a="shaft",
        elem_b="lower_hub",
        min_overlap=0.030,
        name="lower shelf hub is centered on the shaft",
    )
    ctx.expect_overlap(
        carousel,
        carousel,
        axes="xy",
        elem_a="shaft",
        elem_b="upper_hub",
        min_overlap=0.030,
        name="upper shelf hub is centered on the shaft",
    )
    ctx.expect_contact(
        carousel,
        cabinet,
        elem_a="bottom_washer",
        elem_b="bottom_bearing",
        contact_tol=0.001,
        name="lower thrust washer rests on the cabinet bearing",
    )
    ctx.expect_contact(
        carousel,
        cabinet,
        elem_a="top_washer",
        elem_b="top_bearing",
        contact_tol=0.001,
        name="upper thrust washer is captured under the top bearing",
    )

    rest_position = ctx.part_world_position(carousel)
    rest_aabb = ctx.part_element_world_aabb(carousel, elem="lower_shelf_board")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_position = ctx.part_world_position(carousel)
        turned_aabb = ctx.part_element_world_aabb(carousel, elem="lower_shelf_board")

    fixed_center = (
        rest_position is not None
        and turned_position is not None
        and max(abs(rest_position[i] - turned_position[i]) for i in range(3)) < 1e-6
    )
    ctx.check(
        "continuous shelf assembly rotates about the fixed vertical shaft",
        fixed_center,
        details=f"rest={rest_position}, turned={turned_position}",
    )
    if rest_aabb is not None and turned_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        rest_dy = rest_aabb[1][1] - rest_aabb[0][1]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        turned_dy = turned_aabb[1][1] - turned_aabb[0][1]
        reoriented = abs(rest_dx - turned_dy) < 0.020 and abs(rest_dy - turned_dx) < 0.020
    else:
        reoriented = False
    ctx.check(
        "kidney shelf footprint reorients with the continuous joint",
        reoriented,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
