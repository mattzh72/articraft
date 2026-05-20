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


SEAT_RADIUS = 0.182
SEAT_RIM_OUTER_RADIUS = 0.196
SEAT_RIM_HEIGHT = 0.026
SEAT_TOP_Z = 0.028
HINGE_X = -0.190
HINGE_Z = 0.057


def _polar_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + index * 2.0 * math.pi / count),
            radius * math.sin(phase + index * 2.0 * math.pi / count),
        )
        for index in range(count)
    ]


def _build_seat_shell() -> cq.Workplane:
    seat_plate = (
        cq.Workplane("XY")
        .circle(SEAT_RADIUS)
        .extrude(0.004)
        .translate((0.0, 0.0, SEAT_TOP_Z - 0.004))
    )

    perforation_points: list[tuple[float, float]] = []
    perforation_points.extend(_polar_points(0.045, 6, phase=math.pi / 6.0))
    perforation_points.extend(_polar_points(0.082, 10, phase=0.0))
    perforation_points.extend(_polar_points(0.118, 14, phase=math.pi / 14.0))
    perforation_points.extend(_polar_points(0.150, 18, phase=0.0))

    seat_plate = (
        seat_plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(perforation_points)
        .hole(0.013)
    )

    rolled_rim = (
        cq.Workplane("XY")
        .circle(SEAT_RIM_OUTER_RADIUS)
        .circle(SEAT_RADIUS)
        .extrude(SEAT_RIM_HEIGHT)
    )

    hub = cq.Workplane("XY").circle(0.050).extrude(SEAT_TOP_Z - 0.002)
    lower_boss = cq.Workplane("XY").circle(0.062).extrude(0.008)

    seat_shell = rolled_rim.union(seat_plate).union(hub).union(lower_boss)

    rib_template = (
        cq.Workplane("XY")
        .box(0.106, 0.022, SEAT_TOP_Z - 0.010)
        .translate((0.109, 0.0, (SEAT_TOP_Z - 0.010) / 2.0 + 0.010))
    )
    for angle_deg in (0.0, 60.0, 120.0, 180.0, 240.0, 300.0):
        rib = rib_template.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        seat_shell = seat_shell.union(rib)

    return seat_shell


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_swivel_bar_stool")

    base_metal = model.material("base_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    seat_metal = model.material("seat_metal", rgba=(0.47, 0.50, 0.52, 1.0))
    pad_material = model.material("pad_material", rgba=(0.20, 0.23, 0.25, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.240, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=base_metal,
        name="base_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=base_metal,
        name="lower_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.051, length=0.646),
        origin=Origin(xyz=(0.0, 0.0, 0.387)),
        material=base_metal,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.076, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=base_metal,
        name="head",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_build_seat_shell(), "seat_shell"),
        material=seat_metal,
        name="seat_shell",
    )
    for index, y_pos in enumerate((-0.055, 0.055)):
        seat.visual(
            Box((0.062, 0.030, 0.038)),
            origin=Origin(xyz=(-0.152, y_pos, 0.037)),
            material=seat_metal,
            name=f"rear_bracket_{index}",
        )
    for index, y_pos in enumerate((-0.037, 0.037)):
        seat.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(HINGE_X, y_pos, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=seat_metal,
            name=f"hinge_barrel_{index}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.0085, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seat_metal,
        name="hinge_knuckle",
    )
    backrest.visual(
        Box((0.020, 0.030, 0.010)),
        origin=Origin(xyz=(-0.014, 0.0, 0.010)),
        material=seat_metal,
        name="hinge_tab",
    )
    backrest.visual(
        Box((0.022, 0.180, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, 0.070)),
        material=seat_metal,
        name="back_bridge",
    )
    backrest.visual(
        Box((0.014, 0.032, 0.110)),
        origin=Origin(xyz=(-0.020, -0.056, 0.095)),
        material=seat_metal,
        name="back_strut_0",
    )
    backrest.visual(
        Box((0.014, 0.032, 0.110)),
        origin=Origin(xyz=(-0.020, 0.056, 0.095)),
        material=seat_metal,
        name="back_strut_1",
    )
    backrest.visual(
        Box((0.012, 0.280, 0.150)),
        origin=Origin(xyz=(-0.028, 0.0, 0.135)),
        material=seat_metal,
        name="back_panel",
    )
    backrest.visual(
        Box((0.012, 0.048, 0.074)),
        origin=Origin(xyz=(-0.026, 0.0, 0.046)),
        material=seat_metal,
        name="back_spine",
    )
    backrest.visual(
        Box((0.040, 0.260, 0.110)),
        origin=Origin(xyz=(-0.003, 0.0, 0.135)),
        material=pad_material,
        name="back_pad",
    )

    swivel = model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=6.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    swivel.meta["semantic"] = "continuous seat swivel"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    swivel = object_model.get_articulation("pedestal_to_seat")
    back_hinge = object_model.get_articulation("seat_to_backrest")

    ctx.check(
        "seat swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower is None
        and swivel.motion_limits.upper is None,
        details=(
            f"type={swivel.articulation_type}, "
            f"lower={None if swivel.motion_limits is None else swivel.motion_limits.lower}, "
            f"upper={None if swivel.motion_limits is None else swivel.motion_limits.upper}"
        ),
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        min_overlap=0.10,
        elem_a="seat_shell",
        elem_b="head",
        name="seat stays centered above the pedestal head",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="seat_shell",
        negative_elem="head",
        min_gap=0.0,
        max_gap=0.020,
        name="seat shell sits just above the pedestal head",
    )
    ctx.expect_gap(
        backrest,
        seat,
        axis="z",
        positive_elem="back_pad",
        negative_elem="seat_shell",
        min_gap=0.040,
        name="upright back pad clears the perforated seat",
    )

    rest_seat_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_seat_pos = ctx.part_world_position(seat)
    ctx.check(
        "seat rotates about a fixed pedestal axis",
        rest_seat_pos is not None
        and turned_seat_pos is not None
        and abs(rest_seat_pos[0] - turned_seat_pos[0]) < 1e-6
        and abs(rest_seat_pos[1] - turned_seat_pos[1]) < 1e-6
        and abs(rest_seat_pos[2] - turned_seat_pos[2]) < 1e-6,
        details=f"rest={rest_seat_pos}, turned={turned_seat_pos}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    back_upper = 1.45
    if back_hinge.motion_limits is not None and back_hinge.motion_limits.upper is not None:
        back_upper = back_hinge.motion_limits.upper

    with ctx.pose({back_hinge: back_upper}):
        folded_pad_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        ctx.expect_overlap(
            backrest,
            seat,
            axes="xy",
            elem_a="back_pad",
            elem_b="seat_shell",
            min_overlap=0.10,
            name="folded back pad lands over the seat footprint",
        )
        ctx.expect_gap(
            backrest,
            seat,
            axis="z",
            positive_elem="back_pad",
            negative_elem="seat_shell",
            min_gap=0.0,
            max_gap=0.045,
            name="folded back pad rests just above the seat shell",
        )

    rest_pad_center_z = _aabb_center_z(rest_pad_aabb)
    folded_pad_center_z = _aabb_center_z(folded_pad_aabb)
    ctx.check(
        "backrest folds downward",
        rest_pad_center_z is not None
        and folded_pad_center_z is not None
        and folded_pad_center_z < rest_pad_center_z - 0.070,
        details=f"rest_z={rest_pad_center_z}, folded_z={folded_pad_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
