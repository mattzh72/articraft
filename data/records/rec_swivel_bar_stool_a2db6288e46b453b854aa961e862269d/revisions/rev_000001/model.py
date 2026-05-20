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


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _member_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_member_rpy(a, b)),
        material=material,
        name=name,
    )


def _washer(outer_radius: float, inner_radius: float, height: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _seat_shape():
    return (
        cq.Workplane("XY")
        .circle(0.158)
        .workplane(offset=0.030)
        .circle(0.197)
        .workplane(offset=0.030)
        .circle(0.184)
        .loft(combine=True)
    )


def _flap_shape():
    return cq.Workplane("XY").rect(0.132, 0.092).extrude(0.016)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_lift_swivel_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.19, 0.20, 0.22, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    charcoal_vinyl = model.material("charcoal_vinyl", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.222, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_trim,
        name="foot_pad",
    )
    base.visual(
        Cylinder(radius=0.225, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed_steel,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.200, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=satin_aluminum,
        name="base_cap",
    )
    base.visual(
        mesh_from_cadquery(_washer(0.042, 0.0275, 0.325), "lower_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=brushed_steel,
        name="sleeve",
    )
    base.visual(
        mesh_from_cadquery(_washer(0.185, 0.168, 0.014), "foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        material=satin_aluminum,
        name="foot_ring",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        start = (0.041 * math.cos(angle), 0.041 * math.sin(angle), 0.282)
        end = (0.168 * math.cos(angle), 0.168 * math.sin(angle), 0.309)
        _add_member(
            base,
            start,
            end,
            radius=0.007,
            material=brushed_steel,
            name=f"ring_brace_{index}",
        )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.024, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_aluminum,
        name="piston",
    )
    lift_column.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brushed_steel,
        name="guide_ring",
    )
    lift_column.visual(
        Cylinder(radius=0.038, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        material=brushed_steel,
        name="head_collar",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.082, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_trim,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.090, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_trim,
        name="seat_core",
    )
    seat.visual(
        mesh_from_cadquery(_seat_shape(), "seat_cushion"),
        material=charcoal_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.140, 0.100, 0.006)),
        origin=Origin(xyz=(-0.005, 0.0, 0.063)),
        material=matte_black,
        name="flap_pad",
    )
    seat.visual(
        Box((0.022, 0.028, 0.024)),
        origin=Origin(xyz=(0.187, 0.0, 0.012)),
        material=dark_trim,
        name="lever_mount",
    )
    for index, y_pos in enumerate((-0.012, 0.012)):
        seat.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(
                xyz=(0.204, y_pos, 0.012),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_trim,
            name=f"lever_lug_{index}",
        )
    for index, y_pos in enumerate((-0.018, 0.018)):
        seat.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(
                xyz=(-0.074, y_pos, 0.064),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_trim,
            name=f"hinge_lug_{index}",
        )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Cylinder(radius=0.0052, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_pin",
    )
    side_lever.visual(
        Box((0.108, 0.012, 0.010)),
        origin=Origin(xyz=(0.054, 0.0, -0.006)),
        material=dark_trim,
        name="lever_arm",
    )
    side_lever.visual(
        Cylinder(radius=0.0058, length=0.026),
        origin=Origin(xyz=(0.108, 0.0, -0.023)),
        material=dark_trim,
        name="lever_grip",
    )

    cover_flap = model.part("cover_flap")
    cover_flap.visual(
        Cylinder(radius=0.0042, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="flap_barrel",
    )
    cover_flap.visual(
        mesh_from_cadquery(_flap_shape(), "cover_flap"),
        origin=Origin(xyz=(0.070, 0.0, -0.001)),
        material=matte_black,
        name="cover_panel",
    )

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.22,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "seat_spin",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=side_lever,
        origin=Origin(xyz=(0.204, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.5,
            lower=0.0,
            upper=0.75,
        ),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=cover_flap,
        origin=Origin(xyz=(-0.070, 0.0, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    side_lever = object_model.get_part("side_lever")
    cover_flap = object_model.get_part("cover_flap")

    lift_slide = object_model.get_articulation("lift_slide")
    seat_spin = object_model.get_articulation("seat_spin")
    lever_pivot = object_model.get_articulation("lever_pivot")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="piston",
        outer_elem="sleeve",
        margin=0.0,
        name="piston stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="piston",
        elem_b="sleeve",
        min_overlap=0.110,
        name="piston remains inserted at rest",
    )
    ctx.expect_gap(
        cover_flap,
        seat,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="flap_pad",
        min_gap=0.0005,
        max_gap=0.0020,
        name="closed flap sits just above seat pad",
    )

    seat_rest = ctx.part_world_position(seat)
    lever_rest = ctx.part_world_position(side_lever)

    with ctx.pose({lift_slide: lift_slide.motion_limits.upper}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="piston",
            outer_elem="sleeve",
            margin=0.0,
            name="piston stays centered in sleeve when raised",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="piston",
            elem_b="sleeve",
            min_overlap=0.070,
            name="piston remains retained when raised",
        )
        seat_raised = ctx.part_world_position(seat)

    ctx.check(
        "seat height increases with gas lift",
        seat_rest is not None
        and seat_raised is not None
        and seat_raised[2] > seat_rest[2] + 0.09,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    with ctx.pose({seat_spin: math.pi / 2.0}):
        lever_spun = ctx.part_world_position(side_lever)

    ctx.check(
        "seat swivel carries side lever around column",
        lever_rest is not None
        and lever_spun is not None
        and math.hypot(lever_spun[0] - lever_rest[0], lever_spun[1] - lever_rest[1]) > 0.18,
        details=f"rest={lever_rest}, spun={lever_spun}",
    )

    lever_rest_aabb = ctx.part_world_aabb(side_lever)
    with ctx.pose({lever_pivot: lever_pivot.motion_limits.upper}):
        lever_lifted_aabb = ctx.part_world_aabb(side_lever)

    ctx.check(
        "side lever rotates upward",
        lever_rest_aabb is not None
        and lever_lifted_aabb is not None
        and lever_lifted_aabb[1][2] > lever_rest_aabb[1][2] + 0.03,
        details=f"rest={lever_rest_aabb}, lifted={lever_lifted_aabb}",
    )

    flap_rest_aabb = ctx.part_world_aabb(cover_flap)
    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        flap_open_aabb = ctx.part_world_aabb(cover_flap)

    ctx.check(
        "cover flap opens upward",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][2] > flap_rest_aabb[1][2] + 0.05,
        details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
