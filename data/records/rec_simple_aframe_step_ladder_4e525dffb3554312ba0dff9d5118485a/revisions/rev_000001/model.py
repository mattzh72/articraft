from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _beam_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    depth: float,
    width: float,
    material,
) -> None:
    """Add a rectangular extrusion whose long axis follows an X-Z segment."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = sqrt(dx * dx + dz * dz)
    pitch = atan2(dx, dz)
    part.visual(
        Box((depth, width, length)),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    tread_edge = model.material("dark_grip_rubber", rgba=(0.045, 0.047, 0.050, 1.0))
    cap_red = model.material("red_molded_top_cap", rgba=(0.82, 0.08, 0.045, 1.0))
    hinge_steel = model.material("zinc_hinge_steel", rgba=(0.52, 0.54, 0.56, 1.0))

    front = model.part("front_frame")

    rail_y = 0.285
    front_rail_start_z = 0.045
    front_rail_end_z = 1.365
    front_rail_start_x = -0.365
    front_rail_end_x = 0.045

    for side, y in enumerate((-rail_y, rail_y)):
        _beam_between(
            front,
            f"front_rail_{side}",
            (front_rail_start_x, y, front_rail_start_z),
            (front_rail_end_x, y, front_rail_end_z),
            depth=0.050,
            width=0.050,
            material=aluminum,
        )
        front.visual(
            Box((0.18, 0.115, 0.040)),
            origin=Origin(xyz=(front_rail_start_x - 0.01, y, 0.030)),
            material=tread_edge,
            name=f"front_foot_{side}",
        )

    # Wide, horizontal climbing treads.  Each tread slightly penetrates the side
    # rails so the part reads as a riveted one-piece climbing frame.
    tread_zs = (0.355, 0.680, 1.005)
    for tread_i, z in enumerate(tread_zs):
        t = (z - front_rail_start_z) / (front_rail_end_z - front_rail_start_z)
        rail_x = front_rail_start_x + t * (front_rail_end_x - front_rail_start_x)
        tread_x = rail_x + 0.035
        front.visual(
            Box((0.235, 0.675, 0.050)),
            origin=Origin(xyz=(tread_x, 0.0, z)),
            material=aluminum,
            name=f"tread_{tread_i}",
        )
        for rib_i, rib_x_offset in enumerate((-0.070, 0.000, 0.070)):
            front.visual(
                Box((0.018, 0.640, 0.008)),
                origin=Origin(xyz=(tread_x + rib_x_offset, 0.0, z + 0.029)),
                material=tread_edge,
                name=f"tread_{tread_i}_grip_{rib_i}",
            )

    # A molded utility top cap / platform, plus the short clevis bracket that
    # carries the rear folding frame close to the top.
    front.visual(
        Box((0.325, 0.745, 0.090)),
        origin=Origin(xyz=(-0.085, 0.0, 1.395)),
        material=cap_red,
        name="top_cap",
    )
    front.visual(
        Box((0.190, 0.500, 0.012)),
        origin=Origin(xyz=(-0.090, 0.0, 1.439)),
        material=tread_edge,
        name="top_tray_inset",
    )
    for side, y in enumerate((-0.378, 0.378)):
        front.visual(
            Box((0.145, 0.034, 0.070)),
            origin=Origin(xyz=(0.125, y, 1.360)),
            material=hinge_steel,
            name=f"top_bracket_{side}",
        )
        front.visual(
            Cylinder(radius=0.016, length=0.028),
            origin=Origin(xyz=(0.170, y, 1.360), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=f"pin_head_{side}",
        )

    rear = model.part("rear_frame")

    rear_leg_start = (0.125, 0.0, -0.140)
    rear_leg_end = (0.625, 0.0, -1.325)
    rear_ys = (-0.285, 0.285)

    # The child frame is authored in the hinge frame.  A compact hinge barrel
    # and two short straps keep the folding member visually close to the cap.
    rear.visual(
        Cylinder(radius=0.022, length=0.722),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel",
    )
    for side, y in enumerate(rear_ys):
        _beam_between(
            rear,
            f"short_hinge_link_{side}",
            (0.018, y, -0.010),
            (rear_leg_start[0], y, rear_leg_start[2]),
            depth=0.040,
            width=0.050,
            material=hinge_steel,
        )
        _beam_between(
            rear,
            f"rear_leg_{side}",
            (rear_leg_start[0], y, rear_leg_start[2]),
            (rear_leg_end[0], y, rear_leg_end[2]),
            depth=0.050,
            width=0.050,
            material=aluminum,
        )
        rear.visual(
            Box((0.180, 0.115, 0.035)),
            origin=Origin(xyz=(rear_leg_end[0] + 0.010, y, rear_leg_end[2] - 0.016)),
            material=tread_edge,
            name=f"rear_foot_{side}",
        )

    for bar_i, bar_t in enumerate((0.42, 0.72)):
        x = rear_leg_start[0] + bar_t * (rear_leg_end[0] - rear_leg_start[0])
        z = rear_leg_start[2] + bar_t * (rear_leg_end[2] - rear_leg_start[2])
        rear.visual(
            Box((0.055, 0.675, 0.048)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_crossbar_{bar_i}",
        )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.170, 0.0, 1.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "single top hinge",
        len(object_model.articulations) == 1 and hinge.axis == (0.0, 1.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="y",
        elem_a="top_cap",
        elem_b="hinge_barrel",
        min_overlap=0.50,
        name="rear frame hinge spans the top cap width",
    )
    ctx.expect_gap(
        rear,
        front,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="top_cap",
        min_gap=0.030,
        max_gap=0.090,
        name="short folding hinge sits just behind the cap",
    )

    with ctx.pose({hinge: 0.0}):
        open_aabb = ctx.part_world_aabb(rear)
    with ctx.pose({hinge: 0.70}):
        folded_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "rear support folds toward the climbing frame",
        open_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][0] < open_aabb[1][0] - 0.30,
        details=f"open={open_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
