from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel(width: float, height: float, thickness: float, radius: float, name: str):
    """Rounded-rectangle upholstered panel, with width along X and height along local Z."""
    panel = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        center=True,
    )
    # ExtrudeGeometry's profile is in XY and thickness is along Z; rotate it so
    # the cushion face is in XZ and the soft thickness runs front-to-back in Y.
    panel.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(panel, name)


def _rounded_cushion(width: float, depth: float, thickness: float, radius: float, name: str):
    """Horizontal rounded-square cushion, with soft thickness along local Z."""
    cushion = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        thickness,
        center=True,
    )
    return mesh_from_geometry(cushion, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.78, 0.74, 1.0))
    shadow = model.material("black_shadow", rgba=(0.025, 0.022, 0.02, 1.0))
    leather = model.material("warm_tan_upholstery", rgba=(0.55, 0.36, 0.20, 1.0))
    seam = model.material("dark_seam_piping", rgba=(0.12, 0.075, 0.04, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.275, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=chrome,
        name="round_base",
    )
    base.visual(
        Cylinder(radius=0.075, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=chrome,
        name="base_boss",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.655),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=chrome,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.6825)),
        material=chrome,
        name="upper_bearing_cup",
    )

    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.235, tube=0.014, radial_segments=18, tubular_segments=72),
            "foot_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=chrome,
        name="foot_ring",
    )
    for idx, (x, y, yaw) in enumerate(
        (
            (0.118, 0.0, math.pi / 2.0),
            (-0.118, 0.0, math.pi / 2.0),
            (0.0, 0.118, 0.0),
            (0.0, -0.118, 0.0),
        )
    ):
        if x:
            origin = Origin(xyz=(x, y, 0.315), rpy=(0.0, math.pi / 2.0, 0.0))
        else:
            origin = Origin(xyz=(x, y, 0.315), rpy=(math.pi / 2.0, 0.0, 0.0))
        base.visual(
            Cylinder(radius=0.010, length=0.250),
            origin=origin,
            material=chrome,
            name=f"ring_spoke_{idx}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.185, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=chrome,
        name="swivel_plate",
    )
    seat.visual(
        Box((0.405, 0.042, 0.028)),
        origin=Origin(xyz=(0.0, 0.205, 0.035)),
        material=chrome,
        name="rear_seat_rail",
    )
    seat.visual(
        Box((0.405, 0.042, 0.028)),
        origin=Origin(xyz=(0.0, -0.205, 0.035)),
        material=chrome,
        name="front_seat_rail",
    )
    seat.visual(
        Box((0.042, 0.405, 0.028)),
        origin=Origin(xyz=(0.205, 0.0, 0.035)),
        material=chrome,
        name="side_seat_rail_0",
    )
    seat.visual(
        Box((0.042, 0.405, 0.028)),
        origin=Origin(xyz=(-0.205, 0.0, 0.035)),
        material=chrome,
        name="side_seat_rail_1",
    )
    seat.visual(
        _rounded_cushion(0.465, 0.465, 0.085, 0.060, "seat_cushion"),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=leather,
        name="seat_cushion",
    )
    # Subtle welt strips embedded in the upholstery surface make the square seat
    # read as a padded cushion rather than a bare block.
    seat.visual(
        Box((0.405, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.196, 0.108)),
        material=seam,
        name="rear_seam",
    )
    seat.visual(
        Box((0.405, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.196, 0.108)),
        material=seam,
        name="front_seam",
    )
    seat.visual(
        Box((0.008, 0.405, 0.006)),
        origin=Origin(xyz=(0.196, 0.0, 0.108)),
        material=seam,
        name="side_seam_0",
    )
    seat.visual(
        Box((0.008, 0.405, 0.006)),
        origin=Origin(xyz=(-0.196, 0.0, 0.108)),
        material=seam,
        name="side_seam_1",
    )
    seat.visual(
        Box((0.055, 0.090, 0.018)),
        origin=Origin(xyz=(-0.145, 0.238, 0.052)),
        material=chrome,
        name="hinge_leaf_0",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(-0.145, 0.255, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="seat_hinge_barrel_0",
    )
    seat.visual(
        Box((0.055, 0.090, 0.018)),
        origin=Origin(xyz=(0.145, 0.238, 0.052)),
        material=chrome,
        name="hinge_leaf_1",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(0.145, 0.255, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="seat_hinge_barrel_1",
    )

    back = model.part("back")
    back.visual(
        Cylinder(radius=0.010, length=0.385),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="back_hinge_barrel",
    )
    for idx, x in enumerate((-0.070, 0.070)):
        back.visual(
            Box((0.022, 0.036, 0.230)),
            origin=Origin(xyz=(x, 0.035, 0.115), rpy=(-0.20, 0.0, 0.0)),
            material=chrome,
            name=f"back_upright_{idx}",
        )
    back.visual(
        _rounded_panel(0.425, 0.335, 0.060, 0.050, "back_cushion"),
        origin=Origin(xyz=(0.0, 0.080, 0.215), rpy=(-0.20, 0.0, 0.0)),
        material=leather,
        name="back_cushion",
    )
    back.visual(
        Box((0.330, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, 0.135, 0.250), rpy=(-0.20, 0.0, 0.0)),
        material=seam,
        name="back_seam",
    )
    back.visual(
        Box((0.330, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.020, 0.043), rpy=(-0.20, 0.0, 0.0)),
        material=shadow,
        name="lower_back_plate",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.04),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(0.0, 0.255, 0.083)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=0.45),
        motion_properties=MotionProperties(damping=0.5, friction=0.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    swivel = object_model.get_articulation("base_to_seat")
    hinge = object_model.get_articulation("seat_to_back")

    ctx.allow_overlap(
        seat,
        back,
        elem_a="seat_hinge_barrel_0",
        elem_b="back_hinge_barrel",
        reason="The back hinge pin is intentionally captured inside the seat hinge knuckle.",
    )
    ctx.allow_overlap(
        seat,
        back,
        elem_a="seat_hinge_barrel_1",
        elem_b="back_hinge_barrel",
        reason="The back hinge pin is intentionally captured inside the seat hinge knuckle.",
    )

    ctx.check(
        "seat has continuous swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "rear back has limited tilt hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and hinge.motion_limits.upper is not None
        and 0.35 <= hinge.motion_limits.upper <= 0.60,
        details=f"type={hinge.articulation_type}, limits={hinge.motion_limits}",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        min_gap=-0.001,
        max_gap=0.006,
        positive_elem="swivel_plate",
        negative_elem="upper_bearing_cup",
        name="seat swivel plate rests on pedestal cup",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_a="swivel_plate",
        elem_b="upper_bearing_cup",
        name="swivel plate is centered above pedestal",
    )
    ctx.expect_overlap(
        back,
        seat,
        axes="x",
        min_overlap=0.05,
        elem_a="back_hinge_barrel",
        elem_b="seat_hinge_barrel_0",
        name="back hinge shares the short rear hinge line",
    )
    ctx.expect_overlap(
        back,
        seat,
        axes="x",
        min_overlap=0.05,
        elem_a="back_hinge_barrel",
        elem_b="seat_hinge_barrel_1",
        name="back hinge spans both rear knuckles",
    )
    ctx.expect_within(
        back,
        seat,
        axes="yz",
        inner_elem="back_hinge_barrel",
        outer_elem="seat_hinge_barrel_0",
        margin=0.002,
        name="hinge pin is centered in rear knuckle",
    )

    rest_back = ctx.part_world_aabb(back)
    with ctx.pose({hinge: 0.45}):
        tilted_back = ctx.part_world_aabb(back)
    ctx.check(
        "positive back hinge tilts rearward",
        rest_back is not None
        and tilted_back is not None
        and tilted_back[1][1] > rest_back[1][1] + 0.020,
        details=f"rest={rest_back}, tilted={tilted_back}",
    )

    return ctx.report()


object_model = build_object_model()
