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


def _rounded_box(size: tuple[float, float, float], radius: float, edges: str):
    """Small CadQuery helper for rounded manufactured panels."""

    return cq.Workplane("XY").box(*size).edges(edges).fillet(radius)


def _tube_angle_xz(dx: float, dz: float) -> float:
    """Return the Y rotation that aligns a cylinder's local Z axis to (dx, 0, dz)."""

    return math.atan2(dx, dz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_chair")

    plastic = model.material("warm_gray_plastic", color=(0.56, 0.54, 0.50, 1.0))
    dark_plastic = model.material("charcoal_grip_plastic", color=(0.06, 0.06, 0.055, 1.0))
    steel = model.material("brushed_steel", color=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("black_rubber", color=(0.012, 0.011, 0.010, 1.0))

    seat_z = 0.455
    seat_size = (0.42, 0.44, 0.035)
    seat_bottom = seat_z - seat_size[2] / 2.0
    front_hinge_x = 0.195
    rear_hinge_x = -0.195
    leg_hinge_z = seat_bottom - 0.012
    back_hinge_x = -0.228
    back_hinge_z = seat_z + 0.003

    seat_panel_mesh = mesh_from_cadquery(
        _rounded_box(seat_size, 0.030, "|Z"),
        "rounded_seat_panel",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    back_panel_mesh = mesh_from_cadquery(
        _rounded_box((0.026, 0.382, 0.235), 0.028, "|X"),
        "rounded_back_panel",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )

    seat = model.part("seat")
    seat.visual(
        seat_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, seat_z)),
        material=plastic,
        name="seat_panel",
    )
    # Seat pan ribs, underside pivot brackets, and exposed pins are one rigid
    # seat assembly. The boxes slightly enter the underside of the panel to read
    # as bolted/welded hardware rather than floating plates.
    for y in (-0.145, 0.145):
        seat.visual(
            Box((0.34, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, y, seat_bottom - 0.004)),
            material=dark_plastic,
            name=f"underside_rib_{'neg' if y < 0 else 'pos'}",
        )
    for hinge_name, x in (("front", front_hinge_x), ("rear", rear_hinge_x)):
        seat.visual(
            Cylinder(radius=0.006, length=0.455),
            origin=Origin(xyz=(x, 0.0, leg_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{hinge_name}_hinge_pin",
        )
        for y in (-0.200, 0.200):
            seat.visual(
                Box((0.045, 0.018, 0.040)),
                origin=Origin(xyz=(x, y, leg_hinge_z + 0.010)),
                material=steel,
                name=f"{hinge_name}_hinge_ear_{'neg' if y < 0 else 'pos'}",
            )
    seat.visual(
        Cylinder(radius=0.006, length=0.455),
        origin=Origin(xyz=(back_hinge_x, 0.0, back_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="back_hinge_pin",
    )
    for y in (-0.205, 0.205):
        seat.visual(
            Box((0.048, 0.018, 0.060)),
            origin=Origin(xyz=(back_hinge_x, y, back_hinge_z - 0.020)),
            material=steel,
            name=f"back_hinge_ear_{'neg' if y < 0 else 'pos'}",
        )

    front_leg = model.part("front_leg")
    front_leg.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    for y in (-0.145, 0.145):
        front_leg.visual(
            Box((0.040, 0.028, 0.028)),
            origin=Origin(xyz=(0.002, y, -0.024)),
            material=steel,
            name=f"hinge_lug_{'neg' if y < 0 else 'pos'}",
        )
    front_dx = 0.078
    leg_top_z = -0.020
    leg_bottom_z = -0.420
    leg_dz = leg_bottom_z - leg_top_z
    leg_len = math.hypot(front_dx, leg_dz)
    leg_angle = _tube_angle_xz(front_dx, leg_dz)
    for y in (-0.145, 0.145):
        front_leg.visual(
            Cylinder(radius=0.011, length=leg_len),
            origin=Origin(
                xyz=(front_dx / 2.0, y, (leg_top_z + leg_bottom_z) / 2.0),
                rpy=(0.0, leg_angle, 0.0),
            ),
            material=steel,
            name=f"leg_tube_{'neg' if y < 0 else 'pos'}",
        )
    front_leg.visual(
        Cylinder(radius=0.010, length=0.330),
        origin=Origin(xyz=(front_dx, 0.0, leg_bottom_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="floor_crossbar",
    )
    for y in (-0.145, 0.145):
        front_leg.visual(
            Box((0.085, 0.034, 0.018)),
            origin=Origin(xyz=(front_dx, y, leg_bottom_z - 0.008)),
            material=rubber,
            name=f"foot_pad_{'neg' if y < 0 else 'pos'}",
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    for y in (-0.145, 0.145):
        rear_leg.visual(
            Box((0.040, 0.028, 0.028)),
            origin=Origin(xyz=(-0.002, y, -0.024)),
            material=steel,
            name=f"hinge_lug_{'neg' if y < 0 else 'pos'}",
        )
    rear_dx = -0.078
    rear_angle = _tube_angle_xz(rear_dx, leg_dz)
    for y in (-0.145, 0.145):
        rear_leg.visual(
            Cylinder(radius=0.011, length=leg_len),
            origin=Origin(
                xyz=(rear_dx / 2.0, y, (leg_top_z + leg_bottom_z) / 2.0),
                rpy=(0.0, rear_angle, 0.0),
            ),
            material=steel,
            name=f"leg_tube_{'neg' if y < 0 else 'pos'}",
        )
    rear_leg.visual(
        Cylinder(radius=0.010, length=0.330),
        origin=Origin(xyz=(rear_dx, 0.0, leg_bottom_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="floor_crossbar",
    )
    for y in (-0.145, 0.145):
        rear_leg.visual(
            Box((0.085, 0.034, 0.018)),
            origin=Origin(xyz=(rear_dx, y, leg_bottom_z - 0.008)),
            material=rubber,
            name=f"foot_pad_{'neg' if y < 0 else 'pos'}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    for y in (-0.158, 0.158):
        backrest.visual(
            Box((0.036, 0.026, 0.028)),
            origin=Origin(xyz=(-0.003, y, 0.024)),
            material=steel,
            name=f"hinge_lug_{'neg' if y < 0 else 'pos'}",
        )
    back_dx, back_dz = -0.115, 0.505
    back_base_z = 0.020
    upright_dz = back_dz - back_base_z
    back_len = math.hypot(back_dx, upright_dz)
    back_angle = _tube_angle_xz(back_dx, upright_dz)
    for y in (-0.158, 0.158):
        backrest.visual(
            Cylinder(radius=0.010, length=back_len),
            origin=Origin(
                xyz=(back_dx / 2.0, y, (back_base_z + back_dz) / 2.0),
                rpy=(0.0, back_angle, 0.0),
            ),
            material=steel,
            name=f"upright_{'neg' if y < 0 else 'pos'}",
        )
    backrest.visual(
        back_panel_mesh,
        origin=Origin(xyz=(back_dx * 0.66, 0.0, back_dz * 0.66), rpy=(0.0, back_angle, 0.0)),
        material=plastic,
        name="back_panel",
    )
    backrest.visual(
        Cylinder(radius=0.009, length=0.345),
        origin=Origin(xyz=(back_dx * 0.94, 0.0, back_dz * 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="top_crossbar",
    )

    model.articulation(
        "seat_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=front_leg,
        origin=Origin(xyz=(front_hinge_x, 0.0, leg_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "seat_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=rear_leg,
        origin=Origin(xyz=(rear_hinge_x, 0.0, leg_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(back_hinge_x, 0.0, back_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    front_leg = object_model.get_part("front_leg")
    rear_leg = object_model.get_part("rear_leg")
    backrest = object_model.get_part("backrest")
    front_joint = object_model.get_articulation("seat_to_front_leg")
    rear_joint = object_model.get_articulation("seat_to_rear_leg")
    back_joint = object_model.get_articulation("seat_to_backrest")

    for child, pin_name, reason in (
        (front_leg, "front_hinge_pin", "front leg sleeve intentionally rotates around the captured steel hinge pin"),
        (rear_leg, "rear_hinge_pin", "rear leg sleeve intentionally rotates around the captured steel hinge pin"),
        (backrest, "back_hinge_pin", "backrest sleeve intentionally rotates around the captured steel hinge pin"),
    ):
        ctx.allow_overlap(seat, child, elem_a=pin_name, elem_b="hinge_sleeve", reason=reason)
        ctx.expect_within(
            seat,
            child,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="hinge_sleeve",
            margin=0.0,
            name=f"{pin_name} is radially captured by sleeve",
        )
        ctx.expect_overlap(
            seat,
            child,
            axes="y",
            elem_a=pin_name,
            elem_b="hinge_sleeve",
            min_overlap=0.30,
            name=f"{pin_name} passes through sleeve width",
        )

    # Open sitting pose: legs reach below the seat and the backrest rises behind it.
    front_open = ctx.part_world_aabb(front_leg)
    rear_open = ctx.part_world_aabb(rear_leg)
    back_open = ctx.part_world_aabb(backrest)
    seat_open = ctx.part_world_aabb(seat)
    ctx.check(
        "open legs reach floor height",
        front_open is not None
        and rear_open is not None
        and seat_open is not None
        and front_open[0][2] < 0.02
        and rear_open[0][2] < 0.02
        and seat_open[0][2] > 0.38,
        details=f"front={front_open}, rear={rear_open}, seat={seat_open}",
    )
    ctx.check(
        "backrest rises above seat",
        back_open is not None and seat_open is not None and back_open[1][2] > seat_open[1][2] + 0.38,
        details=f"back={back_open}, seat={seat_open}",
    )

    # Folded pose: the two leg frames swing inward and upward while the backrest
    # tips forward toward the seat, proving the primary user-facing mechanisms.
    with ctx.pose({front_joint: 1.35, rear_joint: 1.35, back_joint: 1.55}):
        front_fold = ctx.part_world_aabb(front_leg)
        rear_fold = ctx.part_world_aabb(rear_leg)
        back_fold = ctx.part_world_aabb(backrest)
    ctx.check(
        "front leg folds under seat",
        front_open is not None
        and front_fold is not None
        and front_fold[0][2] > front_open[0][2] + 0.18
        and front_fold[0][0] < front_open[0][0] - 0.20,
        details=f"open={front_open}, folded={front_fold}",
    )
    ctx.check(
        "rear leg folds under seat",
        rear_open is not None
        and rear_fold is not None
        and rear_fold[0][2] > rear_open[0][2] + 0.18
        and rear_fold[1][0] > rear_open[1][0] + 0.20,
        details=f"open={rear_open}, folded={rear_fold}",
    )
    ctx.check(
        "backrest folds forward",
        back_open is not None
        and back_fold is not None
        and back_fold[1][0] > back_open[1][0] + 0.35
        and back_fold[1][2] < back_open[1][2] - 0.28,
        details=f"open={back_open}, folded={back_fold}",
    )

    return ctx.report()


object_model = build_object_model()
