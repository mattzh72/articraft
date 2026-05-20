from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _slab_origin_between(p0: tuple[float, float], p1: tuple[float, float]) -> tuple[Origin, float]:
    """Return an Origin that points a Box local +X from p0 to p1 in the XZ plane."""
    dx = p1[0] - p0[0]
    dz = p1[1] - p0[1]
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    return Origin(xyz=((p0[0] + p1[0]) * 0.5, 0.0, (p0[1] + p1[1]) * 0.5), rpy=(0.0, pitch, 0.0)), length


def _add_sling_panel(part, *, name: str, p0: tuple[float, float], p1: tuple[float, float], width: float, thickness: float, material):
    origin, length = _slab_origin_between(p0, p1)
    part.visual(Box((length, width, thickness)), origin=origin, material=material, name=name)


def _add_y_tube(part, *, name: str, x: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zero_gravity_patio_lounge")

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    dark_hardware = model.material("black_hinge_hardware", color=(0.03, 0.035, 0.04, 1.0))
    fabric = model.material("woven_navy_sling", color=(0.05, 0.14, 0.28, 1.0))
    edge_webbing = model.material("dark_edge_webbing", color=(0.015, 0.025, 0.045, 1.0))
    rubber = model.material("black_rubber_feet", color=(0.01, 0.01, 0.01, 1.0))

    support = model.part("support")

    # Two continuous side loops form the rigid patio-chair support frame.
    for y, suffix in [(-0.44, "0"), (0.44, "1")]:
        side_points = [
            (-0.84, y, 0.055),
            (-0.58, y, 0.075),
            (-0.20, y, 0.56),
            (0.00, y, 0.55),
            (0.12, y, 0.60),
            (0.66, y, 0.34),
            (0.82, y, 0.06),
            (0.38, y, 0.045),
            (-0.32, y, 0.045),
        ]
        side_loop = wire_from_points(
            side_points,
            radius=0.024,
            radial_segments=16,
            closed_path=True,
            cap_ends=False,
            corner_mode="fillet",
            corner_radius=0.075,
            corner_segments=8,
        )
        support.visual(mesh_from_geometry(side_loop, f"side_loop_{suffix}"), material=aluminum, name=f"side_loop_{suffix}")

    _add_y_tube(support, name="rear_ground_bar", x=-0.70, z=0.06, length=0.92, radius=0.022, material=aluminum)
    _add_y_tube(support, name="front_ground_bar", x=0.74, z=0.06, length=0.92, radius=0.022, material=aluminum)
    _add_y_tube(support, name="front_stabilizer", x=0.66, z=0.34, length=0.90, radius=0.019, material=aluminum)
    _add_y_tube(support, name="recline_pin", x=0.0, z=0.55, length=0.96, radius=0.020, material=dark_hardware)
    for y, suffix in [(-0.44, "0"), (0.44, "1")]:
        support.visual(Box((0.22, 0.10, 0.18)), origin=Origin(xyz=(0.66, y, 0.34)), material=aluminum, name=f"front_stabilizer_socket_{suffix}")
        support.visual(Box((0.08, 0.08, 0.08)), origin=Origin(xyz=(0.00, y, 0.55)), material=dark_hardware, name=f"recline_boss_{suffix}")

    for x, name in [(-0.70, "rear_foot"), (0.74, "front_foot")]:
        support.visual(Box((0.18, 0.98, 0.035)), origin=Origin(xyz=(x, 0.0, 0.023)), material=rubber, name=name)

    seat_frame = model.part("seat_frame")

    for y, suffix in [(-0.35, "0"), (0.35, "1")]:
        rail_points = [
            (-0.72, y, 0.60),
            (-0.42, y, 0.34),
            (0.00, y, 0.00),
            (0.84, y, -0.08),
        ]
        rail = wire_from_points(
            rail_points,
            radius=0.021,
            radial_segments=16,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.045,
            corner_segments=8,
        )
        seat_frame.visual(mesh_from_geometry(rail, f"seat_side_rail_{suffix}"), material=aluminum, name=f"side_rail_{suffix}")

    _add_y_tube(seat_frame, name="recline_sleeve", x=0.0, z=0.0, length=0.74, radius=0.031, material=dark_hardware)
    _add_y_tube(seat_frame, name="back_top_bar", x=-0.72, z=0.60, length=0.74, radius=0.020, material=aluminum)
    _add_y_tube(seat_frame, name="lumbar_bar", x=-0.38, z=0.31, length=0.70, radius=0.017, material=aluminum)
    _add_y_tube(seat_frame, name="seat_front_bar", x=0.76, z=-0.08, length=0.74, radius=0.020, material=aluminum)
    _add_y_tube(seat_frame, name="foot_hinge_rod", x=0.84, z=-0.08, length=0.76, radius=0.017, material=dark_hardware)
    for y, suffix in [(-0.35, "0"), (0.35, "1")]:
        seat_frame.visual(Box((0.16, 0.08, 0.12)), origin=Origin(xyz=(0.02, y, 0.00)), material=dark_hardware, name=f"recline_knuckle_{suffix}")

    _add_sling_panel(seat_frame, name="back_sling", p0=(-0.66, 0.55), p1=(-0.05, 0.04), width=0.66, thickness=0.026, material=fabric)
    _add_sling_panel(seat_frame, name="seat_sling", p0=(0.12, -0.025), p1=(0.70, -0.075), width=0.66, thickness=0.026, material=fabric)
    _add_sling_panel(seat_frame, name="back_webbing", p0=(-0.70, 0.59), p1=(-0.09, 0.08), width=0.68, thickness=0.012, material=edge_webbing)

    footrest = model.part("footrest")

    for y, suffix in [(-0.33, "0"), (0.33, "1")]:
        foot_rail = wire_from_points(
            [
                (0.04, y, -0.005),
                (0.30, y, -0.055),
                (0.68, y, -0.12),
            ],
            radius=0.020,
            radial_segments=16,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=6,
        )
        footrest.visual(mesh_from_geometry(foot_rail, f"foot_side_rail_{suffix}"), material=aluminum, name=f"side_rail_{suffix}")

    _add_y_tube(footrest, name="hinge_sleeve", x=0.0, z=0.0, length=0.62, radius=0.027, material=dark_hardware)
    _add_y_tube(footrest, name="toe_bar", x=0.68, z=-0.12, length=0.70, radius=0.020, material=aluminum)
    _add_y_tube(footrest, name="mid_bar", x=0.34, z=-0.06, length=0.70, radius=0.016, material=aluminum)
    for y, suffix in [(-0.33, "0"), (0.33, "1")]:
        footrest.visual(Box((0.034, 0.07, 0.034)), origin=Origin(xyz=(0.035, y, -0.035)), material=dark_hardware, name=f"hinge_knuckle_{suffix}")
        footrest.visual(Box((0.07, 0.07, 0.05)), origin=Origin(xyz=(0.34, y, -0.06)), material=aluminum, name=f"mid_bar_socket_{suffix}")
    _add_sling_panel(footrest, name="leg_sling", p0=(0.04, -0.01), p1=(0.64, -0.115), width=0.64, thickness=0.025, material=fabric)

    model.articulation(
        "recline_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=seat_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-0.30, upper=0.62),
    )

    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=footrest,
        origin=Origin(xyz=(0.84, 0.0, -0.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.70, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    seat_frame = object_model.get_part("seat_frame")
    footrest = object_model.get_part("footrest")
    recline = object_model.get_articulation("recline_axis")
    foot_hinge = object_model.get_articulation("footrest_hinge")

    ctx.allow_overlap(
        support,
        seat_frame,
        elem_a="recline_pin",
        elem_b="recline_sleeve",
        reason="The main seat-back frame sleeve is intentionally captured around the support-frame recline pin.",
    )
    for knuckle in ("recline_knuckle_0", "recline_knuckle_1"):
        ctx.allow_overlap(
            support,
            seat_frame,
            elem_a="recline_pin",
            elem_b=knuckle,
            reason="The side recline knuckle wraps the transverse support pin as part of the exposed hinge barrel.",
        )
    ctx.allow_overlap(
        seat_frame,
        footrest,
        elem_a="foot_hinge_rod",
        elem_b="hinge_sleeve",
        reason="The lower-leg panel hinge sleeve is intentionally nested on the seat-front hinge rod.",
    )

    ctx.expect_overlap(
        support,
        seat_frame,
        axes="y",
        elem_a="recline_pin",
        elem_b="recline_sleeve",
        min_overlap=0.55,
        name="recline sleeve spans the transverse support pin",
    )
    for knuckle in ("recline_knuckle_0", "recline_knuckle_1"):
        ctx.expect_overlap(
            support,
            seat_frame,
            axes="y",
            elem_a="recline_pin",
            elem_b=knuckle,
            min_overlap=0.04,
            name=f"{knuckle} captures the recline pin",
        )
    ctx.expect_overlap(
        seat_frame,
        footrest,
        axes="y",
        elem_a="foot_hinge_rod",
        elem_b="hinge_sleeve",
        min_overlap=0.40,
        name="footrest sleeve spans the front hinge rod",
    )
    ctx.expect_gap(
        footrest,
        seat_frame,
        axis="x",
        positive_elem="leg_sling",
        negative_elem="seat_front_bar",
        min_gap=0.005,
        max_gap=0.130,
        name="footrest panel sits directly ahead of the seat frame",
    )

    ctx.check(
        "two independent transverse revolute axes",
        recline.axis == (0.0, -1.0, 0.0)
        and foot_hinge.axis == (0.0, -1.0, 0.0)
        and recline.motion_limits is not None
        and foot_hinge.motion_limits is not None,
        details=f"recline_axis={recline.axis}, footrest_hinge={foot_hinge.axis}",
    )

    rest_toe = ctx.part_element_world_aabb(footrest, elem="toe_bar")
    with ctx.pose({foot_hinge: 0.80}):
        raised_toe = ctx.part_element_world_aabb(footrest, elem="toe_bar")
    ctx.check(
        "footrest hinge raises the lower-leg panel",
        rest_toe is not None
        and raised_toe is not None
        and raised_toe[1][2] > rest_toe[1][2] + 0.30,
        details=f"rest_toe={rest_toe}, raised_toe={raised_toe}",
    )

    rest_back = ctx.part_element_world_aabb(seat_frame, elem="back_top_bar")
    rest_front = ctx.part_element_world_aabb(seat_frame, elem="seat_front_bar")
    with ctx.pose({recline: 0.55}):
        reclined_back = ctx.part_element_world_aabb(seat_frame, elem="back_top_bar")
        reclined_front = ctx.part_element_world_aabb(seat_frame, elem="seat_front_bar")
    ctx.check(
        "recline axis lowers the back and lifts the knee end",
        rest_back is not None
        and reclined_back is not None
        and rest_front is not None
        and reclined_front is not None
        and reclined_back[0][2] < rest_back[0][2] - 0.20
        and reclined_front[1][2] > rest_front[1][2] + 0.25,
        details=f"rest_back={rest_back}, reclined_back={reclined_back}, rest_front={rest_front}, reclined_front={reclined_front}",
    )

    return ctx.report()


object_model = build_object_model()
