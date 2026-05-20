from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 1.35
RIM_RADIUS = 0.84
SEAT_PIVOT_RADIUS = 0.73
WHEEL_HALF_DEPTH = 0.13
HUB_ARM_START = 0.082


def _cylinder_between_xz(part, name, start, end, radius, material):
    """Add a cylinder whose endpoints share the same local Y coordinate."""
    x0, y0, z0 = start
    x1, y1, z1 = end
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _cylinder_x(part, name, center, length, radius, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, name, center, length, radius, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _seat_material(index: int) -> str:
    return ("seat_blue", "seat_yellow", "seat_red")[index % 3]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_observation_wheel")

    model.material("painted_steel", color=(0.58, 0.58, 0.55, 1.0))
    model.material("dark_rubber", color=(0.05, 0.05, 0.05, 1.0))
    model.material("wheel_red", color=(0.78, 0.05, 0.04, 1.0))
    model.material("hub_yellow", color=(0.95, 0.72, 0.10, 1.0))
    model.material("seat_blue", color=(0.05, 0.28, 0.90, 1.0))
    model.material("seat_yellow", color=(0.95, 0.78, 0.07, 1.0))
    model.material("seat_red", color=(0.85, 0.08, 0.06, 1.0))

    frame = model.part("frame")
    # Two side A-frames with horizontal foot rails and cross ties.
    for side_index, y in enumerate((-0.32, 0.32)):
        side_name = f"side_{side_index}"
        _cylinder_between_xz(
            frame,
            f"{side_name}_leg_0",
            (-0.68, y, 0.06),
            (0.0, y, AXLE_Z),
            0.026,
            "painted_steel",
        )
        _cylinder_between_xz(
            frame,
            f"{side_name}_leg_1",
            (0.68, y, 0.06),
            (0.0, y, AXLE_Z),
            0.026,
            "painted_steel",
        )
        _cylinder_x(
            frame,
            f"{side_name}_foot",
            (0.0, y, 0.055),
            1.55,
            0.025,
            "painted_steel",
        )
        _cylinder_x(
            frame,
            f"{side_name}_brace",
            (0.0, y, 0.48),
            0.96,
            0.017,
            "painted_steel",
        )
        frame.visual(
            Cylinder(radius=0.065, length=0.075),
            origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_rubber",
            name=f"{side_name}_bearing",
        )

    _cylinder_y(frame, "front_tie", (-0.68, 0.0, 0.055), 0.68, 0.022, "painted_steel")
    _cylinder_y(frame, "rear_tie", (0.68, 0.0, 0.055), 0.68, 0.022, "painted_steel")
    frame.visual(
        Cylinder(radius=0.034, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="painted_steel",
        name="axle",
    )

    wheel = model.part("wheel")
    rim_mesh = mesh_from_geometry(
        TorusGeometry(RIM_RADIUS, 0.014, radial_segments=18, tubular_segments=96),
        "observation_wheel_rim",
    )
    for side_index, y in enumerate((-WHEEL_HALF_DEPTH, WHEEL_HALF_DEPTH)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="wheel_red",
            name=f"rim_{side_index}",
        )
        for arm_index in range(6):
            theta = 2.0 * math.pi * arm_index / 6.0
            x = RIM_RADIUS * math.sin(theta)
            z = RIM_RADIUS * math.cos(theta)
            inner_x = HUB_ARM_START * math.sin(theta)
            inner_z = HUB_ARM_START * math.cos(theta)
            _cylinder_between_xz(
                wheel,
                f"arm_{side_index}_{arm_index}",
                (inner_x, y, inner_z),
                (x, y, z),
                0.011,
                "wheel_red",
            )

    for arm_index in range(6):
        theta = 2.0 * math.pi * arm_index / 6.0 + math.pi / 6.0
        _cylinder_y(
            wheel,
            f"rim_spacer_{arm_index}",
            (RIM_RADIUS * math.sin(theta), 0.0, RIM_RADIUS * math.cos(theta)),
            2.0 * WHEEL_HALF_DEPTH + 0.030,
            0.010,
            "wheel_red",
        )

    wheel.visual(
        Cylinder(radius=0.066, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hub_yellow",
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.092, length=0.040),
        origin=Origin(xyz=(0.0, -WHEEL_HALF_DEPTH, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hub_yellow",
        name="hub_cap_0",
    )
    wheel.visual(
        Cylinder(radius=0.092, length=0.040),
        origin=Origin(xyz=(0.0, WHEEL_HALF_DEPTH, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hub_yellow",
        name="hub_cap_1",
    )

    wheel_joint = model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6),
    )

    for seat_index in range(6):
        theta = 2.0 * math.pi * seat_index / 6.0
        pivot_x = SEAT_PIVOT_RADIUS * math.sin(theta)
        pivot_z = SEAT_PIVOT_RADIUS * math.cos(theta)

        seat = model.part(f"seat_{seat_index}")
        seat_color = _seat_material(seat_index)
        # Hanger yoke: a short pin and two vertical rods, visibly suspended below
        # the radial arm while fitting between the two side rims.
        _cylinder_y(seat, "hanger_pin", (0.0, 0.0, 0.0), 0.242, 0.011, "painted_steel")
        seat.visual(
            Cylinder(radius=0.0075, length=0.21),
            origin=Origin(xyz=(0.0, -0.072, -0.105)),
            material="painted_steel",
            name="hanger_rod_0",
        )
        seat.visual(
            Cylinder(radius=0.0075, length=0.21),
            origin=Origin(xyz=(0.0, 0.072, -0.105)),
            material="painted_steel",
            name="hanger_rod_1",
        )
        seat.visual(
            Cylinder(radius=0.008, length=0.17),
            origin=Origin(xyz=(0.0, 0.0, -0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="painted_steel",
            name="lower_hanger_bar",
        )

        # Open bucket: a floor, high back, side cheeks, and a low front lip.
        seat.visual(
            Box((0.24, 0.19, 0.026)),
            origin=Origin(xyz=(0.015, 0.0, -0.342)),
            material=seat_color,
            name="floor",
        )
        seat.visual(
            Box((0.028, 0.19, 0.19)),
            origin=Origin(xyz=(-0.118, 0.0, -0.258)),
            material=seat_color,
            name="back",
        )
        seat.visual(
            Box((0.24, 0.026, 0.145)),
            origin=Origin(xyz=(0.015, -0.095, -0.278)),
            material=seat_color,
            name="side_0",
        )
        seat.visual(
            Box((0.24, 0.026, 0.145)),
            origin=Origin(xyz=(0.015, 0.095, -0.278)),
            material=seat_color,
            name="side_1",
        )
        seat.visual(
            Box((0.026, 0.19, 0.075)),
            origin=Origin(xyz=(0.148, 0.0, -0.318)),
            material=seat_color,
            name="front_lip",
        )

        model.articulation(
            f"wheel_to_seat_{seat_index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=seat,
            origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.2),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    seat_0 = object_model.get_part("seat_0")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle",
        elem_b="hub_barrel",
        reason="The fixed steel axle is intentionally captured inside the rotating hub bearing.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle",
        elem_b="hub_cap_0",
        reason="The axle passes through the near hub cap as a bearing shaft.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle",
        elem_b="hub_cap_1",
        reason="The axle passes through the far hub cap as a bearing shaft.",
    )
    ctx.expect_within(
        frame,
        wheel,
        axes="xz",
        inner_elem="axle",
        outer_elem="hub_barrel",
        margin=0.001,
        name="axle centered in wheel hub",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="y",
        elem_a="axle",
        elem_b="hub_barrel",
        min_overlap=0.28,
        name="hub retains axle through its width",
    )
    ctx.expect_contact(
        seat_0,
        seat_0,
        elem_a="hanger_rod_0",
        elem_b="lower_hanger_bar",
        contact_tol=0.002,
        name="seat hanger rod reaches lower bar",
    )

    with ctx.pose({wheel_joint: math.pi / 2.0}):
        floor_aabb = ctx.part_element_world_aabb(seat_0, elem="floor")
        pivot = ctx.part_world_position(seat_0)
        if floor_aabb is None or pivot is None:
            ctx.fail("mimicked seat stays upright", "seat floor or pivot position was unavailable")
        else:
            floor_z_span = floor_aabb[1][2] - floor_aabb[0][2]
            drop = pivot[2] - floor_aabb[1][2]
            ctx.check(
                "mimicked seat stays upright",
                floor_z_span < 0.040 and drop > 0.25,
                details=f"floor_z_span={floor_z_span:.4f}, pivot_to_floor_top={drop:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
