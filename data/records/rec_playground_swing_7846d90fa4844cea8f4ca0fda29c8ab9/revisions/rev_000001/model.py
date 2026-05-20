from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


LINK_LENGTH = 1.24
SIDE_SPACING_X = 1.76
LINK_SPACING_Y = 0.72
UPPER_Z = 2.08
LOWER_Z = UPPER_Z - LINK_LENGTH
LEFT_X = -0.88
RIGHT_X = 0.88
FRONT_Y = -0.36
REAR_Y = 0.36


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material | str,
    name: str,
) -> None:
    """Add a cylinder whose local +Z spans start to end in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder {name}")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _x_cylinder(
    part,
    xyz: tuple[float, float, float],
    radius: float,
    length: float,
    material: Material | str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(
    part,
    xyz: tuple[float, float, float],
    radius: float,
    length: float,
    material: Material | str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_glider_swing")

    frame_green = model.material("powder_coated_green", rgba=(0.05, 0.36, 0.14, 1.0))
    dark_steel = model.material("dark_pivot_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    galvanized = model.material("galvanized_pin", rgba=(0.63, 0.66, 0.62, 1.0))
    warm_wood = model.material("sealed_wood_slats", rgba=(0.72, 0.44, 0.18, 1.0))
    black_rubber = model.material("black_foot_caps", rgba=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("frame")

    # A freestanding playground frame: A-frame legs, ground skids, an overhead
    # cross tube, and fore-aft pivot rails so the two links on each side have
    # separate upper pivots instead of hanging from one point.
    _x_cylinder(frame, (0.0, 0.0, 2.20), 0.045, 2.55, frame_green, "top_crossbar")
    _y_cylinder(frame, (LEFT_X, 0.0, UPPER_Z), 0.030, 1.02, frame_green, "left_pivot_rail")
    _y_cylinder(frame, (RIGHT_X, 0.0, UPPER_Z), 0.030, 1.02, frame_green, "right_pivot_rail")

    _cylinder_between(frame, (LEFT_X, 0.0, UPPER_Z), (LEFT_X, 0.0, 2.20), 0.026, frame_green, "left_rail_drop")
    _cylinder_between(frame, (RIGHT_X, 0.0, UPPER_Z), (RIGHT_X, 0.0, 2.20), 0.026, frame_green, "right_rail_drop")

    _y_cylinder(frame, (-1.18, 0.0, 0.055), 0.035, 1.92, frame_green, "left_ground_skid")
    _y_cylinder(frame, (1.18, 0.0, 0.055), 0.035, 1.92, frame_green, "right_ground_skid")
    _x_cylinder(frame, (0.0, FRONT_Y - 0.55, 0.085), 0.026, 2.42, frame_green, "front_base_tie")
    _x_cylinder(frame, (0.0, REAR_Y + 0.55, 0.085), 0.026, 2.42, frame_green, "rear_base_tie")

    for idx, (x0, y0, name) in enumerate(
        (
            (-1.18, FRONT_Y - 0.55, "left_front_leg"),
            (-1.18, REAR_Y + 0.55, "left_rear_leg"),
            (1.18, FRONT_Y - 0.55, "right_front_leg"),
            (1.18, REAR_Y + 0.55, "right_rear_leg"),
        )
    ):
        top_x = -1.03 if x0 < 0.0 else 1.03
        _cylinder_between(frame, (x0, y0, 0.08), (top_x, 0.0, 2.20), 0.038, frame_green, name)
        frame.visual(
            Sphere(radius=0.050),
            origin=Origin(xyz=(x0, y0, 0.045)),
            material=black_rubber,
            name=f"foot_cap_{idx}",
        )

    # Short pivot pins run along the bench width and carry the outboard link
    # plates.  The separate pins make the upper revolute pivots legible.
    for x, side in ((LEFT_X, "left"), (RIGHT_X, "right")):
        for y, fore_aft in ((FRONT_Y, "front"), (REAR_Y, "rear")):
            _x_cylinder(
                frame,
                (x, y, UPPER_Z),
                0.020,
                0.34,
                galvanized,
                f"{side}_{fore_aft}_upper_pin",
            )
            frame.visual(
                Box((0.09, 0.035, 0.13)),
                origin=Origin(xyz=(x, y, UPPER_Z + 0.035)),
                material=dark_steel,
                name=f"{side}_{fore_aft}_hanger_plate",
            )

    bench = model.part("bench")
    # The bench part frame is the left-front lower pivot.  All bench geometry is
    # offset from that pivot so the lower joint can keep the seat level while the
    # four parallel links swing.
    seat_center_x = SIDE_SPACING_X * 0.5
    for i, y in enumerate((0.11, 0.25, 0.39, 0.53)):
        bench.visual(
            Box((1.62, 0.095, 0.045)),
            origin=Origin(xyz=(seat_center_x, y, -0.315)),
            material=warm_wood,
            name=f"seat_slats_{i}",
        )

    # Under-seat rails overlap every slat and tie them into one rigid bench.
    for x, name in ((0.14, "left_seat_rail"), (seat_center_x, "center_seat_rail"), (1.62, "right_seat_rail")):
        bench.visual(
            Box((0.050, 0.60, 0.055)),
            origin=Origin(xyz=(x, 0.32, -0.345)),
            material=dark_steel,
            name=name,
        )

    for i, (z, height) in enumerate(((-0.145, 0.085), (0.015, 0.085), (0.175, 0.085))):
        bench.visual(
            Box((1.62, 0.055, height)),
            origin=Origin(xyz=(seat_center_x, 0.675, z)),
            material=warm_wood,
            name=f"back_slats_{i}",
        )

    for x, side in ((-0.035, "left"), (SIDE_SPACING_X + 0.035, "right")):
        bench.visual(
            Box((0.055, 0.80, 0.600)),
            origin=Origin(xyz=(x, 0.36, -0.115)),
            material=dark_steel,
            name=f"{side}_side_bracket",
        )
        bench.visual(
            Box((0.070, 0.065, 0.62)),
            origin=Origin(xyz=(x, 0.675, -0.050)),
            material=dark_steel,
            name=f"{side}_back_post",
        )
        bench.visual(
            Box((0.060, 0.72, 0.040)),
            origin=Origin(xyz=(x, 0.34, -0.010)),
            material=dark_steel,
            name=f"{side}_arm_rail",
        )

    # Inboard cheek plates bridge the side brackets to the wooden slats.  They
    # keep the bench authored as one supported assembly while leaving the
    # outboard lower link eyes clear to rotate on their pins.
    for x, side in ((0.040, "left"), (SIDE_SPACING_X - 0.040, "right")):
        bench.visual(
            Box((0.140, 0.78, 0.560)),
            origin=Origin(xyz=(x, 0.36, -0.135)),
            material=dark_steel,
            name=f"{side}_cheek_plate",
        )

    # Lower pivot pins are part of the bench side brackets.  They are slightly
    # smaller than the link eyes, like a real pin running through a bushing.
    for x_local, side in ((0.0, "left"), (SIDE_SPACING_X, "right")):
        for y, fore_aft in ((0.0, "front"), (LINK_SPACING_Y, "rear")):
            _x_cylinder(
                bench,
                (x_local, y, 0.0),
                0.020,
                0.34,
                galvanized,
                f"{side}_{fore_aft}_lower_pin",
            )

    def add_side_link(name: str, outboard_x: float, material: Material) -> None:
        link = model.part(name)
        link.visual(
            Box((0.038, 0.035, LINK_LENGTH - 0.10)),
            origin=Origin(xyz=(outboard_x, 0.0, -LINK_LENGTH * 0.5)),
            material=material,
            name="strap",
        )
        _x_cylinder(link, (outboard_x, 0.0, 0.0), 0.060, 0.052, material, "upper_eye")
        _x_cylinder(link, (outboard_x, 0.0, -LINK_LENGTH), 0.060, 0.052, material, "lower_eye")

    add_side_link("left_front_link", -0.12, dark_steel)
    add_side_link("left_rear_link", -0.12, dark_steel)
    add_side_link("right_front_link", 0.12, dark_steel)
    add_side_link("right_rear_link", 0.12, dark_steel)

    swing_limits = MotionLimits(effort=60.0, velocity=1.6, lower=-0.36, upper=0.36)
    master = model.articulation(
        "frame_to_left_front_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_front_link",
        origin=Origin(xyz=(LEFT_X, FRONT_Y, UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=swing_limits,
    )

    for joint_name, child, x, y in (
        ("frame_to_left_rear_link", "left_rear_link", LEFT_X, REAR_Y),
        ("frame_to_right_front_link", "right_front_link", RIGHT_X, FRONT_Y),
        ("frame_to_right_rear_link", "right_rear_link", RIGHT_X, REAR_Y),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=child,
            origin=Origin(xyz=(x, y, UPPER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=swing_limits,
            mimic=Mimic(master.name, multiplier=1.0),
        )

    model.articulation(
        "left_front_link_to_bench",
        ArticulationType.REVOLUTE,
        parent="left_front_link",
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=swing_limits,
        mimic=Mimic(master.name, multiplier=-1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bench = object_model.get_part("bench")
    master = object_model.get_articulation("frame_to_left_front_link")

    for side in ("left", "right"):
        for fore_aft in ("front", "rear"):
            link = object_model.get_part(f"{side}_{fore_aft}_link")
            ctx.allow_overlap(
                frame,
                link,
                elem_a=f"{side}_{fore_aft}_upper_pin",
                elem_b="upper_eye",
                reason="The upper pivot pin is intentionally captured inside the link eye bushing.",
            )
            ctx.expect_within(
                frame,
                link,
                axes="yz",
                inner_elem=f"{side}_{fore_aft}_upper_pin",
                outer_elem="upper_eye",
                margin=0.002,
                name=f"{side} {fore_aft} upper pin sits in eye",
            )
            ctx.expect_overlap(
                frame,
                link,
                axes="x",
                elem_a=f"{side}_{fore_aft}_upper_pin",
                elem_b="upper_eye",
                min_overlap=0.020,
                name=f"{side} {fore_aft} upper pin has bearing length",
            )

            ctx.allow_overlap(
                bench,
                link,
                elem_a=f"{side}_{fore_aft}_lower_pin",
                elem_b="lower_eye",
                reason="The bench lower pivot pin is intentionally captured inside the link eye bushing.",
            )
            ctx.expect_within(
                bench,
                link,
                axes="yz",
                inner_elem=f"{side}_{fore_aft}_lower_pin",
                outer_elem="lower_eye",
                margin=0.002,
                name=f"{side} {fore_aft} lower pin sits in eye",
            )
            ctx.expect_overlap(
                bench,
                link,
                axes="x",
                elem_a=f"{side}_{fore_aft}_lower_pin",
                elem_b="lower_eye",
                min_overlap=0.020,
                name=f"{side} {fore_aft} lower pin has bearing length",
            )

    rest_pos = ctx.part_world_position(bench)
    rest_front = ctx.part_element_world_aabb(bench, elem="seat_slats_0")
    rest_rear = ctx.part_element_world_aabb(bench, elem="seat_slats_3")

    with ctx.pose({master: 0.32}):
        swung_pos = ctx.part_world_position(bench)
        swung_front = ctx.part_element_world_aabb(bench, elem="seat_slats_0")
        swung_rear = ctx.part_element_world_aabb(bench, elem="seat_slats_3")
        for side in ("left", "right"):
            for fore_aft in ("front", "rear"):
                link = object_model.get_part(f"{side}_{fore_aft}_link")
                ctx.expect_within(
                    bench,
                    link,
                    axes="yz",
                    inner_elem=f"{side}_{fore_aft}_lower_pin",
                    outer_elem="lower_eye",
                    margin=0.006,
                    name=f"{side} {fore_aft} lower pivot stays aligned while swinging",
                )

    def _mid_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "bench swings fore and aft",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )
    ctx.check(
        "seat is level at rest",
        _mid_z(rest_front) is not None
        and _mid_z(rest_rear) is not None
        and abs(_mid_z(rest_front) - _mid_z(rest_rear)) < 0.004,
        details=f"front={rest_front}, rear={rest_rear}",
    )
    ctx.check(
        "seat stays level while swinging",
        _mid_z(swung_front) is not None
        and _mid_z(swung_rear) is not None
        and abs(_mid_z(swung_front) - _mid_z(swung_rear)) < 0.006,
        details=f"front={swung_front}, rear={swung_rear}",
    )

    return ctx.report()


object_model = build_object_model()
