from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _cylinder_origin_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cylinder endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    origin, length = _cylinder_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="park_a_frame_swing")

    frame_green = Material("powder_coated_green", rgba=(0.05, 0.31, 0.20, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.68, 0.70, 0.67, 1.0))
    dark_rubber = Material("black_molded_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    concrete = Material("weathered_concrete", rgba=(0.55, 0.52, 0.45, 1.0))
    bolt_dark = Material("dark_bolt_heads", rgba=(0.06, 0.065, 0.06, 1.0))

    frame = model.part("frame")

    # Permanent outdoor A-frame: four splayed steel legs on small concrete pads,
    # a thick top beam, spreader bars, and realistic swing hanger eyes.
    frame.visual(
        Cylinder(radius=0.065, length=3.55),
        origin=Origin(xyz=(0.0, 0.0, 2.45), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_green,
        name="top_beam",
    )

    leg_tops = [(-1.55, 0.0, 2.39), (1.55, 0.0, 2.39)]
    foot_y = 0.84
    for side, x in enumerate((-1.55, 1.55)):
        for fore_index, y in enumerate((foot_y, -foot_y)):
            idx = side * 2 + fore_index
            _add_tube_between(
                frame,
                (x, 0.0, 2.39),
                (x, y, 0.06),
                radius=0.055,
                material=frame_green,
                name=f"leg_{idx}",
            )
            frame.visual(
                Box((0.42, 0.30, 0.06)),
                origin=Origin(xyz=(x, y, 0.03)),
                material=concrete,
                name=f"concrete_pad_{idx}",
            )
            frame.visual(
                Cylinder(radius=0.115, length=0.028),
                origin=Origin(xyz=(x, y, 0.075)),
                material=galvanized,
                name=f"foot_flange_{idx}",
            )
            for bx, by in ((-0.055, -0.055), (-0.055, 0.055), (0.055, -0.055), (0.055, 0.055)):
                frame.visual(
                    Cylinder(radius=0.012, length=0.025),
                    origin=Origin(xyz=(x + bx, y + by, 0.087)),
                    material=bolt_dark,
                    name=f"anchor_bolt_{idx}_{bx:+.2f}_{by:+.2f}",
                )

    # Cross members tie the two A-frame ends together and keep the lower
    # structure visibly rigid rather than looking like four independent poles.
    _add_tube_between(
        frame,
        (-1.55, 0.65, 0.55),
        (1.55, 0.65, 0.55),
        radius=0.035,
        material=frame_green,
        name="front_spreader",
    )
    _add_tube_between(
        frame,
        (-1.55, -0.65, 0.55),
        (1.55, -0.65, 0.55),
        radius=0.035,
        material=frame_green,
        name="rear_spreader",
    )
    for i, x in enumerate((-1.55, 1.55)):
        _add_tube_between(
            frame,
            (x, -0.62, 0.68),
            (x, 0.62, 0.68),
            radius=0.032,
            material=frame_green,
            name=f"side_spreader_{i}",
        )
        frame.visual(
            Box((0.18, 0.10, 0.12)),
            origin=Origin(xyz=(x, 0.0, 2.39)),
            material=frame_green,
            name=f"top_socket_{i}",
        )

    for i, x in enumerate((-0.32, 0.32)):
        frame.visual(
            Box((0.14, 0.16, 0.050)),
            origin=Origin(xyz=(x, 0.0, 2.45)),
            material=galvanized,
            name=f"beam_clamp_{i}",
        )
        for y in (-0.025, 0.025):
            frame.visual(
                Box((0.055, 0.018, 0.145)),
                origin=Origin(xyz=(x, y, 2.365)),
                material=galvanized,
                name=f"hanger_strap_{i}_{y:+.2f}",
            )
            frame.visual(
                Cylinder(radius=0.023, length=0.010),
                origin=Origin(xyz=(x, y, 2.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=galvanized,
                name=f"hanger_bushing_{i}_{y:+.2f}",
            )

    swing = model.part("swing")

    seat_profile = rounded_rect_profile(0.76, 0.30, 0.055, corner_segments=10)
    seat_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(seat_profile, 0.070, cap=True, closed=True),
        "rounded_rubber_seat",
    )
    swing.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.745)),
        material=dark_rubber,
        name="seat",
    )
    swing.visual(
        Box((0.66, 0.035, 0.038)),
        origin=Origin(xyz=(0.0, 0.145, -1.700)),
        material=dark_rubber,
        name="front_lip",
    )
    swing.visual(
        Box((0.66, 0.035, 0.038)),
        origin=Origin(xyz=(0.0, -0.145, -1.700)),
        material=dark_rubber,
        name="rear_lip",
    )
    for i, y in enumerate((-0.085, 0.0, 0.085)):
        swing.visual(
            Box((0.62, 0.018, 0.025)),
            origin=Origin(xyz=(0.0, y, -1.788)),
            material=dark_rubber,
            name=f"underside_rib_{i}",
        )

    chain_length = 1.690
    for i, x in enumerate((-0.32, 0.32)):
        swing.visual(
            Cylinder(radius=0.016, length=0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"pivot_pin_{i}",
        )
        swing.visual(
            Cylinder(radius=0.0085, length=chain_length),
            origin=Origin(xyz=(x, 0.0, -chain_length * 0.5)),
            material=galvanized,
            name=f"chain_core_{i}",
        )
        # Alternating small cross-links give the two suspended rods a chain
        # texture while every link remains mechanically tied to the core.
        for j in range(21):
            z = -0.075 - j * 0.077
            if j % 2 == 0:
                rpy = (0.0, math.pi / 2.0, 0.0)
                length = 0.070
            else:
                rpy = (math.pi / 2.0, 0.0, 0.0)
                length = 0.056
            swing.visual(
                Cylinder(radius=0.0058, length=length),
                origin=Origin(xyz=(x, 0.0, z), rpy=rpy),
                material=galvanized,
                name=f"chain_link_{i}_{j}",
            )
        swing.visual(
            Box((0.090, 0.050, 0.135)),
            origin=Origin(xyz=(x, 0.0, -1.655)),
            material=galvanized,
            name=f"seat_hanger_{i}",
        )
        for bx in (-0.023, 0.023):
            swing.visual(
                Cylinder(radius=0.010, length=0.012),
                origin=Origin(xyz=(x + bx, 0.026, -1.705), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_dark,
                name=f"seat_bolt_{i}_{bx:+.2f}",
            )

    model.articulation(
        "frame_to_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing = object_model.get_part("swing")
    joint = object_model.get_articulation("frame_to_swing")

    ctx.expect_gap(
        frame,
        swing,
        axis="z",
        positive_elem="top_beam",
        negative_elem="seat",
        min_gap=1.4,
        name="seat hangs well below the top beam",
    )
    ctx.expect_within(
        swing,
        frame,
        axes="x",
        inner_elem="seat",
        outer_elem="top_beam",
        margin=0.02,
        name="seat width stays under the support beam",
    )

    def _seat_center_y() -> float | None:
        aabb = ctx.part_element_world_aabb(swing, elem="seat")
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[1] + upper[1]) * 0.5

    rest_pos = _seat_center_y()
    with ctx.pose({joint: 0.55}):
        forward_pos = _seat_center_y()
    with ctx.pose({joint: -0.55}):
        rear_pos = _seat_center_y()

    ctx.check(
        "positive swing moves seat forward",
        rest_pos is not None
        and forward_pos is not None
        and forward_pos > rest_pos + 0.02,
        details=f"rest={rest_pos}, forward={forward_pos}",
    )
    ctx.check(
        "negative swing moves seat rearward",
        rest_pos is not None
        and rear_pos is not None
        and rear_pos < rest_pos - 0.02,
        details=f"rest={rest_pos}, rear={rear_pos}",
    )

    return ctx.report()


object_model = build_object_model()
