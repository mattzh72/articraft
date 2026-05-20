from __future__ import annotations

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded box helper, sized in meters."""
    sx, sy, sz = size
    solid = cq.Workplane("XY").box(sx, sy, sz)
    return solid.edges("|Z").fillet(min(radius, sx * 0.45, sy * 0.45))


def _molded_seat_shape() -> cq.Workplane:
    """One continuous molded-plastic seat with a raised rim and shallow pan."""
    base = _rounded_box((0.68, 0.42, 0.070), 0.055)
    recess = _rounded_box((0.49, 0.25, 0.090), 0.050).translate((0.0, 0.0, 0.056))
    seat = base.cut(recess)

    front_lip = _rounded_box((0.60, 0.060, 0.095), 0.024).translate((0.0, -0.205, 0.035))
    rear_lip = _rounded_box((0.60, 0.060, 0.115), 0.026).translate((0.0, 0.205, 0.045))
    side_lip_0 = _rounded_box((0.070, 0.33, 0.095), 0.026).translate((0.322, 0.0, 0.035))
    side_lip_1 = _rounded_box((0.070, 0.33, 0.095), 0.026).translate((-0.322, 0.0, 0.035))

    return seat.union(front_lip).union(rear_lip).union(side_lip_0).union(side_lip_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_backyard_swing")

    wood = model.material("warm_wood", rgba=(0.55, 0.34, 0.17, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    pin_steel = model.material("bright_pin_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    orange_plastic = model.material("molded_orange_plastic", rgba=(0.95, 0.36, 0.08, 1.0))
    cover_plastic = model.material("slightly_darker_cover_plastic", rgba=(0.82, 0.22, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.45, 0.105, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 1.78)),
        material=wood,
        name="crossbeam",
    )
    for idx, x in enumerate((-0.67, 0.67)):
        frame.visual(
            Box((0.090, 0.090, 1.76)),
            origin=Origin(xyz=(x, 0.0, 0.88)),
            material=wood,
            name=f"upright_{idx}",
        )
        frame.visual(
            Box((0.22, 0.72, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=wood,
            name=f"foot_{idx}",
        )

    # Compact metal clevises below the beam carry the two upper hanger pivots.
    for idx, x in enumerate((-0.36, 0.36)):
        for ear, dx in enumerate((-0.046, 0.046)):
            frame.visual(
                Box((0.010, 0.044, 0.115)),
                origin=Origin(xyz=(x + dx, 0.0, 1.705)),
                material=dark_steel,
                name=f"hanger_ear_{idx}_{ear}",
            )
        frame.visual(
            Cylinder(radius=0.0188, length=0.110),
            origin=Origin(xyz=(x, 0.0, 1.650), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pin_steel,
            name=f"pivot_pin_{idx}",
        )

    seat = model.part("seat")
    seat_shell = mesh_from_cadquery(
        _molded_seat_shape(),
        "molded_seat_shell",
        tolerance=0.0015,
        angular_tolerance=0.12,
    )
    seat.visual(
        seat_shell,
        origin=Origin(xyz=(0.0, 0.0, -1.155)),
        material=orange_plastic,
        name="seat_shell",
    )

    eye_mesh = mesh_from_geometry(TorusGeometry(0.024, 0.0055, radial_segments=24, tubular_segments=12), "hanger_eye")
    lower_eye_mesh = mesh_from_geometry(
        TorusGeometry(0.021, 0.0050, radial_segments=24, tubular_segments=12),
        "lower_hanger_eye",
    )
    for idx, x in enumerate((-0.36, 0.36)):
        seat.visual(
            eye_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hanger_eye_{idx}",
        )
        seat.visual(
            Box((0.024, 0.012, 1.045)),
            origin=Origin(xyz=(x, 0.0, -0.545)),
            material=dark_steel,
            name=f"hanger_link_{idx}",
        )
        seat.visual(
            lower_eye_mesh,
            origin=Origin(xyz=(x, 0.0, -1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"lower_eye_{idx}",
        )
        seat.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(xyz=(x, 0.0, -1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pin_steel,
            name=f"lower_pin_{idx}",
        )

    # Small hinge pins on the molded side walls support the two side covers.
    cover_hinge_x = (-0.402, 0.402)
    for idx, x in enumerate(cover_hinge_x):
        side_sign = 1.0 if x > 0.0 else -1.0
        seat.visual(
            Cylinder(radius=0.006, length=0.180),
            origin=Origin(xyz=(x, 0.0, -1.080), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_steel,
            name=f"cover_pin_{idx}",
        )
        seat.visual(
            Box((0.060, 0.034, 0.034)),
            origin=Origin(xyz=(x - side_sign * 0.020, -0.106, -1.080)),
            material=orange_plastic,
            name=f"cover_lug_{idx}_0",
        )
        seat.visual(
            Box((0.060, 0.034, 0.034)),
            origin=Origin(xyz=(x - side_sign * 0.020, 0.106, -1.080)),
            material=orange_plastic,
            name=f"cover_lug_{idx}_1",
        )

    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 1.650)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.50, upper=0.50),
    )

    for idx, sign in enumerate((-1.0, 1.0)):
        cover = model.part(f"cover_{idx}")
        cover.visual(
            Box((0.026, 0.170, 0.135)),
            origin=Origin(xyz=(sign * 0.019, 0.0, -0.072)),
            material=cover_plastic,
            name="cover_panel",
        )
        cover.visual(
            Box((0.012, 0.132, 0.012)),
            origin=Origin(xyz=(sign * 0.034, 0.0, -0.118)),
            material=orange_plastic,
            name="raised_rib",
        )
        cover.visual(
            Cylinder(radius=0.012, length=0.160),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=cover_plastic,
            name="hinge_knuckle",
        )

        model.articulation(
            f"seat_to_cover_{idx}",
            ArticulationType.REVOLUTE,
            parent=seat,
            child=cover,
            origin=Origin(xyz=(sign * 0.402, 0.0, -1.080)),
            axis=(0.0, -sign, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    cover_0 = object_model.get_part("cover_0")
    cover_1 = object_model.get_part("cover_1")
    swing = object_model.get_articulation("frame_to_seat")
    cover_joint_0 = object_model.get_articulation("seat_to_cover_0")
    cover_joint_1 = object_model.get_articulation("seat_to_cover_1")

    for idx, cover in enumerate((cover_0, cover_1)):
        ctx.allow_overlap(
            seat,
            cover,
            elem_a=f"cover_pin_{idx}",
            elem_b="hinge_knuckle",
            reason="The side cover knuckle intentionally wraps around the small hinge pin fixed to the molded seat.",
        )
        ctx.expect_overlap(
            seat,
            cover,
            axes="yz",
            elem_a=f"cover_pin_{idx}",
            elem_b="hinge_knuckle",
            min_overlap=0.010,
            name=f"cover {idx} hinge pin is captured",
        )

    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            seat,
            elem_a=f"pivot_pin_{idx}",
            elem_b=f"hanger_eye_{idx}",
            reason="The upper hanger eye is intentionally captured around the crossbeam pivot pin.",
        )
        ctx.expect_overlap(
            seat,
            frame,
            axes="xz",
            elem_a=f"hanger_eye_{idx}",
            elem_b=f"pivot_pin_{idx}",
            min_overlap=0.010,
            name=f"hanger {idx} is centered on upper pivot",
        )

    ctx.expect_gap(
        seat,
        cover_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.075,
        positive_elem="seat_shell",
        negative_elem="cover_panel",
        name="closed cover 0 sits just outside seat side",
    )
    ctx.expect_gap(
        cover_1,
        seat,
        axis="x",
        min_gap=0.0,
        max_gap=0.075,
        positive_elem="cover_panel",
        negative_elem="seat_shell",
        name="closed cover 1 sits just outside seat side",
    )

    def _aabb_center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    def _aabb_center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) * 0.5

    rest_seat_y = _aabb_center_y(ctx.part_world_aabb(seat))
    with ctx.pose({swing: 0.45}):
        swung_seat_y = _aabb_center_y(ctx.part_world_aabb(seat))
    ctx.check(
        "seat swings forward about upper pivots",
        rest_seat_y is not None and swung_seat_y is not None and swung_seat_y > rest_seat_y + 0.30,
        details=f"rest_y={rest_seat_y}, swung_y={swung_seat_y}",
    )

    rest_cover_0_x = _aabb_center_x(ctx.part_world_aabb(cover_0))
    rest_cover_1_x = _aabb_center_x(ctx.part_world_aabb(cover_1))
    with ctx.pose({cover_joint_0: 1.0}):
        open_cover_0_x = _aabb_center_x(ctx.part_world_aabb(cover_0))
    with ctx.pose({cover_joint_1: 1.0}):
        open_cover_1_x = _aabb_center_x(ctx.part_world_aabb(cover_1))
    ctx.check(
        "cover 0 rotates outward",
        rest_cover_0_x is not None and open_cover_0_x is not None and open_cover_0_x < rest_cover_0_x - 0.035,
        details=f"rest_x={rest_cover_0_x}, open_x={open_cover_0_x}",
    )
    ctx.check(
        "cover 1 rotates outward",
        rest_cover_1_x is not None and open_cover_1_x is not None and open_cover_1_x > rest_cover_1_x + 0.035,
        details=f"rest_x={rest_cover_1_x}, open_x={open_cover_1_x}",
    )

    return ctx.report()


object_model = build_object_model()
