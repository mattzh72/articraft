from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _cylinder_between(part, start, end, radius, material, name, segments=24):
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _rounded_pad_mesh(length: float, width: float, thickness: float, name: str):
    profile = rounded_rect_profile(length, width, radius=0.045, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    steel = model.material("black_powder_coated_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    bracket = model.material("dark_anodized_brackets", rgba=(0.10, 0.105, 0.11, 1.0))
    pad = model.material("black_textured_vinyl", rgba=(0.015, 0.015, 0.017, 1.0))
    seam = model.material("subtle_pad_seams", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    wheel_plastic = model.material("dark_grey_wheel_hub", rgba=(0.16, 0.16, 0.17, 1.0))
    pin = model.material("brushed_pivot_pins", rgba=(0.55, 0.55, 0.52, 1.0))

    frame = model.part("center_frame")

    # Compact, low home-gym tubular base.
    for y, suffix in ((-0.19, "0"), (0.19, "1")):
        _cylinder_between(frame, (-0.78, y, 0.115), (0.62, y, 0.115), 0.022, steel, f"side_rail_{suffix}")
    _cylinder_between(frame, (-0.76, -0.31, 0.12), (-0.76, 0.31, 0.12), 0.024, steel, "front_crossmember")
    _cylinder_between(frame, (0.62, -0.30, 0.095), (0.62, 0.30, 0.095), 0.026, steel, "rear_floor_tube")
    _cylinder_between(frame, (-0.58, 0.0, 0.155), (0.43, 0.0, 0.155), 0.018, steel, "center_spine")
    _cylinder_between(frame, (-0.82, -0.36, 0.085), (-0.82, 0.36, 0.085), 0.010, pin, "front_wheel_axle")

    # Hinge bridges and support-post tubes rising from the low rails to the pads.
    frame.visual(
        Cylinder(radius=0.016, length=0.46),
        origin=Origin(xyz=(0.03, 0.0, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="back_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.38),
        origin=Origin(xyz=(-0.15, 0.0, 0.33), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="seat_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.40),
        origin=Origin(xyz=(0.52, 0.0, 0.14), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="rear_pivot_pin",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        _cylinder_between(
            frame,
            (-0.31, side * 0.19, 0.13),
            (-0.15, side * 0.195, 0.33),
            0.014,
            steel,
            f"seat_upright_{suffix}",
        )
        _cylinder_between(
            frame,
            (0.15, side * 0.19, 0.13),
            (0.03, side * 0.255, 0.34),
            0.014,
            steel,
            f"back_upright_{suffix}",
        )
        _cylinder_between(frame, (-0.76, side * 0.145, 0.12), (-0.82, side * 0.145, 0.085), 0.012, steel, f"axle_strut_{suffix}")

    for y, suffix in ((-0.225, "0"), (0.225, "1")):
        frame.visual(
            Box((0.075, 0.018, 0.050)),
            origin=Origin(xyz=(0.03, y, 0.332)),
            material=bracket,
            name=f"back_hinge_cheek_{suffix}",
        )
        frame.visual(
            Box((0.065, 0.016, 0.044)),
            origin=Origin(xyz=(-0.15, y * 0.78, 0.324)),
            material=bracket,
            name=f"seat_hinge_cheek_{suffix}",
        )
        frame.visual(
            Box((0.055, 0.016, 0.044)),
            origin=Origin(xyz=(0.52, y * 0.82, 0.134)),
            material=bracket,
            name=f"rear_pivot_cheek_{suffix}",
        )

    # Rubber caps on the floor tube and front transport-wheel axle struts.
    for y, suffix in ((-0.315, "0"), (0.315, "1")):
        frame.visual(
            Cylinder(radius=0.028, length=0.030),
            origin=Origin(xyz=(0.62, y, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rear_foot_cap_{suffix}",
        )

    for x, suffix in ((-0.38, "0"), (0.28, "1")):
        _cylinder_between(frame, (x, -0.19, 0.115), (x, 0.0, 0.155), 0.010, steel, f"spine_tie_{suffix}_0")
        _cylinder_between(frame, (x, 0.19, 0.115), (x, 0.0, 0.155), 0.010, steel, f"spine_tie_{suffix}_1")

    handle_geom = tube_from_spline_points(
        [
            (-0.755, -0.115, 0.145),
            (-0.895, -0.115, 0.148),
            (-0.960, 0.0, 0.150),
            (-0.895, 0.115, 0.148),
            (-0.755, 0.115, 0.145),
        ],
        radius=0.015,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(handle_geom, "front_handle"), material=steel, name="front_handle")

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad_mesh(0.88, 0.32, 0.075, "backrest_cushion"),
        origin=Origin(xyz=(0.47, 0.0, 0.074)),
        material=pad,
        name="cushion",
    )
    backrest.visual(
        Box((0.76, 0.006, 0.004)),
        origin=Origin(xyz=(0.47, 0.0, 0.1125)),
        material=seam,
        name="center_stitch",
    )
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        backrest.visual(
            Box((0.78, 0.026, 0.030)),
            origin=Origin(xyz=(0.42, y, 0.025)),
            material=bracket,
            name=f"support_rail_{suffix}",
        )
    backrest.visual(
        Cylinder(radius=0.020, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="pivot_sleeve",
    )
    backrest.visual(
        Box((0.09, 0.30, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, 0.021)),
        material=bracket,
        name="hinge_plate",
    )

    seat = model.part("seat_pad")
    seat.visual(
        _rounded_pad_mesh(0.52, 0.31, 0.072, "seat_cushion"),
        origin=Origin(xyz=(-0.14, 0.0, 0.072)),
        material=pad,
        name="cushion",
    )
    seat.visual(
        Box((0.40, 0.006, 0.004)),
        origin=Origin(xyz=(-0.14, 0.0, 0.109)),
        material=seam,
        name="center_stitch",
    )
    for y, suffix in ((-0.10, "0"), (0.10, "1")):
        seat.visual(
            Box((0.42, 0.025, 0.030)),
            origin=Origin(xyz=(-0.12, y, 0.025)),
            material=bracket,
            name=f"support_rail_{suffix}",
        )
    seat.visual(
        Cylinder(radius=0.018, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="pivot_sleeve",
    )
    seat.visual(
        Box((0.075, 0.28, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.020)),
        material=bracket,
        name="hinge_plate",
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="pivot_sleeve",
    )
    for y, suffix in ((-0.12, "0"), (0.12, "1")):
        rear_support.visual(
            Box((0.018, 0.030, 0.015)),
            origin=Origin(xyz=(-0.025, y, 0.0145)),
            material=bracket,
            name=f"link_lug_{suffix}",
        )
        _cylinder_between(rear_support, (-0.024, y, 0.014), (-0.28, y, 0.196), 0.012, bracket, f"side_link_{suffix}")
    _cylinder_between(rear_support, (-0.28, -0.16, 0.196), (-0.28, 0.16, 0.196), 0.014, bracket, "upper_roller")

    wheel_geom = WheelGeometry(
        0.052,
        0.038,
        rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.020,
            width=0.030,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.027, hole_diameter=0.0035),
        ),
        face=WheelFace(dish_depth=0.0035, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.007),
        bore=WheelBore(style="round", diameter=0.021),
    )
    tire_geom = TireGeometry(
        0.075,
        0.044,
        inner_radius=0.052,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="ribbed", depth=0.003, count=16, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.0015),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    for y, suffix in ((-0.33, "0"), (0.33, "1")):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(mesh_from_geometry(wheel_geom, f"wheel_hub_{suffix}"), material=wheel_plastic, name="hub")
        wheel.visual(mesh_from_geometry(tire_geom, f"wheel_tire_{suffix}"), material=rubber, name="tire")
        model.articulation(
            f"frame_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.82, y, 0.085), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.03, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=0.0, upper=1.15),
    )
    model.articulation(
        "frame_to_seat_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.15, 0.0, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=0.0, upper=0.50),
    )
    model.articulation(
        "frame_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_support,
        origin=Origin(xyz=(0.52, 0.0, 0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.55, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("center_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat_pad")
    rear_support = object_model.get_part("rear_support")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    back_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat_pad")
    rear_pivot = object_model.get_articulation("frame_to_rear_support")

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_hinge_pin",
        elem_b="pivot_sleeve",
        reason="The backrest sleeve is intentionally captured around the transverse frame hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        elem_a="back_hinge_pin",
        elem_b="pivot_sleeve",
        min_overlap=0.30,
        name="backrest hinge sleeve captures the frame pin",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_pin",
        elem_b="pivot_sleeve",
        reason="The seat hinge sleeve is intentionally modeled around its separate frame pin.",
    )
    ctx.expect_overlap(
        frame,
        seat,
        axes="y",
        elem_a="seat_hinge_pin",
        elem_b="pivot_sleeve",
        min_overlap=0.26,
        name="seat hinge sleeve captures the frame pin",
    )
    ctx.allow_overlap(
        frame,
        rear_support,
        elem_a="rear_pivot_pin",
        elem_b="pivot_sleeve",
        reason="The rear support link lower sleeve is intentionally pinned to the frame pivot.",
    )
    ctx.expect_overlap(
        frame,
        rear_support,
        axes="y",
        elem_a="rear_pivot_pin",
        elem_b="pivot_sleeve",
        min_overlap=0.30,
        name="rear support lower sleeve captures the frame pivot",
    )

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        min_gap=0.035,
        max_gap=0.14,
        positive_elem="cushion",
        negative_elem="cushion",
        name="split pads keep a visible hinge gap",
    )
    ctx.expect_overlap(
        wheel_0,
        wheel_1,
        axes="xz",
        min_overlap=0.06,
        elem_a="tire",
        elem_b="tire",
        name="front transport wheels are a matched pair on one axle",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    rest_seat_aabb = ctx.part_world_aabb(seat)
    rest_support_aabb = ctx.part_world_aabb(rear_support)
    with ctx.pose({back_hinge: 0.85, seat_hinge: 0.35, rear_pivot: 0.45}):
        raised_back_aabb = ctx.part_world_aabb(backrest)
        raised_seat_aabb = ctx.part_world_aabb(seat)
        raised_support_aabb = ctx.part_world_aabb(rear_support)

    ctx.check(
        "backrest pivots upward from the center frame",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.20,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )
    ctx.check(
        "seat pad pivots on its own front hinge",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.05,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )
    ctx.check(
        "rear support link swings on the lower frame pivot",
        rest_support_aabb is not None
        and raised_support_aabb is not None
        and abs(raised_support_aabb[1][0] - rest_support_aabb[1][0]) > 0.04,
        details=f"rest={rest_support_aabb}, raised={raised_support_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
