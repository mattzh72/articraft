from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
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
)


def _tube_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")

    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    segments: int = 24,
) -> None:
    origin, length = _tube_origin(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _rounded_pad_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (math.cos(2.0 * math.pi * i / segments) * radius, math.sin(2.0 * math.pi * i / segments) * radius)
        for i in range(segments)
    ]


def _roller_cap_mesh(name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.046, 40),
            [_circle_profile(0.017, 32)],
            0.014,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_decline_bench")

    steel = model.material("satin_black_steel", rgba=(0.02, 0.022, 0.022, 1.0))
    vinyl = model.material("black_vinyl_pad", rgba=(0.015, 0.015, 0.017, 1.0))
    foam = model.material("dense_black_foam", rgba=(0.005, 0.005, 0.006, 1.0))
    cap = model.material("dark_gray_plastic", rgba=(0.10, 0.105, 0.11, 1.0))
    axle = model.material("brushed_steel_axle", rgba=(0.48, 0.48, 0.46, 1.0))

    frame = model.part("frame")

    # Low, full-size tubular home-gym base.  The X axis runs from rear (-X) to
    # front (+X), Y is bench width, and Z is up.
    for y, suffix in ((-0.22, "0"), (0.22, "1")):
        _add_tube(frame, f"base_rail_{suffix}", (-0.82, y, 0.12), (0.94, y, 0.12), 0.024, steel)
    _add_tube(frame, "rear_crossbar", (-0.82, -0.34, 0.12), (-0.82, 0.34, 0.12), 0.026, steel)
    _add_tube(frame, "front_crossbar", (0.94, -0.34, 0.12), (0.94, 0.34, 0.12), 0.026, steel)
    _add_tube(frame, "middle_crossbar", (0.18, -0.22, 0.12), (0.18, 0.22, 0.12), 0.022, steel)
    _add_tube(frame, "front_center_rail", (0.18, 0.0, 0.12), (0.94, 0.0, 0.12), 0.020, steel)

    # Hinge stands and rails for the adjustable back and seat pads.
    for y, suffix in ((-0.20, "0"), (0.20, "1")):
        _add_tube(frame, f"back_hinge_strut_{suffix}", (-0.47, y, 0.12), (-0.12, y, 0.50), 0.021, steel)
        _add_tube(frame, f"seat_hinge_post_{suffix}", (0.36, y, 0.12), (0.36, y, 0.50), 0.021, steel)
        _add_tube(frame, f"top_side_rail_{suffix}", (-0.12, y, 0.50), (0.36, y, 0.50), 0.018, steel)
    _add_tube(frame, "back_hinge_axle", (-0.12, -0.27, 0.50), (-0.12, 0.27, 0.50), 0.012, axle)
    _add_tube(frame, "seat_hinge_axle", (0.36, -0.25, 0.50), (0.36, 0.25, 0.50), 0.012, axle)

    # Front roller tower and its wide support axle.
    _add_tube(frame, "front_upright", (0.76, 0.0, 0.12), (0.88, 0.0, 0.55), 0.024, steel)
    _add_tube(frame, "front_upright_brace_0", (0.62, -0.18, 0.12), (0.86, 0.0, 0.47), 0.016, steel)
    _add_tube(frame, "front_upright_brace_1", (0.62, 0.18, 0.12), (0.86, 0.0, 0.47), 0.016, steel)
    _add_tube(frame, "front_roller_axle", (0.88, -0.36, 0.55), (0.88, 0.36, 0.55), 0.014, axle)
    frame.visual(
        Box((0.11, 0.06, 0.035)),
        origin=Origin(xyz=(0.88, 0.0, 0.55)),
        material=steel,
        name="roller_axle_block",
    )

    # Small rear wheel axle and bent-looking dropout tubes under the rear of the frame.
    _add_tube(frame, "rear_wheel_axle", (-0.92, -0.325, 0.070), (-0.92, 0.325, 0.070), 0.010, axle)
    _add_tube(frame, "rear_wheel_mount_0", (-0.82, -0.18, 0.12), (-0.92, -0.275, 0.070), 0.014, steel)
    _add_tube(frame, "rear_wheel_mount_1", (-0.82, 0.18, 0.12), (-0.92, 0.275, 0.070), 0.014, steel)

    # Backrest part: local frame is exactly on the central hinge line.  The pad
    # is authored in a shallow declined rest angle and positive joint travel
    # lifts the rear end upward.
    backrest = model.part("backrest")
    back_len = 1.06
    back_width = 0.39
    back_thick = 0.075
    decline = math.radians(10.0)
    back_pad_origin = Origin(xyz=(-0.57, 0.0, 0.035), rpy=(0.0, -decline, 0.0))
    backrest.visual(
        _rounded_pad_mesh(back_len, back_width, back_thick, 0.075, "backrest_cushion"),
        origin=back_pad_origin,
        material=vinyl,
        name="back_cushion",
    )
    _add_tube(backrest, "back_hinge_sleeve", (0.0, -0.16, 0.0), (0.0, 0.16, 0.0), 0.024, axle)
    _add_tube(
        backrest,
        "back_spine",
        (-0.01, 0.0, -0.018),
        (-0.92 * math.cos(decline), 0.0, -0.92 * math.sin(decline) - 0.010),
        0.017,
        steel,
    )
    _add_tube(
        backrest,
        "back_pad_strap",
        (-0.38 * math.cos(decline), -0.17, -0.38 * math.sin(decline) + 0.004),
        (-0.38 * math.cos(decline), 0.17, -0.38 * math.sin(decline) + 0.004),
        0.010,
        steel,
    )
    for y, suffix in ((-0.11, "0"), (0.11, "1")):
        _add_tube(backrest, f"back_pad_mount_{suffix}", (-0.38, y, -0.055), (-0.38, y, 0.045), 0.012, steel)

    seat = model.part("seat")
    seat_len = 0.46
    seat_width = 0.39
    seat_thick = 0.070
    seat.visual(
        _rounded_pad_mesh(seat_len, seat_width, seat_thick, 0.060, "seat_cushion"),
        origin=Origin(xyz=(-0.5 * seat_len, 0.0, 0.095)),
        material=vinyl,
        name="seat_cushion",
    )
    _add_tube(seat, "seat_hinge_sleeve", (0.0, -0.145, 0.0), (0.0, 0.145, 0.0), 0.023, axle)
    _add_tube(seat, "seat_spine", (-0.01, 0.0, -0.015), (-0.38, 0.0, 0.012), 0.015, steel)
    _add_tube(seat, "seat_pad_strap", (-0.20, -0.16, 0.008), (-0.20, 0.16, 0.008), 0.009, steel)
    for y, suffix in ((-0.11, "0"), (0.11, "1")):
        _add_tube(seat, f"seat_pad_mount_{suffix}", (-0.20, y, 0.006), (-0.20, y, 0.070), 0.012, steel)

    for y, name in ((-0.19, "roller_0"), (0.19, "roller_1")):
        roller = model.part(name)
        roller.visual(
            Cylinder(radius=0.075, length=0.220),
            material=foam,
            name="foam_roller",
        )
        roller.visual(
            _roller_cap_mesh(f"{name}_end_cap_0"),
            origin=Origin(xyz=(0.0, 0.0, -0.117)),
            material=cap,
            name="end_cap_0",
        )
        roller.visual(
            _roller_cap_mesh(f"{name}_end_cap_1"),
            origin=Origin(xyz=(0.0, 0.0, 0.117)),
            material=cap,
            name="end_cap_1",
        )
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(0.88, y, 0.55), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=12.0),
        )

    wheel_tire = TireGeometry(
        0.055,
        0.045,
        inner_radius=0.039,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="ribbed", depth=0.003, count=16, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    wheel_hub = WheelGeometry(
        0.041,
        0.042,
        rim=WheelRim(inner_radius=0.027, flange_height=0.004, flange_thickness=0.003, bead_seat_depth=0.002),
        hub=WheelHub(
            radius=0.018,
            width=0.035,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.012),
    )
    for y, name in ((-0.34, "wheel_0"), (0.34, "wheel_1")):
        wheel = model.part(name)
        wheel.visual(mesh_from_geometry(wheel_tire, f"{name}_tire"), material=foam, name="tire")
        wheel.visual(mesh_from_geometry(wheel_hub, f"{name}_hub"), material=cap, name="hub")
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.92, y, 0.070), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=18.0),
        )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.12, 0.0, 0.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.15),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.36, 0.0, 0.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")

    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    roller_joint_0 = object_model.get_articulation("frame_to_roller_0")
    roller_joint_1 = object_model.get_articulation("frame_to_roller_1")
    wheel_joint_0 = object_model.get_articulation("frame_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("frame_to_wheel_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_hinge_axle",
        elem_b="back_hinge_sleeve",
        reason="The backrest sleeve is intentionally captured around the visible hinge axle.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        elem_a="back_hinge_axle",
        elem_b="back_hinge_sleeve",
        axes="y",
        min_overlap=0.25,
        name="backrest sleeve shares the hinge axle length",
    )

    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_axle",
        elem_b="seat_hinge_sleeve",
        reason="The seat sleeve is intentionally captured around its forward hinge axle.",
    )
    ctx.expect_overlap(
        frame,
        seat,
        elem_a="seat_hinge_axle",
        elem_b="seat_hinge_sleeve",
        axes="y",
        min_overlap=0.22,
        name="seat sleeve shares the forward hinge axle length",
    )

    for roller, name in ((roller_0, "roller_0"), (roller_1, "roller_1")):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a="front_roller_axle",
            elem_b="foam_roller",
            reason=f"{name} is a foam roller rotating around the continuous front support axle.",
        )
        for cap_name in ("end_cap_0", "end_cap_1"):
            ctx.allow_overlap(
                frame,
                roller,
                elem_a="front_roller_axle",
                elem_b=cap_name,
                reason=f"{name} has a retained plastic end cap seated around the support axle.",
            )
            ctx.expect_within(
                frame,
                roller,
                inner_elem="front_roller_axle",
                outer_elem=cap_name,
                axes="xz",
                margin=0.002,
                name=f"{name} {cap_name} is centered on the axle",
            )
        ctx.expect_overlap(
            frame,
            roller,
            elem_a="front_roller_axle",
            elem_b="foam_roller",
            axes="y",
            min_overlap=0.18,
            name=f"{name} remains sleeved on the support axle",
        )
        ctx.expect_within(
            frame,
            roller,
            inner_elem="front_roller_axle",
            outer_elem="foam_roller",
            axes="xz",
            margin=0.002,
            name=f"{name} surrounds the axle in cross section",
        )

    for wheel, name in ((wheel_0, "wheel_0"), (wheel_1, "wheel_1")):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_wheel_axle",
            elem_b="hub",
            reason=f"{name} is captured on the small rear transport axle through its hub bore.",
        )
        ctx.expect_within(
            frame,
            wheel,
            inner_elem="rear_wheel_axle",
            outer_elem="hub",
            axes="xz",
            margin=0.002,
            name=f"{name} hub is centered on the rear axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            elem_a="rear_wheel_axle",
            elem_b="hub",
            axes="y",
            min_overlap=0.004,
            name=f"{name} hub remains retained on the rear axle",
        )

    ctx.check(
        "backrest and seat have bounded hinges",
        back_hinge.articulation_type == ArticulationType.REVOLUTE
        and seat_hinge.articulation_type == ArticulationType.REVOLUTE
        and back_hinge.motion_limits is not None
        and seat_hinge.motion_limits is not None
        and back_hinge.motion_limits.upper >= 1.0
        and seat_hinge.motion_limits.upper >= 0.40,
    )
    ctx.check(
        "rollers and rear wheels spin continuously",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            for joint in (roller_joint_0, roller_joint_1, wheel_joint_0, wheel_joint_1)
        ),
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.95}):
        raised_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest raises from declined rest pose",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.20,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.40}):
        raised_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat pad tilts upward on its forward hinge",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.07,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
