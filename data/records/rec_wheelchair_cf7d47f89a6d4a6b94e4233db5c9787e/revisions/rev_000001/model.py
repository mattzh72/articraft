from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _tube_rpy(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    radial = math.hypot(dx, dy)
    pitch = math.atan2(radial, dz)
    yaw = math.atan2(dy, dx) if radial > 1e-9 else 0.0
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    extend: float = 0.006,
) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    a = (p0[0] - ux * extend, p0[1] - uy * extend, p0[2] - uz * extend)
    b = (p1[0] + ux * extend, p1[1] + uy * extend, p1[2] + uz * extend)
    full_length = length + 2.0 * extend
    part.visual(
        Cylinder(radius=radius, length=full_length),
        origin=Origin(
            xyz=((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5),
            rpy=_tube_rpy(a, b),
        ),
        material=material,
        name=name,
    )


def _add_rear_wheel_visuals(part, side_sign: float, name_prefix: str, rim_mat: Material, tire_mat: Material) -> None:
    wheel = WheelGeometry(
        0.255,
        0.040,
        rim=WheelRim(inner_radius=0.178, flange_height=0.012, flange_thickness=0.004, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.044,
            width=0.050,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.052, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=12, thickness=0.0035, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire = TireGeometry(
        0.310,
        0.044,
        inner_radius=0.250,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    part.visual(
        mesh_from_geometry(tire, f"{name_prefix}_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_mat,
        name="tire",
    )
    part.visual(
        mesh_from_geometry(wheel, f"{name_prefix}_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_mat,
        name="rim_spokes_hub",
    )
    part.visual(
        mesh_from_geometry(TorusGeometry(0.265, 0.005, radial_segments=18, tubular_segments=96), f"{name_prefix}_handrim"),
        origin=Origin(xyz=(0.0, side_sign * 0.043, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rim_mat,
        name="push_rim",
    )
    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0 + math.pi / 8.0
        x = 0.265 * math.cos(angle)
        z = 0.265 * math.sin(angle)
        _add_tube(
            part,
            f"push_rim_tab_{i}",
            (x, side_sign * 0.015, z),
            (x, side_sign * 0.043, z),
            0.0025,
            rim_mat,
            extend=0.001,
        )


def _add_caster_wheel_visuals(part, name_prefix: str, rim_mat: Material, tire_mat: Material) -> None:
    tire = TireGeometry(
        0.075,
        0.034,
        inner_radius=0.049,
        carcass=TireCarcass(belt_width_ratio=0.76, sidewall_bulge=0.025),
        sidewall=TireSidewall(style="rounded", bulge=0.025),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    part.visual(
        mesh_from_geometry(tire, f"{name_prefix}_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_mat,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark_rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    fabric = model.material("navy_woven_fabric", rgba=(0.025, 0.040, 0.075, 1.0))
    fabric_seam = model.material("raised_black_seams", rgba=(0.008, 0.009, 0.011, 1.0))
    footrest_mat = model.material("black_composite", rgba=(0.03, 0.032, 0.034, 1.0))

    frame = model.part("frame")

    tube_r = 0.018
    side_y = 0.245
    for sign, label in ((1.0, "left"), (-1.0, "right")):
        y = sign * side_y
        _add_tube(frame, f"{label}_seat_rail", (-0.39, y, 0.485), (0.31, y, 0.485), tube_r, aluminum)
        _add_tube(frame, f"{label}_lower_rail", (-0.42, y, 0.250), (0.40, y, 0.205), tube_r, aluminum)
        _add_tube(frame, f"{label}_rear_post", (-0.39, y, 0.235), (-0.405, y, 1.010), tube_r, aluminum)
        _add_tube(frame, f"{label}_front_post", (0.305, y, 0.485), (0.405, y, 0.205), tube_r, aluminum)
        _add_tube(frame, f"{label}_diagonal_brace", (-0.335, y, 0.485), (0.255, y, 0.225), 0.014, aluminum)
        _add_tube(frame, f"{label}_back_handle_tube", (-0.405, y, 0.940), (-0.545, y, 1.045), tube_r, aluminum)
        _add_tube(frame, f"{label}_push_grip", (-0.545, y, 1.045), (-0.675, y, 1.045), 0.019, dark_rubber, extend=0.002)
        _add_tube(frame, f"{label}_armrest_front_post", (0.205, y, 0.485), (0.205, y, 0.665), 0.011, aluminum)
        _add_tube(frame, f"{label}_armrest_rear_post", (-0.280, y, 0.485), (-0.280, y, 0.665), 0.011, aluminum)
        frame.visual(
            Box((0.500, 0.062, 0.036)),
            origin=Origin(xyz=(-0.040, sign * 0.264, 0.689)),
            material=dark_rubber,
            name=f"{label}_armrest_pad",
        )
        _add_tube(frame, f"{label}_caster_sleeve", (0.385, sign * 0.205, 0.225), (0.385, sign * 0.205, 0.355), 0.020, aluminum)
        _add_tube(frame, f"{label}_footrest_hanger", (0.285, sign * 0.190, 0.465), (0.525, sign * 0.155, 0.160), 0.012, aluminum)
        _add_tube(frame, f"{label}_footplate_strut", (0.525, sign * 0.155, 0.160), (0.600, sign * 0.135, 0.105), 0.010, aluminum)
        frame.visual(
            Box((0.230, 0.165, 0.024)),
            origin=Origin(xyz=(0.635, sign * 0.125, 0.090), rpy=(0.0, -0.10, 0.0)),
            material=footrest_mat,
            name=f"{label}_footplate",
        )

    # Cross members make the two side supports one welded, nearly symmetric chair frame.
    _add_tube(frame, "front_crossbar", (0.305, -0.270, 0.485), (0.305, 0.270, 0.485), tube_r, aluminum)
    _add_tube(frame, "rear_seat_crossbar", (-0.365, -0.270, 0.485), (-0.365, 0.270, 0.485), tube_r, aluminum)
    _add_tube(frame, "rear_axle_tube", (-0.365, -0.355, 0.310), (-0.365, 0.355, 0.310), 0.020, aluminum, extend=0.0)
    _add_tube(frame, "front_lower_crossbar", (0.385, -0.235, 0.355), (0.385, 0.235, 0.355), 0.016, aluminum)
    _add_tube(frame, "back_top_crossbar", (-0.405, -0.270, 0.965), (-0.405, 0.270, 0.965), tube_r, aluminum)
    _add_tube(frame, "back_lower_crossbar", (-0.398, -0.270, 0.590), (-0.398, 0.270, 0.590), 0.015, aluminum)
    _add_tube(frame, "underseat_crossbrace_a", (-0.365, -0.245, 0.485), (0.385, 0.245, 0.225), 0.014, aluminum, extend=0.018)
    _add_tube(frame, "underseat_crossbrace_b", (-0.365, 0.245, 0.485), (0.385, -0.245, 0.225), 0.014, aluminum, extend=0.018)

    frame.visual(
        Box((0.600, 0.470, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, 0.515)),
        material=fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.035, 0.470, 0.430)),
        origin=Origin(xyz=(-0.425, 0.0, 0.770)),
        material=fabric,
        name="back_sling",
    )
    for i, x in enumerate((-0.230, -0.055, 0.120)):
        frame.visual(
            Box((0.012, 0.474, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.535)),
            material=fabric_seam,
            name=f"seat_seam_{i}",
        )
    for i, z in enumerate((0.675, 0.800, 0.925)):
        frame.visual(
            Box((0.006, 0.474, 0.012)),
            origin=Origin(xyz=(-0.445, 0.0, z)),
            material=fabric_seam,
            name=f"back_seam_{i}",
        )

    left_rear_wheel = model.part("left_rear_wheel")
    right_rear_wheel = model.part("right_rear_wheel")
    _add_rear_wheel_visuals(left_rear_wheel, 1.0, "left_rear_wheel", aluminum, dark_rubber)
    _add_rear_wheel_visuals(right_rear_wheel, -1.0, "right_rear_wheel", aluminum, dark_rubber)

    left_caster = model.part("left_caster")
    right_caster = model.part("right_caster")
    for caster, sign, label in ((left_caster, 1.0, "left"), (right_caster, -1.0, "right")):
        caster.visual(
            Cylinder(radius=0.015, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=aluminum,
            name="swivel_stem",
        )
        caster.visual(
            Cylinder(radius=0.032, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
            material=aluminum,
            name="fork_crown",
        )
        caster.visual(
            Box((0.044, 0.014, 0.120)),
            origin=Origin(xyz=(0.0, 0.043, -0.108)),
            material=aluminum,
            name="outer_fork_arm",
        )
        caster.visual(
            Box((0.044, 0.014, 0.120)),
            origin=Origin(xyz=(0.0, -0.043, -0.108)),
            material=aluminum,
            name="inner_fork_arm",
        )
        _add_tube(caster, "fork_front_bridge", (-0.028, -0.043, -0.060), (-0.028, 0.043, -0.060), 0.009, aluminum)
        _add_tube(caster, "fork_rear_bridge", (0.028, -0.043, -0.060), (0.028, 0.043, -0.060), 0.009, aluminum)
        _add_tube(caster, "wheel_axle", (0.0, -0.052, -0.150), (0.0, 0.052, -0.150), 0.008, aluminum, extend=0.0)
        _add_tube(caster, "outer_axle_boss", (0.0, sign * 0.030, -0.150), (0.0, sign * 0.052, -0.150), 0.010, aluminum, extend=0.0)
        _add_tube(caster, "inner_axle_boss", (0.0, -sign * 0.030, -0.150), (0.0, -sign * 0.052, -0.150), 0.010, aluminum, extend=0.0)

    left_caster_wheel = model.part("left_caster_wheel")
    right_caster_wheel = model.part("right_caster_wheel")
    _add_caster_wheel_visuals(left_caster_wheel, "left_caster_wheel", aluminum, dark_rubber)
    _add_caster_wheel_visuals(right_caster_wheel, "right_caster_wheel", aluminum, dark_rubber)

    fast_spin = MotionLimits(effort=4.0, velocity=20.0)
    caster_spin = MotionLimits(effort=2.0, velocity=15.0)
    swivel = MotionLimits(effort=2.0, velocity=8.0)
    model.articulation(
        "frame_to_left_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.365, 0.360, 0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=fast_spin,
    )
    model.articulation(
        "frame_to_right_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.365, -0.360, 0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=fast_spin,
    )
    model.articulation(
        "frame_to_left_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(0.385, 0.205, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swivel,
    )
    model.articulation(
        "frame_to_right_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(0.385, -0.205, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swivel,
    )
    model.articulation(
        "left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=caster_spin,
    )
    model.articulation(
        "right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=caster_spin,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_left = object_model.get_articulation("frame_to_left_rear_wheel")
    rear_right = object_model.get_articulation("frame_to_right_rear_wheel")
    swivel_left = object_model.get_articulation("frame_to_left_caster")
    swivel_right = object_model.get_articulation("frame_to_right_caster")
    caster_left = object_model.get_articulation("left_caster_to_wheel")
    caster_right = object_model.get_articulation("right_caster_to_wheel")

    for joint in (rear_left, rear_right, swivel_left, swivel_right, caster_left, caster_right):
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.check("rear wheel axles are lateral", rear_left.axis == (0.0, 1.0, 0.0) and rear_right.axis == (0.0, 1.0, 0.0))
    ctx.check("caster swivels are vertical", swivel_left.axis == (0.0, 0.0, 1.0) and swivel_right.axis == (0.0, 0.0, 1.0))
    ctx.check("caster wheels spin laterally", caster_left.axis == (0.0, 1.0, 0.0) and caster_right.axis == (0.0, 1.0, 0.0))

    left_wheel_pos = ctx.part_world_position("left_rear_wheel")
    right_wheel_pos = ctx.part_world_position("right_rear_wheel")
    ctx.check(
        "rear wheels are mirrored about centerline",
        left_wheel_pos is not None
        and right_wheel_pos is not None
        and abs(left_wheel_pos[0] - right_wheel_pos[0]) < 1e-6
        and abs(left_wheel_pos[2] - right_wheel_pos[2]) < 1e-6
        and abs(left_wheel_pos[1] + right_wheel_pos[1]) < 1e-6,
        details=f"left={left_wheel_pos}, right={right_wheel_pos}",
    )

    left_caster_pos = ctx.part_world_position("left_caster")
    right_caster_pos = ctx.part_world_position("right_caster")
    ctx.check(
        "front caster stems are mirrored about centerline",
        left_caster_pos is not None
        and right_caster_pos is not None
        and abs(left_caster_pos[0] - right_caster_pos[0]) < 1e-6
        and abs(left_caster_pos[2] - right_caster_pos[2]) < 1e-6
        and abs(left_caster_pos[1] + right_caster_pos[1]) < 1e-6,
        details=f"left={left_caster_pos}, right={right_caster_pos}",
    )

    frame = object_model.get_part("frame")
    ctx.allow_overlap(
        frame,
        "left_rear_wheel",
        elem_a="rear_axle_tube",
        elem_b="rim_spokes_hub",
        reason="The rear axle end is intentionally seated inside the wheel hub bore.",
    )
    ctx.allow_overlap(
        frame,
        "right_rear_wheel",
        elem_a="rear_axle_tube",
        elem_b="rim_spokes_hub",
        reason="The rear axle end is intentionally seated inside the wheel hub bore.",
    )
    ctx.allow_overlap(
        "left_caster",
        "left_caster_wheel",
        elem_a="wheel_axle",
        elem_b="hub",
        reason="The caster wheel hub is intentionally captured around the fork axle.",
    )
    ctx.allow_overlap(
        "right_caster",
        "right_caster_wheel",
        elem_a="wheel_axle",
        elem_b="hub",
        reason="The caster wheel hub is intentionally captured around the fork axle.",
    )
    ctx.allow_overlap(
        frame,
        "left_caster",
        elem_a="left_caster_sleeve",
        elem_b="swivel_stem",
        reason="The caster swivel stem is intentionally nested in the frame bearing sleeve.",
    )
    ctx.allow_overlap(
        frame,
        "right_caster",
        elem_a="right_caster_sleeve",
        elem_b="swivel_stem",
        reason="The caster swivel stem is intentionally nested in the frame bearing sleeve.",
    )

    ctx.expect_overlap("left_rear_wheel", frame, axes="z", min_overlap=0.035, elem_a="tire", elem_b="rear_axle_tube")
    ctx.expect_overlap("right_rear_wheel", frame, axes="z", min_overlap=0.035, elem_a="tire", elem_b="rear_axle_tube")
    ctx.expect_gap("left_rear_wheel", frame, axis="y", max_penetration=0.040, elem_a="rim_spokes_hub", elem_b="rear_axle_tube")
    ctx.expect_gap(frame, "right_rear_wheel", axis="y", max_penetration=0.040, positive_elem="rear_axle_tube", negative_elem="rim_spokes_hub")
    ctx.expect_overlap("left_caster_wheel", "left_caster", axes="xyz", min_overlap=0.010, elem_a="hub", elem_b="wheel_axle")
    ctx.expect_overlap("right_caster_wheel", "right_caster", axes="xyz", min_overlap=0.010, elem_a="hub", elem_b="wheel_axle")
    ctx.expect_overlap("left_caster", frame, axes="z", min_overlap=0.018, elem_a="swivel_stem", elem_b="left_caster_sleeve")
    ctx.expect_overlap("right_caster", frame, axes="z", min_overlap=0.018, elem_a="swivel_stem", elem_b="right_caster_sleeve")

    with ctx.pose({"frame_to_left_caster": math.pi / 2.0, "frame_to_right_caster": -math.pi / 2.0}):
        left_after = ctx.part_world_position("left_caster_wheel")
        right_after = ctx.part_world_position("right_caster_wheel")
    ctx.check(
        "caster wheel centers remain under swivel stems while swiveling",
        left_after is not None
        and right_after is not None
        and abs(left_after[2] - right_after[2]) < 1e-6
        and abs(left_after[1] + right_after[1]) < 1e-6,
        details=f"left={left_after}, right={right_after}",
    )

    return ctx.report()


object_model = build_object_model()
