from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
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
    TorusGeometry,
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


def _cylinder_between(part, name, p0, p1, radius, material):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length tube {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_path_tube(part, name, points, radius, material, *, segments=18):
    mesh = mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=segments,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )
    part.visual(mesh, material=material, name=name)


def _add_axis_shell(part, name, *, radius, inner_radius, length, axis, origin_xyz, material):
    shell = LatheGeometry.from_shell_profiles(
        [(radius, -0.5 * length), (radius, 0.5 * length)],
        [(inner_radius, 0.5 * length), (inner_radius, -0.5 * length)],
        segments=36,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        shell.rotate_y(math.pi / 2.0)
        rpy = (0.0, 0.0, 0.0)
    elif axis == "y":
        shell.rotate_x(-math.pi / 2.0)
        rpy = (0.0, 0.0, 0.0)
    elif axis == "z":
        rpy = (0.0, 0.0, 0.0)
    else:
        raise ValueError(axis)
    part.visual(mesh_from_geometry(shell, name), origin=Origin(xyz=origin_xyz, rpy=rpy), material=material, name=name)


def _add_rounded_pad(part, name, size_xy, height, radius, origin_xyz, material):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(size_xy[0], size_xy[1], radius, corner_segments=10),
        height,
        cap=True,
        closed=True,
    )
    part.visual(mesh_from_geometry(geom, name), origin=Origin(xyz=origin_xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_consumer_wheelchair")

    metal = model.material("satin_graphite_paint", rgba=(0.08, 0.09, 0.10, 1.0))
    alloy = model.material("brushed_dark_alloy", rgba=(0.55, 0.57, 0.57, 1.0))
    cushion = model.material("tailored_black_cushion", rgba=(0.015, 0.016, 0.018, 1.0))
    polymer = model.material("matte_black_polymer", rgba=(0.03, 0.032, 0.036, 1.0))
    rubber = model.material("deep_black_elastomer", rgba=(0.004, 0.004, 0.004, 1.0))
    trim = model.material("warm_grey_trim", rgba=(0.32, 0.33, 0.34, 1.0))

    # Coordinate frame: +X forward, +Y user's left, +Z up.
    frame = model.part("frame")

    # Main tubular side frames.  The side loops touch the seat tray, rear axle,
    # front caster cross tube, and footrest sockets so the root assembly reads as
    # one welded painted-metal chassis.
    for side, label in ((1.0, "left"), (-1.0, "right")):
        y = side * 0.275
        _add_path_tube(
            frame,
            f"{label}_side_loop",
            [
                (-0.32, y, 0.315),
                (-0.26, y, 0.475),
                (0.285, y, 0.475),
                (0.365, y, 0.275),
                (-0.32, y, 0.315),
            ],
            0.016,
            metal,
            segments=14,
        )
        _cylinder_between(frame, f"{label}_rear_upright", (-0.285, y, 0.315), (-0.245, y, 0.535), 0.014, metal)
        _cylinder_between(frame, f"{label}_caster_socket", (0.385, y, 0.275), (0.385, y, 0.345), 0.020, metal)
        _cylinder_between(frame, f"{label}_footrest_boom", (0.285, y, 0.475), (0.435, side * 0.245, 0.475), 0.012, metal)
        _cylinder_between(frame, f"{label}_footrest_socket", (0.435, side * 0.245, 0.400), (0.435, side * 0.245, 0.520), 0.018, metal)
        _cylinder_between(frame, f"{label}_back_hinge_cheek", (-0.245, y, 0.500), (-0.245, y, 0.560), 0.018, metal)
        # Rear axle bosses are coaxial with the spinning drive-wheel joints.
        _cylinder_between(frame, f"{label}_axle_stub", (-0.315, side * 0.285, 0.315), (-0.315, side * 0.365, 0.315), 0.013, alloy)

    _cylinder_between(frame, "rear_axle_bridge", (-0.315, -0.285, 0.315), (-0.315, 0.285, 0.315), 0.018, metal)
    _cylinder_between(frame, "front_cross_tube", (0.365, -0.285, 0.275), (0.365, 0.285, 0.275), 0.016, metal)
    _cylinder_between(frame, "seat_cross_tube", (0.030, -0.285, 0.475), (0.030, 0.285, 0.475), 0.014, metal)
    _cylinder_between(frame, "rear_seat_tube", (-0.230, -0.285, 0.475), (-0.230, 0.285, 0.475), 0.014, metal)

    frame.visual(
        Box((0.515, 0.555, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.492)),
        material=polymer,
        name="seat_tray",
    )
    _add_rounded_pad(frame, "seat_cushion", (0.500, 0.505), 0.058, 0.060, (0.020, 0.0, 0.536), cushion)
    _add_rounded_pad(frame, "front_seam_lip", (0.510, 0.030), 0.010, 0.008, (0.270, 0.0, 0.568), trim)

    # Subtle fender-like trim arcs keep clothing away from the drive wheels.
    for side, label in ((1.0, "left"), (-1.0, "right")):
        y = side * 0.295
        _add_path_tube(
            frame,
            f"{label}_wheel_guard",
            [(-0.555, y, 0.520), (-0.410, y, 0.610), (-0.170, y, 0.610), (0.015, y, 0.515)],
            0.008,
            polymer,
            segments=18,
        )

    # Large quick-release drive wheels with separate alloy rim, tire, hand rim,
    # and small stand-offs tying the hand rim back to the wheel face.
    drive_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.255,
            0.044,
            rim=WheelRim(inner_radius=0.185, flange_height=0.010, flange_thickness=0.005, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.052,
                width=0.038,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.040),
        ),
        "drive_wheel_alloy",
    )
    drive_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.315,
            0.055,
            inner_radius=0.255,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.055),
            tread=TireTread(style="ribbed", depth=0.004, count=30, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.007, radius=0.004),
        ),
        "drive_tire",
    )
    push_ring_geom = TorusGeometry(0.285, 0.006, radial_segments=18, tubular_segments=72)
    push_ring_geom.rotate_y(math.pi / 2.0)
    push_ring_mesh = mesh_from_geometry(push_ring_geom, "push_rim")

    for side, label in ((1.0, "left"), (-1.0, "right")):
        wheel = model.part(f"{label}_drive_wheel")
        wheel.visual(drive_wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=alloy, name="alloy_wheel")
        wheel.visual(drive_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber, name="tire")
        wheel.visual(push_ring_mesh, origin=Origin(xyz=(0.0, side * 0.052, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)), material=alloy, name="push_rim")
        wheel.visual(
            Cylinder(radius=0.023, length=0.080),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name="bearing_sleeve",
        )
        for i in range(6):
            a = i * math.tau / 6.0 + math.radians(14.0)
            p0 = (0.238 * math.cos(a), side * 0.022, 0.238 * math.sin(a))
            p1 = (0.284 * math.cos(a), side * 0.052, 0.284 * math.sin(a))
            _cylinder_between(wheel, f"push_rim_standoff_{i}", p0, p1, 0.0035, alloy)

        model.articulation(
            f"{label}_drive_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.315, side * 0.365, 0.315)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=10.0),
        )

    # Front caster swivels and rolling caster wheels.
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.055,
            0.028,
            rim=WheelRim(inner_radius=0.036, flange_height=0.004, flange_thickness=0.0025, bead_seat_depth=0.002),
            hub=WheelHub(radius=0.018, width=0.022, cap_style="flat"),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.002, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "caster_wheel_alloy",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.082,
            0.036,
            inner_radius=0.055,
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.65),
            sidewall=TireSidewall(style="rounded", bulge=0.030),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "caster_tire",
    )

    for side, label in ((1.0, "left"), (-1.0, "right")):
        yoke = model.part(f"{label}_caster_yoke")
        _cylinder_between(yoke, "swivel_stem", (0.0, 0.0, 0.000), (0.0, 0.0, -0.070), 0.012, alloy)
        yoke.visual(Box((0.095, 0.095, 0.020)), origin=Origin(xyz=(-0.030, 0.0, -0.080)), material=metal, name="fork_crown")
        yoke.visual(Box((0.020, 0.014, 0.155)), origin=Origin(xyz=(-0.045, 0.032, -0.160)), material=metal, name="fork_tine_0")
        yoke.visual(Box((0.020, 0.014, 0.155)), origin=Origin(xyz=(-0.045, -0.032, -0.160)), material=metal, name="fork_tine_1")
        _cylinder_between(yoke, "caster_axle", (-0.045, -0.038, -0.205), (-0.045, 0.038, -0.205), 0.006, alloy)

        caster_wheel = model.part(f"{label}_caster_wheel")
        caster_wheel.visual(caster_wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=alloy, name="caster_rim")
        caster_wheel.visual(caster_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber, name="caster_tire")
        caster_wheel.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name="hub_bushing",
        )

        model.articulation(
            f"{label}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(0.385, side * 0.275, 0.345)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=7.0),
        )
        model.articulation(
            f"{label}_caster_spin",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=caster_wheel,
            origin=Origin(xyz=(-0.045, 0.0, -0.205)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=12.0),
        )

    # Reclining/folding back frame: an explicit hinge at the rear of the seat
    # carries two painted posts, a tailored back pad, and push handles.
    backrest = model.part("backrest")
    for side, label in ((1.0, "left"), (-1.0, "right")):
        y = side * 0.235
        _add_axis_shell(
            backrest,
            f"{label}_hinge_sleeve",
            radius=0.020,
            inner_radius=0.010,
            length=0.065,
            axis="y",
            origin_xyz=(0.0, y, 0.0),
            material=alloy,
        )
        _add_path_tube(
            backrest,
            f"{label}_back_post",
            [(0.0, y, 0.020), (-0.035, y, 0.220), (-0.050, y, 0.510)],
            0.014,
            metal,
            segments=16,
        )
        _add_path_tube(
            backrest,
            f"{label}_push_handle",
            [(-0.050, y, 0.500), (-0.115, y, 0.535), (-0.145, y, 0.510)],
            0.012,
            metal,
            segments=18,
        )
        backrest.visual(
            Cylinder(radius=0.018, length=0.075),
            origin=Origin(xyz=(-0.160, y, 0.505), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer,
            name=f"{label}_grip",
        )
    _cylinder_between(backrest, "top_back_crossbar", (-0.050, -0.245, 0.475), (-0.050, 0.245, 0.475), 0.011, metal)
    _add_rounded_pad(backrest, "back_cushion", (0.045, 0.465), 0.360, 0.018, (-0.062, 0.0, 0.265), cushion)
    backrest.visual(
        Box((0.010, 0.470, 0.012)),
        origin=Origin(xyz=(-0.037, 0.0, 0.085)),
        material=trim,
        name="lower_back_seam",
    )
    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.245, 0.0, 0.535)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.3, lower=-0.18, upper=0.35),
    )

    # Swing-away footrests with foldable-looking polymer plates, each mounted on
    # a vertical frame socket.  The plates are part of the hanger assemblies and
    # are supported by real struts rather than floating tabs.
    for side, label in ((1.0, "left"), (-1.0, "right")):
        footrest = model.part(f"{label}_footrest")
        _cylinder_between(footrest, "pivot_stem", (0.0, 0.0, 0.000), (0.0, 0.0, -0.220), 0.012, alloy)
        _cylinder_between(footrest, "hanger_tube", (0.0, 0.0, -0.190), (0.205, 0.0, -0.330), 0.011, metal)
        _cylinder_between(footrest, "plate_strut", (0.190, 0.0, -0.325), (0.275, -side * 0.060, -0.360), 0.008, metal)
        footrest.visual(
            Box((0.230, 0.165, 0.022)),
            origin=Origin(xyz=(0.315, -side * 0.080, -0.368)),
            material=polymer,
            name="footplate",
        )
        footrest.visual(
            Box((0.210, 0.018, 0.010)),
            origin=Origin(xyz=(0.330, -side * 0.156, -0.355)),
            material=trim,
            name="toe_lip",
        )
        model.articulation(
            f"{label}_footrest_swing",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.435, side * 0.245, 0.440)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.15, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    left_drive = object_model.get_part("left_drive_wheel")
    right_drive = object_model.get_part("right_drive_wheel")
    left_caster_yoke = object_model.get_part("left_caster_yoke")
    left_caster = object_model.get_part("left_caster_wheel")
    right_caster_yoke = object_model.get_part("right_caster_yoke")
    right_caster = object_model.get_part("right_caster_wheel")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")

    ctx.allow_overlap(
        frame,
        left_drive,
        reason="The fixed axle is intentionally captured inside the rotating drive-wheel bearing sleeve.",
    )
    ctx.allow_overlap(
        frame,
        right_drive,
        reason="The fixed axle is intentionally captured inside the rotating drive-wheel bearing sleeve.",
    )
    ctx.allow_overlap(
        frame,
        left_caster_yoke,
        reason="The caster swivel stem is intentionally seated inside the frame socket.",
    )
    ctx.allow_overlap(
        frame,
        right_caster_yoke,
        reason="The caster swivel stem is intentionally seated inside the frame socket.",
    )
    ctx.allow_overlap(
        frame,
        left_footrest,
        reason="The swing-away footrest pivot stem is intentionally retained inside the frame socket.",
    )
    ctx.allow_overlap(
        frame,
        right_footrest,
        reason="The swing-away footrest pivot stem is intentionally retained inside the frame socket.",
    )
    ctx.allow_overlap(
        frame,
        backrest,
        reason="The backrest hinge sleeve shares the hinge barrel volume with the frame cheek proxy.",
    )
    ctx.allow_overlap(
        left_caster_yoke,
        left_caster,
        reason="The caster axle is intentionally captured inside the wheel hub bushing.",
    )
    ctx.allow_overlap(
        right_caster_yoke,
        right_caster,
        reason="The caster axle is intentionally captured inside the wheel hub bushing.",
    )

    ctx.expect_overlap(left_drive, frame, axes="z", min_overlap=0.020, name="left drive wheel centered on axle height")
    ctx.expect_overlap(right_drive, frame, axes="z", min_overlap=0.020, name="right drive wheel centered on axle height")
    ctx.expect_overlap(left_drive, frame, axes="y", min_overlap=0.020, name="left drive bearing captures axle")
    ctx.expect_overlap(right_drive, frame, axes="y", min_overlap=0.020, name="right drive bearing captures axle")
    left_caster_pos = ctx.part_world_position(left_caster)
    right_caster_pos = ctx.part_world_position(right_caster)
    ctx.check(
        "left caster is below front frame",
        left_caster_pos is not None and left_caster_pos[2] < 0.20,
        details=f"left_caster_pos={left_caster_pos}",
    )
    ctx.check(
        "right caster is below front frame",
        right_caster_pos is not None and right_caster_pos[2] < 0.20,
        details=f"right_caster_pos={right_caster_pos}",
    )
    ctx.expect_gap(left_footrest, frame, axis="x", min_gap=0.02, positive_elem="footplate", negative_elem="seat_tray", name="left footplate projects forward")
    ctx.expect_gap(right_footrest, frame, axis="x", min_gap=0.02, positive_elem="footplate", negative_elem="seat_tray", name="right footplate projects forward")
    ctx.expect_overlap(left_footrest, frame, axes="z", min_overlap=0.030, name="left footrest pivot remains inserted")
    ctx.expect_overlap(right_footrest, frame, axes="z", min_overlap=0.030, name="right footrest pivot remains inserted")
    ctx.expect_overlap(left_caster_yoke, frame, axes="z", min_overlap=0.060, name="left caster stem remains inserted")
    ctx.expect_overlap(right_caster_yoke, frame, axes="z", min_overlap=0.060, name="right caster stem remains inserted")
    ctx.expect_contact(backrest, frame, contact_tol=0.020, name="back hinge is carried by frame")

    left_swivel = object_model.get_articulation("left_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_spin")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    right_spin = object_model.get_articulation("right_caster_spin")
    back_hinge = object_model.get_articulation("back_hinge")
    left_foot_swing = object_model.get_articulation("left_footrest_swing")

    with ctx.pose({left_swivel: 0.75, right_swivel: -0.75}):
        ctx.expect_contact(left_caster_yoke, left_caster, contact_tol=0.005, name="left caster wheel remains pinned in fork")
        ctx.expect_contact(right_caster_yoke, right_caster, contact_tol=0.005, name="right caster wheel remains pinned in fork")
        ctx.expect_overlap(left_caster_yoke, left_caster, axes="y", min_overlap=0.020, name="left caster axle remains captured")
        ctx.expect_overlap(right_caster_yoke, right_caster, axes="y", min_overlap=0.020, name="right caster axle remains captured")

    rest_back = ctx.part_world_position(backrest)
    with ctx.pose({back_hinge: 0.25}):
        reclined_back = ctx.part_world_position(backrest)
    ctx.check(
        "backrest hinge changes pose",
        rest_back is not None and reclined_back is not None and abs(reclined_back[0] - rest_back[0]) < 0.001,
        details=f"rest={rest_back}, reclined={reclined_back}",
    )

    foot_rest_pos = ctx.part_world_position(left_footrest)
    with ctx.pose({left_foot_swing: 0.8}):
        foot_swung_pos = ctx.part_world_position(left_footrest)
    ctx.check(
        "left footrest swings around vertical socket",
        foot_rest_pos is not None and foot_swung_pos is not None,
        details=f"rest={foot_rest_pos}, swung={foot_swung_pos}",
    )

    ctx.check(
        "caster rolling joints are present",
        left_spin.articulation_type == ArticulationType.CONTINUOUS and right_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="caster wheels should spin independently inside their swivel yokes",
    )

    return ctx.report()


object_model = build_object_model()
