from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
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
    tube_from_spline_points,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material: Material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _superellipse_loop(width: float, length: float, y_center: float, z: float, *, segments: int = 36) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    exponent = 3.0
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        x = (width * 0.5) * math.copysign(abs(c) ** (2.0 / exponent), c)
        y = y_center + (length * 0.5) * math.copysign(abs(s) ** (2.0 / exponent), s)
        points.append((x, y, z))
    return points


def _build_hollow_tray() -> MeshGeometry:
    """Thin open basin with rolled rim, sloped walls, and a real floor thickness."""
    geom = MeshGeometry()
    outer_top = _superellipse_loop(0.36, 0.50, -0.20, 0.145)
    inner_top = _superellipse_loop(0.315, 0.455, -0.20, 0.128)
    inner_floor = _superellipse_loop(0.175, 0.285, -0.19, 0.035)
    outer_floor = _superellipse_loop(0.215, 0.330, -0.19, 0.016)

    loops = []
    for loop in (outer_top, inner_top, inner_floor, outer_floor):
        ids = [geom.add_vertex(*point) for point in loop]
        loops.append(ids)

    outer_top_ids, inner_top_ids, inner_floor_ids, outer_floor_ids = loops
    count = len(outer_top_ids)
    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, outer_floor_ids[i], outer_floor_ids[j], outer_top_ids[j], outer_top_ids[i])
        _add_quad(geom, inner_top_ids[i], inner_top_ids[j], inner_floor_ids[j], inner_floor_ids[i])
        _add_quad(geom, outer_top_ids[i], outer_top_ids[j], inner_top_ids[j], inner_top_ids[i])
        _add_quad(geom, outer_floor_ids[j], outer_floor_ids[i], inner_floor_ids[i], inner_floor_ids[j])

    top_center = geom.add_vertex(0.0, -0.19, 0.035)
    underside_center = geom.add_vertex(0.0, -0.19, 0.016)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(top_center, inner_floor_ids[i], inner_floor_ids[j])
        geom.add_face(underside_center, outer_floor_ids[j], outer_floor_ids[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_wheelbarrow")

    tray_red = model.material("powder_red", rgba=(0.74, 0.16, 0.12, 1.0))
    frame_grey = model.material("folding_grey", rgba=(0.34, 0.36, 0.37, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    silver = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    grip_rubber = model.material("soft_grips", rgba=(0.055, 0.065, 0.060, 1.0))

    frame = model.part("frame")
    # The root frame is a compact, connected tubular spine.  Its origin is the
    # tray-dump hinge line, so the tray can rotate realistically about the nose.
    frame.visual(
        Cylinder(radius=0.012, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_grey,
        name="tray_hinge_pin",
    )
    for side, x in enumerate((-0.090, 0.090)):
        rail = tube_from_spline_points(
            [(x, 0.000, -0.004), (x * 1.10, -0.160, -0.022), (x * 1.22, -0.350, -0.026)],
            radius=0.009,
            samples_per_segment=14,
            radial_segments=16,
        )
        frame.visual(mesh_from_geometry(rail, f"undertray_rail_{side}"), material=frame_grey, name=f"undertray_rail_{side}")
    _add_member(frame, (-0.125, -0.330, -0.026), (0.125, -0.330, -0.026), 0.0085, frame_grey, name="rear_crossbar")
    frame.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.0, 0.074, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_grey,
        name="fork_pivot_bar",
    )
    for side, x in enumerate((-0.090, 0.090)):
        _add_member(
            frame,
            (x, 0.000, -0.004),
            (x, 0.074, -0.022),
            0.007,
            frame_grey,
            name=f"front_upright_{side}",
        )
    for side, x in enumerate((-0.155, 0.155)):
        leg_x = x * 0.76
        frame.visual(
            Box((0.032, 0.040, 0.040)),
            origin=Origin(xyz=(x * 0.70, 0.070, -0.030)),
            material=dark_steel,
            name=f"fork_hinge_cheek_{side}",
        )
        _add_member(
            frame,
            (math.copysign(0.125, x), -0.330, -0.026),
            (x, -0.315, -0.037),
            0.0065,
            frame_grey,
            name=f"handle_socket_bridge_{side}",
        )
        _add_member(
            frame,
            (math.copysign(0.120, x), -0.330, -0.026),
            (leg_x, -0.360, -0.032),
            0.0065,
            frame_grey,
            name=f"leg_hinge_bridge_{side}",
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(-0.155, -0.315, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_socket_0",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.155, -0.315, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_socket_1",
    )
    frame.visual(
        Box((0.040, 0.030, 0.030)),
        origin=Origin(xyz=(-0.118, -0.360, -0.052)),
        material=dark_steel,
        name="leg_hinge_block_0",
    )
    frame.visual(
        Box((0.040, 0.030, 0.030)),
        origin=Origin(xyz=(0.118, -0.360, -0.052)),
        material=dark_steel,
        name="leg_hinge_block_1",
    )

    tray = model.part("tray")
    tray.visual(mesh_from_geometry(_build_hollow_tray(), "hollow_red_tray"), material=tray_red, name="tray_basin")
    rim = tube_from_spline_points(
        _superellipse_loop(0.365, 0.505, -0.20, 0.148, segments=48),
        radius=0.009,
        closed_spline=True,
        samples_per_segment=4,
        radial_segments=16,
    )
    tray.visual(mesh_from_geometry(rim, "rolled_tray_rim"), material=tray_red, name="rolled_rim")
    tray.visual(
        Box((0.245, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, -0.020, 0.034)),
        material=tray_red,
        name="front_nose_web",
    )
    tray.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tray_red,
        name="tray_hinge_ear_0",
    )
    tray.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tray_red,
        name="tray_hinge_ear_1",
    )
    tray.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(-0.125, -0.020, 0.016)),
        material=tray_red,
        name="hinge_tab_0",
    )
    tray.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.125, -0.020, 0.016)),
        material=tray_red,
        name="hinge_tab_1",
    )
    tray.visual(
        Box((0.250, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.335, 0.012)),
        material=dark_steel,
        name="rear_stop_pad",
    )

    front_wheel_fork = model.part("front_wheel_fork")
    front_wheel_fork.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_grey,
        name="fork_hinge_sleeve",
    )
    front_wheel_fork.visual(
        Cylinder(radius=0.0075, length=0.155),
        origin=Origin(xyz=(0.0, 0.078, -0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="through_axle",
    )
    for side, x in enumerate((-0.064, 0.064)):
        arm = tube_from_spline_points(
            [(x, 0.0, -0.018), (x, 0.040, -0.056), (x, 0.078, -0.095)],
            radius=0.008,
            samples_per_segment=10,
            radial_segments=16,
        )
        front_wheel_fork.visual(mesh_from_geometry(arm, f"fork_arm_{side}"), material=frame_grey, name=f"fork_arm_{side}")
        front_wheel_fork.visual(
            Cylinder(radius=0.011, length=0.025),
            origin=Origin(xyz=(x, 0.078, -0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"axle_stub_{side}",
        )
    _add_member(front_wheel_fork, (-0.064, 0.015, -0.012), (0.064, 0.015, -0.012), 0.007, frame_grey, name="fork_bridge")

    wheel = model.part("wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.085,
            0.046,
            inner_radius=0.060,
            carcass=TireCarcass(belt_width_ratio=0.64, sidewall_bulge=0.05),
            tread=TireTread(style="circumferential", depth=0.004, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "wheelbarrow_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.058,
            0.050,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.0025, bead_seat_depth=0.002),
            hub=WheelHub(radius=0.020, width=0.040, cap_style="domed", bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.0035)),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "wheelbarrow_rim",
    )
    wheel.visual(tire_mesh, material=rubber, name="tire")
    wheel.visual(wheel_mesh, material=silver, name="rim_and_hub")

    rear_legs = model.part("rear_legs")
    rear_legs.visual(
        Cylinder(radius=0.012, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_grey,
        name="leg_hinge_tube",
    )
    rear_leg_0 = tube_from_spline_points(
        [(-0.105, 0.000, 0.000), (-0.110, -0.040, -0.090), (-0.121, -0.075, -0.155)],
        radius=0.008,
        samples_per_segment=12,
        radial_segments=16,
    )
    rear_legs.visual(mesh_from_geometry(rear_leg_0, "rear_leg_0"), material=frame_grey, name="rear_leg_0")
    rear_legs.visual(
        Box((0.076, 0.032, 0.014)),
        origin=Origin(xyz=(-0.121, -0.085, -0.166)),
        material=dark_steel,
        name="rear_foot_0",
    )
    rear_leg_1 = tube_from_spline_points(
        [(0.105, 0.000, 0.000), (0.110, -0.040, -0.090), (0.121, -0.075, -0.155)],
        radius=0.008,
        samples_per_segment=12,
        radial_segments=16,
    )
    rear_legs.visual(mesh_from_geometry(rear_leg_1, "rear_leg_1"), material=frame_grey, name="rear_leg_1")
    rear_legs.visual(
        Box((0.076, 0.032, 0.014)),
        origin=Origin(xyz=(0.121, -0.085, -0.166)),
        material=dark_steel,
        name="rear_foot_1",
    )
    _add_member(rear_legs, (-0.132, -0.082, -0.155), (0.132, -0.082, -0.155), 0.0065, frame_grey, name="foot_crossbar")

    handles = model.part("handles")
    handle_loop = tube_from_spline_points(
        [
            (-0.155, 0.050, 0.000),
            (-0.160, -0.090, 0.020),
            (-0.170, -0.260, 0.058),
            (-0.110, -0.310, 0.066),
            (0.110, -0.310, 0.066),
            (0.170, -0.260, 0.058),
            (0.160, -0.090, 0.020),
            (0.155, 0.050, 0.000),
        ],
        radius=0.0095,
        samples_per_segment=14,
        radial_segments=18,
    )
    handles.visual(mesh_from_geometry(handle_loop, "folding_handle_loop"), material=frame_grey, name="handle_loop")
    handles.visual(
        Cylinder(radius=0.014, length=0.200),
        origin=Origin(xyz=(0.0, -0.310, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="soft_grip",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tray,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "frame_to_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_wheel_fork,
        origin=Origin(xyz=(0.0, 0.074, -0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.15),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_wheel_fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.078, -0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=12.0),
    )
    model.articulation(
        "frame_to_rear_legs",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_legs,
        origin=Origin(xyz=(0.0, -0.360, -0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.3, lower=-1.15, upper=0.0),
    )
    model.articulation(
        "frame_to_handles",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handles,
        origin=Origin(xyz=(0.0, -0.365, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.22, lower=0.0, upper=0.150),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    front_wheel_fork = object_model.get_part("front_wheel_fork")
    wheel = object_model.get_part("wheel")
    rear_legs = object_model.get_part("rear_legs")
    handles = object_model.get_part("handles")
    tray_joint = object_model.get_articulation("frame_to_tray")
    fork_joint = object_model.get_articulation("frame_to_fork")
    leg_joint = object_model.get_articulation("frame_to_rear_legs")
    handle_slide = object_model.get_articulation("frame_to_handles")

    ctx.allow_overlap(
        frame,
        tray,
        elem_a="tray_hinge_pin",
        elem_b="tray_hinge_ear_0",
        reason="The tray hinge ear wraps around the fixed dump-hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        tray,
        elem_a="tray_hinge_pin",
        elem_b="tray_hinge_ear_1",
        reason="The tray hinge ear wraps around the fixed dump-hinge pin.",
    )
    for ear_name in ("tray_hinge_ear_0", "tray_hinge_ear_1"):
        ctx.expect_overlap(
            frame,
            tray,
            axes="x",
            elem_a="tray_hinge_pin",
            elem_b=ear_name,
            min_overlap=0.015,
            name=f"{ear_name} is captured on tray hinge pin",
        )

    ctx.allow_overlap(
        frame,
        front_wheel_fork,
        elem_a="fork_pivot_bar",
        elem_b="fork_hinge_sleeve",
        reason="The folding fork sleeve rotates around the compact front pivot bar.",
    )
    ctx.expect_overlap(
        frame,
        front_wheel_fork,
        axes="x",
        elem_a="fork_pivot_bar",
        elem_b="fork_hinge_sleeve",
        min_overlap=0.12,
        name="fork sleeve is retained on pivot bar",
    )

    ctx.allow_overlap(
        front_wheel_fork,
        wheel,
        elem_a="through_axle",
        elem_b="rim_and_hub",
        reason="The fork axle is intentionally captured through the wheel hub bore.",
    )
    ctx.expect_overlap(
        front_wheel_fork,
        wheel,
        axes="x",
        elem_a="through_axle",
        elem_b="rim_and_hub",
        min_overlap=0.045,
        name="front axle passes through wheel hub",
    )

    for block_name, leg_name in (("leg_hinge_block_0", "rear_leg_0"), ("leg_hinge_block_1", "rear_leg_1")):
        ctx.allow_overlap(
            frame,
            rear_legs,
            elem_a=block_name,
            elem_b="leg_hinge_tube",
            reason="The folding rear stance tube is captured inside the hinge block.",
        )
        ctx.allow_overlap(
            frame,
            rear_legs,
            elem_a=block_name,
            elem_b=leg_name,
            reason="The hinge block intentionally encloses the adjacent welded root of the folding leg.",
        )
        ctx.expect_overlap(
            frame,
            rear_legs,
            axes="x",
            elem_a=block_name,
            elem_b="leg_hinge_tube",
            min_overlap=0.020,
            name=f"{block_name} captures rear leg hinge tube",
        )
        ctx.expect_overlap(
            frame,
            rear_legs,
            axes="x",
            elem_a=block_name,
            elem_b=leg_name,
            min_overlap=0.020,
            name=f"{leg_name} root is tucked into hinge block",
        )

    ctx.allow_overlap(
        frame,
        handles,
        elem_a="handle_socket_0",
        elem_b="handle_loop",
        reason="The telescoping handle tube is intentionally retained inside the left frame sleeve.",
    )
    ctx.allow_overlap(
        frame,
        handles,
        elem_a="handle_socket_1",
        elem_b="handle_loop",
        reason="The telescoping handle tube is intentionally retained inside the right frame sleeve.",
    )
    for socket_name in ("handle_socket_0", "handle_socket_1"):
        ctx.expect_overlap(
            frame,
            handles,
            axes="y",
            elem_a=socket_name,
            elem_b="handle_loop",
            min_overlap=0.025,
            name=f"{socket_name} retains the sliding handle",
        )

    ctx.expect_gap(
        tray,
        wheel,
        axis="z",
        positive_elem="tray_basin",
        negative_elem="tire",
        min_gap=0.030,
        name="tray clears the front wheel at rest",
    )
    ctx.expect_overlap(
        wheel,
        front_wheel_fork,
        axes="xz",
        min_overlap=0.030,
        name="fork visibly straddles wheel axle zone",
    )
    ctx.expect_gap(
        tray,
        rear_legs,
        axis="z",
        positive_elem="tray_basin",
        min_gap=0.040,
        name="rear leg hinge has tray clearance",
    )

    tray_rest = ctx.part_element_world_aabb(tray, elem="rear_stop_pad")
    with ctx.pose({tray_joint: 0.85}):
        tray_dumped = ctx.part_element_world_aabb(tray, elem="rear_stop_pad")
        ctx.expect_gap(
            tray,
            wheel,
            axis="z",
            positive_elem="tray_basin",
            negative_elem="tire",
            min_gap=0.030,
            name="dumped tray still clears wheel",
        )
    ctx.check(
        "tray dump hinge raises rear of basin",
        tray_rest is not None and tray_dumped is not None and tray_dumped[1][2] > tray_rest[1][2] + 0.12,
        details=f"rest={tray_rest}, dumped={tray_dumped}",
    )

    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({fork_joint: 1.0}):
        forked_wheel = ctx.part_world_position(wheel)
    ctx.check(
        "front fork folds upward for storage",
        wheel_rest is not None and forked_wheel is not None and forked_wheel[2] > wheel_rest[2] + 0.040,
        details=f"rest={wheel_rest}, folded={forked_wheel}",
    )

    foot_rest = ctx.part_world_aabb(rear_legs)
    with ctx.pose({leg_joint: -1.05}):
        leg_stowed = ctx.part_world_aabb(rear_legs)
    ctx.check(
        "rear legs fold flat rearward",
        foot_rest is not None and leg_stowed is not None and leg_stowed[1][2] > foot_rest[1][2] + 0.040,
        details=f"rest={foot_rest}, stowed={leg_stowed}",
    )

    handle_rest = ctx.part_world_position(handles)
    with ctx.pose({handle_slide: 0.150}):
        handle_retracted = ctx.part_world_position(handles)
        for socket_name in ("handle_socket_0", "handle_socket_1"):
            ctx.expect_overlap(
                frame,
                handles,
                axes="y",
                elem_a=socket_name,
                elem_b="handle_loop",
                min_overlap=0.025,
                name=f"{socket_name} still retains retracted handle",
            )
    ctx.check(
        "handles retract to shorten footprint",
        handle_rest is not None and handle_retracted is not None and handle_retracted[1] > handle_rest[1] + 0.10,
        details=f"rest={handle_rest}, retracted={handle_retracted}",
    )

    return ctx.report()


object_model = build_object_model()
