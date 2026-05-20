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
    MotionProperties,
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


def _tray_shell_geometry() -> MeshGeometry:
    """Thin open-topped flared wheelbarrow tray, built as a continuous shell."""

    # Each cross-section is a closed loop around the sheet-metal material:
    # outside left wall -> outside floor -> outside right wall -> inside right
    # wall -> inside floor -> inside left wall.  The open top is intentionally
    # left empty, while front/rear end walls are capped.
    sections = [
        (-0.58, 0.50, 0.28, 0.775, 0.455),
        (-0.36, 0.70, 0.42, 0.790, 0.390),
        (0.20, 0.84, 0.58, 0.800, 0.375),
        (0.62, 0.74, 0.48, 0.785, 0.440),
    ]
    thickness = 0.026
    bevel = 0.018

    loops: list[list[tuple[float, float, float]]] = []
    for x, top_w, bottom_w, z_top, z_bottom in sections:
        outer_top = top_w * 0.5
        outer_bottom = bottom_w * 0.5
        inner_top = outer_top - thickness
        inner_bottom = max(outer_bottom - thickness, 0.02)
        outer_center_z = z_bottom - bevel
        inner_center_z = z_bottom + thickness - bevel * 0.55
        loop = [
            (x, -outer_top, z_top),
            (x, -outer_bottom, z_bottom),
            (x, 0.0, outer_center_z),
            (x, outer_bottom, z_bottom),
            (x, outer_top, z_top),
            (x, inner_top, z_top - thickness),
            (x, inner_bottom, z_bottom + thickness),
            (x, 0.0, inner_center_z),
            (x, -inner_bottom, z_bottom + thickness),
            (x, -inner_top, z_top - thickness),
        ]
        loops.append(loop)

    geom = MeshGeometry()
    idx_loops: list[list[int]] = []
    for loop in loops:
        idx_loops.append([geom.add_vertex(*p) for p in loop])

    n = len(idx_loops[0])
    for a_loop, b_loop in zip(idx_loops[:-1], idx_loops[1:]):
        for i in range(n):
            a0 = a_loop[i]
            a1 = a_loop[(i + 1) % n]
            b0 = b_loop[i]
            b1 = b_loop[(i + 1) % n]
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)

    # Cap the front and rear sheet sections to make the tray read as a formed tub.
    for loop in (idx_loops[0], idx_loops[-1]):
        cx = sum(geom.vertices[i][0] for i in loop) / n
        cy = sum(geom.vertices[i][1] for i in loop) / n
        cz = sum(geom.vertices[i][2] for i in loop) / n
        c = geom.add_vertex(cx, cy, cz)
        for i in range(n):
            geom.add_face(c, loop[i], loop[(i + 1) % n])

    return geom


def _rim_tube_geometry():
    points = [
        (-0.58, -0.25, 0.775),
        (-0.36, -0.35, 0.790),
        (0.20, -0.42, 0.800),
        (0.62, -0.37, 0.785),
        (0.62, 0.37, 0.785),
        (0.20, 0.42, 0.800),
        (-0.36, 0.35, 0.790),
        (-0.58, 0.25, 0.775),
    ]
    return tube_from_spline_points(
        points,
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
        closed_spline=True,
        cap_ends=True,
    )


def _tube(points, radius, *, radial_segments=18):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=14,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelbarrow")

    matte_tray = model.material("matte_sage_tray", rgba=(0.16, 0.27, 0.21, 1.0))
    satin_rim = model.material("satin_graphite_rim", rgba=(0.12, 0.13, 0.13, 1.0))
    satin_frame = model.material("satin_tube_graphite", rgba=(0.20, 0.22, 0.22, 1.0))
    brushed_metal = model.material("brushed_steel_hardware", rgba=(0.68, 0.66, 0.60, 1.0))
    black_rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    warm_grip = model.material("charcoal_rubber_grip", rgba=(0.055, 0.050, 0.045, 1.0))

    frame = model.part("frame")
    tray = model.part("tray")
    wheel = model.part("wheel")
    rear_stand = model.part("rear_stand")

    tray.visual(
        mesh_from_geometry(_tray_shell_geometry(), "formed_tray_shell"),
        material=matte_tray,
        name="tray_shell",
    )
    tray.visual(
        mesh_from_geometry(_rim_tube_geometry(), "rolled_tray_rim"),
        material=satin_rim,
        name="rolled_rim",
    )
    # Subtle pressed ribs and rear seam break lines are separate satin details
    # seated on the shell, giving the premium tray a cleaner manufactured read.
    for i, y in enumerate((-0.18, 0.18)):
        tray.visual(
            Box((0.76, 0.018, 0.018)),
            origin=Origin(xyz=(0.08, y, 0.414), rpy=(0.0, 0.09 if y < 0 else -0.09, 0.0)),
            material=satin_rim,
            name=f"floor_rib_{i}",
        )
    tray.visual(
        Box((0.030, 0.62, 0.024)),
        origin=Origin(xyz=(0.60, 0.0, 0.662)),
        material=satin_rim,
        name="rear_seam",
    )

    main_frame_points = [
        (0.98, -0.46, 0.625),
        (0.72, -0.47, 0.525),
        (0.42, -0.47, 0.345),
        (0.18, -0.47, 0.365),
        (-0.43, -0.24, 0.315),
        (-0.66, -0.150, 0.265),
        (-0.735, -0.145, 0.205),
        (-0.790, -0.145, 0.430),
        (-0.825, 0.000, 0.490),
        (-0.790, 0.145, 0.430),
        (-0.735, 0.145, 0.205),
        (-0.66, 0.150, 0.265),
        (-0.43, 0.24, 0.315),
        (0.18, 0.47, 0.365),
        (0.42, 0.47, 0.345),
        (0.72, 0.47, 0.525),
        (0.98, 0.46, 0.625),
    ]
    frame.visual(
        mesh_from_geometry(_tube(main_frame_points, 0.0185), "continuous_tubular_frame"),
        material=satin_frame,
        name="main_tube",
    )

    for i, x in enumerate((-0.08, 0.42)):
        frame.visual(
            mesh_from_geometry(
                _tube([(x, -0.465, 0.345), (x, 0.465, 0.345)], 0.014),
                f"tray_crossbar_{i}",
            ),
            material=satin_frame,
            name=f"tray_crossbar_{i}",
        )
    for i, y in enumerate((-0.245, 0.245)):
        frame.visual(
            Box((0.70, 0.050, 0.020)),
            origin=Origin(xyz=(0.18, y, 0.364)),
            material=black_rubber,
            name=f"tray_saddle_{i}",
        )

    frame.visual(
        mesh_from_geometry(
            _tube([(0.72, -0.470, 0.505), (0.72, 0.470, 0.505)], 0.016),
            "stand_hinge_barrel",
        ),
        material=brushed_metal,
        name="stand_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(-0.735, 0.0, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="axle_pin",
    )
    for i, y in enumerate((-0.205, 0.205)):
        frame.visual(
            Cylinder(radius=0.038, length=0.014),
            origin=Origin(xyz=(-0.735, y, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"axle_washer_{i}",
        )
    for i, y in enumerate((-0.46, 0.46)):
        frame.visual(
            Cylinder(radius=0.025, length=0.17),
            origin=Origin(xyz=(0.985, y, 0.625), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_grip,
            name=f"handle_grip_{i}",
        )

    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.190,
                0.090,
                inner_radius=0.128,
                carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
                tread=TireTread(style="block", depth=0.010, count=24, land_ratio=0.56),
                grooves=(
                    TireGroove(center_offset=-0.020, width=0.006, depth=0.003),
                    TireGroove(center_offset=0.020, width=0.006, depth=0.003),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.128,
                0.076,
                rim=WheelRim(
                    inner_radius=0.082,
                    flange_height=0.010,
                    flange_thickness=0.005,
                    bead_seat_depth=0.004,
                ),
                hub=WheelHub(
                    radius=0.035,
                    width=0.066,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(
                        count=6,
                        circle_diameter=0.052,
                        hole_diameter=0.005,
                    ),
                ),
                face=WheelFace(dish_depth=0.008, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.014),
                bore=WheelBore(style="round", diameter=0.030),
            ),
            "front_wheel_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=brushed_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.0155, length=0.086),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="bearing_bushing",
    )

    stand_points = [
        (0.0, -0.32, 0.0),
        (0.10, -0.35, -0.17),
        (0.30, -0.38, -0.405),
        (0.36, -0.37, -0.430),
        (0.36, 0.37, -0.430),
        (0.30, 0.38, -0.405),
        (0.10, 0.35, -0.17),
        (0.0, 0.32, 0.0),
    ]
    rear_stand.visual(
        mesh_from_geometry(_tube(stand_points, 0.016, radial_segments=18), "rear_stand_tube"),
        material=satin_frame,
        name="stand_tube",
    )
    rear_stand.visual(
        Box((0.105, 0.055, 0.028)),
        origin=Origin(xyz=(0.36, -0.37, -0.445), rpy=(0.0, 0.0, 0.04)),
        material=black_rubber,
        name="foot_pad_0",
    )
    rear_stand.visual(
        Box((0.105, 0.055, 0.028)),
        origin=Origin(xyz=(0.36, 0.37, -0.445), rpy=(0.0, 0.0, -0.04)),
        material=black_rubber,
        name="foot_pad_1",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(-0.735, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "frame_to_rear_stand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_stand,
        origin=Origin(xyz=(0.72, 0.0, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.35, friction=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("wheel")
    rear_stand = object_model.get_part("rear_stand")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    stand_joint = object_model.get_articulation("frame_to_rear_stand")

    ctx.expect_overlap(
        tray,
        frame,
        axes="x",
        elem_a="tray_shell",
        elem_b="main_tube",
        min_overlap=0.75,
        name="formed tray is carried by long tubular frame",
    )
    for i in range(2):
        ctx.allow_overlap(
            frame,
            tray,
            elem_a=f"tray_saddle_{i}",
            elem_b="tray_shell",
            reason="Low-profile rubber saddle pad is slightly compressed into the formed tray underside at the bolted support interface.",
        )
        ctx.expect_gap(
            tray,
            frame,
            axis="z",
            positive_elem="tray_shell",
            negative_elem=f"tray_saddle_{i}",
            max_penetration=0.018,
            name=f"tray saddle {i} has only shallow compression",
        )
    ctx.expect_within(
        wheel,
        frame,
        axes="y",
        inner_elem="tire",
        outer_elem="axle_pin",
        margin=0.18,
        name="single wheel stays centered on front axle hardware",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="hub",
        elem_b="axle_pin",
        min_overlap=0.065,
        name="hub spans the axle pin",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle_pin",
        elem_b="bearing_bushing",
        reason="The front wheel bearing bushing is intentionally captured on the stationary axle pin.",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="bearing_bushing",
        elem_b="axle_pin",
        min_overlap=0.080,
        name="bearing bushing remains captured on axle",
    )
    ctx.allow_overlap(
        frame,
        rear_stand,
        elem_a="stand_hinge_barrel",
        elem_b="stand_tube",
        reason="The folding rear support stand has hinge knuckles represented as locally nested tube-and-barrel hardware.",
    )
    ctx.expect_overlap(
        rear_stand,
        frame,
        axes="y",
        elem_a="stand_tube",
        elem_b="stand_hinge_barrel",
        min_overlap=0.58,
        name="rear stand hinge spans both handle-side brackets",
    )

    rest_wheel = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi * 0.75}):
        spun_wheel = ctx.part_world_position(wheel)
    ctx.check(
        "rolling wheel spins in place",
        rest_wheel is not None
        and spun_wheel is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_wheel, spun_wheel)),
        details=f"rest={rest_wheel}, spun={spun_wheel}",
    )

    rest_stand_aabb = ctx.part_world_aabb(rear_stand)
    with ctx.pose({stand_joint: 1.25}):
        folded_stand_aabb = ctx.part_world_aabb(rear_stand)
    ctx.check(
        "rear stand folds upward from support stance",
        rest_stand_aabb is not None
        and folded_stand_aabb is not None
        and folded_stand_aabb[0][2] > rest_stand_aabb[0][2] + 0.05,
        details=f"rest={rest_stand_aabb}, folded={folded_stand_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
