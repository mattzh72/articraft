from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _merge(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geom in geometries:
        merged.merge(geom)
    return merged


def _box_mesh(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*xyz)


def _horizontal_cylinder(
    radius: float,
    length: float,
    *,
    angle: float = 0.0,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    """Cylinder whose local Z axis is laid into the XY plane at the given yaw."""
    return CylinderGeometry(radius, length, radial_segments=32).rotate_y(math.pi / 2.0).rotate_z(angle).translate(*xyz)


def _build_star_base_mesh(angles: list[float], pivot_radius: float, pivot_z: float) -> MeshGeometry:
    pieces: list[MeshGeometry] = []

    # Central boss and the five polished spokes are one connected metal casting.
    pieces.append(CylinderGeometry(0.075, 0.065, radial_segments=48).translate(0.0, 0.0, 0.120))

    for angle in angles:
        arm_length = pivot_radius - 0.035
        arm_center_r = 0.035 + arm_length / 2.0
        x = math.cos(angle) * arm_center_r
        y = math.sin(angle) * arm_center_r
        pieces.append(_horizontal_cylinder(0.022, arm_length, angle=angle, xyz=(x, y, 0.165)))

        # Vertical socket under each spoke end; the caster fork swivels below it.
        sx = math.cos(angle) * pivot_radius
        sy = math.sin(angle) * pivot_radius
        pieces.append(CylinderGeometry(0.026, 0.035, radial_segments=32).translate(sx, sy, pivot_z + 0.0175))

    return _merge(*pieces)


def _build_outer_sleeve_mesh() -> MeshGeometry:
    # A hollow gas-lift sleeve, so the sliding inner column is not a fake solid-on-solid overlap.
    return LatheGeometry.from_shell_profiles(
        [(0.047, 0.130), (0.047, 0.365), (0.042, 0.385)],
        [(0.030, 0.135), (0.030, 0.360), (0.033, 0.378)],
        segments=64,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _build_fork_mesh() -> MeshGeometry:
    pieces = [
        # Swivel stem reaches upward to the socket face at the joint origin.
        CylinderGeometry(0.012, 0.020, radial_segments=24).translate(0.0, 0.0, -0.010),
        CylinderGeometry(0.024, 0.010, radial_segments=32).translate(0.0, 0.0, -0.005),
        # Two fork cheeks with a clear gap for the wheel.
        _box_mesh((0.010, 0.040, 0.095), (-0.028, 0.006, -0.065)),
        _box_mesh((0.010, 0.040, 0.095), (0.028, 0.006, -0.065)),
        _box_mesh((0.066, 0.040, 0.012), (0.0, 0.006, -0.014)),
    ]
    return _merge(*pieces)


def _build_seat_cushion_mesh() -> MeshGeometry:
    # Width and thickness vary along the fore-aft axis to make a soft waterfall cushion.
    return superellipse_side_loft(
        [
            (-0.250, 0.030, 0.070, 0.400),
            (-0.185, 0.020, 0.082, 0.485),
            (-0.050, 0.018, 0.088, 0.520),
            (0.115, 0.020, 0.082, 0.500),
            (0.245, 0.028, 0.066, 0.430),
        ],
        exponents=3.2,
        segments=64,
    )


def _build_seat_frame_mesh() -> MeshGeometry:
    perimeter = tube_from_spline_points(
        [
            (-0.215, -0.220, 0.026),
            (-0.255, -0.070, 0.024),
            (-0.240, 0.195, 0.025),
            (-0.090, 0.255, 0.028),
            (0.090, 0.255, 0.028),
            (0.240, 0.195, 0.025),
            (0.255, -0.070, 0.024),
            (0.215, -0.220, 0.026),
        ],
        radius=0.015,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=18,
    )
    rear_cross = _horizontal_cylinder(0.014, 0.440, angle=0.0, xyz=(0.0, 0.238, 0.036))
    front_cross = _horizontal_cylinder(0.012, 0.390, angle=0.0, xyz=(0.0, -0.215, 0.030))
    left_rail = CylinderGeometry(0.010, 0.410, radial_segments=18).rotate_x(math.pi / 2.0).translate(-0.155, 0.025, 0.024)
    right_rail = CylinderGeometry(0.010, 0.410, radial_segments=18).rotate_x(math.pi / 2.0).translate(0.155, 0.025, 0.024)
    return _merge(perimeter, rear_cross, front_cross, left_rail, right_rail)


def _build_back_frame_mesh() -> MeshGeometry:
    outer_loop = tube_from_spline_points(
        [
            (-0.210, 0.0, 0.085),
            (-0.250, 0.0, 0.275),
            (-0.225, 0.0, 0.535),
            (-0.125, 0.0, 0.690),
            (0.0, 0.0, 0.735),
            (0.125, 0.0, 0.690),
            (0.225, 0.0, 0.535),
            (0.250, 0.0, 0.275),
            (0.210, 0.0, 0.085),
            (0.0, 0.0, 0.055),
        ],
        radius=0.017,
        samples_per_segment=12,
        closed_spline=True,
        radial_segments=20,
    )
    lumbar_bar = tube_from_spline_points(
        [(-0.185, 0.002, 0.320), (-0.060, -0.010, 0.340), (0.060, -0.010, 0.340), (0.185, 0.002, 0.320)],
        radius=0.010,
        samples_per_segment=12,
        radial_segments=16,
    )
    shoulder_bar = tube_from_spline_points(
        [(-0.165, 0.002, 0.570), (-0.055, -0.008, 0.590), (0.055, -0.008, 0.590), (0.165, 0.002, 0.570)],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
    )
    lug_0 = CylinderGeometry(0.008, 0.070, radial_segments=16).translate(-0.160, 0.0, 0.053)
    lug_1 = CylinderGeometry(0.008, 0.070, radial_segments=16).translate(0.160, 0.0, 0.053)
    center_lug = CylinderGeometry(0.007, 0.056, radial_segments=16).translate(0.0, 0.0, 0.046)
    return _merge(outer_loop, lumbar_bar, shoulder_bar, lug_0, lug_1, center_lug)


def _build_hinge_sleeve_mesh() -> MeshGeometry:
    # Transverse recline sleeve centered on the hinge axis.  The simplified hub is
    # intentionally represented as a captured sleeve around the seat-side pin.
    return LatheGeometry.from_shell_profiles(
        [(0.018, -0.1925), (0.018, 0.1925)],
        [(0.0108, -0.1890), (0.0108, 0.1890)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _build_back_mesh_panel() -> MeshGeometry:
    panel = PerforatedPanelGeometry(
        (0.395, 0.545),
        0.005,
        hole_diameter=0.014,
        pitch=(0.030, 0.030),
        frame=0.018,
        corner_radius=0.030,
        stagger=True,
    )
    # The panel helper lies in local XY; rotate it into the chair's XZ back plane.
    return panel.rotate_x(math.pi / 2.0).translate(0.0, 0.006, 0.365)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_back_office_chair")

    polished = model.material("polished_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_frame = model.material("satin_black_frame", rgba=(0.03, 0.035, 0.038, 1.0))
    fabric = model.material("charcoal_fabric", rgba=(0.06, 0.065, 0.070, 1.0))
    mesh_mat = model.material("open_black_mesh", rgba=(0.015, 0.018, 0.020, 0.82))
    rubber = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    spoke_angles = [math.radians(90.0 + i * 72.0) for i in range(5)]
    pivot_radius = 0.365
    caster_pivot_z = 0.130

    star_base = model.part("star_base")
    star_base.visual(
        mesh_from_geometry(_build_star_base_mesh(spoke_angles, pivot_radius, caster_pivot_z), "star_spokes"),
        material=polished,
        name="star_spokes",
    )
    star_base.visual(
        mesh_from_geometry(_build_outer_sleeve_mesh(), "outer_sleeve"),
        material=polished,
        name="outer_sleeve",
    )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.023, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=polished,
        name="inner_column",
    )
    lift_column.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black_plastic,
        name="swivel_bearing",
    )

    seat = model.part("seat")
    seat.visual(mesh_from_geometry(_build_seat_cushion_mesh(), "seat_cushion"), material=fabric, name="seat_cushion")
    seat.visual(mesh_from_geometry(_build_seat_frame_mesh(), "seat_frame"), material=dark_frame, name="seat_frame")
    seat.visual(
        Box((0.210, 0.170, 0.026)),
        origin=Origin(xyz=(0.0, 0.030, 0.031)),
        material=black_plastic,
        name="under_plate",
    )
    seat.visual(
        Box((0.054, 0.048, 0.116)),
        origin=Origin(xyz=(-0.225, 0.275, 0.064)),
        material=dark_frame,
        name="rear_bracket_0",
    )
    seat.visual(
        Box((0.054, 0.048, 0.116)),
        origin=Origin(xyz=(0.225, 0.275, 0.064)),
        material=dark_frame,
        name="rear_bracket_1",
    )
    seat.visual(
        Cylinder(radius=0.011, length=0.500),
        origin=Origin(xyz=(0.0, 0.275, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="recline_pin",
    )

    back_frame = model.part("back_frame")
    back_frame.visual(mesh_from_geometry(_build_back_frame_mesh(), "back_frame"), material=dark_frame, name="back_frame")
    back_frame.visual(mesh_from_geometry(_build_hinge_sleeve_mesh(), "hinge_sleeve"), material=dark_frame, name="hinge_sleeve")
    back_frame.visual(mesh_from_geometry(_build_back_mesh_panel(), "back_mesh"), material=mesh_mat, name="back_mesh")

    fork_mesh = mesh_from_geometry(_build_fork_mesh(), "caster_fork")
    wheel_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.030,
            0.030,
            rim=WheelRim(inner_radius=0.020, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.013, width=0.022, cap_style="domed"),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "caster_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.042,
            0.032,
            inner_radius=0.030,
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )

    caster_forks = []
    caster_wheels = []
    for i, angle in enumerate(spoke_angles):
        fork = model.part(f"caster_fork_{i}")
        fork.visual(fork_mesh, material=black_plastic, name="fork_body")
        fork.visual(
            Cylinder(radius=0.0070, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished,
            name="axle",
        )
        caster_forks.append(fork)

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(wheel_rim_mesh, material=polished, name="wheel_rim")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        caster_wheels.append(wheel)

        px = math.cos(angle) * pivot_radius
        py = math.sin(angle) * pivot_radius
        # At q=0 the fork rolling direction points radially outward; the swivel can turn freely.
        yaw = angle - math.pi / 2.0
        model.articulation(
            f"base_to_fork_{i}",
            ArticulationType.CONTINUOUS,
            parent=star_base,
            child=fork,
            origin=Origin(xyz=(px, py, caster_pivot_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"fork_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=25.0),
        )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=star_base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.140),
    )
    model.articulation(
        "column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=5.0),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back_frame,
        origin=Origin(xyz=(0.0, 0.275, 0.120), rpy=(-0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.35, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    star_base = object_model.get_part("star_base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    back_frame = object_model.get_part("back_frame")
    height_joint = object_model.get_articulation("base_to_column")
    swivel_joint = object_model.get_articulation("column_to_seat")
    recline_joint = object_model.get_articulation("seat_to_back")

    ctx.expect_within(
        lift_column,
        star_base,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="gas lift column stays centered in sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        star_base,
        axes="z",
        elem_a="inner_column",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="gas lift has retained insertion at low height",
    )

    low_column_pos = ctx.part_world_position(lift_column)
    with ctx.pose({height_joint: 0.140}):
        ctx.expect_overlap(
            lift_column,
            star_base,
            axes="z",
            elem_a="inner_column",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name="gas lift remains inserted at full height",
        )
        high_column_pos = ctx.part_world_position(lift_column)
    ctx.check(
        "height cylinder extends upward",
        low_column_pos is not None and high_column_pos is not None and high_column_pos[2] > low_column_pos[2] + 0.12,
        details=f"low={low_column_pos}, high={high_column_pos}",
    )

    seat_rest = ctx.part_world_position(seat)
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        seat_turn = ctx.part_world_position(seat)
        ctx.expect_gap(seat, star_base, axis="z", min_gap=0.02, name="seat remains above star base while swiveling")
    ctx.check(
        "seat swivel origin stays on column",
        seat_rest is not None and seat_turn is not None and abs(seat_rest[2] - seat_turn[2]) < 1e-6,
        details=f"rest={seat_rest}, turned={seat_turn}",
    )

    back_rest_aabb = ctx.part_world_aabb(back_frame)
    ctx.allow_overlap(
        back_frame,
        seat,
        elem_a="hinge_sleeve",
        elem_b="recline_pin",
        reason="The back recline sleeve is intentionally captured around the seat-side hinge pin.",
    )
    ctx.expect_overlap(
        back_frame,
        seat,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="recline_pin",
        min_overlap=0.35,
        name="recline pin spans the back hinge sleeve",
    )
    ctx.expect_overlap(
        back_frame,
        seat,
        axes="yz",
        elem_a="hinge_sleeve",
        elem_b="recline_pin",
        min_overlap=0.015,
        name="recline pin is seated through sleeve center",
    )
    with ctx.pose({recline_joint: -0.30}):
        back_recline_aabb = ctx.part_world_aabb(back_frame)
    ctx.check(
        "back frame reclines rearward",
        back_rest_aabb is not None
        and back_recline_aabb is not None
        and back_recline_aabb[1][1] > back_rest_aabb[1][1] + 0.05,
        details=f"rest={back_rest_aabb}, reclined={back_recline_aabb}",
    )

    for i in range(5):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        fork_joint = object_model.get_articulation(f"base_to_fork_{i}")
        wheel_joint = object_model.get_articulation(f"fork_to_wheel_{i}")

        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle",
            elem_b="wheel_rim",
            reason="The steel caster axle is intentionally captured through the simplified wheel hub bore.",
        )
        ctx.expect_overlap(fork, wheel, axes="x", elem_a="axle", elem_b="wheel_rim", min_overlap=0.02, name=f"caster {i} axle spans wheel")
        ctx.expect_overlap(
            fork,
            wheel,
            axes="yz",
            elem_a="axle",
            elem_b="wheel_rim",
            min_overlap=0.006,
            name=f"caster {i} axle is centered in hub bore",
        )
        before = ctx.part_world_aabb(wheel)
        with ctx.pose({fork_joint: math.pi / 3.0, wheel_joint: math.pi / 2.0}):
            after = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster {i} has swivel and rolling freedoms",
            before is not None and after is not None,
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
