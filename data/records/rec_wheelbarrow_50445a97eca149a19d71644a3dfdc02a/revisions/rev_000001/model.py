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


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _tray_shell_geometry() -> MeshGeometry:
    """Open heavy-gauge tray: closed metal shell with a visible hollow basin."""

    mesh = MeshGeometry()

    # The top ring is wider and longer than the bottom, giving the wheelbarrow
    # its real flared trough silhouette while leaving a true open cavity.
    outer_bottom = [
        (0.525, -0.225, 0.540),
        (0.525, 0.225, 0.540),
        (-0.525, 0.225, 0.540),
        (-0.525, -0.225, 0.540),
    ]
    outer_top = [
        (0.690, -0.410, 0.900),
        (0.690, 0.410, 0.900),
        (-0.690, 0.410, 0.900),
        (-0.690, -0.410, 0.900),
    ]
    inner_top = [
        (0.630, -0.350, 0.850),
        (0.630, 0.350, 0.850),
        (-0.630, 0.350, 0.850),
        (-0.630, -0.350, 0.850),
    ]
    inner_bottom = [
        (0.445, -0.170, 0.610),
        (0.445, 0.170, 0.610),
        (-0.445, 0.170, 0.610),
        (-0.445, -0.170, 0.610),
    ]

    rings = []
    for ring in (outer_bottom, outer_top, inner_top, inner_bottom):
        rings.append([mesh.add_vertex(*p) for p in ring])
    ob, ot, it, ib = rings

    # Exterior side walls.
    for i in range(4):
        _quad(mesh, ob[i], ob[(i + 1) % 4], ot[(i + 1) % 4], ot[i])
    # Rolled heavy lip/rim around the open top.
    for i in range(4):
        _quad(mesh, ot[i], ot[(i + 1) % 4], it[(i + 1) % 4], it[i])
    # Interior basin walls.
    for i in range(4):
        _quad(mesh, it[i], it[(i + 1) % 4], ib[(i + 1) % 4], ib[i])
    # Underside and basin floor.
    _quad(mesh, ob[3], ob[2], ob[1], ob[0])
    _quad(mesh, ib[0], ib[1], ib[2], ib[3])

    return mesh


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    origin, length = _cylinder_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_disc(
    part,
    name: str,
    center: tuple[float, float, float],
    radius: float,
    thickness: float,
    material: Material,
    *,
    along_y: bool = True,
) -> None:
    rpy = (-math.pi / 2.0, 0.0, 0.0) if along_y else (0.0, math.pi / 2.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=thickness),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelbarrow")

    galvanized = model.material("galvanized_tray", rgba=(0.58, 0.62, 0.61, 1.0))
    dark_steel = model.material("powder_coated_steel", rgba=(0.08, 0.10, 0.11, 1.0))
    safety_yellow = model.material("safety_yellow_guard", rgba=(1.0, 0.78, 0.05, 1.0))
    red_lock = model.material("red_lockout", rgba=(0.82, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.76, 0.73, 0.66, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_geometry(_tray_shell_geometry(), "hollow_tray"),
        material=galvanized,
        name="hollow_tray",
    )

    # Rolled rim tubes and flared tray stiffeners.
    top_corners = [
        (0.690, -0.410, 0.900),
        (0.690, 0.410, 0.900),
        (-0.690, 0.410, 0.900),
        (-0.690, -0.410, 0.900),
    ]
    for i in range(4):
        _add_tube(chassis, f"tray_rim_{i}", top_corners[i], top_corners[(i + 1) % 4], 0.020, dark_steel)

    # Two continuous structural rails form the handles, undercarriage, and fork
    # load path instead of decorative disconnected members.
    for side, y in enumerate((-0.340, 0.340)):
        rail_start = (-1.300, y, 0.620)
        rail_mid = (-0.250, y * 0.82, 0.525)
        rail_front = (0.740, y * 0.48, 0.312)
        _add_tube(chassis, f"main_rail_rear_{side}", rail_start, rail_mid, 0.024, dark_steel)
        _add_tube(chassis, f"main_rail_front_{side}", rail_mid, rail_front, 0.024, dark_steel)
        _add_tube(chassis, f"fork_upright_{side}", (0.560, y * 0.58, 0.600), rail_front, 0.026, dark_steel)
        _add_tube(chassis, f"fork_upper_brace_{side}", (0.520, y * 0.58, 0.600), (0.180, y * 0.78, 0.555), 0.020, dark_steel)
        _add_tube(chassis, f"rubber_grip_{side}", (-1.350, y, 0.620), (-1.135, y, 0.620), 0.036, rubber)

    # Crossmembers physically connect the two side rails and support the tray.
    _add_tube(chassis, "rear_handle_crossbar", (-0.950, -0.340, 0.560), (-0.950, 0.340, 0.560), 0.018, dark_steel)
    _add_tube(chassis, "front_tray_crossbar", (0.280, -0.300, 0.540), (0.280, 0.300, 0.540), 0.020, dark_steel)
    _add_tube(chassis, "rear_tray_crossbar", (-0.380, -0.300, 0.535), (-0.380, 0.300, 0.535), 0.020, dark_steel)
    _add_tube(chassis, "front_fork_bridge", (0.570, -0.190, 0.610), (0.570, 0.190, 0.610), 0.024, safety_yellow)
    axle_origin, axle_length = _cylinder_origin_between((0.740, -0.245, 0.312), (0.740, 0.245, 0.312))
    chassis.visual(
        Cylinder(radius=0.026, length=axle_length),
        origin=axle_origin,
        material=zinc,
        name="axle",
    )

    # Rear stance: splayed legs, ground pads, and a crossbar make the resting
    # triangle obvious together with the front wheel.
    for side, y in enumerate((-0.310, 0.310)):
        _add_tube(chassis, f"rear_leg_{side}", (-0.470, y, 0.520), (-0.690, y * 1.10, 0.060), 0.026, dark_steel)
        _add_tube(chassis, f"rear_leg_brace_{side}", (-0.610, y * 1.05, 0.250), (-0.190, y * 0.92, 0.525), 0.017, dark_steel)
        chassis.visual(
            Box((0.220, 0.090, 0.030)),
            origin=Origin(xyz=(-0.730, y * 1.12, 0.020), rpy=(0.0, 0.0, 0.10 if y > 0 else -0.10)),
            material=rubber,
            name=f"foot_pad_{side}",
        )
        chassis.visual(
            Box((0.130, 0.020, 0.130)),
            origin=Origin(xyz=(-0.500, y * 1.02, 0.505), rpy=(0.0, 0.0, 0.18 if y > 0 else -0.18)),
            material=safety_yellow,
            name=f"leg_gusset_{side}",
        )
        chassis.visual(
            Box((0.080, 0.080, 0.065)),
            origin=Origin(xyz=(-0.690, y * 1.10, 0.055)),
            material=dark_steel,
            name=f"foot_socket_{side}",
        )
    _add_tube(chassis, "rear_foot_crossbar", (-0.730, -0.355, 0.035), (-0.730, 0.355, 0.035), 0.017, dark_steel)

    # High-stress tray-to-frame saddle plates and anti-tip / over-travel blocks.
    for x in (-0.380, 0.280):
        chassis.visual(Box((0.210, 0.080, 0.020)), origin=Origin(xyz=(x, -0.250, 0.550)), material=safety_yellow, name=f"saddle_plate_neg_{x}")
        chassis.visual(Box((0.210, 0.080, 0.020)), origin=Origin(xyz=(x, 0.250, 0.550)), material=safety_yellow, name=f"saddle_plate_pos_{x}")
    chassis.visual(Box((0.160, 0.090, 0.060)), origin=Origin(xyz=(-0.660, -0.315, 0.300)), material=safety_yellow, name="tip_stop_0")
    chassis.visual(Box((0.160, 0.090, 0.060)), origin=Origin(xyz=(-0.660, 0.315, 0.300)), material=safety_yellow, name="tip_stop_1")

    # Fork cheek plates remain outside the tire and show the axle capture.
    for side, y in enumerate((-0.205, 0.205)):
        chassis.visual(
            Box((0.150, 0.018, 0.230)),
            origin=Origin(xyz=(0.715, y, 0.405), rpy=(0.0, 0.12 if y > 0 else -0.12, 0.0)),
            material=safety_yellow,
            name=f"fork_cheek_{side}",
        )
        for z in (0.312, 0.500):
            _add_disc(chassis, f"fork_bolt_{side}_{int(z*1000)}", (0.715, y + (0.012 if y > 0 else -0.012), z), 0.018, 0.007, zinc)

    # Side pinch guards and a fender plate around the rolling wheel.
    for side, y in enumerate((-0.235, 0.235)):
        hoop = tube_from_spline_points(
            [
                (0.525, y, 0.365),
                (0.560, y, 0.555),
                (0.710, y, 0.670),
                (0.925, y, 0.565),
                (0.970, y, 0.365),
            ],
            radius=0.013,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
        chassis.visual(mesh_from_geometry(hoop, f"side_wheel_guard_{side}"), material=safety_yellow, name=f"side_wheel_guard_{side}")
        _add_tube(chassis, f"guard_strut_front_{side}", (0.930, y, 0.380), (0.760, y * 0.72, 0.312), 0.012, safety_yellow)
        _add_tube(chassis, f"guard_strut_rear_{side}", (0.535, y, 0.380), (0.610, y * 0.76, 0.312), 0.012, safety_yellow)
    chassis.visual(Box((0.480, 0.220, 0.026)), origin=Origin(xyz=(0.745, 0.0, 0.640), rpy=(0.0, -0.08, 0.0)), material=safety_yellow, name="top_wheel_fender")

    # Brake/lockout bracket with two cheeks, a base strap, and stop blocks. The
    # moving red latch is a separate articulated part below.
    chassis.visual(Box((0.120, 0.070, 0.018)), origin=Origin(xyz=(0.455, -0.320, 0.438)), material=safety_yellow, name="lockout_base")
    chassis.visual(Box((0.050, 0.040, 0.170)), origin=Origin(xyz=(0.405, -0.390, 0.510)), material=safety_yellow, name="lockout_mount_strap")
    chassis.visual(Box((0.075, 0.008, 0.085)), origin=Origin(xyz=(0.455, -0.352, 0.480)), material=safety_yellow, name="lockout_cheek_0")
    chassis.visual(Box((0.075, 0.008, 0.085)), origin=Origin(xyz=(0.455, -0.288, 0.480)), material=safety_yellow, name="lockout_cheek_1")
    chassis.visual(Box((0.045, 0.022, 0.040)), origin=Origin(xyz=(0.350, -0.370, 0.595)), material=safety_yellow, name="lock_stop_upper")
    chassis.visual(Box((0.045, 0.022, 0.040)), origin=Origin(xyz=(0.565, -0.370, 0.390)), material=safety_yellow, name="lock_stop_lower")
    _add_tube(chassis, "lockout_support_strut", (0.405, -0.390, 0.438), (0.280, -0.300, 0.540), 0.012, safety_yellow)
    _add_tube(chassis, "lock_stop_spine", (0.350, -0.370, 0.595), (0.565, -0.370, 0.390), 0.010, safety_yellow)
    _add_tube(chassis, "lock_stop_tie", (0.455, -0.352, 0.480), (0.455, -0.370, 0.495), 0.009, safety_yellow)

    for x in (-0.380, 0.280):
        for y in (-0.250, 0.250):
            _add_disc(chassis, f"saddle_bolt_{x}_{y}", (x, y, 0.565), 0.014, 0.006, zinc, along_y=False)

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.300,
                0.140,
                inner_radius=0.220,
                carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
                tread=TireTread(style="block", depth=0.014, count=24, land_ratio=0.55),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.013, radius=0.004),
            ),
            "tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.215,
                0.105,
                rim=WheelRim(inner_radius=0.145, flange_height=0.016, flange_thickness=0.006, bead_seat_depth=0.006),
                hub=WheelHub(
                    radius=0.055,
                    width=0.085,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.074, hole_diameter=0.008),
                ),
                face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.006, window_radius=0.020),
                bore=WheelBore(style="round", diameter=0.050),
            ),
            "steel_wheel",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=galvanized,
        name="steel_wheel",
    )

    brake_latch = model.part("brake_latch")
    _add_disc(brake_latch, "pivot_barrel", (0.0, 0.0, 0.0), 0.017, 0.056, dark_steel)
    _add_tube(brake_latch, "red_handle", (0.000, 0.000, 0.000), (-0.205, 0.000, 0.160), 0.012, red_lock)
    brake_latch.visual(
        Box((0.060, 0.030, 0.040)),
        origin=Origin(xyz=(-0.235, 0.0, 0.180), rpy=(0.0, 0.0, -0.20)),
        material=rubber,
        name="handle_grip",
    )
    _add_tube(brake_latch, "pawl_link", (0.000, 0.000, 0.000), (0.095, 0.000, -0.020), 0.010, red_lock)
    brake_latch.visual(Box((0.075, 0.025, 0.025)), origin=Origin(xyz=(0.120, 0.0, -0.035), rpy=(0.0, 0.45, 0.0)), material=red_lock, name="pawl_tooth")

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=(0.740, 0.0, 0.312)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )

    model.articulation(
        "lockout_pivot",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=brake_latch,
        origin=Origin(xyz=(0.455, -0.320, 0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    wheel = object_model.get_part("wheel")
    brake_latch = object_model.get_part("brake_latch")
    axle_spin = object_model.get_articulation("axle_spin")
    lockout_pivot = object_model.get_articulation("lockout_pivot")

    ctx.allow_overlap(
        chassis,
        wheel,
        elem_a="axle",
        elem_b="steel_wheel",
        reason="The fixed axle is intentionally captured through the modeled wheel hub/bearing bore.",
    )
    ctx.expect_overlap(
        chassis,
        wheel,
        axes="xyz",
        elem_a="axle",
        elem_b="steel_wheel",
        min_overlap=0.045,
        name="axle remains captured through wheel hub",
    )

    ctx.check(
        "wheel rotates on a visible transverse axle",
        axle_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(axle_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={axle_spin.articulation_type}, axis={axle_spin.axis}",
    )
    ctx.check(
        "lockout latch has limited over-travel",
        lockout_pivot.motion_limits is not None
        and lockout_pivot.motion_limits.lower == 0.0
        and 0.70 <= lockout_pivot.motion_limits.upper <= 0.80,
        details=f"limits={lockout_pivot.motion_limits}",
    )

    tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    foot_aabb = ctx.part_element_world_aabb(chassis, elem="foot_pad_0")
    ctx.check(
        "wheel and rear foot define a stable ground stance",
        tire_aabb is not None
        and foot_aabb is not None
        and abs(tire_aabb[0][2] - foot_aabb[0][2]) <= 0.010
        and tire_aabb[0][0] > foot_aabb[0][0] + 1.2,
        details=f"tire_aabb={tire_aabb}, foot_aabb={foot_aabb}",
    )

    rest_pawl = ctx.part_element_world_aabb(brake_latch, elem="pawl_tooth")
    with ctx.pose({lockout_pivot: lockout_pivot.motion_limits.upper}):
        engaged_pawl = ctx.part_element_world_aabb(brake_latch, elem="pawl_tooth")
    ctx.check(
        "lockout pawl swings downward toward the wheel",
        rest_pawl is not None
        and engaged_pawl is not None
        and engaged_pawl[0][2] < rest_pawl[0][2] - 0.025,
        details=f"rest={rest_pawl}, engaged={engaged_pawl}",
    )

    return ctx.report()


object_model = build_object_model()
