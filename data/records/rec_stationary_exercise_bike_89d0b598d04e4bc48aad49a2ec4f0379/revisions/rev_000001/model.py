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
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _tube_origin_between(
    p0: tuple[float, float, float], p1: tuple[float, float, float]
) -> tuple[Origin, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_tube(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    origin, length = _tube_origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _circle_profile(
    cx: float, cy: float, radius: float, *, segments: int = 96, reverse: bool = False
) -> list[tuple[float, float]]:
    pts = [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]
    if reverse:
        pts.reverse()
    return pts


def _capsule_profile(
    p0: tuple[float, float], p1: tuple[float, float], radius: float, *, segments: int = 24
) -> list[tuple[float, float]]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.sqrt(dx * dx + dy * dy)
    ux = dx / length
    uy = dy / length
    nx = -uy
    ny = ux
    pts: list[tuple[float, float]] = []
    # semicircle around p1 from the left side of the centerline to the right side
    base = math.atan2(ny, nx)
    for i in range(segments + 1):
        a = base - math.pi * i / segments
        pts.append((p1[0] + radius * math.cos(a), p1[1] + radius * math.sin(a)))
    # semicircle around p0 back to the starting side
    base = math.atan2(-ny, -nx)
    for i in range(segments + 1):
        a = base - math.pi * i / segments
        pts.append((p0[0] + radius * math.cos(a), p0[1] + radius * math.sin(a)))
    return pts


def _saddle_mesh():
    # A low, padded saddle: narrow nose, wider rear, and a slightly crowned top.
    sections = [
        (-0.155, 0.050, 0.028),
        (-0.080, 0.082, 0.038),
        (0.030, 0.145, 0.052),
        (0.135, 0.190, 0.058),
    ]
    loops = []
    for x, width, thick in sections:
        loop = []
        for i in range(28):
            a = 2.0 * math.pi * i / 28
            lateral = 0.5 * width * math.cos(a)
            # Flatten the underside and crown the upper half.
            height = 0.425 + 0.5 * thick * math.sin(a)
            if math.sin(a) < -0.35:
                height = 0.425 - 0.20 * thick
            # LoftGeometry expects each profile to be an XY loop at constant Z.
            # Author the section as (lateral, height, length), then remap axes.
            loop.append((lateral, height, x))
        loops.append(loop)

    from sdk import LoftGeometry

    geom = LoftGeometry(loops, cap=True, closed=True)
    geom.vertices = [(z, x, y) for (x, y, z) in geom.vertices]
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_stationary_exercise_bike")

    powder_black = model.material("powder_coated_black", rgba=(0.02, 0.024, 0.025, 1.0))
    charcoal = model.material("molded_charcoal", rgba=(0.065, 0.070, 0.074, 1.0))
    graphite = model.material("textured_graphite", rgba=(0.13, 0.14, 0.14, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    fastener_zinc = model.material("zinc_fasteners", rgba=(0.78, 0.76, 0.70, 1.0))
    warning_yellow = model.material("safety_yellow", rgba=(0.95, 0.66, 0.04, 1.0))

    frame = model.part("frame")

    # Wide utility-style stabilizer rails with rubber feet and cross braces.
    for y in (-0.255, 0.255):
        frame.visual(
            Box((1.34, 0.075, 0.070)),
            origin=Origin(xyz=(0.02, y, 0.055)),
            material=powder_black,
            name=f"base_rail_{0 if y < 0 else 1}",
        )
        for x in (-0.625, 0.665):
            frame.visual(
                Box((0.115, 0.092, 0.025)),
                origin=Origin(xyz=(x, y, 0.018)),
                material=dark_rubber,
                name=f"rubber_foot_{0 if y < 0 else 1}_{0 if x < 0 else 1}",
            )
    for x, name in ((-0.58, "front_crossbar"), (0.59, "rear_crossbar"), (0.04, "center_crossbar")):
        frame.visual(
            Box((0.115, 0.630, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=powder_black,
            name=name,
        )

    # Main welded tubular frame: heavy-wall tubes deliberately overlap at the lugs.
    _add_tube(frame, "main_spine", (-0.58, 0.0, 0.100), (0.43, 0.0, 0.355), 0.032, powder_black)
    for y in (-0.180, 0.180):
        _add_tube(frame, f"seat_stay_{0 if y < 0 else 1}", (0.18, y, 0.180), (0.32, y, 0.835), 0.034, powder_black)
    _add_tube(frame, "head_lower", (-0.62, 0.0, 0.100), (-0.80, 0.0, 0.470), 0.033, powder_black)
    _add_tube(frame, "head_upper", (-0.80, 0.0, 0.470), (-0.55, 0.0, 1.075), 0.033, powder_black)
    _add_tube(frame, "top_tube_0", (-0.55, -0.150, 0.925), (0.32, -0.150, 0.835), 0.028, powder_black)
    _add_tube(frame, "top_tube_1", (-0.55, 0.150, 0.925), (0.32, 0.150, 0.835), 0.028, powder_black)
    for y in (-0.090, 0.090):
        _add_tube(frame, f"flywheel_fork_{0 if y < 0 else 1}", (-0.60, y, 0.115), (-0.42, y, 0.455), 0.024, powder_black)
    for y in (-0.255, 0.255):
        _add_tube(frame, f"crank_strut_{0 if y < 0 else 1}", (0.05, y, 0.105), (0.02, y, 0.430), 0.023, powder_black)

    # Thick welded gussets and serviceable clamp blocks.
    frame.visual(Box((0.150, 0.360, 0.075)), origin=Origin(xyz=(0.00, 0.0, 0.485)), material=powder_black, name="crank_lug")
    for y, suffix in ((-0.160, "0"), (0.160, "1")):
        frame.visual(
            Box((0.120, 0.095, 0.080)),
            origin=Origin(xyz=(-0.42, y, 0.455)),
            material=powder_black,
            name=f"flywheel_lug_{suffix}",
        )
    frame.visual(Box((0.125, 0.420, 0.070)), origin=Origin(xyz=(0.32, 0.0, 0.835)), material=powder_black, name="seat_lug")
    frame.visual(Box((0.110, 0.340, 0.065)), origin=Origin(xyz=(-0.55, 0.0, 1.075)), material=powder_black, name="handlebar_lug")
    frame.visual(Box((0.082, 0.090, 0.082)), origin=Origin(xyz=(-0.80, 0.0, 0.470)), material=powder_black, name="head_bend_lug")

    # Hollow-looking square sleeve for the sliding saddle post.
    frame.visual(
        Box((0.092, 0.092, 0.500)),
        origin=Origin(xyz=(0.32, 0.0, 0.590)),
        material=powder_black,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.122, 0.112, 0.030)),
        origin=Origin(xyz=(0.32, 0.0, 0.845)),
        material=satin_steel,
        name="seat_clamp_collar",
    )
    frame.visual(
        Box((0.270, 0.110, 0.170)),
        origin=Origin(xyz=(0.030, 0.0, 0.365)),
        material=powder_black,
        name="center_web",
    )

    # Bottom bracket bearing collars kept outside the rotating crank axle.
    frame.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, -0.104, 0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="crank_bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.104, 0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="crank_bearing_1",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(-0.42, -0.060, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="flywheel_bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(-0.42, 0.060, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="flywheel_bearing_1",
    )

    # Flywheel guard side rings and standoffs.  The annular panels leave the flywheel face visible.
    outer = _circle_profile(-0.42, 0.455, 0.335, segments=104)
    inner = _circle_profile(-0.42, 0.455, 0.258, segments=104, reverse=True)
    guard_mesh = ExtrudeWithHolesGeometry(outer, (inner,), 0.018, center=True).rotate_x(math.pi / 2.0)
    frame.visual(
        mesh_from_geometry(guard_mesh.copy(), "flywheel_guard_panel_0"),
        origin=Origin(xyz=(0.0, 0.063, 0.0)),
        material=charcoal,
        name="guard_panel_0",
    )
    frame.visual(
        mesh_from_geometry(guard_mesh.copy(), "flywheel_guard_panel_1"),
        origin=Origin(xyz=(0.0, -0.063, 0.0)),
        material=charcoal,
        name="guard_panel_1",
    )

    for i, angle in enumerate((math.radians(28), math.radians(84), math.radians(156), math.radians(232), math.radians(306))):
        x = -0.42 + 0.306 * math.cos(angle)
        z = 0.455 + 0.306 * math.sin(angle)
        frame.visual(
            Cylinder(radius=0.012, length=0.150),
            origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener_zinc,
            name=f"guard_standoff_{i}",
        )
        for y in (-0.078, 0.078):
            frame.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=fastener_zinc,
                name=f"guard_bolt_{i}_{0 if y < 0 else 1}",
            )

    belt_profile = _capsule_profile((-0.42, 0.455), (0.00, 0.485), 0.135, segments=28)
    belt_guard = ExtrudeGeometry(belt_profile, 0.034, center=True).rotate_x(math.pi / 2.0)
    frame.visual(
        mesh_from_geometry(belt_guard, "side_belt_guard"),
        origin=Origin(xyz=(0.0, -0.550, 0.0)),
        material=charcoal,
        name="belt_guard",
    )
    for x, z in ((-0.30, 0.462), (-0.10, 0.478)):
        frame.visual(
            Cylinder(radius=0.016, length=0.055),
            origin=Origin(xyz=(x, -0.585, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener_zinc,
            name=f"belt_guard_boss_{0 if x < -0.2 else 1}",
        )
    _add_tube(frame, "belt_guard_mount", (-0.42, -0.550, 0.455), (-0.42, -0.160, 0.455), 0.010, fastener_zinc)

    # Rugged handlebars, over-molded grips, and an exposed resistance-knob bracket.
    _add_tube(frame, "handlebar_riser", (-0.55, 0.0, 1.045), (-0.42, 0.0, 1.170), 0.026, powder_black)
    _add_tube(frame, "handlebar_cross", (-0.42, -0.250, 1.170), (-0.42, 0.250, 1.170), 0.024, powder_black)
    for y, suffix in ((-0.315, "0"), (0.315, "1")):
        _add_tube(frame, f"bar_grip_{suffix}", (-0.42, y * 0.78, 1.170), (-0.42, y, 1.170), 0.030, dark_rubber)
    frame.visual(Box((0.088, 0.044, 0.062)), origin=Origin(xyz=(-0.40, 0.235, 0.870)), material=charcoal, name="resistance_mount")
    _add_tube(frame, "resistance_bracket", (-0.455, 0.158, 0.895), (-0.455, 0.225, 0.870), 0.012, powder_black)
    _add_tube(frame, "tension_cable", (-0.40, 0.230, 0.840), (-0.06, 0.060, 0.525), 0.004, dark_rubber)

    # Exposed bolt heads at high-load lugs and saddle collar.
    for x, z, base in ((0.0, 0.535, "crank"), (-0.42, 0.520, "flywheel"), (0.32, 0.845, "seat")):
        for y in (-0.058, 0.058):
            frame.visual(
                Cylinder(radius=0.013, length=0.008),
                origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=fastener_zinc,
                name=f"{base}_lug_bolt_{0 if y < 0 else 1}",
            )

    # Rotating flywheel, detailed as a serviceable metal wheel inside the guard.
    flywheel = model.part("flywheel")
    flywheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.235,
                0.055,
                rim=WheelRim(inner_radius=0.175, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.003),
                hub=WheelHub(
                    radius=0.055,
                    width=0.060,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.070, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.006, window_radius=0.018),
                bore=WheelBore(style="round", diameter=0.020),
            ),
            "weighted_flywheel",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=graphite,
        name="flywheel_wheel",
    )
    flywheel.visual(
        Cylinder(radius=0.026, length=0.105),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="flywheel_axle",
    )

    # Crank axle, arms, pins, and a visible drive pulley.
    crank = model.part("crank")
    crank.visual(Cylinder(radius=0.024, length=0.420), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="crank_axle")
    crank.visual(
        Cylinder(radius=0.105, length=0.026),
        origin=Origin(xyz=(0.0, -0.400, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="drive_pulley",
    )
    _add_tube(crank, "crank_root_0", (0.0, 0.210, 0.0), (0.0, 0.339, 0.0), 0.012, satin_steel)
    _add_tube(crank, "crank_root_1", (0.0, -0.210, 0.0), (0.0, -0.339, 0.0), 0.012, satin_steel)
    _add_tube(crank, "pulley_collar", (0.0, -0.339, 0.0), (0.0, -0.400, 0.0), 0.015, satin_steel)
    _add_tube(crank, "crank_arm_0", (0.0, 0.339, 0.0), (0.0, 0.339, -0.180), 0.012, satin_steel)
    _add_tube(crank, "crank_arm_1", (0.0, -0.339, 0.0), (0.0, -0.339, 0.180), 0.012, satin_steel)
    crank.visual(
        Cylinder(radius=0.014, length=0.072),
        origin=Origin(xyz=(0.0, 0.339, -0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pedal_spindle_0",
    )
    crank.visual(Sphere(radius=0.026), origin=Origin(xyz=(0.0, 0.339, -0.180)), material=satin_steel, name="arm_boss_0")
    crank.visual(
        Cylinder(radius=0.014, length=0.072),
        origin=Origin(xyz=(0.0, -0.339, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pedal_spindle_1",
    )
    crank.visual(Sphere(radius=0.026), origin=Origin(xyz=(0.0, -0.339, 0.180)), material=satin_steel, name="arm_boss_1")

    pedal_0 = model.part("pedal_0")
    pedal_0.visual(Box((0.135, 0.062, 0.026)), origin=Origin(xyz=(0.035, 0.040, 0.0)), material=dark_rubber, name="pedal_pad")
    pedal_0.visual(Box((0.120, 0.010, 0.036)), origin=Origin(xyz=(0.035, 0.008, 0.0)), material=warning_yellow, name="pedal_reflector")
    pedal_0.visual(Cylinder(radius=0.010, length=0.050), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="pedal_bushing")

    pedal_1 = model.part("pedal_1")
    pedal_1.visual(Box((0.135, 0.062, 0.026)), origin=Origin(xyz=(0.035, -0.040, 0.0)), material=dark_rubber, name="pedal_pad")
    pedal_1.visual(Box((0.120, 0.010, 0.036)), origin=Origin(xyz=(0.035, -0.008, 0.0)), material=warning_yellow, name="pedal_reflector")
    pedal_1.visual(Cylinder(radius=0.010, length=0.050), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=satin_steel, name="pedal_bushing")

    # Sliding saddle support with hidden retained insertion.
    seat_post = model.part("seat_post")
    seat_post.visual(Box((0.056, 0.056, 0.580)), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=satin_steel, name="inner_mast")
    seat_post.visual(Box((0.140, 0.105, 0.030)), origin=Origin(xyz=(0.025, 0.0, 0.365)), material=powder_black, name="seat_rail_clamp")
    for y in (-0.045, 0.045):
        _add_tube(seat_post, f"saddle_rail_{0 if y < 0 else 1}", (-0.105, y, 0.370), (0.128, y, 0.390), 0.006, satin_steel)
    for x, suffix in ((-0.080, "0"), (0.090, "1")):
        seat_post.visual(
            Box((0.030, 0.120, 0.036)),
            origin=Origin(xyz=(x, 0.0, 0.400)),
            material=powder_black,
            name=f"saddle_support_{suffix}",
        )
    seat_post.visual(mesh_from_geometry(_saddle_mesh(), "padded_saddle"), material=dark_rubber, name="saddle_pad")

    seat_knob = model.part("seat_knob")
    seat_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.070,
                0.033,
                body_style="lobed",
                base_diameter=0.046,
                top_diameter=0.064,
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=10, depth=0.002),
                bore=KnobBore(style="round", diameter=0.010),
                body_reliefs=(KnobRelief(style="top_recess", width=0.024, depth=0.002),),
            ),
            "seat_clamp_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="knob_cap",
    )
    seat_knob.visual(
        Cylinder(radius=0.008, length=0.125),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="knob_stem",
    )

    resistance_knob = model.part("resistance_knob")
    resistance_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.030,
                body_style="faceted",
                base_diameter=0.052,
                top_diameter=0.048,
                edge_radius=0.001,
                grip=KnobGrip(style="knurled", count=24, depth=0.0012, helix_angle_deg=18.0),
                bore=KnobBore(style="round", diameter=0.009),
            ),
            "resistance_knob_cap",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="knob_cap",
    )
    resistance_knob.visual(
        Cylinder(radius=0.007, length=0.105),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="knob_stem",
    )

    crank_joint = model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(-0.42, 0.0, 0.455)),
        axis=(0.0, 1.0, 0.0),
        mimic=Mimic(crank_joint.name, multiplier=3.4, offset=0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=35.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "crank_to_pedal_0",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_0,
        origin=Origin(xyz=(0.0, 0.375, -0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.01),
    )
    model.articulation(
        "crank_to_pedal_1",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_1,
        origin=Origin(xyz=(0.0, -0.375, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.01),
    )
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.32, 0.0, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=180.0, velocity=0.20),
        motion_properties=MotionProperties(damping=5.0, friction=2.0),
    )
    model.articulation(
        "frame_to_seat_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=seat_knob,
        origin=Origin(xyz=(0.32, 0.112, 0.700)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
    )
    model.articulation(
        "frame_to_resistance_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=resistance_knob,
        origin=Origin(xyz=(-0.40, 0.285, 0.870)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.10, friction=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank")
    flywheel = object_model.get_part("flywheel")
    seat_post = object_model.get_part("seat_post")
    seat_knob = object_model.get_part("seat_knob")
    resistance_knob = object_model.get_part("resistance_knob")

    crank_joint = object_model.get_articulation("frame_to_crank")
    seat_slide = object_model.get_articulation("frame_to_seat_post")

    # The mast and control stems intentionally enter simplified solid sleeves/brackets.
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_mast",
        reason="The adjustable saddle mast is intentionally represented as a retained sliding member inside the heavy square sleeve.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_lug",
        elem_b="inner_mast",
        reason="The upper welded saddle lug surrounds the sliding mast and is modeled as a solid collar proxy.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_clamp_collar",
        elem_b="inner_mast",
        reason="The metal clamp collar surrounds the sliding saddle mast at the sleeve mouth.",
    )
    ctx.allow_overlap(
        frame,
        seat_knob,
        elem_a="seat_sleeve",
        elem_b="knob_stem",
        reason="The clamp-knob threaded stem deliberately penetrates the saddle sleeve to show the locking hardware.",
    )
    ctx.allow_overlap(
        frame,
        resistance_knob,
        elem_a="resistance_mount",
        elem_b="knob_stem",
        reason="The resistance knob's threaded stem is seated through the mounted tension bracket.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_lug",
        elem_b="crank_axle",
        reason="The crank axle is intentionally captured through the bottom-bracket bearing lug.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_bearing_0",
        elem_b="crank_axle",
        reason="The crank axle is intentionally nested in the service-side bearing collar.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_bearing_1",
        elem_b="crank_axle",
        reason="The crank axle is intentionally nested in the opposite bearing collar.",
    )
    ctx.allow_overlap(
        crank,
        object_model.get_part("pedal_0"),
        elem_a="pedal_spindle_0",
        elem_b="pedal_bushing",
        reason="The pedal spindle is represented seated inside the pedal bushing.",
    )
    ctx.allow_overlap(
        crank,
        object_model.get_part("pedal_1"),
        elem_a="pedal_spindle_1",
        elem_b="pedal_bushing",
        reason="The pedal spindle is represented seated inside the pedal bushing.",
    )
    ctx.allow_overlap(
        crank,
        object_model.get_part("pedal_0"),
        elem_a="arm_boss_0",
        elem_b="pedal_bushing",
        reason="The pedal bushing is locally captured by the crank-arm boss at the spindle.",
    )
    ctx.allow_overlap(
        crank,
        object_model.get_part("pedal_1"),
        elem_a="arm_boss_1",
        elem_b="pedal_bushing",
        reason="The pedal bushing is locally captured by the crank-arm boss at the spindle.",
    )
    ctx.allow_overlap(
        seat_knob,
        seat_post,
        elem_a="knob_stem",
        elem_b="inner_mast",
        reason="The saddle clamp screw deliberately bears into the sliding mast.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_bearing_0",
        elem_b="flywheel_axle",
        reason="The flywheel axle is intentionally captured in the service-side bearing block.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_bearing_1",
        elem_b="flywheel_axle",
        reason="The flywheel axle is intentionally captured in the opposite bearing block.",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="saddle mast centered in sleeve",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_mast",
        elem_b="seat_sleeve",
        min_overlap=0.18,
        name="lower saddle mast has long retained insertion",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_mast",
        elem_b="seat_lug",
        min_overlap=0.060,
        name="saddle mast passes through upper lug",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_mast",
        elem_b="seat_clamp_collar",
        min_overlap=0.025,
        name="saddle mast passes through clamp collar",
    )
    with ctx.pose({seat_slide: 0.180}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised saddle mast still centered",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_mast",
            elem_b="seat_sleeve",
            min_overlap=0.025,
            name="raised saddle mast retains insertion",
        )

    ctx.expect_overlap(
        seat_knob,
        frame,
        axes="y",
        elem_a="knob_stem",
        elem_b="seat_sleeve",
        min_overlap=0.050,
        name="seat clamp stem passes through sleeve",
    )
    ctx.expect_overlap(
        resistance_knob,
        frame,
        axes="y",
        elem_a="knob_stem",
        elem_b="resistance_mount",
        min_overlap=0.020,
        name="resistance stem enters bracket",
    )

    ctx.expect_overlap(crank, frame, axes="yz", elem_a="crank_axle", elem_b="crank_lug", min_overlap=0.045, name="crank axle carried by bottom bracket")
    ctx.expect_overlap(crank, frame, axes="y", elem_a="crank_axle", elem_b="crank_bearing_0", min_overlap=0.020, name="crank axle seated in bearing 0")
    ctx.expect_overlap(crank, frame, axes="y", elem_a="crank_axle", elem_b="crank_bearing_1", min_overlap=0.020, name="crank axle seated in bearing 1")
    ctx.expect_overlap(
        crank,
        object_model.get_part("pedal_0"),
        axes="y",
        elem_a="pedal_spindle_0",
        elem_b="pedal_bushing",
        min_overlap=0.020,
        name="pedal 0 spindle retained in bushing",
    )
    ctx.expect_overlap(
        crank,
        object_model.get_part("pedal_1"),
        axes="y",
        elem_a="pedal_spindle_1",
        elem_b="pedal_bushing",
        min_overlap=0.020,
        name="pedal 1 spindle retained in bushing",
    )
    ctx.expect_overlap(
        crank,
        object_model.get_part("pedal_0"),
        axes="y",
        elem_a="arm_boss_0",
        elem_b="pedal_bushing",
        min_overlap=0.010,
        name="pedal 0 bushing captured by arm boss",
    )
    ctx.expect_overlap(
        crank,
        object_model.get_part("pedal_1"),
        axes="y",
        elem_a="arm_boss_1",
        elem_b="pedal_bushing",
        min_overlap=0.010,
        name="pedal 1 bushing captured by arm boss",
    )
    ctx.expect_within(flywheel, frame, axes="xz", inner_elem="flywheel_wheel", outer_elem="guard_panel_0", margin=0.080, name="flywheel sits inside guard ring")
    ctx.expect_overlap(flywheel, frame, axes="y", elem_a="flywheel_axle", elem_b="flywheel_bearing_0", min_overlap=0.004, name="flywheel axle seated in bearing 0")
    ctx.expect_overlap(flywheel, frame, axes="y", elem_a="flywheel_axle", elem_b="flywheel_bearing_1", min_overlap=0.004, name="flywheel axle seated in bearing 1")

    low_pos = ctx.part_world_position(object_model.get_part("pedal_0"))
    with ctx.pose({crank_joint: math.pi}):
        high_pos = ctx.part_world_position(object_model.get_part("pedal_0"))
    ctx.check(
        "crank rotation carries pedal around axle",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.30,
        details=f"pedal_0 rest={low_pos}, half_turn={high_pos}",
    )

    return ctx.report()


object_model = build_object_model()
