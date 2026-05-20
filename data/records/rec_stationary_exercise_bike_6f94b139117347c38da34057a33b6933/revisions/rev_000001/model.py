from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    LoftGeometry,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _origin_between(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that points a local +Z cylinder from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return Origin(), 0.0
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux) if abs(ux) + abs(uy) > 1e-9 else 0.0
    return (
        Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _tube(part, p0, p1, radius, *, material: Material | str, name: str) -> None:
    origin, length = _origin_between(p0, p1)
    if length > 0.0:
        part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _saddle_mesh() -> MeshGeometry:
    """Tapered, crowned exercise-bike saddle mesh in local coordinates."""

    sections = [
        (-0.175, 0.036, 0.034, -0.004),
        (-0.110, 0.055, 0.042, 0.003),
        (-0.015, 0.105, 0.054, 0.010),
        (0.105, 0.122, 0.052, 0.006),
        (0.165, 0.092, 0.043, -0.002),
    ]
    loops = []
    samples = 24
    for x, half_width, height, camber in sections:
        loop = []
        for i in range(samples):
            a = 2.0 * math.pi * i / samples
            # Fuller, flatter upper cushion than a true ellipse.
            y = half_width * math.copysign(abs(math.cos(a)) ** 0.72, math.cos(a))
            z = camber + 0.5 * height * math.copysign(abs(math.sin(a)) ** 0.55, math.sin(a))
            # Slight center valley on the top surface, as on a real saddle.
            if z > camber:
                z -= 0.006 * (1.0 - min(1.0, abs(y) / max(half_width, 1e-6)))
            loop.append((x, y, z))
        loops.append(loop)

    geom = MeshGeometry()
    indices = []
    for loop in loops:
        indices.append([geom.add_vertex(*p) for p in loop])
    for a in range(len(indices) - 1):
        for i in range(samples):
            j = (i + 1) % samples
            geom.add_face(indices[a][i], indices[a + 1][i], indices[a + 1][j])
            geom.add_face(indices[a][i], indices[a + 1][j], indices[a][j])
    # Cap the nose and rear with centers so the cushion is a closed solid.
    for loop_index, reverse in ((0, True), (len(indices) - 1, False)):
        loop = loops[loop_index]
        cx = sum(p[0] for p in loop) / samples
        cy = sum(p[1] for p in loop) / samples
        cz = sum(p[2] for p in loop) / samples
        center_idx = geom.add_vertex(cx, cy, cz)
        for i in range(samples):
            j = (i + 1) % samples
            if reverse:
                geom.add_face(center_idx, indices[loop_index][j], indices[loop_index][i])
            else:
                geom.add_face(center_idx, indices[loop_index][i], indices[loop_index][j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_air_bike")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.038, 0.042, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.19, 0.20, 0.21, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.32, 0.33, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.61, 1.0))
    rubber = model.material("textured_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    accent_red = model.material("red_accent", rgba=(0.78, 0.025, 0.018, 1.0))
    display_green = model.material("display_green", rgba=(0.08, 0.45, 0.18, 1.0))

    frame = model.part("frame")

    # Heavy rectangular base with wide stabilizer feet.
    for side, y in (("left", 0.245), ("right", -0.245)):
        _tube(frame, (-0.82, y, 0.070), (0.98, y, 0.070), 0.027, material=matte_black, name=f"{side}_base_rail")
        _tube(frame, (-0.64, y * 0.75, 0.095), (0.055, y * 1.45, 0.338), 0.030, material=matte_black, name=f"{side}_main_down_tube")
        _tube(frame, (0.055, y * 1.45, 0.338), (0.565, y * 0.55, 0.582), 0.026, material=matte_black, name=f"{side}_fan_top_stay")
        _tube(frame, (0.565, y * 0.55, 0.582), (0.925, y * 0.82, 0.098), 0.028, material=matte_black, name=f"{side}_fan_fork")
        _tube(frame, (-0.70, y, 0.070), (-0.50, y * 0.45, 0.500), 0.024, material=matte_black, name=f"{side}_seat_brace")
        _tube(frame, (-0.48, y * 0.40, 0.500), (0.055, y * 1.45, 0.338), 0.022, material=matte_black, name=f"{side}_cross_brace")

    _tube(frame, (0.94, -0.42, 0.060), (0.94, 0.42, 0.060), 0.038, material=matte_black, name="front_stabilizer")
    _tube(frame, (-0.80, -0.40, 0.060), (-0.80, 0.40, 0.060), 0.036, material=matte_black, name="rear_stabilizer")
    _tube(frame, (-0.58, -0.245, 0.080), (-0.58, 0.245, 0.080), 0.026, material=matte_black, name="seat_base_crossbar")
    _tube(frame, (0.055, -0.315, 0.338), (0.055, -0.250, 0.338), 0.042, material=gunmetal, name="bottom_cup_0")
    _tube(frame, (0.055, 0.250, 0.338), (0.055, 0.315, 0.338), 0.042, material=gunmetal, name="bottom_cup_1")
    _tube(frame, (0.565, -0.235, 0.582), (0.565, -0.170, 0.582), 0.032, material=gunmetal, name="fan_bearing_0")
    _tube(frame, (0.565, 0.170, 0.582), (0.565, 0.235, 0.582), 0.032, material=gunmetal, name="fan_bearing_1")
    _tube(frame, (0.565, -0.235, 0.582), (0.565, 0.235, 0.582), 0.018, material=brushed_steel, name="fan_axle_pin")

    # Rubber leveling pads and protective foot caps.
    for i, (x, y) in enumerate(((0.94, 0.435), (0.94, -0.435), (-0.80, 0.415), (-0.80, -0.415))):
        frame.visual(Box((0.185, 0.060, 0.034)), origin=Origin(xyz=(x, y, 0.026)), material=rubber, name=f"foot_pad_{i}")

    # Seat mast, rails, and a properly tapered saddle.
    _tube(frame, (-0.58, 0.0, 0.080), (-0.51, 0.0, 0.165), 0.026, material=matte_black, name="seat_mast_foot")
    _tube(frame, (-0.51, 0.0, 0.165), (-0.535, 0.0, 0.805), 0.034, material=matte_black, name="seat_mast")
    _tube(frame, (-0.535, -0.060, 0.810), (-0.535, 0.060, 0.810), 0.016, material=brushed_steel, name="seat_clamp_bar")
    _tube(frame, (-0.535, 0.0, 0.810), (-0.535, 0.0, 0.842), 0.018, material=brushed_steel, name="seat_clamp_post")
    _tube(frame, (-0.680, -0.048, 0.842), (-0.420, -0.048, 0.842), 0.010, material=brushed_steel, name="seat_rail_0")
    _tube(frame, (-0.680, 0.048, 0.842), (-0.420, 0.048, 0.842), 0.010, material=brushed_steel, name="seat_rail_1")
    frame.visual(Box((0.090, 0.145, 0.030)), origin=Origin(xyz=(-0.535, 0.0, 0.842)), material=gunmetal, name="saddle_clamp")
    frame.visual(Box((0.150, 0.110, 0.018)), origin=Origin(xyz=(-0.555, 0.0, 0.858)), material=rubber, name="saddle_underpad")
    frame.visual(mesh_from_geometry(_saddle_mesh(), "wide_airbike_saddle"), origin=Origin(xyz=(-0.555, 0.0, 0.865)), material=rubber, name="saddle")

    # Fan cage: deep front/rear guards plus many welded radial spokes.
    cage_ring = mesh_from_geometry(TorusGeometry(0.382, 0.014, radial_segments=24, tubular_segments=96), "fan_cage_ring")
    for side, y in (("front", 0.110), ("rear", -0.110)):
        frame.visual(cage_ring, origin=Origin(xyz=(0.565, y, 0.582), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_black, name=f"{side}_cage_ring")
        frame.visual(Cylinder(radius=0.088, length=0.014), origin=Origin(xyz=(0.565, y, 0.582), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=satin_black, name=f"{side}_cage_hub")
        for i in range(16):
            a = 2.0 * math.pi * i / 16
            p0 = (0.565 + 0.072 * math.cos(a), y, 0.582 + 0.072 * math.sin(a))
            p1 = (0.565 + 0.382 * math.cos(a), y, 0.582 + 0.382 * math.sin(a))
            _tube(frame, p0, p1, 0.0055, material=satin_black, name=f"{side}_cage_spoke_{i}")
    for i, a in enumerate((math.radians(28), math.radians(152), math.radians(208), math.radians(332))):
        x = 0.565 + 0.382 * math.cos(a)
        z = 0.582 + 0.382 * math.sin(a)
        _tube(frame, (x, -0.110, z), (x, 0.110, z), 0.010, material=satin_black, name=f"cage_depth_tie_{i}")

    # Belt guard and console are rigidly mounted to the frame.
    _tube(frame, (0.040, -0.525, 0.335), (0.555, -0.525, 0.580), 0.024, material=satin_black, name="belt_upper_guard")
    _tube(frame, (0.040, -0.530, 0.300), (0.555, -0.530, 0.535), 0.014, material=rubber, name="belt_lower_run")
    _tube(frame, (0.052, -0.292, 0.334), (0.040, -0.525, 0.335), 0.010, material=satin_black, name="belt_rear_standoff")
    _tube(frame, (0.565, -0.204, 0.582), (0.555, -0.525, 0.580), 0.010, material=satin_black, name="belt_front_standoff")
    _tube(frame, (0.565, -0.110, 0.944), (0.500, 0.0, 0.930), 0.012, material=matte_black, name="console_yoke_0")
    _tube(frame, (0.565, 0.110, 0.944), (0.500, 0.0, 0.930), 0.012, material=matte_black, name="console_yoke_1")
    _tube(frame, (0.500, 0.0, 0.930), (0.350, 0.0, 1.075), 0.018, material=matte_black, name="console_stem")
    frame.visual(Box((0.230, 0.035, 0.145)), origin=Origin(xyz=(0.300, 0.0, 1.105), rpy=(0.0, 0.25, 0.0)), material=gunmetal, name="console_body")
    frame.visual(Box((0.160, 0.012, 0.075)), origin=Origin(xyz=(0.275, 0.018, 1.105), rpy=(0.0, 0.25, 0.0)), material=display_green, name="console_display")

    # Handlebar pivot brackets just outside the main frame.
    for side, y, sgn in (("left", 0.430, 1.0), ("right", -0.430, -1.0)):
        _tube(frame, (0.055, sgn * 0.355, 0.338), (0.115, y - sgn * 0.030, 0.338), 0.014, material=matte_black, name=f"{side}_pivot_base_tie")
        _tube(frame, (0.115, y - sgn * 0.030, 0.338), (0.185, y - sgn * 0.050, 0.415), 0.018, material=matte_black, name=f"{side}_pivot_support")
        frame.visual(Box((0.075, 0.020, 0.092)), origin=Origin(xyz=(0.210, y - sgn * 0.055, 0.450)), material=dark_steel, name=f"{side}_pivot_plate")
        _tube(frame, (0.210, y - sgn * 0.070, 0.450), (0.210, y - sgn * 0.018, 0.450), 0.020, material=brushed_steel, name=f"{side}_pivot_pin")

    # Fan wheel spins on the front axle.
    fan_wheel = model.part("fan_wheel")
    rotor = FanRotorGeometry(
        0.318,
        0.058,
        10,
        thickness=0.070,
        blade_pitch_deg=34.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=18.0, camber=0.18),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.026, rear_collar_radius=0.052, bore_diameter=0.030),
        shroud=FanRotorShroud(thickness=0.012, depth=0.070, clearance=0.002, lip_depth=0.004),
    )
    fan_wheel.visual(mesh_from_geometry(rotor, "airbike_fan_rotor"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_steel, name="fan_rotor")
    fan_wheel.visual(
        mesh_from_geometry(TorusGeometry(0.323, 0.014, radial_segments=20, tubular_segments=96), "fan_flywheel_rim"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="flywheel_rim",
    )

    # Crank assembly, with opposed arms and a side chainring.
    crank = model.part("crank")
    _tube(crank, (0.0, -0.285, 0.0), (0.0, 0.285, 0.0), 0.021, material=brushed_steel, name="crank_spindle")
    crank.visual(Cylinder(radius=0.116, length=0.018), origin=Origin(xyz=(0.0, -0.166, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=gunmetal, name="chainring")
    crank.visual(Cylinder(radius=0.082, length=0.020), origin=Origin(xyz=(0.0, -0.190, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=accent_red, name="chainring_guard")
    _tube(crank, (0.0, -0.210, 0.0), (0.0, -0.210, -0.178), 0.013, material=brushed_steel, name="crank_arm_0")
    _tube(crank, (0.0, 0.210, 0.0), (0.0, 0.210, 0.178), 0.013, material=brushed_steel, name="crank_arm_1")
    crank.visual(Sphere(0.025), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=brushed_steel, name="crank_hub")

    # Pedals each spin on their spindle at the crank-arm end.
    for idx, (y, z, outward) in enumerate(((-0.210, -0.178, -1.0), (0.210, 0.178, 1.0))):
        pedal = model.part(f"pedal_{idx}")
        _tube(pedal, (0.0, 0.0, 0.0), (0.0, outward * 0.123, 0.0), 0.009, material=brushed_steel, name="pedal_spindle")
        pedal.visual(Box((0.135, 0.070, 0.028)), origin=Origin(xyz=(0.0, outward * 0.145, 0.0)), material=rubber, name="pedal_pad")
        pedal.visual(Box((0.150, 0.012, 0.040)), origin=Origin(xyz=(0.0, outward * 0.145, 0.029)), material=gunmetal, name="toe_cage_bar")
        model.articulation(
            f"crank_to_pedal_{idx}",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=pedal,
            origin=Origin(xyz=(0.0, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )

    # Independent push-pull handlebars with rubber grips and lower bushings.
    handle_joints = []
    for side, y, outward, axis_sign in (("left", 0.430, 1.0, 1.0), ("right", -0.430, -1.0, -1.0)):
        handle = model.part(f"{side}_handlebar")
        handle_path = [
            (0.000, 0.000, 0.000),
            (0.075, 0.000, 0.250),
            (0.020, 0.000, 0.580),
            (-0.135, 0.000, 0.820),
        ]
        handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(handle_path, radius=0.020, samples_per_segment=14, radial_segments=18, cap_ends=True),
                f"{side}_handlebar_tube",
            ),
            material=matte_black,
            name="handle_tube",
        )
        _tube(handle, (-0.135, 0.000, 0.820), (-0.135, outward * 0.155, 0.820), 0.022, material=rubber, name="grip")
        _tube(handle, (0.000, -outward * 0.034, 0.000), (0.000, outward * 0.034, 0.000), 0.030, material=gunmetal, name="pivot_bushing")
        _tube(handle, (0.000, 0.000, 0.010), (0.115, 0.000, -0.155), 0.012, material=dark_steel, name="link_stub")
        joint = model.articulation(
            f"{side}_handle_swing",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=handle,
            origin=Origin(xyz=(0.210, y, 0.450)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=3.0, lower=-0.55, upper=0.55),
        )
        handle_joints.append(joint)

    crank_joint = model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.055, 0.0, 0.338)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=10.0),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=fan_wheel,
        origin=Origin(xyz=(0.565, 0.0, 0.582)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=18.0),
        mimic=Mimic(joint="crank_spin", multiplier=4.0, offset=0.0),
    )

    # Store semantic notes for consumers; handlebar joints remain separately controllable.
    model.meta["mechanisms"] = {
        "crank": crank_joint.name,
        "fan": "fan_spin",
        "handlebars": [joint.name for joint in handle_joints],
        "pedals": ["crank_to_pedal_0", "crank_to_pedal_1"],
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fan = object_model.get_part("fan_wheel")
    crank = object_model.get_part("crank")
    frame = object_model.get_part("frame")
    left_handle = object_model.get_part("left_handlebar")
    right_handle = object_model.get_part("right_handlebar")
    pedal_0 = object_model.get_part("pedal_0")
    pedal_1 = object_model.get_part("pedal_1")

    crank_joint = object_model.get_articulation("crank_spin")
    fan_joint = object_model.get_articulation("fan_spin")
    left_joint = object_model.get_articulation("left_handle_swing")
    right_joint = object_model.get_articulation("right_handle_swing")

    ctx.check("crank is continuous", crank_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("fan is continuous", fan_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("fan follows crank", fan_joint.mimic is not None and fan_joint.mimic.joint == "crank_spin")
    ctx.check(
        "handlebars have push pull limits",
        left_joint.motion_limits is not None
        and right_joint.motion_limits is not None
        and left_joint.motion_limits.lower <= -0.5
        and left_joint.motion_limits.upper >= 0.5
        and right_joint.motion_limits.lower <= -0.5
        and right_joint.motion_limits.upper >= 0.5,
    )
    for idx in (0, 1):
        joint = object_model.get_articulation(f"crank_to_pedal_{idx}")
        ctx.check(f"pedal_{idx} spins on its axle", joint.articulation_type == ArticulationType.CONTINUOUS)

    ctx.allow_overlap(
        frame,
        fan,
        elem_a="fan_axle_pin",
        elem_b="fan_rotor",
        reason="The fixed axle pin is intentionally captured inside the fan hub bearing.",
    )
    ctx.expect_overlap(
        frame,
        fan,
        axes="xyz",
        min_overlap=0.018,
        name="fan hub is captured by fixed axle",
    )
    for side, handle in (("left", left_handle), ("right", right_handle)):
        ctx.allow_overlap(
            frame,
            handle,
            elem_a=f"{side}_pivot_pin",
            elem_b="pivot_bushing",
            reason="The moving handlebar bushing is intentionally retained on a fixed pivot pin.",
        )
        ctx.expect_overlap(
            frame,
            handle,
            axes="xyz",
            min_overlap=0.010,
            name=f"{side} handle bushing is pinned",
        )
    for idx, pedal, arm_name in ((0, pedal_0, "crank_arm_0"), (1, pedal_1, "crank_arm_1")):
        ctx.allow_overlap(
            crank,
            pedal,
            elem_a=arm_name,
            elem_b="pedal_spindle",
            reason="The pedal spindle is intentionally seated through the crank-arm eye.",
        )
        ctx.expect_overlap(
            crank,
            pedal,
            axes="yz",
            min_overlap=0.006,
            name=f"pedal_{idx} spindle seats in crank arm",
        )
    for cup_name in ("bottom_cup_0", "bottom_cup_1"):
        ctx.allow_overlap(
            frame,
            crank,
            elem_a=cup_name,
            elem_b="crank_spindle",
            reason="The rotating crank spindle is intentionally captured inside the bottom-bracket bearing cup.",
        )
        ctx.expect_overlap(
            frame,
            crank,
            axes="xyz",
            min_overlap=0.015,
            name=f"{cup_name} captures crank spindle",
        )

    ctx.expect_origin_distance(fan, crank, axes="xz", min_dist=0.45, max_dist=0.70, name="fan and crank are realistically spaced")
    ctx.expect_origin_gap(left_handle, frame, axis="y", min_gap=0.350, max_gap=0.500, name="left handle sits outside frame")
    ctx.expect_origin_gap(frame, right_handle, axis="y", min_gap=0.350, max_gap=0.500, name="right handle sits outside frame")

    rest_left_aabb = ctx.part_world_aabb(left_handle)
    with ctx.pose({left_joint: 0.45, right_joint: 0.45, crank_joint: 0.9}):
        moved_left_aabb = ctx.part_world_aabb(left_handle)
        moved_right_aabb = ctx.part_world_aabb(right_handle)
        ctx.check(
            "left handle swings through a visible arc",
            rest_left_aabb is not None
            and moved_left_aabb is not None
            and abs(moved_left_aabb[1][0] - rest_left_aabb[1][0]) > 0.020,
            details=f"rest={rest_left_aabb}, moved={moved_left_aabb}",
        )
        ctx.check(
            "opposite handle remains mounted while posed",
            moved_right_aabb is not None and moved_right_aabb[0][2] > 0.25,
            details=f"right_aabb={moved_right_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
