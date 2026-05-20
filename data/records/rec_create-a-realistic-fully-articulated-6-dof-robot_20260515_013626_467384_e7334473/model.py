from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 40, *, center=(0.0, 0.0)):
    cx, cy = center
    return [
        (
            cx + math.cos(2.0 * math.pi * i / segments) * radius,
            cy + math.sin(2.0 * math.pi * i / segments) * radius,
        )
        for i in range(segments)
    ]


def _rounded_rect_section(width: float, depth: float, z: float, radius: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=5)]


def _add_top_bolt_grid(part, mat, *, xs, ys, z, radius=0.014, height=0.010, prefix="bolt"):
    for ix, x in enumerate(xs):
        for iy, y in enumerate(ys):
            part.visual(
                Cylinder(radius=radius, length=height),
                origin=Origin(xyz=(x, y, z)),
                material=mat,
                name=f"{prefix}_{ix}_{iy}",
            )


def _add_side_bolts_x(part, mat, *, x, ys, zs, radius=0.012, height=0.008, prefix="side_bolt"):
    # Cylinder is along local Z by default; rotating about Y places its axis on X.
    for iy, y in enumerate(ys):
        for iz, z in enumerate(zs):
            part.visual(
                Cylinder(radius=radius, length=height),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=mat,
                name=f"{prefix}_{iy}_{iz}",
            )


def _add_band_boxes(part, mat, *, zs, width, depth, thickness, prefix):
    for i, z in enumerate(zs):
        part.visual(
            Box((width + 0.010, depth + 0.010, thickness)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=mat,
            name=f"{prefix}_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="factory_6dof_robot_arm")

    anthracite = model.material("anthracite_grey", rgba=(0.11, 0.12, 0.13, 1.0))
    brushed = model.material("brushed_dark_metal", rgba=(0.38, 0.39, 0.38, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.011, 0.012, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.018, 0.018, 0.017, 1.0))
    orange = model.material("safety_orange", rgba=(1.0, 0.38, 0.04, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    cable = model.material("cable_bundle_black", rgba=(0.0, 0.0, 0.0, 1.0))
    sticker = model.material("calibration_white", rgba=(0.94, 0.93, 0.86, 1.0))
    translucent = model.material("smoked_transparent_cover", rgba=(0.55, 0.82, 0.95, 0.38))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.66, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=anthracite,
        name="mounting_plate",
    )
    base.visual(
        Box((0.76, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.305, 0.080)),
        material=anthracite,
        name="front_stiffener",
    )
    base.visual(
        Box((0.76, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.305, 0.080)),
        material=anthracite,
        name="rear_stiffener",
    )
    base.visual(
        Box((0.075, 0.55, 0.045)),
        origin=Origin(xyz=(-0.34, 0.0, 0.082)),
        material=anthracite,
        name="side_stiffener_0",
    )
    base.visual(
        Box((0.075, 0.55, 0.045)),
        origin=Origin(xyz=(0.34, 0.0, 0.082)),
        material=anthracite,
        name="side_stiffener_1",
    )
    _add_top_bolt_grid(
        base,
        brushed,
        xs=(-0.31, 0.31),
        ys=(-0.23, 0.23),
        z=0.093,
        radius=0.020,
        height=0.012,
        prefix="anchor_bolt",
    )
    # Black cable-exit ports and a bolted terminal block on the front edge.
    base.visual(
        Box((0.32, 0.030, 0.070)),
        origin=Origin(xyz=(-0.06, -0.325, 0.060)),
        material=black,
        name="cable_port",
    )
    base.visual(
        Box((0.25, 0.045, 0.038)),
        origin=Origin(xyz=(0.10, -0.302, 0.075)),
        material=brushed,
        name="terminal_block",
    )
    for i, x in enumerate((-0.18, -0.12, -0.06, 0.00, 0.06)):
        geom = tube_from_spline_points(
            [(x, -0.336, 0.065), (x + 0.03, -0.42, 0.050), (x + 0.12, -0.48, 0.040)],
            radius=0.007,
            samples_per_segment=8,
            radial_segments=12,
        )
        base.visual(mesh_from_geometry(geom, f"base_cable_{i}"), material=cable, name=f"base_cable_{i}")

    yaw = model.part("yaw")
    yaw.visual(
        Cylinder(radius=0.265, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed,
        name="turntable_disk",
    )
    yaw.visual(
        Cylinder(radius=0.225, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=anthracite,
        name="servo_housing",
    )
    yaw.visual(
        Box((0.34, 0.28, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=anthracite,
        name="yaw_motor_box",
    )
    yaw.visual(
        Box((0.27, 0.060, 0.075)),
        origin=Origin(xyz=(0.0, -0.150, 0.110)),
        material=black,
        name="yaw_cable_exit",
    )
    _add_top_bolt_grid(yaw, brushed, xs=(-0.17, 0.17), ys=(-0.17, 0.17), z=0.035, radius=0.012, height=0.012, prefix="turntable_bolt")
    # Shoulder yoke cheeks mounted to the yaw housing.
    yaw.visual(
        Box((0.055, 0.32, 0.30)),
        origin=Origin(xyz=(-0.205, 0.0, 0.280)),
        material=anthracite,
        name="shoulder_cheek_0",
    )
    yaw.visual(
        Box((0.055, 0.32, 0.30)),
        origin=Origin(xyz=(0.205, 0.0, 0.280)),
        material=anthracite,
        name="shoulder_cheek_1",
    )
    yaw.visual(
        Box((0.47, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, 0.145, 0.150)),
        material=anthracite,
        name="shoulder_bridge",
    )
    yaw.visual(
        Cylinder(radius=0.062, length=0.075),
        origin=Origin(xyz=(-0.232, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="shoulder_bearing_0",
    )
    yaw.visual(
        Cylinder(radius=0.062, length=0.075),
        origin=Origin(xyz=(0.232, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="shoulder_bearing_1",
    )
    _add_side_bolts_x(yaw, brushed, x=-0.236, ys=(-0.105, 0.105), zs=(0.205, 0.300, 0.395), prefix="shoulder_bolt_0")
    _add_side_bolts_x(yaw, brushed, x=0.236, ys=(-0.105, 0.105), zs=(0.205, 0.300, 0.395), prefix="shoulder_bolt_1")
    # Two live-looking hydraulic lines from the yaw box into the shoulder cheeks.
    for i, x in enumerate((-0.055, 0.055)):
        geom = tube_from_spline_points(
            [(x, -0.145, 0.140), (x, -0.195, 0.245), (x * 1.6, -0.135, 0.305)],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=14,
        )
        yaw.visual(mesh_from_geometry(geom, f"hydraulic_line_{i}"), material=cable, name=f"hydraulic_line_{i}")
        yaw.visual(
            Box((0.035, 0.020, 0.018)),
            origin=Origin(xyz=(x, -0.143, 0.140)),
            material=brushed,
            name=f"hose_clamp_{i}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.112, length=0.356),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anthracite,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Cylinder(radius=0.138, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="shoulder_boot",
    )
    upper_arm.visual(
        Box((0.25, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=anthracite,
        name="shoulder_saddle",
    )
    upper_mesh = LoftGeometry(
        [
            _rounded_rect_section(0.220, 0.155, 0.100, 0.020),
            _rounded_rect_section(0.185, 0.135, 0.420, 0.018),
            _rounded_rect_section(0.145, 0.105, 0.680, 0.014),
        ],
        cap=True,
        closed=True,
    )
    upper_arm.visual(mesh_from_geometry(upper_mesh, "upper_arm_taper"), material=anthracite, name="tapered_body")
    upper_arm.visual(
        Box((0.042, 0.018, 0.500)),
        origin=Origin(xyz=(0.0, -0.075, 0.390)),
        material=black,
        name="inner_cable_channel",
    )
    _add_band_boxes(upper_arm, orange, zs=(0.235, 0.445, 0.655), width=0.186, depth=0.132, thickness=0.024, prefix="safety_band")
    for i, z in enumerate((0.28, 0.52)):
        geom = tube_from_spline_points(
            [(-0.070, -0.079, z - 0.12), (-0.075, -0.079, z), (-0.060, -0.079, z + 0.13)],
            radius=0.0045,
            samples_per_segment=8,
            radial_segments=10,
        )
        upper_arm.visual(mesh_from_geometry(geom, f"upper_wire_{i}"), material=cable, name=f"upper_wire_{i}")

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.105, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anthracite,
        name="elbow_actuator",
    )
    # Visible reduction gear behind a translucent cover.
    forearm.visual(
        Cylinder(radius=0.073, length=0.022),
        origin=Origin(xyz=(0.0, -0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="elbow_gear_disk",
    )
    for i in range(16):
        a = 2.0 * math.pi * i / 16
        forearm.visual(
            Box((0.012, 0.012, 0.030)),
            origin=Origin(
                xyz=(math.cos(a) * 0.082, -0.072, math.sin(a) * 0.082),
                rpy=(0.0, a, 0.0),
            ),
            material=brushed,
            name=f"gear_tooth_{i}",
        )
    forearm.visual(
        Cylinder(radius=0.092, length=0.012),
        origin=Origin(xyz=(0.0, -0.088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=translucent,
        name="gear_cover",
    )
    forearm.visual(
        Box((0.25, 0.045, 0.120)),
        origin=Origin(xyz=(0.0, 0.100, 0.000)),
        material=anthracite,
        name="elbow_mount_bracket",
    )
    _add_side_bolts_x(forearm, brushed, x=-0.116, ys=(0.095,), zs=(-0.040, 0.040), prefix="elbow_bolt_0")
    _add_side_bolts_x(forearm, brushed, x=0.116, ys=(0.095,), zs=(-0.040, 0.040), prefix="elbow_bolt_1")
    fore_mesh = LoftGeometry(
        [
            _rounded_rect_section(0.150, 0.105, 0.075, 0.014),
            _rounded_rect_section(0.125, 0.090, 0.330, 0.012),
            _rounded_rect_section(0.100, 0.074, 0.590, 0.010),
        ],
        cap=True,
        closed=True,
    )
    forearm.visual(mesh_from_geometry(fore_mesh, "forearm_taper"), material=anthracite, name="slim_body")
    forearm.visual(
        Box((0.035, 0.016, 0.455)),
        origin=Origin(xyz=(0.055, -0.044, 0.340)),
        material=translucent,
        name="clear_cable_conduit",
    )
    for i, x in enumerate((0.047, 0.055, 0.063)):
        geom = tube_from_spline_points(
            [(x, -0.044, 0.110), (x, -0.046, 0.340), (x, -0.043, 0.575)],
            radius=0.0023,
            samples_per_segment=6,
            radial_segments=8,
        )
        forearm.visual(mesh_from_geometry(geom, f"internal_wire_{i}"), material=cable, name=f"internal_wire_{i}")
    for i, z in enumerate((0.190, 0.380, 0.560)):
        forearm.visual(
            Box((0.040, 0.003, 0.026)),
            origin=Origin(xyz=(-0.052, -0.039, z)),
            material=sticker,
            name=f"calibration_sticker_{i}",
        )
        forearm.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(-0.052, -0.043, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"target_dot_{i}",
        )

    wrist_roll = model.part("wrist_roll")
    wrist_roll.visual(
        Cylinder(radius=0.082, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=anthracite,
        name="roll_motor",
    )
    wrist_roll.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=rubber,
        name="roll_boot",
    )
    wrist_roll.visual(
        Box((0.095, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.060, 0.040)),
        material=black,
        name="encoder_housing",
    )
    wrist_roll.visual(
        Box((0.055, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, -0.075, 0.045)),
        material=brushed,
        name="fluid_manifold",
    )
    for i, x in enumerate((-0.020, 0.020)):
        geom = tube_from_spline_points(
            [(x, -0.090, 0.030), (x, -0.110, 0.075), (x * 0.8, -0.058, 0.105)],
            radius=0.0032,
            samples_per_segment=8,
            radial_segments=8,
        )
        wrist_roll.visual(mesh_from_geometry(geom, f"fluid_line_{i}"), material=cable, name=f"fluid_line_{i}")

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=brushed,
        name="pitch_coupler",
    )
    wrist_pitch.visual(
        Box((0.160, 0.054, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anthracite,
        name="pitch_yoke",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.058, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="pitch_trunnion",
    )
    wrist_pitch.visual(
        Box((0.130, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=anthracite,
        name="pitch_motor_case",
    )
    _add_side_bolts_x(wrist_pitch, brushed, x=-0.085, ys=(-0.022, 0.022), zs=(-0.020, 0.020), prefix="pitch_bolt_0")
    _add_side_bolts_x(wrist_pitch, brushed, x=0.085, ys=(-0.022, 0.022), zs=(-0.020, 0.020), prefix="pitch_bolt_1")

    wrist_yaw = model.part("wrist_yaw")
    wrist_yaw.visual(
        Cylinder(radius=0.064, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=anthracite,
        name="yaw_cartridge",
    )
    wrist_yaw.visual(
        Cylinder(radius=0.054, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=rubber,
        name="yaw_boot",
    )
    flange_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.040, 64),
        [_circle_profile(0.0032, 18, center=(math.cos(a) * 0.025, math.sin(a) * 0.025)) for a in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)]
        + [_circle_profile(0.012, 32)],
        0.014,
        center=True,
        cap=True,
        closed=True,
    )
    wrist_yaw.visual(
        mesh_from_geometry(flange_geom, "iso_9409_a50_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed,
        name="tool_flange",
    )
    # Orange annular label outside the actual 50 mm M6 hole pattern.
    label_ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.043, 64),
        [_circle_profile(0.034, 48)],
        0.004,
        center=True,
        cap=True,
        closed=True,
    )
    wrist_yaw.visual(
        mesh_from_geometry(label_ring_geom, "flange_orange_annulus"),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=orange,
        name="flange_label_ring",
    )
    for i, a in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        wrist_yaw.visual(
            Cylinder(radius=0.0032, length=0.006),
            origin=Origin(xyz=(math.cos(a) * 0.025, math.sin(a) * 0.025, 0.088)),
            material=black,
            name=f"m6_hole_marker_{i}",
        )
    wrist_yaw.visual(
        Box((0.025, 0.018, 0.050)),
        origin=Origin(xyz=(0.057, 0.0, 0.040)),
        material=brushed,
        name="air_feedthrough",
    )
    wrist_yaw.visual(
        Box((0.022, 0.016, 0.046)),
        origin=Origin(xyz=(-0.057, 0.0, 0.040)),
        material=brushed,
        name="encoder_plug",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "shoulder_roll",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=360.0, velocity=1.1, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.4, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_roll,
        origin=Origin(xyz=(0.0, 0.0, 0.6325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=3.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=wrist_roll,
        child=wrist_pitch,
        origin=Origin(xyz=(0.0, 0.0, 0.163)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=3.0, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "wrist_yaw",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=wrist_yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = (
        "base_yaw",
        "shoulder_roll",
        "elbow_pitch",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    )
    ctx.check(
        "six revolute axes",
        len(object_model.articulations) == 6
        and all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in joint_names),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    expected_axes = {
        "base_yaw": (0.0, 0.0, 1.0),
        "shoulder_roll": (1.0, 0.0, 0.0),
        "elbow_pitch": (1.0, 0.0, 0.0),
        "wrist_roll": (0.0, 0.0, 1.0),
        "wrist_pitch": (1.0, 0.0, 0.0),
        "wrist_yaw": (0.0, 1.0, 0.0),
    }
    for name, axis in expected_axes.items():
        actual = tuple(round(v, 6) for v in object_model.get_articulation(name).axis)
        ctx.check(f"{name} axis", actual == axis, details=f"expected={axis}, actual={actual}")

    flange = object_model.get_part("wrist_yaw")
    ctx.check(
        "iso a50 four m6 markers",
        all(flange.get_visual(f"m6_hole_marker_{i}") is not None for i in range(4)),
        details="tool flange includes four 6.4 mm through-hole markers on a 50 mm bolt circle",
    )
    ctx.expect_overlap(
        "wrist_yaw",
        "wrist_yaw",
        axes="xy",
        elem_a="tool_flange",
        elem_b="flange_label_ring",
        min_overlap=0.030,
        name="flange label surrounds iso pattern",
    )

    return ctx.report()


object_model = build_object_model()
