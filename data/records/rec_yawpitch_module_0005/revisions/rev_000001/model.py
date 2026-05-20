from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.300
BASE_W = 0.220
BASE_T = 0.018
PEDESTAL_L = 0.180
PEDESTAL_W = 0.140
PEDESTAL_H = 0.045

YAW_AXIS_Z = BASE_T + PEDESTAL_H + 0.014
STATOR_INNER_R = 0.049
STATOR_OUTER_R = 0.067
STATOR_H = 0.028

PITCH_AXIS_Z = 0.140
TRUNNION_BORE_R = 0.019
TRUNNION_R = 0.019
YOKE_HALF_SPAN_X = 0.078


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length / 2.0)
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (center[0] - length / 2.0, center[1], center[2])
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (center[0], center[1] - length / 2.0, center[2])
    )


def _ring_z(
    inner_radius: float,
    outer_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_z(outer_radius, length, center).cut(_cyl_z(inner_radius, length + 0.002, center))


def _ring_x(
    inner_radius: float,
    outer_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_x(outer_radius, length, center).cut(_cyl_x(inner_radius, length + 0.002, center))


def _bolt_circle_z(
    radius: float,
    bolt_radius: float,
    height: float,
    z_center: float,
    count: int,
    start_angle: float = 0.0,
) -> cq.Workplane:
    hardware = None
    for i in range(count):
        angle = start_angle + i * (math.tau / count)
        bolt = _cyl_z(
            bolt_radius,
            height,
            (radius * math.cos(angle), radius * math.sin(angle), z_center),
        )
        hardware = bolt if hardware is None else hardware.union(bolt)
    assert hardware is not None
    return hardware


def _bolt_circle_x(
    radius: float,
    bolt_radius: float,
    length: float,
    center: tuple[float, float, float],
    count: int,
    start_angle: float = 0.0,
) -> cq.Workplane:
    hardware = None
    for i in range(count):
        angle = start_angle + i * (math.tau / count)
        bolt = _cyl_x(
            bolt_radius,
            length,
            (
                center[0],
                center[1] + radius * math.cos(angle),
                center[2] + radius * math.sin(angle),
            ),
        )
        hardware = bolt if hardware is None else hardware.union(bolt)
    assert hardware is not None
    return hardware


def _build_base_structure() -> cq.Workplane:
    base = _box((BASE_L, BASE_W, BASE_T), (0.0, 0.0, BASE_T / 2.0))
    pedestal = _box(
        (PEDESTAL_L, PEDESTAL_W, PEDESTAL_H),
        (0.0, 0.0, BASE_T + PEDESTAL_H / 2.0),
    )
    pedestal = pedestal.cut(_box((0.092, 0.060, 0.020), (0.0, 0.0, BASE_T + 0.024)))

    structure = base.union(pedestal)

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            structure = structure.union(
                _box(
                    (0.040, 0.032, 0.008),
                    (sx * (BASE_L / 2.0 - 0.030), sy * (BASE_W / 2.0 - 0.024), 0.004),
                )
            )

    for sy in (-1.0, 1.0):
        structure = structure.union(_box((0.018, 0.050, 0.040), (-0.054, sy * 0.044, 0.043)))
        structure = structure.union(_box((0.018, 0.050, 0.040), (0.054, sy * 0.044, 0.043)))

    structure = structure.union(
        _ring_z(
            STATOR_INNER_R - 0.006,
            STATOR_OUTER_R + 0.010,
            0.006,
            (0.0, 0.0, BASE_T + PEDESTAL_H + 0.003),
        )
    )

    for sy in (-1.0, 1.0):
        structure = structure.union(_box((0.030, 0.016, 0.040), (0.0, sy * 0.086, 0.083)))

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            structure = structure.cut(
                _cyl_z(
                    0.006,
                    BASE_T + 0.012,
                    (x_sign * (BASE_L / 2.0 - 0.030), y_sign * (BASE_W / 2.0 - 0.024), BASE_T / 2.0),
                )
            )

    return structure


def _build_side_cover(x_sign: float) -> cq.Workplane:
    cover = _box((0.004, 0.084, 0.052), (x_sign * 0.092, 0.0, 0.041))
    for y in (-0.028, 0.028):
        for z in (0.028, 0.054):
            cover = cover.union(_cyl_x(0.0045, 0.004, (x_sign * 0.096, y, z)))
    return cover


def _build_yaw_pointer() -> cq.Workplane:
    pointer = _box((0.016, 0.010, 0.018), (0.0, 0.079, 0.033))
    pointer = pointer.union(_box((0.004, 0.022, 0.006), (0.0, 0.070, 0.043)))
    pointer = pointer.union(_cyl_z(0.0035, 0.006, (0.0, 0.084, 0.029)))
    return pointer


def _build_stator_ring() -> cq.Workplane:
    stator = _ring_z(STATOR_INNER_R, STATOR_OUTER_R, STATOR_H, (0.0, 0.0, YAW_AXIS_Z))
    stator = stator.cut(_box((0.022, 0.090, 0.018), (0.0, 0.0, YAW_AXIS_Z - 0.002)))
    return stator


def _build_yaw_core() -> cq.Workplane:
    core = _ring_z(0.025, 0.058, 0.020, (0.0, 0.0, 0.028))
    core = core.union(_box((0.170, 0.110, 0.014), (0.0, 0.0, 0.031)))
    core = core.union(_box((0.120, 0.022, 0.044), (0.0, -0.044, 0.060)))
    core = core.union(_box((0.104, 0.014, 0.026), (0.0, 0.044, 0.050)))
    core = core.cut(_box((0.074, 0.048, 0.020), (0.0, 0.006, 0.031)))
    core = core.cut(_box((0.040, 0.020, 0.018), (0.0, -0.044, 0.060)))
    return core


def _build_yoke(x_sign: float) -> cq.Workplane:
    yoke = _box((0.010, 0.112, 0.136), (x_sign * 0.088, 0.0, 0.103))
    yoke = yoke.cut(_box((0.016, 0.074, 0.096), (x_sign * 0.088, 0.0, 0.103)))
    yoke = yoke.union(_box((0.034, 0.018, 0.062), (x_sign * 0.066, 0.046, 0.094)))
    yoke = yoke.union(_box((0.034, 0.018, 0.062), (x_sign * 0.066, -0.046, 0.094)))
    yoke = yoke.union(_box((0.026, 0.026, 0.024), (x_sign * 0.060, 0.0, 0.052)))
    yoke = yoke.union(
        _ring_x(
            0.014,
            0.026,
            0.006,
            (x_sign * 0.081, 0.0, PITCH_AXIS_Z),
        )
    )
    yoke = yoke.union(
        _ring_x(
            0.014,
            0.031,
            0.006,
            (x_sign * 0.091, 0.0, PITCH_AXIS_Z),
        )
    )
    yoke = yoke.cut(_cyl_x(0.014, 0.020, (x_sign * 0.086, 0.0, PITCH_AXIS_Z)))

    for y in (-0.030, 0.030):
        yoke = yoke.union(_cyl_x(0.004, 0.004, (x_sign * 0.089, y, PITCH_AXIS_Z - 0.020)))
        yoke = yoke.union(_cyl_x(0.004, 0.004, (x_sign * 0.089, y, PITCH_AXIS_Z + 0.020)))
    return yoke


def _build_index_band() -> cq.Workplane:
    band = _ring_z(0.062, 0.072, 0.003, (0.0, 0.0, 0.0395))
    for i in range(24):
        angle = i * (math.tau / 24.0)
        radius = 0.067
        tick_len = 0.012 if i % 6 == 0 else 0.008
        tick = _box(
            (0.003, tick_len, 0.003),
            (radius * math.cos(angle), radius * math.sin(angle), 0.0425),
        )
        band = band.union(tick)
    return band


def _build_stop_finger() -> cq.Workplane:
    finger = _box((0.012, 0.032, 0.006), (0.0, 0.069, 0.034))
    finger = finger.union(_box((0.018, 0.010, 0.010), (0.0, 0.087, 0.034)))
    return finger


def _build_yaw_bolt_hardware() -> cq.Workplane:
    return _bolt_circle_z(0.052, 0.0036, 0.007, 0.0205, 12, start_angle=math.pi / 12.0)


def _build_cradle_frame() -> cq.Workplane:
    frame = _box((0.060, 0.094, 0.096), (0.0, 0.0, -0.010))
    frame = frame.cut(_box((0.042, 0.060, 0.076), (0.0, 0.0, -0.010)))
    frame = frame.cut(_box((0.078, 0.046, 0.034), (0.0, 0.0, 0.022)))
    frame = frame.union(_box((0.074, 0.028, 0.016), (0.0, 0.0, -0.044)))
    frame = frame.union(_box((0.074, 0.014, 0.018), (0.0, 0.040, -0.018)))
    frame = frame.union(_box((0.074, 0.014, 0.018), (0.0, -0.040, -0.018)))
    frame = frame.union(_box((0.062, 0.016, 0.012), (0.0, 0.040, 0.026)))
    frame = frame.union(_box((0.062, 0.016, 0.012), (0.0, -0.040, -0.034)))
    frame = frame.union(_box((0.028, 0.044, 0.020), (-0.045, 0.0, 0.000)))
    frame = frame.union(_box((0.028, 0.044, 0.020), (0.045, 0.0, 0.000)))
    frame = frame.union(_box((0.018, 0.022, 0.054), (-0.055, 0.0, 0.000)))
    frame = frame.union(_box((0.018, 0.022, 0.054), (0.055, 0.0, 0.000)))
    return frame


def _build_trunnion_body(x_sign: float) -> cq.Workplane:
    trunnion = _box((0.020, 0.034, 0.044), (x_sign * 0.053, 0.0, 0.0))
    trunnion = trunnion.union(_box((0.018, 0.026, 0.052), (x_sign * 0.062, 0.0, 0.0)))
    trunnion = trunnion.union(_box((0.012, 0.040, 0.020), (x_sign * 0.046, 0.0, 0.0)))
    return trunnion


def _build_trunnion_journal(x_sign: float) -> cq.Workplane:
    return _cyl_x(0.010, 0.026, (x_sign * 0.081, 0.0, 0.0))


def _build_trunnion_retainer(x_sign: float) -> cq.Workplane:
    return _cyl_x(0.018, 0.002, (x_sign * 0.095, 0.0, 0.0))


def _build_pitch_stop_block(x_sign: float) -> cq.Workplane:
    stop = _box((0.016, 0.020, 0.018), (x_sign * 0.072, 0.046, PITCH_AXIS_Z + 0.034))
    stop = stop.union(_box((0.016, 0.020, 0.018), (x_sign * 0.072, -0.046, PITCH_AXIS_Z - 0.034)))
    stop = stop.union(_cyl_x(0.0035, 0.004, (x_sign * 0.081, 0.046, PITCH_AXIS_Z + 0.034)))
    stop = stop.union(_cyl_x(0.0035, 0.004, (x_sign * 0.081, -0.046, PITCH_AXIS_Z - 0.034)))
    return stop


def _build_pitch_bearing_cap(x_sign: float) -> cq.Workplane:
    cap = _ring_x(0.0155, 0.029, 0.004, (x_sign * 0.098, 0.0, PITCH_AXIS_Z))
    cap = cap.union(_box((0.004, 0.018, 0.016), (x_sign * 0.095, 0.022, PITCH_AXIS_Z)))
    cap = cap.union(_box((0.004, 0.018, 0.016), (x_sign * 0.095, -0.022, PITCH_AXIS_Z)))
    cap = cap.union(_cyl_x(0.0105, 0.002, (x_sign * 0.095, 0.0, PITCH_AXIS_Z)))
    return cap


def _build_pitch_cap_hardware(x_sign: float) -> cq.Workplane:
    hardware = None
    for y in (-0.022, 0.022):
        for z in (-0.014, 0.014):
            bolt = _cyl_x(0.0032, 0.006, (x_sign * 0.098, y, PITCH_AXIS_Z + z))
            hardware = bolt if hardware is None else hardware.union(bolt)
    assert hardware is not None
    return hardware


def _build_pitch_index_band(x_sign: float) -> cq.Workplane:
    band = _ring_x(0.019, 0.024, 0.003, (x_sign * 0.0895, 0.0, 0.0))
    band = band.union(_box((0.004, 0.006, 0.020), (x_sign * 0.087, 0.0, 0.0)))
    for angle_deg in range(-60, 61, 15):
        angle = math.radians(angle_deg)
        tick_len = 0.008 if angle_deg % 30 == 0 else 0.005
        tick = _box(
            (0.002, 0.003, tick_len),
            (
                x_sign * 0.091,
                0.021 * math.cos(angle),
                0.021 * math.sin(angle),
            ),
        )
        band = band.union(tick)
    return band


def _build_pitch_limit_tabs(x_sign: float) -> cq.Workplane:
    tabs = _box((0.010, 0.016, 0.014), (x_sign * 0.091, 0.046, 0.032))
    tabs = tabs.union(_box((0.010, 0.016, 0.014), (x_sign * 0.091, -0.046, -0.032)))
    tabs = tabs.union(_box((0.030, 0.006, 0.010), (x_sign * 0.076, 0.041, 0.026)))
    tabs = tabs.union(_box((0.030, 0.006, 0.010), (x_sign * 0.076, -0.041, -0.026)))
    return tabs


def _build_rear_access_cover() -> cq.Workplane:
    cover = _box((0.032, 0.004, 0.040), (0.0, -0.049, -0.028))
    for x in (-0.011, 0.011):
        for z in (-0.040, -0.016):
            cover = cover.union(_cyl_y(0.0035, 0.004, (x, -0.051, z)))
    return cover


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_module", assets=ASSETS)

    coated = model.material("coated_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    machined = model.material("machined_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    bronze = model.material("bearing_bronze", rgba=(0.64, 0.49, 0.28, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.12, 0.13, 0.14, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_build_base_structure(), "base_structure.obj", assets=ASSETS),
        material=coated,
        name="base_structure",
    )
    base_frame.visual(
        mesh_from_cadquery(_build_stator_ring(), "base_stator_ring.obj", assets=ASSETS),
        material=bronze,
        name="yaw_stator_ring",
    )
    base_frame.visual(
        mesh_from_cadquery(_build_side_cover(-1.0), "base_left_cover.obj", assets=ASSETS),
        material=machined,
        name="left_access_cover",
    )
    base_frame.visual(
        mesh_from_cadquery(_build_side_cover(1.0), "base_right_cover.obj", assets=ASSETS),
        material=machined,
        name="right_access_cover",
    )
    base_frame.visual(
        mesh_from_cadquery(_build_yaw_pointer(), "yaw_pointer.obj", assets=ASSETS),
        material=dark_hardware,
        name="yaw_pointer",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.090)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    yaw_rotor = model.part("yaw_rotor")
    yaw_rotor.visual(
        mesh_from_cadquery(_build_yaw_core(), "yaw_core.obj", assets=ASSETS),
        material=coated,
        name="yaw_carrier",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_cyl_z(0.040, 0.036, (0.0, 0.0, 0.012)), "yaw_spindle.obj", assets=ASSETS),
        material=machined,
        name="yaw_spindle",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(
            _ring_z(0.022, 0.062, 0.004, (0.0, 0.0, 0.016)),
            "yaw_thrust_flange.obj",
            assets=ASSETS,
        ),
        material=machined,
        name="yaw_thrust_flange",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_yoke(-1.0), "yaw_left_yoke.obj", assets=ASSETS),
        material=machined,
        name="left_yoke",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_yoke(1.0), "yaw_right_yoke.obj", assets=ASSETS),
        material=machined,
        name="right_yoke",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_index_band(), "yaw_index_band.obj", assets=ASSETS),
        material=dark_hardware,
        name="index_band",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_stop_finger(), "yaw_stop_finger.obj", assets=ASSETS),
        material=dark_hardware,
        name="yaw_stop_finger",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_yaw_bolt_hardware(), "yaw_bolt_hardware.obj", assets=ASSETS),
        material=dark_hardware,
        name="yaw_bolt_hardware",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_pitch_bearing_cap(-1.0), "left_pitch_bearing_cap.obj", assets=ASSETS),
        material=machined,
        name="left_pitch_bearing_cap",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_pitch_bearing_cap(1.0), "right_pitch_bearing_cap.obj", assets=ASSETS),
        material=machined,
        name="right_pitch_bearing_cap",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_pitch_cap_hardware(-1.0), "left_pitch_cap_hardware.obj", assets=ASSETS),
        material=dark_hardware,
        name="left_pitch_cap_hardware",
    )
    yaw_rotor.visual(
        mesh_from_cadquery(_build_pitch_cap_hardware(1.0), "right_pitch_cap_hardware.obj", assets=ASSETS),
        material=dark_hardware,
        name="right_pitch_cap_hardware",
    )
    yaw_rotor.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.180)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_build_cradle_frame(), "pitch_frame.obj", assets=ASSETS),
        material=machined,
        name="cradle_frame",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_build_trunnion_body(-1.0), "pitch_left_trunnion_body.obj", assets=ASSETS),
        material=bronze,
        name="left_trunnion_body",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_build_trunnion_body(1.0), "pitch_right_trunnion_body.obj", assets=ASSETS),
        material=bronze,
        name="right_trunnion_body",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_build_trunnion_journal(-1.0), "pitch_left_trunnion_journal.obj", assets=ASSETS),
        material=machined,
        name="left_trunnion_journal",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_build_trunnion_journal(1.0), "pitch_right_trunnion_journal.obj", assets=ASSETS),
        material=machined,
        name="right_trunnion_journal",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_build_rear_access_cover(), "pitch_rear_cover.obj", assets=ASSETS),
        material=coated,
        name="rear_access_cover",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.100, 0.120)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=yaw_rotor,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_rotor,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-1.05, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    yaw_rotor = object_model.get_part("yaw_rotor")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    stator_ring = base_frame.get_visual("yaw_stator_ring")
    left_cover = base_frame.get_visual("left_access_cover")
    right_cover = base_frame.get_visual("right_access_cover")

    yaw_spindle = yaw_rotor.get_visual("yaw_spindle")
    thrust_flange = yaw_rotor.get_visual("yaw_thrust_flange")
    left_yoke = yaw_rotor.get_visual("left_yoke")
    right_yoke = yaw_rotor.get_visual("right_yoke")
    left_pitch_bearing_cap = yaw_rotor.get_visual("left_pitch_bearing_cap")
    right_pitch_bearing_cap = yaw_rotor.get_visual("right_pitch_bearing_cap")
    index_band = yaw_rotor.get_visual("index_band")
    stop_finger = yaw_rotor.get_visual("yaw_stop_finger")

    left_trunnion_body = pitch_cradle.get_visual("left_trunnion_body")
    right_trunnion_body = pitch_cradle.get_visual("right_trunnion_body")
    left_trunnion = pitch_cradle.get_visual("left_trunnion_journal")
    right_trunnion = pitch_cradle.get_visual("right_trunnion_journal")
    rear_cover = pitch_cradle.get_visual("rear_access_cover")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        pitch_cradle,
        yaw_rotor,
        elem_a=left_trunnion,
        elem_b=left_pitch_bearing_cap,
        reason="left pitch journal is intentionally captured by a bearing cap and shoulder sleeve",
    )
    ctx.allow_overlap(
        pitch_cradle,
        yaw_rotor,
        elem_a=right_trunnion,
        elem_b=right_pitch_bearing_cap,
        reason="right pitch journal is intentionally captured by a bearing cap and shoulder sleeve",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("three_major_parts_present", len(object_model.parts) == 3, f"got {len(object_model.parts)} parts")
    ctx.check(
        "two_axis_articulation_tree",
        len(object_model.articulations) == 2,
        f"got {len(object_model.articulations)} articulations",
    )

    ctx.expect_origin_distance(
        yaw_rotor,
        base_frame,
        axes="xy",
        max_dist=0.001,
        name="yaw_axis_is_centered_on_base",
    )
    ctx.expect_contact(
        yaw_rotor,
        base_frame,
        elem_a=thrust_flange,
        elem_b=stator_ring,
        name="yaw_thrust_flange_is_seated_on_stator_ring",
    )
    ctx.expect_gap(
        yaw_rotor,
        base_frame,
        axis="z",
        positive_elem=thrust_flange,
        negative_elem=stator_ring,
        max_gap=0.0005,
        max_penetration=0.0,
        name="yaw_bearing_stack_has_no_rest_gap",
    )
    ctx.expect_origin_distance(
        pitch_cradle,
        yaw_rotor,
        axes="xy",
        max_dist=0.001,
        name="pitch_axis_stays_on_yaw_centerplane",
    )
    ctx.expect_contact(
        pitch_cradle,
        yaw_rotor,
        elem_a=left_trunnion,
        elem_b=left_pitch_bearing_cap,
        name="left_trunnion_journal_is_captured_by_left_bearing_cap",
    )
    ctx.expect_contact(
        pitch_cradle,
        yaw_rotor,
        elem_a=right_trunnion,
        elem_b=right_pitch_bearing_cap,
        name="right_trunnion_journal_is_captured_by_right_bearing_cap",
    )
    ctx.expect_overlap(
        yaw_rotor,
        base_frame,
        axes="xy",
        elem_a=index_band,
        elem_b=stator_ring,
        min_overlap=0.120,
        name="index_band_tracks_over_stator_bearing_circle",
    )
    ctx.expect_gap(
        pitch_cradle,
        yaw_rotor,
        axis="z",
        positive_elem=left_trunnion_body,
        negative_elem=index_band,
        min_gap=0.070,
        name="pitch_trunnions_sit_above_yaw_index_band",
    )
    ctx.expect_overlap(
        pitch_cradle,
        yaw_rotor,
        axes="yz",
        elem_a=left_trunnion_body,
        elem_b=left_yoke,
        min_overlap=0.030,
        name="left_trunnion_body_is_nested_inside_left_yoke_window",
    )
    ctx.expect_overlap(
        pitch_cradle,
        yaw_rotor,
        axes="yz",
        elem_a=right_trunnion_body,
        elem_b=right_yoke,
        min_overlap=0.030,
        name="right_trunnion_body_is_nested_inside_right_yoke_window",
    )
    ctx.expect_within(
        pitch_cradle,
        yaw_rotor,
        axes="yz",
        inner_elem=left_trunnion,
        outer_elem=left_yoke,
        margin=0.004,
        name="left_journal_runs_within_left_yoke_profile",
    )
    ctx.expect_within(
        pitch_cradle,
        yaw_rotor,
        axes="yz",
        inner_elem=right_trunnion,
        outer_elem=right_yoke,
        margin=0.004,
        name="right_journal_runs_within_right_yoke_profile",
    )

    left_cover_aabb = ctx.part_element_world_aabb(base_frame, elem=left_cover)
    right_cover_aabb = ctx.part_element_world_aabb(base_frame, elem=right_cover)
    rear_cover_aabb = ctx.part_element_world_aabb(pitch_cradle, elem=rear_cover)
    left_cover_center = _aabb_center(left_cover_aabb)
    right_cover_center = _aabb_center(right_cover_aabb)
    rear_cover_center = _aabb_center(rear_cover_aabb)
    ctx.check(
        "base_side_covers_are_symmetrically_mounted",
        left_cover_center is not None
        and right_cover_center is not None
        and left_cover_center[0] < -0.085
        and right_cover_center[0] > 0.085
        and abs(left_cover_center[2] - right_cover_center[2]) < 0.001,
        f"left={left_cover_center}, right={right_cover_center}",
    )
    ctx.check(
        "pitch_rear_access_cover_is_rear_facing",
        rear_cover_center is not None and rear_cover_center[1] < -0.047,
        f"rear cover center={rear_cover_center}",
    )

    stop_finger_rest = _aabb_center(ctx.part_element_world_aabb(yaw_rotor, elem=stop_finger))
    with ctx.pose({yaw_joint: 1.40}):
        ctx.expect_contact(
            yaw_rotor,
            base_frame,
            elem_a=thrust_flange,
            elem_b=stator_ring,
            name="yaw_support_persists_at_positive_yaw_pose",
        )
        stop_finger_yawed = _aabb_center(ctx.part_element_world_aabb(yaw_rotor, elem=stop_finger))
    ctx.check(
        "yaw_joint_rotates_an_off_axis_follower",
        stop_finger_rest is not None
        and stop_finger_yawed is not None
        and abs(stop_finger_rest[0] - stop_finger_yawed[0]) > 0.045,
        f"rest={stop_finger_rest}, yawed={stop_finger_yawed}",
    )

    rear_cover_rest = rear_cover_center
    with ctx.pose({pitch_joint: 0.82}):
        ctx.expect_contact(
            pitch_cradle,
            yaw_rotor,
            elem_a=left_trunnion,
            elem_b=left_pitch_bearing_cap,
            name="left_bearing_capture_persists_at_positive_pitch",
        )
        ctx.expect_contact(
            pitch_cradle,
            yaw_rotor,
            elem_a=right_trunnion,
            elem_b=right_pitch_bearing_cap,
            name="right_bearing_capture_persists_at_positive_pitch",
        )
        rear_cover_pitched = _aabb_center(ctx.part_element_world_aabb(pitch_cradle, elem=rear_cover))
    ctx.check(
        "pitch_joint_swings_the_cradle_about_the_x_axis",
        rear_cover_rest is not None
        and rear_cover_pitched is not None
        and abs(rear_cover_rest[1] - rear_cover_pitched[1]) > 0.020
        and abs(rear_cover_rest[2] - rear_cover_pitched[2]) > 0.020,
        f"rest={rear_cover_rest}, pitched={rear_cover_pitched}",
    )

    with ctx.pose({yaw_joint: -1.25, pitch_joint: -0.78}):
        ctx.expect_contact(
            pitch_cradle,
            yaw_rotor,
            elem_a=left_trunnion,
            elem_b=left_pitch_bearing_cap,
            name="left_bearing_capture_persists_in_combined_pose",
        )
        ctx.expect_contact(
            pitch_cradle,
            yaw_rotor,
            elem_a=right_trunnion,
            elem_b=right_pitch_bearing_cap,
            name="right_bearing_capture_persists_in_combined_pose",
        )
        ctx.expect_contact(
            yaw_rotor,
            base_frame,
            elem_a=thrust_flange,
            elem_b=stator_ring,
            name="yaw_support_persists_in_combined_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
