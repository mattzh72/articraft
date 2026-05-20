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


SHOULDER_Z = 0.245
UPPER_LENGTH = 0.280
FOREARM_LENGTH = 0.230
WRIST_LENGTH = 0.120


def _mesh(shape: cq.Workplane | cq.Shape, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS, tolerance=0.0008, angular_tolerance=0.08)


def _bolt_ring_y(
    *,
    center_x: float,
    center_z: float,
    y_center: float,
    pitch_radius: float,
    bolt_radius: float,
    bolt_length: float,
    count: int = 6,
    angle_offset: float = 0.0,
) -> cq.Workplane:
    pts = [
        (
            center_x + pitch_radius * math.cos(angle_offset + (2.0 * math.pi * i / count)),
            center_z + pitch_radius * math.sin(angle_offset + (2.0 * math.pi * i / count)),
        )
        for i in range(count)
    ]
    return cq.Workplane("XZ").pushPoints(pts).circle(bolt_radius).extrude(bolt_length / 2.0, both=True).translate((0.0, y_center, 0.0))


def _corner_bolts_y(
    *,
    center_x: float,
    center_z: float,
    y_center: float,
    span_x: float,
    span_z: float,
    bolt_radius: float,
    bolt_length: float,
) -> cq.Workplane:
    pts = [
        (center_x - span_x / 2.0, center_z - span_z / 2.0),
        (center_x - span_x / 2.0, center_z + span_z / 2.0),
        (center_x + span_x / 2.0, center_z - span_z / 2.0),
        (center_x + span_x / 2.0, center_z + span_z / 2.0),
    ]
    return cq.Workplane("XZ").pushPoints(pts).circle(bolt_radius).extrude(bolt_length / 2.0, both=True).translate((0.0, y_center, 0.0))


def _plate_xz(points: list[tuple[float, float]], thickness: float, y_center: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness / 2.0, both=True).translate((0.0, y_center, 0.0))


def _joint_cover_stack(
    *,
    center_x: float,
    center_z: float,
    y_sign: float,
    outer_face_y: float,
    boss_radius: float,
    boss_length: float,
    cover_radius: float,
    cover_thickness: float,
    bolt_pitch: float,
    bolt_radius: float,
    bolt_length: float,
    count: int = 6,
) -> cq.Workplane:
    boss_center_y = y_sign * (abs(outer_face_y) + boss_length / 2.0)
    cover_center_y = y_sign * (abs(outer_face_y) + boss_length + cover_thickness / 2.0)
    boss = (
        cq.Workplane("XZ")
        .circle(boss_radius)
        .extrude(boss_length / 2.0, both=True)
        .translate((center_x, boss_center_y, center_z))
    )
    cover = (
        cq.Workplane("XZ")
        .circle(cover_radius)
        .extrude(cover_thickness / 2.0, both=True)
        .translate((center_x, cover_center_y, center_z))
    )
    bolts = _bolt_ring_y(
        center_x=center_x,
        center_z=center_z,
        y_center=cover_center_y,
        pitch_radius=bolt_pitch,
        bolt_radius=bolt_radius,
        bolt_length=bolt_length,
        count=count,
        angle_offset=math.pi / count,
    )
    return boss.union(cover).union(bolts)


def _access_cover_y(
    *,
    center_x: float,
    center_z: float,
    y_face: float,
    size_x: float,
    size_z: float,
    thickness: float,
    bolt_radius: float,
    bolt_length: float,
) -> cq.Workplane:
    cover_center_y = y_face + math.copysign(thickness / 2.0, y_face)
    cover = (
        cq.Workplane("XZ")
        .rect(size_x, size_z)
        .extrude(thickness / 2.0, both=True)
        .translate((center_x, cover_center_y, center_z))
    )
    bolts = _corner_bolts_y(
        center_x=center_x,
        center_z=center_z,
        y_center=cover_center_y,
        span_x=size_x * 0.72,
        span_z=size_z * 0.72,
        bolt_radius=bolt_radius,
        bolt_length=bolt_length,
    )
    return cover.union(bolts)


def _box_beam(
    *,
    start_x: float,
    length: float,
    width: float,
    height: float,
    wall: float,
    window_centers: list[float],
    window_length: float,
    window_height: float,
) -> cq.Workplane:
    beam = cq.Workplane("XY").box(length, width, height).translate((start_x + length / 2.0, 0.0, 0.0))
    inner = cq.Workplane("XY").box(length - 0.030, width - 2.0 * wall, height - 2.0 * wall).translate(
        (start_x + length / 2.0 + 0.004, 0.0, 0.0)
    )
    beam = beam.cut(inner)
    for x_center in window_centers:
        cutter = cq.Workplane("XY").box(window_length, width * 1.5, window_height).translate((x_center, 0.0, 0.0))
        beam = beam.cut(cutter)
    return beam


def _make_base_frame() -> tuple[cq.Workplane, cq.Workplane]:
    base = cq.Workplane("XY").box(0.260, 0.180, 0.024).translate((0.0, 0.0, 0.012))
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints([(-0.085, -0.055), (-0.085, 0.055), (0.085, -0.055), (0.085, 0.055)])
        .slot2D(0.028, 0.012, angle=0.0)
        .cutThruAll()
    )
    pedestal = cq.Workplane("XY").box(0.108, 0.088, 0.128).translate((-0.018, 0.0, 0.088))
    shoulder_bridge = cq.Workplane("XY").box(0.064, 0.060, 0.030).translate((-0.030, 0.0, 0.165))
    front_gusset = _plate_xz([(-0.078, 0.024), (-0.030, 0.024), (0.006, 0.112), (-0.078, 0.112)], 0.018, 0.026)
    rear_gusset = _plate_xz([(-0.065, 0.024), (0.012, 0.024), (0.012, 0.118), (-0.035, 0.118)], 0.018, -0.026)
    spine = cq.Workplane("XY").box(0.040, 0.042, 0.076).translate((-0.048, 0.0, 0.166))
    frame = base.union(pedestal).union(shoulder_bridge).union(front_gusset).union(rear_gusset).union(spine)
    access_left = _access_cover_y(
        center_x=-0.020,
        center_z=0.090,
        y_face=0.044,
        size_x=0.060,
        size_z=0.072,
        thickness=0.004,
        bolt_radius=0.0016,
        bolt_length=0.003,
    )
    access_right = _access_cover_y(
        center_x=-0.020,
        center_z=0.090,
        y_face=-0.044,
        size_x=0.060,
        size_z=0.072,
        thickness=0.004,
        bolt_radius=0.0016,
        bolt_length=0.003,
    )
    frame = frame.union(access_left).union(access_right)

    plate_points = [
        (-0.055, 0.120),
        (0.016, 0.120),
        (0.056, 0.168),
        (0.056, 0.292),
        (0.022, 0.312),
        (-0.034, 0.312),
        (-0.055, 0.274),
    ]
    left_plate = _plate_xz(plate_points, 0.012, 0.039)
    right_plate = _plate_xz(plate_points, 0.012, -0.039)
    hole_left = cq.Workplane("XZ").circle(0.0185).extrude(0.010, both=True).translate((0.0, 0.039, SHOULDER_Z))
    hole_right = cq.Workplane("XZ").circle(0.0185).extrude(0.010, both=True).translate((0.0, -0.039, SHOULDER_Z))
    left_plate = left_plate.cut(hole_left)
    right_plate = right_plate.cut(hole_right)
    yoke = left_plate.union(right_plate)
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=0.0,
            center_z=SHOULDER_Z,
            y_sign=1.0,
            outer_face_y=0.045,
            boss_radius=0.028,
            boss_length=0.010,
            cover_radius=0.034,
            cover_thickness=0.004,
            bolt_pitch=0.026,
            bolt_radius=0.0016,
            bolt_length=0.003,
        )
    )
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=0.0,
            center_z=SHOULDER_Z,
            y_sign=-1.0,
            outer_face_y=-0.045,
            boss_radius=0.028,
            boss_length=0.010,
            cover_radius=0.034,
            cover_thickness=0.004,
            bolt_pitch=0.026,
            bolt_radius=0.0016,
            bolt_length=0.003,
        )
    )
    return frame, yoke


def _make_upper_link() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    shoulder_hub = cq.Workplane("XZ").circle(0.023).extrude(0.027, both=True)
    shoulder_hub = shoulder_hub.union(cq.Workplane("XZ").circle(0.0245).extrude(0.003, both=True).translate((0.0, 0.030, 0.0)))
    shoulder_hub = shoulder_hub.union(cq.Workplane("XZ").circle(0.0245).extrude(0.003, both=True).translate((0.0, -0.030, 0.0)))
    shoulder_hub = shoulder_hub.union(cq.Workplane("XZ").circle(0.011).extrude(0.002, both=True).translate((0.0, 0.035, 0.0)))
    shoulder_hub = shoulder_hub.union(cq.Workplane("XZ").circle(0.011).extrude(0.002, both=True).translate((0.0, -0.035, 0.0)))
    root_block = cq.Workplane("XY").box(0.046, 0.050, 0.052).translate((0.022, 0.0, 0.0))
    upper_gusset = _plate_xz([(-0.004, 0.018), (0.040, 0.018), (0.058, 0.038), (0.012, 0.038)], 0.036, 0.0)
    lower_gusset = _plate_xz([(-0.004, -0.018), (0.040, -0.018), (0.058, -0.038), (0.012, -0.038)], 0.036, 0.0)
    shoulder_hub = shoulder_hub.union(root_block).union(upper_gusset).union(lower_gusset)

    beam = _box_beam(
        start_x=0.030,
        length=0.200,
        width=0.054,
        height=0.060,
        wall=0.006,
        window_centers=[0.112, 0.170],
        window_length=0.032,
        window_height=0.026,
    )
    lower_stiffener = cq.Workplane("XY").box(0.182, 0.022, 0.010).translate((0.131, 0.0, -0.035))
    beam = beam.union(lower_stiffener)
    beam = beam.union(
        _access_cover_y(
            center_x=0.130,
            center_z=0.0,
            y_face=0.027,
            size_x=0.070,
            size_z=0.040,
            thickness=0.0035,
            bolt_radius=0.0014,
            bolt_length=0.0025,
        )
    )

    plate_points = [
        (0.206, -0.050),
        (0.270, -0.050),
        (0.300, -0.024),
        (0.300, 0.044),
        (0.272, 0.056),
        (0.220, 0.056),
        (0.198, 0.014),
    ]
    left_plate = _plate_xz(plate_points, 0.012, 0.035)
    right_plate = _plate_xz(plate_points, 0.012, -0.035)
    hole_left = cq.Workplane("XZ").circle(0.0175).extrude(0.010, both=True).translate((UPPER_LENGTH, 0.035, 0.0))
    hole_right = cq.Workplane("XZ").circle(0.0175).extrude(0.010, both=True).translate((UPPER_LENGTH, -0.035, 0.0))
    left_plate = left_plate.cut(hole_left)
    right_plate = right_plate.cut(hole_right)
    yoke = left_plate.union(right_plate)
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=UPPER_LENGTH,
            center_z=0.0,
            y_sign=1.0,
            outer_face_y=0.041,
            boss_radius=0.024,
            boss_length=0.009,
            cover_radius=0.029,
            cover_thickness=0.0035,
            bolt_pitch=0.021,
            bolt_radius=0.0014,
            bolt_length=0.0025,
        )
    )
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=UPPER_LENGTH,
            center_z=0.0,
            y_sign=-1.0,
            outer_face_y=-0.041,
            boss_radius=0.024,
            boss_length=0.009,
            cover_radius=0.029,
            cover_thickness=0.0035,
            bolt_pitch=0.021,
            bolt_radius=0.0014,
            bolt_length=0.0025,
        )
    )
    return shoulder_hub, beam, yoke


def _make_forearm_link() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    elbow_hub = cq.Workplane("XZ").circle(0.021).extrude(0.023, both=True)
    elbow_hub = elbow_hub.union(cq.Workplane("XZ").circle(0.0225).extrude(0.003, both=True).translate((0.0, 0.026, 0.0)))
    elbow_hub = elbow_hub.union(cq.Workplane("XZ").circle(0.0225).extrude(0.003, both=True).translate((0.0, -0.026, 0.0)))
    elbow_hub = elbow_hub.union(cq.Workplane("XZ").circle(0.010).extrude(0.002, both=True).translate((0.0, 0.031, 0.0)))
    elbow_hub = elbow_hub.union(cq.Workplane("XZ").circle(0.010).extrude(0.002, both=True).translate((0.0, -0.031, 0.0)))
    root_block = cq.Workplane("XY").box(0.040, 0.044, 0.046).translate((0.020, 0.0, 0.0))
    upper_gusset = _plate_xz([(-0.004, 0.016), (0.036, 0.016), (0.054, 0.034), (0.012, 0.034)], 0.032, 0.0)
    lower_gusset = _plate_xz([(-0.004, -0.016), (0.036, -0.016), (0.054, -0.034), (0.012, -0.034)], 0.032, 0.0)
    elbow_hub = elbow_hub.union(root_block).union(upper_gusset).union(lower_gusset)

    beam = _box_beam(
        start_x=0.028,
        length=0.162,
        width=0.046,
        height=0.052,
        wall=0.005,
        window_centers=[0.092, 0.142],
        window_length=0.026,
        window_height=0.022,
    )
    dorsal_rib = cq.Workplane("XY").box(0.150, 0.018, 0.008).translate((0.103, 0.0, 0.030))
    beam = beam.union(dorsal_rib)
    beam = beam.union(
        _access_cover_y(
            center_x=0.112,
            center_z=0.0,
            y_face=-0.023,
            size_x=0.060,
            size_z=0.034,
            thickness=0.003,
            bolt_radius=0.0013,
            bolt_length=0.0024,
        )
    )

    plate_points = [
        (0.170, -0.044),
        (0.220, -0.044),
        (0.248, -0.020),
        (0.248, 0.040),
        (0.222, 0.050),
        (0.182, 0.050),
        (0.160, 0.012),
    ]
    left_plate = _plate_xz(plate_points, 0.012, 0.031)
    right_plate = _plate_xz(plate_points, 0.012, -0.031)
    hole_left = cq.Workplane("XZ").circle(0.0155).extrude(0.010, both=True).translate((FOREARM_LENGTH, 0.031, 0.0))
    hole_right = cq.Workplane("XZ").circle(0.0155).extrude(0.010, both=True).translate((FOREARM_LENGTH, -0.031, 0.0))
    left_plate = left_plate.cut(hole_left)
    right_plate = right_plate.cut(hole_right)
    yoke = left_plate.union(right_plate)
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=FOREARM_LENGTH,
            center_z=0.0,
            y_sign=1.0,
            outer_face_y=0.037,
            boss_radius=0.021,
            boss_length=0.008,
            cover_radius=0.026,
            cover_thickness=0.003,
            bolt_pitch=0.018,
            bolt_radius=0.0013,
            bolt_length=0.0022,
        )
    )
    yoke = yoke.union(
        _joint_cover_stack(
            center_x=FOREARM_LENGTH,
            center_z=0.0,
            y_sign=-1.0,
            outer_face_y=-0.037,
            boss_radius=0.021,
            boss_length=0.008,
            cover_radius=0.026,
            cover_thickness=0.003,
            bolt_pitch=0.018,
            bolt_radius=0.0013,
            bolt_length=0.0022,
        )
    )
    return elbow_hub, beam, yoke


def _make_wrist_link() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    wrist_hub = cq.Workplane("XZ").circle(0.018).extrude(0.019, both=True)
    wrist_hub = wrist_hub.union(cq.Workplane("XZ").circle(0.0195).extrude(0.003, both=True).translate((0.0, 0.022, 0.0)))
    wrist_hub = wrist_hub.union(cq.Workplane("XZ").circle(0.0195).extrude(0.003, both=True).translate((0.0, -0.022, 0.0)))
    wrist_hub = wrist_hub.union(cq.Workplane("XZ").circle(0.0085).extrude(0.0015, both=True).translate((0.0, 0.0265, 0.0)))
    wrist_hub = wrist_hub.union(cq.Workplane("XZ").circle(0.0085).extrude(0.0015, both=True).translate((0.0, -0.0265, 0.0)))
    root_block = cq.Workplane("XY").box(0.034, 0.038, 0.040).translate((0.017, 0.0, 0.0))
    upper_gusset = _plate_xz([(-0.004, 0.014), (0.028, 0.014), (0.044, 0.028), (0.010, 0.028)], 0.026, 0.0)
    lower_gusset = _plate_xz([(-0.004, -0.014), (0.028, -0.014), (0.044, -0.028), (0.010, -0.028)], 0.026, 0.0)
    wrist_hub = wrist_hub.union(root_block).union(upper_gusset).union(lower_gusset)

    carrier = _box_beam(
        start_x=0.024,
        length=0.084,
        width=0.040,
        height=0.046,
        wall=0.005,
        window_centers=[0.064],
        window_length=0.020,
        window_height=0.018,
    )
    guide_fin = _plate_xz([(0.030, -0.012), (0.094, -0.012), (0.112, -0.028), (0.030, -0.028)], 0.014, 0.0)
    carrier = carrier.union(guide_fin)
    carrier = carrier.union(
        _access_cover_y(
            center_x=0.070,
            center_z=0.0,
            y_face=0.020,
            size_x=0.038,
            size_z=0.028,
            thickness=0.0028,
            bolt_radius=0.0012,
            bolt_length=0.002,
        )
    )

    flange_profile = (
        cq.Workplane("YZ")
        .circle(0.036)
        .extrude(0.005, both=True)
        .translate((0.116, 0.0, 0.0))
    )
    flange_cross = cq.Workplane("YZ").rect(0.060, 0.048).extrude(0.003, both=True).translate((0.108, 0.0, 0.0))
    flange = flange_profile.union(flange_cross)
    pilot = cq.Workplane("YZ").circle(0.012).extrude(0.010, both=True).translate((0.116, 0.0, 0.0))
    flange = flange.cut(pilot)
    slot_cutter = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, 0.020), (0.0, -0.020), (0.020, 0.0), (-0.020, 0.0)])
        .slot2D(0.012, 0.006, angle=90.0)
        .extrude(0.010, both=True)
        .translate((0.116, 0.0, 0.0))
    )
    flange = flange.cut(slot_cutter)
    flange_bolts = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, 0.026), (0.0, -0.026), (0.026, 0.0), (-0.026, 0.0)])
        .circle(0.0016)
        .extrude(0.0015, both=True)
        .translate((0.121, 0.0, 0.0))
    )
    flange = flange.union(flange_bolts)
    return wrist_hub, carrier, flange


def _fk_positions(shoulder: float, elbow: float, wrist: float) -> dict[str, tuple[float, float, float]]:
    elbow_pos = (
        UPPER_LENGTH * math.cos(shoulder),
        0.0,
        SHOULDER_Z + UPPER_LENGTH * math.sin(shoulder),
    )
    wrist_pos = (
        elbow_pos[0] + FOREARM_LENGTH * math.cos(shoulder + elbow),
        0.0,
        elbow_pos[2] + FOREARM_LENGTH * math.sin(shoulder + elbow),
    )
    flange_pos = (
        wrist_pos[0] + WRIST_LENGTH * math.cos(shoulder + elbow + wrist),
        0.0,
        wrist_pos[2] + WRIST_LENGTH * math.sin(shoulder + elbow + wrist),
    )
    return {
        "upper": (0.0, 0.0, SHOULDER_Z),
        "forearm": elbow_pos,
        "wrist": wrist_pos,
        "flange_tip": flange_pos,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_chain_study", assets=ASSETS)

    steel = model.material("steel", rgba=(0.34, 0.36, 0.39, 1.0))
    machined = model.material("machined", rgba=(0.62, 0.66, 0.70, 1.0))
    dark_cap = model.material("dark_cap", rgba=(0.16, 0.17, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.28, 0.30, 1.0))

    base = model.part("base_support")
    base_frame, base_yoke = _make_base_frame()
    base.visual(_mesh(base_frame, "base_frame.obj"), material=steel, name="base_frame")
    base.visual(_mesh(base_yoke, "base_yoke.obj"), material=dark_cap, name="shoulder_yoke")
    base.inertial = Inertial.from_geometry(Box((0.260, 0.180, 0.314)), mass=8.0, origin=Origin(xyz=(0.0, 0.0, 0.157)))

    upper = model.part("upper_link")
    shoulder_hub, upper_beam, upper_yoke = _make_upper_link()
    upper.visual(_mesh(shoulder_hub, "upper_shoulder_hub.obj"), material=machined, name="shoulder_hub")
    upper.visual(_mesh(upper_beam, "upper_beam.obj"), material=graphite, name="upper_beam")
    upper.visual(_mesh(upper_yoke, "upper_elbow_yoke.obj"), material=dark_cap, name="elbow_yoke")
    upper.inertial = Inertial.from_geometry(Box((0.304, 0.082, 0.112)), mass=3.2, origin=Origin(xyz=(0.152, 0.0, 0.0)))

    forearm = model.part("forearm_link")
    elbow_hub, forearm_beam, forearm_yoke = _make_forearm_link()
    forearm.visual(_mesh(elbow_hub, "forearm_elbow_hub.obj"), material=machined, name="elbow_hub")
    forearm.visual(_mesh(forearm_beam, "forearm_beam.obj"), material=graphite, name="forearm_beam")
    forearm.visual(_mesh(forearm_yoke, "forearm_wrist_yoke.obj"), material=dark_cap, name="wrist_yoke")
    forearm.inertial = Inertial.from_geometry(Box((0.248, 0.074, 0.100)), mass=2.2, origin=Origin(xyz=(0.124, 0.0, 0.0)))

    wrist = model.part("wrist_link")
    wrist_hub, wrist_carrier, wrist_flange = _make_wrist_link()
    wrist.visual(_mesh(wrist_hub, "wrist_hub.obj"), material=machined, name="wrist_hub")
    wrist.visual(_mesh(wrist_carrier, "wrist_carrier.obj"), material=graphite, name="wrist_carrier")
    wrist.visual(_mesh(wrist_flange, "wrist_flange.obj"), material=dark_cap, name="tool_flange")
    wrist.inertial = Inertial.from_geometry(Box((0.132, 0.060, 0.080)), mass=1.1, origin=Origin(xyz=(0.066, 0.0, 0.0)))

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-0.65, upper=1.20),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.2, lower=0.00, upper=1.70),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.8, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_support")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm_link")
    wrist = object_model.get_part("wrist_link")

    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    base_yoke = base.get_visual("shoulder_yoke")
    upper_hub = upper.get_visual("shoulder_hub")
    upper_elbow_yoke = upper.get_visual("elbow_yoke")
    forearm_hub = forearm.get_visual("elbow_hub")
    forearm_wrist_yoke = forearm.get_visual("wrist_yoke")
    wrist_hub = wrist.get_visual("wrist_hub")
    wrist_flange = wrist.get_visual("tool_flange")

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
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.check("base part present", base is not None, "missing base_support")
    ctx.check("upper part present", upper is not None, "missing upper_link")
    ctx.check("forearm part present", forearm is not None, "missing forearm_link")
    ctx.check("wrist part present", wrist is not None, "missing wrist_link")

    ctx.expect_contact(upper, base, elem_a=upper_hub, elem_b=base_yoke, contact_tol=0.0006, name="shoulder hub seats in base yoke")
    ctx.expect_overlap(upper, base, elem_a=upper_hub, elem_b=base_yoke, axes="yz", min_overlap=0.040, name="shoulder hub spans yoke")
    ctx.expect_contact(forearm, upper, elem_a=forearm_hub, elem_b=upper_elbow_yoke, contact_tol=0.0006, name="elbow hub seats in upper yoke")
    ctx.expect_overlap(forearm, upper, elem_a=forearm_hub, elem_b=upper_elbow_yoke, axes="yz", min_overlap=0.034, name="elbow hub spans yoke")
    ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=forearm_wrist_yoke, contact_tol=0.0006, name="wrist hub seats in forearm yoke")
    ctx.expect_overlap(wrist, forearm, elem_a=wrist_hub, elem_b=forearm_wrist_yoke, axes="yz", min_overlap=0.028, name="wrist hub spans yoke")

    rest = _fk_positions(0.0, 0.0, 0.0)
    forearm_rest = ctx.part_world_position(forearm)
    wrist_rest = ctx.part_world_position(wrist)
    ctx.check(
        "elbow pivot at upper link tip",
        forearm_rest is not None
        and abs(forearm_rest[0] - rest["forearm"][0]) < 1e-4
        and abs(forearm_rest[2] - rest["forearm"][2]) < 1e-4,
        f"forearm origin {forearm_rest} != expected {rest['forearm']}",
    )
    ctx.check(
        "wrist pivot at forearm tip",
        wrist_rest is not None
        and abs(wrist_rest[0] - rest["wrist"][0]) < 1e-4
        and abs(wrist_rest[2] - rest["wrist"][2]) < 1e-4,
        f"wrist origin {wrist_rest} != expected {rest['wrist']}",
    )

    with ctx.pose({shoulder: 0.85, elbow: 0.65, wrist_joint: -0.45}):
        target = _fk_positions(0.85, 0.65, -0.45)
        forearm_pos = ctx.part_world_position(forearm)
        wrist_pos = ctx.part_world_position(wrist)
        ctx.check(
            "pose kinematics move elbow upward",
            forearm_pos is not None
            and abs(forearm_pos[0] - target["forearm"][0]) < 2e-3
            and abs(forearm_pos[2] - target["forearm"][2]) < 2e-3
            and forearm_pos[2] > SHOULDER_Z + 0.12,
            f"forearm posed at {forearm_pos}, expected near {target['forearm']}",
        )
        ctx.check(
            "pose kinematics move wrist to chained target",
            wrist_pos is not None
            and abs(wrist_pos[0] - target["wrist"][0]) < 2e-3
            and abs(wrist_pos[2] - target["wrist"][2]) < 2e-3,
            f"wrist posed at {wrist_pos}, expected near {target['wrist']}",
        )
        ctx.expect_contact(upper, base, elem_a=upper_hub, elem_b=base_yoke, contact_tol=0.0008, name="shoulder contact in raised pose")
        ctx.expect_contact(forearm, upper, elem_a=forearm_hub, elem_b=upper_elbow_yoke, contact_tol=0.0008, name="elbow contact in raised pose")
        ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=forearm_wrist_yoke, contact_tol=0.0008, name="wrist contact in raised pose")
        ctx.expect_origin_gap(wrist, base, axis="z", min_gap=0.020, name="wrist stays above base in raised pose")

    with ctx.pose({shoulder: 0.30, elbow: 1.35, wrist_joint: 0.80}):
        target = _fk_positions(0.30, 1.35, 0.80)
        wrist_pos = ctx.part_world_position(wrist)
        flange_pos = ctx.part_element_world_aabb(wrist, elem=wrist_flange)
        ctx.check(
            "folded pose keeps wrist pivot on expected arc",
            wrist_pos is not None
            and abs(wrist_pos[0] - target["wrist"][0]) < 2e-3
            and abs(wrist_pos[2] - target["wrist"][2]) < 2e-3,
            f"folded wrist position {wrist_pos}, expected near {target['wrist']}",
        )
        ctx.check(
            "tool flange remains clear of ground plane",
            flange_pos is not None and flange_pos[0][2] > 0.030,
            f"tool flange aabb {flange_pos} dips too low",
        )
        ctx.expect_contact(upper, base, elem_a=upper_hub, elem_b=base_yoke, contact_tol=0.0008, name="shoulder contact in folded pose")
        ctx.expect_contact(forearm, upper, elem_a=forearm_hub, elem_b=upper_elbow_yoke, contact_tol=0.0008, name="elbow contact in folded pose")
        ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=forearm_wrist_yoke, contact_tol=0.0008, name="wrist contact in folded pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
