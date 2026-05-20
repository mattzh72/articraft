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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PLATE_CENTER_X = 0.155
PLATE_THICKNESS = 0.018
CAP_THICKNESS = 0.024
SHAFT_Z = 0.145
INPUT_Y = -0.090
INTERMEDIATE_Y = 0.005
LAYSHAFT_Y = 0.105
VERTICAL_X = 0.110
VERTICAL_Y = LAYSHAFT_Y
VERTICAL_ORIGIN_Z = 0.038
TOP_COVER_SPLIT_Z = 0.223
BEVEL_COVER_SPLIT_Z = 0.082

STAGE1_PINION_TEETH = 16
STAGE1_GEAR_TEETH = 30
STAGE2_PINION_TEETH = 14
STAGE2_GEAR_TEETH = 32
BEVEL_PINION_TEETH = 12
BEVEL_GEAR_TEETH = 20


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cyl_x(length: float, radius: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def _cyl_z(length: float, radius: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _gear_profile_points(
    root_radius: float,
    outer_radius: float,
    teeth: int,
    *,
    phase: float = 0.0,
    tooth_half_fraction: float = 0.24,
    gap_half_fraction: float = 0.46,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    tooth_pitch = 2.0 * math.pi / teeth
    for index in range(teeth):
        center_angle = phase + index * tooth_pitch
        gap_half = tooth_pitch * gap_half_fraction / 2.0
        tooth_half = tooth_pitch * tooth_half_fraction / 2.0
        for radius, angle in (
            (root_radius, center_angle - gap_half),
            (outer_radius, center_angle - tooth_half),
            (outer_radius, center_angle + tooth_half),
            (root_radius, center_angle + gap_half),
        ):
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _spur_gear_x(
    *,
    teeth: int,
    root_radius: float,
    outer_radius: float,
    width: float,
    hub_radius: float,
    hub_length: float,
    bore_radius: float,
    phase: float = 0.0,
    web_scale: float = 0.75,
    hole_count: int = 6,
) -> cq.Workplane:
    profile_points = _gear_profile_points(root_radius, outer_radius, teeth, phase=phase)
    gear = cq.Workplane("YZ").polyline(profile_points).close().extrude(width).translate((-width / 2.0, 0.0, 0.0))
    gear = gear.union(_cyl_x(hub_length, hub_radius))
    gear = gear.union(_cyl_x(width * 0.70, root_radius * web_scale))
    gear = gear.cut(_cyl_x(hub_length + 0.030, bore_radius))
    key_depth = min(0.0030, hub_radius - bore_radius - 0.0010)
    if key_depth > 0.0005:
        gear = gear.cut(_box((hub_length + 0.020, bore_radius * 0.95, key_depth), (0.0, 0.0, bore_radius + key_depth / 2.0)))
    if hole_count > 0 and root_radius > 0.030:
        hole_radius = max(0.0035, root_radius * 0.12)
        hole_ring = root_radius * 0.44
        for index in range(hole_count):
            angle = phase + 2.0 * math.pi * index / hole_count
            gear = gear.cut(_cyl_x(width * 0.78, hole_radius, (0.0, hole_ring * math.cos(angle), hole_ring * math.sin(angle))))
    return gear


def _bevel_gear_x(
    *,
    face_width: float,
    large_radius: float,
    small_radius: float,
    bore_radius: float,
    hub_radius: float,
    hub_length: float,
    teeth: int,
    phase: float = 0.0,
) -> cq.Workplane:
    body = cq.Workplane("YZ").circle(large_radius).workplane(offset=face_width).circle(small_radius).loft(combine=True)
    body = body.translate((-face_width / 2.0, 0.0, 0.0))
    body = body.union(_cyl_x(hub_length, hub_radius, (-0.006, 0.0, 0.0)))
    body = body.cut(_cyl_x(hub_length + 0.030, bore_radius))
    tooth_length = face_width * 0.62
    tooth_depth = max(0.0045, (large_radius - small_radius) * 0.55)
    tooth_width = max(0.0060, large_radius * 0.13)
    tooth_center_x = -face_width * 0.18
    for index in range(teeth):
        angle = phase + 2.0 * math.pi * index / teeth
        tooth = _box(
            (tooth_length, tooth_width, tooth_depth),
            (tooth_center_x, 0.0, large_radius + tooth_depth / 2.0),
        ).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(angle))
        body = body.union(tooth)
    return body


def _bevel_gear_z(
    *,
    face_width: float,
    large_radius: float,
    small_radius: float,
    bore_radius: float,
    hub_radius: float,
    hub_length: float,
    teeth: int,
    phase: float = 0.0,
) -> cq.Workplane:
    body = cq.Workplane("XY").circle(large_radius).workplane(offset=face_width).circle(small_radius).loft(combine=True)
    body = body.translate((0.0, 0.0, -face_width / 2.0))
    body = body.union(_cyl_z(hub_length, hub_radius, (0.0, 0.0, 0.010)))
    body = body.cut(_cyl_z(hub_length + face_width + 0.030, bore_radius))
    tooth_length = face_width * 0.62
    tooth_depth = max(0.0045, (large_radius - small_radius) * 0.55)
    tooth_width = max(0.0060, large_radius * 0.13)
    tooth_center_z = -face_width * 0.18
    for index in range(teeth):
        angle = phase + 2.0 * math.pi * index / teeth
        tooth = _box(
            (tooth_width, tooth_depth, tooth_length),
            (0.0, large_radius + tooth_depth / 2.0, tooth_center_z),
        ).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(angle))
        body = body.union(tooth)
    return body


def _shaft_x(
    *,
    total_length: float,
    core_radius: float,
    journal_radius: float,
    journal_length: float,
    collar_radius: float,
    left_collar_center: float,
    right_collar_center: float,
    left_coupling: bool = False,
    right_coupling: bool = False,
) -> cq.Workplane:
    shaft = _cyl_x(total_length, core_radius)
    shaft = shaft.union(_cyl_x(journal_length, journal_radius, (-PLATE_CENTER_X, 0.0, 0.0)))
    shaft = shaft.union(_cyl_x(journal_length, journal_radius, (PLATE_CENTER_X, 0.0, 0.0)))
    shaft = shaft.union(_cyl_x(0.006, collar_radius, (left_collar_center, 0.0, 0.0)))
    shaft = shaft.union(_cyl_x(0.006, collar_radius, (right_collar_center, 0.0, 0.0)))
    shaft = shaft.union(_cyl_x(0.012, journal_radius * 1.10, (-PLATE_CENTER_X + 0.026, 0.0, 0.0)))
    shaft = shaft.union(_cyl_x(0.012, journal_radius * 1.10, (PLATE_CENTER_X - 0.026, 0.0, 0.0)))
    if left_coupling:
        flange_center = -0.205
        shaft = shaft.union(_cyl_x(0.014, 0.028, (flange_center, 0.0, 0.0)))
        shaft = shaft.union(_cyl_x(0.026, 0.017, (flange_center + 0.016, 0.0, 0.0)))
        for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            shaft = shaft.cut(_cyl_x(0.018, 0.0030, (flange_center, 0.016 * math.cos(angle), 0.016 * math.sin(angle))))
    if right_coupling:
        flange_center = 0.205
        shaft = shaft.union(_cyl_x(0.014, 0.028, (flange_center, 0.0, 0.0)))
        shaft = shaft.union(_cyl_x(0.026, 0.017, (flange_center - 0.016, 0.0, 0.0)))
        for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
            shaft = shaft.cut(_cyl_x(0.018, 0.0030, (flange_center, 0.016 * math.cos(angle), 0.016 * math.sin(angle))))
    return shaft


def _shaft_z() -> cq.Workplane:
    shaft = _cyl_z(0.280, 0.0115, (0.0, 0.0, 0.140))
    shaft = shaft.union(_cyl_z(0.038, 0.0170, (0.0, 0.0, 0.040)))
    shaft = shaft.union(_cyl_z(0.038, 0.0170, (0.0, 0.0, 0.215)))
    shaft = shaft.union(_cyl_z(0.006, 0.0240, (0.0, 0.0, BEVEL_COVER_SPLIT_Z - VERTICAL_ORIGIN_Z)))
    shaft = shaft.union(_cyl_z(0.006, 0.0240, (0.0, 0.0, 0.225)))
    shaft = shaft.union(_cyl_z(0.014, 0.030, (0.0, 0.0, 0.257)))
    shaft = shaft.union(_cyl_z(0.024, 0.018, (0.0, 0.0, 0.239)))
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        shaft = shaft.cut(_cyl_z(0.018, 0.0030, (0.016 * math.cos(angle), 0.016 * math.sin(angle), 0.257)))
    return shaft


def _bearing_cap_group(sign: int) -> cq.Workplane:
    cap_group = _box((0.001, 0.001, 0.001), (0.0, 0.0, 0.0))
    cap_group = cap_group.cut(_box((0.002, 0.002, 0.002), (0.0, 0.0, 0.0)))
    cap_local_offsets = (INPUT_Y - INTERMEDIATE_Y, 0.0, LAYSHAFT_Y - INTERMEDIATE_Y)
    outward_center = sign * (CAP_THICKNESS / 2.0)
    for y_offset in cap_local_offsets:
        cap = _box((CAP_THICKNESS, 0.064, 0.022), (outward_center, y_offset, 0.011))
        upper_half = _cyl_x(CAP_THICKNESS, 0.031, (outward_center, y_offset, 0.0)).cut(
            _box((CAP_THICKNESS + 0.010, 0.090, 0.060), (outward_center, y_offset, -0.030))
        )
        cap = cap.union(upper_half)
        cap = cap.cut(_cyl_x(CAP_THICKNESS + 0.012, 0.0185, (outward_center, y_offset, 0.0)))
        for bolt_y in (-0.024, 0.024):
            bolt = _cyl_x(0.008, 0.0050, (sign * (CAP_THICKNESS + 0.004), y_offset + bolt_y, 0.010))
            cap = cap.union(bolt)
        cap_group = cap_group.union(cap)
    return cap_group


def _top_cover_shape() -> cq.Workplane:
    cover = _box((0.314, 0.244, 0.010), (0.0, 0.0, 0.005))
    cover = cover.union(_box((0.314, 0.012, 0.020), (0.0, 0.116, 0.010)))
    cover = cover.union(_box((0.314, 0.012, 0.020), (0.0, -0.116, 0.010)))
    cover = cover.union(_box((0.012, 0.244, 0.020), (0.151, 0.0, 0.010)))
    cover = cover.union(_box((0.012, 0.244, 0.020), (-0.151, 0.0, 0.010)))
    cover = cover.cut(_box((0.196, 0.104, 0.018), (0.0, 0.0, 0.006)))
    cover = cover.union(_box((0.124, 0.068, 0.008), (0.018, 0.0, 0.014)))
    for x_pos in (-0.135, -0.060, 0.060, 0.135):
        for y_pos in (-0.108, 0.108):
            cover = cover.union(_cyl_z(0.006, 0.0045, (x_pos, y_pos, 0.013)))
    return cover


def _bevel_cover_shape() -> cq.Workplane:
    cover = _box((0.118, 0.096, 0.012), (0.0, 0.0, 0.006))
    cover = cover.union(_cyl_z(0.058, 0.044, (0.0, 0.0, 0.033)))
    cover = cover.union(_box((0.072, 0.036, 0.010), (0.0, 0.0, 0.061)))
    cover = cover.cut(_cyl_z(0.090, 0.019, (0.0, 0.0, 0.025)))
    cover = cover.cut(_box((0.048, 0.022, 0.020), (0.0, 0.0, 0.055)))
    for x_pos, y_pos in ((-0.040, -0.028), (-0.040, 0.028), (0.040, -0.028), (0.040, 0.028)):
        cover = cover.union(_cyl_z(0.008, 0.005, (x_pos, y_pos, 0.012)))
    return cover


def _frame_shape() -> cq.Workplane:
    frame = _box((0.380, 0.040, 0.024), (0.0, 0.155, 0.012))
    frame = frame.union(_box((0.380, 0.040, 0.024), (0.0, -0.155, 0.012)))
    frame = frame.union(_box((0.034, 0.310, 0.022), (-0.155, 0.0, 0.011)))
    frame = frame.union(_box((0.034, 0.310, 0.022), (0.155, 0.0, 0.011)))
    frame = frame.union(_box((0.034, 0.220, 0.016), (0.0, 0.0, 0.016)))
    frame = frame.union(_box((0.034, 0.300, 0.020), (0.0, 0.0, 0.223)))
    frame = frame.union(_box((0.332, 0.020, 0.018), (0.0, 0.145, 0.223)))
    frame = frame.union(_box((0.332, 0.020, 0.018), (0.0, -0.145, 0.223)))
    for plate_x in (-PLATE_CENTER_X, PLATE_CENTER_X):
        plate = _box((PLATE_THICKNESS, 0.286, 0.192), (plate_x, 0.0, 0.118))
        plate = plate.cut(_box((PLATE_THICKNESS + 0.006, 0.170, 0.102), (plate_x, 0.0, 0.126)))
        plate = plate.cut(_box((PLATE_THICKNESS + 0.006, 0.090, 0.032), (plate_x, -0.002, 0.187)))
        plate = plate.cut(_cyl_x(0.030, 0.030, (plate_x, 0.110, 0.070)))
        plate = plate.cut(_cyl_x(0.030, 0.024, (plate_x, -0.118, 0.072)))
        frame = frame.union(plate)
        boss_extension = 0.022
        boss_center_x = plate_x + math.copysign((PLATE_THICKNESS + boss_extension) / 2.0, plate_x)
        for y_pos in (INPUT_Y, INTERMEDIATE_Y, LAYSHAFT_Y):
            pad = _box((boss_extension, 0.066, 0.046), (boss_center_x, y_pos, SHAFT_Z - 0.014))
            boss = _cyl_x(boss_extension, 0.031, (boss_center_x, y_pos, SHAFT_Z))
            boss = boss.cut(_box((boss_extension + 0.008, 0.090, 0.060), (boss_center_x, y_pos, SHAFT_Z + 0.030)))
            frame = frame.union(pad).union(boss)
    frame = frame.union(_cyl_z(0.060, 0.046, (VERTICAL_X, VERTICAL_Y, 0.052)))
    frame = frame.union(_box((0.092, 0.080, 0.014), (VERTICAL_X, VERTICAL_Y, BEVEL_COVER_SPLIT_Z - 0.007)))
    frame = frame.union(_box((0.112, 0.018, 0.060), (0.154, 0.130, 0.074)))
    frame = frame.union(_box((0.018, 0.078, 0.082), (0.145, 0.130, 0.085)))
    frame = frame.union(_box((0.070, 0.014, 0.050), (0.120, 0.122, 0.070)))
    frame = frame.union(_box((0.110, 0.014, 0.040), (0.110, 0.020, 0.040)))
    for y_pos in (INPUT_Y, INTERMEDIATE_Y, LAYSHAFT_Y):
        frame = frame.cut(_cyl_x(0.420, 0.0185, (0.0, y_pos, SHAFT_Z)))
    frame = frame.cut(_cyl_z(0.170, 0.0190, (VERTICAL_X, VERTICAL_Y, 0.120)))
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_assembly_study", assets=ASSETS)

    frame_mat = model.material("frame_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    cover_mat = model.material("cover_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    steel_mat = model.material("machined_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    gear_mat = model.material("gear_steel", rgba=(0.52, 0.55, 0.58, 1.0))
    fastener_mat = model.material("fastener_black", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_shape(), "frame_shell.obj", assets=ASSETS), material=frame_mat, name="frame_shell")
    frame.inertial = Inertial.from_geometry(Box((0.380, 0.330, 0.245)), mass=34.0, origin=Origin(xyz=(0.0, 0.0, 0.1225)))

    left_caps = model.part("left_bearing_caps")
    left_caps.visual(
        mesh_from_cadquery(_bearing_cap_group(-1), "left_bearing_caps.obj", assets=ASSETS),
        material=cover_mat,
        name="left_caps",
    )
    left_caps.inertial = Inertial.from_geometry(Box((0.032, 0.240, 0.050)), mass=3.6)

    right_caps = model.part("right_bearing_caps")
    right_caps.visual(
        mesh_from_cadquery(_bearing_cap_group(1), "right_bearing_caps.obj", assets=ASSETS),
        material=cover_mat,
        name="right_caps",
    )
    right_caps.inertial = Inertial.from_geometry(Box((0.032, 0.240, 0.050)), mass=3.6)

    top_cover = model.part("top_inspection_cover")
    top_cover.visual(
        mesh_from_cadquery(_top_cover_shape(), "top_inspection_cover.obj", assets=ASSETS),
        material=cover_mat,
        name="top_cover",
    )
    top_cover.inertial = Inertial.from_geometry(Box((0.314, 0.244, 0.026)), mass=3.2, origin=Origin(xyz=(0.0, 0.0, 0.013)))

    bevel_cover = model.part("bevel_housing_cover")
    bevel_cover.visual(
        mesh_from_cadquery(_bevel_cover_shape(), "bevel_housing_cover.obj", assets=ASSETS),
        material=cover_mat,
        name="bevel_cover",
    )
    bevel_cover.inertial = Inertial.from_geometry(Box((0.118, 0.096, 0.072)), mass=2.8, origin=Origin(xyz=(0.0, 0.0, 0.036)))

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(
            _shaft_x(
                total_length=0.420,
                core_radius=0.0105,
                journal_radius=0.0168,
                journal_length=0.036,
                collar_radius=0.0235,
                left_collar_center=-(PLATE_CENTER_X + CAP_THICKNESS + 0.003),
                right_collar_center=PLATE_CENTER_X + CAP_THICKNESS + 0.003,
                left_coupling=True,
            ),
            "input_shaft_body.obj",
            assets=ASSETS,
        ),
        material=steel_mat,
        name="shaft_body",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            _spur_gear_x(
                teeth=STAGE1_PINION_TEETH,
                root_radius=0.0285,
                outer_radius=0.0350,
                width=0.020,
                hub_radius=0.020,
                hub_length=0.030,
                bore_radius=0.0120,
                phase=0.08,
                hole_count=4,
            ).translate((-0.078, 0.0, 0.0)),
            "input_stage1_pinion.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="stage1_pinion",
    )
    input_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.420),
        mass=6.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    intermediate_shaft = model.part("intermediate_shaft")
    intermediate_shaft.visual(
        mesh_from_cadquery(
            _shaft_x(
                total_length=0.400,
                core_radius=0.0115,
                journal_radius=0.0175,
                journal_length=0.036,
                collar_radius=0.0240,
                left_collar_center=-(PLATE_CENTER_X + CAP_THICKNESS + 0.003),
                right_collar_center=PLATE_CENTER_X + CAP_THICKNESS + 0.003,
            ),
            "intermediate_shaft_body.obj",
            assets=ASSETS,
        ),
        material=steel_mat,
        name="shaft_body",
    )
    intermediate_shaft.visual(
        mesh_from_cadquery(
            _spur_gear_x(
                teeth=STAGE1_GEAR_TEETH,
                root_radius=0.0490,
                outer_radius=0.0570,
                width=0.022,
                hub_radius=0.024,
                hub_length=0.034,
                bore_radius=0.0130,
                phase=0.18,
                hole_count=6,
            ).translate((-0.078, 0.0, 0.0)),
            "intermediate_stage1_gear.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="stage1_gear",
    )
    intermediate_shaft.visual(
        mesh_from_cadquery(
            _spur_gear_x(
                teeth=STAGE2_PINION_TEETH,
                root_radius=0.0245,
                outer_radius=0.0315,
                width=0.018,
                hub_radius=0.020,
                hub_length=0.026,
                bore_radius=0.0130,
                phase=0.03,
                web_scale=0.80,
                hole_count=4,
            ).translate((0.044, 0.0, 0.0)),
            "intermediate_stage2_pinion.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="stage2_pinion",
    )
    intermediate_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.400),
        mass=9.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    layshaft = model.part("layshaft")
    layshaft.visual(
        mesh_from_cadquery(
            _shaft_x(
                total_length=0.420,
                core_radius=0.0120,
                journal_radius=0.0180,
                journal_length=0.036,
                collar_radius=0.0245,
                left_collar_center=-(PLATE_CENTER_X + CAP_THICKNESS + 0.003),
                right_collar_center=PLATE_CENTER_X + CAP_THICKNESS + 0.003,
                right_coupling=True,
            ),
            "layshaft_body.obj",
            assets=ASSETS,
        ),
        material=steel_mat,
        name="shaft_body",
    )
    layshaft.visual(
        mesh_from_cadquery(
            _spur_gear_x(
                teeth=STAGE2_GEAR_TEETH,
                root_radius=0.0560,
                outer_radius=0.0640,
                width=0.022,
                hub_radius=0.026,
                hub_length=0.036,
                bore_radius=0.0135,
                phase=0.11,
                hole_count=6,
            ).translate((0.028, 0.0, 0.0)),
            "layshaft_stage2_gear.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="stage2_gear",
    )
    layshaft.visual(
        mesh_from_cadquery(
            _bevel_gear_x(
                face_width=0.028,
                large_radius=0.034,
                small_radius=0.018,
                bore_radius=0.0135,
                hub_radius=0.022,
                hub_length=0.036,
                teeth=BEVEL_PINION_TEETH,
                phase=0.21,
            ).translate((0.094, 0.0, 0.0)),
            "layshaft_bevel_pinion.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="bevel_pinion",
    )
    layshaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.420),
        mass=10.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    vertical_output = model.part("vertical_output")
    vertical_output.visual(
        mesh_from_cadquery(_shaft_z(), "vertical_output_shaft.obj", assets=ASSETS),
        material=steel_mat,
        name="shaft_body",
    )
    vertical_output.visual(
        mesh_from_cadquery(
            _bevel_gear_z(
                face_width=0.028,
                large_radius=0.044,
                small_radius=0.020,
                bore_radius=0.0130,
                hub_radius=0.023,
                hub_length=0.036,
                teeth=BEVEL_GEAR_TEETH,
                phase=0.04,
            ).translate((0.0, 0.0, 0.121)),
            "vertical_output_bevel_gear.obj",
            assets=ASSETS,
        ),
        material=gear_mat,
        name="bevel_gear",
    )
    vertical_output.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.280),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
    )

    model.articulation(
        "frame_to_left_caps",
        ArticulationType.FIXED,
        parent=frame,
        child=left_caps,
        origin=Origin(xyz=(-PLATE_CENTER_X - PLATE_THICKNESS / 2.0, INTERMEDIATE_Y, SHAFT_Z)),
    )
    model.articulation(
        "frame_to_right_caps",
        ArticulationType.FIXED,
        parent=frame,
        child=right_caps,
        origin=Origin(xyz=(PLATE_CENTER_X + PLATE_THICKNESS / 2.0, INTERMEDIATE_Y, SHAFT_Z)),
    )
    model.articulation(
        "frame_to_top_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=top_cover,
        origin=Origin(xyz=(0.0, 0.0, TOP_COVER_SPLIT_Z)),
    )
    model.articulation(
        "frame_to_bevel_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=bevel_cover,
        origin=Origin(xyz=(VERTICAL_X, VERTICAL_Y, BEVEL_COVER_SPLIT_Z)),
    )
    model.articulation(
        "frame_to_input_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(0.0, INPUT_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_intermediate_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=intermediate_shaft,
        origin=Origin(xyz=(0.0, INTERMEDIATE_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=10.0),
    )
    model.articulation(
        "frame_to_layshaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=layshaft,
        origin=Origin(xyz=(0.0, LAYSHAFT_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=9.0),
    )
    model.articulation(
        "frame_to_vertical_output",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=vertical_output,
        origin=Origin(xyz=(VERTICAL_X, VERTICAL_Y, VERTICAL_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    frame = object_model.get_part("frame")
    left_caps = object_model.get_part("left_bearing_caps")
    right_caps = object_model.get_part("right_bearing_caps")
    top_cover = object_model.get_part("top_inspection_cover")
    bevel_cover = object_model.get_part("bevel_housing_cover")
    input_shaft = object_model.get_part("input_shaft")
    intermediate_shaft = object_model.get_part("intermediate_shaft")
    layshaft = object_model.get_part("layshaft")
    vertical_output = object_model.get_part("vertical_output")

    left_caps_shell = left_caps.get_visual("left_caps")
    right_caps_shell = right_caps.get_visual("right_caps")
    top_cover_shell = top_cover.get_visual("top_cover")
    bevel_cover_shell = bevel_cover.get_visual("bevel_cover")
    input_pinion = input_shaft.get_visual("stage1_pinion")
    stage1_gear = intermediate_shaft.get_visual("stage1_gear")
    stage2_pinion = intermediate_shaft.get_visual("stage2_pinion")
    stage2_gear = layshaft.get_visual("stage2_gear")
    bevel_pinion = layshaft.get_visual("bevel_pinion")
    bevel_gear = vertical_output.get_visual("bevel_gear")

    input_joint = object_model.get_articulation("frame_to_input_shaft")
    intermediate_joint = object_model.get_articulation("frame_to_intermediate_shaft")
    layshaft_joint = object_model.get_articulation("frame_to_layshaft")
    vertical_joint = object_model.get_articulation("frame_to_vertical_output")

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

    ctx.expect_contact(left_caps, frame, elem_a=left_caps_shell, elem_b=frame.get_visual("frame_shell"), contact_tol=0.0008)
    ctx.expect_contact(right_caps, frame, elem_a=right_caps_shell, elem_b=frame.get_visual("frame_shell"), contact_tol=0.0008)
    ctx.expect_contact(top_cover, frame, elem_a=top_cover_shell, elem_b=frame.get_visual("frame_shell"), contact_tol=0.0008)
    ctx.expect_contact(bevel_cover, frame, elem_a=bevel_cover_shell, elem_b=frame.get_visual("frame_shell"), contact_tol=0.0008)

    ctx.expect_gap(frame, left_caps, axis="x", max_gap=0.0010, max_penetration=0.0)
    ctx.expect_gap(right_caps, frame, axis="x", max_gap=0.0010, max_penetration=0.0)
    ctx.expect_gap(top_cover, frame, axis="z", max_gap=0.0010, max_penetration=0.0)
    ctx.expect_gap(bevel_cover, frame, axis="z", max_gap=0.0010, max_penetration=0.0)

    ctx.expect_origin_distance(input_shaft, intermediate_shaft, axes="y", min_dist=0.094, max_dist=0.096)
    ctx.expect_origin_distance(intermediate_shaft, layshaft, axes="y", min_dist=0.099, max_dist=0.101)
    ctx.expect_origin_distance(vertical_output, layshaft, axes="y", max_dist=0.001)
    ctx.expect_origin_distance(vertical_output, layshaft, axes="x", min_dist=0.109, max_dist=0.111)

    ctx.expect_overlap(input_shaft, intermediate_shaft, axes="xz", min_overlap=0.018, elem_a=input_pinion, elem_b=stage1_gear)
    ctx.expect_overlap(intermediate_shaft, layshaft, axes="xz", min_overlap=0.016, elem_a=stage2_pinion, elem_b=stage2_gear)
    ctx.expect_overlap(vertical_output, layshaft, axes="yz", min_overlap=0.020, elem_a=bevel_gear, elem_b=bevel_pinion)

    operating_input = 0.42
    operating_intermediate = -operating_input * STAGE1_PINION_TEETH / STAGE1_GEAR_TEETH
    operating_layshaft = -operating_intermediate * STAGE2_PINION_TEETH / STAGE2_GEAR_TEETH
    operating_vertical = -operating_layshaft * BEVEL_PINION_TEETH / BEVEL_GEAR_TEETH

    with ctx.pose(
        {
            input_joint: operating_input,
            intermediate_joint: operating_intermediate,
            layshaft_joint: operating_layshaft,
            vertical_joint: operating_vertical,
        }
    ):
        ctx.expect_overlap(input_shaft, intermediate_shaft, axes="xz", min_overlap=0.018, elem_a=input_pinion, elem_b=stage1_gear)
        ctx.expect_overlap(intermediate_shaft, layshaft, axes="xz", min_overlap=0.016, elem_a=stage2_pinion, elem_b=stage2_gear)
        ctx.expect_overlap(vertical_output, layshaft, axes="yz", min_overlap=0.020, elem_a=bevel_gear, elem_b=bevel_pinion)
        ctx.expect_gap(top_cover, frame, axis="z", max_gap=0.0010, max_penetration=0.0)
        ctx.expect_origin_distance(input_shaft, intermediate_shaft, axes="y", min_dist=0.094, max_dist=0.096)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
