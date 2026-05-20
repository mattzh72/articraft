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

BASE_PLATE = (0.24, 0.18, 0.02)
COLUMN = (0.09, 0.12, 0.19)
TOP_PLATE = (0.14, 0.10, 0.02)
SHOULDER_Z = 0.236

UPPER_BEAM_START = 0.070
UPPER_BEAM_LENGTH = 0.258
UPPER_BEAM_WIDTH = 0.085
UPPER_BEAM_HEIGHT = 0.075
UPPER_BEAM_Z = 0.045
ELBOW_X = 0.330
ELBOW_Z = 0.045

FOREARM_BEAM_START = 0.040
FOREARM_BEAM_LENGTH = 0.225
FOREARM_BEAM_WIDTH = 0.072
FOREARM_BEAM_HEIGHT = 0.062
WRIST_X = 0.272


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .translate(center)
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _annulus_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate(center)
    )


def _annulus_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _annulus_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _combine(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _bolt_circle_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + ((2.0 * math.pi * i) / count)),
            radius * math.sin(phase + ((2.0 * math.pi * i) / count)),
        )
        for i in range(count)
    ]


def _beam_module(
    *,
    start_x: float,
    length: float,
    width: float,
    height: float,
    center_z: float,
    wall: float,
    top_cover_width: float,
    panel_side: float,
) -> cq.Workplane:
    outer = _box((length, width, height), (start_x + (length / 2.0), 0.0, center_z))
    inner = _box(
        (length - 0.024, width - (2.0 * wall), height - (2.0 * wall)),
        (start_x + (length / 2.0), 0.0, center_z),
    )
    beam = outer.cut(inner)

    for frac in (0.32, 0.70):
        beam = beam.cut(
            _box(
                (length * 0.18, width + 0.010, height * 0.42),
                (start_x + (length * frac), 0.0, center_z - (height * 0.02)),
            )
        )

    cover_t = 0.008
    top_cover = _box(
        (length * 0.58, top_cover_width, cover_t),
        (
            start_x + (length * 0.56),
            0.0,
            center_z + (height / 2.0) + (cover_t / 2.0),
        ),
    )
    panel_t = 0.004
    panel = _box(
        (length * 0.40, panel_t, height * 0.56),
        (
            start_x + (length * 0.52),
            panel_side * ((width / 2.0) + (panel_t / 2.0)),
            center_z - 0.004,
        ),
    )

    assembly = beam.union(top_cover).union(panel)

    for x in (
        start_x + (length * 0.34),
        start_x + (length * 0.52),
        start_x + (length * 0.70),
    ):
        assembly = assembly.union(
            _cyl_z(
                0.003,
                0.005,
                (
                    x,
                    0.0,
                    center_z + (height / 2.0) + cover_t + 0.0025,
                ),
            )
        )

    for x, z in (
        (start_x + (length * 0.39), center_z + (height * 0.17)),
        (start_x + (length * 0.65), center_z + (height * 0.17)),
        (start_x + (length * 0.39), center_z - (height * 0.19)),
        (start_x + (length * 0.65), center_z - (height * 0.19)),
    ):
        assembly = assembly.union(
            _cyl_y(
                0.0028,
                0.005,
                (
                    x,
                    panel_side * ((width / 2.0) + panel_t + 0.0025),
                    z,
                ),
            )
        )

    return assembly


def _build_base_structure() -> cq.Workplane:
    base_plate = _box(BASE_PLATE, (0.0, 0.0, BASE_PLATE[2] / 2.0))

    column_outer = _box(COLUMN, (0.0, 0.0, BASE_PLATE[2] + (COLUMN[2] / 2.0)))
    column_inner = _box(
        (0.066, 0.096, 0.166),
        (0.0, 0.0, BASE_PLATE[2] + (COLUMN[2] / 2.0)),
    )
    column = column_outer.cut(column_inner)

    top_plate = _box(TOP_PLATE, (0.0, 0.0, 0.210))

    brace_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.045, 0.020),
                (0.045, 0.020),
                (0.045, 0.126),
                (0.012, 0.200),
                (-0.045, 0.200),
            ]
        )
        .close()
        .extrude(0.014)
    )
    brace_pos = brace_profile.translate((0.0, 0.053, 0.0))
    brace_neg = brace_profile.translate((0.0, -0.067, 0.0))

    access_cover = _box((0.058, 0.004, 0.098), (0.0, (COLUMN[1] / 2.0) + 0.002, 0.115))

    structure = _combine(base_plate, column, top_plate, brace_pos, brace_neg, access_cover)

    for x, z in ((-0.018, 0.150), (0.018, 0.150), (-0.018, 0.080), (0.018, 0.080)):
        structure = structure.union(
            _cyl_y(0.0032, 0.005, (x, (COLUMN[1] / 2.0) + 0.0045, z))
        )

    for x, y in (
        (-0.090, -0.060),
        (-0.090, 0.060),
        (0.090, -0.060),
        (0.090, 0.060),
    ):
        structure = structure.union(_cyl_z(0.010, 0.012, (x, y, 0.006)))

    return structure


def _build_base_rear_access_cover() -> cq.Workplane:
    panel_center_y = -(COLUMN[1] / 2.0) - 0.002
    cover = _box((0.058, 0.004, 0.098), (0.0, panel_center_y, 0.115))

    for x, z in ((-0.018, 0.150), (0.018, 0.150), (-0.018, 0.080), (0.018, 0.080)):
        cover = cover.union(_cyl_y(0.0032, 0.005, (x, panel_center_y - 0.0045, z)))

    return cover


def _build_shoulder_ring() -> cq.Workplane:
    ring = _annulus_z(0.075, 0.048, 0.032, (0.0, 0.0, SHOULDER_Z))
    ring = ring.cut(
        cq.Workplane("XY", origin=(0.0, 0.0, SHOULDER_Z - 0.018))
        .pushPoints(_bolt_circle_points(0.061, 10, phase=math.pi / 10.0))
        .circle(0.0035)
        .extrude(0.036)
    )

    for x, y in _bolt_circle_points(0.061, 10, phase=math.pi / 10.0):
        ring = ring.union(_cyl_z(0.0042, 0.004, (x, y, SHOULDER_Z + 0.018)))

    return ring


def _build_upper_shoulder_hub() -> cq.Workplane:
    lift = 0.004
    collar = _annulus_z(0.060, 0.020, 0.008, (0.0, 0.0, 0.020 + lift))
    drum = _annulus_z(0.045, 0.018, 0.044, (0.0, 0.0, 0.046 + lift))
    top_ring = _annulus_z(0.052, 0.028, 0.012, (0.0, 0.0, 0.072 + lift))
    drive_block = _box((0.072, 0.072, 0.048), (0.056, 0.0, 0.046 + lift))

    hub = _combine(collar, drum, top_ring, drive_block)

    for x, y in _bolt_circle_points(0.040, 8):
        hub = hub.union(_cyl_z(0.0032, 0.005, (x, y, 0.0815 + lift)))

    return hub


def _build_upper_beam() -> cq.Workplane:
    beam = _beam_module(
        start_x=UPPER_BEAM_START,
        length=UPPER_BEAM_LENGTH,
        width=UPPER_BEAM_WIDTH,
        height=UPPER_BEAM_HEIGHT,
        center_z=UPPER_BEAM_Z,
        wall=0.010,
        top_cover_width=0.032,
        panel_side=1.0,
    )
    beam = beam.cut(_cyl_y(0.0205, 0.120, (ELBOW_X, 0.0, ELBOW_Z)))
    beam = beam.cut(_box((0.040, 0.058, 0.050), (ELBOW_X - 0.006, 0.0, ELBOW_Z)))
    return beam


def _build_cable_cover(
    *,
    start_x: float,
    length: float,
    width: float,
    center_z: float,
    cover_height: float,
    side: float,
) -> cq.Workplane:
    plate_t = 0.008
    cover_length = length * 0.44
    x_center = start_x + (length * 0.53)
    outer_y = side * ((width / 2.0) + (plate_t / 2.0))

    body = _box((cover_length, plate_t, cover_height), (x_center, outer_y, center_z + 0.002))
    top_lip = _box(
        (cover_length * 0.72, 0.018, 0.006),
        (
            x_center,
            side * ((width / 2.0) - 0.009),
            center_z + (cover_height / 2.0) + 0.005,
        ),
    )
    bottom_lip = _box(
        (cover_length * 0.66, 0.014, 0.006),
        (
            x_center,
            side * ((width / 2.0) - 0.007),
            center_z - (cover_height / 2.0) - 0.001,
        ),
    )

    cover = _combine(body, top_lip, bottom_lip)

    fastener_y = side * ((width / 2.0) + plate_t + 0.0025)
    for x, z in (
        (x_center - (cover_length * 0.30), center_z + (cover_height * 0.25)),
        (x_center + (cover_length * 0.30), center_z + (cover_height * 0.25)),
        (x_center - (cover_length * 0.30), center_z - (cover_height * 0.22)),
        (x_center + (cover_length * 0.30), center_z - (cover_height * 0.22)),
    ):
        cover = cover.union(_cyl_y(0.0028, 0.005, (x, fastener_y, z)))

    return cover


def _build_upper_cable_cover() -> cq.Workplane:
    return _build_cable_cover(
        start_x=UPPER_BEAM_START,
        length=UPPER_BEAM_LENGTH,
        width=UPPER_BEAM_WIDTH,
        center_z=UPPER_BEAM_Z,
        cover_height=0.042,
        side=-1.0,
    )


def _build_upper_support_webs() -> cq.Workplane:
    web_pos = _box((0.032, 0.008, 0.022), (0.286, 0.026, 0.090))
    web_neg = _box((0.032, 0.008, 0.022), (0.286, -0.026, 0.090))
    tie_cap = _box((0.038, 0.056, 0.008), (0.290, 0.0, 0.103))
    rear_pad = _box((0.024, 0.044, 0.010), (0.274, 0.0, 0.082))

    assembly = _combine(web_pos, web_neg, tie_cap, rear_pad)
    for y in (-0.020, 0.020):
        assembly = assembly.union(_cyl_z(0.0030, 0.004, (0.290, y, 0.109)))

    return assembly


def _build_elbow_clevis() -> cq.Workplane:
    plate_y = 0.050
    plate_t = 0.010
    plate = _box((0.064, plate_t, 0.112), (ELBOW_X - 0.014, plate_y, ELBOW_Z))
    opposite_plate = _box((0.064, plate_t, 0.112), (ELBOW_X - 0.014, -plate_y, ELBOW_Z))

    rear_saddle = _box((0.056, 0.060, 0.076), (ELBOW_X - 0.068, 0.0, ELBOW_Z + 0.002))
    upper_tie = _box((0.034, 0.108, 0.012), (ELBOW_X - 0.048, 0.0, ELBOW_Z + 0.042))
    lower_tie = _box((0.026, 0.092, 0.010), (ELBOW_X - 0.050, 0.0, ELBOW_Z - 0.042))
    upper_rib = _box((0.020, 0.068, 0.012), (ELBOW_X - 0.028, 0.0, ELBOW_Z + 0.024))
    lower_rib = _box((0.020, 0.068, 0.012), (ELBOW_X - 0.028, 0.0, ELBOW_Z - 0.024))

    clevis = _combine(
        plate,
        opposite_plate,
        rear_saddle,
        upper_tie,
        lower_tie,
        upper_rib,
        lower_rib,
    )
    clevis = clevis.cut(_cyl_y(0.0161, 0.140, (ELBOW_X, 0.0, ELBOW_Z)))
    clevis = clevis.cut(_box((0.094, 0.088, 0.080), (ELBOW_X + 0.014, 0.0, ELBOW_Z)))
    clevis = clevis.cut(_box((0.034, 0.062, 0.046), (ELBOW_X - 0.018, 0.0, ELBOW_Z)))

    for side in (-1.0, 1.0):
        clevis = clevis.union(_annulus_y(0.026, 0.0165, 0.004, (ELBOW_X, side * 0.057, ELBOW_Z)))
        clevis = clevis.union(_box((0.024, 0.004, 0.048), (ELBOW_X - 0.018, side * 0.057, ELBOW_Z)))
        for x, z in (
            (ELBOW_X - 0.016, ELBOW_Z + 0.024),
            (ELBOW_X - 0.016, ELBOW_Z - 0.024),
        ):
            clevis = clevis.union(_cyl_y(0.0042, 0.004, (x, side * 0.057, z)))
        for x, z in (
            (ELBOW_X - 0.014, ELBOW_Z + 0.018),
            (ELBOW_X + 0.008, ELBOW_Z + 0.018),
            (ELBOW_X - 0.014, ELBOW_Z - 0.018),
            (ELBOW_X + 0.008, ELBOW_Z - 0.018),
        ):
            clevis = clevis.union(_cyl_y(0.0026, 0.003, (x, side * 0.0615, z)))

    return clevis


def _build_forearm_elbow_hub() -> cq.Workplane:
    shaft = _cyl_y(0.0145, 0.090, (0.0, 0.0, 0.0))
    collar_pos = _annulus_y(0.020, 0.0145, 0.008, (0.0, 0.041, 0.0))
    collar_neg = _annulus_y(0.020, 0.0145, 0.008, (0.0, -0.041, 0.0))

    knuckle_core = _box((0.018, 0.014, 0.024), (0.016, 0.0, 0.0))
    top_link = _box((0.046, 0.010, 0.012), (0.056, 0.0, 0.018))
    bottom_link = _box((0.046, 0.010, 0.012), (0.056, 0.0, -0.018))
    center_spine = _box((0.026, 0.012, 0.020), (0.040, 0.0, 0.0))
    beam_adapter = _box((0.090, 0.040, 0.054), (0.110, 0.0, 0.0))
    beam_adapter = beam_adapter.cut(_box((0.046, 0.016, 0.024), (0.118, 0.0, 0.0)))
    top_gusset = _box((0.020, 0.024, 0.012), (0.074, 0.0, 0.020))
    bottom_gusset = _box((0.020, 0.024, 0.012), (0.074, 0.0, -0.020))

    hub = _combine(
        shaft,
        collar_pos,
        collar_neg,
        knuckle_core,
        top_link,
        bottom_link,
        center_spine,
        beam_adapter,
        top_gusset,
        bottom_gusset,
    )

    for side in (-1.0, 1.0):
        for x, z in ((-0.012, 0.016), (0.012, 0.016), (-0.012, -0.016), (0.012, -0.016)):
            hub = hub.union(_cyl_y(0.0028, 0.004, (x, side * 0.062, z)))

    return hub


def _build_forearm_beam() -> cq.Workplane:
    return _beam_module(
        start_x=FOREARM_BEAM_START,
        length=FOREARM_BEAM_LENGTH,
        width=FOREARM_BEAM_WIDTH,
        height=FOREARM_BEAM_HEIGHT,
        center_z=0.0,
        wall=0.010,
        top_cover_width=0.026,
        panel_side=-1.0,
    )


def _build_forearm_cable_cover() -> cq.Workplane:
    return _build_cable_cover(
        start_x=FOREARM_BEAM_START,
        length=FOREARM_BEAM_LENGTH,
        width=FOREARM_BEAM_WIDTH,
        center_z=0.0,
        cover_height=0.038,
        side=1.0,
    )


def _build_forearm_support_webs() -> cq.Workplane:
    web_pos = _box((0.042, 0.010, 0.052), (0.244, 0.022, 0.0))
    web_neg = _box((0.042, 0.010, 0.052), (0.244, -0.022, 0.0))
    tie_cap = _box((0.030, 0.050, 0.010), (0.246, 0.0, 0.028))

    assembly = _combine(web_pos, web_neg, tie_cap)
    for y in (-0.018, 0.018):
        assembly = assembly.union(_cyl_z(0.0028, 0.004, (0.247, y, 0.034)))

    return assembly


def _build_wrist_ring() -> cq.Workplane:
    ring = _annulus_x(0.052, 0.030, 0.034, (WRIST_X, 0.0, 0.0))
    bridge = _box((0.040, 0.082, 0.060), (WRIST_X - 0.028, 0.0, 0.0))
    bridge = bridge.cut(_box((0.024, 0.044, 0.030), (WRIST_X - 0.030, 0.0, 0.0)))
    ring = ring.union(bridge)

    for y, z in _bolt_circle_points(0.040, 8, phase=math.pi / 8.0):
        ring = ring.union(_cyl_x(0.0032, 0.004, (WRIST_X + 0.019, y, z)))

    return ring


def _build_wrist_hub() -> cq.Workplane:
    collar = _annulus_x(0.046, 0.020, 0.008, (0.021, 0.0, 0.0))
    barrel = _annulus_x(0.032, 0.018, 0.060, (0.055, 0.0, 0.0))
    joint_ring = _annulus_x(0.039, 0.028, 0.012, (0.037, 0.0, 0.0))
    hub = _combine(collar, barrel, joint_ring)

    for y, z in _bolt_circle_points(0.032, 6):
        hub = hub.union(_cyl_x(0.0030, 0.004, (0.046, y, z)))

    return hub


def _build_wrist_bracket() -> cq.Workplane:
    bracket = _box((0.070, 0.078, 0.080), (0.082, 0.0, 0.0))
    bracket = bracket.cut(_box((0.045, 0.050, 0.045), (0.078, 0.0, 0.0)))
    bracket = bracket.union(_box((0.026, 0.070, 0.090), (0.100, 0.0, 0.0)))
    return bracket


def _build_wrist_flange() -> cq.Workplane:
    flange = _annulus_x(0.048, 0.022, 0.014, (0.122, 0.0, 0.0))
    flange = flange.cut(
        cq.Workplane("YZ", origin=(0.115, 0.0, 0.0))
        .pushPoints(_bolt_circle_points(0.034, 6))
        .circle(0.0042)
        .extrude(0.020)
    )
    for y, z in _bolt_circle_points(0.034, 6):
        flange = flange.union(_cyl_x(0.0050, 0.004, (0.132, y, z)))
    return flange


def _build_wrist_cable_cover() -> cq.Workplane:
    cover = _box((0.055, 0.024, 0.012), (0.072, 0.0, 0.040))
    cover = cover.union(_cyl_z(0.0028, 0.004, (0.060, 0.0, 0.048)))
    cover = cover.union(_cyl_z(0.0028, 0.004, (0.084, 0.0, 0.048)))
    return cover


def _build_wrist_service_cover() -> cq.Workplane:
    cover = _box((0.054, 0.028, 0.010), (0.078, 0.0, -0.051))
    cover = cover.union(_cyl_z(0.0028, 0.004, (0.064, 0.0, -0.057)))
    cover = cover.union(_cyl_z(0.0028, 0.004, (0.092, 0.0, -0.057)))
    cover = cover.union(_box((0.010, 0.010, 0.010), (0.062, 0.0, -0.041)))
    cover = cover.union(_box((0.010, 0.010, 0.010), (0.094, 0.0, -0.041)))
    return cover


def _add_visual(part, shape: cq.Workplane, filename: str, *, name: str, material) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_study_arm", assets=ASSETS)

    structure = model.material("structure", rgba=(0.24, 0.27, 0.30, 1.0))
    machined = model.material("machined", rgba=(0.61, 0.64, 0.68, 1.0))
    cover = model.material("cover", rgba=(0.41, 0.44, 0.47, 1.0))

    base_frame = model.part("base_frame")
    _add_visual(
        base_frame,
        _build_base_structure(),
        "base_structure.obj",
        name="base_structure",
        material=structure,
    )
    _add_visual(
        base_frame,
        _build_shoulder_ring(),
        "base_shoulder_ring.obj",
        name="shoulder_ring",
        material=machined,
    )
    _add_visual(
        base_frame,
        _build_base_rear_access_cover(),
        "base_rear_access_cover.obj",
        name="rear_access_cover",
        material=cover,
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.26)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    upper_arm = model.part("upper_arm")
    _add_visual(
        upper_arm,
        _build_upper_shoulder_hub(),
        "upper_shoulder_hub.obj",
        name="shoulder_hub",
        material=machined,
    )
    _add_visual(
        upper_arm,
        _build_upper_beam(),
        "upper_beam.obj",
        name="upper_beam",
        material=structure,
    )
    _add_visual(
        upper_arm,
        _build_upper_cable_cover(),
        "upper_cable_cover.obj",
        name="upper_cable_cover",
        material=cover,
    )
    _add_visual(
        upper_arm,
        _build_elbow_clevis(),
        "upper_elbow_clevis.obj",
        name="elbow_clevis",
        material=cover,
    )
    _add_visual(
        upper_arm,
        _build_upper_support_webs(),
        "upper_support_webs.obj",
        name="upper_support_webs",
        material=structure,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.11)),
        mass=14.0,
        origin=Origin(xyz=(0.17, 0.0, 0.05)),
    )

    forearm = model.part("forearm")
    _add_visual(
        forearm,
        _build_forearm_elbow_hub(),
        "forearm_elbow_hub.obj",
        name="elbow_hub",
        material=machined,
    )
    _add_visual(
        forearm,
        _build_forearm_beam(),
        "forearm_beam.obj",
        name="forearm_beam",
        material=structure,
    )
    _add_visual(
        forearm,
        _build_forearm_cable_cover(),
        "forearm_cable_cover.obj",
        name="forearm_cable_cover",
        material=cover,
    )
    _add_visual(
        forearm,
        _build_wrist_ring(),
        "forearm_wrist_ring.obj",
        name="wrist_ring",
        material=cover,
    )
    _add_visual(
        forearm,
        _build_forearm_support_webs(),
        "forearm_support_webs.obj",
        name="forearm_support_webs",
        material=structure,
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.31, 0.09, 0.09)),
        mass=9.5,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    _add_visual(
        wrist,
        _build_wrist_hub(),
        "wrist_hub.obj",
        name="wrist_hub",
        material=machined,
    )
    _add_visual(
        wrist,
        _build_wrist_bracket(),
        "wrist_bracket.obj",
        name="wrist_bracket",
        material=structure,
    )
    _add_visual(
        wrist,
        _build_wrist_flange(),
        "wrist_flange.obj",
        name="wrist_flange",
        material=machined,
    )
    _add_visual(
        wrist,
        _build_wrist_cable_cover(),
        "wrist_cable_cover.obj",
        name="wrist_cable_cover",
        material=cover,
    )
    _add_visual(
        wrist,
        _build_wrist_service_cover(),
        "wrist_service_cover.obj",
        name="wrist_service_cover",
        material=cover,
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.10)),
        mass=4.2,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_X, 0.0, ELBOW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-1.8, upper=1.2),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(WRIST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.4, lower=-3.1, upper=3.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    shoulder_ring = base_frame.get_visual("shoulder_ring")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    elbow_clevis = upper_arm.get_visual("elbow_clevis")
    elbow_hub = forearm.get_visual("elbow_hub")
    wrist_ring = forearm.get_visual("wrist_ring")
    wrist_hub = wrist.get_visual("wrist_hub")
    wrist_cover = wrist.get_visual("wrist_cable_cover")
    wrist_service_cover = wrist.get_visual("wrist_service_cover")
    wrist_flange = wrist.get_visual("wrist_flange")
    rear_access_cover = base_frame.get_visual("rear_access_cover")
    upper_beam = upper_arm.get_visual("upper_beam")
    upper_cable_cover = upper_arm.get_visual("upper_cable_cover")
    upper_support_webs = upper_arm.get_visual("upper_support_webs")
    forearm_beam = forearm.get_visual("forearm_beam")
    forearm_cable_cover = forearm.get_visual("forearm_cable_cover")
    forearm_support_webs = forearm.get_visual("forearm_support_webs")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a=elbow_hub,
        elem_b=elbow_clevis,
        reason="The elbow study intentionally uses a nested trunnion-and-clevis joint with the forearm hub captured inside the upper-arm clevis around the bearing axis.",
    )

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_adjacent=True,
    )

    ctx.check(
        "parts_present",
        all(part is not None for part in (base_frame, upper_arm, forearm, wrist)),
        "Expected base_frame, upper_arm, forearm, and wrist parts.",
    )
    ctx.check(
        "detail_visuals_present",
        all(
            visual is not None
            for visual in (
                rear_access_cover,
                upper_cable_cover,
                upper_support_webs,
                forearm_cable_cover,
                forearm_support_webs,
                wrist_service_cover,
            )
        ),
        "Expected access covers, cable covers, and support webs on the mechanical study arm.",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    def _interval(aabb, axis_index: int):
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[axis_index], upper[axis_index])

    def _overlap(interval_a, interval_b):
        if interval_a is None or interval_b is None:
            return None
        return min(interval_a[1], interval_b[1]) - max(interval_a[0], interval_b[0])

    rear_cover_aabb = ctx.part_element_world_aabb(base_frame, elem=rear_access_cover)
    upper_beam_aabb = ctx.part_element_world_aabb(upper_arm, elem=upper_beam)
    upper_cover_aabb = ctx.part_element_world_aabb(upper_arm, elem=upper_cable_cover)
    upper_webs_aabb = ctx.part_element_world_aabb(upper_arm, elem=upper_support_webs)
    elbow_clevis_aabb = ctx.part_element_world_aabb(upper_arm, elem=elbow_clevis)
    forearm_beam_aabb = ctx.part_element_world_aabb(forearm, elem=forearm_beam)
    forearm_cover_aabb = ctx.part_element_world_aabb(forearm, elem=forearm_cable_cover)
    forearm_webs_aabb = ctx.part_element_world_aabb(forearm, elem=forearm_support_webs)
    wrist_service_cover_aabb = ctx.part_element_world_aabb(wrist, elem=wrist_service_cover)
    wrist_bracket_aabb = ctx.part_element_world_aabb(wrist, elem=wrist.get_visual("wrist_bracket"))

    ctx.check(
        "rear_access_cover_mounts_to_back_face",
        rear_cover_aabb is not None
        and rear_cover_aabb[1][1] < -0.058
        and rear_cover_aabb[0][1] < -0.060,
        f"Expected rear access cover to sit on the negative-y face of the base column; aabb={rear_cover_aabb}",
    )
    ctx.check(
        "upper_cable_cover_sits_outboard_of_beam",
        upper_cover_aabb is not None
        and upper_beam_aabb is not None
        and _center_from_aabb(upper_cover_aabb) is not None
        and _center_from_aabb(upper_beam_aabb) is not None
        and upper_cover_aabb[0][1] < upper_beam_aabb[0][1] - 0.008
        and _center_from_aabb(upper_cover_aabb)[1] < (_center_from_aabb(upper_beam_aabb)[1] - 0.015)
        and _overlap(_interval(upper_cover_aabb, 0), _interval(upper_beam_aabb, 0)) is not None
        and _overlap(_interval(upper_cover_aabb, 0), _interval(upper_beam_aabb, 0)) > 0.08,
        f"Expected upper cable cover to read as an outboard removable channel cover; cover={upper_cover_aabb}, beam={upper_beam_aabb}",
    )
    ctx.check(
        "upper_support_webs_bridge_beam_and_clevis",
        upper_webs_aabb is not None
        and upper_beam_aabb is not None
        and elbow_clevis_aabb is not None
        and _overlap(_interval(upper_webs_aabb, 0), _interval(upper_beam_aabb, 0)) is not None
        and _overlap(_interval(upper_webs_aabb, 0), _interval(upper_beam_aabb, 0)) > 0.03
        and _overlap(_interval(upper_webs_aabb, 0), _interval(elbow_clevis_aabb, 0)) is not None
        and _overlap(_interval(upper_webs_aabb, 0), _interval(elbow_clevis_aabb, 0)) > 0.02,
        f"Expected upper support webs to tie the beam into the elbow clevis; webs={upper_webs_aabb}, beam={upper_beam_aabb}, clevis={elbow_clevis_aabb}",
    )
    ctx.check(
        "forearm_cable_cover_sits_outboard_of_beam",
        forearm_cover_aabb is not None
        and forearm_beam_aabb is not None
        and _center_from_aabb(forearm_cover_aabb) is not None
        and _center_from_aabb(forearm_beam_aabb) is not None
        and forearm_cover_aabb[1][1] > forearm_beam_aabb[1][1] + 0.008
        and _center_from_aabb(forearm_cover_aabb)[1] > (_center_from_aabb(forearm_beam_aabb)[1] + 0.010)
        and _overlap(_interval(forearm_cover_aabb, 0), _interval(forearm_beam_aabb, 0)) is not None
        and _overlap(_interval(forearm_cover_aabb, 0), _interval(forearm_beam_aabb, 0)) > 0.07,
        f"Expected forearm cable cover to sit on the opposite beam wall; cover={forearm_cover_aabb}, beam={forearm_beam_aabb}",
    )
    ctx.check(
        "forearm_support_webs_bridge_beam_and_wrist_ring",
        forearm_webs_aabb is not None
        and forearm_beam_aabb is not None
        and _overlap(_interval(forearm_webs_aabb, 0), _interval(forearm_beam_aabb, 0)) is not None
        and _overlap(_interval(forearm_webs_aabb, 0), _interval(forearm_beam_aabb, 0)) > 0.015
        and forearm_webs_aabb[1][0] > 0.260,
        f"Expected forearm support webs to run into the wrist-side structure; webs={forearm_webs_aabb}, beam={forearm_beam_aabb}",
    )
    ctx.check(
        "wrist_service_cover_sits_below_bracket",
        wrist_service_cover_aabb is not None
        and wrist_bracket_aabb is not None
        and _center_from_aabb(wrist_service_cover_aabb) is not None
        and _center_from_aabb(wrist_bracket_aabb) is not None
        and wrist_service_cover_aabb[0][2] < wrist_bracket_aabb[0][2]
        and _center_from_aabb(wrist_service_cover_aabb)[2] < (_center_from_aabb(wrist_bracket_aabb)[2] - 0.035)
        and _overlap(_interval(wrist_service_cover_aabb, 0), _interval(wrist_bracket_aabb, 0)) is not None
        and _overlap(_interval(wrist_service_cover_aabb, 0), _interval(wrist_bracket_aabb, 0)) > 0.03,
        f"Expected wrist service cover to mount beneath the wrist bracket; service_cover={wrist_service_cover_aabb}, bracket={wrist_bracket_aabb}",
    )

    ctx.expect_contact(
        upper_arm,
        base_frame,
        elem_a=shoulder_hub,
        elem_b=shoulder_ring,
        name="shoulder_hub_contacts_ring",
    )
    ctx.expect_origin_distance(
        forearm,
        upper_arm,
        axes="x",
        min_dist=0.31,
        max_dist=0.35,
        name="shoulder_to_elbow_joint_spacing",
    )
    ctx.expect_overlap(
        upper_arm,
        base_frame,
        axes="xy",
        min_overlap=0.10,
        elem_a=shoulder_hub,
        elem_b=shoulder_ring,
        name="shoulder_joint_has_shared_footprint",
    )
    ctx.expect_gap(
        upper_arm,
        base_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem=shoulder_hub,
        negative_elem=shoulder_ring,
        name="shoulder_joint_stack_gap",
    )

    ctx.expect_contact(
        forearm,
        upper_arm,
        contact_tol=0.0015,
        elem_a=elbow_hub,
        elem_b=elbow_clevis,
        name="elbow_hub_contacts_clevis",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        min_overlap=0.04,
        elem_a=elbow_hub,
        elem_b=elbow_clevis,
        name="elbow_joint_has_clear_trunnion_engagement",
    )

    ctx.expect_contact(
        wrist,
        forearm,
        elem_a=wrist_hub,
        elem_b=wrist_ring,
        name="wrist_hub_contacts_ring",
    )
    ctx.expect_origin_distance(
        wrist,
        forearm,
        axes="x",
        min_dist=0.25,
        max_dist=0.29,
        name="elbow_to_wrist_joint_spacing",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="yz",
        min_overlap=0.05,
        elem_a=wrist_hub,
        elem_b=wrist_ring,
        name="wrist_joint_has_ring_engagement",
    )
    rest_wrist_pos = ctx.part_world_position(wrist)
    rest_cover_center = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem=wrist_cover))
    rest_service_cover_center = _center_from_aabb(wrist_service_cover_aabb)
    rest_flange_center = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem=wrist_flange))

    with ctx.pose({shoulder_joint: 1.0}):
        swung_wrist_pos = ctx.part_world_position(wrist)
        ctx.expect_contact(
            upper_arm,
            base_frame,
            elem_a=shoulder_hub,
            elem_b=shoulder_ring,
            name="shoulder_contact_persists_when_rotated",
        )
    ctx.check(
        "shoulder_rotates_arm_in_plan",
        rest_wrist_pos is not None
        and swung_wrist_pos is not None
        and swung_wrist_pos[1] > 0.20
        and swung_wrist_pos[0] < (rest_wrist_pos[0] - 0.10),
        f"Expected shoulder yaw to swing wrist laterally; rest={rest_wrist_pos}, swung={swung_wrist_pos}",
    )

    with ctx.pose({elbow_joint: 1.0}):
        elbowed_wrist_pos = ctx.part_world_position(wrist)
        ctx.expect_contact(
            forearm,
            upper_arm,
            contact_tol=0.0015,
            elem_a=elbow_hub,
            elem_b=elbow_clevis,
            name="elbow_contact_persists_when_flexed",
        )
    ctx.check(
        "elbow_drops_wrist",
        rest_wrist_pos is not None
        and elbowed_wrist_pos is not None
        and elbowed_wrist_pos[2] < (rest_wrist_pos[2] - 0.16),
        f"Expected elbow flexion to lower wrist; rest={rest_wrist_pos}, flexed={elbowed_wrist_pos}",
    )

    with ctx.pose({wrist_joint: math.pi / 2.0}):
        rolled_cover_center = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem=wrist_cover))
        rolled_service_cover_center = _center_from_aabb(
            ctx.part_element_world_aabb(wrist, elem=wrist_service_cover)
        )
        rolled_flange_center = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem=wrist_flange))
        ctx.expect_contact(
            wrist,
            forearm,
            elem_a=wrist_hub,
            elem_b=wrist_ring,
            name="wrist_contact_persists_when_rolled",
        )
    ctx.check(
        "wrist_roll_moves_asymmetric_cover",
        rest_cover_center is not None
        and rolled_cover_center is not None
        and abs(rolled_cover_center[1] - rest_cover_center[1]) > 0.025
        and abs(rolled_cover_center[2] - rest_cover_center[2]) > 0.020,
        f"Expected wrist cover to orbit around roll axis; rest={rest_cover_center}, rolled={rolled_cover_center}",
    )
    ctx.check(
        "wrist_service_cover_orbits_opposite_side",
        rest_service_cover_center is not None
        and rolled_service_cover_center is not None
        and abs(rolled_service_cover_center[1] - rest_service_cover_center[1]) > 0.020
        and abs(rolled_service_cover_center[2] - rest_service_cover_center[2]) > 0.020,
        f"Expected lower service cover to move around the wrist roll axis; rest={rest_service_cover_center}, rolled={rolled_service_cover_center}",
    )
    ctx.check(
        "wrist_flange_stays_on_axis_when_rolled",
        rest_flange_center is not None
        and rolled_flange_center is not None
        and abs(rolled_flange_center[1] - rest_flange_center[1]) < 0.003
        and abs(rolled_flange_center[2] - rest_flange_center[2]) < 0.003,
        f"Expected wrist flange to stay coaxial through roll; rest={rest_flange_center}, rolled={rolled_flange_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
