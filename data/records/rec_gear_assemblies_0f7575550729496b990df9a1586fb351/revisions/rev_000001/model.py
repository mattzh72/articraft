from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _star_points(outer_radius: float, root_radius: float, teeth: int, phase: float = 0.0) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = phase + math.tau * i / (teeth * 2)
        radius = outer_radius if i % 2 == 0 else root_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _spur_gear_x(
    *,
    width: float,
    outer_radius: float,
    root_radius: float,
    hub_radius: float,
    hub_length: float,
    teeth: int,
    x_offset: float,
    phase: float = 0.0,
) -> cq.Workplane:
    tooth_body = (
        cq.Workplane("YZ")
        .polyline(_star_points(outer_radius, root_radius, teeth, phase=phase))
        .close()
        .extrude(width / 2.0, both=True)
    )
    hub = _cylinder_x(hub_radius, hub_length, (0.0, 0.0, 0.0))
    return tooth_body.union(hub).translate((x_offset, 0.0, 0.0))


def _frustum_z(
    *,
    large_radius: float,
    small_radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(large_radius)
        .workplane(offset=length)
        .circle(small_radius)
        .loft(combine=True)
        .translate((center[0], center[1], center[2] - length / 2.0))
    )


def _bevel_gear_z(
    *,
    large_radius: float,
    small_radius: float,
    face_width: float,
    hub_radius: float,
    hub_length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    frustum = _frustum_z(
        large_radius=large_radius,
        small_radius=small_radius,
        length=face_width,
        center=center,
    )
    hub = _cylinder_z(hub_radius, hub_length, center)
    return frustum.union(hub)


def _bevel_gear_x(
    *,
    large_radius: float,
    small_radius: float,
    face_width: float,
    hub_radius: float,
    hub_length: float,
    x_offset: float,
) -> cq.Workplane:
    frustum = (
        _frustum_z(
            large_radius=large_radius,
            small_radius=small_radius,
            length=face_width,
        )
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        .translate((x_offset, 0.0, 0.0))
    )
    hub = _cylinder_x(hub_radius, hub_length, (x_offset, 0.0, 0.0))
    return frustum.union(hub)


def _horizontal_shaft_core(
    *,
    shaft_radius: float,
    shaft_length: float,
    collar_radius: float,
    collar_thickness: float,
    collar_center: float,
    left_stub_length: float = 0.0,
    right_stub_length: float = 0.0,
    stub_radius: float | None = None,
) -> cq.Workplane:
    core = _cylinder_x(shaft_radius, shaft_length, (0.0, 0.0, 0.0))
    core = core.union(_cylinder_x(collar_radius, collar_thickness, (-collar_center, 0.0, 0.0)))
    core = core.union(_cylinder_x(collar_radius, collar_thickness, (collar_center, 0.0, 0.0)))
    if stub_radius is None:
        stub_radius = shaft_radius
    if left_stub_length > 0.0:
        left_start = -shaft_length / 2.0
        core = core.union(
            _cylinder_x(
                stub_radius,
                left_stub_length,
                (left_start - left_stub_length / 2.0, 0.0, 0.0),
            )
        )
    if right_stub_length > 0.0:
        right_start = shaft_length / 2.0
        core = core.union(
            _cylinder_x(
                stub_radius,
                right_stub_length,
                (right_start + right_stub_length / 2.0, 0.0, 0.0),
            )
        )
    return core


def _vertical_shaft_core(
    *,
    shaft_radius: float,
    shaft_length: float,
    shaft_center_z: float,
    collar_radius: float,
    collar_thickness: float,
    lower_collar_center_z: float,
    upper_collar_center_z: float,
    top_drive_radius: float,
    top_drive_length: float,
    top_drive_center_z: float,
) -> cq.Workplane:
    core = _cylinder_z(shaft_radius, shaft_length, (0.0, 0.0, shaft_center_z))
    core = core.union(_cylinder_z(collar_radius, collar_thickness, (0.0, 0.0, lower_collar_center_z)))
    core = core.union(_cylinder_z(collar_radius, collar_thickness, (0.0, 0.0, upper_collar_center_z)))
    core = core.union(_cylinder_z(top_drive_radius, top_drive_length, (0.0, 0.0, top_drive_center_z)))
    return core


def _make_housing() -> cq.Workplane:
    input_x = -0.058
    layshaft_z = 0.122
    output_z = 0.066

    body = _box((0.34, 0.16, 0.018), (0.0, 0.0, 0.009))
    body = body.union(_box((0.30, 0.018, 0.082), (0.0, -0.071, 0.05)))
    body = body.union(_box((0.20, 0.018, 0.042), (-0.042, -0.071, 0.182)))

    for x_pos in (-0.135, 0.135):
        body = body.union(_box((0.065, 0.034, 0.012), (x_pos, 0.0, 0.006)))

    body = body.union(_cylinder_z(0.041, 0.01, (input_x, 0.0, 0.023)))
    body = body.union(_cylinder_z(0.03, 0.046, (input_x, 0.0, 0.051)))
    body = body.union(_box((0.08, 0.024, 0.054), (input_x, -0.046, 0.047)))
    body = body.union(_box((0.022, 0.052, 0.118), (input_x - 0.047, -0.018, 0.151)))
    body = body.union(_box((0.022, 0.052, 0.118), (input_x + 0.047, -0.018, 0.151)))
    body = body.union(_box((0.116, 0.05, 0.014), (input_x, -0.01, 0.213)))
    body = body.union(_cylinder_z(0.026, 0.014, (input_x, 0.0, 0.222)))
    body = body.union(_cylinder_z(0.038, 0.008, (input_x, 0.0, 0.236)))

    for z_pos, web_height in ((output_z, 0.09), (layshaft_z, 0.128)):
        for sign in (-1.0, 1.0):
            x_plate = sign * 0.148
            x_boss = sign * 0.177
            x_flange = sign * 0.189
            body = body.union(_box((0.024, 0.05, web_height), (x_plate, 0.0, web_height / 2.0)))
            body = body.union(_box((0.024, 0.042, 0.032), (x_plate, -0.028, z_pos)))
            body = body.union(_cylinder_x(0.025, 0.03, (x_boss, 0.0, z_pos)))
            body = body.union(_cylinder_x(0.037, 0.008, (x_flange, 0.0, z_pos)))

    body = body.union(_box((0.075, 0.018, 0.034), (-0.12, -0.061, 0.128)))
    body = body.union(_box((0.07, 0.018, 0.03), (0.086, -0.061, 0.104)))

    cutters = _cylinder_z(0.0125, 0.26, (input_x, 0.0, 0.125))
    for z_pos in (layshaft_z, output_z):
        for sign in (-1.0, 1.0):
            cutters = cutters.union(_cylinder_x(0.0125, 0.065, (sign * 0.166, 0.0, z_pos)))
            x_face = sign * 0.189
            for dy, dz in ((0.022, 0.022), (0.022, -0.022), (-0.022, 0.022), (-0.022, -0.022)):
                cutters = cutters.union(_cylinder_x(0.0032, 0.012, (x_face, dy, z_pos + dz)))

    for dx, dy in ((0.022, 0.022), (0.022, -0.022), (-0.022, 0.022), (-0.022, -0.022)):
        cutters = cutters.union(_cylinder_z(0.0032, 0.02, (input_x + dx, dy, 0.236)))

    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.035, 0.035):
            cutters = cutters.union(_cylinder_z(0.005, 0.02, (x_pos, y_pos, 0.006)))

    return body.cut(cutters)


def _make_input_core() -> cq.Workplane:
    core = _cylinder_z(0.010, 0.23, (0.0, 0.0, 0.0))
    core = core.union(_cylinder_z(0.016, 0.006, (0.0, 0.0, -0.108)))
    core = core.union(_cylinder_z(0.016, 0.006, (0.0, 0.0, 0.108)))
    core = core.union(_cylinder_z(0.014, 0.032, (0.0, 0.0, 0.129)))
    return core


def _make_input_bevel() -> cq.Workplane:
    return _bevel_gear_z(
        large_radius=0.022,
        small_radius=0.008,
        face_width=0.022,
        hub_radius=0.014,
        hub_length=0.018,
        center=(0.0, 0.0, 0.0),
    )


def _make_layshaft_core() -> cq.Workplane:
    core = _cylinder_x(0.01, 0.104, (-0.144, 0.0, 0.0))
    core = core.union(_cylinder_x(0.01, 0.25, (0.071, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.017, 0.006, (-0.199, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.017, 0.006, (0.199, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.01, 0.02, (0.206, 0.0, 0.0)))
    return core


def _make_layshaft_bevel() -> cq.Workplane:
    return _bevel_gear_x(
        large_radius=0.022,
        small_radius=0.008,
        face_width=0.022,
        hub_radius=0.015,
        hub_length=0.03,
        x_offset=-0.064,
    )


def _make_layshaft_spur() -> cq.Workplane:
    return _spur_gear_x(
        width=0.02,
        outer_radius=0.028,
        root_radius=0.0245,
        hub_radius=0.018,
        hub_length=0.034,
        teeth=18,
        x_offset=0.078,
        phase=math.radians(5.0),
    )


def _make_output_core() -> cq.Workplane:
    core = _horizontal_shaft_core(
        shaft_radius=0.01,
        shaft_length=0.392,
        collar_radius=0.017,
        collar_thickness=0.006,
        collar_center=0.199,
        left_stub_length=0.0,
        right_stub_length=0.045,
        stub_radius=0.011,
    )
    return core.union(_cylinder_x(0.022, 0.008, (0.249, 0.0, 0.0)))


def _make_output_spur() -> cq.Workplane:
    return _spur_gear_x(
        width=0.018,
        outer_radius=0.025,
        root_radius=0.0215,
        hub_radius=0.016,
        hub_length=0.03,
        teeth=15,
        x_offset=0.078,
        phase=math.radians(17.0),
    )


INPUT_X = -0.09
INPUT_LOWER_Z = 0.06
INPUT_UPPER_Z = 0.186
LAY_LEFT_X = 0.035
OUTPUT_LEFT_X = 0.045
RIGHT_BEARING_X = 0.155
LAY_Z = 0.122
OUTPUT_Z = 0.066
BASE_TOP_Z = 0.018
BRIDGE_CENTER_Z = 0.205


def _frame_housing() -> cq.Workplane:
    body = _box((0.34, 0.16, BASE_TOP_Z), (0.0, 0.0, BASE_TOP_Z / 2.0))
    body = body.union(_box((0.19, 0.018, 0.096), (0.03, -0.071, 0.066)))
    body = body.union(_box((0.11, 0.05, 0.014), (INPUT_X, -0.01, BRIDGE_CENTER_Z)))
    body = body.union(_box((0.018, 0.04, 0.18), (INPUT_X - 0.04, -0.018, 0.108)))
    body = body.union(_box((0.018, 0.04, 0.18), (INPUT_X + 0.04, -0.018, 0.108)))
    body = body.union(_box((0.05, 0.016, 0.024), (INPUT_X - 0.04, -0.046, 0.188)))
    body = body.union(_box((0.05, 0.016, 0.024), (INPUT_X + 0.04, -0.046, 0.188)))
    body = body.union(_box((0.12, 0.014, 0.028), (0.1, -0.05, 0.11)))
    body = body.union(_box((0.07, 0.03, 0.012), (-0.132, 0.0, 0.006)))
    body = body.union(_box((0.07, 0.03, 0.012), (0.14, 0.0, 0.006)))

    body = body.union(_cylinder_z(0.018, 0.018, (INPUT_X, 0.0, INPUT_LOWER_Z)))
    body = body.union(_cylinder_z(0.026, 0.006, (INPUT_X, 0.0, INPUT_LOWER_Z - 0.012)))
    body = body.union(_cylinder_z(0.018, 0.018, (INPUT_X, 0.0, INPUT_UPPER_Z)))
    body = body.union(_cylinder_z(0.026, 0.006, (INPUT_X, 0.0, INPUT_UPPER_Z + 0.012)))

    for x_pos in (LAY_LEFT_X, RIGHT_BEARING_X):
        for z_pos in (LAY_Z, OUTPUT_Z):
            body = body.union(_cylinder_x(0.018, 0.018, (x_pos, 0.0, z_pos)))
            body = body.union(_cylinder_x(0.026, 0.006, (x_pos + 0.012, 0.0, z_pos)))
            body = body.union(_box((0.018, 0.036, z_pos - BASE_TOP_Z), (x_pos, 0.0, (z_pos + BASE_TOP_Z) / 2.0)))

    holes = cq.Workplane("XY")
    for x_pos in (-0.132, 0.14):
        for y_pos in (-0.035, 0.035):
            holes = holes.union(_cylinder_z(0.0045, 0.02, (x_pos, y_pos, 0.006)))
    holes = holes.union(_cylinder_z(0.008, 0.24, (INPUT_X, 0.0, 0.12)))
    holes = holes.union(_cylinder_x(0.008, 0.40, (0.02, 0.0, LAY_Z)))
    holes = holes.union(_cylinder_x(0.008, 0.40, (0.02, 0.0, OUTPUT_Z)))
    body = body.cut(holes)
    body = body.union(_input_lower_cartridge().translate((INPUT_X, 0.0, INPUT_LOWER_Z)))
    body = body.union(_input_upper_cartridge().translate((INPUT_X, 0.0, INPUT_UPPER_Z)))
    body = body.union(_horizontal_cartridge(LAY_Z, -1.0).translate((LAY_LEFT_X, 0.0, LAY_Z)))
    body = body.union(_horizontal_cartridge(LAY_Z, 1.0).translate((RIGHT_BEARING_X, 0.0, LAY_Z)))
    body = body.union(_horizontal_cartridge(OUTPUT_Z, -1.0).translate((OUTPUT_LEFT_X, 0.0, OUTPUT_Z)))
    body = body.union(_horizontal_cartridge(OUTPUT_Z, 1.0).translate((RIGHT_BEARING_X, 0.0, OUTPUT_Z)))
    return body


def _horizontal_cartridge(axis_z: float, outer_sign: float) -> cq.Workplane:
    base_to_axis = axis_z - BASE_TOP_Z
    pedestal_h = base_to_axis - 0.012
    body = _cylinder_x(0.018, 0.018, (0.0, 0.0, 0.0))
    body = body.union(_cylinder_x(0.026, 0.006, (outer_sign * 0.009, 0.0, 0.0)))
    body = body.union(_box((0.022, 0.032, pedestal_h), (0.0, 0.0, -0.012 - pedestal_h / 2.0)))
    body = body.union(_box((0.038, 0.026, 0.008), (0.0, 0.0, -base_to_axis + 0.004)))

    cutters = _cylinder_x(0.0095, 0.05, (0.0, 0.0, 0.0))
    for y_pos in (-0.009, 0.009):
        cutters = cutters.union(_cylinder_z(0.0028, 0.012, (0.0, y_pos, -base_to_axis + 0.004)))
    return body.cut(cutters)


def _input_lower_cartridge() -> cq.Workplane:
    base_to_axis = INPUT_LOWER_Z - BASE_TOP_Z
    pedestal_h = base_to_axis - 0.012
    body = _cylinder_z(0.018, 0.018, (0.0, 0.0, 0.0))
    body = body.union(_cylinder_z(0.026, 0.006, (0.0, 0.0, -0.009)))
    body = body.union(_box((0.034, 0.034, pedestal_h), (0.0, 0.0, -0.012 - pedestal_h / 2.0)))
    body = body.union(_box((0.048, 0.026, 0.008), (0.0, 0.0, -base_to_axis + 0.004)))

    cutters = _cylinder_z(0.0095, 0.05, (0.0, 0.0, 0.0))
    for x_pos in (-0.012, 0.012):
        cutters = cutters.union(_cylinder_z(0.0028, 0.012, (x_pos, 0.0, -base_to_axis + 0.004)))
    return body.cut(cutters)


def _input_upper_cartridge() -> cq.Workplane:
    body = _cylinder_z(0.018, 0.018, (0.0, 0.0, 0.0))
    body = body.union(_cylinder_z(0.026, 0.006, (0.0, 0.0, 0.009)))
    return body.cut(_cylinder_z(0.0095, 0.05, (0.0, 0.0, 0.0)))


def _input_core_rebuilt() -> cq.Workplane:
    core = _cylinder_z(0.008, 0.08, (0.0, 0.0, 0.015))
    core = core.union(_cylinder_z(0.008, 0.086, (0.0, 0.0, 0.112)))
    core = core.union(_cylinder_z(0.013, 0.004, (0.0, 0.0, 0.0)))
    core = core.union(_cylinder_z(0.013, 0.004, (0.0, 0.0, 0.126)))
    core = core.union(_cylinder_z(0.0115, 0.028, (0.0, 0.0, 0.14)))
    return core


def _input_bevel_rebuilt() -> cq.Workplane:
    return _bevel_gear_z(
        large_radius=0.02,
        small_radius=0.0075,
        face_width=0.022,
        hub_radius=0.012,
        hub_length=0.014,
        center=(0.0, 0.0, 0.062),
    )


def _layshaft_core_rebuilt() -> cq.Workplane:
    core = _cylinder_x(0.008, 0.27, (0.03, 0.0, 0.0))
    core = core.union(_cylinder_x(0.013, 0.004, (0.011, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.013, 0.004, (0.119, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.0105, 0.028, (0.145, 0.0, 0.0)))
    return core


def _layshaft_bevel_rebuilt() -> cq.Workplane:
    return _bevel_gear_x(
        large_radius=0.021,
        small_radius=0.0075,
        face_width=0.02,
        hub_radius=0.012,
        hub_length=0.008,
        x_offset=-0.09,
    )


def _layshaft_spur_rebuilt() -> cq.Workplane:
    return _spur_gear_x(
        width=0.018,
        outer_radius=0.027,
        root_radius=0.0235,
        hub_radius=0.015,
        hub_length=0.024,
        teeth=18,
        x_offset=0.065,
        phase=math.radians(4.0),
    )


def _output_core_rebuilt() -> cq.Workplane:
    core = _cylinder_x(0.008, 0.19, (0.05, 0.0, 0.0))
    core = core.union(_cylinder_x(0.013, 0.004, (0.011, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.013, 0.004, (0.109, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.0105, 0.034, (0.132, 0.0, 0.0)))
    core = core.union(_cylinder_x(0.017, 0.006, (0.154, 0.0, 0.0)))
    return core


def _output_spur_rebuilt() -> cq.Workplane:
    return _spur_gear_x(
        width=0.016,
        outer_radius=0.0247,
        root_radius=0.0212,
        hub_radius=0.014,
        hub_length=0.022,
        teeth=15,
        x_offset=0.055,
        phase=math.radians(16.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_transfer_gearbox")

    housing_mat = model.material("housing_iron", rgba=(0.27, 0.29, 0.31, 1.0))
    shaft_mat = model.material("shaft_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    gear_mat = model.material("gear_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_frame_housing(), "housing_shell"),
        material=housing_mat,
        name="housing_shell",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(_input_core_rebuilt(), "input_core_mesh"),
        material=shaft_mat,
        name="input_core",
    )
    input_shaft.visual(
        mesh_from_cadquery(_input_bevel_rebuilt(), "input_bevel_mesh"),
        material=gear_mat,
        name="input_bevel",
    )

    layshaft = model.part("layshaft")
    layshaft.visual(
        mesh_from_cadquery(_layshaft_core_rebuilt(), "layshaft_core_mesh"),
        material=shaft_mat,
        name="layshaft_core",
    )
    layshaft.visual(
        mesh_from_cadquery(_layshaft_bevel_rebuilt(), "layshaft_bevel_mesh"),
        material=gear_mat,
        name="layshaft_bevel",
    )
    layshaft.visual(
        mesh_from_cadquery(_layshaft_spur_rebuilt(), "layshaft_spur_mesh"),
        material=gear_mat,
        name="layshaft_spur",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        mesh_from_cadquery(_output_core_rebuilt(), "output_core_mesh"),
        material=shaft_mat,
        name="output_core",
    )
    output_shaft.visual(
        mesh_from_cadquery(_output_spur_rebuilt(), "output_spur_mesh"),
        material=gear_mat,
        name="output_spur",
    )

    model.articulation(
        "input_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_X, 0.0, INPUT_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )
    model.articulation(
        "layshaft_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=layshaft,
        origin=Origin(xyz=(LAY_LEFT_X, 0.0, LAY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=30.0),
    )
    model.articulation(
        "output_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_LEFT_X, 0.0, OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    layshaft = object_model.get_part("layshaft")
    output_shaft = object_model.get_part("output_shaft")
    input_rotation = object_model.get_articulation("input_rotation")
    layshaft_rotation = object_model.get_articulation("layshaft_rotation")
    output_rotation = object_model.get_articulation("output_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        housing,
        input_shaft,
        reason="input shaft runs inside integrated housing bearing bores and cartridge shells",
    )
    ctx.allow_overlap(
        housing,
        layshaft,
        reason="layshaft runs through integrated bearing cartridges that are modeled as part of the housing casting",
    )
    ctx.allow_overlap(
        housing,
        output_shaft,
        reason="output shaft runs through integrated bearing cartridges that are modeled as part of the housing casting",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "input shaft rotates about vertical axis",
        tuple(input_rotation.axis) == (0.0, 0.0, 1.0),
        f"expected vertical axis, got {input_rotation.axis}",
    )
    ctx.check(
        "layshaft rotates about horizontal axis",
        tuple(layshaft_rotation.axis) == (1.0, 0.0, 0.0),
        f"expected x axis, got {layshaft_rotation.axis}",
    )
    ctx.check(
        "output shaft rotates about horizontal axis",
        tuple(output_rotation.axis) == (1.0, 0.0, 0.0),
        f"expected x axis, got {output_rotation.axis}",
    )

    ctx.expect_contact(input_shaft, housing, name="input shaft is supported by integrated housing cartridges")
    ctx.expect_contact(layshaft, housing, name="layshaft is supported by integrated housing cartridges")
    ctx.expect_contact(output_shaft, housing, name="output shaft is supported by integrated housing cartridges")

    ctx.expect_overlap(
        input_shaft,
        layshaft,
        axes="y",
        elem_a="input_bevel",
        elem_b="layshaft_bevel",
        min_overlap=0.014,
        name="bevel gears share a common mesh band",
    )

    ctx.expect_origin_distance(
        layshaft,
        output_shaft,
        axes="z",
        min_dist=0.054,
        max_dist=0.058,
        name="spur shaft center distance is believable",
    )
    ctx.expect_overlap(
        layshaft,
        output_shaft,
        axes="xy",
        elem_a="layshaft_spur",
        elem_b="output_spur",
        min_overlap=0.012,
        name="spur gears line up along the same mesh face width",
    )
    ctx.expect_gap(
        layshaft,
        output_shaft,
        axis="z",
        positive_elem="layshaft_spur",
        negative_elem="output_spur",
        min_gap=0.002,
        max_gap=0.005,
        name="spur gears keep a small running clearance",
    )

    input_bevel_aabb = ctx.part_element_world_aabb(input_shaft, elem="input_bevel")
    layshaft_bevel_aabb = ctx.part_element_world_aabb(layshaft, elem="layshaft_bevel")
    if input_bevel_aabb is not None and layshaft_bevel_aabb is not None:
        input_top_z = input_bevel_aabb[1][2]
        layshaft_left_x = layshaft_bevel_aabb[0][0]
        ctx.check(
            "input bevel rises toward the layshaft axis",
            0.129 <= input_top_z <= 0.136,
            f"input bevel top z={input_top_z}",
        )
        ctx.check(
            "layshaft bevel overhang reaches toward the input axis",
            -0.068 <= layshaft_left_x <= -0.062,
            f"layshaft bevel left x={layshaft_left_x}",
        )
    else:
        ctx.fail("bevel gear visuals are queryable", "missing bevel gear visual AABB")

    with ctx.pose(
        {
            input_rotation: 0.6,
            layshaft_rotation: -0.75,
            output_rotation: 0.9,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotating shafts stay clear in a representative service pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
