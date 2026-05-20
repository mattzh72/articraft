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

STEEL = (0.39, 0.41, 0.44, 1.0)
DARK_STEEL = (0.23, 0.25, 0.28, 1.0)
HARDWARE = (0.60, 0.61, 0.63, 1.0)

LOWER_AXIS = (0.160, 0.000, 0.180)
MIDDLE_AXIS = (-0.140, -0.070, 0.390)
UPPER_AXIS = (0.000, 0.125, 0.620)


def _cylinder(axis: str, radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    solid = cq.Workplane("XY").cylinder(length, radius)
    if axis == "x":
        solid = solid.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    elif axis == "y":
        solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    elif axis != "z":
        raise ValueError(f"unsupported axis {axis!r}")
    return solid.translate(center)


def _bar_xz(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    thickness_x: float,
    width_y: float,
) -> cq.Workplane:
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.hypot(dx, dz)
    angle = math.degrees(math.atan2(dx, dz))
    mid = ((p0[0] + p1[0]) * 0.5, p0[1], (p0[2] + p1[2]) * 0.5)
    return (
        cq.Workplane("XY")
        .box(thickness_x, width_y, length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        .translate(mid)
    )


def _gusset_xz(
    points: list[tuple[float, float]],
    *,
    y_center: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, y_center - thickness * 0.5, 0.0))
    )


def _gusset_yz(
    points: list[tuple[float, float]],
    *,
    x_center: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((x_center - thickness * 0.5, 0.0, 0.0))
    )


def _bolt_heads(
    axis: str,
    centers: list[tuple[float, float, float]],
    *,
    radius: float = 0.005,
    length: float = 0.004,
) -> cq.Workplane:
    heads = None
    for center in centers:
        head = _cylinder(axis, radius, length, center)
        heads = head if heads is None else heads.union(head)
    return heads if heads is not None else cq.Workplane("XY")


def _make_spine_truss() -> cq.Workplane:
    shape = cq.Workplane("XY").box(0.340, 0.240, 0.020).translate((0.0, 0.0, 0.010))
    shape = shape.union(cq.Workplane("XY").box(0.140, 0.120, 0.050).translate((0.0, 0.0, 0.045)))
    shape = shape.union(cq.Workplane("XY").box(0.034, 0.040, 0.640).translate((-0.048, -0.010, 0.350)))
    shape = shape.union(cq.Workplane("XY").box(0.034, 0.040, 0.640).translate((0.048, -0.010, 0.350)))
    shape = shape.union(cq.Workplane("XY").box(0.120, 0.016, 0.620).translate((0.0, -0.062, 0.340)))
    shape = shape.union(cq.Workplane("XY").box(0.120, 0.026, 0.020).translate((0.0, 0.015, 0.280)))
    shape = shape.union(cq.Workplane("XY").box(0.100, 0.026, 0.020).translate((0.0, 0.015, 0.520)))
    shape = shape.union(_bar_xz((-0.115, 0.052, 0.060), (0.000, 0.052, 0.305), 0.016, 0.020))
    shape = shape.union(_bar_xz((0.115, 0.052, 0.060), (0.000, 0.052, 0.305), 0.016, 0.020))
    shape = shape.union(_bar_xz((-0.115, -0.052, 0.060), (0.000, -0.052, 0.305), 0.016, 0.020))
    shape = shape.union(_bar_xz((0.115, -0.052, 0.060), (0.000, -0.052, 0.305), 0.016, 0.020))
    shape = shape.union(_bar_xz((-0.085, 0.050, 0.305), (0.000, 0.050, 0.585), 0.014, 0.018))
    shape = shape.union(_bar_xz((0.085, 0.050, 0.305), (0.000, 0.050, 0.585), 0.014, 0.018))
    shape = shape.union(_bar_xz((-0.085, -0.050, 0.305), (0.000, -0.050, 0.585), 0.014, 0.018))
    shape = shape.union(_bar_xz((0.085, -0.050, 0.305), (0.000, -0.050, 0.585), 0.014, 0.018))
    shape = shape.union(
        _gusset_xz([(0.000, 0.070), (0.090, 0.070), (0.000, 0.200)], y_center=0.060, thickness=0.010)
    )
    shape = shape.union(
        _gusset_xz([(0.000, 0.070), (-0.090, 0.070), (0.000, 0.200)], y_center=0.060, thickness=0.010)
    )
    shape = shape.union(
        _gusset_xz([(0.000, 0.070), (0.090, 0.070), (0.000, 0.200)], y_center=-0.070, thickness=0.010)
    )
    shape = shape.union(
        _gusset_xz([(0.000, 0.070), (-0.090, 0.070), (0.000, 0.200)], y_center=-0.070, thickness=0.010)
    )
    return shape


def _make_lower_housing() -> cq.Workplane:
    x0, y0, z0 = LOWER_AXIS
    shape = cq.Workplane("XY").box(0.118, 0.068, 0.024).translate((0.098, 0.000, z0 - 0.008))
    shape = shape.union(cq.Workplane("XY").box(0.080, 0.082, 0.030).translate((x0, y0, z0 - 0.026)))
    shape = shape.union(cq.Workplane("XY").box(0.080, 0.082, 0.030).translate((x0, y0, z0 + 0.026)))
    shape = shape.union(cq.Workplane("XY").box(0.046, 0.012, 0.082).translate((x0 + 0.015, y0 + 0.029, z0)))
    shape = shape.union(cq.Workplane("XY").box(0.046, 0.012, 0.082).translate((x0 + 0.015, y0 - 0.029, z0)))
    shape = shape.union(
        _gusset_yz([(-0.034, z0 - 0.050), (0.000, z0 - 0.050), (0.000, z0 + 0.030)], x_center=0.112, thickness=0.010)
    )
    shape = shape.union(
        _gusset_yz([(0.034, z0 - 0.050), (0.000, z0 - 0.050), (0.000, z0 + 0.030)], x_center=0.112, thickness=0.010)
    )
    shape = shape.cut(_cylinder("z", 0.0142, 0.120, (x0, y0, z0)))
    shape = shape.cut(cq.Workplane("XY").box(0.024, 0.048, 0.054).translate((0.199, 0.000, z0)))
    flange = _cylinder("z", 0.026, 0.010, (x0, y0, z0 - 0.026)).cut(_cylinder("z", 0.0165, 0.012, (x0, y0, z0 - 0.026)))
    flange = flange.union(
        _cylinder("z", 0.026, 0.010, (x0, y0, z0 + 0.026)).cut(_cylinder("z", 0.0165, 0.012, (x0, y0, z0 + 0.026)))
    )
    return shape.union(flange)


def _make_middle_housing() -> cq.Workplane:
    x0, y0, z0 = MIDDLE_AXIS
    shape = cq.Workplane("XY").box(0.110, 0.050, 0.024).translate((-0.084, -0.050, z0))
    shape = shape.union(cq.Workplane("XY").box(0.082, 0.030, 0.082).translate((x0, y0 - 0.025, z0)))
    shape = shape.union(cq.Workplane("XY").box(0.082, 0.030, 0.082).translate((x0, y0 + 0.025, z0)))
    shape = shape.union(cq.Workplane("XY").box(0.048, 0.072, 0.012).translate((x0 + 0.015, y0, z0 + 0.029)))
    shape = shape.union(cq.Workplane("XY").box(0.048, 0.072, 0.012).translate((x0 + 0.015, y0, z0 - 0.029)))
    shape = shape.union(
        _gusset_xz([(-0.060, z0 - 0.040), (0.000, z0 - 0.040), (0.000, z0 + 0.030)], y_center=y0 + 0.030, thickness=0.010)
    )
    shape = shape.union(
        _gusset_xz([(-0.060, z0 - 0.040), (0.000, z0 - 0.040), (0.000, z0 + 0.030)], y_center=y0 - 0.040, thickness=0.010)
    )
    shape = shape.cut(_cylinder("y", 0.0138, 0.124, (x0, y0, z0)))
    shape = shape.cut(cq.Workplane("XY").box(0.054, 0.018, 0.054).translate((x0, y0 - 0.056, z0)))
    flange = _cylinder("y", 0.025, 0.010, (x0, y0 - 0.025, z0)).cut(_cylinder("y", 0.0160, 0.012, (x0, y0 - 0.025, z0)))
    flange = flange.union(
        _cylinder("y", 0.025, 0.010, (x0, y0 + 0.025, z0)).cut(_cylinder("y", 0.0160, 0.012, (x0, y0 + 0.025, z0)))
    )
    return shape.union(flange)


def _make_upper_housing() -> cq.Workplane:
    x0, y0, z0 = UPPER_AXIS
    shape = cq.Workplane("XY").box(0.060, 0.112, 0.024).translate((0.000, 0.070, z0))
    shape = shape.union(cq.Workplane("XY").box(0.030, 0.078, 0.082).translate((x0 - 0.035, y0, z0)))
    shape = shape.union(cq.Workplane("XY").box(0.030, 0.078, 0.082).translate((x0 + 0.035, y0, z0)))
    shape = shape.union(cq.Workplane("XY").box(0.094, 0.046, 0.012).translate((x0, y0 + 0.010, z0 + 0.029)))
    shape = shape.union(cq.Workplane("XY").box(0.094, 0.046, 0.012).translate((x0, y0 + 0.010, z0 - 0.029)))
    shape = shape.union(
        _gusset_yz([(y0 - 0.030, z0 - 0.038), (y0 + 0.024, z0 - 0.038), (y0 + 0.024, z0 + 0.028)], x_center=-0.040, thickness=0.010)
    )
    shape = shape.union(
        _gusset_yz([(y0 - 0.030, z0 - 0.038), (y0 + 0.024, z0 - 0.038), (y0 + 0.024, z0 + 0.028)], x_center=0.030, thickness=0.010)
    )
    shape = shape.cut(_cylinder("x", 0.0136, 0.128, (x0, y0, z0)))
    shape = shape.cut(cq.Workplane("XY").box(0.056, 0.018, 0.056).translate((0.000, 0.157, z0)))
    flange = _cylinder("x", 0.025, 0.010, (x0 - 0.035, y0, z0)).cut(_cylinder("x", 0.0158, 0.012, (x0 - 0.035, y0, z0)))
    flange = flange.union(
        _cylinder("x", 0.025, 0.010, (x0 + 0.035, y0, z0)).cut(_cylinder("x", 0.0158, 0.012, (x0 + 0.035, y0, z0)))
    )
    return shape.union(flange)


def _make_lower_stop_tab() -> cq.Workplane:
    x0, y0, z0 = LOWER_AXIS
    return cq.Workplane("XY").box(0.018, 0.020, 0.036).translate((x0 + 0.024, y0 + 0.075, z0))


def _make_middle_stop_tab() -> cq.Workplane:
    x0, y0, z0 = MIDDLE_AXIS
    return cq.Workplane("XY").box(0.028, 0.028, 0.014).translate((x0 + 0.033, y0, z0 - 0.065))


def _make_upper_stop_tab() -> cq.Workplane:
    x0, y0, z0 = UPPER_AXIS
    return cq.Workplane("XY").box(0.030, 0.024, 0.014).translate((x0, y0 + 0.033, z0 + 0.066))


def _make_lower_branch_core() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.170, 0.046, 0.030).translate((0.150, 0.000, 0.000))
    beam = beam.cut(cq.Workplane("XY").box(0.132, 0.024, 0.018).translate((0.158, 0.000, 0.000)))
    saddle = cq.Workplane("XY").box(0.070, 0.060, 0.026).translate((0.050, 0.000, 0.000))
    outer_collar = _cylinder("z", 0.028, 0.012, (0.000, 0.000, 0.050))
    outer_collar = outer_collar.union(_cylinder("z", 0.028, 0.012, (0.000, 0.000, -0.050)))
    web_left = (
        cq.Workplane("XZ")
        .polyline([(0.010, -0.015), (0.130, -0.015), (0.055, 0.050)])
        .close()
        .extrude(0.008)
        .translate((0.0, 0.016, 0.0))
    )
    web_right = (
        cq.Workplane("XZ")
        .polyline([(0.010, -0.015), (0.130, -0.015), (0.055, 0.050)])
        .close()
        .extrude(0.008)
        .translate((0.0, -0.024, 0.0))
    )
    return beam.union(saddle).union(outer_collar).union(web_left).union(web_right)


def _make_lower_tip_pad() -> cq.Workplane:
    pad = cq.Workplane("XY").box(0.050, 0.056, 0.018).translate((0.272, 0.000, 0.000))
    pad = pad.cut(cq.Workplane("XY").box(0.028, 0.022, 0.028).translate((0.272, 0.000, 0.000)))
    return pad


def _make_lower_stop_lug() -> cq.Workplane:
    return cq.Workplane("XY").box(0.018, 0.020, 0.030).translate((0.055, 0.010, 0.000))


def _make_middle_branch_core() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.172, 0.036, 0.030).translate((0.148, 0.000, 0.018))
    beam = beam.cut(cq.Workplane("XY").box(0.134, 0.018, 0.016).translate((0.156, 0.000, 0.018)))
    saddle = cq.Workplane("XY").box(0.072, 0.052, 0.032).translate((0.045, 0.000, 0.000))
    collar = _cylinder("y", 0.026, 0.012, (0.000, -0.050, 0.000))
    collar = collar.union(_cylinder("y", 0.026, 0.012, (0.000, 0.050, 0.000)))
    rib_left = (
        cq.Workplane("XZ")
        .polyline([(0.010, -0.012), (0.130, 0.005), (0.055, 0.058)])
        .close()
        .extrude(0.008)
        .translate((0.0, 0.016, 0.0))
    )
    rib_right = (
        cq.Workplane("XZ")
        .polyline([(0.010, -0.012), (0.130, 0.005), (0.055, 0.058)])
        .close()
        .extrude(0.008)
        .translate((0.0, -0.024, 0.0))
    )
    return beam.union(saddle).union(collar).union(rib_left).union(rib_right)


def _make_middle_tip_pad() -> cq.Workplane:
    pad = cq.Workplane("XY").box(0.050, 0.050, 0.018).translate((0.255, 0.000, 0.032))
    pad = pad.cut(cq.Workplane("XY").box(0.024, 0.018, 0.026).translate((0.255, 0.000, 0.032)))
    return pad


def _make_middle_stop_lug() -> cq.Workplane:
    return cq.Workplane("XY").box(0.022, 0.030, 0.018).translate((0.055, 0.000, -0.010))


def _make_upper_branch_core() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.034, 0.176, 0.032).translate((0.000, 0.148, 0.020))
    beam = beam.cut(cq.Workplane("XY").box(0.018, 0.138, 0.018).translate((0.000, 0.156, 0.020)))
    saddle = cq.Workplane("XY").box(0.060, 0.072, 0.032).translate((0.000, 0.050, 0.000))
    collar = _cylinder("x", 0.026, 0.012, (-0.050, 0.000, 0.000))
    collar = collar.union(_cylinder("x", 0.026, 0.012, (0.050, 0.000, 0.000)))
    rib_left = (
        cq.Workplane("YZ")
        .polyline([(0.015, -0.012), (0.138, 0.005), (0.060, 0.058)])
        .close()
        .extrude(0.008)
        .translate((0.016, 0.0, 0.0))
    )
    rib_right = (
        cq.Workplane("YZ")
        .polyline([(0.015, -0.012), (0.138, 0.005), (0.060, 0.058)])
        .close()
        .extrude(0.008)
        .translate((-0.024, 0.0, 0.0))
    )
    return beam.union(saddle).union(collar).union(rib_left).union(rib_right)


def _make_upper_tip_pad() -> cq.Workplane:
    pad = cq.Workplane("XY").box(0.052, 0.050, 0.018).translate((0.000, 0.248, 0.042))
    pad = pad.cut(cq.Workplane("XY").box(0.024, 0.020, 0.026).translate((0.000, 0.248, 0.042)))
    return pad


def _make_upper_stop_lug() -> cq.Workplane:
    return cq.Workplane("XY").box(0.030, 0.018, 0.022).translate((0.000, 0.055, 0.010))


def _make_cover(axis: str) -> cq.Workplane:
    if axis == "lower":
        plate = cq.Workplane("XY").box(0.006, 0.060, 0.070)
        boss = cq.Workplane("XY").box(0.004, 0.030, 0.032).translate((0.005, 0.0, 0.0))
        bolts = _bolt_heads(
            "x",
            [
                (0.005, 0.022, 0.024),
                (0.005, -0.022, 0.024),
                (0.005, 0.022, -0.024),
                (0.005, -0.022, -0.024),
            ],
        )
        return plate.union(boss).union(bolts)
    if axis == "middle":
        plate = cq.Workplane("XY").box(0.058, 0.006, 0.060)
        boss = cq.Workplane("XY").box(0.032, 0.004, 0.028).translate((0.0, -0.005, 0.0))
        bolts = _bolt_heads(
            "y",
            [
                (0.022, -0.005, 0.022),
                (-0.022, -0.005, 0.022),
                (0.022, -0.005, -0.022),
                (-0.022, -0.005, -0.022),
            ],
        )
        return plate.union(boss).union(bolts)
    if axis == "upper":
        plate = cq.Workplane("XY").box(0.060, 0.006, 0.062)
        boss = cq.Workplane("XY").box(0.034, 0.004, 0.030).translate((0.0, 0.005, 0.0))
        bolts = _bolt_heads(
            "y",
            [
                (0.022, 0.005, 0.022),
                (-0.022, 0.005, 0.022),
                (0.022, 0.005, -0.022),
                (-0.022, 0.005, -0.022),
            ],
        )
        return plate.union(boss).union(bolts)
    raise ValueError(f"unsupported cover axis {axis!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="branching_tree_mechanical_study", assets=ASSETS)
    steel = model.material("steel", rgba=STEEL)
    dark_steel = model.material("dark_steel", rgba=DARK_STEEL)
    hardware = model.material("hardware", rgba=HARDWARE)

    spine = model.part("spine_frame")
    spine.visual(
        mesh_from_cadquery(_make_spine_truss(), "spine_truss.obj", assets=ASSETS),
        material=steel,
        name="spine_truss",
    )
    spine.visual(
        mesh_from_cadquery(_make_lower_housing(), "lower_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="lower_housing",
    )
    spine.visual(
        mesh_from_cadquery(_make_middle_housing(), "middle_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="middle_housing",
    )
    spine.visual(
        mesh_from_cadquery(_make_upper_housing(), "upper_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="upper_housing",
    )
    spine.visual(
        mesh_from_cadquery(_make_lower_stop_tab(), "lower_stop_tab.obj", assets=ASSETS),
        material=hardware,
        name="lower_stop_tab",
    )
    spine.visual(
        mesh_from_cadquery(_make_middle_stop_tab(), "middle_stop_tab.obj", assets=ASSETS),
        material=hardware,
        name="middle_stop_tab",
    )
    spine.visual(
        mesh_from_cadquery(_make_upper_stop_tab(), "upper_stop_tab.obj", assets=ASSETS),
        material=hardware,
        name="upper_stop_tab",
    )
    spine.inertial = Inertial.from_geometry(
        Box((0.340, 0.240, 0.760)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
    )

    lower_cover = model.part("lower_access_cover")
    lower_cover.visual(
        mesh_from_cadquery(_make_cover("lower"), "lower_access_cover.obj", assets=ASSETS),
        material=hardware,
        name="lower_cover_plate",
    )
    lower_cover.inertial = Inertial.from_geometry(Box((0.020, 0.060, 0.070)), mass=0.35)

    middle_cover = model.part("middle_access_cover")
    middle_cover.visual(
        mesh_from_cadquery(_make_cover("middle"), "middle_access_cover.obj", assets=ASSETS),
        material=hardware,
        name="middle_cover_plate",
    )
    middle_cover.inertial = Inertial.from_geometry(Box((0.058, 0.020, 0.060)), mass=0.30)

    upper_cover = model.part("upper_access_cover")
    upper_cover.visual(
        mesh_from_cadquery(_make_cover("upper"), "upper_access_cover.obj", assets=ASSETS),
        material=hardware,
        name="upper_cover_plate",
    )
    upper_cover.inertial = Inertial.from_geometry(Box((0.060, 0.020, 0.062)), mass=0.30)

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(_cylinder("z", 0.014, 0.110, (0.0, 0.0, 0.0)), "lower_journal.obj", assets=ASSETS),
        material=hardware,
        name="lower_journal",
    )
    lower_branch.visual(
        mesh_from_cadquery(_make_lower_branch_core(), "lower_branch_core.obj", assets=ASSETS),
        material=steel,
        name="lower_branch_core",
    )
    lower_branch.visual(
        mesh_from_cadquery(_make_lower_tip_pad(), "lower_tip_pad.obj", assets=ASSETS),
        material=dark_steel,
        name="lower_tip_pad",
    )
    lower_branch.visual(
        mesh_from_cadquery(_make_lower_stop_lug(), "lower_stop_lug.obj", assets=ASSETS),
        material=hardware,
        name="lower_stop_lug",
    )
    lower_branch.inertial = Inertial.from_geometry(
        Box((0.320, 0.080, 0.120)),
        mass=3.0,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
    )

    middle_branch = model.part("middle_branch")
    middle_branch.visual(
        mesh_from_cadquery(_cylinder("y", 0.0136, 0.110, (0.0, 0.0, 0.0)), "middle_journal.obj", assets=ASSETS),
        material=hardware,
        name="middle_journal",
    )
    middle_branch.visual(
        mesh_from_cadquery(_make_middle_branch_core(), "middle_branch_core.obj", assets=ASSETS),
        material=steel,
        name="middle_branch_core",
    )
    middle_branch.visual(
        mesh_from_cadquery(_make_middle_tip_pad(), "middle_tip_pad.obj", assets=ASSETS),
        material=dark_steel,
        name="middle_tip_pad",
    )
    middle_branch.visual(
        mesh_from_cadquery(_make_middle_stop_lug(), "middle_stop_lug.obj", assets=ASSETS),
        material=hardware,
        name="middle_stop_lug",
    )
    middle_branch.inertial = Inertial.from_geometry(
        Box((0.300, 0.090, 0.120)),
        mass=2.8,
        origin=Origin(xyz=(0.150, 0.0, 0.015)),
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(_cylinder("x", 0.0134, 0.110, (0.0, 0.0, 0.0)), "upper_journal.obj", assets=ASSETS),
        material=hardware,
        name="upper_journal",
    )
    upper_branch.visual(
        mesh_from_cadquery(_make_upper_branch_core(), "upper_branch_core.obj", assets=ASSETS),
        material=steel,
        name="upper_branch_core",
    )
    upper_branch.visual(
        mesh_from_cadquery(_make_upper_tip_pad(), "upper_tip_pad.obj", assets=ASSETS),
        material=dark_steel,
        name="upper_tip_pad",
    )
    upper_branch.visual(
        mesh_from_cadquery(_make_upper_stop_lug(), "upper_stop_lug.obj", assets=ASSETS),
        material=hardware,
        name="upper_stop_lug",
    )
    upper_branch.inertial = Inertial.from_geometry(
        Box((0.120, 0.300, 0.140)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.145, 0.020)),
    )

    model.articulation(
        "spine_to_lower_cover",
        ArticulationType.FIXED,
        parent=spine,
        child=lower_cover,
        origin=Origin(xyz=(0.200, 0.000, LOWER_AXIS[2])),
    )
    model.articulation(
        "spine_to_middle_cover",
        ArticulationType.FIXED,
        parent=spine,
        child=middle_cover,
        origin=Origin(xyz=(MIDDLE_AXIS[0], -0.117, MIDDLE_AXIS[2])),
    )
    model.articulation(
        "spine_to_upper_cover",
        ArticulationType.FIXED,
        parent=spine,
        child=upper_cover,
        origin=Origin(xyz=(UPPER_AXIS[0], 0.160, UPPER_AXIS[2])),
    )

    model.articulation(
        "lower_rotary_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=LOWER_AXIS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=-1.05, upper=1.10),
    )
    model.articulation(
        "middle_rotary_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=middle_branch,
        origin=Origin(xyz=MIDDLE_AXIS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.90, upper=0.82),
    )
    model.articulation(
        "upper_rotary_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=UPPER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.80, upper=0.86),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    spine = object_model.get_part("spine_frame")
    lower_cover = object_model.get_part("lower_access_cover")
    middle_cover = object_model.get_part("middle_access_cover")
    upper_cover = object_model.get_part("upper_access_cover")
    lower_branch = object_model.get_part("lower_branch")
    middle_branch = object_model.get_part("middle_branch")
    upper_branch = object_model.get_part("upper_branch")

    lower_joint = object_model.get_articulation("lower_rotary_branch")
    middle_joint = object_model.get_articulation("middle_rotary_branch")
    upper_joint = object_model.get_articulation("upper_rotary_branch")

    lower_housing = spine.get_visual("lower_housing")
    middle_housing = spine.get_visual("middle_housing")
    upper_housing = spine.get_visual("upper_housing")
    lower_stop_tab = spine.get_visual("lower_stop_tab")
    middle_stop_tab = spine.get_visual("middle_stop_tab")
    upper_stop_tab = spine.get_visual("upper_stop_tab")

    lower_cover_plate = lower_cover.get_visual("lower_cover_plate")
    middle_cover_plate = middle_cover.get_visual("middle_cover_plate")
    upper_cover_plate = upper_cover.get_visual("upper_cover_plate")

    lower_journal = lower_branch.get_visual("lower_journal")
    middle_journal = middle_branch.get_visual("middle_journal")
    upper_journal = upper_branch.get_visual("upper_journal")
    lower_tip = lower_branch.get_visual("lower_tip_pad")
    middle_tip = middle_branch.get_visual("middle_tip_pad")
    upper_tip = upper_branch.get_visual("upper_tip_pad")
    lower_lug = lower_branch.get_visual("lower_stop_lug")
    middle_lug = middle_branch.get_visual("middle_stop_lug")
    upper_lug = upper_branch.get_visual("upper_stop_lug")

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

    ctx.expect_contact(lower_branch, spine, elem_a=lower_journal, elem_b=lower_housing, contact_tol=0.0008)
    ctx.expect_contact(middle_branch, spine, elem_a=middle_journal, elem_b=middle_housing, contact_tol=0.0008)
    ctx.expect_contact(upper_branch, spine, elem_a=upper_journal, elem_b=upper_housing, contact_tol=0.0008)

    ctx.expect_overlap(lower_branch, spine, axes="xy", elem_a=lower_journal, elem_b=lower_housing, min_overlap=0.026)
    ctx.expect_overlap(middle_branch, spine, axes="xz", elem_a=middle_journal, elem_b=middle_housing, min_overlap=0.026)
    ctx.expect_overlap(upper_branch, spine, axes="yz", elem_a=upper_journal, elem_b=upper_housing, min_overlap=0.026)

    ctx.expect_gap(lower_cover, spine, axis="x", positive_elem=lower_cover_plate, negative_elem=lower_housing, max_gap=0.0008, max_penetration=0.0)
    ctx.expect_gap(spine, middle_cover, axis="y", positive_elem=middle_housing, negative_elem=middle_cover_plate, max_gap=0.0008, max_penetration=0.0)
    ctx.expect_gap(upper_cover, spine, axis="y", positive_elem=upper_cover_plate, negative_elem=upper_housing, max_gap=0.0008, max_penetration=0.0)

    ctx.expect_overlap(lower_cover, spine, axes="yz", elem_a=lower_cover_plate, elem_b=lower_housing, min_overlap=0.040)
    ctx.expect_overlap(middle_cover, spine, axes="xz", elem_a=middle_cover_plate, elem_b=middle_housing, min_overlap=0.040)
    ctx.expect_overlap(upper_cover, spine, axes="xz", elem_a=upper_cover_plate, elem_b=upper_housing, min_overlap=0.040)

    with ctx.pose({lower_joint: 1.00}):
        ctx.expect_overlap(lower_branch, spine, axes="xz", elem_a=lower_lug, elem_b=lower_stop_tab, min_overlap=0.012)
        ctx.expect_gap(spine, lower_branch, axis="y", positive_elem=lower_stop_tab, negative_elem=lower_lug, min_gap=0.001, max_gap=0.010)

    with ctx.pose({middle_joint: 0.75}):
        ctx.expect_overlap(middle_branch, spine, axes="xy", elem_a=middle_lug, elem_b=middle_stop_tab, min_overlap=0.012)
        ctx.expect_gap(middle_branch, spine, axis="z", positive_elem=middle_lug, negative_elem=middle_stop_tab, min_gap=0.001, max_gap=0.010)

    with ctx.pose({upper_joint: 0.75}):
        ctx.expect_overlap(upper_branch, spine, axes="xy", elem_a=upper_lug, elem_b=upper_stop_tab, min_overlap=0.012)
        ctx.expect_gap(spine, upper_branch, axis="z", positive_elem=upper_stop_tab, negative_elem=upper_lug, min_gap=0.001, max_gap=0.010)

    with ctx.pose({lower_joint: 0.85, middle_joint: 0.65, upper_joint: 0.70}):
        ctx.expect_gap(middle_branch, lower_branch, axis="z", positive_elem=middle_tip, negative_elem=lower_tip, min_gap=0.120)
        ctx.expect_gap(upper_branch, middle_branch, axis="z", positive_elem=upper_tip, negative_elem=middle_tip, min_gap=0.080)

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=18, ignore_adjacent=False)

    def _element_center(part, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    with ctx.pose({lower_joint: 0.0}):
        lower_rest = _element_center(lower_branch, "lower_tip_pad")
    with ctx.pose({lower_joint: 0.85}):
        lower_swept = _element_center(lower_branch, "lower_tip_pad")
    ctx.check(
        "lower_branch_xy_sweep",
        lower_rest is not None
        and lower_swept is not None
        and abs(lower_rest[2] - lower_swept[2]) < 0.010
        and lower_swept[1] > lower_rest[1] + 0.140,
        f"lower tip centers: rest={lower_rest}, swept={lower_swept}",
    )

    with ctx.pose({middle_joint: 0.0}):
        middle_rest = _element_center(middle_branch, "middle_tip_pad")
    with ctx.pose({middle_joint: 0.65}):
        middle_swept = _element_center(middle_branch, "middle_tip_pad")
    ctx.check(
        "middle_branch_xz_sweep",
        middle_rest is not None
        and middle_swept is not None
        and abs(middle_rest[1] - middle_swept[1]) < 0.010
        and middle_swept[2] < middle_rest[2] - 0.120,
        f"middle tip centers: rest={middle_rest}, swept={middle_swept}",
    )

    with ctx.pose({upper_joint: 0.0}):
        upper_rest = _element_center(upper_branch, "upper_tip_pad")
    with ctx.pose({upper_joint: 0.70}):
        upper_swept = _element_center(upper_branch, "upper_tip_pad")
    ctx.check(
        "upper_branch_yz_sweep",
        upper_rest is not None
        and upper_swept is not None
        and abs(upper_rest[0] - upper_swept[0]) < 0.010
        and upper_swept[2] > upper_rest[2] + 0.120
        and upper_swept[1] < upper_rest[1] - 0.060,
        f"upper tip centers: rest={upper_rest}, swept={upper_swept}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
