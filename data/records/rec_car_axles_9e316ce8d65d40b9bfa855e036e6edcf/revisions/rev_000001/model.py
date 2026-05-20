from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length <= 1e-9:
        return (0.0, 1.0, 0.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _de_dion_centerline(samples: int = 53) -> list[tuple[float, float, float]]:
    """Smooth, shallowly bowed transverse axle tube centerline."""
    half_span = 0.88
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        u = -1.0 + 2.0 * i / (samples - 1)
        y = u * half_span
        bow = 1.0 - u * u
        x = -0.040 * bow
        z = 0.420 + 0.035 * (1.0 - abs(u) ** 1.5)
        pts.append((x, y, z))
    return pts


def _slotted_hollow_tube_geometry(
    centerline: list[tuple[float, float, float]],
    *,
    outer_radius: float,
    inner_radius: float,
    radial_segments: int = 34,
    slot_center_angle: float = math.pi / 2.0,
    slot_width: float = 1.05,
) -> MeshGeometry:
    """A C-section hollow tube, with a front cutaway slot to reveal the shafts."""
    geom = MeshGeometry()
    start_angle = slot_center_angle + slot_width / 2.0
    end_angle = slot_center_angle + 2.0 * math.pi - slot_width / 2.0
    angles = [
        start_angle + (end_angle - start_angle) * j / (radial_segments - 1)
        for j in range(radial_segments)
    ]

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    up = (0.0, 0.0, 1.0)
    for i, p in enumerate(centerline):
        if i == 0:
            tangent = (
                centerline[1][0] - p[0],
                centerline[1][1] - p[1],
                centerline[1][2] - p[2],
            )
        elif i == len(centerline) - 1:
            tangent = (
                p[0] - centerline[i - 1][0],
                p[1] - centerline[i - 1][1],
                p[2] - centerline[i - 1][2],
            )
        else:
            tangent = (
                centerline[i + 1][0] - centerline[i - 1][0],
                centerline[i + 1][1] - centerline[i - 1][1],
                centerline[i + 1][2] - centerline[i - 1][2],
            )
        t = _normalize(tangent)
        side = _normalize(_cross(t, up))
        vertical = _normalize(_cross(side, t))

        outer_row: list[int] = []
        inner_row: list[int] = []
        for a in angles:
            # a=0 is visually "up"; a=pi/2 is the forward/+X cutaway side.
            direction = (
                math.cos(a) * vertical[0] + math.sin(a) * side[0],
                math.cos(a) * vertical[1] + math.sin(a) * side[1],
                math.cos(a) * vertical[2] + math.sin(a) * side[2],
            )
            outer_row.append(
                geom.add_vertex(
                    p[0] + outer_radius * direction[0],
                    p[1] + outer_radius * direction[1],
                    p[2] + outer_radius * direction[2],
                )
            )
            inner_row.append(
                geom.add_vertex(
                    p[0] + inner_radius * direction[0],
                    p[1] + inner_radius * direction[1],
                    p[2] + inner_radius * direction[2],
                )
            )
        outer.append(outer_row)
        inner.append(inner_row)

    rows = len(centerline)
    cols = len(angles)
    for i in range(rows - 1):
        for j in range(cols - 1):
            # Outer skin.
            geom.add_face(outer[i][j], outer[i + 1][j], outer[i + 1][j + 1])
            geom.add_face(outer[i][j], outer[i + 1][j + 1], outer[i][j + 1])
            # Inner bore skin.
            geom.add_face(inner[i][j + 1], inner[i + 1][j + 1], inner[i + 1][j])
            geom.add_face(inner[i][j + 1], inner[i + 1][j], inner[i][j])

    # Longitudinal lips along the cutaway edges.
    for i in range(rows - 1):
        for j in (0, cols - 1):
            geom.add_face(outer[i][j], inner[i][j], inner[i + 1][j])
            geom.add_face(outer[i][j], inner[i + 1][j], outer[i + 1][j])

    # Annular end faces, leaving the bore open.
    for j in range(cols - 1):
        geom.add_face(outer[0][j], outer[0][j + 1], inner[0][j + 1])
        geom.add_face(outer[0][j], inner[0][j + 1], inner[0][j])
        geom.add_face(outer[-1][j + 1], outer[-1][j], inner[-1][j])
        geom.add_face(outer[-1][j + 1], inner[-1][j], inner[-1][j + 1])

    return geom


def _origin_y(y: float = 0.0) -> Origin:
    return Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rear_de_dion_tube_axle")

    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    tube_steel = model.material("tube_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    cut_edge = model.material("cut_edge", rgba=(0.55, 0.56, 0.57, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    cv_boot = model.material("cv_boot", rgba=(0.015, 0.014, 0.013, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.05, 0.055, 0.06, 1.0))
    bolt_yellow = model.material("bolt_yellow", rgba=(0.86, 0.63, 0.18, 1.0))

    chassis = model.part("chassis_mount")
    chassis.visual(
        Box((0.40, 0.62, 0.070)),
        origin=Origin(xyz=(-0.060, 0.0, 0.820)),
        material=satin_black,
        name="crossmember",
    )
    chassis.visual(
        Box((0.095, 0.028, 0.225)),
        origin=Origin(xyz=(-0.060, -0.1565, 0.680)),
        material=satin_black,
        name="bracket_0",
    )
    chassis.visual(
        Box((0.095, 0.028, 0.225)),
        origin=Origin(xyz=(-0.060, 0.1565, 0.680)),
        material=satin_black,
        name="bracket_1",
    )
    chassis.visual(
        Cylinder(radius=0.018, length=0.345),
        origin=Origin(xyz=(-0.060, 0.0, 0.620), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt_yellow,
        name="lateral_mount_bolt",
    )

    tube = model.part("de_dion_tube")
    tube_shell = _slotted_hollow_tube_geometry(
        _de_dion_centerline(),
        outer_radius=0.080,
        inner_radius=0.060,
        radial_segments=36,
    )
    tube.visual(
        mesh_from_geometry(tube_shell, "curved_slotted_de_dion_tube"),
        material=tube_steel,
        name="tube_shell",
    )
    tube.visual(
        Box((0.180, 0.245, 0.090)),
        origin=Origin(xyz=(-0.040, 0.0, 0.525)),
        material=tube_steel,
        name="lateral_clamp_block",
    )
    tube.visual(
        Box((0.120, 0.285, 0.018)),
        origin=Origin(xyz=(-0.040, 0.0, 0.575)),
        material=cut_edge,
        name="clamp_top_plate",
    )
    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.054, tube=0.029, radial_segments=18, tubular_segments=52),
        "bearing_ring",
    )
    for side, y in (("side_0", -0.890), ("side_1", 0.890)):
        tube.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, 0.420), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_dark,
            name=f"end_bearing_{side}",
        )
        tube.visual(
            Box((0.075, 0.050, 0.110)),
            origin=Origin(xyz=(-0.006, y, 0.520)),
            material=tube_steel,
            name=f"caliper_lug_{side}",
        )
    support_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0215, tube=0.0060, radial_segments=14, tubular_segments=36),
        "shaft_support_bushing",
    )
    for idx, y in enumerate((-0.560, -0.320, 0.320, 0.560)):
        tube.visual(
            support_mesh,
            origin=Origin(xyz=(-0.020, y, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_dark,
            name=f"shaft_bushing_{idx}",
        )
        tube.visual(
            Box((0.018, 0.020, 0.060)),
            origin=Origin(xyz=(-0.020, y, 0.386)),
            material=cut_edge,
            name=f"bushing_web_{idx}",
        )

    def add_halfshaft(name: str, y_center: float, inner_sign: float) -> None:
        shaft = model.part(name)
        shaft.visual(
            Cylinder(radius=0.016, length=0.650),
            origin=_origin_y(),
            material=bright_steel,
            name="shaft_core",
        )
        for idx, local_y in enumerate((-0.315, 0.315)):
            shaft.visual(
                Sphere(radius=0.031),
                origin=Origin(xyz=(0.0, local_y, 0.0)),
                material=bright_steel,
                name=f"cv_ball_{idx}",
            )
            for rib in range(4):
                offset = local_y - inner_sign * (0.030 + rib * 0.018)
                shaft.visual(
                    Cylinder(radius=0.027 - rib * 0.0025, length=0.022),
                    origin=_origin_y(offset),
                    material=cv_boot,
                    name=f"boot_rib_{idx}_{rib}",
                )
        model.articulation(
            f"tube_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=tube,
            child=shaft,
            origin=Origin(xyz=(-0.020, y_center, 0.435)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=95.0),
        )

    add_halfshaft("half_shaft_0", -0.455, -1.0)
    add_halfshaft("half_shaft_1", 0.455, 1.0)

    def add_hub(name: str, side_sign: float) -> None:
        hub = model.part(name)
        hub.visual(
            Cylinder(radius=0.026, length=0.150),
            origin=_origin_y(side_sign * 0.035),
            material=bright_steel,
            name="stub_axle",
        )
        hub.visual(
            Cylinder(radius=0.118, length=0.026),
            origin=_origin_y(side_sign * 0.090),
            material=hub_metal,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.150, length=0.014),
            origin=_origin_y(side_sign * 0.052),
            material=bright_steel,
            name="brake_rotor",
        )
        hub.visual(
            Cylinder(radius=0.042, length=0.040),
            origin=_origin_y(side_sign * 0.126),
            material=bearing_dark,
            name="center_cap",
        )
        for stud in range(5):
            angle = 2.0 * math.pi * stud / 5.0
            hub.visual(
                Cylinder(radius=0.0065, length=0.030),
                origin=Origin(
                    xyz=(
                        0.072 * math.cos(angle),
                        side_sign * 0.115,
                        0.072 * math.sin(angle),
                    ),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=bolt_yellow,
                name=f"wheel_stud_{stud}",
            )
        model.articulation(
            f"tube_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=tube,
            child=hub,
            origin=Origin(xyz=(0.0, side_sign * 0.890, 0.420)),
            axis=(0.0, side_sign, 0.0),
            motion_limits=MotionLimits(effort=260.0, velocity=80.0),
        )

    add_hub("wheel_hub_0", -1.0)
    add_hub("wheel_hub_1", 1.0)

    model.articulation(
        "chassis_to_tube",
        ArticulationType.FIXED,
        parent=chassis,
        child=tube,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tube = object_model.get_part("de_dion_tube")
    half_0 = object_model.get_part("half_shaft_0")
    half_1 = object_model.get_part("half_shaft_1")
    hub_0 = object_model.get_part("wheel_hub_0")
    hub_1 = object_model.get_part("wheel_hub_1")

    shaft_bearing_pairs = (
        (half_0, "shaft_bushing_0"),
        (half_0, "shaft_bushing_1"),
        (half_1, "shaft_bushing_2"),
        (half_1, "shaft_bushing_3"),
    )
    for shaft, bushing in shaft_bearing_pairs:
        ctx.allow_overlap(
            tube,
            shaft,
            elem_a=bushing,
            elem_b="shaft_core",
            reason=(
                "The half-shaft core is intentionally captured with a slight "
                "pressed bearing-seat overlap inside the de Dion tube."
            ),
        )

    for hub, bearing in (
        (hub_0, "end_bearing_side_0"),
        (hub_1, "end_bearing_side_1"),
    ):
        ctx.allow_overlap(
            tube,
            hub,
            elem_a=bearing,
            elem_b="stub_axle",
            reason=(
                "The wheel hub stub axle is intentionally seated through the "
                "end bearing race with a tiny captured-shaft overlap."
            ),
        )

    continuous_names = (
        "tube_to_half_shaft_0",
        "tube_to_half_shaft_1",
        "tube_to_wheel_hub_0",
        "tube_to_wheel_hub_1",
    )
    ctx.check(
        "half shafts and hubs are continuous joints",
        all(
            object_model.get_articulation(name).articulation_type
            == ArticulationType.CONTINUOUS
            for name in continuous_names
        ),
        details=str(continuous_names),
    )
    ctx.check(
        "tube is fixed to the chassis",
        object_model.get_articulation("chassis_to_tube").articulation_type
        == ArticulationType.FIXED,
    )

    for shaft in (half_0, half_1):
        ctx.expect_within(
            shaft,
            tube,
            axes="xz",
            inner_elem="shaft_core",
            outer_elem="tube_shell",
            margin=0.002,
            name=f"{shaft.name} is carried inside the tube bore",
        )
        ctx.expect_overlap(
            shaft,
            tube,
            axes="y",
            min_overlap=0.58,
            elem_a="shaft_core",
            elem_b="tube_shell",
            name=f"{shaft.name} runs longitudinally inside the tube",
        )

    for shaft, bushing in shaft_bearing_pairs:
        ctx.expect_within(
            shaft,
            tube,
            axes="xz",
            inner_elem="shaft_core",
            outer_elem=bushing,
            margin=0.001,
            name=f"{shaft.name} is centered in {bushing}",
        )
        ctx.expect_overlap(
            shaft,
            tube,
            axes="y",
            min_overlap=0.010,
            elem_a="shaft_core",
            elem_b=bushing,
            name=f"{shaft.name} is retained by {bushing}",
        )

    for hub in (hub_0, hub_1):
        ctx.expect_within(
            hub,
            tube,
            axes="xz",
            inner_elem="stub_axle",
            outer_elem="tube_shell",
            margin=0.002,
            name=f"{hub.name} is concentric with the tube end",
        )
        ctx.expect_overlap(
            hub,
            tube,
            axes="y",
            min_overlap=0.025,
            elem_a="stub_axle",
            elem_b="tube_shell",
            name=f"{hub.name} stub is retained in the end bearing",
        )

    for hub, bearing in (
        (hub_0, "end_bearing_side_0"),
        (hub_1, "end_bearing_side_1"),
    ):
        ctx.expect_within(
            hub,
            tube,
            axes="xz",
            inner_elem="stub_axle",
            outer_elem=bearing,
            margin=0.001,
            name=f"{hub.name} stub is centered in {bearing}",
        )
        ctx.expect_overlap(
            hub,
            tube,
            axes="y",
            min_overlap=0.020,
            elem_a="stub_axle",
            elem_b=bearing,
            name=f"{hub.name} stub passes through {bearing}",
        )

    return ctx.report()


object_model = build_object_model()
