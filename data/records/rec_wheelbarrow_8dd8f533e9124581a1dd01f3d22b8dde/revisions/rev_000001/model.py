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
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _translated_profile(width: float, depth: float, radius: float, x: float, z: float) -> list[tuple[float, float, float]]:
    return [(px + x, py, z) for px, py in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(px, py, pz) for px, py, pz in points]


def _connect_loops(geom: MeshGeometry, a: list[int], b: list[int]) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(a[i], a[j], b[j])
        geom.add_face(a[i], b[j], b[i])


def _fill_loop(geom: MeshGeometry, loop: list[int], point: tuple[float, float, float], reverse: bool = False) -> None:
    center = geom.add_vertex(*point)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if reverse:
            geom.add_face(center, loop[j], loop[i])
        else:
            geom.add_face(center, loop[i], loop[j])


def _tray_shell() -> MeshGeometry:
    """Thin, open, tapered wheelbarrow tray with an interior floor."""
    geom = MeshGeometry()

    outer_top = _add_loop(geom, _translated_profile(1.12, 0.66, 0.10, 0.02, 0.64))
    outer_bottom = _add_loop(geom, _translated_profile(0.62, 0.36, 0.055, -0.05, 0.34))
    inner_top = _add_loop(geom, _translated_profile(1.02, 0.56, 0.075, 0.02, 0.615))
    inner_bottom = _add_loop(geom, _translated_profile(0.50, 0.26, 0.040, -0.05, 0.370))

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, outer_top, inner_top)
    _connect_loops(geom, inner_top, inner_bottom)
    _connect_loops(geom, inner_bottom, outer_bottom)

    _fill_loop(geom, outer_bottom, (-0.05, 0.0, 0.34), reverse=True)
    _fill_loop(geom, inner_bottom, (-0.05, 0.0, 0.370), reverse=False)

    return geom


def _tray_rim() -> MeshGeometry:
    rim_points = _translated_profile(1.13, 0.67, 0.105, 0.02, 0.645)
    return wire_from_points(
        rim_points,
        radius=0.018,
        radial_segments=16,
        closed_path=True,
        corner_mode="miter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.13, 0.46, 0.20, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.05, 0.055, 0.055, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    rim_yellow = model.material("rim_yellow", rgba=(0.95, 0.72, 0.12, 1.0))

    body = model.part("body")
    body.visual(mesh_from_geometry(_tray_shell(), "tray_shell"), material=tray_green, name="tray_shell")
    body.visual(mesh_from_geometry(_tray_rim(), "tray_rim"), material=tray_green, name="tray_rim")

    for index, side in enumerate((-1.0, 1.0)):
        rail_points = [
            (1.10, side * 0.37, 0.66),
            (0.82, side * 0.35, 0.62),
            (0.50, side * 0.31, 0.42),
            (0.15, side * 0.28, 0.32),
            (-0.38, side * 0.22, 0.28),
            (-0.72, side * 0.12, 0.23),
        ]
        rail = wire_from_points(
            rail_points,
            radius=0.020,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
            corner_segments=10,
        )
        body.visual(mesh_from_geometry(rail, f"side_rail_{index}"), material=dark_metal, name=f"side_rail_{index}")

        body.visual(
            Cylinder(radius=0.033, length=0.24),
            origin=Origin(xyz=(1.08, side * 0.37, 0.66), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"grip_{index}",
        )

        body.visual(
            Cylinder(radius=0.028, length=0.042),
            origin=Origin(xyz=(-0.72, side * 0.089, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"axle_boss_{index}",
        )

    body.visual(
        Cylinder(radius=0.016, length=0.270),
        origin=Origin(xyz=(-0.72, 0.0, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_shaft",
    )

    body.visual(
        Box((0.25, 0.025, 0.070)),
        origin=Origin(xyz=(-0.69, -0.100, 0.235)),
        material=dark_metal,
        name="fork_blade_0",
    )
    body.visual(
        Box((0.25, 0.025, 0.070)),
        origin=Origin(xyz=(-0.69, 0.100, 0.235)),
        material=dark_metal,
        name="fork_blade_1",
    )

    rear_leg = wire_from_points(
        [
            (0.12, -0.31, 0.325),
            (0.60, -0.39, 0.19),
            (0.66, -0.39, 0.030),
            (0.66, 0.39, 0.030),
            (0.60, 0.39, 0.19),
            (0.12, 0.31, 0.325),
        ],
        radius=0.022,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=10,
    )
    body.visual(mesh_from_geometry(rear_leg, "rear_leg"), material=dark_metal, name="rear_leg")

    for index, y in enumerate((-0.39, 0.39)):
        body.visual(
            Box((0.16, 0.090, 0.022)),
            origin=Origin(xyz=(0.66, y, 0.011)),
            material=rubber,
            name=f"foot_pad_{index}",
        )

    for index, x in enumerate((-0.22, 0.12)):
        body.visual(
            Box((0.055, 0.64, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.3225)),
            material=dark_metal,
            name=f"tray_crossbar_{index}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.230,
                0.110,
                inner_radius=0.154,
                carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
                tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
                grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "tire",
        ),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.162,
                0.085,
                rim=WheelRim(
                    inner_radius=0.098,
                    flange_height=0.010,
                    flange_thickness=0.006,
                    bead_seat_depth=0.004,
                ),
                hub=WheelHub(radius=0.038, width=0.070, cap_style="domed"),
                face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.010, window_radius=0.020),
                bore=WheelBore(style="round", diameter=0.030),
            ),
            "rim",
        ),
        material=rim_yellow,
        name="rim",
    )

    model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(-0.72, 0.0, 0.230), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("axle")

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_shaft",
        elem_b="rim",
        reason="The fixed axle shaft is intentionally captured through the wheel hub/bore so the wheel is physically mounted while rotating.",
    )

    ctx.check(
        "front wheel uses continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={axle.articulation_type}",
    )

    ctx.expect_within(
        body,
        wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="rim",
        margin=0.001,
        name="axle shaft is centered in hub profile",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="y",
        elem_a="axle_shaft",
        elem_b="rim",
        min_overlap=0.070,
        name="axle shaft passes through hub width",
    )

    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        positive_elem="fork_blade_1",
        negative_elem="tire",
        min_gap=0.015,
        max_gap=0.060,
        name="positive fork blade clears tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        positive_elem="tire",
        negative_elem="fork_blade_0",
        min_gap=0.015,
        max_gap=0.060,
        name="negative fork blade clears tire sidewall",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="xz",
        elem_a="tire",
        elem_b="fork_blade_0",
        min_overlap=0.045,
        name="wheel sits inside one fork blade profile",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="xz",
        elem_a="tire",
        elem_b="fork_blade_1",
        min_overlap=0.045,
        name="wheel sits inside other fork blade profile",
    )

    rest_position = ctx.part_world_position(wheel)
    with ctx.pose({axle: math.pi}):
        spun_position = ctx.part_world_position(wheel)
        ctx.expect_gap(
            body,
            wheel,
            axis="y",
            positive_elem="fork_blade_1",
            negative_elem="tire",
            min_gap=0.015,
            max_gap=0.060,
            name="spinning tire remains clear of fork",
        )
    ctx.check(
        "wheel spin stays centered on axle",
        rest_position is not None
        and spun_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, spun_position)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
