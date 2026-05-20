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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _add_loop(mesh: MeshGeometry, pts: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in pts]


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    hx = width / 2.0
    hy = depth / 2.0
    return [
        (hx, -hy, z),
        (hx, hy, z),
        (-hx, hy, z),
        (-hx, -hy, z),
    ]


def _connect_loops(mesh: MeshGeometry, lower: list[int], upper: list[int], *, reverse: bool = False) -> None:
    count = len(lower)
    for i in range(count):
        a = lower[i]
        b = lower[(i + 1) % count]
        c = upper[(i + 1) % count]
        d = upper[i]
        if reverse:
            _add_quad(mesh, a, d, c, b)
        else:
            _add_quad(mesh, a, b, c, d)


def _jar_shell_geometry() -> MeshGeometry:
    """Tapered hollow square jar with an open top and a thick raised bottom."""
    mesh = MeshGeometry()

    outer_bottom = _add_loop(mesh, _rect_loop(0.190, 0.200, 0.490))
    inner_floor = _add_loop(mesh, _rect_loop(0.160, 0.170, 0.510))
    outer_top = _add_loop(mesh, _rect_loop(0.220, 0.230, 0.850))
    inner_top = _add_loop(mesh, _rect_loop(0.202, 0.212, 0.850))

    # Exterior tapered walls, inner cavity walls, top lip, and thick raised floor.
    _connect_loops(mesh, outer_bottom, outer_top)
    _connect_loops(mesh, inner_floor, inner_top, reverse=True)
    _connect_loops(mesh, inner_top, outer_top)
    _connect_loops(mesh, outer_bottom, inner_floor, reverse=True)

    # Bottom plate and interior floor plate.
    mesh.add_face(outer_bottom[0], outer_bottom[1], outer_bottom[2])
    mesh.add_face(outer_bottom[0], outer_bottom[2], outer_bottom[3])
    mesh.add_face(inner_floor[0], inner_floor[2], inner_floor[1])
    mesh.add_face(inner_floor[0], inner_floor[3], inner_floor[2])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    clear_polycarbonate = model.material("clear_polycarbonate", rgba=(0.70, 0.92, 1.00, 0.34))
    smoky_polycarbonate = model.material("smoky_polycarbonate", rgba=(0.50, 0.75, 0.90, 0.42))
    white_marking = model.material("white_marking", rgba=(0.95, 0.98, 1.00, 1.0))
    black_marking = model.material("black_marking", rgba=(0.02, 0.02, 0.02, 1.0))
    amber = model.material("amber_safety", rgba=(0.95, 0.56, 0.08, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.240, 0.270, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_black,
        name="base_plinth",
    )
    housing.visual(
        Box((0.205, 0.225, 0.410)),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=satin_black,
        name="motor_column",
    )
    housing.visual(
        Box((0.176, 0.006, 0.250)),
        origin=Origin(xyz=(0.0, -0.1155, 0.250)),
        material=stainless,
        name="front_panel",
    )
    housing.visual(
        Box((0.155, 0.155, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.476)),
        material=dark_rubber,
        name="jar_pad",
    )
    housing.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=stainless,
        name="drive_coupler",
    )
    for i, z in enumerate((0.175, 0.205, 0.235, 0.265)):
        housing.visual(
            Box((0.105, 0.0025, 0.006)),
            origin=Origin(xyz=(0.0, -0.1190, z)),
            material=black_marking,
            name=f"vent_slit_{i}",
        )
    for x in (-0.082, 0.082):
        for y in (-0.100, 0.100):
            housing.visual(
                Box((0.044, 0.044, 0.018)),
                origin=Origin(xyz=(x, y, 0.009)),
                material=dark_rubber,
                name=f"foot_{x:+.3f}_{y:+.3f}",
            )

    jar = model.part("jar")
    jar.visual(
        mesh_from_geometry(_jar_shell_geometry(), "tapered_polycarbonate_jar"),
        material=clear_polycarbonate,
        name="jar_shell",
    )
    jar.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.516)),
        material=dark_rubber,
        name="blade_bearing",
    )
    jar.visual(
        Box((0.234, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.122, 0.857)),
        material=clear_polycarbonate,
        name="front_rim",
    )
    jar.visual(
        Box((0.234, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.122, 0.857)),
        material=clear_polycarbonate,
        name="rear_rim",
    )
    jar.visual(
        Box((0.014, 0.234, 0.014)),
        origin=Origin(xyz=(0.117, 0.0, 0.857)),
        material=clear_polycarbonate,
        name="side_rim_0",
    )
    jar.visual(
        Box((0.014, 0.234, 0.014)),
        origin=Origin(xyz=(-0.117, 0.0, 0.857)),
        material=clear_polycarbonate,
        name="side_rim_1",
    )
    jar.visual(
        Box((0.066, 0.034, 0.025)),
        origin=Origin(xyz=(0.137, 0.0, 0.705)),
        material=clear_polycarbonate,
        name="handle_upper_mount",
    )
    jar.visual(
        Box((0.066, 0.034, 0.025)),
        origin=Origin(xyz=(0.137, 0.0, 0.565)),
        material=clear_polycarbonate,
        name="handle_lower_mount",
    )
    jar.visual(
        Box((0.022, 0.034, 0.175)),
        origin=Origin(xyz=(0.170, 0.0, 0.635)),
        material=clear_polycarbonate,
        name="side_handle",
    )
    jar.visual(
        Box((0.025, 0.035, 0.028)),
        origin=Origin(xyz=(0.0925, 0.138, 0.862)),
        material=stainless,
        name="hinge_ear_0",
    )
    jar.visual(
        Box((0.025, 0.035, 0.028)),
        origin=Origin(xyz=(-0.0925, 0.138, 0.862)),
        material=stainless,
        name="hinge_ear_1",
    )
    model.articulation(
        "housing_to_jar",
        ArticulationType.FIXED,
        parent=housing,
        child=jar,
        origin=Origin(),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.055,
                0.017,
                4,
                thickness=0.006,
                blade_pitch_deg=10.0,
                blade_sweep_deg=8.0,
                blade=FanRotorBlade(shape="broad", camber=0.04, tip_clearance=0.002),
                hub=FanRotorHub(style="flat", rear_collar_height=0.004, rear_collar_radius=0.016),
            ),
            "four_blade_cross",
        ),
        material=stainless,
        name="cross_blades",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="blade_hub",
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.529)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=120.0, lower=0.0, upper=2.0 * math.pi),
    )

    sound_lid = model.part("sound_lid")
    sound_lid.visual(
        Cylinder(radius=0.008, length=0.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    sound_lid.visual(
        Box((0.300, 0.305, 0.012)),
        origin=Origin(xyz=(0.0, -0.170, 0.014)),
        material=smoky_polycarbonate,
        name="top_panel",
    )
    sound_lid.visual(
        Box((0.150, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, 0.010)),
        material=smoky_polycarbonate,
        name="hinge_bridge",
    )
    sound_lid.visual(
        Box((0.010, 0.300, 0.086)),
        origin=Origin(xyz=(0.150, -0.150, -0.034)),
        material=smoky_polycarbonate,
        name="side_flange_0",
    )
    sound_lid.visual(
        Box((0.010, 0.300, 0.086)),
        origin=Origin(xyz=(-0.150, -0.150, -0.034)),
        material=smoky_polycarbonate,
        name="side_flange_1",
    )
    sound_lid.visual(
        Box((0.300, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, -0.305, -0.026)),
        material=smoky_polycarbonate,
        name="front_flange",
    )
    sound_lid.visual(
        Box((0.080, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.310, 0.018)),
        material=dark_rubber,
        name="front_pull",
    )
    model.articulation(
        "jar_to_sound_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=sound_lid,
        origin=Origin(xyz=(0.0, 0.138, 0.862)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    latch = model.part("front_latch")
    latch.visual(
        Box((0.092, 0.010, 0.110)),
        origin=Origin(xyz=(0.0, -0.005, -0.055)),
        material=amber,
        name="latch_paddle",
    )
    latch.visual(
        Cylinder(radius=0.0065, length=0.096),
        origin=Origin(xyz=(0.0, -0.0065, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="latch_barrel",
    )
    model.articulation(
        "housing_to_front_latch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=latch,
        origin=Origin(xyz=(0.0, -0.1185, 0.425)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.85),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="dial_cap",
    )
    speed_dial.visual(
        Box((0.004, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, -0.012, 0.014)),
        material=white_marking,
        name="dial_pointer",
    )
    model.articulation(
        "housing_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_dial,
        origin=Origin(xyz=(0.0, -0.128, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    jar = object_model.get_part("jar")
    blade = object_model.get_part("blade")
    sound_lid = object_model.get_part("sound_lid")
    front_latch = object_model.get_part("front_latch")
    housing = object_model.get_part("housing")
    lid_hinge = object_model.get_articulation("jar_to_sound_lid")
    blade_joint = object_model.get_articulation("jar_to_blade")
    latch_hinge = object_model.get_articulation("housing_to_front_latch")

    ctx.expect_contact(
        blade,
        jar,
        elem_a="blade_hub",
        elem_b="blade_bearing",
        contact_tol=0.001,
        name="blade hub is seated on the jar bearing",
    )
    ctx.expect_contact(
        sound_lid,
        jar,
        elem_a="hinge_barrel",
        elem_b="hinge_ear_0",
        contact_tol=0.0015,
        name="sound lid hinge barrel is captured by the jar hinge ears",
    )
    ctx.expect_gap(
        housing,
        front_latch,
        axis="y",
        max_gap=0.004,
        max_penetration=0.0001,
        positive_elem="front_panel",
        negative_elem="latch_paddle",
        name="closed safety latch lies against the front panel",
    )
    with ctx.pose({lid_hinge: 1.0}):
        ctx.expect_gap(
            sound_lid,
            jar,
            axis="z",
            min_gap=0.030,
            positive_elem="front_pull",
            negative_elem="front_rim",
            name="raised sound lid clears the jar rim",
        )
    with ctx.pose({blade_joint: math.pi / 2.0}):
        ctx.expect_contact(
            blade,
            jar,
            elem_a="blade_hub",
            elem_b="blade_bearing",
            contact_tol=0.001,
            name="rotated blade remains seated on its bearing",
        )
    rest_latch = ctx.part_element_world_aabb(front_latch, elem="latch_paddle")
    with ctx.pose({latch_hinge: 0.65}):
        moved_latch = ctx.part_element_world_aabb(front_latch, elem="latch_paddle")
    ctx.check(
        "safety latch swings outward from the housing",
        rest_latch is not None
        and moved_latch is not None
        and moved_latch[0][1] < rest_latch[0][1] - 0.005,
        details=f"rest={rest_latch}, moved={moved_latch}",
    )

    return ctx.report()


object_model = build_object_model()
