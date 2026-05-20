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
    mesh_from_geometry,
    rounded_rect_profile,
)


def _profile(width: float, depth: float, radius: float) -> list[tuple[float, float]]:
    return rounded_rect_profile(width, depth, radius, corner_segments=8)


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _connect_loops(geom: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geom.add_face(a[i], b[j], b[i])
            geom.add_face(a[i], a[j], b[j])
        else:
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])


def _cap_loop(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    c = geom.add_vertex(*center)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geom.add_face(c, loop[j], loop[i])
        else:
            geom.add_face(c, loop[i], loop[j])


def _loop_at_z(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _profile(width, depth, radius)]


def _hollow_bin_shell() -> MeshGeometry:
    """Tapered open-top wheelie-bin body with an integral thick bottom."""
    geom = MeshGeometry()
    outer_low = _add_loop(geom, _loop_at_z(0.46, 0.52, 0.060, 0.080))
    outer_mid = _add_loop(geom, _loop_at_z(0.54, 0.64, 0.075, 0.520))
    outer_top = _add_loop(geom, _loop_at_z(0.60, 0.72, 0.085, 1.030))
    inner_low = _add_loop(geom, _loop_at_z(0.34, 0.42, 0.050, 0.165))
    inner_top = _add_loop(geom, _loop_at_z(0.50, 0.62, 0.070, 1.005))

    _connect_loops(geom, outer_low, outer_mid)
    _connect_loops(geom, outer_mid, outer_top)
    _connect_loops(geom, inner_top, inner_low, flip=True)
    _connect_loops(geom, outer_top, inner_top)
    _connect_loops(geom, outer_low, inner_low, flip=True)
    return geom


def _ring_mesh(outer_size: tuple[float, float], inner_size: tuple[float, float], height: float, radius: float, name: str):
    geom = MeshGeometry()
    z0 = -height * 0.5
    z1 = height * 0.5
    outer_bottom = _add_loop(geom, [(x, y, z0) for x, y in _profile(outer_size[0], outer_size[1], radius)])
    outer_top = _add_loop(geom, [(x, y, z1) for x, y in _profile(outer_size[0], outer_size[1], radius)])
    inner_bottom = _add_loop(geom, [(x, y, z0) for x, y in _profile(inner_size[0], inner_size[1], max(0.001, radius * 0.75))])
    inner_top = _add_loop(geom, [(x, y, z1) for x, y in _profile(inner_size[0], inner_size[1], max(0.001, radius * 0.75))])
    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, inner_top, inner_bottom, flip=True)
    _connect_loops(geom, outer_top, inner_top)
    _connect_loops(geom, inner_bottom, outer_bottom)
    return mesh_from_geometry(geom, name)


def _solid_prism_mesh(width: float, depth: float, height: float, radius: float, name: str):
    geom = MeshGeometry()
    z0 = -height * 0.5
    z1 = height * 0.5
    bottom = _add_loop(geom, [(x, y, z0) for x, y in _profile(width, depth, radius)])
    top = _add_loop(geom, [(x, y, z1) for x, y in _profile(width, depth, radius)])
    _connect_loops(geom, bottom, top)
    _cap_loop(geom, bottom, (0.0, 0.0, z0), flip=True)
    _cap_loop(geom, top, (0.0, 0.0, z1))
    return mesh_from_geometry(geom, name)


def _add_weathered_shell_details(bin_shell, *, body_green, dark_green, rubber_black, galvanized) -> None:
    bin_shell.visual(
        _ring_mesh((0.62, 0.74), (0.52, 0.62), 0.012, 0.075, "top_seal_ring"),
        origin=Origin(xyz=(0.0, 0.0, 1.036)),
        material=rubber_black,
        name="top_seal",
    )
    bin_shell.visual(Box((0.56, 0.040, 0.055)), origin=Origin(xyz=(0.0, 0.360, 0.965)), material=dark_green, name="front_reinforced_lip")
    bin_shell.visual(Box((0.56, 0.040, 0.055)), origin=Origin(xyz=(0.0, -0.360, 0.965)), material=dark_green, name="rear_reinforced_lip")
    bin_shell.visual(Box((0.040, 0.56, 0.050)), origin=Origin(xyz=(0.300, 0.0, 0.960)), material=dark_green, name="side_lip_0")
    bin_shell.visual(Box((0.040, 0.56, 0.050)), origin=Origin(xyz=(-0.300, 0.0, 0.960)), material=dark_green, name="side_lip_1")

    for idx, x in enumerate((-0.245, 0.245)):
        bin_shell.visual(Box((0.045, 0.050, 0.740)), origin=Origin(xyz=(x, 0.333, 0.550)), material=dark_green, name=f"front_rib_{idx}")
        bin_shell.visual(Box((0.045, 0.050, 0.700)), origin=Origin(xyz=(x, -0.335, 0.540)), material=dark_green, name=f"rear_rib_{idx}")
    for idx, x in enumerate((-0.255, 0.255)):
        bin_shell.visual(Box((0.045, 0.170, 0.135)), origin=Origin(xyz=(x, -0.305, 0.245)), material=dark_green, name=f"wheel_guard_{idx}")

    bin_shell.visual(
        Cylinder(radius=0.020, length=0.560),
        origin=Origin(xyz=(0.0, -0.405, 0.920), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_green,
        name="rear_pull_handle",
    )
    bin_shell.visual(Box((0.050, 0.060, 0.185)), origin=Origin(xyz=(0.255, -0.372, 0.870)), material=body_green, name="handle_mount_0")
    bin_shell.visual(Box((0.050, 0.060, 0.185)), origin=Origin(xyz=(-0.255, -0.372, 0.870)), material=body_green, name="handle_mount_1")

    bin_shell.visual(
        Cylinder(radius=0.018, length=0.638),
        origin=Origin(xyz=(0.0, -0.300, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_axle",
    )
    bin_shell.visual(Box((0.520, 0.085, 0.080)), origin=Origin(xyz=(0.0, -0.275, 0.125)), material=dark_green, name="axle_saddle")

    bin_shell.visual(Box((0.670, 0.100, 0.070)), origin=Origin(xyz=(0.0, -0.405, 0.950)), material=dark_green, name="hinge_rain_shroud")
    for idx, x in enumerate((-0.250, 0.0, 0.250)):
        bin_shell.visual(Box((0.095, 0.035, 0.150)), origin=Origin(xyz=(x, -0.450, 1.015)), material=dark_green, name=f"hinge_post_{idx}")
        bin_shell.visual(Box((0.095, 0.070, 0.025)), origin=Origin(xyz=(x, -0.420, 1.084)), material=dark_green, name=f"hinge_bridge_{idx}")
        bin_shell.visual(
            Cylinder(radius=0.018, length=0.100),
            origin=Origin(xyz=(x, -0.390, 1.094), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"fixed_hinge_knuckle_{idx}",
        )
        bin_shell.visual(Box((0.105, 0.050, 0.032)), origin=Origin(xyz=(x, -0.420, 1.080)), material=dark_green, name=f"hinge_boss_{idx}")


def _add_lid_geometry(lid, *, lid_green, rubber_black, galvanized) -> None:
    lid.visual(
        _solid_prism_mesh(0.74, 0.86, 0.040, 0.095, "lid_rounded_panel"),
        origin=Origin(xyz=(0.0, 0.450, -0.020)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(Box((0.740, 0.040, 0.060)), origin=Origin(xyz=(0.0, 0.805, -0.070)), material=lid_green, name="drip_skirt")
    lid.visual(Box((0.040, 0.720, 0.060)), origin=Origin(xyz=(0.360, 0.430, -0.070)), material=lid_green, name="side_drip_0")
    lid.visual(Box((0.040, 0.720, 0.060)), origin=Origin(xyz=(-0.360, 0.430, -0.070)), material=lid_green, name="side_drip_1")
    lid.visual(
        _ring_mesh((0.62, 0.72), (0.52, 0.62), 0.012, 0.065, "lid_compression_gasket"),
        origin=Origin(xyz=(0.0, 0.390, -0.046)),
        material=rubber_black,
        name="lid_gasket",
    )
    lid.visual(Box((0.460, 0.060, 0.040)), origin=Origin(xyz=(0.0, 0.840, -0.045)), material=lid_green, name="front_grip_lip")
    lid.visual(Box((0.390, 0.032, 0.026)), origin=Origin(xyz=(0.0, 0.808, -0.075)), material=rubber_black, name="front_drip_edge")
    for idx, x in enumerate((-0.125, 0.125)):
        lid.visual(
            Cylinder(radius=0.018, length=0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"moving_hinge_knuckle_{idx}",
        )
        lid.visual(Box((0.120, 0.120, 0.018)), origin=Origin(xyz=(x, 0.055, -0.024)), material=lid_green, name=f"hinge_leaf_{idx}")


def _add_wheel_geometry(wheel, *, prefix: str, rubber_black, wheel_plastic, galvanized, bushing_side: float) -> None:
    wheel.visual(
        Cylinder(radius=0.105, length=0.072),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.074, length=0.064),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_plastic,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.032, length=0.082),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(bushing_side * 0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="inner_bushing",
    )
    for idx in range(16):
        theta = idx * math.tau / 16.0
        wheel.visual(
            Box((0.078, 0.012, 0.014)),
            origin=Origin(
                xyz=(0.0, -math.sin(theta) * 0.105, math.cos(theta) * 0.105),
                rpy=(theta, 0.0, 0.0),
            ),
            material=rubber_black,
            name=f"tread_block_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wheelie_bin")

    body_green = model.material("uv_stable_green", rgba=(0.04, 0.31, 0.15, 1.0))
    dark_green = model.material("dark_reinforced_plastic", rgba=(0.025, 0.20, 0.10, 1.0))
    lid_green = model.material("slightly_lighter_lid", rgba=(0.05, 0.37, 0.18, 1.0))
    rubber_black = model.material("black_epdm_rubber", rgba=(0.015, 0.017, 0.015, 1.0))
    galvanized = model.material("galvanized_stainless", rgba=(0.72, 0.74, 0.70, 1.0))
    wheel_plastic = model.material("dark_wheel_plastic", rgba=(0.055, 0.060, 0.060, 1.0))

    bin_shell = model.part("bin_shell")
    bin_shell.visual(
        mesh_from_geometry(_hollow_bin_shell(), "tapered_hollow_bin_shell"),
        material=body_green,
        name="hollow_shell",
    )
    bin_shell.visual(Box((0.360, 0.440, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.160)), material=dark_green, name="bottom_floor")
    _add_weathered_shell_details(
        bin_shell,
        body_green=body_green,
        dark_green=dark_green,
        rubber_black=rubber_black,
        galvanized=galvanized,
    )

    lid = model.part("lid")
    _add_lid_geometry(lid, lid_green=lid_green, rubber_black=rubber_black, galvanized=galvanized)

    wheel_0 = model.part("wheel_0")
    _add_wheel_geometry(wheel_0, prefix="wheel_0", rubber_black=rubber_black, wheel_plastic=wheel_plastic, galvanized=galvanized, bushing_side=-1.0)

    wheel_1 = model.part("wheel_1")
    _add_wheel_geometry(wheel_1, prefix="wheel_1", rubber_black=rubber_black, wheel_plastic=wheel_plastic, galvanized=galvanized, bushing_side=1.0)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=bin_shell,
        child=lid,
        origin=Origin(xyz=(0.0, -0.390, 1.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=math.radians(105.0)),
    )

    model.articulation(
        "wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=bin_shell,
        child=wheel_0,
        origin=Origin(xyz=(0.360, -0.300, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=bin_shell,
        child=wheel_1,
        origin=Origin(xyz=(-0.360, -0.300, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bin_shell = object_model.get_part("bin_shell")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        bin_shell,
        axis="z",
        positive_elem="lid_gasket",
        negative_elem="top_seal",
        max_gap=0.0005,
        max_penetration=0.0,
        name="closed lid gasket seats on raised seal",
    )
    ctx.expect_overlap(
        lid,
        bin_shell,
        axes="xy",
        elem_a="lid_gasket",
        elem_b="top_seal",
        min_overlap=0.50,
        name="gasket fully overlaps the bin mouth seal",
    )

    ctx.expect_gap(
        wheel_0,
        bin_shell,
        axis="x",
        positive_elem="hub",
        negative_elem="rear_axle",
        max_gap=0.0005,
        max_penetration=0.0,
        name="wheel_0 hub bears against axle end",
    )
    ctx.expect_overlap(
        wheel_0,
        bin_shell,
        axes="yz",
        elem_a="hub",
        elem_b="rear_axle",
        min_overlap=0.030,
        name="wheel_0 hub is coaxial with axle",
    )
    ctx.expect_gap(
        bin_shell,
        wheel_1,
        axis="x",
        positive_elem="rear_axle",
        negative_elem="hub",
        max_gap=0.0005,
        max_penetration=0.0,
        name="wheel_1 hub bears against axle end",
    )
    ctx.expect_overlap(
        wheel_1,
        bin_shell,
        axes="yz",
        elem_a="hub",
        elem_b="rear_axle",
        min_overlap=0.030,
        name="wheel_1 hub is coaxial with axle",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_grip_lip")
    with ctx.pose({lid_hinge: 1.2}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_grip_lip")
    ctx.check(
        "lid hinge raises the front lip",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.50,
        details=f"closed={closed_front}, open={open_front}",
    )

    return ctx.report()


object_model = build_object_model()
