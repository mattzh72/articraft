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


def _add_loop(mesh: MeshGeometry, x: float, profile: list[tuple[float, float]]) -> list[int]:
    """Add a loop in a YZ plane. Profile coordinates are (y, z)."""
    return [mesh.add_vertex(x, y, z) for y, z in profile]


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _stepped_tunnel_mesh() -> MeshGeometry:
    """One connected hollow, rounded-rectangle tunnel with molded end flanges."""
    opening_w, opening_h = 0.300, 0.380
    tunnel_w, tunnel_h = 0.360, 0.460
    flange_w, flange_h = 0.460, 0.560
    # Stations make a tapered molded trim at each face and a straight tunnel liner
    # through the wall thickness along X.
    stations = [
        (-0.115, flange_w, flange_h, 0.040),
        (-0.092, tunnel_w, tunnel_h, 0.030),
        (0.092, tunnel_w, tunnel_h, 0.030),
        (0.115, flange_w, flange_h, 0.040),
    ]

    mesh = MeshGeometry()
    inner_profile = rounded_rect_profile(opening_w, opening_h, 0.025, corner_segments=8)
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []
    for x, width, height, radius in stations:
        outer_loops.append(_add_loop(mesh, x, rounded_rect_profile(width, height, radius, corner_segments=8)))
        inner_loops.append(_add_loop(mesh, x, inner_profile))

    n = len(inner_profile)
    for s in range(len(stations) - 1):
        o0, o1 = outer_loops[s], outer_loops[s + 1]
        i0, i1 = inner_loops[s], inner_loops[s + 1]
        for j in range(n):
            k = (j + 1) % n
            _quad(mesh, o0[j], o0[k], o1[k], o1[j])
            _quad(mesh, i0[k], i0[j], i1[j], i1[k])

    # Annular faces at the interior and exterior openings.
    o0, i0 = outer_loops[0], inner_loops[0]
    o1, i1 = outer_loops[-1], inner_loops[-1]
    for j in range(n):
        k = (j + 1) % n
        _quad(mesh, o0[k], o0[j], i0[j], i0[k])
        _quad(mesh, o1[j], o1[k], i1[k], i1[j])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tunnel_pet_door")

    molded_plastic = model.material("warm_gray_molded_plastic", rgba=(0.62, 0.60, 0.56, 1.0))
    dark_gasket = model.material("dark_flexible_gasket", rgba=(0.04, 0.045, 0.045, 1.0))
    brushed_pin = model.material("brushed_steel_pin", rgba=(0.70, 0.70, 0.66, 1.0))
    amber_flap = model.material("smoky_amber_translucent_flap", rgba=(0.74, 0.52, 0.25, 0.52))
    smoke_hood = model.material("smoke_clear_weather_hood", rgba=(0.46, 0.55, 0.62, 0.45))

    frame = model.part("tunnel_frame")
    frame.visual(
        mesh_from_geometry(_stepped_tunnel_mesh(), "tunnel_frame_shell"),
        material=molded_plastic,
        name="tunnel_shell",
    )
    # Interior flap hinge: a pin carried by molded side bosses just under the
    # inner top lip of the tunnel.
    frame.visual(
        Cylinder(radius=0.0035, length=0.318),
        origin=Origin(xyz=(-0.075, 0.0, 0.176), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="inner_hinge_pin",
    )
    for suffix, y in (("0", -0.170), ("1", 0.170)):
        frame.visual(
            Box((0.030, 0.026, 0.038)),
            origin=Origin(xyz=(-0.075, y, 0.176)),
            material=molded_plastic,
            name=f"inner_hinge_boss_{suffix}",
        )

    # Exterior weather-hood hinge: slightly proud of the outside trim face.
    frame.visual(
        Cylinder(radius=0.0035, length=0.372),
        origin=Origin(xyz=(0.130, 0.0, 0.238), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="hood_hinge_pin",
    )
    for suffix, y in (("0", -0.192), ("1", 0.192)):
        frame.visual(
            Box((0.032, 0.024, 0.040)),
            origin=Origin(xyz=(0.124, y, 0.238)),
            material=molded_plastic,
            name=f"hood_hinge_boss_{suffix}",
        )

    # Flush screw caps on the inner trim indicate the fixed frame is fastened
    # through the wall, not merely floating around the opening.
    for row, z in enumerate((-0.210, 0.210)):
        for col, y in enumerate((-0.180, 0.180)):
            frame.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(-0.117, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brushed_pin,
                name=f"inner_screw_{row}_{col}",
            )

    flap = model.part("inner_flap")
    flap.visual(
        Box((0.006, 0.272, 0.338)),
        origin=Origin(xyz=(0.0, 0.0, -0.176)),
        material=amber_flap,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0070, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gasket,
        name="flap_hinge_barrel",
    )
    # Rubber perimeter and a small magnetic weight strip at the bottom.
    flap.visual(
        Box((0.008, 0.286, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_gasket,
        name="flap_top_gasket",
    )
    flap.visual(
        Box((0.008, 0.286, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.342)),
        material=dark_gasket,
        name="flap_bottom_weight",
    )
    for suffix, y in (("0", -0.142), ("1", 0.142)):
        flap.visual(
            Box((0.008, 0.012, 0.328)),
            origin=Origin(xyz=(0.0, y, -0.176)),
            material=dark_gasket,
            name=f"flap_side_gasket_{suffix}",
        )

    hood = model.part("weather_hood")
    hood.visual(
        Cylinder(radius=0.0062, length=0.348),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gasket,
        name="hood_hinge_barrel",
    )
    hood.visual(
        Box((0.010, 0.340, 0.030)),
        origin=Origin(xyz=(0.0105, 0.0, -0.016)),
        material=dark_gasket,
        name="hood_hinge_leaf",
    )
    hood.visual(
        Box((0.118, 0.372, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, -0.020), rpy=(0.0, -0.18, 0.0)),
        material=smoke_hood,
        name="hood_roof",
    )
    hood.visual(
        Box((0.010, 0.368, 0.074)),
        origin=Origin(xyz=(0.119, 0.0, -0.066)),
        material=smoke_hood,
        name="hood_front_lip",
    )
    for suffix, y in (("0", -0.188), ("1", 0.188)):
        hood.visual(
            Box((0.110, 0.008, 0.124)),
            origin=Origin(xyz=(0.066, y, -0.074)),
            material=smoke_hood,
            name=f"hood_side_cheek_{suffix}",
        )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(-0.075, 0.0, 0.176)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "frame_to_hood",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hood,
        origin=Origin(xyz=(0.130, 0.0, 0.238)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.2, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("tunnel_frame")
    flap = object_model.get_part("inner_flap")
    hood = object_model.get_part("weather_hood")
    flap_joint = object_model.get_articulation("frame_to_flap")
    hood_joint = object_model.get_articulation("frame_to_hood")

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="inner_hinge_pin",
        elem_b="flap_hinge_barrel",
        reason="The flap barrel is intentionally modeled around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        flap,
        axes="y",
        elem_a="inner_hinge_pin",
        elem_b="flap_hinge_barrel",
        min_overlap=0.220,
        name="flap hinge barrel shares the horizontal pin span",
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="x",
        positive_elem="flap_hinge_barrel",
        negative_elem="inner_hinge_pin",
        max_penetration=0.012,
        name="flap hinge pin capture is local",
    )

    ctx.allow_overlap(
        frame,
        hood,
        elem_a="hood_hinge_pin",
        elem_b="hood_hinge_barrel",
        reason="The exterior hood barrel is intentionally captured on its fixed hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        hood,
        axes="y",
        elem_a="hood_hinge_pin",
        elem_b="hood_hinge_barrel",
        min_overlap=0.330,
        name="hood hinge barrel shares the horizontal pin span",
    )
    ctx.expect_gap(
        hood,
        frame,
        axis="x",
        positive_elem="hood_hinge_barrel",
        negative_elem="hood_hinge_pin",
        max_penetration=0.011,
        name="hood hinge pin capture is local",
    )

    ctx.expect_within(
        flap,
        frame,
        axes="yz",
        inner_elem="flap_panel",
        outer_elem="tunnel_shell",
        margin=0.0,
        name="closed flap sits within the framed tunnel outline",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.0}):
        opened_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "flap swings inward from upper hinge",
        closed_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[0][0] < closed_flap_aabb[0][0] - 0.08,
        details=f"closed={closed_flap_aabb}, opened={opened_flap_aabb}",
    )

    closed_hood_aabb = ctx.part_element_world_aabb(hood, elem="hood_front_lip")
    with ctx.pose({hood_joint: 1.0}):
        opened_hood_aabb = ctx.part_element_world_aabb(hood, elem="hood_front_lip")
    ctx.check(
        "weather hood lifts away from exterior opening",
        closed_hood_aabb is not None
        and opened_hood_aabb is not None
        and opened_hood_aabb[1][2] > closed_hood_aabb[1][2] + 0.04,
        details=f"closed={closed_hood_aabb}, opened={opened_hood_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
