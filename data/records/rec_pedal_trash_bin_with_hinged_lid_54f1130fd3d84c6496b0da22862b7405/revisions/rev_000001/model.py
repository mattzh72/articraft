from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _bridge(geom: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(a[i], b[j], a[j])
            geom.add_face(a[i], b[i], b[j])
        else:
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> int:
    c = geom.add_vertex(*center)
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(c, loop[j], loop[i])
        else:
            geom.add_face(c, loop[i], loop[j])
    return c


def _rounded_loop(
    depth: float,
    width: float,
    radius: float,
    z: float,
    *,
    x_offset: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_offset, y, z)
        for x, y in rounded_rect_profile(depth, width, radius, corner_segments=corner_segments)
    ]


def _make_hollow_bin_shell() -> MeshGeometry:
    """Thin-walled, open-topped waste-bin body with a slightly rolled rim."""
    depth = 0.270
    width = 0.180
    wall = 0.006
    bottom_thickness = 0.014
    body_h = 0.405
    rim_h = 0.020

    geom = MeshGeometry()
    loops: dict[str, list[int]] = {}

    outer_sections = [
        ("outer_bottom", depth * 0.90, width * 0.88, 0.020, 0.000),
        ("outer_shoulder", depth * 0.985, width * 0.975, 0.027, body_h),
        ("outer_rim", depth + 0.015, width + 0.015, 0.032, body_h + rim_h),
    ]
    inner_sections = [
        (
            "inner_floor",
            depth * 0.90 - 2.0 * wall,
            width * 0.88 - 2.0 * wall,
            0.015,
            bottom_thickness,
        ),
        ("inner_shoulder", depth - 0.030, width - 0.030, 0.022, body_h + 0.002),
        ("inner_rim", depth - 0.042, width - 0.042, 0.020, body_h + rim_h + 0.001),
    ]

    for name, d, w, r, z in outer_sections + inner_sections:
        loops[name] = _add_loop(geom, _rounded_loop(d, w, r, z))

    # Exterior and interior walls.
    _bridge(geom, loops["outer_bottom"], loops["outer_shoulder"])
    _bridge(geom, loops["outer_shoulder"], loops["outer_rim"])
    _bridge(geom, loops["inner_floor"], loops["inner_shoulder"], flip=True)
    _bridge(geom, loops["inner_shoulder"], loops["inner_rim"], flip=True)

    # Rolled top lip and closed bottom floor leave a real visible opening.
    _bridge(geom, loops["outer_rim"], loops["inner_rim"], flip=True)
    _bridge(geom, loops["outer_bottom"], loops["inner_floor"])
    _cap_loop(geom, loops["outer_bottom"], (0.0, 0.0, 0.0), flip=True)
    _cap_loop(geom, loops["inner_floor"], (0.0, 0.0, bottom_thickness), flip=False)
    return geom


def _make_lid_shell() -> MeshGeometry:
    """A hollow domed rectangular lid, authored in the rear hinge frame."""
    depth = 0.306
    width = 0.205
    thickness = 0.004
    rear_clearance = 0.006

    # The hinge axis is local x=0; the lid extends forward along +X.
    outer_specs = [
        (depth, width, 0.034, 0.000, rear_clearance + depth / 2.0),
        (depth * 0.97, width * 0.94, 0.034, 0.025, rear_clearance + depth / 2.0),
        (depth * 0.82, width * 0.73, 0.030, 0.050, rear_clearance + depth / 2.0),
        (depth * 0.38, width * 0.31, 0.025, 0.067, rear_clearance + depth / 2.0),
    ]
    inner_specs = [
        (d - 2.0 * thickness, w - 2.0 * thickness, max(0.010, r - thickness), z - thickness, xoff)
        for d, w, r, z, xoff in outer_specs
    ]

    geom = MeshGeometry()
    outer = [
        _add_loop(geom, _rounded_loop(d, w, r, z, x_offset=xoff, corner_segments=10))
        for d, w, r, z, xoff in outer_specs
    ]
    inner = [
        _add_loop(geom, _rounded_loop(d, w, r, z, x_offset=xoff, corner_segments=10))
        for d, w, r, z, xoff in inner_specs
    ]

    for a, b in zip(outer[:-1], outer[1:]):
        _bridge(geom, a, b)
    for a, b in zip(inner[:-1], inner[1:]):
        _bridge(geom, a, b, flip=True)

    # Bottom rolled edge and the small closed crest make the lid a thin hollow shell.
    _bridge(geom, outer[0], inner[0], flip=True)
    _bridge(geom, outer[-1], inner[-1])
    top_x = outer_specs[-1][4]
    _cap_loop(geom, outer[-1], (top_x, 0.0, outer_specs[-1][3]), flip=False)
    _cap_loop(geom, inner[-1], (top_x, 0.0, inner_specs[-1][3]), flip=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_office_pedal_bin")

    body_plastic = Material("satin charcoal plastic", color=(0.08, 0.10, 0.11, 1.0))
    lid_plastic = Material("warm gray lid plastic", color=(0.52, 0.55, 0.56, 1.0))
    dark_plastic = Material("black molded plastic", color=(0.015, 0.016, 0.017, 1.0))
    hinge_metal = Material("brushed hinge metal", color=(0.72, 0.70, 0.65, 1.0))
    rubber = Material("matte black rubber", color=(0.005, 0.005, 0.004, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_hollow_bin_shell(), "hollow_bin_body"),
        material=body_plastic,
        name="hollow_shell",
    )

    # Four low rubber feet are slightly inset into the floor so the bin reads as supported.
    for i, x in enumerate((-0.085, 0.085)):
        for j, y in enumerate((-0.052, 0.052)):
            body.visual(
                Box((0.034, 0.024, 0.010)),
                origin=Origin(xyz=(x, y, 0.002)),
                material=rubber,
                name=f"foot_{i}_{j}",
            )

    # Rear hinge barrels mounted to the rigid body, with gaps for the lid knuckle.
    hinge_x = -0.154
    hinge_z = 0.431
    for name, y in (("rear_hinge_barrel_0", -0.066), ("rear_hinge_barrel_1", 0.066)):
        body.visual(
            Cylinder(radius=0.0075, length=0.038),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=name,
        )
        body.visual(
            Box((0.020, 0.034, 0.006)),
            origin=Origin(xyz=(hinge_x + 0.008, y, hinge_z - 0.006)),
            material=hinge_metal,
            name=f"{name}_leaf",
        )

    # Lower front pivot lugs for the foot pedal, outside the pedal's central hub.
    pedal_pivot_x = 0.149
    pedal_pivot_z = 0.060
    for name, y in (("pedal_lug_0", -0.071), ("pedal_lug_1", 0.071)):
        body.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(xyz=(pedal_pivot_x, y, pedal_pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=name,
        )
        body.visual(
            Box((0.040, 0.020, 0.034)),
            origin=Origin(xyz=(0.132, y, pedal_pivot_z - 0.002)),
            material=body_plastic,
            name=f"{name}_rib",
        )

    # Two small side pivots for the folding carry handle on one side wall.
    handle_axis_y = 0.116
    handle_axis_z = 0.292
    for name, x in (("side_pivot_0", -0.074), ("side_pivot_1", 0.074)):
        body.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(xyz=(x, 0.097, handle_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=name,
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_make_lid_shell(), "domed_hollow_lid"),
        material=lid_plastic,
        name="domed_shell",
    )
    lid.visual(
        Cylinder(radius=0.0070, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.024, 0.062, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material=hinge_metal,
        name="hinge_leaf",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.0095, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="pivot_hub",
    )
    pedal.visual(
        Box((0.078, 0.026, 0.012)),
        origin=Origin(xyz=(0.039, 0.0, -0.008)),
        material=dark_plastic,
        name="pedal_arm",
    )
    pedal.visual(
        Box((0.105, 0.118, 0.012)),
        origin=Origin(xyz=(0.094, 0.0, -0.018)),
        material=dark_plastic,
        name="foot_pad",
    )
    pedal.visual(
        Box((0.086, 0.096, 0.003)),
        origin=Origin(xyz=(0.098, 0.0, -0.010)),
        material=rubber,
        name="rubber_tread",
    )

    handle_path = [
        (-0.074, 0.0, 0.0),
        (-0.082, 0.0, -0.040),
        (-0.074, 0.0, -0.126),
        (0.0, 0.0, -0.148),
        (0.074, 0.0, -0.126),
        (0.082, 0.0, -0.040),
        (0.074, 0.0, 0.0),
    ]
    handle = model.part("side_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                handle_path,
                radius=0.0042,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "folding_side_handle",
        ),
        material=hinge_metal,
        name="handle_tube",
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    pedal_pivot = model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(pedal_pivot_x, 0.0, pedal_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=0.45),
    )
    handle_pivot = model.articulation(
        "body_to_side_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, handle_axis_y, handle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    # Keep local names referenced so static linters do not treat the joints as accidental.
    _ = (lid_hinge, pedal_pivot, handle_pivot)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("side_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_pivot = object_model.get_articulation("body_to_pedal")
    handle_pivot = object_model.get_articulation("body_to_side_handle")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="domed_shell",
        negative_elem="hollow_shell",
        min_gap=0.001,
        max_gap=0.020,
        name="closed domed lid sits just above the rolled rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="domed_shell",
        elem_b="hollow_shell",
        min_overlap=0.12,
        name="lid footprint covers the narrow bin opening",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="y",
        positive_elem="handle_tube",
        negative_elem="hollow_shell",
        min_gap=0.004,
        max_gap=0.035,
        name="folded side handle is proud of the side wall",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="domed_shell")
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="domed_shell")
    ctx.check(
        "rear hinge opens the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045
        and open_lid_aabb[0][0] < closed_lid_aabb[0][0] - 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    with ctx.pose({pedal_pivot: 0.42}):
        depressed_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    ctx.check(
        "lower transverse pedal pivot depresses the foot pad",
        rest_pad is not None
        and depressed_pad is not None
        and depressed_pad[0][2] < rest_pad[0][2] - 0.020,
        details=f"rest={rest_pad}, depressed={depressed_pad}",
    )

    folded_handle = ctx.part_element_world_aabb(handle, elem="handle_tube")
    with ctx.pose({handle_pivot: 1.25}):
        raised_handle = ctx.part_element_world_aabb(handle, elem="handle_tube")
    ctx.check(
        "side carry handle folds outward on its two pivots",
        folded_handle is not None
        and raised_handle is not None
        and raised_handle[1][1] > folded_handle[1][1] + 0.070
        and raised_handle[0][2] > folded_handle[0][2] + 0.035,
        details=f"folded={folded_handle}, raised={raised_handle}",
    )

    return ctx.report()


object_model = build_object_model()
