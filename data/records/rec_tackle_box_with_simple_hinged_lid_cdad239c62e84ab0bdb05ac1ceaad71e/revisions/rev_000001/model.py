from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_loop(
    geom: MeshGeometry,
    profile: list[tuple[float, float]],
    z: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[int]:
    return [geom.add_vertex(x + dx, y + dy, z) for x, y in profile]


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _bridge(geom: MeshGeometry, lower: list[int], upper: list[int], *, reverse: bool = False) -> None:
    count = len(lower)
    for index in range(count):
        nxt = (index + 1) % count
        if reverse:
            _quad(geom, lower[index], upper[index], upper[nxt], lower[nxt])
        else:
            _quad(geom, lower[index], lower[nxt], upper[nxt], upper[index])


def _cap_fan(geom: MeshGeometry, loop: list[int], points: list[tuple[float, float]], z: float, *, reverse: bool = False) -> None:
    cx = sum(x for x, _ in points) / len(points)
    cy = sum(y for _, y in points) / len(points)
    center = geom.add_vertex(cx, cy, z)
    count = len(loop)
    for index in range(count):
        nxt = (index + 1) % count
        if reverse:
            geom.add_face(center, loop[nxt], loop[index])
        else:
            geom.add_face(center, loop[index], loop[nxt])


def _solid_rounded_plate(width: float, depth: float, radius: float, z0: float, z1: float, *, y_center: float = 0.0) -> MeshGeometry:
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    shifted = [(x, y + y_center) for x, y in profile]
    geom = MeshGeometry()
    bottom = _add_loop(geom, shifted, z0)
    top = _add_loop(geom, shifted, z1)
    _bridge(geom, bottom, top)
    _cap_fan(geom, bottom, shifted, z0, reverse=True)
    _cap_fan(geom, top, shifted, z1)
    return geom


def _rounded_ring(width: float, depth: float, wall: float, radius: float, z0: float, z1: float, *, y_center: float = 0.0) -> MeshGeometry:
    outer = rounded_rect_profile(width, depth, radius, corner_segments=8)
    inner = rounded_rect_profile(width - 2.0 * wall, depth - 2.0 * wall, max(radius - wall, 0.002), corner_segments=8)
    outer = [(x, y + y_center) for x, y in outer]
    inner = [(x, y + y_center) for x, y in inner]

    geom = MeshGeometry()
    outer_bottom = _add_loop(geom, outer, z0)
    outer_top = _add_loop(geom, outer, z1)
    inner_bottom = _add_loop(geom, inner, z0)
    inner_top = _add_loop(geom, inner, z1)

    _bridge(geom, outer_bottom, outer_top)
    _bridge(geom, inner_bottom, inner_top, reverse=True)
    for index in range(len(outer_top)):
        nxt = (index + 1) % len(outer_top)
        _quad(geom, outer_top[index], outer_top[nxt], inner_top[nxt], inner_top[index])
        _quad(geom, inner_bottom[index], inner_bottom[nxt], outer_bottom[nxt], outer_bottom[index])
    return geom


def _open_tray_shell(width: float, depth: float, height: float, wall: float, floor: float, radius: float) -> MeshGeometry:
    outer = rounded_rect_profile(width, depth, radius, corner_segments=10)
    inner = rounded_rect_profile(width - 2.0 * wall, depth - 2.0 * wall, max(radius - wall, 0.003), corner_segments=10)
    geom = MeshGeometry()

    outer_bottom = _add_loop(geom, outer, 0.0)
    outer_top = _add_loop(geom, outer, height)
    inner_floor = _add_loop(geom, inner, floor)
    inner_top = _add_loop(geom, inner, height)

    _bridge(geom, outer_bottom, outer_top)
    _bridge(geom, inner_floor, inner_top, reverse=True)
    for index in range(len(outer_top)):
        nxt = (index + 1) % len(outer_top)
        _quad(geom, outer_top[index], outer_top[nxt], inner_top[nxt], inner_top[index])
    _cap_fan(geom, outer_bottom, outer, 0.0, reverse=True)
    _cap_fan(geom, inner_floor, inner, floor)
    return geom


def _knuckle_mesh(segments: tuple[tuple[float, float], ...], radius: float) -> MeshGeometry:
    geom = MeshGeometry()
    for start, end in segments:
        geom.merge(
            CylinderGeometry(radius, end - start, radial_segments=32)
            .rotate_y(pi / 2.0)
            .translate((start + end) * 0.5, 0.0, 0.0)
        )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_hinged_tackle_box")

    matte_graphite = model.material("matte_graphite", rgba=(0.12, 0.14, 0.15, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.20, 0.23, 0.24, 1.0))
    inner_graphite = model.material("inner_graphite", rgba=(0.16, 0.18, 0.19, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.016, 0.017, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.66, 1.0))
    satin_titanium = model.material("satin_titanium", rgba=(0.42, 0.44, 0.45, 1.0))
    warm_accent = model.material("warm_accent", rgba=(0.72, 0.53, 0.25, 1.0))

    width = 0.420
    depth = 0.240
    body_height = 0.160
    wall = 0.012
    floor = 0.012

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_open_tray_shell(width, depth, body_height, wall, floor, 0.030), "body_shell"),
        material=matte_graphite,
        name="body_shell",
    )
    base.visual(
        mesh_from_geometry(_rounded_ring(width - 0.010, depth - 0.010, 0.006, 0.024, body_height - 0.0015, body_height + 0.0005), "top_gasket"),
        material=rubber_black,
        name="top_gasket",
    )
    base.visual(
        Box((0.390, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, -0.034, 0.038)),
        material=inner_graphite,
        name="front_divider",
    )
    base.visual(
        Box((0.390, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.040, 0.038)),
        material=inner_graphite,
        name="rear_divider",
    )
    base.visual(
        Box((0.006, 0.188, 0.052)),
        origin=Origin(xyz=(-0.070, 0.002, 0.038)),
        material=inner_graphite,
        name="divider_0",
    )
    base.visual(
        Box((0.006, 0.188, 0.052)),
        origin=Origin(xyz=(0.070, 0.002, 0.038)),
        material=inner_graphite,
        name="divider_1",
    )
    base.visual(
        Box((0.105, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.003, 0.094)),
        material=satin_titanium,
        name="keeper_plate",
    )
    base.visual(
        Box((0.080, 0.009, 0.014)),
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.012, 0.072)),
        material=brushed_steel,
        name="keeper_bar",
    )
    for screw_x in (-0.038, 0.038):
        base.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(screw_x, -depth * 0.5 - 0.008, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"keeper_screw_{0 if screw_x < 0 else 1}",
        )
    base.visual(
        Box((0.392, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.009, 0.146)),
        material=satin_titanium,
        name="rear_hinge_leaf",
    )
    base.visual(
        mesh_from_geometry(
            _knuckle_mesh(((-0.205, -0.135), (-0.035, 0.035), (0.135, 0.205)), 0.010),
            "base_hinge_knuckles",
        ),
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.006, body_height + 0.010)),
        material=brushed_steel,
        name="base_hinge_knuckles",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.440),
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.006, body_height + 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="rear_hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_solid_rounded_plate(0.435, 0.245, 0.034, -0.007, 0.019, y_center=-0.134), "lid_panel"),
        material=satin_charcoal,
        name="lid_panel",
    )
    lid.visual(
        mesh_from_geometry(_rounded_ring(0.372, 0.192, 0.006, 0.020, -0.021, -0.006, y_center=-0.128), "inner_lip"),
        material=rubber_black,
        name="inner_lip",
    )
    lid.visual(
        Box((0.278, 0.074, 0.003)),
        origin=Origin(xyz=(0.0, -0.128, 0.0205)),
        material=rubber_black,
        name="grip_recess",
    )
    lid.visual(
        Box((0.292, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.083, 0.022)),
        material=warm_accent,
        name="grip_rear_trim",
    )
    lid.visual(
        Box((0.292, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.173, 0.022)),
        material=warm_accent,
        name="grip_front_trim",
    )
    lid.visual(
        Box((0.008, 0.082, 0.006)),
        origin=Origin(xyz=(-0.150, -0.128, 0.022)),
        material=warm_accent,
        name="grip_trim_0",
    )
    lid.visual(
        Box((0.008, 0.082, 0.006)),
        origin=Origin(xyz=(0.150, -0.128, 0.022)),
        material=warm_accent,
        name="grip_trim_1",
    )
    lid.visual(
        Box((0.392, 0.015, 0.004)),
        origin=Origin(xyz=(0.0, -0.013, -0.007)),
        material=satin_titanium,
        name="lid_hinge_leaf",
    )
    lid.visual(
        mesh_from_geometry(_knuckle_mesh(((-0.128, -0.045), (0.045, 0.128)), 0.010), "rear_hinge_knuckles"),
        origin=Origin(),
        material=brushed_steel,
        name="rear_hinge_knuckles",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.090),
        origin=Origin(xyz=(0.0, -0.265, -0.0015), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="latch_pivot_pin",
    )
    lid.visual(
        Box((0.115, 0.016, 0.032)),
        origin=Origin(xyz=(0.0, -0.257, -0.023)),
        material=satin_titanium,
        name="latch_mount_plate",
    )
    lid.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(-0.041, -0.265, -0.008)),
        material=satin_titanium,
        name="latch_ear_0",
    )
    lid.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.041, -0.265, -0.008)),
        material=satin_titanium,
        name="latch_ear_1",
    )
    for screw_x in (-0.043, 0.043):
        lid.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(screw_x, -0.263, -0.023), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"latch_screw_{0 if screw_x < 0 else 1}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="latch_pivot_barrel",
    )
    latch.visual(
        Box((0.058, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -0.006, -0.043)),
        material=satin_titanium,
        name="latch_pull_plate",
    )
    latch.visual(
        Box((0.076, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, -0.087)),
        material=brushed_steel,
        name="latch_hook_lip",
    )
    latch.visual(
        Box((0.044, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, -0.014, -0.042)),
        material=rubber_black,
        name="finger_pad",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.006, body_height + 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.5, velocity=2.2, lower=0.0, upper=1.28),
    )
    latch_hinge = model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, -0.265, -0.0015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=0.75),
    )

    # Keep references intentionally used by tests discoverable in the model tree.
    lid_hinge.meta["function"] = "single rear hinge line for the top lid"
    latch_hinge.meta["function"] = "small snap latch swings away from the keeper"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_hinge = object_model.get_articulation("latch_hinge")

    ctx.allow_overlap(
        base,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="rear_hinge_knuckles",
        reason="The satin hinge pin is intentionally captured inside the lid-side hinge knuckles.",
    )
    ctx.allow_overlap(
        lid,
        latch,
        elem_a="latch_pivot_pin",
        elem_b="latch_pivot_barrel",
        reason="The latch barrel rotates around the small pin carried by the lid latch mount.",
    )

    ctx.expect_within(
        base,
        lid,
        axes="yz",
        inner_elem="rear_hinge_pin",
        outer_elem="rear_hinge_knuckles",
        margin=0.001,
        name="hinge pin sits inside lid knuckles",
    )
    ctx.expect_overlap(
        base,
        lid,
        axes="x",
        elem_a="rear_hinge_pin",
        elem_b="rear_hinge_knuckles",
        min_overlap=0.16,
        name="hinge knuckles retain pin length",
    )
    ctx.expect_within(
        lid,
        latch,
        axes="yz",
        inner_elem="latch_pivot_pin",
        outer_elem="latch_pivot_barrel",
        margin=0.0015,
        name="latch barrel surrounds pivot pin",
    )
    ctx.expect_overlap(
        lid,
        latch,
        axes="x",
        elem_a="latch_pivot_pin",
        elem_b="latch_pivot_barrel",
        min_overlap=0.050,
        name="latch pivot pin spans the barrel",
    )

    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_gasket",
            min_gap=0.001,
            max_gap=0.006,
            name="closed lid has a tight gasket seam",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.22,
            name="lid footprint covers the compact box shell",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)
        closed_latch_aabb = ctx.part_world_aabb(latch)

    with ctx.pose({lid_hinge: 1.18, latch_hinge: 0.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.68}):
        swung_latch_aabb = ctx.part_world_aabb(latch)

    ctx.check(
        "lid opens upward around rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.13,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "latch swings forward off keeper",
        closed_latch_aabb is not None
        and swung_latch_aabb is not None
        and swung_latch_aabb[0][1] < closed_latch_aabb[0][1] - 0.025,
        details=f"closed={closed_latch_aabb}, swung={swung_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
