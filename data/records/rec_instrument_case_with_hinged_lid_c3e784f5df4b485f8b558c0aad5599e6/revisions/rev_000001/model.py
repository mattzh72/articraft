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
)


CASE_LENGTH = 0.88
CASE_HALF_LENGTH = CASE_LENGTH / 2.0
HINGE_Y = 0.185
HINGE_Z = 0.105


def _outline_loop(
    *,
    inset: float = 0.0,
    z: float = 0.0,
    samples: int = 96,
    local_from_hinge: bool = False,
) -> list[tuple[float, float, float]]:
    """Smooth sax-case footprint: bell-end bulge, narrower neck end, asymmetric sides."""

    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = 2.0 * math.pi * i / samples
        c = math.cos(t)
        s = math.sin(t)
        # Long axis is the hinge axis.  The bell end is the negative-X end.
        x = (CASE_HALF_LENGTH - inset) * c
        bell_bulge = math.exp(-((x + 0.28) / 0.16) ** 2)
        body_bulge = math.exp(-((x + 0.02) / 0.34) ** 2)
        neck_taper = math.exp(-((x - 0.28) / 0.18) ** 2)
        half_width = 0.070 + 0.095 * bell_bulge + 0.030 * body_bulge - 0.010 * neck_taper
        half_width = max(0.050, half_width - inset)
        # Front side hangs slightly fuller, so the case is not a simple oval.
        side_factor = 1.08 if s < 0.0 else 0.94
        y_center = 0.010 * bell_bulge - 0.006 * neck_taper
        y = y_center + s * half_width * side_factor
        if local_from_hinge:
            pts.append((x, y - HINGE_Y, z - HINGE_Z))
        else:
            pts.append((x, y, z))
    return pts


def _deep_lower_bottom_z(x: float) -> float:
    """Lower shell is deeper under the sax bell end."""

    bell_depth = math.exp(-((x + 0.30) / 0.20) ** 2)
    body_depth = math.exp(-((x + 0.02) / 0.44) ** 2)
    neck_relief = math.exp(-((x - 0.34) / 0.18) ** 2)
    return -(0.060 + 0.058 * bell_depth + 0.018 * body_depth - 0.004 * neck_relief)


def _build_lower_shell_geometry() -> MeshGeometry:
    """Open-top molded tray: tall/deep bell end, shallower neck end."""

    geom = MeshGeometry()
    top = _outline_loop(inset=0.000, z=0.090)
    chine = _outline_loop(inset=0.012, z=0.020)
    bottom_xy = _outline_loop(inset=0.030, z=0.0)
    bottom = [(x, y, _deep_lower_bottom_z(x)) for x, y, _ in bottom_xy]
    sections = [top, chine, bottom]
    section_indices: list[list[int]] = []

    for section in sections:
        section_indices.append([geom.add_vertex(x, y, z) for x, y, z in section])

    n = len(top)
    for a, b in zip(section_indices[:-1], section_indices[1:]):
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])

    # Curved bottom cap; a fan is sufficient because the footprint remains star-shaped.
    cx = sum(p[0] for p in bottom) / n
    cy = sum(p[1] for p in bottom) / n
    cz = min(p[2] for p in bottom) + 0.004
    center = geom.add_vertex(cx, cy, cz)
    btm = section_indices[-1]
    for i in range(n):
        geom.add_face(center, btm[i], btm[(i + 1) % n])

    return geom


def _build_lid_geometry() -> MeshGeometry:
    """Hollow fitted lid, authored in the lid frame whose origin is the hinge axis."""

    geom = MeshGeometry()
    rim = _outline_loop(inset=0.006, z=0.097, local_from_hinge=True)
    shoulder = _outline_loop(inset=0.020, z=0.133, local_from_hinge=True)
    crown_base = _outline_loop(inset=0.040, z=0.157, local_from_hinge=True)
    crown: list[tuple[float, float, float]] = []
    for x, y, z in crown_base:
        world_y = y + HINGE_Y
        radial = min(1.0, (abs(x) / 0.42) ** 2 + (abs(world_y) / 0.17) ** 2)
        crown.append((x, y, z + 0.014 * (1.0 - radial)))

    sections = [rim, shoulder, crown]
    section_indices: list[list[int]] = []
    for section in sections:
        section_indices.append([geom.add_vertex(x, y, z) for x, y, z in section])

    n = len(rim)
    for a, b in zip(section_indices[:-1], section_indices[1:]):
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])

    # Top cap, leaving the underside open like a real molded lid.
    cx = sum(p[0] for p in crown) / n
    cy = sum(p[1] for p in crown) / n
    cz = max(p[2] for p in crown) + 0.002
    center = geom.add_vertex(cx, cy, cz)
    top = section_indices[-1]
    for i in range(n):
        geom.add_face(center, top[(i + 1) % n], top[i])

    return geom


def _build_gasket_ring_geometry() -> MeshGeometry:
    """Thin rubber seam strip around the tray mouth."""

    geom = MeshGeometry()
    outer_lo = _outline_loop(inset=0.002, z=0.088)
    outer_hi = _outline_loop(inset=0.002, z=0.095)
    inner_lo = _outline_loop(inset=0.022, z=0.088)
    inner_hi = _outline_loop(inset=0.022, z=0.095)
    loops = [outer_lo, outer_hi, inner_hi, inner_lo]
    idx = [[geom.add_vertex(x, y, z) for x, y, z in loop] for loop in loops]
    n = len(outer_lo)

    # Outer wall, top face, inner wall, bottom face of the ring.
    pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
    for a_i, b_i in pairs:
        a = idx[a_i]
        b = idx[b_i]
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="asymmetrical_saxophone_case")

    shell_plastic = model.material("black_molded_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    lid_plastic = model.material("satin_black_lid", rgba=(0.025, 0.027, 0.030, 1.0))
    rubber = model.material("dark_rubber_gasket", rgba=(0.003, 0.003, 0.003, 1.0))
    metal = model.material("brushed_chrome_hardware", rgba=(0.72, 0.70, 0.64, 1.0))
    rib_mat = model.material("subtle_molded_ridges", rgba=(0.040, 0.042, 0.045, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_geometry(_build_lower_shell_geometry(), "asym_lower_shell"),
        material=shell_plastic,
        name="asym_lower_shell",
    )
    lower.visual(
        mesh_from_geometry(_build_gasket_ring_geometry(), "perimeter_gasket"),
        material=rubber,
        name="perimeter_gasket",
    )
    lower.visual(
        Box((0.66, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.160, 0.082)),
        material=shell_plastic,
        name="rear_hinge_flange",
    )
    lower.visual(
        Box((0.64, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, HINGE_Y + 0.002, HINGE_Z - 0.019)),
        material=metal,
        name="lower_hinge_leaf",
    )
    lower.visual(
        Cylinder(radius=0.009, length=0.68),
        origin=Origin(xyz=(0.0, HINGE_Y + 0.006, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_hinge_pin",
    )
    lower.visual(
        Box((0.085, 0.006, 0.040)),
        origin=Origin(xyz=(0.030, -0.158, 0.070)),
        material=metal,
        name="latch_backplate",
    )
    lower.visual(
        Box((0.145, 0.060, 0.044)),
        origin=Origin(xyz=(0.030, -0.130, 0.067)),
        material=shell_plastic,
        name="front_latch_boss",
    )
    lower.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(-0.026, -0.166, 0.086)),
        material=metal,
        name="latch_lug_0",
    )
    lower.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.086, -0.166, 0.086)),
        material=metal,
        name="latch_lug_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_geometry(), "fitted_lid_shell"),
        material=lid_plastic,
        name="fitted_lid_shell",
    )
    # Raised molded ribs make the lid read as a molded case rather than a flat cap.
    lid.visual(
        Cylinder(radius=0.088, length=0.007),
        origin=Origin(xyz=(-0.275, -0.176, 0.060)),
        material=rib_mat,
        name="bell_molded_rib",
    )
    lid.visual(
        Box((0.56, 0.018, 0.007)),
        origin=Origin(xyz=(0.065, -0.260, 0.056)),
        material=rib_mat,
        name="front_molded_rib",
    )
    lid.visual(
        Box((0.45, 0.014, 0.006)),
        origin=Origin(xyz=(0.030, -0.110, 0.058)),
        material=rib_mat,
        name="rear_molded_rib",
    )
    lid.visual(
        Box((0.64, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, -0.010)),
        material=metal,
        name="upper_hinge_leaf",
    )
    lid.visual(
        Box((0.65, 0.046, 0.010)),
        origin=Origin(xyz=(0.0, -0.027, 0.000)),
        material=lid_plastic,
        name="upper_hinge_flange",
    )
    lid.visual(
        Box((0.074, 0.010, 0.018)),
        origin=Origin(xyz=(0.030, -0.352, 0.030)),
        material=metal,
        name="latch_keeper",
    )
    lid.visual(
        Box((0.094, 0.080, 0.030)),
        origin=Origin(xyz=(0.030, -0.310, 0.035)),
        material=lid_plastic,
        name="keeper_mount",
    )

    latch = model.part("latch_lever")
    latch.visual(
        Box((0.076, 0.007, 0.072)),
        origin=Origin(xyz=(0.0, -0.007, -0.026)),
        material=metal,
        name="flip_lever",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.098),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="lever_pivot_bar",
    )
    latch.visual(
        Box((0.050, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=metal,
        name="keeper_hook",
    )

    model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lower_to_latch",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=latch,
        origin=Origin(xyz=(0.030, -0.168, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_lever")
    lid_hinge = object_model.get_articulation("lower_to_lid")
    latch_pivot = object_model.get_articulation("lower_to_latch")

    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="fitted_lid_shell",
        elem_b="asym_lower_shell",
        min_overlap=0.20,
        name="lid follows the molded lower outline",
    )
    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="fitted_lid_shell",
        negative_elem="perimeter_gasket",
        min_gap=0.001,
        max_gap=0.008,
        name="closed lid sits just above the gasket seam",
    )

    lower_box = ctx.part_world_aabb(lower)
    ctx.check(
        "lower shell has saxophone-case proportions",
        lower_box is not None
        and (lower_box[1][0] - lower_box[0][0]) > 0.80
        and (lower_box[1][1] - lower_box[0][1]) > 0.25
        and (lower_box[1][2] - lower_box[0][2]) > 0.18,
        details=f"lower aabb={lower_box}",
    )

    closed_lid_box = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_box = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about the rear longitudinal hinge",
        closed_lid_box is not None
        and open_lid_box is not None
        and open_lid_box[1][2] > closed_lid_box[1][2] + 0.18,
        details=f"closed={closed_lid_box}, open={open_lid_box}",
    )

    closed_latch_box = ctx.part_world_aabb(latch)
    with ctx.pose({latch_pivot: 0.8}):
        flipped_latch_box = ctx.part_world_aabb(latch)
    ctx.check(
        "front latch lever flips outward on its own pivot",
        closed_latch_box is not None
        and flipped_latch_box is not None
        and flipped_latch_box[0][1] < closed_latch_box[0][1] - 0.012,
        details=f"closed={closed_latch_box}, flipped={flipped_latch_box}",
    )

    return ctx.report()


object_model = build_object_model()
