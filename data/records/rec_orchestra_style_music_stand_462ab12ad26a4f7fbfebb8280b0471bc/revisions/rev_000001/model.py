from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rod_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("rod endpoints must be distinct")

    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _add_rod(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
) -> None:
    origin, length = _rod_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _tube_shell_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    black_metal = model.material("satin_black_powder_coat", rgba=(0.005, 0.006, 0.006, 1.0))
    worn_edge = model.material("slightly_worn_black_edges", rgba=(0.035, 0.038, 0.036, 1.0))
    rubber = model.material("matte_rubber_feet", rgba=(0.0, 0.0, 0.0, 1.0))

    lower_tube = model.part("lower_tube")
    lower_tube.visual(
        _tube_shell_mesh("lower_tube_shell_mesh", 0.022, 0.014, 0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=black_metal,
        name="lower_tube_shell",
    )
    lower_tube.visual(
        _tube_shell_mesh("top_clamp_collar_mesh", 0.031, 0.0145, 0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material=worn_edge,
        name="top_collar",
    )
    lower_tube.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=worn_edge,
        name="leg_hub",
    )
    # Three splayed tubular legs make the stand read as a free-standing orchestra
    # music stand while remaining one connected lower support-tube assembly.
    for i, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        hub = (0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.145)
        foot = (0.340 * math.cos(angle), 0.340 * math.sin(angle), 0.025)
        _add_rod(lower_tube, hub, foot, 0.008, material=black_metal, name=f"tripod_leg_{i}")
        lower_tube.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=foot),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.011, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black_metal,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        material=worn_edge,
        name="lower_glide_bushing",
    )
    mast.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=worn_edge,
        name="head_collar",
    )
    mast.visual(
        Box((0.042, 0.064, 0.018)),
        origin=Origin(xyz=(0.0, -0.022, 0.474)),
        material=black_metal,
        name="head_bracket",
    )
    mast.visual(
        Box((0.034, 0.046, 0.030)),
        origin=Origin(xyz=(-0.081, -0.035, 0.495)),
        material=black_metal,
        name="hinge_yoke_0",
    )
    mast.visual(
        Box((0.034, 0.046, 0.030)),
        origin=Origin(xyz=(0.081, -0.035, 0.495)),
        material=black_metal,
        name="hinge_yoke_1",
    )
    _add_rod(
        mast,
        (-0.065, -0.035, 0.475),
        (0.065, -0.035, 0.475),
        0.006,
        material=black_metal,
        name="yoke_bridge",
    )

    desk = model.part("desk")
    # Hinge barrel captured between the two yoke ears at the mast head.
    _add_rod(desk, (-0.065, 0.0, 0.0), (0.065, 0.0, 0.0), 0.008, material=worn_edge, name="desk_hinge_barrel")
    # Main wire-frame music desk: perimeter rails, vertical wires, and fan braces.
    _add_rod(desk, (-0.285, 0.0, 0.050), (0.285, 0.0, 0.050), 0.0042, material=black_metal, name="bottom_rail")
    _add_rod(desk, (-0.285, 0.0, 0.390), (0.285, 0.0, 0.390), 0.0042, material=black_metal, name="top_rail")
    _add_rod(desk, (-0.285, 0.0, 0.050), (-0.285, 0.0, 0.390), 0.0042, material=black_metal, name="side_rail_0")
    _add_rod(desk, (0.285, 0.0, 0.050), (0.285, 0.0, 0.390), 0.0042, material=black_metal, name="side_rail_1")
    for i, x in enumerate((-0.190, -0.095, 0.0, 0.095, 0.190)):
        _add_rod(desk, (x, 0.0, 0.050), (x, 0.0, 0.390), 0.0026, material=black_metal, name=f"vertical_wire_{i}")
    for i, end in enumerate(((-0.285, 0.0, 0.390), (-0.142, 0.0, 0.390), (0.142, 0.0, 0.390), (0.285, 0.0, 0.390))):
        _add_rod(desk, (0.0, 0.0, 0.000), end, 0.0024, material=black_metal, name=f"fan_wire_{i}")
    _add_rod(desk, (-0.220, 0.0, 0.215), (0.220, 0.0, 0.215), 0.0024, material=black_metal, name="middle_wire")
    # Shelf hinge mounts hang just beneath the desk's bottom rail and remain part
    # of the desk frame; the shelf's central barrel fits between them.
    for x, suffix in ((-0.255, "0"), (0.255, "1")):
        _add_rod(desk, (x, 0.0, 0.050), (x, -0.018, 0.020), 0.0030, material=black_metal, name=f"shelf_hanger_{suffix}")
        _add_rod(desk, (x - 0.018, -0.018, 0.020), (x + 0.018, -0.018, 0.020), 0.0055, material=worn_edge, name=f"shelf_hinge_knuckle_{suffix}")

    shelf = model.part("shelf")
    _add_rod(shelf, (-0.237, 0.0, 0.0), (0.237, 0.0, 0.0), 0.0050, material=worn_edge, name="shelf_hinge_barrel")
    _add_rod(shelf, (-0.245, -0.085, 0.0), (0.245, -0.085, 0.0), 0.0050, material=black_metal, name="front_lip")
    _add_rod(shelf, (-0.215, 0.0, 0.0), (-0.245, -0.085, 0.0), 0.0035, material=black_metal, name="shelf_arm_0")
    _add_rod(shelf, (0.215, 0.0, 0.0), (0.245, -0.085, 0.0), 0.0035, material=black_metal, name="shelf_arm_1")
    _add_rod(shelf, (-0.245, -0.085, 0.0), (-0.245, -0.085, 0.040), 0.0030, material=black_metal, name="lip_post_0")
    _add_rod(shelf, (0.245, -0.085, 0.0), (0.245, -0.085, 0.040), 0.0030, material=black_metal, name="lip_post_1")
    _add_rod(shelf, (-0.245, -0.085, 0.040), (0.245, -0.085, 0.040), 0.0030, material=black_metal, name="raised_lip")

    model.articulation(
        "tube_to_mast",
        ArticulationType.PRISMATIC,
        parent=lower_tube,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.32),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, -0.035, 0.500), rpy=(math.radians(-12.0), 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.35, upper=0.75),
    )
    model.articulation(
        "desk_to_shelf",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=shelf,
        origin=Origin(xyz=(0.0, -0.018, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_tube = object_model.get_part("lower_tube")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    shelf = object_model.get_part("shelf")
    tube_to_mast = object_model.get_articulation("tube_to_mast")
    mast_to_desk = object_model.get_articulation("mast_to_desk")
    desk_to_shelf = object_model.get_articulation("desk_to_shelf")

    ctx.allow_overlap(
        lower_tube,
        mast,
        elem_a="lower_tube_shell",
        elem_b="lower_glide_bushing",
        reason=(
            "The hidden low-friction bushing is intentionally seated against the "
            "inside of the telescoping lower support tube to keep the sliding mast captured."
        ),
    )

    ctx.check(
        "primary mechanisms are articulated",
        tube_to_mast.articulation_type == ArticulationType.PRISMATIC
        and mast_to_desk.articulation_type == ArticulationType.REVOLUTE
        and desk_to_shelf.articulation_type == ArticulationType.REVOLUTE,
    )

    ctx.expect_within(
        mast,
        lower_tube,
        axes="xy",
        inner_elem="mast_tube",
        outer_elem="lower_tube_shell",
        margin=0.001,
        name="sliding mast is centered in lower tube",
    )
    ctx.expect_within(
        mast,
        lower_tube,
        axes="xy",
        inner_elem="lower_glide_bushing",
        outer_elem="lower_tube_shell",
        margin=0.001,
        name="glide bushing is captured inside lower tube",
    )
    ctx.expect_overlap(
        mast,
        lower_tube,
        axes="z",
        elem_a="lower_glide_bushing",
        elem_b="lower_tube_shell",
        min_overlap=0.02,
        name="glide bushing remains inside sleeve at rest",
    )
    ctx.expect_overlap(
        mast,
        lower_tube,
        axes="z",
        elem_a="mast_tube",
        elem_b="lower_tube_shell",
        min_overlap=0.35,
        name="collapsed mast remains deeply inserted",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({tube_to_mast: 0.32}):
        extended_mast_pos = ctx.part_world_position(mast)
        ctx.expect_within(
            mast,
            lower_tube,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="lower_tube_shell",
            margin=0.001,
            name="extended mast remains centered",
        )
        ctx.expect_overlap(
            mast,
            lower_tube,
            axes="z",
            elem_a="mast_tube",
            elem_b="lower_tube_shell",
            min_overlap=0.08,
            name="extended mast retains insertion",
        )
    ctx.check(
        "mast slides upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.30,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    def _element_center(part, elem_name: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem_name)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({mast_to_desk: -0.30}):
        low_top = _element_center(desk, "top_rail")
    with ctx.pose({mast_to_desk: 0.65}):
        high_top = _element_center(desk, "top_rail")
    ctx.check(
        "desk rotates on mast-head hinge",
        low_top is not None
        and high_top is not None
        and abs(high_top[1] - low_top[1]) > 0.18,
        details=f"low_top={low_top}, high_top={high_top}",
    )

    open_lip = _element_center(shelf, "front_lip")
    with ctx.pose({desk_to_shelf: 1.25}):
        folded_lip = _element_center(shelf, "front_lip")
    ctx.check(
        "lower shelf folds upward on its hinge",
        open_lip is not None and folded_lip is not None and folded_lip[2] > open_lip[2] + 0.06,
        details=f"open_lip={open_lip}, folded_lip={folded_lip}",
    )

    return ctx.report()


object_model = build_object_model()
