from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]):
    return model.material(name, rgba=rgba)


def _box_xz(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    thickness: float,
    *,
    material,
    name: str,
) -> None:
    """A rectangular beam whose long local-X axis lies between two XZ-plane points."""
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.hypot(dx, dz)
    angle_y = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, p0[1], (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _box_yz(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    thickness: float,
    *,
    material,
    name: str,
) -> None:
    """A rectangular beam whose long local-Y axis lies between two YZ-plane points."""
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.hypot(dy, dz)
    angle_x = math.atan2(dz, dy)
    part.visual(
        Box((thickness, length, thickness)),
        origin=Origin(
            xyz=(p0[0], (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(angle_x, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_lattice_mast(mast, *, height: float, material, dark_material) -> None:
    leg_offset = 0.46
    leg_size = 0.12
    bottom = 0.22
    top = height - 0.24
    leg_height = top - bottom

    mast.visual(
        Box((2.25, 2.25, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_material,
        name="base_slab",
    )
    mast.visual(
        Box((1.15, 1.15, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.45)),
        material=dark_material,
        name="top_deck",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            mast.visual(
                Box((leg_size, leg_size, leg_height)),
                origin=Origin(xyz=(sx * leg_offset, sy * leg_offset, bottom + leg_height * 0.5)),
                material=material,
                name=f"leg_{int(sx > 0)}_{int(sy > 0)}",
            )

    levels = [0.82, 1.82, 2.82, 3.82, 4.82, 5.82]
    for i, z in enumerate(levels):
        for sy in (-1.0, 1.0):
            mast.visual(
                Box((1.04, 0.075, 0.075)),
                origin=Origin(xyz=(0.0, sy * leg_offset, z)),
                material=material,
                name=f"x_ring_{i}_{int(sy > 0)}",
            )
        for sx in (-1.0, 1.0):
            mast.visual(
                Box((0.075, 1.04, 0.075)),
                origin=Origin(xyz=(sx * leg_offset, 0.0, z)),
                material=material,
                name=f"y_ring_{i}_{int(sx > 0)}",
            )

    bay_bottoms = [0.32, 1.32, 2.32, 3.32, 4.32]
    for i, z0 in enumerate(bay_bottoms):
        z1 = z0 + 1.00
        for sy in (-1.0, 1.0):
            if i % 2 == 0:
                p0 = (-leg_offset, sy * leg_offset, z0)
                p1 = (leg_offset, sy * leg_offset, z1)
            else:
                p0 = (leg_offset, sy * leg_offset, z0)
                p1 = (-leg_offset, sy * leg_offset, z1)
            _box_xz(mast, p0, p1, 0.055, material=material, name=f"side_diag_{i}_{int(sy > 0)}")
        for sx in (-1.0, 1.0):
            if i % 2 == 0:
                p0 = (sx * leg_offset, -leg_offset, z0)
                p1 = (sx * leg_offset, leg_offset, z1)
            else:
                p0 = (sx * leg_offset, leg_offset, z0)
                p1 = (sx * leg_offset, -leg_offset, z1)
            _box_yz(mast, p0, p1, 0.055, material=material, name=f"rear_diag_{i}_{int(sx > 0)}")

    # The exposed fork at the tower head receives the main boom's central hinge barrel.
    mast.visual(
        Box((0.40, 0.14, 0.58)),
        origin=Origin(xyz=(0.0, -0.30, height)),
        material=dark_material,
        name="top_yoke_0",
    )
    mast.visual(
        Cylinder(radius=0.095, length=0.13),
        origin=Origin(xyz=(0.0, -0.39, height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_material,
        name="top_bushing_0",
    )
    mast.visual(
        Box((0.32, 0.14, 0.24)),
        origin=Origin(xyz=(0.0, -0.30, height - 0.24)),
        material=dark_material,
        name="top_support_0",
    )
    mast.visual(
        Box((0.40, 0.14, 0.58)),
        origin=Origin(xyz=(0.0, 0.30, height)),
        material=dark_material,
        name="top_yoke_1",
    )
    mast.visual(
        Cylinder(radius=0.095, length=0.13),
        origin=Origin(xyz=(0.0, 0.39, height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_material,
        name="top_bushing_1",
    )
    mast.visual(
        Box((0.32, 0.14, 0.24)),
        origin=Origin(xyz=(0.0, 0.30, height - 0.24)),
        material=dark_material,
        name="top_support_1",
    )


def _add_truss_boom(
    part,
    *,
    length: float,
    width: float,
    height: float,
    start: float,
    end_clearance: float,
    material,
    dark_material,
    prefix: str,
) -> None:
    y = width * 0.5
    z = height * 0.5
    chord = 0.075
    end = length - end_clearance
    chord_len = end - start

    for sy in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            part.visual(
                Box((chord_len, chord, chord)),
                origin=Origin(xyz=(start + chord_len * 0.5, sy * y, sz * z)),
                material=material,
                name=f"{prefix}_chord_{int(sy > 0)}_{int(sz > 0)}",
            )

    bay_count = 6
    bay = chord_len / bay_count
    for i in range(bay_count + 1):
        x = start + i * bay
        for sz in (-1.0, 1.0):
            part.visual(
                Box((0.065, width + 0.08, 0.060)),
                origin=Origin(xyz=(x, 0.0, sz * z)),
                material=material,
                name=f"{prefix}_cross_{i}_{int(sz > 0)}",
            )

    for i in range(bay_count):
        x0 = start + i * bay
        x1 = start + (i + 1) * bay
        for sy in (-1.0, 1.0):
            if i % 2 == 0:
                p0 = (x0, sy * y, -z)
                p1 = (x1, sy * y, z)
            else:
                p0 = (x0, sy * y, z)
                p1 = (x1, sy * y, -z)
            _box_xz(
                part,
                p0,
                p1,
                0.050,
                material=dark_material,
                name=f"{prefix}_web_{i}_{int(sy > 0)}",
            )

    # Compact root web ties the hinge barrel to the four chords.
    part.visual(
        Box((0.46, 0.16, height + 0.06)),
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
        material=dark_material,
        name=f"{prefix}_root_web",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knuckle_boom_tower_crane")

    crane_yellow = _mat(model, "crane_yellow", (1.0, 0.73, 0.04, 1.0))
    dark_steel = _mat(model, "dark_steel", (0.08, 0.085, 0.09, 1.0))
    cable_black = _mat(model, "cable_black", (0.015, 0.015, 0.013, 1.0))
    warning_red = _mat(model, "warning_red", (0.78, 0.05, 0.03, 1.0))

    mast_height = 6.50
    main_len = 5.40
    secondary_len = 4.05

    mast = model.part("mast")
    _add_lattice_mast(mast, height=mast_height, material=crane_yellow, dark_material=dark_steel)

    main_boom = model.part("main_boom")
    main_boom.visual(
        Cylinder(radius=0.155, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="main_hinge_barrel",
    )
    main_boom.visual(
        Cylinder(radius=0.050, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warning_red,
        name="main_pin",
    )
    _add_truss_boom(
        main_boom,
        length=main_len,
        width=0.55,
        height=0.46,
        start=0.30,
        end_clearance=0.18,
        material=crane_yellow,
        dark_material=dark_steel,
        prefix="main",
    )
    main_boom.visual(
        Box((0.55, 0.10, 0.48)),
        origin=Origin(xyz=(main_len, -0.265, 0.0)),
        material=dark_steel,
        name="tip_fork_0",
    )
    main_boom.visual(
        Cylinder(radius=0.080, length=0.09),
        origin=Origin(xyz=(main_len, -0.265, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tip_bushing_0",
    )
    main_boom.visual(
        Box((0.55, 0.10, 0.48)),
        origin=Origin(xyz=(main_len, 0.265, 0.0)),
        material=dark_steel,
        name="tip_fork_1",
    )
    main_boom.visual(
        Cylinder(radius=0.080, length=0.09),
        origin=Origin(xyz=(main_len, 0.265, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tip_bushing_1",
    )

    secondary_boom = model.part("secondary_boom")
    secondary_boom.visual(
        Cylinder(radius=0.135, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knuckle_barrel",
    )
    secondary_boom.visual(
        Cylinder(radius=0.045, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warning_red,
        name="knuckle_pin",
    )
    _add_truss_boom(
        secondary_boom,
        length=secondary_len,
        width=0.43,
        height=0.36,
        start=0.44,
        end_clearance=0.06,
        material=crane_yellow,
        dark_material=dark_steel,
        prefix="secondary",
    )
    for sy, hanger_name in ((-1.0, "hook_hanger_0"), (1.0, "hook_hanger_1")):
        secondary_boom.visual(
            Box((0.26, 0.075, 0.42)),
            origin=Origin(xyz=(secondary_len, sy * 0.165, -0.10)),
            material=dark_steel,
            name=hanger_name,
        )
    secondary_boom.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(secondary_len, 0.0, -0.28)),
        material=dark_steel,
        name="swivel_socket",
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.060, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=dark_steel,
        name="swivel_stem",
    )
    hook_block.visual(
        Cylinder(radius=0.155, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material=dark_steel,
        name="swivel_cap",
    )
    hook_block.visual(
        Box((0.36, 0.26, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
        material=crane_yellow,
        name="sheave_block",
    )
    hook_block.visual(
        Cylinder(radius=0.125, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.49), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="sheave_wheel",
    )
    hook_block.visual(
        Cylinder(radius=0.038, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, -0.865)),
        material=dark_steel,
        name="hook_shank",
    )
    hook_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.98),
            (0.0, 0.0, -1.13),
            (0.07, 0.0, -1.30),
            (0.25, 0.0, -1.38),
            (0.39, 0.0, -1.25),
            (0.36, 0.0, -1.08),
            (0.22, 0.0, -1.02),
        ],
        radius=0.035,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    hook_block.visual(
        mesh_from_geometry(hook_geom, "load_hook"),
        material=dark_steel,
        name="load_hook",
    )
    hook_block.visual(
        Cylinder(radius=0.018, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
        material=cable_black,
        name="reeved_cable",
    )

    model.articulation(
        "mast_to_main_boom",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=main_boom,
        origin=Origin(xyz=(0.0, 0.0, mast_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.25, lower=-0.50, upper=0.82),
    )
    model.articulation(
        "main_to_secondary_boom",
        ArticulationType.REVOLUTE,
        parent=main_boom,
        child=secondary_boom,
        origin=Origin(xyz=(main_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.35, lower=-1.45, upper=1.15),
    )
    model.articulation(
        "secondary_to_hook_block",
        ArticulationType.REVOLUTE,
        parent=secondary_boom,
        child=hook_block,
        origin=Origin(xyz=(secondary_len, 0.0, -0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    main = object_model.get_part("main_boom")
    secondary = object_model.get_part("secondary_boom")
    hook = object_model.get_part("hook_block")
    main_joint = object_model.get_articulation("mast_to_main_boom")
    knuckle_joint = object_model.get_articulation("main_to_secondary_boom")
    swivel_joint = object_model.get_articulation("secondary_to_hook_block")

    ctx.check("single fixed mast root", object_model.root_parts() == [mast])
    ctx.check("main boom hinge axis is horizontal", tuple(main_joint.axis) == (0.0, -1.0, 0.0))
    ctx.check("knuckle hinge axis is horizontal", tuple(knuckle_joint.axis) == (0.0, -1.0, 0.0))
    ctx.check("hook swivel axis is vertical", tuple(swivel_joint.axis) == (0.0, 0.0, 1.0))

    ctx.allow_overlap(
        main,
        mast,
        elem_a="main_pin",
        elem_b="top_yoke_0",
        reason="The boom pivot pin is intentionally represented as captured through the mast-head yoke cheek.",
    )
    ctx.allow_overlap(
        main,
        mast,
        elem_a="main_pin",
        elem_b="top_yoke_1",
        reason="The boom pivot pin is intentionally represented as captured through the mast-head yoke cheek.",
    )
    ctx.allow_overlap(
        main,
        mast,
        elem_a="main_pin",
        elem_b="top_bushing_0",
        reason="The boom pivot pin intentionally runs through the mast-head bushing bore.",
    )
    ctx.allow_overlap(
        main,
        mast,
        elem_a="main_pin",
        elem_b="top_bushing_1",
        reason="The boom pivot pin intentionally runs through the mast-head bushing bore.",
    )
    ctx.allow_overlap(
        main,
        secondary,
        elem_a="tip_fork_0",
        elem_b="knuckle_pin",
        reason="The knuckle pin is intentionally represented as passing through the main-boom fork cheek.",
    )
    ctx.allow_overlap(
        main,
        secondary,
        elem_a="tip_fork_1",
        elem_b="knuckle_pin",
        reason="The knuckle pin is intentionally represented as passing through the main-boom fork cheek.",
    )
    ctx.allow_overlap(
        main,
        secondary,
        elem_a="tip_bushing_0",
        elem_b="knuckle_pin",
        reason="The knuckle pin intentionally runs through the main-boom tip bushing bore.",
    )
    ctx.allow_overlap(
        main,
        secondary,
        elem_a="tip_bushing_1",
        elem_b="knuckle_pin",
        reason="The knuckle pin intentionally runs through the main-boom tip bushing bore.",
    )
    ctx.allow_overlap(
        secondary,
        hook,
        elem_a="swivel_socket",
        elem_b="swivel_stem",
        reason="The hook swivel stem is intentionally captured inside the simplified boom-end bearing socket.",
    )

    ctx.expect_overlap(main, mast, axes="yz", min_overlap=0.25, elem_a="main_hinge_barrel", name="main hinge sits in mast yoke")
    for yoke_name in ("top_yoke_0", "top_yoke_1"):
        ctx.expect_overlap(
            main,
            mast,
            axes="yz",
            min_overlap=0.05,
            elem_a="main_pin",
            elem_b=yoke_name,
            name=f"main pin passes through {yoke_name}",
        )
    for bushing_name in ("top_bushing_0", "top_bushing_1"):
        ctx.expect_overlap(
            main,
            mast,
            axes="yz",
            min_overlap=0.05,
            elem_a="main_pin",
            elem_b=bushing_name,
            name=f"main pin is retained by {bushing_name}",
        )
    for fork_name in ("tip_fork_0", "tip_fork_1"):
        ctx.expect_overlap(
            main,
            secondary,
            axes="yz",
            min_overlap=0.04,
            elem_a=fork_name,
            elem_b="knuckle_pin",
            name=f"knuckle pin passes through {fork_name}",
        )
    for bushing_name in ("tip_bushing_0", "tip_bushing_1"):
        ctx.expect_overlap(
            main,
            secondary,
            axes="yz",
            min_overlap=0.04,
            elem_a=bushing_name,
            elem_b="knuckle_pin",
            name=f"knuckle pin is retained by {bushing_name}",
        )
    ctx.expect_overlap(
        secondary,
        main,
        axes="xz",
        min_overlap=0.20,
        elem_a="knuckle_barrel",
        elem_b="tip_fork_1",
        name="secondary knuckle sits inside main tip fork",
    )
    ctx.expect_within(
        hook,
        secondary,
        axes="xy",
        margin=0.0,
        inner_elem="swivel_stem",
        outer_elem="swivel_socket",
        name="hook swivel stem is centered in bearing socket",
    )
    ctx.expect_overlap(
        hook,
        secondary,
        axes="z",
        min_overlap=0.05,
        elem_a="swivel_stem",
        elem_b="swivel_socket",
        name="hook swivel stem remains captured vertically",
    )

    rest_main_aabb = ctx.part_world_aabb(main)
    with ctx.pose({main_joint: 0.65}):
        lifted_main_aabb = ctx.part_world_aabb(main)
    ctx.check(
        "main boom upper pose raises the arm",
        rest_main_aabb is not None
        and lifted_main_aabb is not None
        and lifted_main_aabb[1][2] > rest_main_aabb[1][2] + 1.6,
        details=f"rest={rest_main_aabb}, lifted={lifted_main_aabb}",
    )

    rest_hook_aabb = ctx.part_world_aabb(hook)
    with ctx.pose({knuckle_joint: -0.95}):
        folded_hook_aabb = ctx.part_world_aabb(hook)
    ctx.check(
        "secondary knuckle lower pose drops the hook",
        rest_hook_aabb is not None
        and folded_hook_aabb is not None
        and folded_hook_aabb[0][2] < rest_hook_aabb[0][2] - 1.2,
        details=f"rest={rest_hook_aabb}, folded={folded_hook_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
