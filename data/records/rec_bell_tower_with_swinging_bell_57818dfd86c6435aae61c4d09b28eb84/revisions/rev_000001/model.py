from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _bar_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return a transform for a box whose local +X axis spans start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("bar endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / length)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_square_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    tube: float,
    *,
    material,
    name: str,
) -> None:
    origin, length = _bar_origin(start, end)
    part.visual(
        Box((length, tube, tube)),
        origin=origin,
        material=material,
        name=name,
    )


def _bell_shell_mesh():
    outer_profile = [
        (0.070, -0.205),
        (0.125, -0.245),
        (0.175, -0.330),
        (0.230, -0.500),
        (0.310, -0.675),
        (0.420, -0.835),
        (0.480, -0.925),
        (0.495, -0.955),
    ]
    inner_profile = [
        (0.035, -0.250),
        (0.095, -0.305),
        (0.145, -0.410),
        (0.205, -0.565),
        (0.285, -0.720),
        (0.390, -0.865),
        (0.445, -0.925),
        (0.455, -0.945),
    ]
    bell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    bell.merge(TorusGeometry(radius=0.472, tube=0.026, radial_segments=18, tubular_segments=96).translate(0.0, 0.0, -0.936))
    bell.merge(TorusGeometry(radius=0.155, tube=0.012, radial_segments=12, tubular_segments=64).translate(0.0, 0.0, -0.305))
    return mesh_from_geometry(bell, "hollow_bell_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_campanile_bell_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.63, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.58, 0.58, 0.55, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.63, 0.42, 0.18, 1.0))
    black = model.material("blackened_iron", rgba=(0.05, 0.045, 0.04, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.90, 1.90, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="concrete_pad",
    )

    column_xy = [(-0.62, -0.62), (0.62, -0.62), (0.62, 0.62), (-0.62, 0.62)]
    for index, (x, y) in enumerate(column_xy):
        tower.visual(
            Box((0.22, 0.22, 0.055)),
            origin=Origin(xyz=(x, y, 0.19)),
            material=dark_steel,
            name=f"base_plate_{index}",
        )
        tower.visual(
            Box((0.095, 0.095, 5.84)),
            origin=Origin(xyz=(x, y, 3.08)),
            material=galvanized,
            name=f"column_{index}",
        )

    levels = [0.44, 1.28, 2.12, 2.96, 3.80, 4.56, 5.28, 5.92]
    for level_index, z in enumerate(levels):
        _add_square_tube(
            tower,
            (-0.62, -0.62, z),
            (0.62, -0.62, z),
            0.060,
            material=galvanized,
            name=f"front_girt_{level_index}",
        )
        _add_square_tube(
            tower,
            (-0.62, 0.62, z),
            (0.62, 0.62, z),
            0.060,
            material=galvanized,
            name=f"rear_girt_{level_index}",
        )
        _add_square_tube(
            tower,
            (-0.62, -0.62, z),
            (-0.62, 0.62, z),
            0.060,
            material=galvanized,
            name=f"side_girt_{level_index}",
        )
        _add_square_tube(
            tower,
            (0.62, -0.62, z),
            (0.62, 0.62, z),
            0.060,
            material=galvanized,
            name=f"side_girt_b_{level_index}",
        )

    brace_levels = levels[:-1]
    for bay, z0 in enumerate(brace_levels):
        z1 = levels[bay + 1]
        if bay % 2 == 0:
            front_a, front_b = (-0.62, -0.62, z0), (0.62, -0.62, z1)
            rear_a, rear_b = (0.62, 0.62, z0), (-0.62, 0.62, z1)
            side_a, side_b = (-0.62, 0.62, z0), (-0.62, -0.62, z1)
            side_c, side_d = (0.62, -0.62, z0), (0.62, 0.62, z1)
        else:
            front_a, front_b = (0.62, -0.62, z0), (-0.62, -0.62, z1)
            rear_a, rear_b = (-0.62, 0.62, z0), (0.62, 0.62, z1)
            side_a, side_b = (-0.62, -0.62, z0), (-0.62, 0.62, z1)
            side_c, side_d = (0.62, 0.62, z0), (0.62, -0.62, z1)
        _add_square_tube(tower, front_a, front_b, 0.045, material=galvanized, name=f"front_diagonal_{bay}")
        _add_square_tube(tower, rear_a, rear_b, 0.045, material=galvanized, name=f"rear_diagonal_{bay}")
        if bay == 5:
            # Split the side-bracing around the belfry bearing opening so the
            # bell's horizontal trunnion has a real clear slot through the side frames.
            _add_square_tube(tower, (-0.62, -0.62, z0), (-0.62, -0.25, 4.88), 0.045, material=galvanized, name="side_diagonal_5a")
            _add_square_tube(tower, (-0.62, 0.25, 5.00), (-0.62, 0.62, z1), 0.045, material=galvanized, name="side_diagonal_5b")
            _add_square_tube(tower, (0.62, 0.62, z0), (0.62, 0.25, 4.88), 0.045, material=galvanized, name="side_diagonal_5c")
            _add_square_tube(tower, (0.62, -0.25, 5.00), (0.62, -0.62, z1), 0.045, material=galvanized, name="side_diagonal_5d")
        else:
            _add_square_tube(tower, side_a, side_b, 0.045, material=galvanized, name=f"side_diagonal_{bay}")
            _add_square_tube(tower, side_c, side_d, 0.045, material=galvanized, name=f"side_diagonal_b_{bay}")

    tower.visual(
        Box((1.58, 1.58, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 6.04)),
        material=dark_steel,
        name="flat_top_cap",
    )
    tower.visual(
        Box((1.46, 1.46, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 5.92)),
        material=galvanized,
        name="cap_frame",
    )

    # Open belfry bearing stage: side truss beams carry the rotating headstock.
    tower.visual(
        Box((0.13, 0.48, 0.12)),
        origin=Origin(xyz=(-0.62, -0.39, 4.96)),
        material=dark_steel,
        name="bearing_crossbeam_0",
    )
    tower.visual(
        Box((0.13, 0.48, 0.12)),
        origin=Origin(xyz=(-0.62, 0.39, 4.96)),
        material=dark_steel,
        name="bearing_crossbeam_0b",
    )
    tower.visual(
        Cylinder(radius=0.090, length=0.22),
        origin=Origin(xyz=(-0.62, 0.0, 4.96), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_sleeve_0",
    )
    tower.visual(
        Box((0.18, 0.10, 0.18)),
        origin=Origin(xyz=(-0.62, -0.105, 4.96)),
        material=dark_steel,
        name="bearing_plate_0a",
    )
    tower.visual(
        Box((0.18, 0.10, 0.18)),
        origin=Origin(xyz=(-0.62, 0.105, 4.96)),
        material=dark_steel,
        name="bearing_plate_0b",
    )
    tower.visual(
        Box((0.18, 0.18, 0.07)),
        origin=Origin(xyz=(-0.62, 0.0, 5.055)),
        material=dark_steel,
        name="bearing_plate_0c",
    )
    tower.visual(
        Box((0.18, 0.18, 0.07)),
        origin=Origin(xyz=(-0.62, 0.0, 4.865)),
        material=dark_steel,
        name="bearing_plate_0d",
    )
    tower.visual(
        Box((0.13, 0.48, 0.12)),
        origin=Origin(xyz=(0.62, -0.39, 4.96)),
        material=dark_steel,
        name="bearing_crossbeam_1",
    )
    tower.visual(
        Box((0.13, 0.48, 0.12)),
        origin=Origin(xyz=(0.62, 0.39, 4.96)),
        material=dark_steel,
        name="bearing_crossbeam_1b",
    )
    tower.visual(
        Cylinder(radius=0.090, length=0.22),
        origin=Origin(xyz=(0.62, 0.0, 4.96), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_sleeve_1",
    )
    tower.visual(
        Box((0.18, 0.10, 0.18)),
        origin=Origin(xyz=(0.62, -0.105, 4.96)),
        material=dark_steel,
        name="bearing_plate_1a",
    )
    tower.visual(
        Box((0.18, 0.10, 0.18)),
        origin=Origin(xyz=(0.62, 0.105, 4.96)),
        material=dark_steel,
        name="bearing_plate_1b",
    )
    tower.visual(
        Box((0.18, 0.18, 0.07)),
        origin=Origin(xyz=(0.62, 0.0, 5.055)),
        material=dark_steel,
        name="bearing_plate_1c",
    )
    tower.visual(
        Box((0.18, 0.18, 0.07)),
        origin=Origin(xyz=(0.62, 0.0, 4.865)),
        material=dark_steel,
        name="bearing_plate_1d",
    )

    tower.inertial = Inertial.from_geometry(
        Box((1.90, 1.90, 6.10)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.045, length=1.40),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    bell.visual(
        Box((1.02, 0.17, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_steel,
        name="headstock",
    )
    bell.visual(
        Box((0.12, 0.11, 0.32)),
        origin=Origin(xyz=(-0.18, 0.0, -0.18)),
        material=dark_steel,
        name="hanger_strap_0",
    )
    bell.visual(
        Box((0.12, 0.11, 0.32)),
        origin=Origin(xyz=(0.18, 0.0, -0.18)),
        material=dark_steel,
        name="hanger_strap_1",
    )
    bell.visual(
        Cylinder(radius=0.105, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=bronze,
        name="crown_boss",
    )
    bell.visual(_bell_shell_mesh(), material=bronze, name="bell_shell")
    bell.visual(
        Cylinder(radius=0.018, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, -0.575)),
        material=black,
        name="clapper_rod",
    )
    bell.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.825)),
        material=black,
        name="clapper_ball",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=1.05),
        mass=360.0,
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 4.96)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=1.2,
            lower=-0.70,
            upper=0.70,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("bell_swing")

    ctx.allow_overlap(
        tower,
        bell,
        elem_a="bearing_sleeve_0",
        elem_b="trunnion_shaft",
        reason="The bell trunnion shaft is intentionally seated inside the left bearing sleeve proxy.",
    )
    ctx.allow_overlap(
        tower,
        bell,
        elem_a="bearing_sleeve_1",
        elem_b="trunnion_shaft",
        reason="The bell trunnion shaft is intentionally seated inside the right bearing sleeve proxy.",
    )
    ctx.expect_within(
        bell,
        tower,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_sleeve_0",
        margin=0.001,
        name="left sleeve surrounds shaft axis",
    )
    ctx.expect_within(
        bell,
        tower,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_sleeve_1",
        margin=0.001,
        name="right sleeve surrounds shaft axis",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="bearing_sleeve_0",
        min_overlap=0.08,
        name="left trunnion remains captured",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="bearing_sleeve_1",
        min_overlap=0.08,
        name="right trunnion remains captured",
    )

    with ctx.pose({hinge: 0.70}):
        moved = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({hinge: 0.0}):
        rest = ctx.part_element_world_aabb(bell, elem="bell_shell")
        ctx.expect_gap(
            tower,
            bell,
            axis="z",
            positive_elem="bearing_crossbeam_0",
            negative_elem="bell_shell",
            min_gap=0.08,
            name="bell clears side bearing beam below hinge",
        )

    ctx.check(
        "bell swings forward and upward",
        rest is not None
        and moved is not None
        and moved[1][1] > rest[1][1] + 0.40
        and ((moved[0][2] + moved[1][2]) * 0.5) > ((rest[0][2] + rest[1][2]) * 0.5) + 0.08,
        details=f"rest={rest}, swung={moved}",
    )

    return ctx.report()


object_model = build_object_model()
