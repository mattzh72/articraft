from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_round_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _fork_tine_mesh(name: str):
    """One-piece tapered L-shaped fork tine, authored in XZ and extruded in Y."""
    width = 0.12
    profile = [
        (-0.035, 0.000),
        (0.020, 0.000),
        (1.075, 0.014),
        (1.165, 0.034),
        (1.070, 0.058),
        (0.135, 0.090),
        (0.080, 0.520),
        (-0.035, 0.520),
    ]
    tine = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(width)
        .translate((0.0, -width / 2.0, 0.0))
    )
    return mesh_from_cadquery(tine, name, tolerance=0.002, angular_tolerance=0.15)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_mounted_fork_carriage")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.62, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.02, 0.02, 0.02, 1.0))
    hydraulic_chrome = model.material("hydraulic_chrome", rgba=(0.74, 0.76, 0.78, 1.0))
    hose_black = model.material("hose_black", rgba=(0.03, 0.03, 0.035, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.72, 0.98, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.04)),
        material=dark_steel,
        name="floor_base",
    )
    mast.visual(
        Box((0.34, 0.86, 0.10)),
        origin=Origin(xyz=(-0.05, 0.0, 0.13)),
        material=dark_steel,
        name="mast_foot",
    )

    mast.visual(
        Box((0.080, 0.105, 2.14)),
        origin=Origin(xyz=(-0.045, -0.34, 1.15)),
        material=dark_steel,
        name="upright_neg",
    )
    mast.visual(
        Box((0.014, 0.092, 2.00)),
        origin=Origin(xyz=(0.004, -0.34, 1.18)),
        material=worn_steel,
        name="guide_strip_neg",
    )
    mast.visual(
        Box((0.080, 0.105, 2.14)),
        origin=Origin(xyz=(-0.045, 0.34, 1.15)),
        material=dark_steel,
        name="upright_pos",
    )
    mast.visual(
        Box((0.014, 0.092, 2.00)),
        origin=Origin(xyz=(0.004, 0.34, 1.18)),
        material=worn_steel,
        name="guide_strip_pos",
    )

    mast.visual(
        Box((0.135, 0.82, 0.085)),
        origin=Origin(xyz=(-0.035, 0.0, 0.245)),
        material=dark_steel,
        name="lower_crosshead",
    )
    mast.visual(
        Box((0.135, 0.82, 0.090)),
        origin=Origin(xyz=(-0.035, 0.0, 2.205)),
        material=dark_steel,
        name="top_crosshead",
    )
    mast.visual(
        Box((0.110, 0.76, 0.055)),
        origin=Origin(xyz=(-0.075, 0.0, 1.34)),
        material=rail_steel,
        name="rear_tie_bar",
    )

    for x in (-0.090, -0.010):
        _add_round_member(
            mast,
            (x, -0.38, 0.32),
            (x, 0.38, 1.22),
            radius=0.014,
            material=rail_steel,
            name="diagonal_brace",
        )
        _add_round_member(
            mast,
            (x, 0.38, 0.32),
            (x, -0.38, 1.22),
            radius=0.014,
            material=rail_steel,
            name="diagonal_brace",
        )
        _add_round_member(
            mast,
            (x, -0.38, 1.28),
            (x, 0.38, 2.08),
            radius=0.014,
            material=rail_steel,
            name="upper_diagonal_brace",
        )
        _add_round_member(
            mast,
            (x, 0.38, 1.28),
            (x, -0.38, 2.08),
            radius=0.014,
            material=rail_steel,
            name="upper_diagonal_brace",
        )

    mast.visual(
        Cylinder(radius=0.045, length=0.92),
        origin=Origin(xyz=(-0.095, 0.0, 0.82)),
        material=dark_steel,
        name="lift_cylinder",
    )
    mast.visual(
        Cylinder(radius=0.026, length=0.78),
        origin=Origin(xyz=(-0.095, 0.0, 1.58)),
        material=hydraulic_chrome,
        name="lift_rod",
    )
    mast.visual(
        Box((0.13, 0.20, 0.055)),
        origin=Origin(xyz=(-0.095, 0.0, 0.35)),
        material=dark_steel,
        name="lower_cylinder_mount",
    )
    mast.visual(
        Box((0.13, 0.20, 0.050)),
        origin=Origin(xyz=(-0.095, 0.0, 1.985)),
        material=dark_steel,
        name="upper_cylinder_mount",
    )
    for y in (-0.17, 0.17):
        mast.visual(
            Cylinder(radius=0.038, length=0.09),
            origin=Origin(xyz=(-0.005, y, 2.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rail_steel,
            name="top_chain_sheave",
        )
        mast.visual(
            Box((0.026, 0.030, 1.42)),
            origin=Origin(xyz=(0.030, y, 1.40)),
            material=hose_black,
            name="lift_chain_run",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.055, 0.78, 0.56)),
        origin=Origin(xyz=(0.000, 0.0, 0.385)),
        material=safety_yellow,
        name="back_plate",
    )
    carriage.visual(
        Box((0.105, 0.92, 0.095)),
        origin=Origin(xyz=(0.025, 0.0, 0.125)),
        material=safety_yellow,
        name="lower_fork_bar",
    )
    carriage.visual(
        Box((0.105, 0.92, 0.095)),
        origin=Origin(xyz=(0.025, 0.0, 0.655)),
        material=safety_yellow,
        name="upper_fork_bar",
    )
    for y in (-0.42, 0.42):
        carriage.visual(
            Box((0.095, 0.080, 0.66)),
            origin=Origin(xyz=(0.022, y, 0.385)),
            material=safety_yellow,
            name="side_upright",
        )
        carriage.visual(
            Box((0.050, 0.065, 0.52)),
            origin=Origin(xyz=(-0.020, y, 0.855)),
            material=safety_yellow,
            name="load_guard_post",
        )

    for z in (0.72, 0.89, 1.06):
        carriage.visual(
            Box((0.040, 0.82, 0.035)),
            origin=Origin(xyz=(-0.018, 0.0, z)),
            material=safety_yellow,
            name="load_guard_bar",
        )

    for y in (-0.34, 0.34):
        for z in (0.205, 0.585):
            carriage.visual(
                Cylinder(radius=0.037, length=0.040),
                origin=Origin(xyz=(-0.064, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber_black,
                name="guide_roller",
            )
            carriage.visual(
                Box((0.040, 0.062, 0.052)),
                origin=Origin(xyz=(-0.040, y, z)),
                material=dark_steel,
                name="roller_bracket",
            )

    carriage.visual(
        Box((0.040, 0.20, 0.065)),
        origin=Origin(xyz=(0.055, 0.0, 0.655)),
        material=dark_steel,
        name="chain_anchor",
    )
    for y in (-0.13, 0.13):
        carriage.visual(
            Box((0.018, 0.030, 0.50)),
            origin=Origin(xyz=(0.055, y, 0.42)),
            material=hose_black,
            name="carriage_chain",
        )

    tine_mesh_0 = _fork_tine_mesh("fork_tine_0")
    tine_mesh_1 = _fork_tine_mesh("fork_tine_1")
    for index, y in enumerate((-0.25, 0.25)):
        fork = model.part(f"fork_{index}")
        fork.visual(tine_mesh_0 if index == 0 else tine_mesh_1, material=dark_steel, name="tapered_tine")
        fork.visual(
            Box((0.030, 0.130, 0.075)),
            origin=Origin(xyz=(-0.020, 0.0, 0.125)),
            material=dark_steel,
            name="lower_hook_pad",
        )
        fork.visual(
            Box((0.030, 0.130, 0.075)),
            origin=Origin(xyz=(-0.020, 0.0, 0.475)),
            material=dark_steel,
            name="upper_hook_pad",
        )
        fork.visual(
            Cylinder(radius=0.014, length=0.145),
            origin=Origin(xyz=(-0.018, 0.0, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name="lock_pin",
        )
        model.articulation(
            f"carriage_to_fork_{index}",
            ArticulationType.FIXED,
            parent=carriage,
            child=fork,
            origin=Origin(xyz=(0.110, y, 0.0875)),
        )

    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.112, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.35, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    fork_0 = object_model.get_part("fork_0")
    fork_1 = object_model.get_part("fork_1")
    lift = object_model.get_articulation("carriage_lift")

    ctx.expect_overlap(
        carriage,
        mast,
        axes="y",
        min_overlap=0.60,
        name="carriage spans between mast guide rails",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        min_gap=0.0,
        max_gap=0.08,
        positive_elem="guide_roller",
        negative_elem="guide_strip_pos",
        name="guide rollers ride just in front of mast strips",
    )
    for fork in (fork_0, fork_1):
        ctx.expect_contact(
            fork,
            carriage,
            elem_a="tapered_tine",
            elem_b="upper_fork_bar",
            contact_tol=0.006,
            name=f"{fork.name} heel hooks onto carriage bar",
        )
        ctx.expect_gap(
            fork,
            mast,
            axis="x",
            min_gap=0.12,
            positive_elem="tapered_tine",
            negative_elem="guide_strip_pos",
            name=f"{fork.name} projects forward clear of mast",
        )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_fork_pos = ctx.part_world_position(fork_0)
    with ctx.pose({lift: 0.75}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        raised_fork_pos = ctx.part_world_position(fork_0)
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            min_gap=-2.00,
            max_penetration=2.00,
            name="raised carriage remains inside mast height envelope",
        )

    ctx.check(
        "carriage lift moves upward",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.70,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )
    ctx.check(
        "forks lift with carriage",
        rest_fork_pos is not None
        and raised_fork_pos is not None
        and raised_fork_pos[2] > rest_fork_pos[2] + 0.70,
        details=f"rest={rest_fork_pos}, raised={raised_fork_pos}",
    )

    return ctx.report()


object_model = build_object_model()
