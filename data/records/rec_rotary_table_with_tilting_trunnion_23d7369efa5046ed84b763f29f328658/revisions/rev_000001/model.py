from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length * 0.5))
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]):
    return (
        _cyl_z(radius, length, (0.0, 0.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _ring_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length * 0.5))
    )


def _ring_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return (
        _ring_z(outer_radius, inner_radius, length, (0.0, 0.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _rotated_slot_cutter(
    *,
    radial_center: float,
    length: float,
    width: float,
    z_min: float,
    depth: float,
    angle_deg: float,
):
    return (
        cq.Workplane("XY")
        .slot2D(length, width)
        .extrude(depth)
        .translate((radial_center, 0.0, z_min))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    )


def _sector_plate_x(
    *,
    inner_radius: float,
    outer_radius: float,
    angle0: float,
    angle1: float,
    thickness: float,
    center: tuple[float, float, float],
):
    """Annular sector in a local YZ plane, extruded along X."""
    pts: list[tuple[float, float]] = []
    steps = 32
    for i in range(steps + 1):
        a = math.radians(angle0 + (angle1 - angle0) * i / steps)
        pts.append((outer_radius * math.cos(a), outer_radius * math.sin(a)))
    for i in range(steps, -1, -1):
        a = math.radians(angle0 + (angle1 - angle0) * i / steps)
        pts.append((inner_radius * math.cos(a), inner_radius * math.sin(a)))

    return (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(thickness)
        .translate((center[0] - thickness * 0.5, center[1], center[2]))
    )


def _build_base_frame_mesh():
    base = _box((1.12, 0.82, 0.075), (0.0, 0.0, 0.0375))

    # Machined plinth rails and a transverse torque tube base tie the side yokes
    # into one welded/machined stationary frame.
    for y in (-0.31, 0.31):
        base = base.union(_box((0.90, 0.075, 0.075), (0.0, y, 0.112)))
    base = base.union(_box((0.22, 0.68, 0.07), (0.0, 0.0, 0.115)))

    for sign in (-1.0, 1.0):
        x = sign * 0.48
        cheek = _box((0.14, 0.34, 0.52), (x, 0.0, 0.335))
        cheek = cheek.cut(_cyl_x(0.078, 0.22, (x, 0.0, 0.45)))
        base = base.union(cheek)

        # Webbed feet and gussets under each trunnion cheek.
        base = base.union(_box((0.20, 0.46, 0.085), (x, 0.0, 0.125)))
        base = base.union(_box((0.075, 0.50, 0.19), (x - sign * 0.012, 0.0, 0.225)))

        # Exposed bearing cheek/flange, bored through for the trunnion journal.
        base = base.union(_ring_x(0.155, 0.078, 0.045, (sign * 0.568, 0.0, 0.45)))

        # Four cap screw bosses on the bearing flange.
        for y, z in ((0.108, 0.515), (-0.108, 0.515), (0.108, 0.385), (-0.108, 0.385)):
            base = base.union(_cyl_x(0.014, 0.020, (sign * 0.594, y, z)))

    # A slotted sector stop bracket on one side shows the tilt limit hardware.
    sector = _sector_plate_x(
        inner_radius=0.235,
        outer_radius=0.292,
        angle0=-63.0,
        angle1=63.0,
        thickness=0.032,
        center=(0.604, 0.0, 0.45),
    )
    slot = _sector_plate_x(
        inner_radius=0.252,
        outer_radius=0.270,
        angle0=-51.0,
        angle1=51.0,
        thickness=0.050,
        center=(0.604, 0.0, 0.45),
    )
    base = base.union(sector.cut(slot))
    # A real welded stop sector is gusseted back into the bearing cheek instead
    # of hanging from a paper-thin edge.
    base = base.union(_box((0.090, 0.140, 0.120), (0.598, 0.205, 0.450)))
    for angle in (-58.0, 58.0):
        a = math.radians(angle)
        y = 0.280 * math.cos(a)
        z = 0.45 + 0.280 * math.sin(a)
        base = base.union(_cyl_x(0.018, 0.055, (0.623, y, z)))

    return base


def _build_cradle_mesh():
    cradle = _ring_z(0.318, 0.262, 0.060, (0.0, 0.0, -0.045))

    # Thick side yokes just outside the rotating table OD.
    for sign in (-1.0, 1.0):
        cradle = cradle.union(_box((0.070, 0.66, 0.17), (sign * 0.285, 0.0, -0.010)))
        cradle = cradle.union(_ring_x(0.132, 0.060, 0.060, (sign * 0.325, 0.0, 0.0)))
        cradle = cradle.union(_cyl_x(0.081, 0.215, (sign * 0.410, 0.0, 0.0)))

    # Front/rear boxed rails close the cradle and keep the central bearing ring rigid.
    for y in (-0.316, 0.316):
        cradle = cradle.union(_box((0.63, 0.055, 0.105), (0.0, y, -0.020)))
        cradle = cradle.union(_box((0.42, 0.030, 0.135), (0.0, y * 0.93, -0.010)))

    # Rotary spindle bearing collar: the table spindle runs in the open bore.
    cradle = cradle.union(_ring_z(0.112, 0.052, 0.136, (0.0, 0.0, -0.080)))
    cradle = cradle.union(_ring_z(0.145, 0.060, 0.024, (0.0, 0.0, -0.016)))
    cradle = cradle.union(_ring_z(0.138, 0.060, 0.024, (0.0, 0.0, -0.144)))

    # Four machined spokes tie the spin bearing collar into the surrounding
    # rotary support ring while staying below the spinning faceplate.
    for sign in (-1.0, 1.0):
        cradle = cradle.union(_box((0.235, 0.030, 0.040), (sign * 0.195, 0.0, -0.082)))
        cradle = cradle.union(_box((0.030, 0.235, 0.040), (0.0, sign * 0.195, -0.082)))

    # Moving stop lug that sweeps past the fixed sector stop.
    cradle = cradle.union(_box((0.065, 0.040, 0.050), (0.333, 0.255, 0.075)))
    cradle = cradle.union(_cyl_x(0.014, 0.040, (0.378, 0.255, 0.075)))

    return cradle


def _build_table_mesh():
    disk = _cyl_z(0.238, 0.055, (0.0, 0.0, 0.0275))

    # Four real through T-slot throats and wider shallow counter-slots in the face.
    for angle in (0.0, 90.0, 180.0, 270.0):
        disk = disk.cut(
            _rotated_slot_cutter(
                radial_center=0.132,
                length=0.150,
                width=0.024,
                z_min=-0.010,
                depth=0.082,
                angle_deg=angle,
            )
        )
        disk = disk.cut(
            _rotated_slot_cutter(
                radial_center=0.138,
                length=0.165,
                width=0.050,
                z_min=0.034,
                depth=0.035,
                angle_deg=angle,
            )
        )

    # Bolt circle and dowel holes on the machined faceplate.
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        r = 0.178
        disk = disk.cut(_cyl_z(0.0065, 0.080, (r * math.cos(a), r * math.sin(a), 0.030)))
    for i in range(8):
        a = 2.0 * math.pi * (i + 0.5) / 8.0
        r = 0.105
        disk = disk.cut(_cyl_z(0.0045, 0.080, (r * math.cos(a), r * math.sin(a), 0.030)))

    # Underside spindle and visible center hub are the table-side rotating parts.
    table = disk.union(_cyl_z(0.054, 0.172, (0.0, 0.0, -0.074)))
    table = table.union(_cyl_z(0.073, 0.050, (0.0, 0.0, 0.075)))
    table = table.union(_cyl_z(0.018, 0.028, (0.0, 0.0, 0.114)))
    return table


def _add_access_cover(model: ArticulatedObject, parent, *, sign: float, material, screw_material):
    cover = model.part(f"access_cover_{0 if sign < 0 else 1}")
    cover.visual(
        # Cover plate: local X is the bearing axis after this visual rotation.
        cq_mesh := mesh_from_cadquery(
            _cyl_x(0.086, 0.016, (0.0, 0.0, 0.0)),
            f"access_cover_{0 if sign < 0 else 1}_plate",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        material=material,
        name="cover_plate",
    )
    del cq_mesh
    for idx, (y, z) in enumerate(((0.052, 0.052), (-0.052, 0.052), (0.052, -0.052), (-0.052, -0.052))):
        cover.visual(
            mesh_from_cadquery(
                _cyl_x(0.012, 0.007, (sign * 0.010, y, z)),
                f"access_cover_{0 if sign < 0 else 1}_screw_{idx}",
                tolerance=0.001,
                angular_tolerance=0.08,
            ),
            material=screw_material,
            name=f"screw_{idx}",
        )

    model.articulation(
        f"access_cover_{0 if sign < 0 else 1}_mount",
        ArticulationType.FIXED,
        parent=parent,
        child=cover,
        origin=Origin(xyz=(sign * 0.5985, 0.0, 0.45)),
    )
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_trunnion_rotary_table")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.30, 0.31, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.07, 0.08, 0.085, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.38, 0.40, 0.40, 1.0))
    brass_tags = model.material("brass_tags", rgba=(0.78, 0.62, 0.25, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(
            _build_base_frame_mesh(),
            "stationary_trunnion_base",
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        material=cast_iron,
        name="base_frame",
    )
    base_frame.visual(
        Box((0.16, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.372, 0.088)),
        material=brass_tags,
        name="engraved_zero_tag",
    )

    cradle = model.part("trunnion_cradle")
    cradle.visual(
        mesh_from_cadquery(
            _build_cradle_mesh(),
            "tilting_trunnion_cradle",
            tolerance=0.0012,
            angular_tolerance=0.08,
        ),
        material=oiled_steel,
        name="trunnion_cradle",
    )

    rotary_table = model.part("rotary_table")
    rotary_table.visual(
        mesh_from_cadquery(
            _build_table_mesh(),
            "slotted_rotary_faceplate",
            tolerance=0.001,
            angular_tolerance=0.06,
        ),
        material=machined_steel,
        name="faceplate",
    )

    _add_access_cover(model, base_frame, sign=-1.0, material=dark_oxide, screw_material=machined_steel)
    _add_access_cover(model, base_frame, sign=1.0, material=dark_oxide, screw_material=machined_steel)

    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.55,
            lower=math.radians(-55.0),
            upper=math.radians(55.0),
        ),
    )

    model.articulation(
        "table_spin",
        ArticulationType.CONTINUOUS,
        parent=cradle,
        child=rotary_table,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    cradle = object_model.get_part("trunnion_cradle")
    table = object_model.get_part("rotary_table")
    tilt = object_model.get_articulation("trunnion_tilt")
    spin = object_model.get_articulation("table_spin")

    ctx.allow_overlap(
        base,
        cradle,
        elem_a="base_frame",
        elem_b="trunnion_cradle",
        reason=(
            "The trunnion journal shafts are intentionally represented as captured "
            "inside the side bearing bores with a tiny local interference fit."
        ),
    )
    ctx.allow_overlap(
        cradle,
        table,
        elem_a="trunnion_cradle",
        elem_b="faceplate",
        reason=(
            "The rotary table spindle is intentionally seated in the cradle bearing "
            "collar with a small hidden interference so the study reads as supported."
        ),
    )

    ctx.check(
        "primary mechanisms are explicit",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"tilt={tilt.articulation_type}, spin={spin.articulation_type}",
    )
    ctx.check(
        "tilt uses opposed trunnion axis",
        tuple(round(v, 3) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    ctx.expect_within(
        table,
        cradle,
        axes="xy",
        inner_elem="faceplate",
        outer_elem="trunnion_cradle",
        margin=0.0,
        name="faceplate is nested inside the cradle footprint",
    )
    ctx.expect_overlap(
        table,
        cradle,
        axes="z",
        elem_a="faceplate",
        elem_b="trunnion_cradle",
        min_overlap=0.090,
        name="spindle remains inserted in the cradle bearing height",
    )
    ctx.expect_overlap(
        cradle,
        base,
        axes="x",
        elem_a="trunnion_cradle",
        elem_b="base_frame",
        min_overlap=0.12,
        name="trunnion journals are seated in the side bearing cheeks",
    )

    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({tilt: math.radians(45.0), spin: math.radians(35.0)}):
        tilted_aabb = ctx.part_world_aabb(table)
        ctx.expect_overlap(
            cradle,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="trunnion_cradle",
            elem_b="base_frame",
            name="tilted trunnion journals remain captured by side bearings",
        )

    ctx.check(
        "tilt visibly changes the table plane",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][2] - tilted_aabb[0][2]) > (rest_aabb[1][2] - rest_aabb[0][2]) + 0.10,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
