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


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float):
    """CadQuery tube/ring, centered on its local Z axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="truck_bed_missile_launcher_module")

    olive = model.material("mat_olive_drab", rgba=(0.22, 0.28, 0.17, 1.0))
    dark_olive = model.material("mat_dark_olive", rgba=(0.12, 0.16, 0.10, 1.0))
    black = model.material("mat_black_bore", rgba=(0.01, 0.012, 0.010, 1.0))
    worn_steel = model.material("mat_worn_steel", rgba=(0.42, 0.43, 0.40, 1.0))
    rubber = model.material("mat_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    slewing_ring_mesh = mesh_from_cadquery(
        _annular_cylinder(0.48, 0.28, 0.060),
        "slewing_ring_annulus",
        tolerance=0.0015,
        angular_tolerance=0.06,
    )
    upper_ring_mesh = mesh_from_cadquery(
        _annular_cylinder(0.45, 0.26, 0.050),
        "upper_bearing_annulus",
        tolerance=0.0015,
        angular_tolerance=0.06,
    )
    yoke_bushing_mesh = mesh_from_cadquery(
        _annular_cylinder(0.125, 0.084, 0.110),
        "elevation_bushing_annulus",
        tolerance=0.001,
        angular_tolerance=0.05,
    )
    tube_shell_mesh = mesh_from_cadquery(
        _annular_cylinder(0.108, 0.086, 1.55),
        "hollow_launch_tube",
        tolerance=0.0012,
        angular_tolerance=0.04,
    )

    base = model.part("base_frame")
    # Truck-bed style skid frame: long side rails, cross members, deck plate, and
    # a stationary lower slewing race.  Members overlap slightly within the same
    # rigid part so the frame reads as welded steel rather than loose beams.
    base.visual(Box((2.55, 0.12, 0.20)), origin=Origin(xyz=(0.0, -0.70, 0.10)), material=dark_olive, name="side_rail_0")
    base.visual(Box((2.55, 0.12, 0.20)), origin=Origin(xyz=(0.0, 0.70, 0.10)), material=dark_olive, name="side_rail_1")
    base.visual(Box((0.16, 1.54, 0.18)), origin=Origin(xyz=(-1.18, 0.0, 0.10)), material=dark_olive, name="end_crossmember_0")
    base.visual(Box((0.16, 1.54, 0.18)), origin=Origin(xyz=(1.18, 0.0, 0.10)), material=dark_olive, name="end_crossmember_1")
    base.visual(Box((0.14, 1.42, 0.14)), origin=Origin(xyz=(-0.42, 0.0, 0.116)), material=dark_olive, name="mid_crossmember_0")
    base.visual(Box((0.14, 1.42, 0.14)), origin=Origin(xyz=(0.42, 0.0, 0.116)), material=dark_olive, name="mid_crossmember_1")
    base.visual(Box((1.32, 1.18, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=olive, name="deck_plate")
    base.visual(slewing_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.255)), material=worn_steel, name="lower_slewing_ring")
    base.visual(Cylinder(radius=0.08, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.255)), material=worn_steel, name="center_bearing_boss")
    for x in (-1.02, 1.02):
        for y in (-0.61, 0.61):
            base.visual(
                Box((0.34, 0.20, 0.035)),
                origin=Origin(xyz=(x, y, 0.0175)),
                material=rubber,
                name=f"skid_pad_{x}_{y}",
            )

    azimuth = model.part("azimuth_stage")
    azimuth.visual(upper_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.025)), material=worn_steel, name="upper_slewing_ring")
    azimuth.visual(Box((0.88, 0.76, 0.065)), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=olive, name="rotating_deck")
    azimuth.visual(Box((0.34, 0.34, 0.32)), origin=Origin(xyz=(0.0, 0.0, 0.245)), material=olive, name="pedestal")
    azimuth.visual(Box((0.20, 0.11, 0.43)), origin=Origin(xyz=(-0.02, -0.51, 0.310)), material=olive, name="elevation_arm_0")
    azimuth.visual(Box((0.20, 0.11, 0.43)), origin=Origin(xyz=(-0.02, 0.51, 0.310)), material=olive, name="elevation_arm_1")
    azimuth.visual(Box((0.24, 1.08, 0.09)), origin=Origin(xyz=(-0.02, 0.0, 0.155)), material=olive, name="arm_foot_bridge")
    azimuth.visual(Box((0.13, 1.05, 0.12)), origin=Origin(xyz=(-0.18, 0.0, 0.405)), material=dark_olive, name="rear_tie_beam")
    for y, suffix in [(-0.53, "0"), (0.53, "1")]:
        azimuth.visual(
            yoke_bushing_mesh,
            origin=Origin(xyz=(0.0, y, 0.650), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"elevation_bushing_{suffix}",
        )

    cradle = model.part("tube_cradle")
    # The child frame is on the transverse trunnion axis.  Tubes point along +X
    # at q=0; positive elevation rotates them upward around -Y.
    cradle.visual(
        Cylinder(radius=0.066, length=1.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.118, length=0.040),
        origin=Origin(xyz=(0.0, -0.455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_collar_0",
    )
    cradle.visual(
        Cylinder(radius=0.118, length=0.040),
        origin=Origin(xyz=(0.0, 0.455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_collar_1",
    )
    cradle.visual(Box((0.56, 0.055, 0.39)), origin=Origin(xyz=(0.19, -0.40, 0.215)), material=olive, name="side_cheek_0")
    cradle.visual(Box((0.56, 0.055, 0.39)), origin=Origin(xyz=(0.19, 0.40, 0.215)), material=olive, name="side_cheek_1")
    cradle.visual(Box((0.28, 0.86, 0.075)), origin=Origin(xyz=(0.23, 0.0, 0.080)), material=olive, name="rear_lower_saddle")
    cradle.visual(Box((0.13, 0.86, 0.075)), origin=Origin(xyz=(0.32, 0.0, 0.590)), material=olive, name="rear_upper_clamp")

    tube_y = (-0.255, 0.0, 0.255)
    tube_z = (0.205, 0.465)
    for row, z in enumerate(tube_z):
        for col, y in enumerate(tube_y):
            cradle.visual(
                tube_shell_mesh,
                origin=Origin(xyz=(0.91, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_olive,
                name=f"tube_{row}_{col}",
            )
            # A recessed dark disk gives each open muzzle visible bore depth.
            cradle.visual(
                Cylinder(radius=0.089, length=0.024),
                origin=Origin(xyz=(1.682, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"bore_shadow_{row}_{col}",
            )

    # Spacer grids at the rear and muzzle ends tie the individual cylindrical
    # tubes into one rigid bank without hiding the six round launch openings.
    for station, x in [("rear", 0.33), ("front", 1.46)]:
        cradle.visual(Box((0.09, 0.86, 0.055)), origin=Origin(xyz=(x, 0.0, 0.335)), material=olive, name=f"{station}_row_spacer")
        cradle.visual(Box((0.09, 0.050, 0.58)), origin=Origin(xyz=(x, -0.128, 0.335)), material=olive, name=f"{station}_column_spacer_0")
        cradle.visual(Box((0.09, 0.050, 0.58)), origin=Origin(xyz=(x, 0.128, 0.335)), material=olive, name=f"{station}_column_spacer_1")
        cradle.visual(Box((0.10, 0.86, 0.055)), origin=Origin(xyz=(x, 0.0, 0.055)), material=olive, name=f"{station}_bottom_clamp")
        cradle.visual(Box((0.10, 0.86, 0.055)), origin=Origin(xyz=(x, 0.0, 0.615)), material=olive, name=f"{station}_top_clamp")
        cradle.visual(Box((0.10, 0.055, 0.61)), origin=Origin(xyz=(x, -0.405, 0.335)), material=olive, name=f"{station}_side_clamp_0")
        cradle.visual(Box((0.10, 0.055, 0.61)), origin=Origin(xyz=(x, 0.405, 0.335)), material=olive, name=f"{station}_side_clamp_1")

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=base,
        child=azimuth,
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.6, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7000.0, velocity=0.45, lower=0.0, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    azimuth = object_model.get_part("azimuth_stage")
    cradle = object_model.get_part("tube_cradle")
    az_joint = object_model.get_articulation("azimuth")
    el_joint = object_model.get_articulation("elevation")

    ctx.expect_gap(
        azimuth,
        base,
        axis="z",
        positive_elem="upper_slewing_ring",
        negative_elem="lower_slewing_ring",
        max_gap=0.002,
        max_penetration=0.0001,
        name="upper slewing ring rests on lower race",
    )
    ctx.expect_overlap(
        azimuth,
        base,
        axes="xy",
        elem_a="upper_slewing_ring",
        elem_b="lower_slewing_ring",
        min_overlap=0.35,
        name="slewing rings are coaxial",
    )
    ctx.expect_within(
        cradle,
        azimuth,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="elevation_bushing_0",
        margin=0.030,
        name="trunnion shaft fits through one yoke bushing",
    )
    ctx.expect_overlap(
        cradle,
        azimuth,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="elevation_bushing_0",
        min_overlap=0.06,
        name="trunnion is retained in side bushing",
    )

    def _aabb_center(elem_name: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(cradle, elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    with ctx.pose({az_joint: 0.0, el_joint: 0.0}):
        level_front = _aabb_center("front_top_clamp")
    with ctx.pose({az_joint: 0.0, el_joint: 0.70}):
        raised_front = _aabb_center("front_top_clamp")
    ctx.check(
        "positive elevation raises tube bank muzzle",
        level_front is not None
        and raised_front is not None
        and raised_front[2] > level_front[2] + 0.55
        and raised_front[0] < level_front[0] - 0.20,
        details=f"level={level_front}, raised={raised_front}",
    )

    with ctx.pose({az_joint: 0.0, el_joint: 0.0}):
        straight_front = _aabb_center("front_top_clamp")
    with ctx.pose({az_joint: 0.75, el_joint: 0.0}):
        slewed_front = _aabb_center("front_top_clamp")
    ctx.check(
        "azimuth joint slews the launcher about a vertical axis",
        straight_front is not None
        and slewed_front is not None
        and abs(slewed_front[1] - straight_front[1]) > 0.55
        and abs(slewed_front[2] - straight_front[2]) < 0.02,
        details=f"straight={straight_front}, slewed={slewed_front}",
    )

    return ctx.report()


object_model = build_object_model()
