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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guitar_effects_carry_case")

    black_shell = model.material("black_powder_coat", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.025, 0.026, 0.024, 1.0))
    smoky = model.material("smoky_polycarbonate", rgba=(0.16, 0.19, 0.21, 0.38))
    metal = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("blackened_hardware", rgba=(0.055, 0.055, 0.052, 1.0))

    lower_tray = model.part("lower_tray")

    # A broad, shallow, hollow lower shell: bottom pan plus separate rim walls.
    lower_tray.visual(
        Box((0.760, 0.400, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=black_shell,
        name="bottom_pan",
    )
    lower_tray.visual(
        Box((0.024, 0.382, 0.090)),
        origin=Origin(xyz=(-0.372, 0.0, 0.057)),
        material=black_shell,
        name="side_wall_0",
    )
    lower_tray.visual(
        Box((0.024, 0.382, 0.090)),
        origin=Origin(xyz=(0.372, 0.0, 0.057)),
        material=black_shell,
        name="side_wall_1",
    )
    lower_tray.visual(
        Box((0.760, 0.022, 0.082)),
        origin=Origin(xyz=(0.0, -0.205, 0.052)),
        material=black_shell,
        name="front_wall",
    )
    lower_tray.visual(
        Box((0.760, 0.026, 0.110)),
        origin=Origin(xyz=(0.0, 0.207, 0.065)),
        material=black_shell,
        name="rear_wall",
    )

    # The pedal surface is a raised, angled tray board inside the shell.
    tray_angle = math.radians(8.5)
    lower_tray.visual(
        Box((0.640, 0.300, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, 0.064), rpy=(tray_angle, 0.0, 0.0)),
        material=rubber,
        name="angled_deck",
    )
    lower_tray.visual(
        Box((0.620, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, -0.155, 0.030)),
        material=black_shell,
        name="front_deck_rib",
    )
    lower_tray.visual(
        Box((0.620, 0.026, 0.082)),
        origin=Origin(xyz=(0.0, 0.145, 0.055)),
        material=black_shell,
        name="rear_deck_rib",
    )
    for x in (-0.245, 0.0, 0.245):
        lower_tray.visual(
            Box((0.050, 0.270, 0.010)),
            origin=Origin(xyz=(x, -0.010, 0.073), rpy=(tray_angle, 0.0, 0.0)),
            material=dark_metal,
            name=f"deck_rail_{x:+.3f}",
        )

    # Alternating rear hinge barrels mounted to the lower shell.
    for x, length, name in (
        (-0.310, 0.130, "hinge_barrel_0"),
        (0.000, 0.090, "hinge_barrel_1"),
        (0.310, 0.130, "hinge_barrel_2"),
    ):
        lower_tray.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(x, 0.218, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=name,
        )
    lower_tray.visual(
        Box((0.700, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.213, 0.102)),
        material=metal,
        name="rear_hinge_leaf",
    )

    # Handle pivot lugs on the front face and two latch keepers.
    for i, x in enumerate((-0.300, 0.300)):
        lower_tray.visual(
            Box((0.050, 0.008, 0.040)),
            origin=Origin(xyz=(x, -0.217, 0.075)),
            material=metal,
            name=f"pivot_lug_{i}",
        )
    for i, x in enumerate((-0.180, 0.180)):
        lower_tray.visual(
            Box((0.060, 0.010, 0.026)),
            origin=Origin(xyz=(x, -0.216, 0.053)),
            material=metal,
            name=f"latch_keeper_{i}",
        )

    dust_cover = model.part("dust_cover")
    # The cover frame origin is the rear hinge axis. At q=0 it extends toward
    # local -Y over the tray and leaves its front lip just above the lower rim.
    dust_cover.visual(
        Box((0.782, 0.394, 0.016)),
        origin=Origin(xyz=(0.0, -0.205, 0.066)),
        material=smoky,
        name="top_panel",
    )
    dust_cover.visual(
        Box((0.782, 0.018, 0.068)),
        origin=Origin(xyz=(0.0, -0.410, 0.006)),
        material=smoky,
        name="front_lip",
    )
    dust_cover.visual(
        Box((0.018, 0.392, 0.084)),
        origin=Origin(xyz=(-0.398, -0.205, 0.020)),
        material=smoky,
        name="side_lip_0",
    )
    dust_cover.visual(
        Box((0.018, 0.392, 0.084)),
        origin=Origin(xyz=(0.398, -0.205, 0.020)),
        material=smoky,
        name="side_lip_1",
    )
    dust_cover.visual(
        Box((0.782, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, -0.010, 0.040)),
        material=smoky,
        name="rear_lip",
    )
    for x, length, name in (
        (-0.145, 0.180, "cover_barrel_0"),
        (0.145, 0.180, "cover_barrel_1"),
    ):
        dust_cover.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=name,
        )
        dust_cover.visual(
            Box((length * 0.92, 0.012, 0.012)),
            origin=Origin(xyz=(x, -0.011, 0.010)),
            material=metal,
            name=f"{name}_strap",
        )
    dust_cover.visual(
        Box((0.700, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, -0.018, 0.026)),
        material=metal,
        name="cover_hinge_leaf",
    )
    for i, x in enumerate((-0.180, 0.180)):
        dust_cover.visual(
            Box((0.075, 0.014, 0.024)),
            origin=Origin(xyz=(x, -0.414, 0.020)),
            material=metal,
            name=f"latch_mount_{i}",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_tray,
        child=dust_cover,
        origin=Origin(xyz=(0.0, 0.218, 0.130)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    front_handle = model.part("front_handle")
    handle_tube = tube_from_spline_points(
        [
            (-0.300, -0.008, 0.000),
            (-0.298, -0.012, -0.050),
            (-0.220, -0.018, -0.103),
            (0.000, -0.023, -0.121),
            (0.220, -0.018, -0.103),
            (0.298, -0.012, -0.050),
            (0.300, -0.008, 0.000),
        ],
        radius=0.008,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    front_handle.visual(
        mesh_from_geometry(handle_tube, "front_handle_tube"),
        material=dark_metal,
        name="handle_tube",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_tray,
        child=front_handle,
        origin=Origin(xyz=(0.0, -0.217, 0.075)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    for i, x in enumerate((-0.180, 0.180)):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="latch_barrel",
        )
        latch.visual(
            Box((0.048, 0.008, 0.082)),
            origin=Origin(xyz=(0.0, -0.018, -0.045)),
            material=metal,
            name="latch_plate",
        )
        latch.visual(
            Box((0.044, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.018, -0.091)),
            material=metal,
            name="latch_hook",
        )
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=dust_cover,
            child=latch,
            origin=Origin(xyz=(x, -0.422, 0.020)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.20),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_tray = object_model.get_part("lower_tray")
    dust_cover = object_model.get_part("dust_cover")
    front_handle = object_model.get_part("front_handle")
    rear_hinge = object_model.get_articulation("rear_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_overlap(
        dust_cover,
        lower_tray,
        axes="xy",
        elem_a="top_panel",
        elem_b="bottom_pan",
        min_overlap=0.32,
        name="closed cover spans the broad lower tray",
    )
    ctx.expect_gap(
        dust_cover,
        lower_tray,
        axis="z",
        positive_elem="front_lip",
        negative_elem="front_wall",
        min_gap=0.001,
        max_gap=0.010,
        name="front cover lip sits just above the lower rim",
    )
    ctx.expect_gap(
        lower_tray,
        front_handle,
        axis="y",
        positive_elem="pivot_lug_0",
        negative_elem="handle_tube",
        min_gap=-0.006,
        max_gap=0.004,
        name="handle tube is seated at the front pivot lugs",
    )
    ctx.expect_gap(
        lower_tray,
        dust_cover,
        axis="x",
        positive_elem="hinge_barrel_1",
        negative_elem="cover_barrel_0",
        min_gap=0.006,
        max_gap=0.018,
        name="rear hinge knuckles share a line with a narrow separation gap",
    )

    closed_cover_aabb = ctx.part_world_aabb(dust_cover)
    with ctx.pose({rear_hinge: 1.20}):
        opened_cover_aabb = ctx.part_world_aabb(dust_cover)
    ctx.check(
        "dust cover opens upward from the rear hinge",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.18,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    rest_handle = ctx.part_world_aabb(front_handle)
    with ctx.pose({handle_pivot: 1.10}):
        lifted_handle = ctx.part_world_aabb(front_handle)
    ctx.check(
        "front handle rotates outward on side pivots",
        rest_handle is not None
        and lifted_handle is not None
        and lifted_handle[0][1] < rest_handle[0][1] - 0.035,
        details=f"rest={rest_handle}, lifted={lifted_handle}",
    )

    return ctx.report()


object_model = build_object_model()
