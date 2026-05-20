from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL = (0.80, 0.82, 0.84, 1.0)
SATIN_STEEL = (0.72, 0.74, 0.76, 1.0)
DARK_INSERT = (0.16, 0.17, 0.18, 1.0)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_faucet_with_filter_door")

    steel = model.material("steel", rgba=STEEL)
    satin_steel = model.material("satin_steel", rgba=SATIN_STEEL)
    dark_insert = model.material("dark_insert", rgba=DARK_INSERT)

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_steel,
        name="deck_flange",
    )
    base.visual(
        Box((0.048, 0.058, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=steel,
        name="cover_body",
    )
    base.visual(
        Box((0.002, 0.030, 0.026)),
        origin=Origin(xyz=(0.023, 0.0, 0.020)),
        material=satin_steel,
        name="door_bezel",
    )
    base.visual(
        Box((0.001, 0.024, 0.020)),
        origin=Origin(xyz=(0.0235, 0.0, 0.020)),
        material=dark_insert,
        name="filter_recess",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=satin_steel,
        name="lower_collar",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=steel,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.0125, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=satin_steel,
        name="spout_socket",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, 0.018, 0.066), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="handle_mount",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_steel,
        name="spout_collar",
    )
    spout.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="spout_riser",
    )
    spout.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.022),
                    (0.018, 0.0, 0.052),
                    (0.075, 0.0, 0.078),
                    (0.135, 0.0, 0.072),
                    (0.185, 0.0, 0.041),
                ],
                radius=0.008,
                samples_per_segment=24,
                radial_segments=22,
                cap_ends=True,
            ),
            "bar_faucet_spout_arch",
        ),
        material=steel,
        name="spout_arch",
    )
    spout.visual(
        Cylinder(radius=0.0066, length=0.016),
        origin=Origin(xyz=(0.188, 0.0, 0.032)),
        material=satin_steel,
        name="nozzle",
    )
    spout.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.188, 0.0, 0.022)),
        material=dark_insert,
        name="aerator",
    )

    handle = model.part("handle")
    handle.visual(
        Sphere(radius=0.008),
        material=satin_steel,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.0058, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_barrel",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (0.0, 0.028, 0.003),
                    (0.0, 0.060, 0.012),
                ],
                radius=0.0038,
                samples_per_segment=20,
                radial_segments=18,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
            "bar_faucet_handle_lever",
        ),
        material=steel,
        name="lever",
    )
    handle.visual(
        Box((0.004, 0.018, 0.007)),
        origin=Origin(xyz=(0.0, 0.067, 0.015)),
        material=satin_steel,
        name="grip",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((0.003, 0.022, 0.020)),
        origin=Origin(xyz=(0.0015, -0.011, 0.010)),
        material=steel,
        name="door_panel",
    )
    filter_door.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(xyz=(0.0015, -0.0008, 0.004)),
        material=satin_steel,
        name="hinge_lower",
    )
    filter_door.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(xyz=(0.0015, -0.0008, 0.016)),
        material=satin_steel,
        name="hinge_upper",
    )
    filter_door.visual(
        Box((0.006, 0.004, 0.010)),
        origin=Origin(xyz=(0.004, -0.020, 0.010)),
        material=satin_steel,
        name="pull",
    )

    model.articulation(
        "spout_rotate",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "side_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.030, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.25,
            upper=0.75,
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=filter_door,
        origin=Origin(xyz=(0.024, 0.011, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    door = object_model.get_part("filter_door")

    spout_rotate = object_model.get_articulation("spout_rotate")
    side_handle = object_model.get_articulation("side_handle")
    filter_door = object_model.get_articulation("door_hinge")

    ctx.expect_gap(
        spout,
        base,
        axis="z",
        positive_elem="spout_collar",
        negative_elem="spout_socket",
        max_gap=0.0005,
        max_penetration=0.0,
        name="spout collar seats on the base socket",
    )
    ctx.expect_gap(
        handle,
        base,
        axis="y",
        positive_elem="hub",
        negative_elem="handle_mount",
        max_gap=0.0005,
        max_penetration=0.0,
        name="side handle hub seats on its mount",
    )
    ctx.expect_gap(
        door,
        base,
        axis="x",
        positive_elem="door_panel",
        negative_elem="cover_body",
        max_gap=0.0005,
        max_penetration=0.0,
        name="filter door closes flush to the base cover",
    )
    ctx.expect_overlap(
        door,
        base,
        axes="yz",
        elem_a="door_panel",
        elem_b="cover_body",
        min_overlap=0.018,
        name="filter door stays within the base cover footprint",
    )

    rest_nozzle = _center_from_aabb(ctx.part_element_world_aabb(spout, elem="aerator"))
    with ctx.pose({spout_rotate: math.pi / 2.0}):
        turned_nozzle = _center_from_aabb(ctx.part_element_world_aabb(spout, elem="aerator"))
    ctx.check(
        "spout swivels from front to side",
        rest_nozzle is not None
        and turned_nozzle is not None
        and rest_nozzle[0] > 0.16
        and abs(rest_nozzle[1]) < 0.01
        and turned_nozzle[1] > 0.16
        and abs(turned_nozzle[0]) < 0.04,
        details=f"rest_nozzle={rest_nozzle}, turned_nozzle={turned_nozzle}",
    )

    rest_grip = _center_from_aabb(ctx.part_element_world_aabb(handle, elem="grip"))
    with ctx.pose({side_handle: 0.65}):
        lifted_grip = _center_from_aabb(ctx.part_element_world_aabb(handle, elem="grip"))
    ctx.check(
        "side handle lifts upward on its short pivot",
        rest_grip is not None
        and lifted_grip is not None
        and lifted_grip[2] > rest_grip[2] + 0.02,
        details=f"rest_grip={rest_grip}, lifted_grip={lifted_grip}",
    )

    rest_pull = _center_from_aabb(ctx.part_element_world_aabb(door, elem="pull"))
    with ctx.pose({filter_door: 1.10}):
        opened_pull = _center_from_aabb(ctx.part_element_world_aabb(door, elem="pull"))
    ctx.check(
        "filter door swings outward from the cover",
        rest_pull is not None
        and opened_pull is not None
        and opened_pull[0] > rest_pull[0] + 0.012,
        details=f"rest_pull={rest_pull}, opened_pull={opened_pull}",
    )

    return ctx.report()


object_model = build_object_model()
