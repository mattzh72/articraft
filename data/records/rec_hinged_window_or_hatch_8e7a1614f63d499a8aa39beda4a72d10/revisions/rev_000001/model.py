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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_hinged_awning_window")

    painted_aluminum = model.material("painted_aluminum", rgba=(0.92, 0.93, 0.90, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.015, 0.017, 0.018, 1.0))
    glass_blue = model.material("slightly_blue_glass", rgba=(0.55, 0.78, 0.95, 0.38))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    handle_metal = model.material("satin_handle_metal", rgba=(0.30, 0.31, 0.32, 1.0))

    outer_w = 1.20
    outer_h = 0.85
    frame_profile = 0.08
    frame_depth = 0.10
    sash_w = 1.00
    sash_h = 0.63
    sash_rail = 0.06
    sash_depth = 0.055
    hinge_z = outer_h - frame_profile - 0.015
    hinge_y = 0.060

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((outer_w, frame_depth, frame_profile)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - frame_profile / 2.0)),
        material=painted_aluminum,
        name="top_rail",
    )
    outer_frame.visual(
        Box((outer_w, frame_depth, frame_profile)),
        origin=Origin(xyz=(0.0, 0.0, frame_profile / 2.0)),
        material=painted_aluminum,
        name="bottom_rail",
    )
    for x, name in (
        (-(outer_w - frame_profile) / 2.0, "stile_0"),
        ((outer_w - frame_profile) / 2.0, "stile_1"),
    ):
        outer_frame.visual(
            Box((frame_profile, frame_depth, outer_h)),
            origin=Origin(xyz=(x, 0.0, outer_h / 2.0)),
            material=painted_aluminum,
            name=name,
        )

    # A dark continuous-looking weather seal just inside the frame opening.
    inner_w = outer_w - 2.0 * frame_profile
    inner_h = outer_h - 2.0 * frame_profile
    seal_thick = 0.018
    seal_y = hinge_y - 0.018
    outer_frame.visual(
        Box((inner_w, 0.012, seal_thick)),
        origin=Origin(xyz=(0.0, seal_y, outer_h - frame_profile - seal_thick / 2.0)),
        material=dark_gasket,
        name="upper_seal",
    )
    outer_frame.visual(
        Box((inner_w, 0.012, seal_thick)),
        origin=Origin(xyz=(0.0, seal_y, frame_profile + seal_thick / 2.0)),
        material=dark_gasket,
        name="lower_seal",
    )
    for x, name in (
        (-(inner_w - seal_thick) / 2.0, "side_seal_0"),
        ((inner_w - seal_thick) / 2.0, "side_seal_1"),
    ):
        outer_frame.visual(
            Box((seal_thick, 0.012, inner_h)),
            origin=Origin(xyz=(x, seal_y, frame_profile + inner_h / 2.0)),
            material=dark_gasket,
            name=name,
        )

    hinge_radius = 0.012
    hinge_len_parent = 0.040
    hinge_len_child = 0.022
    for x, name in ((-0.33, "hinge_0"), (0.33, "hinge_1")):
        outer_frame.visual(
            Box((0.13, 0.012, 0.040)),
            origin=Origin(xyz=(x, 0.055, hinge_z + 0.030)),
            material=hinge_metal,
            name=f"{name}_frame_leaf",
        )
        outer_frame.visual(
            Cylinder(radius=hinge_radius, length=hinge_len_parent),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"{name}_frame_knuckle",
        )

    # Small strike plate on the bottom rail to make the latch read as functional.
    outer_frame.visual(
        Box((0.18, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.054, frame_profile + 0.012)),
        material=hinge_metal,
        name="latch_strike",
    )

    sash = model.part("sash")
    sash.visual(
        Box((sash_w, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -0.015 - sash_rail / 2.0)),
        material=painted_aluminum,
        name="top_rail",
    )
    sash.visual(
        Box((sash_w, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_h + sash_rail / 2.0)),
        material=painted_aluminum,
        name="bottom_rail",
    )
    for x, name in (
        (-(sash_w - sash_rail) / 2.0, "stile_0"),
        ((sash_w - sash_rail) / 2.0, "stile_1"),
    ):
        sash.visual(
            Box((sash_rail, sash_depth, sash_h)),
            origin=Origin(xyz=(x, 0.0, -sash_h / 2.0)),
            material=painted_aluminum,
            name=name,
        )
    sash.visual(
        Box((sash_w - 2.0 * sash_rail + 0.020, 0.006, sash_h - 2.0 * sash_rail + 0.020)),
        origin=Origin(xyz=(0.0, -0.004, -sash_h / 2.0)),
        material=glass_blue,
        name="glass_pane",
    )
    # Front glazing gasket, slightly proud of the glass and tucked under the sash rails.
    gasket_w = sash_w - 2.0 * sash_rail + 0.012
    gasket_h = sash_h - 2.0 * sash_rail + 0.012
    gasket_y = sash_depth / 2.0 + 0.001
    sash.visual(
        Box((gasket_w, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, gasket_y, -sash_rail - 0.006)),
        material=dark_gasket,
        name="upper_glazing_gasket",
    )
    sash.visual(
        Box((gasket_w, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, gasket_y, -sash_h + sash_rail + 0.006)),
        material=dark_gasket,
        name="lower_glazing_gasket",
    )
    for x, name in (
        (-(gasket_w - 0.012) / 2.0, "side_glazing_gasket_0"),
        ((gasket_w - 0.012) / 2.0, "side_glazing_gasket_1"),
    ):
        sash.visual(
            Box((0.012, 0.006, gasket_h)),
            origin=Origin(xyz=(x, gasket_y, -sash_h / 2.0)),
            material=dark_gasket,
            name=name,
        )
    for x, name in ((-0.33, "hinge_0"), (0.33, "hinge_1")):
        sash.visual(
            Box((0.13, 0.010, 0.040)),
            origin=Origin(xyz=(x, 0.008, -0.030)),
            material=hinge_metal,
            name=f"{name}_sash_leaf",
        )
        for dx, suffix in ((-0.034, "outer"), (0.034, "inner")):
            sash.visual(
                Cylinder(radius=hinge_radius, length=hinge_len_child),
                origin=Origin(xyz=(x + dx, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_metal,
                name=f"{name}_sash_knuckle_{suffix}",
            )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=sash,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="handle_base",
    )
    latch_handle.visual(
        Box((0.155, 0.018, 0.026)),
        origin=Origin(xyz=(0.075, 0.021, 0.0)),
        material=handle_metal,
        name="handle_bar",
    )
    latch_handle.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.155, 0.022, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="finger_pad",
    )

    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch_handle,
        origin=Origin(xyz=(-0.075, sash_depth / 2.0, -sash_h + 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.65, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    latch_handle = object_model.get_part("latch_handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    latch_pivot = object_model.get_articulation("sash_to_latch")

    ctx.expect_within(
        sash,
        outer_frame,
        axes="x",
        margin=0.001,
        name="closed sash is laterally contained by the frame opening",
    )
    ctx.expect_contact(
        latch_handle,
        sash,
        elem_a="handle_base",
        elem_b="bottom_rail",
        contact_tol=0.0015,
        name="latch handle base sits on the lower sash rail",
    )

    closed_aabb = ctx.part_element_world_aabb(sash, elem="bottom_rail")
    with ctx.pose({sash_hinge: 0.75}):
        opened_aabb = ctx.part_element_world_aabb(sash, elem="bottom_rail")
    if closed_aabb is not None and opened_aabb is not None:
        closed_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
        opened_y = (opened_aabb[0][1] + opened_aabb[1][1]) / 2.0
        ctx.check(
            "sash opens outward from the upper hinge",
            opened_y > closed_y + 0.25,
            details=f"closed_y={closed_y:.3f}, opened_y={opened_y:.3f}",
        )
    else:
        ctx.fail("sash opens outward from the upper hinge", "bottom rail AABB unavailable")

    latched_aabb = ctx.part_element_world_aabb(latch_handle, elem="finger_pad")
    with ctx.pose({latch_pivot: 1.0}):
        turned_aabb = ctx.part_element_world_aabb(latch_handle, elem="finger_pad")
    if latched_aabb is not None and turned_aabb is not None:
        latched_z = (latched_aabb[0][2] + latched_aabb[1][2]) / 2.0
        turned_z = (turned_aabb[0][2] + turned_aabb[1][2]) / 2.0
        ctx.check(
            "latch handle rotates on its face pivot",
            turned_z < latched_z - 0.08,
            details=f"latched_z={latched_z:.3f}, turned_z={turned_z:.3f}",
        )
    else:
        ctx.fail("latch handle rotates on its face pivot", "finger pad AABB unavailable")

    return ctx.report()


object_model = build_object_model()
