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
    model = ArticulatedObject(name="large_dog_door")

    wall_mat = model.material("painted_wall", rgba=(0.86, 0.84, 0.78, 1.0))
    frame_mat = model.material("dark_frame", rgba=(0.08, 0.09, 0.10, 1.0))
    flap_mat = model.material("smoked_rigid_plastic", rgba=(0.22, 0.30, 0.36, 0.46))
    hinge_mat = model.material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    latch_mat = model.material("black_latch", rgba=(0.02, 0.02, 0.018, 1.0))

    # A large-dog size opening set into a wall panel.  X is width, Y is wall
    # thickness/outward direction, and Z is vertical.
    outer_w = 1.00
    outer_h = 1.25
    wall_t = 0.10
    opening_w = 0.62
    opening_h = 0.86
    side_w = (outer_w - opening_w) / 2.0
    cap_h = (outer_h - opening_h) / 2.0

    frame = model.part("wall_frame")
    frame.visual(
        Box((side_w + 0.01, wall_t, outer_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + side_w / 2.0), 0.0, 0.0)),
        material=wall_mat,
        name="wall_side_0",
    )
    frame.visual(
        Box((side_w + 0.01, wall_t, outer_h)),
        origin=Origin(xyz=((opening_w / 2.0 + side_w / 2.0), 0.0, 0.0)),
        material=wall_mat,
        name="wall_side_1",
    )
    frame.visual(
        Box((opening_w + 0.02, wall_t, cap_h + 0.01)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + cap_h / 2.0)),
        material=wall_mat,
        name="wall_cap_top",
    )
    frame.visual(
        Box((opening_w + 0.02, wall_t, cap_h + 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h / 2.0 + cap_h / 2.0))),
        material=wall_mat,
        name="wall_cap_bottom",
    )

    # Raised trim and inner tunnel liner make the fixed rectangular frame read
    # as a real manufactured dog-door frame rather than a flat cutout.
    trim_w = 0.07
    trim_t = 0.035
    trim_outer_w = opening_w + 2.0 * trim_w
    trim_outer_h = opening_h + 2.0 * trim_w
    trim_y = wall_t / 2.0 + trim_t / 2.0
    side_x = opening_w / 2.0 + trim_w / 2.0
    cap_z = opening_h / 2.0 + trim_w / 2.0
    frame.visual(
        Box((trim_w, trim_t, trim_outer_h)),
        origin=Origin(xyz=(-side_x, trim_y, 0.0)),
        material=frame_mat,
        name="side_trim_0",
    )
    frame.visual(
        Box((trim_w, trim_t, trim_outer_h)),
        origin=Origin(xyz=(side_x, trim_y, 0.0)),
        material=frame_mat,
        name="side_trim_1",
    )
    frame.visual(
        Box((trim_outer_w, trim_t, trim_w)),
        origin=Origin(xyz=(0.0, trim_y, cap_z)),
        material=frame_mat,
        name="top_trim",
    )
    frame.visual(
        Box((trim_outer_w, trim_t, trim_w)),
        origin=Origin(xyz=(0.0, trim_y, -cap_z)),
        material=frame_mat,
        name="bottom_trim",
    )
    liner_t = 0.03
    frame.visual(
        Box((liner_t, wall_t + 0.015, opening_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + liner_t / 2.0), 0.0, 0.0)),
        material=frame_mat,
        name="jamb_0",
    )
    frame.visual(
        Box((liner_t, wall_t + 0.015, opening_h)),
        origin=Origin(xyz=((opening_w / 2.0 + liner_t / 2.0), 0.0, 0.0)),
        material=frame_mat,
        name="jamb_1",
    )
    frame.visual(
        Box((opening_w + 2.0 * liner_t, wall_t + 0.015, liner_t)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + liner_t / 2.0)),
        material=frame_mat,
        name="jamb_top",
    )
    frame.visual(
        Box((opening_w + 2.0 * liner_t, wall_t + 0.015, liner_t)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h / 2.0 + liner_t / 2.0))),
        material=frame_mat,
        name="threshold",
    )

    hinge_y = wall_t / 2.0 + trim_t + 0.018
    hinge_z = opening_h / 2.0 - 0.030
    frame.visual(
        Cylinder(radius=0.014, length=opening_w + 0.02),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.30, 0.0, 0.30)):
        frame.visual(
            Box((0.035, 0.040, 0.075)),
            origin=Origin(xyz=(x, wall_t / 2.0 + 0.030, hinge_z + 0.045)),
            material=hinge_mat,
            name=f"hinge_bracket_{i}",
        )

    # The flap part frame is the upper hinge line.  Its rigid panel extends
    # downward in local -Z from that axis.
    flap = model.part("flap")
    flap.visual(
        Box((0.56, 0.026, 0.74)),
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        material=flap_mat,
        name="flap_panel",
    )
    for i, x in enumerate((-0.18, 0.18)):
        flap.visual(
            Cylinder(radius=0.020, length=0.18),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=flap_mat,
            name=f"hinge_sleeve_{i}",
        )
        flap.visual(
            Box((0.120, 0.018, 0.036)),
            origin=Origin(xyz=(x, 0.0, -0.037)),
            material=flap_mat,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=1.05),
    )

    # A small turn tab near the lower center of the flap.  Its own frame is the
    # pivot axis, normal to the flap face, so it rotates across the flap surface.
    latch = model.part("latch_tab")
    latch.visual(
        Box((0.060, 0.014, 0.145)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=latch_mat,
        name="tab_plate",
    )
    latch.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="tab_washer",
    )
    latch.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="pivot_pin",
    )

    model.articulation(
        "flap_to_latch",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=latch,
        origin=Origin(xyz=(0.0, 0.032, -0.675)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-1.4, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("wall_frame")
    flap = object_model.get_part("flap")
    latch = object_model.get_part("latch_tab")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    latch_pivot = object_model.get_articulation("flap_to_latch")

    # Coaxial hinge and latch pins are intentionally represented as captured
    # solid proxies.  Keep the overlap scoped to the named pin/sleeve features.
    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1"):
        ctx.allow_overlap(
            frame,
            flap,
            elem_a="hinge_pin",
            elem_b=sleeve,
            reason="The metal hinge pin is intentionally captured inside the flap hinge sleeve proxy.",
        )
        ctx.expect_overlap(
            frame,
            flap,
            axes="x",
            elem_a="hinge_pin",
            elem_b=sleeve,
            min_overlap=0.12,
            name=f"{sleeve} is retained on the hinge pin",
        )

    ctx.allow_overlap(
        flap,
        latch,
        elem_a="flap_panel",
        elem_b="pivot_pin",
        reason="The latch pivot pin intentionally passes through the rigid flap panel.",
    )
    ctx.expect_overlap(
        flap,
        latch,
        axes="y",
        elem_a="flap_panel",
        elem_b="pivot_pin",
        min_overlap=0.015,
        name="latch pivot penetrates the flap panel",
    )

    ctx.expect_within(
        flap,
        frame,
        axes="x",
        inner_elem="flap_panel",
        outer_elem="top_trim",
        margin=0.0,
        name="closed flap width fits inside the fixed frame",
    )
    ctx.expect_within(
        flap,
        frame,
        axes="z",
        inner_elem="flap_panel",
        outer_elem="side_trim_0",
        margin=0.0,
        name="closed flap height fits inside the fixed frame",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({flap_hinge: 0.0, latch_pivot: 0.0}):
        closed_bottom = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
        closed_tab = _aabb_center(ctx.part_element_world_aabb(latch, elem="tab_plate"))
    with ctx.pose({flap_hinge: 0.75, latch_pivot: 0.0}):
        swung_bottom = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
    with ctx.pose({flap_hinge: 0.0, latch_pivot: 1.0}):
        turned_tab = _aabb_center(ctx.part_element_world_aabb(latch, elem="tab_plate"))

    ctx.check(
        "positive flap rotation swings the rigid flap outward",
        closed_bottom is not None
        and swung_bottom is not None
        and swung_bottom[1] > closed_bottom[1] + 0.20,
        details=f"closed={closed_bottom}, swung={swung_bottom}",
    )
    ctx.check(
        "latch tab rotates on its own lower pivot",
        closed_tab is not None
        and turned_tab is not None
        and abs(turned_tab[0] - closed_tab[0]) > 0.035,
        details=f"closed={closed_tab}, turned={turned_tab}",
    )

    return ctx.report()


object_model = build_object_model()
