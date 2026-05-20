from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hinged_casement_window")

    painted = model.material("painted_white", rgba=(0.92, 0.90, 0.84, 1.0))
    dark_seal = model.material("black_weather_seal", rgba=(0.02, 0.025, 0.025, 1.0))
    metal = model.material("brushed_hinge_metal", rgba=(0.62, 0.60, 0.56, 1.0))
    brass = model.material("aged_brass_handle", rgba=(0.74, 0.55, 0.25, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.58, 0.80, 0.95, 0.38))

    # World/object frame: X is horizontal across the window, Z is vertical, and
    # +Y is the exterior/outward side that the casement sash opens toward.
    frame = model.part("frame")
    frame.visual(
        Box((0.08, 0.12, 1.40)),
        origin=Origin(xyz=(-0.46, 0.0, 0.0)),
        material=painted,
        name="hinge_jamb",
    )
    frame.visual(
        Box((0.08, 0.12, 1.40)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=painted,
        name="lock_jamb",
    )
    frame.visual(
        Box((1.00, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=painted,
        name="head_rail",
    )
    frame.visual(
        Box((1.00, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
        material=painted,
        name="sill_rail",
    )
    # A dark recessed weather seal makes the opening read as a real exterior
    # frame rebate rather than four plain bars.
    frame.visual(
        Box((0.018, 0.014, 1.20)),
        origin=Origin(xyz=(-0.411, 0.038, 0.0)),
        material=dark_seal,
        name="hinge_rebate",
    )
    frame.visual(
        Box((0.018, 0.014, 1.20)),
        origin=Origin(xyz=(0.411, 0.066, 0.0)),
        material=dark_seal,
        name="lock_rebate",
    )
    frame.visual(
        Box((0.82, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, 0.611)),
        material=dark_seal,
        name="head_rebate",
    )
    frame.visual(
        Box((0.82, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, -0.611)),
        material=dark_seal,
        name="sill_rebate",
    )

    # Two exposed butt hinges mounted to the hinge jamb.  The hinge pin axis is
    # exactly the sash joint axis; alternating knuckles show how the leaves
    # interleave without making the hardware a decorative afterthought.
    for idx, zc in enumerate((-0.43, 0.43)):
        suffix = f"{idx}"
        for seg, dz in enumerate((-0.054, 0.054)):
            frame.visual(
                Box((0.060, 0.008, 0.048)),
                origin=Origin(xyz=(-0.430, 0.064, zc + dz)),
                material=metal,
                name=f"fixed_hinge_leaf_{suffix}_{seg}",
            )
            frame.visual(
                Cylinder(radius=0.012, length=0.048),
                origin=Origin(xyz=(-0.400, 0.075, zc + dz)),
                material=metal,
                name=f"fixed_knuckle_{suffix}_{seg}",
            )

    sash = model.part("sash")
    # The sash part frame lies on the vertical hinge pin.  At q=0 all sash
    # geometry extends along local +X from that pin, with the glazing set back
    # from the exterior face.
    sash.visual(
        Box((0.060, 0.045, 1.20)),
        origin=Origin(xyz=(0.030, -0.0375, 0.0)),
        material=painted,
        name="hinge_stile",
    )
    sash.visual(
        Box((0.060, 0.045, 1.20)),
        origin=Origin(xyz=(0.770, -0.0375, 0.0)),
        material=painted,
        name="lock_stile",
    )
    sash.visual(
        Box((0.800, 0.045, 0.050)),
        origin=Origin(xyz=(0.400, -0.0375, 0.575)),
        material=painted,
        name="top_rail",
    )
    sash.visual(
        Box((0.800, 0.045, 0.050)),
        origin=Origin(xyz=(0.400, -0.0375, -0.575)),
        material=painted,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.700, 0.010, 1.120)),
        origin=Origin(xyz=(0.400, -0.046, 0.0)),
        material=glass,
        name="glass_pane",
    )
    sash.visual(
        Box((0.690, 0.012, 0.020)),
        origin=Origin(xyz=(0.400, -0.017, 0.535)),
        material=dark_seal,
        name="top_glazing_bead",
    )
    sash.visual(
        Box((0.690, 0.012, 0.020)),
        origin=Origin(xyz=(0.400, -0.017, -0.535)),
        material=dark_seal,
        name="bottom_glazing_bead",
    )
    sash.visual(
        Box((0.020, 0.012, 1.070)),
        origin=Origin(xyz=(0.065, -0.017, 0.0)),
        material=dark_seal,
        name="hinge_glazing_bead",
    )
    sash.visual(
        Box((0.020, 0.012, 1.070)),
        origin=Origin(xyz=(0.735, -0.017, 0.0)),
        material=dark_seal,
        name="lock_glazing_bead",
    )
    for idx, zc in enumerate((-0.43, 0.43)):
        sash.visual(
            Box((0.060, 0.010, 0.070)),
            origin=Origin(xyz=(0.030, -0.017, zc)),
            material=metal,
            name=f"moving_hinge_leaf_{idx}",
        )
        sash.visual(
            Cylinder(radius=0.012, length=0.048),
            origin=Origin(xyz=(0.000, 0.000, zc)),
            material=metal,
            name=f"moving_knuckle_{idx}",
        )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.400, 0.075, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.40),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_disc",
    )
    handle.visual(
        Box((0.030, 0.014, 0.165)),
        origin=Origin(xyz=(0.0, 0.012, -0.080)),
        material=brass,
        name="lever_bar",
    )
    handle.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.012, -0.155)),
        material=brass,
        name="rounded_grip",
    )

    model.articulation(
        "sash_to_handle",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(xyz=(0.765, -0.006, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    handle = object_model.get_part("handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    handle_pivot = object_model.get_articulation("sash_to_handle")

    ctx.check(
        "sash has vertical side hinge",
        sash_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(sash_hinge.axis) == (0.0, 0.0, 1.0)
        and sash_hinge.motion_limits is not None
        and sash_hinge.motion_limits.lower == 0.0
        and sash_hinge.motion_limits.upper >= 1.2,
        details=f"type={sash_hinge.articulation_type}, axis={sash_hinge.axis}, limits={sash_hinge.motion_limits}",
    )
    ctx.check(
        "handle has its own pivot",
        handle_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(handle_pivot.axis) == (0.0, 1.0, 0.0)
        and handle_pivot.motion_limits is not None
        and handle_pivot.motion_limits.lower < 0.0
        and handle_pivot.motion_limits.upper > 0.0,
        details=f"type={handle_pivot.articulation_type}, axis={handle_pivot.axis}, limits={handle_pivot.motion_limits}",
    )
    ctx.check(
        "two visible hinge sets",
        len([v for v in frame.visuals if v.name and v.name.startswith("fixed_knuckle_")]) == 4
        and len([v for v in sash.visuals if v.name and v.name.startswith("moving_knuckle_")]) == 2,
        details="The frame should carry four fixed knuckle segments and the sash two moving knuckles.",
    )

    with ctx.pose({sash_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.002,
            name="closed sash sits inside exterior frame",
        )
        ctx.expect_contact(
            handle,
            sash,
            elem_a="pivot_disc",
            elem_b="lock_stile",
            contact_tol=0.003,
            name="handle pivot is seated on lock stile",
        )
        closed_lock_aabb = ctx.part_element_world_aabb(sash, elem="lock_stile")
        closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="rounded_grip")

    with ctx.pose({sash_hinge: 1.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="lock_stile",
            negative_elem="lock_jamb",
            min_gap=0.25,
            name="opened lock side swings outward",
        )
        opened_lock_aabb = ctx.part_element_world_aabb(sash, elem="lock_stile")

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    closed_lock = _center(closed_lock_aabb)
    opened_lock = _center(opened_lock_aabb)
    ctx.check(
        "sash free edge moves outward",
        closed_lock is not None and opened_lock is not None and opened_lock[1] > closed_lock[1] + 0.25,
        details=f"closed={closed_lock}, opened={opened_lock}",
    )

    with ctx.pose({sash_hinge: 0.0, handle_pivot: 1.0}):
        turned_grip_aabb = ctx.part_element_world_aabb(handle, elem="rounded_grip")

    closed_grip = _center(closed_grip_aabb)
    turned_grip = _center(turned_grip_aabb)
    ctx.check(
        "lever handle rotates visibly",
        closed_grip is not None
        and turned_grip is not None
        and turned_grip[0] < closed_grip[0] - 0.06
        and turned_grip[2] > closed_grip[2] + 0.04,
        details=f"closed={closed_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
