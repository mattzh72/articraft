from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bottom_hinged_hopper_window")

    painted_aluminum = model.material("warm_white_painted_aluminum", rgba=(0.92, 0.90, 0.84, 1.0))
    sash_finish = model.material("slightly_lighter_sash_finish", rgba=(0.96, 0.95, 0.90, 1.0))
    black_gasket = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("pale_blue_transparent_glass", rgba=(0.55, 0.78, 0.92, 0.38))
    steel = model.material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_handle = model.material("dark_latch_handle", rgba=(0.08, 0.09, 0.09, 1.0))

    frame = model.part("fixed_frame")
    # Rectangular outer frame: 1.10 m wide, 0.80 m high, with a deep
    # inward-facing reveal so the hopper sash visibly sits inside it.
    frame.visual(
        Box((1.10, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_aluminum,
        name="bottom_rail",
    )
    frame.visual(
        Box((1.10, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=painted_aluminum,
        name="top_rail",
    )
    frame.visual(
        Box((0.08, 0.14, 0.80)),
        origin=Origin(xyz=(-0.51, 0.0, 0.40)),
        material=painted_aluminum,
        name="stile_0",
    )
    frame.visual(
        Box((0.08, 0.14, 0.80)),
        origin=Origin(xyz=(0.51, 0.0, 0.40)),
        material=painted_aluminum,
        name="stile_1",
    )
    # Dark weather-strip reveal around the fixed opening.
    frame.visual(
        Box((0.93, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.062, 0.095)),
        material=black_gasket,
        name="lower_weatherstrip",
    )
    frame.visual(
        Box((0.93, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.062, 0.705)),
        material=black_gasket,
        name="upper_weatherstrip",
    )
    frame.visual(
        Box((0.018, 0.018, 0.61)),
        origin=Origin(xyz=(-0.465, -0.062, 0.40)),
        material=black_gasket,
        name="side_weatherstrip_0",
    )
    frame.visual(
        Box((0.018, 0.018, 0.61)),
        origin=Origin(xyz=(0.465, -0.062, 0.40)),
        material=black_gasket,
        name="side_weatherstrip_1",
    )

    hinge_y = 0.092
    hinge_z = 0.095
    hinge_xs = (-0.28, 0.28)
    cyl_x = Origin(rpy=(0.0, pi / 2.0, 0.0))
    for idx, hx in enumerate(hinge_xs):
        # Fixed hinge barrels flank the moving sash knuckle, making a compact
        # clevis that visually captures the lower edge.
        for suffix, dx in (("outer_a", -0.054), ("outer_b", 0.054)):
            frame.visual(
                Cylinder(radius=0.018, length=0.042),
                origin=Origin(xyz=(hx + dx, hinge_y, hinge_z), rpy=cyl_x.rpy),
                material=steel,
                name=f"hinge_{idx}_{suffix}",
            )
            frame.visual(
                Box((0.042, 0.014, 0.060)),
                origin=Origin(xyz=(hx + dx, 0.067, 0.093)),
                material=steel,
                name=f"hinge_{idx}_{suffix}_leaf",
            )

    sash = model.part("glazed_sash")
    # The sash part frame is exactly on the horizontal lower hinge axis.
    sash.visual(
        Box((0.88, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.062, 0.030)),
        material=sash_finish,
        name="lower_rail",
    )
    sash.visual(
        Box((0.88, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.062, 0.570)),
        material=sash_finish,
        name="top_rail",
    )
    sash.visual(
        Box((0.060, 0.045, 0.600)),
        origin=Origin(xyz=(-0.410, -0.062, 0.300)),
        material=sash_finish,
        name="stile_0",
    )
    sash.visual(
        Box((0.060, 0.045, 0.600)),
        origin=Origin(xyz=(0.410, -0.062, 0.300)),
        material=sash_finish,
        name="stile_1",
    )
    sash.visual(
        Box((0.760, 0.010, 0.500)),
        origin=Origin(xyz=(0.0, -0.064, 0.300)),
        material=glass,
        name="glass_pane",
    )
    # Interior glazing stops/gaskets, seated over the glass edges.
    sash.visual(
        Box((0.780, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.037, 0.060)),
        material=black_gasket,
        name="lower_glazing_stop",
    )
    sash.visual(
        Box((0.780, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.037, 0.540)),
        material=black_gasket,
        name="upper_glazing_stop",
    )
    sash.visual(
        Box((0.018, 0.010, 0.480)),
        origin=Origin(xyz=(-0.380, -0.037, 0.300)),
        material=black_gasket,
        name="side_glazing_stop_0",
    )
    sash.visual(
        Box((0.018, 0.010, 0.480)),
        origin=Origin(xyz=(0.380, -0.037, 0.300)),
        material=black_gasket,
        name="side_glazing_stop_1",
    )
    for idx, hx in enumerate(hinge_xs):
        sash.visual(
            Cylinder(radius=0.006, length=0.165),
            origin=Origin(xyz=(hx, 0.0, 0.0), rpy=cyl_x.rpy),
            material=steel,
            name=f"hinge_pin_{idx}",
        )
        sash.visual(
            Cylinder(radius=0.018, length=0.054),
            origin=Origin(xyz=(hx, 0.0, 0.0), rpy=cyl_x.rpy),
            material=steel,
            name=f"hinge_knuckle_{idx}",
        )
        sash.visual(
            Box((0.060, 0.045, 0.020)),
            origin=Origin(xyz=(hx, -0.020, 0.012)),
            material=steel,
            name=f"hinge_leaf_{idx}",
        )
    sash.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, -0.030, 0.555), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="latch_bushing",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # Sash geometry extends upward in local +Z; -X makes positive motion tip
        # the top of the hopper inward toward +Y.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.70),
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_handle,
        name="pivot_cap",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_stem",
    )
    handle.visual(
        Box((0.035, 0.014, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=dark_handle,
        name="lever",
    )
    handle.visual(
        Box((0.060, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=dark_handle,
        name="finger_grip",
    )
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(xyz=(0.0, -0.004, 0.555)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.95, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fixed_frame")
    sash = object_model.get_part("glazed_sash")
    handle = object_model.get_part("latch_handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    latch_pivot = object_model.get_articulation("sash_to_latch")

    ctx.allow_overlap(
        sash,
        handle,
        elem_a="latch_bushing",
        elem_b="pivot_stem",
        reason="The latch stem is intentionally captured through the small bushing on the sash top rail.",
    )
    for idx in (0, 1):
        for barrel in ("outer_a", "outer_b"):
            ctx.allow_overlap(
                frame,
                sash,
                elem_a=f"hinge_{idx}_{barrel}",
                elem_b=f"hinge_pin_{idx}",
                reason="The lower hinge pin intentionally passes through the fixed hinge barrel to capture the hopper sash.",
            )

    with ctx.pose({sash_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.0,
            name="closed sash sits inside outer frame outline",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_rail",
            negative_elem="top_rail",
            min_gap=0.010,
            max_gap=0.040,
            name="top of sash clears fixed head rail",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="lower_rail",
            negative_elem="bottom_rail",
            min_gap=0.005,
            max_gap=0.030,
            name="bottom of sash clears fixed sill rail",
        )
        for idx in (0, 1):
            ctx.expect_gap(
                sash,
                frame,
                axis="x",
                positive_elem=f"hinge_knuckle_{idx}",
                negative_elem=f"hinge_{idx}_outer_a",
                min_gap=0.002,
                max_gap=0.012,
                name=f"hinge {idx} knuckle clears first fixed barrel",
            )
            ctx.expect_gap(
                frame,
                sash,
                axis="x",
                positive_elem=f"hinge_{idx}_outer_b",
                negative_elem=f"hinge_knuckle_{idx}",
                min_gap=0.002,
                max_gap=0.012,
                name=f"hinge {idx} knuckle clears second fixed barrel",
            )
            ctx.expect_overlap(
                sash,
                frame,
                axes="yz",
                elem_a=f"hinge_knuckle_{idx}",
                elem_b=f"hinge_{idx}_outer_a",
                min_overlap=0.025,
                name=f"hinge {idx} barrels share lower hinge axis",
            )
            ctx.expect_within(
                sash,
                frame,
                axes="yz",
                inner_elem=f"hinge_pin_{idx}",
                outer_elem=f"hinge_{idx}_outer_a",
                margin=0.001,
                name=f"hinge {idx} pin is nested in fixed barrel",
            )
        ctx.expect_within(
            handle,
            sash,
            axes="xz",
            inner_elem="pivot_stem",
            outer_elem="latch_bushing",
            margin=0.001,
            name="latch stem is centered in sash bushing",
        )
        ctx.expect_overlap(
            handle,
            sash,
            axes="y",
            elem_a="pivot_stem",
            elem_b="latch_bushing",
            min_overlap=0.014,
            name="latch stem remains seated through bushing",
        )

    def _elem_center(part, elem_name):
        box = ctx.part_element_world_aabb(part, elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({sash_hinge: 0.0}):
        top_closed = _elem_center(sash, "top_rail")
    with ctx.pose({sash_hinge: 0.60}):
        top_tipped = _elem_center(sash, "top_rail")
        for idx in (0, 1):
            ctx.expect_overlap(
                sash,
                frame,
                axes="yz",
                elem_a=f"hinge_knuckle_{idx}",
                elem_b=f"hinge_{idx}_outer_b",
                min_overlap=0.025,
                name=f"opened sash stays captured in hinge {idx}",
            )
    ctx.check(
        "sash tips inward from lower hinge",
        top_closed is not None
        and top_tipped is not None
        and top_tipped[1] > top_closed[1] + 0.25
        and top_tipped[2] < top_closed[2] - 0.04,
        details=f"closed_top={top_closed}, tipped_top={top_tipped}",
    )

    with ctx.pose({latch_pivot: 0.0}):
        grip_locked = _elem_center(handle, "finger_grip")
    with ctx.pose({latch_pivot: 0.80}):
        grip_rotated = _elem_center(handle, "finger_grip")
    ctx.check(
        "latch handle rotates on its own pivot",
        grip_locked is not None
        and grip_rotated is not None
        and abs(grip_rotated[0] - grip_locked[0]) > 0.07,
        details=f"locked_grip={grip_locked}, rotated_grip={grip_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
