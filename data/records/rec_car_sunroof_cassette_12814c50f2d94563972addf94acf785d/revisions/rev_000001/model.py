from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_moonroof_cassette")

    aluminum = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_aluminum = model.material("shadowed_aluminium", rgba=(0.22, 0.24, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    glass = model.material("smoked_safety_glass", rgba=(0.05, 0.09, 0.12, 0.42))
    ceramic = model.material("black_ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.96))

    # Object frame convention:
    # +X is the vehicle/front rail direction, -X is rearward sliding travel.
    # +Z is out of the roof.  The glass top and aluminium top rails are flush at
    # z ~= 0.012 m when both joints are at zero.
    frame = model.part("cassette_frame")

    # Four overlapping extrusions create a true open aperture instead of a
    # collision-solid plate; the overlapped corners read as a welded cassette
    # frame and keep the root part one connected assembly.
    frame.visual(
        Box((0.265, 0.950, 0.040)),
        origin=Origin(xyz=(0.7925, 0.0, -0.008)),
        material=aluminum,
        name="front_frame",
    )
    frame.visual(
        Box((0.265, 0.950, 0.040)),
        origin=Origin(xyz=(-0.7925, 0.0, -0.008)),
        material=aluminum,
        name="rear_frame",
    )
    frame.visual(
        Box((1.850, 0.135, 0.040)),
        origin=Origin(xyz=(0.0, -0.4075, -0.008)),
        material=aluminum,
        name="side_frame_0",
    )
    frame.visual(
        Box((1.850, 0.135, 0.040)),
        origin=Origin(xyz=(0.0, 0.4075, -0.008)),
        material=aluminum,
        name="side_frame_1",
    )

    # Separate rubber seal strips avoid filling the open glass aperture with a
    # collision-solid proxy while still reading as a continuous compressible seal.
    frame.visual(
        Box((1.10, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, -0.345, 0.011)),
        material=black_rubber,
        name="side_gasket_0",
    )
    frame.visual(
        Box((1.10, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.345, 0.011)),
        material=black_rubber,
        name="side_gasket_1",
    )
    frame.visual(
        Box((0.035, 0.660, 0.010)),
        origin=Origin(xyz=(0.530, 0.0, 0.011)),
        material=black_rubber,
        name="front_gasket",
    )
    frame.visual(
        Box((0.035, 0.660, 0.010)),
        origin=Origin(xyz=(-0.535, 0.0, 0.011)),
        material=black_rubber,
        name="rear_gasket",
    )

    # Full-length U-channel style guides.  Their upper flanges are level with
    # the roof skin, while the dark guide slots show where the carriage shoes run.
    frame.visual(
        Box((1.58, 0.055, 0.028)),
        origin=Origin(xyz=(-0.08, -0.365, -0.002)),
        material=aluminum,
        name="guide_rail_0",
    )
    frame.visual(
        Box((1.48, 0.018, 0.004)),
        origin=Origin(xyz=(-0.10, -0.365, 0.014)),
        material=dark_aluminum,
        name="guide_slot_0",
    )
    frame.visual(
        Box((1.58, 0.055, 0.028)),
        origin=Origin(xyz=(-0.08, 0.365, -0.002)),
        material=aluminum,
        name="guide_rail_1",
    )
    frame.visual(
        Box((1.48, 0.018, 0.004)),
        origin=Origin(xyz=(-0.10, 0.365, 0.014)),
        material=dark_aluminum,
        name="guide_slot_1",
    )

    frame.visual(
        Box((0.125, 0.82, 0.035)),
        origin=Origin(xyz=(0.540, 0.0, -0.006)),
        material=aluminum,
        name="front_rail",
    )
    frame.visual(
        Box((0.105, 0.82, 0.032)),
        origin=Origin(xyz=(-0.720, 0.0, -0.010)),
        material=aluminum,
        name="rear_crossmember",
    )
    frame.visual(
        Box((1.42, 0.58, 0.008)),
        origin=Origin(xyz=(-0.095, 0.0, -0.031)),
        material=dark_aluminum,
        name="drain_pan",
    )

    carriage = model.part("slide_carriage")
    # The carriage frame origin is the front hinge/tilt axis.  It rides in the
    # guide rails and translates rearward before/while carrying the glass hinge.
    carriage.visual(
        Cylinder(radius=0.010, length=0.670),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="hinge_tube",
    )
    for suffix, y in (("0", -0.325), ("1", 0.325)):
        carriage.visual(
            Box((0.130, 0.025, 0.018)),
            origin=Origin(xyz=(-0.140, y, 0.003)),
            material=dark_aluminum,
            name=f"front_shoe_{suffix}",
        )
        carriage.visual(
            Box((0.130, 0.025, 0.018)),
            origin=Origin(xyz=(-0.735, y, 0.003)),
            material=dark_aluminum,
            name=f"rear_shoe_{suffix}",
        )
        carriage.visual(
            Box((0.780, 0.012, 0.014)),
            origin=Origin(xyz=(-0.380, y, 0.001)),
            material=dark_aluminum,
            name=f"side_link_{suffix}",
        )

    panel = model.part("glass_panel")
    glass_length = 0.92
    glass_width = 0.64
    glass_thickness = 0.012
    glass_slab = ExtrudeGeometry(
        rounded_rect_profile(glass_length, glass_width, 0.045, corner_segments=12),
        glass_thickness,
        center=True,
    )
    panel.visual(
        mesh_from_geometry(glass_slab, "rounded_glass_panel"),
        # The front edge is set just behind the hinge tube; the panel top is
        # flush with the cassette rail tops in the closed pose.
        origin=Origin(xyz=(-0.480, 0.0, 0.020)),
        material=glass,
        name="glass_slab",
    )
    frit = ExtrudeWithHolesGeometry(
        rounded_rect_profile(glass_length + 0.004, glass_width + 0.004, 0.047, corner_segments=12),
        [rounded_rect_profile(glass_length - 0.130, glass_width - 0.130, 0.030, corner_segments=12)],
        0.002,
        center=True,
    )
    panel.visual(
        mesh_from_geometry(frit, "black_glass_border"),
        origin=Origin(xyz=(-0.480, 0.0, 0.027)),
        material=ceramic,
        name="ceramic_border",
    )
    panel.visual(
        Box((0.050, 0.055, 0.006)),
        origin=Origin(xyz=(-0.012, -0.245, 0.011)),
        material=black_rubber,
        name="front_hinge_lug_0",
    )
    panel.visual(
        Box((0.050, 0.055, 0.006)),
        origin=Origin(xyz=(-0.012, 0.245, 0.011)),
        material=black_rubber,
        name="front_hinge_lug_1",
    )

    slide = model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.450, 0.0, -0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=0.0, upper=0.45),
    )
    model.articulation(
        "carriage_to_panel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=panel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.7, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("cassette_frame")
    carriage = object_model.get_part("slide_carriage")
    panel = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("frame_to_carriage")
    tilt = object_model.get_articulation("carriage_to_panel")

    ctx.allow_overlap(
        carriage,
        panel,
        elem_a="hinge_tube",
        elem_b="front_hinge_lug_0",
        reason="The front glass hinge lug intentionally wraps around the aluminium hinge tube.",
    )
    ctx.allow_overlap(
        carriage,
        panel,
        elem_a="hinge_tube",
        elem_b="front_hinge_lug_1",
        reason="The front glass hinge lug intentionally wraps around the aluminium hinge tube.",
    )

    with ctx.pose({slide: 0.0, tilt: 0.0}):
        for lug in ("front_hinge_lug_0", "front_hinge_lug_1"):
            ctx.expect_gap(
                panel,
                carriage,
                axis="z",
                positive_elem=lug,
                negative_elem="hinge_tube",
                max_penetration=0.004,
                max_gap=0.002,
                name=f"{lug} captures hinge tube vertically",
            )
            ctx.expect_overlap(
                panel,
                carriage,
                axes="xy",
                elem_a=lug,
                elem_b="hinge_tube",
                min_overlap=0.010,
                name=f"{lug} overlaps hinge tube footprint",
            )
        rest_glass_aabb = ctx.part_element_world_aabb(panel, elem="glass_slab")
        rail_aabb = ctx.part_element_world_aabb(frame, elem="guide_rail_0")
        ctx.check(
            "closed glass sits at roof cassette line",
            rest_glass_aabb is not None
            and rail_aabb is not None
            and abs(rest_glass_aabb[1][2] - rail_aabb[1][2]) <= 0.002,
            details=f"glass={rest_glass_aabb}, rail={rail_aabb}",
        )
        ctx.expect_within(
            panel,
            frame,
            axes="y",
            inner_elem="glass_slab",
            outer_elem="front_gasket",
            margin=0.040,
            name="closed glass is centered between side rails",
        )

    rest_pos = ctx.part_world_position(panel)
    rest_rear_z = rest_glass_aabb[1][2] if rest_glass_aabb is not None else None
    with ctx.pose({slide: 0.0, tilt: 0.12}):
        tilted_glass_aabb = ctx.part_element_world_aabb(panel, elem="glass_slab")
        tilted_rear_z = tilted_glass_aabb[1][2] if tilted_glass_aabb is not None else None
        ctx.expect_overlap(
            panel,
            frame,
            axes="y",
            elem_a="glass_slab",
            elem_b="front_gasket",
            min_overlap=0.45,
            name="tilted panel remains centered in aperture",
        )

    with ctx.pose({slide: 0.45, tilt: 0.0}):
        slid_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(
            panel,
            frame,
            axes="x",
            elem_a="glass_slab",
            elem_b="guide_rail_0",
            min_overlap=0.20,
            name="rearward slide remains engaged with guide rail",
        )

    ctx.check(
        "rear edge rises during tilt",
        rest_rear_z is not None and tilted_rear_z is not None and tilted_rear_z > rest_rear_z + 0.045,
        details=f"rest_rear_z={rest_rear_z}, tilted_rear_z={tilted_rear_z}",
    )
    ctx.check(
        "panel slides rearward",
        rest_pos is not None and slid_pos is not None and slid_pos[0] < rest_pos[0] - 0.30,
        details=f"rest={rest_pos}, slid={slid_pos}",
    )

    return ctx.report()


object_model = build_object_model()
