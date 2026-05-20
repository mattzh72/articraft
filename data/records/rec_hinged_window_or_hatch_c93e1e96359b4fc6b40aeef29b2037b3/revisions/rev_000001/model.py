from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hinged_casement_window")

    painted_wood = Material("painted_warm_white", rgba=(0.86, 0.84, 0.76, 1.0))
    sash_paint = Material("painted_sash_white", rgba=(0.93, 0.92, 0.86, 1.0))
    glass = Material("slightly_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))
    brass = Material("aged_brass", rgba=(0.72, 0.55, 0.24, 1.0))
    dark_seal = Material("dark_rubber_gasket", rgba=(0.03, 0.035, 0.035, 1.0))

    frame = model.part("frame")

    # Fixed outer frame: real-size residential casement proportions with a clear
    # center opening, deep jambs, sill, and header.
    frame.visual(
        Box((0.08, 0.10, 1.40)),
        origin=Origin(xyz=(-0.46, 0.0, 0.70)),
        material=painted_wood,
        name="hinge_jamb",
    )
    frame.visual(
        Box((0.08, 0.10, 1.40)),
        origin=Origin(xyz=(0.46, 0.0, 0.70)),
        material=painted_wood,
        name="latch_jamb",
    )
    frame.visual(
        Box((1.00, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
        material=painted_wood,
        name="header",
    )
    frame.visual(
        Box((1.00, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_wood,
        name="sill",
    )
    # Thin dark reveal inside the fixed frame emphasizes the hollow opening and
    # the weather-strip against which the sash closes.
    frame.visual(
        Box((0.018, 0.010, 1.18)),
        origin=Origin(xyz=(-0.411, 0.044, 0.70)),
        material=dark_seal,
        name="hinge_reveal",
    )
    frame.visual(
        Box((0.018, 0.010, 1.18)),
        origin=Origin(xyz=(0.411, 0.044, 0.70)),
        material=dark_seal,
        name="latch_reveal",
    )
    frame.visual(
        Box((0.84, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.044, 1.291)),
        material=dark_seal,
        name="top_reveal",
    )
    frame.visual(
        Box((0.84, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.044, 0.109)),
        material=dark_seal,
        name="bottom_reveal",
    )

    # Stationary leaves and alternating hinge knuckles on the left jamb.  The
    # cylinders are short vertical barrels centered on the sash hinge axis.
    for base_z, tag in ((0.40, "lower"), (1.00, "upper")):
        frame.visual(
            Box((0.062, 0.012, 0.24)),
            origin=Origin(xyz=(-0.469, 0.056, base_z)),
            material=brass,
            name=f"{tag}_fixed_leaf",
        )
        for dz, segment in ((-0.06, "bottom"), (0.06, "top")):
            frame.visual(
                Box((0.018, 0.012, 0.050)),
                origin=Origin(xyz=(-0.435, 0.061, base_z + dz)),
                material=brass,
                name=f"{tag}_{segment}_fixed_bridge",
            )
            frame.visual(
                Cylinder(radius=0.018, length=0.050),
                origin=Origin(xyz=(-0.420, 0.075, base_z + dz)),
                material=brass,
                name=f"{tag}_{segment}_fixed_knuckle",
            )

    sash = model.part("sash")
    # The sash part frame is the vertical hinge line at the mid-height of the
    # window.  At q=0 the glazed sash lies closed in the rectangular frame and
    # extends in local +X away from the hinges.
    sash.visual(
        Box((0.060, 0.040, 1.18)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=sash_paint,
        name="hinge_stile",
    )
    sash.visual(
        Box((0.060, 0.040, 1.18)),
        origin=Origin(xyz=(0.770, 0.0, 0.0)),
        material=sash_paint,
        name="free_stile",
    )
    sash.visual(
        Box((0.800, 0.040, 0.060)),
        origin=Origin(xyz=(0.400, 0.0, 0.560)),
        material=sash_paint,
        name="top_rail",
    )
    sash.visual(
        Box((0.800, 0.040, 0.060)),
        origin=Origin(xyz=(0.400, 0.0, -0.560)),
        material=sash_paint,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.710, 0.012, 1.070)),
        origin=Origin(xyz=(0.400, -0.004, 0.0)),
        material=glass,
        name="glass_pane",
    )
    sash.visual(
        Box((0.705, 0.008, 0.020)),
        origin=Origin(xyz=(0.400, 0.023, 0.535)),
        material=dark_seal,
        name="top_glazing_gasket",
    )
    sash.visual(
        Box((0.705, 0.008, 0.020)),
        origin=Origin(xyz=(0.400, 0.023, -0.535)),
        material=dark_seal,
        name="bottom_glazing_gasket",
    )
    sash.visual(
        Box((0.020, 0.008, 1.035)),
        origin=Origin(xyz=(0.055, 0.023, 0.0)),
        material=dark_seal,
        name="hinge_glazing_gasket",
    )
    sash.visual(
        Box((0.020, 0.008, 1.035)),
        origin=Origin(xyz=(0.735, 0.023, 0.0)),
        material=dark_seal,
        name="free_glazing_gasket",
    )
    # Moving hinge leaves and middle knuckles alternate with the fixed frame
    # knuckles, creating two visible hinge assemblies without interpenetration.
    for local_z, tag in ((-0.30, "lower"), (0.30, "upper")):
        sash.visual(
            Box((0.064, 0.014, 0.24)),
            origin=Origin(xyz=(0.047, -0.020, local_z)),
            material=brass,
            name=f"{tag}_sash_leaf",
        )
        sash.visual(
            Box((0.030, 0.020, 0.050)),
            origin=Origin(xyz=(0.020, -0.003, local_z)),
            material=brass,
            name=f"{tag}_sash_bridge",
        )
        sash.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=brass,
            name=f"{tag}_sash_knuckle",
        )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.420, 0.075, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    handle = model.part("handle")
    # A small brass lever on the free side of the sash.  Its child frame is the
    # pivot center; the lever geometry hangs downward and rotates in the plane
    # of the sash about a short spindle normal to the glass.
    handle.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rose",
    )
    handle.visual(
        Box((0.026, 0.012, 0.170)),
        origin=Origin(xyz=(-0.015, 0.013, -0.085)),
        material=brass,
        name="lever_arm",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(-0.015, 0.023, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="end_grip",
    )

    model.articulation(
        "sash_to_handle",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(xyz=(0.780, 0.027, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.9, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    handle = object_model.get_part("handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    handle_pivot = object_model.get_articulation("sash_to_handle")

    ctx.expect_contact(
        handle,
        sash,
        elem_a="rose",
        elem_b="free_stile",
        contact_tol=0.001,
        name="handle rose seats on the sash stile",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="glass_pane",
        negative_elem="hinge_jamb",
        min_gap=0.004,
        name="closed glass stands proud of the fixed jamb depth",
    )
    ctx.expect_within(
        sash,
        frame,
        axes="xz",
        inner_elem="glass_pane",
        margin=0.55,
        name="glass pane remains inside the rectangular frame outline",
    )

    closed_free_aabb = ctx.part_element_world_aabb(sash, elem="free_stile")
    closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="end_grip")
    with ctx.pose({sash_hinge: 0.85}):
        open_free_aabb = ctx.part_element_world_aabb(sash, elem="free_stile")
    with ctx.pose({handle_pivot: 0.80}):
        rotated_grip_aabb = ctx.part_element_world_aabb(handle, elem="end_grip")

    def aabb_center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    def aabb_center_xz(aabb):
        return None if aabb is None else (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    closed_free_y = aabb_center_y(closed_free_aabb)
    open_free_y = aabb_center_y(open_free_aabb)
    ctx.check(
        "sash revolute joint swings the free side outward",
        closed_free_y is not None and open_free_y is not None and open_free_y > closed_free_y + 0.25,
        details=f"closed_y={closed_free_y}, open_y={open_free_y}",
    )

    closed_grip_xz = aabb_center_xz(closed_grip_aabb)
    rotated_grip_xz = aabb_center_xz(rotated_grip_aabb)
    ctx.check(
        "handle has its own rotating lever pivot",
        closed_grip_xz is not None
        and rotated_grip_xz is not None
        and abs(rotated_grip_xz[0] - closed_grip_xz[0]) > 0.07
        and abs(rotated_grip_xz[1] - closed_grip_xz[1]) > 0.03,
        details=f"closed_xz={closed_grip_xz}, rotated_xz={rotated_grip_xz}",
    )

    return ctx.report()


object_model = build_object_model()
