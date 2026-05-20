from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangulated_prism_along_x(
    width: float, yz_loop: list[tuple[float, float]]
) -> MeshGeometry:
    """Return a closed prism whose cross-section is described in local YZ."""
    geom = MeshGeometry()
    half = width / 2.0

    for x in (-half, half):
        for y, z in yz_loop:
            geom.add_vertex(x, y, z)

    n = len(yz_loop)
    # End caps.
    for i in range(1, n - 1):
        geom.add_face(0, i, i + 1)
        geom.add_face(n, n + i + 1, n + i)

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_sliding_window")

    aluminum = model.material("dark_anodized_aluminum", rgba=(0.20, 0.22, 0.22, 1.0))
    seal = model.material("black_epdm_seal", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("blue_low_e_glass", rgba=(0.50, 0.72, 0.88, 0.38))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))

    frame = model.part("frame")

    # Main thermally-broken aluminum perimeter, sized like a residential slider.
    frame.visual(
        Box((1.66, 0.16, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 1.0625)),
        material=aluminum,
        name="top_rail",
    )
    frame.visual(
        Box((1.66, 0.16, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=aluminum,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.075, 0.16, 1.10)),
        origin=Origin(xyz=(-0.7925, 0.0, 0.550)),
        material=aluminum,
        name="jamb_0",
    )
    frame.visual(
        Box((0.075, 0.16, 1.10)),
        origin=Origin(xyz=(0.7925, 0.0, 0.550)),
        material=aluminum,
        name="jamb_1",
    )

    # Outdoor overhangs shed water away from the joint line.
    frame.visual(
        Box((1.74, 0.080, 0.025)),
        origin=Origin(xyz=(0.0, -0.110, 1.1025)),
        material=aluminum,
        name="drip_cap",
    )
    frame.visual(
        Box((1.74, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.151, 1.075)),
        material=aluminum,
        name="cap_drip_lip",
    )
    sill_nose = _triangulated_prism_along_x(
        1.72,
        [
            (-0.075, 0.095),
            (-0.165, 0.060),
            (-0.165, 0.035),
            (-0.075, 0.035),
        ],
    )
    frame.visual(
        mesh_from_geometry(sill_nose, "sloped_sill_nose"),
        material=aluminum,
        name="sloped_sill",
    )
    frame.visual(
        Box((1.60, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, -0.146, 0.038)),
        material=seal,
        name="drip_kerf",
    )

    # Continuous two-lip channel guides; the sash rides between the lips.
    frame.visual(
        Box((1.44, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.070, 0.120)),
        material=aluminum,
        name="lower_front_guide",
    )
    frame.visual(
        Box((1.44, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.014, 0.120)),
        material=aluminum,
        name="lower_rear_guide",
    )
    frame.visual(
        Box((1.44, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.070, 1.005)),
        material=aluminum,
        name="upper_front_guide",
    )
    frame.visual(
        Box((1.44, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.014, 1.005)),
        material=aluminum,
        name="upper_rear_guide",
    )

    # Replaceable EPDM/brush seals sit just clear of the sliding sash.
    for name, y in (
        ("lower_front_seal", -0.064),
        ("lower_rear_seal", -0.020),
    ):
        frame.visual(
            Box((1.40, 0.006, 0.013)),
            origin=Origin(xyz=(0.0, y, 0.142)),
            material=seal,
            name=name,
        )
    for name, y in (
        ("upper_front_seal", -0.064),
        ("upper_rear_seal", -0.020),
    ):
        frame.visual(
            Box((1.40, 0.006, 0.013)),
            origin=Origin(xyz=(0.0, y, 0.982)),
            material=seal,
            name=name,
        )

    # Rubber end stops are anchored in the track before the sash reaches metal.
    frame.visual(
        Box((0.030, 0.036, 0.090)),
        origin=Origin(xyz=(-0.718, -0.043, 0.145)),
        material=seal,
        name="travel_stop_0",
    )
    frame.visual(
        Box((0.030, 0.036, 0.090)),
        origin=Origin(xyz=(0.538, -0.043, 0.145)),
        material=seal,
        name="travel_stop_1",
    )

    # Fixed rear pane and its retained sash; the sliding sash passes in front.
    frame.visual(
        Box((0.65, 0.006, 0.760)),
        origin=Origin(xyz=(0.380, 0.032, 0.560)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.70, 0.045, 0.050)),
        origin=Origin(xyz=(0.385, 0.030, 0.960)),
        material=aluminum,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.70, 0.045, 0.050)),
        origin=Origin(xyz=(0.385, 0.030, 0.160)),
        material=aluminum,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.050, 0.045, 0.830)),
        origin=Origin(xyz=(0.040, 0.030, 0.560)),
        material=aluminum,
        name="fixed_meeting_stile",
    )
    frame.visual(
        Box((0.050, 0.045, 0.830)),
        origin=Origin(xyz=(0.730, 0.030, 0.560)),
        material=aluminum,
        name="fixed_jamb_stile",
    )
    frame.visual(
        Box((0.018, 0.018, 0.760)),
        origin=Origin(xyz=(0.012, 0.000, 0.560)),
        material=seal,
        name="interlock_seal",
    )

    sliding_sash = model.part("sliding_sash")
    sash_w = 0.720
    sash_h = 0.800
    stile = 0.055
    rail = 0.055
    sash_depth = 0.032

    sliding_sash.visual(
        Box((sash_w, sash_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_h / 2.0 - rail / 2.0)),
        material=aluminum,
        name="sash_top_rail",
    )
    sliding_sash.visual(
        Box((sash_w, sash_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_h / 2.0 + rail / 2.0)),
        material=aluminum,
        name="sash_bottom_rail",
    )
    sliding_sash.visual(
        Box((stile, sash_depth, sash_h)),
        origin=Origin(xyz=(-sash_w / 2.0 + stile / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_stile_0",
    )
    sliding_sash.visual(
        Box((stile, sash_depth, sash_h)),
        origin=Origin(xyz=(sash_w / 2.0 - stile / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="sash_stile_1",
    )
    sliding_sash.visual(
        Box((0.620, 0.006, 0.690)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=glass,
        name="sash_glass",
    )

    # Compression gaskets and interlock fin are captured in the sash extrusion.
    sliding_sash.visual(
        Box((0.660, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.013, 0.327)),
        material=seal,
        name="top_glass_gasket",
    )
    sliding_sash.visual(
        Box((0.660, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.013, -0.327)),
        material=seal,
        name="bottom_glass_gasket",
    )
    sliding_sash.visual(
        Box((0.018, 0.006, 0.690)),
        origin=Origin(xyz=(-0.307, -0.013, 0.0)),
        material=seal,
        name="side_gasket_0",
    )
    sliding_sash.visual(
        Box((0.018, 0.006, 0.690)),
        origin=Origin(xyz=(0.307, -0.013, 0.0)),
        material=seal,
        name="side_gasket_1",
    )
    sliding_sash.visual(
        Box((0.018, 0.012, 0.760)),
        origin=Origin(xyz=(sash_w / 2.0 - 0.018, -0.022, 0.0)),
        material=seal,
        name="meeting_fin",
    )

    # Stainless hardware is compact and mounted through the sash member.
    sliding_sash.visual(
        Box((0.022, 0.013, 0.350)),
        origin=Origin(xyz=(0.325, -0.020, 0.0)),
        material=stainless,
        name="pull_handle",
    )
    for name, z in (("handle_screw_0", -0.145), ("handle_screw_1", 0.145)):
        sliding_sash.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.325, -0.030, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=name,
        )
    sliding_sash.visual(
        Box((0.055, 0.020, 0.025)),
        origin=Origin(xyz=(-0.225, 0.0, -0.410)),
        material=stainless,
        name="roller_0_housing",
    )
    sliding_sash.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.225, 0.0, -0.440), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="roller_0",
    )
    sliding_sash.visual(
        Box((0.055, 0.020, 0.025)),
        origin=Origin(xyz=(0.225, 0.0, -0.410)),
        material=stainless,
        name="roller_1_housing",
    )
    sliding_sash.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.225, 0.0, -0.440), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="roller_1",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(-0.340, -0.043, 0.560)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.500),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sash")

    # The lower rail is captured laterally between both channel lips.
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="sash_bottom_rail",
        negative_elem="lower_front_guide",
        name="front channel clearance",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="lower_rear_guide",
        negative_elem="sash_bottom_rail",
        name="rear channel clearance",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        min_overlap=0.65,
        elem_a="sash_bottom_rail",
        elem_b="lower_front_guide",
        name="sash retained in lower guide",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="x",
        min_gap=0.001,
        max_gap=0.008,
        positive_elem="sash_stile_0",
        negative_elem="travel_stop_0",
        name="closed stop clearance",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="roller_0",
        elem_b="bottom_sill",
        name="roller bears on sill",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.500}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            min_gap=0.002,
            max_gap=0.010,
            positive_elem="sash_bottom_rail",
            negative_elem="lower_front_guide",
            name="extended front clearance",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            min_gap=0.002,
            max_gap=0.012,
            positive_elem="lower_rear_guide",
            negative_elem="sash_bottom_rail",
            name="extended rear clearance",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            min_overlap=0.65,
            elem_a="sash_bottom_rail",
            elem_b="lower_front_guide",
            name="extended guide retention",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            min_gap=0.001,
            max_gap=0.008,
            positive_elem="travel_stop_1",
            negative_elem="sash_stile_1",
            name="open stop clearance",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="roller_1",
            elem_b="bottom_sill",
            name="extended roller bears on sill",
        )
        extended_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash slides rightward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.45,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
