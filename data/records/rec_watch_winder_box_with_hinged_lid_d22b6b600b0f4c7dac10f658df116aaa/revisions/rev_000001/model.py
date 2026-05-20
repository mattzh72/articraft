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
    model = ArticulatedObject(name="weatherproof_watch_winder_box")

    polymer = model.material("uv_stabilized_charcoal_polymer", rgba=(0.035, 0.040, 0.045, 1.0))
    rubber = model.material("black_epdm_gasket", rgba=(0.005, 0.006, 0.006, 1.0))
    stainless = model.material("brushed_316_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    velvet = model.material("deep_navy_velvet", rgba=(0.01, 0.018, 0.055, 1.0))
    cushion = model.material("matte_black_padded_cushion", rgba=(0.015, 0.014, 0.012, 1.0))
    glass = model.material("smoked_sealed_polycarbonate", rgba=(0.28, 0.42, 0.55, 0.38))
    gold = model.material("warm_watch_bezel", rgba=(0.92, 0.70, 0.32, 1.0))

    # Overall scale: a compact single-watch outdoor winder, wide enough for a
    # presentation insert and rugged enough to read as a weatherproof enclosure.
    outer_x = 0.260  # rear-to-front depth
    outer_y = 0.340  # width
    wall = 0.016
    floor = 0.018
    wall_h = 0.102
    base_top = floor + wall_h

    hinge_x = -outer_x / 2.0 - 0.014
    hinge_z = base_top + 0.006

    case = model.part("case")

    # Hollow presentation box: floor plus four structural walls, not a solid
    # block.  The wall tops carry a real gasket ledge.
    case.visual(
        Box((outer_x, outer_y, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=polymer,
        name="floor_pan",
    )
    case.visual(
        Box((wall, outer_y, wall_h)),
        origin=Origin(xyz=(outer_x / 2.0 - wall / 2.0, 0.0, floor + wall_h / 2.0)),
        material=polymer,
        name="front_wall",
    )
    case.visual(
        Box((wall, outer_y, wall_h)),
        origin=Origin(xyz=(-outer_x / 2.0 + wall / 2.0, 0.0, floor + wall_h / 2.0)),
        material=polymer,
        name="rear_wall",
    )
    case.visual(
        Box((outer_x - 2.0 * wall, wall, wall_h)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - wall / 2.0, floor + wall_h / 2.0)),
        material=polymer,
        name="side_wall_0",
    )
    case.visual(
        Box((outer_x - 2.0 * wall, wall, wall_h)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + wall / 2.0, floor + wall_h / 2.0)),
        material=polymer,
        name="side_wall_1",
    )

    # Raised inner presentation insert and a proud EPDM compression gasket on
    # the box rim.  The four gasket strips touch at corners and sit directly on
    # the structural walls.
    case.visual(
        Box((0.205, 0.250, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, floor + 0.005)),
        material=velvet,
        name="velvet_insert",
    )
    case.visual(
        Box((wall - 0.002, outer_y - 0.010, 0.006)),
        origin=Origin(xyz=(outer_x / 2.0 - wall / 2.0, 0.0, base_top + 0.003)),
        material=rubber,
        name="front_gasket",
    )
    case.visual(
        Box((wall - 0.002, outer_y - 0.010, 0.006)),
        origin=Origin(xyz=(-outer_x / 2.0 + wall / 2.0, 0.0, base_top + 0.003)),
        material=rubber,
        name="rear_gasket",
    )
    case.visual(
        Box((outer_x - 2.0 * wall, wall - 0.002, 0.006)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - wall / 2.0, base_top + 0.003)),
        material=rubber,
        name="side_gasket_0",
    )
    case.visual(
        Box((outer_x - 2.0 * wall, wall - 0.002, 0.006)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + wall / 2.0, base_top + 0.003)),
        material=rubber,
        name="side_gasket_1",
    )

    # Exterior feet make the box read as an anchored presentation case rather
    # than a featureless block.
    for i, (fx, fy) in enumerate(
        (
            (-0.090, -0.125),
            (-0.090, 0.125),
            (0.090, -0.125),
            (0.090, 0.125),
        )
    ):
        case.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(fx, fy, -0.003)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    # Corrosion-resistant rear hinge hardware.  The parent barrels sit on two
    # fixed leaves, leaving a clear middle knuckle for the lid.
    for i, y in enumerate((-0.115, 0.115)):
        case.visual(
            Box((0.030, 0.082, 0.006)),
            origin=Origin(xyz=(hinge_x + 0.004, y, base_top - 0.001)),
            material=stainless,
            name=f"case_hinge_leaf_{i}",
        )
        case.visual(
            Cylinder(radius=0.010, length=0.072),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"case_hinge_barrel_{i}",
        )
        for j, sy in enumerate((-0.024, 0.024)):
            case.visual(
                Cylinder(radius=0.0042, length=0.0025),
                origin=Origin(xyz=(hinge_x + 0.004, y + sy, base_top + 0.0033)),
                material=stainless,
                name=f"case_hinge_screw_{i}_{j}",
            )

    # Fixed front latch base: lower than the lid keeper so the closed lid can
    # articulate without a hidden collision.
    case.visual(
        Box((0.006, 0.072, 0.046)),
        origin=Origin(xyz=(outer_x / 2.0 + 0.003, 0.0, 0.061)),
        material=stainless,
        name="latch_base_plate",
    )
    case.visual(
        Box((0.012, 0.050, 0.008)),
        origin=Origin(xyz=(outer_x / 2.0 + 0.009, 0.0, 0.086)),
        material=stainless,
        name="latch_pull_tab",
    )

    # Internal cradle support points: two floor-mounted pedestals with bearing
    # collars.  The rotating child axle is intentionally captured inside these
    # collars and is documented with scoped overlap allowances in run_tests().
    axis_z = 0.070
    bearing_y = 0.082
    bearing_r = 0.018
    pylon_h = axis_z - bearing_r - floor
    for i, y in enumerate((-bearing_y, bearing_y)):
        case.visual(
            Box((0.044, 0.026, pylon_h)),
            origin=Origin(xyz=(0.0, y, floor + pylon_h / 2.0)),
            material=polymer,
            name=f"cradle_pylon_{i}",
        )
        case.visual(
            Box((0.052, 0.030, 0.006)),
            origin=Origin(xyz=(0.0, y, axis_z - bearing_r - 0.003)),
            material=polymer,
            name=f"bearing_saddle_{i}",
        )
        case.visual(
            Cylinder(radius=bearing_r, length=0.020),
            origin=Origin(xyz=(0.0, y, axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"bearing_collar_{i}",
        )

    # Hinged lid.  Its local frame is the hinge line; at q=0 it is closed and
    # the panel extends along local +X.  A sealed smoked window keeps the
    # internal winder visible even in the weatherproof closed pose.
    lid = model.part("lid")
    lid_depth = outer_x + 0.038
    lid_width = outer_y + 0.034
    lid_min_x = -0.006
    lid_center_x = lid_min_x + lid_depth / 2.0
    frame = 0.035
    top_th = 0.024
    top_z = 0.016
    side_y = lid_width / 2.0 - frame / 2.0
    front_x = lid_min_x + lid_depth - frame / 2.0
    rear_x = lid_min_x + frame / 2.0

    lid.visual(
        Box((frame, lid_width, top_th)),
        origin=Origin(xyz=(front_x, 0.0, top_z)),
        material=polymer,
        name="front_top_frame",
    )
    lid.visual(
        Box((frame, 0.144, top_th)),
        origin=Origin(xyz=(rear_x, 0.0, top_z)),
        material=polymer,
        name="rear_top_frame",
    )
    lid.visual(
        Box((lid_depth - 2.0 * frame, frame, top_th)),
        origin=Origin(xyz=(lid_center_x, side_y, top_z)),
        material=polymer,
        name="side_top_frame_0",
    )
    lid.visual(
        Box((lid_depth - 2.0 * frame, frame, top_th)),
        origin=Origin(xyz=(lid_center_x, -side_y, top_z)),
        material=polymer,
        name="side_top_frame_1",
    )
    lid.visual(
        Box((lid_depth - 2.0 * frame + 0.010, lid_width - 2.0 * frame + 0.010, 0.006)),
        origin=Origin(xyz=(lid_center_x, 0.0, top_z)),
        material=glass,
        name="sealed_window",
    )

    # Downturned skirt and drip bead overhang the case sides so rain sheds
    # outward instead of running directly onto the seal.
    skirt_h = 0.034
    skirt_z = 0.004 - skirt_h / 2.0
    lid.visual(
        Box((0.014, lid_width, skirt_h)),
        origin=Origin(xyz=(lid_min_x + lid_depth - 0.007, 0.0, skirt_z)),
        material=polymer,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_depth, 0.014, skirt_h)),
        origin=Origin(xyz=(lid_center_x, lid_width / 2.0 - 0.007, skirt_z)),
        material=polymer,
        name="side_skirt_0",
    )
    lid.visual(
        Box((lid_depth, 0.014, skirt_h)),
        origin=Origin(xyz=(lid_center_x, -lid_width / 2.0 + 0.007, skirt_z)),
        material=polymer,
        name="side_skirt_1",
    )
    lid.visual(
        Box((0.018, lid_width - 0.035, 0.008)),
        origin=Origin(xyz=(lid_min_x + lid_depth - 0.004, 0.0, -0.034)),
        material=rubber,
        name="front_drip_bead",
    )
    lid.visual(
        Box((0.028, lid_width, 0.012)),
        origin=Origin(xyz=(lid_min_x - 0.006, 0.0, 0.028)),
        material=polymer,
        name="hinge_rain_shield",
    )

    # Compression lands under the lid exactly meet the EPDM gasket at the
    # closed pose.
    lid.visual(
        Box((wall - 0.002, outer_y - 0.010, 0.004)),
        origin=Origin(xyz=(outer_x / 2.0 - wall / 2.0 - hinge_x, 0.0, 0.002)),
        material=rubber,
        name="front_compression_land",
    )
    lid.visual(
        Box((wall - 0.002, outer_y - 0.010, 0.004)),
        origin=Origin(xyz=(-outer_x / 2.0 + wall / 2.0 - hinge_x, 0.0, 0.002)),
        material=rubber,
        name="rear_compression_land",
    )
    lid.visual(
        Box((outer_x - 2.0 * wall, wall - 0.002, 0.004)),
        origin=Origin(xyz=(-hinge_x, outer_y / 2.0 - wall / 2.0, 0.002)),
        material=rubber,
        name="side_compression_land_0",
    )
    lid.visual(
        Box((outer_x - 2.0 * wall, wall - 0.002, 0.004)),
        origin=Origin(xyz=(-hinge_x, -outer_y / 2.0 + wall / 2.0, 0.002)),
        material=rubber,
        name="side_compression_land_1",
    )

    # Lid-side hinge knuckle and front keeper.  The middle knuckle is visibly
    # supported by a metal leaf tied into the lid frame.
    lid.visual(
        Box((0.050, 0.126, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.002)),
        material=stainless,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.004, 0.062, 0.022)),
        origin=Origin(xyz=(lid_min_x + lid_depth + 0.002, 0.0, -0.010)),
        material=stainless,
        name="latch_keeper",
    )
    for i, y in enumerate((-0.070, 0.070)):
        lid.visual(
            Cylinder(radius=0.004, length=0.0025),
            origin=Origin(xyz=(rear_x, y, top_z + top_th / 2.0 + 0.00125)),
            material=stainless,
            name=f"lid_frame_screw_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    # Rotating cradle: axle, padded drum, visible watch, and an off-axis marker
    # so spin motion is obvious.  The part frame lies on the supported spin
    # axis between the two fixed bearing collars.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.006, length=0.198),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cushion,
        name="padded_drum",
    )
    cradle.visual(
        Box((0.020, 0.104, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=rubber,
        name="watch_strap",
    )
    cradle.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=gold,
        name="watch_face",
    )

    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    spin = object_model.get_articulation("cradle_spin")

    # The axle is intentionally captured inside simplified solid bearing
    # collars.  These are small local shaft/bushing overlaps, not broad hidden
    # collisions.
    for i in (0, 1):
        ctx.allow_overlap(
            case,
            cradle,
            elem_a=f"bearing_collar_{i}",
            elem_b="axle",
            reason="The rotating cradle axle is intentionally represented as captured inside a stainless bearing collar.",
        )
        ctx.expect_within(
            cradle,
            case,
            axes="xz",
            inner_elem="axle",
            outer_elem=f"bearing_collar_{i}",
            margin=0.0,
            name=f"axle centered in bearing {i}",
        )
        ctx.expect_overlap(
            cradle,
            case,
            axes="y",
            elem_a="axle",
            elem_b=f"bearing_collar_{i}",
            min_overlap=0.014,
            name=f"axle retained through bearing {i}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            case,
            elem_a="front_compression_land",
            elem_b="front_gasket",
            contact_tol=1e-5,
            name="front seal is compressed at closed lid",
        )
        ctx.expect_contact(
            lid,
            case,
            elem_a="rear_compression_land",
            elem_b="rear_gasket",
            contact_tol=1e-5,
            name="rear seal is compressed at closed lid",
        )
        ctx.expect_contact(
            lid,
            case,
            elem_a="side_compression_land_0",
            elem_b="side_gasket_0",
            contact_tol=1e-5,
            name="side seal 0 is compressed at closed lid",
        )
        ctx.expect_contact(
            lid,
            case,
            elem_a="side_compression_land_1",
            elem_b="side_gasket_1",
            contact_tol=1e-5,
            name="side seal 1 is compressed at closed lid",
        )

        closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    with ctx.pose({lid_hinge: 1.35}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    if closed_front is not None and open_front is not None:
        closed_z = (closed_front[0][2] + closed_front[1][2]) / 2.0
        open_z = (open_front[0][2] + open_front[1][2]) / 2.0
        ctx.check(
            "lid hinge raises front overhang",
            open_z > closed_z + 0.10,
            details=f"closed_z={closed_z:.3f}, open_z={open_z:.3f}",
        )
    else:
        ctx.fail("lid hinge raises front overhang", "front skirt AABB was unavailable")

    with ctx.pose({spin: 0.0}):
        face_rest = ctx.part_element_world_aabb(cradle, elem="watch_face")
    with ctx.pose({spin: math.pi / 2.0}):
        face_quarter = ctx.part_element_world_aabb(cradle, elem="watch_face")

    if face_rest is not None and face_quarter is not None:
        rest_center = tuple((face_rest[0][k] + face_rest[1][k]) / 2.0 for k in range(3))
        quarter_center = tuple((face_quarter[0][k] + face_quarter[1][k]) / 2.0 for k in range(3))
        ctx.check(
            "cradle spin moves watch face around supported axle",
            quarter_center[0] > rest_center[0] + 0.025
            and quarter_center[2] < rest_center[2] - 0.025,
            details=f"rest={rest_center}, quarter={quarter_center}",
        )
    else:
        ctx.fail("cradle spin moves watch face around supported axle", "watch face AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
