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
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_face = Material("painted_equipment_gray", color=(0.33, 0.38, 0.40, 1.0))
    raised_frame = Material("dark_raised_frame", color=(0.16, 0.18, 0.18, 1.0))
    stop_lip = Material("black_stop_lip", color=(0.03, 0.035, 0.035, 1.0))
    door_paint = Material("safety_yellow_door", color=(0.86, 0.62, 0.13, 1.0))
    hinge_steel = Material("brushed_hinge_steel", color=(0.62, 0.63, 0.60, 1.0))
    latch_dark = Material("black_latch_hardware", color=(0.02, 0.02, 0.018, 1.0))

    outer_w = 1.10
    outer_h = 1.20
    opening_w = 0.54
    opening_h = 0.72
    sheet_t = 0.036
    frame_bar = 0.055

    face = model.part("equipment_face")

    # Equipment front sheet with a real rectangular through-opening, represented
    # as connected sheet regions around the cutout.
    side_w = (outer_w - opening_w) / 2.0
    cap_h = (outer_h - opening_h) / 2.0
    face_y = -0.017
    face.visual(
        Box((side_w, sheet_t, outer_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + side_w / 2.0), face_y, 0.0)),
        material=painted_face,
        name="face_left_sheet",
    )
    face.visual(
        Box((side_w, sheet_t, outer_h)),
        origin=Origin(xyz=((opening_w / 2.0 + side_w / 2.0), face_y, 0.0)),
        material=painted_face,
        name="face_right_sheet",
    )
    face.visual(
        Box((outer_w, sheet_t, cap_h)),
        origin=Origin(xyz=(0.0, face_y, opening_h / 2.0 + cap_h / 2.0)),
        material=painted_face,
        name="face_top_sheet",
    )
    face.visual(
        Box((outer_w, sheet_t, cap_h)),
        origin=Origin(xyz=(0.0, face_y, -(opening_h / 2.0 + cap_h / 2.0))),
        material=painted_face,
        name="face_bottom_sheet",
    )

    # Raised outer frame around the opening.
    frame_y = 0.013
    face.visual(
        Box((frame_bar, 0.026, opening_h + 2.0 * frame_bar)),
        origin=Origin(xyz=(-(opening_w / 2.0 + frame_bar / 2.0), frame_y, 0.0)),
        material=raised_frame,
        name="frame_hinge_stile",
    )
    face.visual(
        Box((frame_bar, 0.026, opening_h + 2.0 * frame_bar)),
        origin=Origin(xyz=((opening_w / 2.0 + frame_bar / 2.0), frame_y, 0.0)),
        material=raised_frame,
        name="frame_latch_stile",
    )
    face.visual(
        Box((opening_w + 2.0 * frame_bar, 0.026, frame_bar)),
        origin=Origin(xyz=(0.0, frame_y, opening_h / 2.0 + frame_bar / 2.0)),
        material=raised_frame,
        name="frame_top_rail",
    )
    face.visual(
        Box((opening_w + 2.0 * frame_bar, 0.026, frame_bar)),
        origin=Origin(xyz=(0.0, frame_y, -(opening_h / 2.0 + frame_bar / 2.0))),
        material=raised_frame,
        name="frame_bottom_rail",
    )

    # Fixed stop flange / secondary lip just inside the opening.  It is proud of
    # the sheet but below the closed door's rear face so the panel closes onto it.
    lip_w = 0.018
    lip_y = 0.0065
    face.visual(
        Box((lip_w, 0.013, opening_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 - lip_w / 2.0), lip_y, 0.0)),
        material=stop_lip,
        name="stop_lip_hinge",
    )
    face.visual(
        Box((lip_w, 0.013, opening_h)),
        origin=Origin(xyz=((opening_w / 2.0 - lip_w / 2.0), lip_y, 0.0)),
        material=stop_lip,
        name="stop_lip_latch",
    )
    face.visual(
        Box((opening_w, 0.013, lip_w)),
        origin=Origin(xyz=(0.0, lip_y, opening_h / 2.0 - lip_w / 2.0)),
        material=stop_lip,
        name="stop_lip_top",
    )
    face.visual(
        Box((opening_w, 0.013, lip_w)),
        origin=Origin(xyz=(0.0, lip_y, -(opening_h / 2.0 - lip_w / 2.0))),
        material=stop_lip,
        name="stop_lip_bottom",
    )

    hinge_x = -0.282
    hinge_y = 0.034
    hinge_r = 0.010

    # Fixed hinge leaf and alternate hinge knuckles on the equipment face.
    face.visual(
        Box((0.040, 0.008, 0.74)),
        origin=Origin(xyz=(hinge_x - 0.027, 0.026, 0.0)),
        material=hinge_steel,
        name="fixed_hinge_leaf",
    )
    for name, zc, length in (
        ("fixed_hinge_lower", -0.2775, 0.145),
        ("fixed_hinge_center", 0.0, 0.120),
        ("fixed_hinge_upper", 0.2775, 0.145),
    ):
        face.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=hinge_steel,
            name=name,
        )
    face.visual(
        Cylinder(radius=0.004, length=0.735),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    # A small fixed keeper on the latch side makes the opposite edge read as the
    # closed latch edge without adding a second articulation.
    face.visual(
        Box((0.030, 0.012, 0.115)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.019, 0.032, 0.0)),
        material=hinge_steel,
        name="latch_keeper",
    )

    door = model.part("door")
    door_w = 0.520
    door_h = 0.700
    door_t = 0.018
    door_x0 = 0.018
    door_y = 0.030
    door_center_x = door_x0 + door_w / 2.0

    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_center_x, door_y - hinge_y, 0.0)),
        material=door_paint,
        name="door_slab",
    )

    # Folded/raised perimeter ribs on the door face.
    rib_y = door_y + door_t / 2.0 + 0.004
    door.visual(
        Box((0.026, 0.008, door_h - 0.050)),
        origin=Origin(xyz=(door_x0 + 0.018, rib_y - hinge_y, 0.0)),
        material=door_paint,
        name="door_hinge_stile",
    )
    door.visual(
        Box((0.026, 0.008, door_h - 0.050)),
        origin=Origin(xyz=(door_x0 + door_w - 0.018, rib_y - hinge_y, 0.0)),
        material=door_paint,
        name="door_latch_stile",
    )
    door.visual(
        Box((door_w - 0.030, 0.008, 0.024)),
        origin=Origin(xyz=(door_center_x, rib_y - hinge_y, door_h / 2.0 - 0.022)),
        material=door_paint,
        name="door_top_rail",
    )
    door.visual(
        Box((door_w - 0.030, 0.008, 0.024)),
        origin=Origin(xyz=(door_center_x, rib_y - hinge_y, -(door_h / 2.0 - 0.022))),
        material=door_paint,
        name="door_bottom_rail",
    )

    # Moving hinge leaf, two moving knuckles, and local lugs.  The knuckles
    # alternate with the fixed knuckles, so the door remains a single revolute
    # panel without visual interpenetration.
    door.visual(
        Box((0.030, 0.008, 0.640)),
        origin=Origin(xyz=(0.033, 0.031 - hinge_y, 0.0)),
        material=hinge_steel,
        name="moving_hinge_leaf",
    )
    for name, zc, length in (
        ("moving_hinge_lower", -0.1375, 0.125),
        ("moving_hinge_upper", 0.1375, 0.125),
    ):
        door.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_steel,
            name=name,
        )
        door.visual(
            Box((0.012, 0.008, length - 0.012)),
            origin=Origin(xyz=(0.014, 0.0, zc)),
            material=hinge_steel,
            name=f"{name}_lug",
        )

    # Opposite latch-edge treatment: a darker edge strip and visible fasteners
    # on the same panel part, aligned with the fixed keeper on the frame.
    door.visual(
        Box((0.018, 0.010, 0.560)),
        origin=Origin(xyz=(door_x0 + door_w - 0.028, rib_y + 0.003 - hinge_y, 0.0)),
        material=latch_dark,
        name="latch_edge_strip",
    )
    for index, zc in enumerate((-0.18, 0.18)):
        door.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(
                xyz=(door_x0 + door_w - 0.028, 0.053 - hinge_y, zc),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_steel,
            name=f"latch_fastener_{index}",
        )

    model.articulation(
        "face_to_door",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("face_to_door")

    for moving_knuckle in ("moving_hinge_lower", "moving_hinge_upper"):
        ctx.allow_overlap(
            face,
            door,
            elem_a="hinge_pin",
            elem_b=moving_knuckle,
            reason="The fixed hinge pin intentionally passes through the moving hinge knuckle proxy.",
        )
        ctx.expect_within(
            face,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=moving_knuckle,
            margin=0.001,
            name=f"{moving_knuckle} captures the hinge pin radially",
        )
        ctx.expect_overlap(
            face,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=moving_knuckle,
            min_overlap=0.10,
            name=f"{moving_knuckle} has retained pin length",
        )

    ctx.check(
        "door has one vertical revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            face,
            axis="x",
            positive_elem="door_slab",
            negative_elem="frame_hinge_stile",
            min_gap=0.003,
            max_gap=0.020,
            name="closed door clears the hinge-side frame",
        )
        ctx.expect_gap(
            face,
            door,
            axis="x",
            positive_elem="frame_latch_stile",
            negative_elem="door_slab",
            min_gap=0.003,
            max_gap=0.025,
            name="closed door clears the latch-side frame",
        )
        ctx.expect_gap(
            face,
            door,
            axis="z",
            positive_elem="frame_top_rail",
            negative_elem="door_slab",
            min_gap=0.005,
            max_gap=0.020,
            name="closed door clears the top frame rail",
        )
        ctx.expect_gap(
            door,
            face,
            axis="z",
            positive_elem="door_slab",
            negative_elem="frame_bottom_rail",
            min_gap=0.005,
            max_gap=0.020,
            name="closed door clears the bottom frame rail",
        )
        ctx.expect_gap(
            door,
            face,
            axis="y",
            positive_elem="door_slab",
            negative_elem="stop_lip_latch",
            min_gap=0.006,
            max_gap=0.012,
            name="closed door sits just proud of the stop lip",
        )
        ctx.expect_overlap(
            door,
            face,
            axes="z",
            elem_a="door_slab",
            elem_b="stop_lip_latch",
            min_overlap=0.50,
            name="door covers the stop lip vertically",
        )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            door,
            face,
            axis="y",
            positive_elem="door_latch_stile",
            negative_elem="stop_lip_latch",
            min_gap=0.25,
            name="opened panel swings outward from the equipment face",
        )
    ctx.check(
        "opening pose moves the latch edge outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > rest_aabb[1][1] + 0.25,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
