from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FASCIA_WIDTH = 1.10
FASCIA_HEIGHT = 0.45
FRAME_DEPTH = 0.055
FRONT_Y = 0.030
FRAME_CENTER_Y = FRONT_Y - FRAME_DEPTH / 2.0

SLEEVE_WIDTH = 1.02
SLEEVE_HEIGHT = 0.38
SLEEVE_DEPTH = 0.455
SLEEVE_FRONT_Y = -0.008
SLEEVE_CENTER_Y = SLEEVE_FRONT_Y - SLEEVE_DEPTH / 2.0

TOP_RAIL_HEIGHT = 0.070
BOTTOM_RAIL_HEIGHT = 0.070
SIDE_RAIL_WIDTH = 0.080
SIDE_RAIL_HEIGHT = 0.290
CENTER_RAIL_HEIGHT = 0.035

FILTER_MEDIA_WIDTH = 0.880
FILTER_MEDIA_HEIGHT = 0.185
FILTER_MEDIA_DEPTH = 0.050
FILTER_MEDIA_CENTER_Y = -0.015
FILTER_MEDIA_CENTER_Z = 0.045

OUTLET_WIDTH = 0.900
OUTLET_HEIGHT = 0.040
OUTLET_DEPTH = 0.055
OUTLET_CENTER_Y = -0.0175
OUTLET_CENTER_Z = -0.110

DOOR_WIDTH = 0.930
DOOR_HEIGHT = 0.200
DOOR_DEPTH = 0.014
DOOR_CENTER_Y = -DOOR_DEPTH / 2.0
DOOR_HINGE_Z = 0.145
DOOR_RAIL_HEIGHT = 0.018
DOOR_STILE_WIDTH = 0.045
DOOR_STILE_HEIGHT = DOOR_HEIGHT - DOOR_RAIL_HEIGHT
DOOR_SLAT_HEIGHT = 0.013
DOOR_SLAT_WIDTH = DOOR_WIDTH - 2.0 * (DOOR_STILE_WIDTH - 0.004)
DOOR_SLAT_Z = (-0.040, -0.066, -0.092, -0.118, -0.144, -0.170)

FLAP_WIDTH = 0.920
FLAP_HEIGHT = 0.050
FLAP_DEPTH = 0.012
FLAP_CENTER_Y = -FLAP_DEPTH / 2.0
FLAP_HINGE_Z = -0.090


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="through_wall_ptac")

    fascia = model.material("fascia", color=(0.92, 0.92, 0.89))
    fascia_shadow = model.material("fascia_shadow", color=(0.82, 0.83, 0.80))
    charcoal = model.material("charcoal", color=(0.20, 0.22, 0.24))
    filter_gray = model.material("filter_gray", color=(0.56, 0.60, 0.62))

    body = model.part("body")
    body.visual(
        Box((SLEEVE_WIDTH, SLEEVE_DEPTH, SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, SLEEVE_CENTER_Y, 0.0)),
        material=charcoal,
        name="sleeve",
    )
    body.visual(
        Box((FASCIA_WIDTH, FRAME_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, 0.180)),
        material=fascia,
        name="fascia_top",
    )
    body.visual(
        Box((FASCIA_WIDTH, FRAME_DEPTH, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, -0.180)),
        material=fascia,
        name="fascia_bottom",
    )
    for side, x in (("0", -(FASCIA_WIDTH / 2.0 - SIDE_RAIL_WIDTH / 2.0)), ("1", FASCIA_WIDTH / 2.0 - SIDE_RAIL_WIDTH / 2.0)):
        body.visual(
            Box((SIDE_RAIL_WIDTH, FRAME_DEPTH, SIDE_RAIL_HEIGHT)),
            origin=Origin(xyz=(x, FRAME_CENTER_Y, 0.0)),
            material=fascia,
            name=f"fascia_side_{side}",
        )
    body.visual(
        Box((FASCIA_WIDTH, FRAME_DEPTH, CENTER_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, -0.0725)),
        material=fascia_shadow,
        name="outlet_lip",
    )
    body.visual(
        Box((FILTER_MEDIA_WIDTH, FILTER_MEDIA_DEPTH, FILTER_MEDIA_HEIGHT)),
        origin=Origin(xyz=(0.0, FILTER_MEDIA_CENTER_Y, FILTER_MEDIA_CENTER_Z)),
        material=filter_gray,
        name="filter_media",
    )
    body.visual(
        Box((OUTLET_WIDTH, OUTLET_DEPTH, OUTLET_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTLET_CENTER_Y, OUTLET_CENTER_Z)),
        material=charcoal,
        name="outlet_duct",
    )
    body.visual(
        Box((0.190, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, -0.203)),
        material=fascia_shadow,
        name="badge_strip",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((DOOR_WIDTH, DOOR_DEPTH, DOOR_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, DOOR_CENTER_Y, -DOOR_RAIL_HEIGHT / 2.0)),
        material=fascia,
        name="door_top",
    )
    filter_door.visual(
        Box((DOOR_WIDTH, DOOR_DEPTH, DOOR_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, DOOR_CENTER_Y, -(DOOR_HEIGHT - DOOR_RAIL_HEIGHT / 2.0))),
        material=fascia,
        name="door_bottom",
    )
    for side, x in (("0", -(DOOR_WIDTH / 2.0 - DOOR_STILE_WIDTH / 2.0)), ("1", DOOR_WIDTH / 2.0 - DOOR_STILE_WIDTH / 2.0)):
        filter_door.visual(
            Box((DOOR_STILE_WIDTH, DOOR_DEPTH, DOOR_STILE_HEIGHT)),
            origin=Origin(xyz=(x, DOOR_CENTER_Y, -DOOR_STILE_HEIGHT / 2.0)),
            material=fascia,
            name=f"door_stile_{side}",
        )
    for index, z in enumerate(DOOR_SLAT_Z):
        filter_door.visual(
            Box((DOOR_SLAT_WIDTH, DOOR_DEPTH * 0.75, DOOR_SLAT_HEIGHT)),
            origin=Origin(xyz=(0.0, DOOR_CENTER_Y, z)),
            material=fascia_shadow,
            name=f"door_slat_{index}",
        )
    filter_door.visual(
        Box((0.140, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, -(DOOR_HEIGHT - 0.018))),
        material=fascia_shadow,
        name="door_pull",
    )

    discharge_flap = model.part("discharge_flap")
    discharge_flap.visual(
        Box((FLAP_WIDTH, FLAP_DEPTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, FLAP_CENTER_Y, -FLAP_HEIGHT / 2.0)),
        material=fascia,
        name="flap_panel",
    )
    discharge_flap.visual(
        Box((FLAP_WIDTH, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, -(FLAP_HEIGHT - 0.005))),
        material=fascia_shadow,
        name="flap_edge",
    )

    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.22),
    )
    model.articulation(
        "body_to_discharge_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=discharge_flap,
        origin=Origin(xyz=(0.0, FRONT_Y, FLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    filter_door = object_model.get_part("filter_door")
    discharge_flap = object_model.get_part("discharge_flap")
    door_hinge = object_model.get_articulation("body_to_filter_door")
    flap_hinge = object_model.get_articulation("body_to_discharge_flap")

    aabb = ctx.part_world_aabb(body)
    ctx.check("ptac_body_aabb_present", aabb is not None, details=f"aabb={aabb}")
    if aabb is not None:
        min_pt, max_pt = aabb
        width = float(max_pt[0] - min_pt[0])
        depth = float(max_pt[1] - min_pt[1])
        height = float(max_pt[2] - min_pt[2])
        ctx.check("ptac_width_is_hotel_scale", 1.00 <= width <= 1.12, details=f"width={width}")
        ctx.check("ptac_depth_is_through_wall_scale", 0.44 <= depth <= 0.50, details=f"depth={depth}")
        ctx.check("ptac_height_is_hotel_scale", 0.40 <= height <= 0.46, details=f"height={height}")

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            filter_door,
            body,
            axis="y",
            positive_elem="door_bottom",
            negative_elem="filter_media",
            min_gap=0.003,
            max_gap=0.012,
            name="closed filter door sits just ahead of the filter media",
        )
        ctx.expect_overlap(
            filter_door,
            body,
            axes="xz",
            elem_b="filter_media",
            min_overlap=0.16,
            name="closed filter door covers the intake filter area",
        )
        ctx.expect_overlap(
            filter_door,
            body,
            axes="xz",
            elem_a="door_bottom",
            elem_b="filter_media",
            min_overlap=0.008,
            name="lower edge of the closed filter door still overlaps the filter zone",
        )
        ctx.expect_gap(
            discharge_flap,
            body,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="outlet_duct",
            min_gap=0.004,
            max_gap=0.015,
            name="closed discharge flap rests just ahead of the outlet duct",
        )
        ctx.expect_overlap(
            discharge_flap,
            body,
            axes="xz",
            elem_a="flap_panel",
            elem_b="outlet_duct",
            min_overlap=0.035,
            name="closed discharge flap spans the outlet opening",
        )
        closed_door = ctx.part_element_world_aabb(filter_door, elem="door_bottom")
        closed_flap = ctx.part_element_world_aabb(discharge_flap, elem="flap_edge")

    with ctx.pose({door_hinge: 1.05, flap_hinge: 0.72}):
        ctx.expect_gap(
            filter_door,
            body,
            axis="y",
            positive_elem="door_bottom",
            negative_elem="filter_media",
            min_gap=0.14,
            name="filter door swings well clear of the intake opening",
        )
        ctx.expect_gap(
            discharge_flap,
            body,
            axis="y",
            positive_elem="flap_edge",
            negative_elem="outlet_duct",
            min_gap=0.028,
            name="discharge flap opens forward from the outlet lip",
        )
        open_door = ctx.part_element_world_aabb(filter_door, elem="door_bottom")
        open_flap = ctx.part_element_world_aabb(discharge_flap, elem="flap_edge")

    ctx.check(
        "filter door opens outward",
        closed_door is not None
        and open_door is not None
        and float(open_door[0][1]) > float(closed_door[0][1]) + 0.12,
        details=f"closed={closed_door}, open={open_door}",
    )
    ctx.check(
        "discharge flap opens outward",
        closed_flap is not None
        and open_flap is not None
        and float(open_flap[0][1]) > float(closed_flap[0][1]) + 0.020,
        details=f"closed={closed_flap}, open={open_flap}",
    )

    return ctx.report()


object_model = build_object_model()
