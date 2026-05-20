from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_OUTER_RADIUS = 0.108
HOUSING_INNER_RADIUS = 0.102
HOUSING_HEIGHT = 0.318
HOUSING_FLOOR = 0.005

CAP_OUTER_RADIUS = 0.112
CAP_INNER_RADIUS = 0.086
CAP_HEIGHT = 0.018

FILTER_OUTER_RADIUS = 0.079
FILTER_INNER_RADIUS = 0.063
FILTER_BODY_HEIGHT = 0.230
FILTER_SEAT_Z = 0.035
FILTER_TRAVEL = 0.130


def _annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height)
    return outer.cut(inner)


def _housing_shape() -> cq.Workplane:
    lower_ring = _annulus(HOUSING_OUTER_RADIUS, 0.096, 0.024)
    foot_ring = _annulus(HOUSING_OUTER_RADIUS + 0.004, 0.094, 0.008)
    top_band = _annulus(HOUSING_OUTER_RADIUS, HOUSING_INNER_RADIUS, 0.092).translate(
        (0.0, 0.0, HOUSING_HEIGHT - 0.092)
    )
    filter_seat = _annulus(0.090, 0.060, 0.012).translate((0.0, 0.0, FILTER_SEAT_Z - 0.012))
    body = lower_ring.union(foot_ring).union(top_band).union(filter_seat)

    intake_slat = cq.Workplane("XY").box(0.016, 0.008, 0.206).translate((0.102, 0.0, 0.125))
    for index in range(24):
        angle_deg = 15.0 * index
        body = body.union(
            intake_slat.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )

    support_rib = cq.Workplane("XY").box(0.018, 0.012, FILTER_SEAT_Z).translate(
        (0.096, 0.0, FILTER_SEAT_Z * 0.5)
    )
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        body = body.union(
            support_rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )

    return body.combine()


def _cap_shape() -> cq.Workplane:
    ring = _annulus(CAP_OUTER_RADIUS, CAP_INNER_RADIUS, CAP_HEIGHT)
    top_ridge = _annulus(CAP_OUTER_RADIUS - 0.002, CAP_INNER_RADIUS + 0.012, 0.004).translate(
        (0.0, 0.0, CAP_HEIGHT)
    )
    ring = ring.union(top_ridge)

    lug = cq.Workplane("XY").box(0.018, 0.007, 0.004).translate((0.087, 0.0, -0.002))
    for index in range(3):
        angle_deg = 120.0 * index + 18.0
        ring = ring.union(lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    ring = ring.combine()
    ring = ring.edges("|Z").fillet(0.0018)
    ring = ring.edges(">Z").fillet(0.0015)
    return ring


def _filter_shape() -> cq.Workplane:
    cartridge = _annulus(FILTER_OUTER_RADIUS, FILTER_INNER_RADIUS, FILTER_BODY_HEIGHT)

    pleat_cutter = cq.Workplane("XY").box(
        0.010,
        0.0036,
        FILTER_BODY_HEIGHT - 0.024,
    ).translate((FILTER_OUTER_RADIUS - 0.001, 0.0, FILTER_BODY_HEIGHT * 0.5))
    for index in range(30):
        angle_deg = 12.0 * index
        cartridge = cartridge.cut(
            pleat_cutter.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )

    top_band = _annulus(FILTER_OUTER_RADIUS, FILTER_INNER_RADIUS + 0.006, 0.010).translate(
        (0.0, 0.0, FILTER_BODY_HEIGHT - 0.010)
    )
    bottom_band = _annulus(FILTER_OUTER_RADIUS, FILTER_INNER_RADIUS + 0.004, 0.012)
    center_cap = cq.Workplane("XY").circle(0.071).extrude(0.010).translate((0.0, 0.0, FILTER_BODY_HEIGHT - 0.010))
    handle_post_left = cq.Workplane("XY").box(0.006, 0.008, 0.024).translate(
        (-0.023, 0.0, FILTER_BODY_HEIGHT + 0.010)
    )
    handle_post_right = cq.Workplane("XY").box(0.006, 0.008, 0.024).translate(
        (0.023, 0.0, FILTER_BODY_HEIGHT + 0.010)
    )
    handle_bridge = cq.Workplane("XY").box(0.052, 0.008, 0.008).translate(
        (0.0, 0.0, FILTER_BODY_HEIGHT + 0.023)
    )

    cartridge = (
        cartridge.union(top_band)
        .union(bottom_band)
        .union(center_cap)
        .union(handle_post_left)
        .union(handle_post_right)
        .union(handle_bridge)
    )
    cartridge = cartridge.combine()
    cartridge = cartridge.edges("|Z").fillet(0.0012)
    return cartridge


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((low + high) * 0.5 for low, high in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    cap_graphite = model.material("cap_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.84, 0.85, 0.83, 1.0))
    indicator_blue = model.material("indicator_blue", rgba=(0.41, 0.72, 0.90, 0.95))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "air_purifier_housing"),
        material=shell_white,
        name="housing_shell",
    )
    housing.visual(
        Box((0.012, 0.004, 0.004)),
        origin=Origin(xyz=(HOUSING_OUTER_RADIUS - 0.002, 0.0, HOUSING_HEIGHT - 0.001)),
        material=trim_gray,
        name="lock_mark",
    )
    housing.visual(
        Box((0.016, 0.005, 0.003)),
        origin=Origin(xyz=(0.0, HOUSING_OUTER_RADIUS - 0.002, 0.120)),
        material=indicator_blue,
        name="status_light",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shape(), "air_purifier_cap"),
        material=cap_graphite,
        name="cap_ring",
    )
    cap.visual(
        Box((0.020, 0.008, 0.0035)),
        origin=Origin(xyz=(0.0, CAP_OUTER_RADIUS - 0.004, CAP_HEIGHT - 0.0005)),
        material=trim_gray,
        name="grip_tab",
    )

    filter_part = model.part("filter")
    filter_part.visual(
        mesh_from_cadquery(_filter_shape(), "air_purifier_filter"),
        material=filter_gray,
        name="filter_cartridge",
    )

    model.articulation(
        "cap_lock",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_part,
        origin=Origin(xyz=(0.0, 0.0, FILTER_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.12,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cap = object_model.get_part("cap")
    filter_part = object_model.get_part("filter")
    cap_lock = object_model.get_articulation("cap_lock")
    filter_slide = object_model.get_articulation("filter_slide")

    cap_upper = cap_lock.motion_limits.upper or 0.0
    filter_upper = filter_slide.motion_limits.upper or 0.0

    ctx.allow_overlap(
        housing,
        filter_part,
        elem_a="housing_shell",
        elem_b="filter_cartridge",
        reason=(
            "The filter cartridge is intentionally retained inside the purifier body, "
            "and the current overlap classifier treats the enclosing body mesh as "
            "occupying the cartridge cavity."
        ),
    )

    ctx.expect_origin_distance(
        cap,
        housing,
        axes="xy",
        max_dist=0.001,
        name="cap stays centered on the purifier axis",
    )
    ctx.expect_within(
        filter_part,
        housing,
        axes="xy",
        inner_elem="filter_cartridge",
        outer_elem="housing_shell",
        margin=0.030,
        name="filter stays inside the cylindrical footprint",
    )
    ctx.expect_overlap(
        filter_part,
        housing,
        axes="z",
        elem_a="filter_cartridge",
        elem_b="housing_shell",
        min_overlap=0.180,
        name="resting filter remains deeply seated in the housing",
    )

    rest_cap_pos = ctx.part_world_position(cap)
    rest_filter_pos = ctx.part_world_position(filter_part)
    rest_tab_center = _aabb_center(ctx.part_element_world_aabb(cap, elem="grip_tab"))

    with ctx.pose({cap_lock: cap_upper}):
        rotated_cap_pos = ctx.part_world_position(cap)
        rotated_tab_center = _aabb_center(ctx.part_element_world_aabb(cap, elem="grip_tab"))

    with ctx.pose({filter_slide: filter_upper}):
        ctx.expect_within(
            filter_part,
            housing,
            axes="xy",
            inner_elem="filter_cartridge",
            outer_elem="housing_shell",
            margin=0.030,
            name="extended filter stays centered in the housing opening",
        )
        ctx.expect_overlap(
            filter_part,
            housing,
            axes="z",
            elem_a="filter_cartridge",
            elem_b="housing_shell",
            min_overlap=0.085,
            name="extended filter keeps retained insertion",
        )
        extended_filter_pos = ctx.part_world_position(filter_part)

    cap_origin_stable = (
        rest_cap_pos is not None
        and rotated_cap_pos is not None
        and math.dist(rest_cap_pos[:2], rotated_cap_pos[:2]) < 1e-6
        and abs((rotated_cap_pos[2] - rest_cap_pos[2])) < 1e-6
    )
    tab_visibly_rotates = (
        rest_tab_center is not None
        and rotated_tab_center is not None
        and abs(rotated_tab_center[0] - rest_tab_center[0]) > 0.020
        and abs(rotated_tab_center[1] - rest_tab_center[1]) > 0.006
    )
    ctx.check(
        "cap twists in place around the vertical lock axis",
        cap_origin_stable and tab_visibly_rotates,
        details=(
            f"rest_cap={rest_cap_pos}, rotated_cap={rotated_cap_pos}, "
            f"rest_tab={rest_tab_center}, rotated_tab={rotated_tab_center}"
        ),
    )

    filter_pulls_up = (
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[2] > rest_filter_pos[2] + 0.100
    )
    ctx.check(
        "filter cartridge pulls upward out of the housing",
        filter_pulls_up,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


object_model = build_object_model()
