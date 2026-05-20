from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


BODY_RADIUS = 0.092
BODY_INNER_RADIUS = 0.072
BODY_HEIGHT = 0.430
BASE_THICKNESS = 0.035

CAP_JOINT_Z = BODY_HEIGHT - 0.001
FILTER_JOINT_Z = BODY_HEIGHT - 0.025
FILTER_TRAVEL = 0.245


def _rotated_box(radial_size: float, tangential_size: float, height: float, radius: float, z: float, angle_deg: float):
    """A small rectangular cutter/detail centered at a polar location."""
    return (
        cq.Workplane("XY")
        .box(radial_size, tangential_size, height)
        .translate((radius, 0.0, z))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    )


def _tower_body_shape():
    """Cylindrical hollow purifier tower with real cut-through intake slots."""
    shell = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .circle(BODY_INNER_RADIUS)
        .extrude(BODY_HEIGHT - BASE_THICKNESS + 0.004)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    body = shell.cut(cavity)

    # Two staggered bands of vertical intake perforations.  The slots are
    # actual cutouts through the wall so the filter can be seen behind them.
    for band_z in (0.155, 0.295):
        for i in range(16):
            angle = i * 360.0 / 16.0 + (11.25 if band_z > 0.2 else 0.0)
            cutter = _rotated_box(0.050, 0.010, 0.118, BODY_RADIUS, band_z, angle)
            body = body.cut(cutter)
    return body


def _top_cap_shape():
    """Bayonet-style annular cap that rotates around the top opening."""
    top_ring = _annular_ring(0.112, 0.063, 0.019, 0.001)
    outer_skirt = _annular_ring(0.112, 0.102, 0.022, 0.001)
    cap = top_ring.union(outer_skirt)

    # Three low grip pads on the top surface make the twist motion legible.
    for angle in (15.0, 135.0, 255.0):
        pad = _rotated_box(0.030, 0.014, 0.006, 0.090, 0.023, angle)
        cap = cap.union(pad)
    return cap


def _filter_media_shape():
    """Pleated annular HEPA cartridge body in the filter part frame."""
    outer = 0.052
    inner = 0.024
    height = 0.285
    z_min = -0.304
    media = _annular_ring(outer, inner, height, z_min)
    for i in range(28):
        angle = i * 360.0 / 28.0
        rib = _rotated_box(0.006, 0.0032, height, outer + 0.002, z_min + height / 2.0, angle)
        media = media.union(rib)
    return media


def _annular_ring(outer: float, inner: float, height: float, z_min: float):
    solid = cq.Workplane("XY").circle(outer).extrude(height).translate((0.0, 0.0, z_min))
    bore = cq.Workplane("XY").circle(inner).extrude(height + 0.004).translate((0.0, 0.0, z_min - 0.002))
    return solid.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_air_purifier")

    warm_white = Material("warm_white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    soft_gray = Material("soft_gray_rubber", rgba=(0.18, 0.19, 0.20, 1.0))
    charcoal = Material("charcoal_detail", rgba=(0.02, 0.025, 0.03, 1.0))
    paper = Material("pleated_filter_paper", rgba=(0.83, 0.76, 0.58, 1.0))
    blue = Material("blue_unlock_mark", rgba=(0.05, 0.30, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tower_body_shape(), "body_shell", tolerance=0.0012, angular_tolerance=0.08),
        material=warm_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.098, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=soft_gray,
        name="base_foot",
    )
    body.visual(
        mesh_from_cadquery(_annular_ring(0.057, 0.020, 0.052, BASE_THICKNESS), "filter_socket", tolerance=0.001),
        material=soft_gray,
        name="filter_socket",
    )
    # Fixed lock/unlock marks on the stationary tower lip.
    body.visual(
        Box((0.006, 0.006, 0.026)),
        origin=Origin(xyz=(BODY_RADIUS - 0.003, -0.022, BODY_HEIGHT - 0.033)),
        material=charcoal,
        name="lock_mark",
    )
    body.visual(
        Box((0.006, 0.006, 0.026)),
        origin=Origin(xyz=(BODY_RADIUS - 0.003, 0.022, BODY_HEIGHT - 0.033)),
        material=blue,
        name="unlock_mark",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        mesh_from_cadquery(_top_cap_shape(), "top_cap_shell_above_rim", tolerance=0.001, angular_tolerance=0.08),
        material=warm_white,
        name="cap_shell",
    )
    top_cap.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.083, 0.0, 0.0215)),
        material=blue,
        name="unlock_pointer",
    )

    filter_part = model.part("filter")
    filter_part.visual(
        mesh_from_cadquery(_filter_media_shape(), "filter_media_pleated", tolerance=0.001, angular_tolerance=0.08),
        material=paper,
        name="filter_media",
    )
    filter_part.visual(
        mesh_from_cadquery(_annular_ring(0.055, 0.021, 0.018, -0.020), "filter_top_ring_hollow", tolerance=0.001),
        material=warm_white,
        name="filter_top_ring",
    )
    filter_part.visual(
        mesh_from_cadquery(_annular_ring(0.055, 0.021, 0.018, -0.318), "filter_bottom_ring_hollow", tolerance=0.001),
        material=warm_white,
        name="filter_bottom_ring",
    )
    filter_part.visual(
        Cylinder(radius=0.022, length=0.292),
        origin=Origin(xyz=(0.0, 0.0, -0.162)),
        material=charcoal,
        name="perforated_core",
    )

    model.articulation(
        "body_to_top_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0, lower=0.0, upper=math.radians(55.0)),
    )

    model.articulation(
        "body_to_filter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_part,
        origin=Origin(xyz=(0.0, 0.0, FILTER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.25, lower=0.0, upper=FILTER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    top_cap = object_model.get_part("top_cap")
    filter_part = object_model.get_part("filter")
    cap_joint = object_model.get_articulation("body_to_top_cap")
    filter_slide = object_model.get_articulation("body_to_filter")

    ctx.check(
        "top cap twists about the vertical axis",
        cap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(cap_joint.axis) == (0.0, 0.0, 1.0)
        and cap_joint.motion_limits.upper >= math.radians(45.0),
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}, limits={cap_joint.motion_limits}",
    )
    ctx.check(
        "filter slides upward out of the tower",
        filter_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(filter_slide.axis) == (0.0, 0.0, 1.0)
        and filter_slide.motion_limits.upper >= 0.20,
        details=f"type={filter_slide.articulation_type}, axis={filter_slide.axis}, limits={filter_slide.motion_limits}",
    )

    ctx.expect_within(
        filter_part,
        body,
        axes="xy",
        inner_elem="filter_media",
        outer_elem="body_shell",
        margin=0.0,
        name="filter cartridge is centered within the tower cavity footprint",
    )
    ctx.expect_overlap(
        filter_part,
        body,
        axes="z",
        elem_a="filter_media",
        elem_b="body_shell",
        min_overlap=0.24,
        name="resting filter remains deeply inserted",
    )

    rest_filter_pos = ctx.part_world_position(filter_part)
    rest_pointer_aabb = ctx.part_element_world_aabb(top_cap, elem="unlock_pointer")

    with ctx.pose({filter_slide: FILTER_TRAVEL}):
        ctx.expect_overlap(
            filter_part,
            body,
            axes="z",
            elem_a="filter_media",
            elem_b="body_shell",
            min_overlap=0.055,
            name="extended filter keeps retained insertion in the tower",
        )
        extended_filter_pos = ctx.part_world_position(filter_part)

    with ctx.pose({cap_joint: cap_joint.motion_limits.upper}):
        turned_pointer_aabb = ctx.part_element_world_aabb(top_cap, elem="unlock_pointer")

    ctx.check(
        "filter upper pose moves upward",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[2] > rest_filter_pos[2] + 0.20,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    rest_pointer_y = _aabb_center_y(rest_pointer_aabb)
    turned_pointer_y = _aabb_center_y(turned_pointer_aabb)
    ctx.check(
        "cap pointer visibly sweeps toward unlock mark",
        rest_pointer_y is not None and turned_pointer_y is not None and turned_pointer_y > rest_pointer_y + 0.04,
        details=f"rest_y={rest_pointer_y}, turned_y={turned_pointer_y}",
    )

    return ctx.report()


object_model = build_object_model()
