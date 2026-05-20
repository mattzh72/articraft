from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LENS_LENGTH = 0.240
FRONT_BORE_RADIUS = 0.031
FRONT_RECESS_RADIUS = 0.034
FOCUS_RING_CENTER_X = 0.163
FOCUS_RING_LENGTH = 0.070
IRIS_RING_CENTER_X = 0.040
IRIS_RING_LENGTH = 0.032
MARKER_SLOT_CENTER_X = 0.218
MARKER_SLOT_CENTER_Y = 0.039
MARKER_SLOT_LENGTH = 0.024


def _cylinder_segment(x0: float, x1: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(x1 - x0).translate((x0, 0.0, 0.0))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    body = shapes[0]
    for shape in shapes[1:]:
        body = body.union(shape)
    return body


def _build_housing_shape() -> cq.Workplane:
    outer_segments = [
        _cylinder_segment(0.000, 0.018, 0.034),
        _cylinder_segment(0.018, 0.024, 0.0405),
        _cylinder_segment(0.024, 0.056, 0.0385),
        _cylinder_segment(0.056, 0.060, 0.0405),
        _cylinder_segment(0.060, 0.122, 0.040),
        _cylinder_segment(0.122, 0.128, 0.043),
        _cylinder_segment(0.128, 0.198, 0.041),
        _cylinder_segment(0.198, 0.204, 0.043),
        _cylinder_segment(0.204, 0.229, 0.046),
        _cylinder_segment(0.229, 0.240, 0.050),
    ]
    housing = _union_all(outer_segments)

    main_bore = cq.Workplane("YZ").circle(FRONT_BORE_RADIUS).extrude(LENS_LENGTH)
    front_recess = (
        cq.Workplane("YZ")
        .circle(FRONT_RECESS_RADIUS)
        .extrude(0.030)
        .translate((0.210, 0.0, 0.0))
    )
    slot_cut = (
        cq.Workplane("XY")
        .box(MARKER_SLOT_LENGTH, 0.024, 0.011)
        .translate((MARKER_SLOT_CENTER_X, MARKER_SLOT_CENTER_Y, 0.0))
    )

    return housing.cut(main_bore).cut(front_recess).cut(slot_cut)


def _build_geared_ring_shape(
    *,
    length: float,
    inner_radius: float,
    base_outer_radius: float,
    tooth_depth: float,
    tooth_count: int,
    smooth_patch_center_deg: float | None = None,
    smooth_patch_width_deg: float = 0.0,
) -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(base_outer_radius)
        .extrude(length)
        .translate((-length * 0.5, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length)
        .translate((-length * 0.5, 0.0, 0.0))
    )
    ring = outer.cut(inner)

    tooth_pitch = 360.0 / tooth_count
    tooth_width = (2.0 * math.pi * base_outer_radius / tooth_count) * 0.55
    patch_half_width = smooth_patch_width_deg * 0.5

    for idx in range(tooth_count):
        angle_deg = idx * tooth_pitch
        if smooth_patch_center_deg is not None:
            delta = ((angle_deg - smooth_patch_center_deg + 180.0) % 360.0) - 180.0
            if abs(delta) <= patch_half_width:
                continue

        tooth = (
            cq.Workplane("XY")
            .box(length, tooth_depth, tooth_width)
            .translate((0.0, base_outer_radius + tooth_depth * 0.5, 0.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        ring = ring.union(tooth)

    return ring


def _build_marker_tab_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.010, 0.010, 0.009).translate((0.0, 0.010, 0.0))
    neck = cq.Workplane("XY").box(0.010, 0.024, 0.011).translate((0.0, 0.0, 0.0))
    shoe = cq.Workplane("XY").box(0.016, 0.008, 0.012).translate((0.0, -0.015, 0.0))
    return cap.union(neck).union(shoe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_cinema_lens")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))
    marker_cream = model.material("marker_cream", rgba=(0.85, 0.82, 0.72, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing_shell"),
        material=anodized_black,
        name="housing_shell",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            _build_geared_ring_shape(
                length=FOCUS_RING_LENGTH,
                inner_radius=0.0418,
                base_outer_radius=0.0465,
                tooth_depth=0.0032,
                tooth_count=40,
                smooth_patch_center_deg=90.0,
                smooth_patch_width_deg=22.0,
            ),
            "focus_ring_band",
        ),
        material=grip_black,
        name="focus_band",
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        mesh_from_cadquery(
            _build_geared_ring_shape(
                length=IRIS_RING_LENGTH,
                inner_radius=0.0392,
                base_outer_radius=0.0425,
                tooth_depth=0.0023,
                tooth_count=28,
                smooth_patch_center_deg=88.0,
                smooth_patch_width_deg=16.0,
            ),
            "iris_ring_band",
        ),
        material=grip_black,
        name="iris_band",
    )

    marker_tab = model.part("marker_tab")
    marker_tab.visual(
        mesh_from_cadquery(_build_marker_tab_shape(), "marker_tab"),
        material=marker_cream,
        name="marker_body",
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=focus_ring,
        origin=Origin(xyz=(FOCUS_RING_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )
    model.articulation(
        "iris_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=iris_ring,
        origin=Origin(xyz=(IRIS_RING_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "marker_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=marker_tab,
        origin=Origin(xyz=(MARKER_SLOT_CENTER_X, MARKER_SLOT_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.03, lower=-0.006, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")
    marker_tab = object_model.get_part("marker_tab")

    focus_rotation = object_model.get_articulation("focus_rotation")
    iris_rotation = object_model.get_articulation("iris_rotation")
    marker_slide = object_model.get_articulation("marker_slide")

    ctx.allow_overlap(
        focus_ring,
        housing,
        elem_a="focus_band",
        elem_b="housing_shell",
        reason="The focus band is a hollow concentric ring wrapped around the barrel shell; this close coaxial fit is an authored nested fit rather than a visible collision.",
    )
    ctx.allow_overlap(
        iris_ring,
        housing,
        elem_a="iris_band",
        elem_b="housing_shell",
        reason="The iris band is a hollow concentric ring around the rear barrel section; the reported mesh overlap is an intended nested ring fit.",
    )
    ctx.allow_overlap(
        marker_tab,
        housing,
        elem_a="marker_body",
        elem_b="housing_shell",
        reason="The magnification marker is intentionally retained inside the side slot as a close-fitting slider guide.",
    )

    ctx.expect_origin_distance(
        focus_ring,
        housing,
        axes="yz",
        max_dist=0.0001,
        name="focus ring stays coaxial with housing",
    )
    ctx.expect_origin_distance(
        iris_ring,
        housing,
        axes="yz",
        max_dist=0.0001,
        name="iris ring stays coaxial with housing",
    )
    ctx.expect_origin_gap(
        marker_tab,
        housing,
        axis="y",
        min_gap=0.035,
        max_gap=0.043,
        name="marker tab sits on the lens sidewall",
    )

    rest_focus = ctx.part_world_position(focus_ring)
    rest_iris = ctx.part_world_position(iris_ring)
    rest_marker = ctx.part_world_position(marker_tab)

    with ctx.pose({focus_rotation: math.pi / 2.0, iris_rotation: -math.pi / 3.0, marker_slide: 0.006}):
        turned_focus = ctx.part_world_position(focus_ring)
        turned_iris = ctx.part_world_position(iris_ring)
        extended_marker = ctx.part_world_position(marker_tab)
        ctx.expect_origin_distance(
            focus_ring,
            housing,
            axes="yz",
            max_dist=0.0001,
            name="focus ring remains coaxial while turning",
        )
        ctx.expect_origin_distance(
            iris_ring,
            housing,
            axes="yz",
            max_dist=0.0001,
            name="iris ring remains coaxial while turning",
        )

    with ctx.pose(marker_slide=-0.006):
        retracted_marker = ctx.part_world_position(marker_tab)

    ctx.check(
        "focus ring rotates in place without axial drift",
        rest_focus is not None
        and turned_focus is not None
        and abs(turned_focus[0] - rest_focus[0]) <= 1e-6
        and abs(turned_focus[1] - rest_focus[1]) <= 1e-6
        and abs(turned_focus[2] - rest_focus[2]) <= 1e-6,
        details=f"rest={rest_focus}, turned={turned_focus}",
    )
    ctx.check(
        "iris ring rotates in place without axial drift",
        rest_iris is not None
        and turned_iris is not None
        and abs(turned_iris[0] - rest_iris[0]) <= 1e-6
        and abs(turned_iris[1] - rest_iris[1]) <= 1e-6
        and abs(turned_iris[2] - rest_iris[2]) <= 1e-6,
        details=f"rest={rest_iris}, turned={turned_iris}",
    )
    ctx.check(
        "marker tab travels forward and backward in its slot",
        rest_marker is not None
        and extended_marker is not None
        and retracted_marker is not None
        and extended_marker[0] > rest_marker[0] + 0.005
        and retracted_marker[0] < rest_marker[0] - 0.005,
        details=f"retracted={retracted_marker}, rest={rest_marker}, extended={extended_marker}",
    )

    return ctx.report()


object_model = build_object_model()
