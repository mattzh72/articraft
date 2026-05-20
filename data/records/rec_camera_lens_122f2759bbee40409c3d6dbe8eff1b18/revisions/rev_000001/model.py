from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_band(
    length: float,
    inner_radius: float,
    outer_radius: float,
    *,
    x_start: float = 0.0,
):
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    if inner_radius > 0.0:
        cutter = (
            cq.Workplane("YZ")
            .circle(inner_radius)
            .extrude(length + 0.002)
            .translate((-0.001, 0.0, 0.0))
        )
        outer = outer.cut(cutter)
    return outer.translate((x_start, 0.0, 0.0))


def _solid_cylinder(length: float, radius: float, *, x_start: float = 0.0):
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x_start, 0.0, 0.0))


def _ribbed_ring(
    *,
    length: float,
    inner_radius: float,
    body_radius: float,
    rib_radius: float,
    rib_count: int,
):
    ring = _annular_band(length, inner_radius, body_radius, x_start=-(length * 0.5))
    if rib_count <= 0:
        return ring

    rib_width = length / (rib_count * 1.9)
    usable = max(length - rib_width, rib_width)
    if rib_count == 1:
        centers = [0.0]
    else:
        step = usable / (rib_count - 1)
        centers = [(-usable * 0.5) + step * index for index in range(rib_count)]

    for center_x in centers:
        ring = ring.union(
            _annular_band(
                rib_width,
                inner_radius,
                rib_radius,
                x_start=center_x - (rib_width * 0.5),
            )
        )
    return ring


def _build_housing_mesh():
    body = _solid_cylinder(0.010, 0.0415, x_start=0.000)
    body = body.union(_solid_cylinder(0.018, 0.0400, x_start=0.010))
    body = body.union(_solid_cylinder(0.019, 0.0390, x_start=0.028))
    body = body.union(_solid_cylinder(0.004, 0.0440, x_start=0.047))
    body = body.union(_solid_cylinder(0.037, 0.0410, x_start=0.051))
    body = body.union(_annular_band(0.044, 0.0385, 0.0540, x_start=0.084))
    return body


def _build_front_trim_mesh():
    return _annular_band(0.0065, 0.0390, 0.0560, x_start=0.1275)


def _build_inner_tube_mesh():
    tube = _annular_band(0.092, 0.0270, 0.0368, x_start=0.000)
    tube = tube.union(_annular_band(0.010, 0.0270, 0.0380, x_start=0.028))
    tube = tube.union(_annular_band(0.010, 0.0265, 0.0392, x_start=0.056))
    tube = tube.union(_annular_band(0.006, 0.0230, 0.0418, x_start=0.086))
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_camera_lens_module")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.45, 0.47, 0.49, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.18, 0.28, 0.33, 0.55))
    index_white = model.material("index_white", rgba=(0.86, 0.88, 0.90, 1.0))

    housing_body_mesh = mesh_from_cadquery(_build_housing_mesh(), "lens_housing_body")
    front_trim_mesh = mesh_from_cadquery(_build_front_trim_mesh(), "lens_front_trim")
    zoom_ring_mesh = mesh_from_cadquery(
        _ribbed_ring(
            length=0.034,
            inner_radius=0.0415,
            body_radius=0.0492,
            rib_radius=0.0518,
            rib_count=8,
        ),
        "zoom_ring_shell",
    )
    clutch_ring_mesh = mesh_from_cadquery(
        _ribbed_ring(
            length=0.017,
            inner_radius=0.0395,
            body_radius=0.0458,
            rib_radius=0.0475,
            rib_count=5,
        ),
        "clutch_ring_shell",
    )
    inner_tube_mesh = mesh_from_cadquery(_build_inner_tube_mesh(), "inner_tube_shell")

    housing = model.part("housing")
    housing.visual(housing_body_mesh, material=shell_black, name="housing_shell")
    housing.visual(front_trim_mesh, material=trim_gray, name="front_trim")
    housing.visual(
        Box((0.024, 0.014, 0.004)),
        origin=Origin(xyz=(0.049, 0.0, 0.045)),
        material=satin_gray,
        name="scale_frame",
    )
    housing.visual(
        Box((0.018, 0.010, 0.002)),
        origin=Origin(xyz=(0.049, 0.0, 0.046)),
        material=glass_blue,
        name="scale_window",
    )
    housing.visual(
        Box((0.009, 0.0015, 0.0008)),
        origin=Origin(xyz=(0.0455, 0.0, 0.0468)),
        material=index_white,
        name="scale_mark_near",
    )
    housing.visual(
        Box((0.009, 0.0015, 0.0008)),
        origin=Origin(xyz=(0.0525, 0.0, 0.0468)),
        material=index_white,
        name="scale_mark_far",
    )
    housing.visual(
        Box((0.003, 0.0015, 0.0012)),
        origin=Origin(xyz=(0.049, 0.0, 0.0470)),
        material=index_white,
        name="scale_index",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(zoom_ring_mesh, material=rubber_black, name="zoom_shell")

    clutch_ring = model.part("clutch_ring")
    clutch_ring.visual(clutch_ring_mesh, material=rubber_black, name="clutch_shell")
    clutch_ring.visual(
        Box((0.008, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        material=trim_gray,
        name="clutch_tab",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(inner_tube_mesh, material=shell_black, name="tube_shell")
    inner_tube.visual(
        Cylinder(radius=0.0234, length=0.003),
        origin=Origin(xyz=(0.0875, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=glass_blue,
        name="front_glass",
    )

    model.articulation(
        "housing_to_zoom_ring",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=zoom_ring,
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "housing_to_clutch_ring",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=clutch_ring,
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "housing_to_inner_tube",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=inner_tube,
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.024,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    zoom_ring = object_model.get_part("zoom_ring")
    clutch_ring = object_model.get_part("clutch_ring")
    inner_tube = object_model.get_part("inner_tube")

    zoom_joint = object_model.get_articulation("housing_to_zoom_ring")
    clutch_joint = object_model.get_articulation("housing_to_clutch_ring")
    tube_joint = object_model.get_articulation("housing_to_inner_tube")

    ctx.check(
        "zoom ring uses continuous rotation",
        zoom_joint.articulation_type == ArticulationType.CONTINUOUS
        and zoom_joint.motion_limits is not None
        and zoom_joint.motion_limits.lower is None
        and zoom_joint.motion_limits.upper is None,
        details=(
            f"type={zoom_joint.articulation_type}, "
            f"limits={zoom_joint.motion_limits}"
        ),
    )
    ctx.check(
        "clutch ring uses continuous rotation",
        clutch_joint.articulation_type == ArticulationType.CONTINUOUS
        and clutch_joint.motion_limits is not None
        and clutch_joint.motion_limits.lower is None
        and clutch_joint.motion_limits.upper is None,
        details=(
            f"type={clutch_joint.articulation_type}, "
            f"limits={clutch_joint.motion_limits}"
        ),
    )
    ctx.check(
        "inner tube uses prismatic travel",
        tube_joint.articulation_type == ArticulationType.PRISMATIC
        and tube_joint.motion_limits is not None
        and tube_joint.motion_limits.lower == 0.0
        and tube_joint.motion_limits.upper is not None
        and tube_joint.motion_limits.upper >= 0.022,
        details=(
            f"type={tube_joint.articulation_type}, "
            f"limits={tube_joint.motion_limits}"
        ),
    )

    ctx.allow_overlap(
        housing,
        zoom_ring,
        elem_a="housing_shell",
        elem_b="zoom_shell",
        reason=(
            "The zoom ring is modeled around a simplified solid support sleeve "
            "inside the main housing shell."
        ),
    )
    ctx.allow_overlap(
        housing,
        inner_tube,
        elem_a="housing_shell",
        elem_b="tube_shell",
        reason=(
            "The extending inner tube is represented telescoping through a "
            "simplified solid housing guide sleeve."
        ),
    )

    ctx.expect_origin_distance(
        zoom_ring,
        housing,
        axes="yz",
        max_dist=1e-6,
        name="zoom ring stays coaxial with housing",
    )
    ctx.expect_origin_distance(
        clutch_ring,
        housing,
        axes="yz",
        max_dist=1e-6,
        name="clutch ring stays coaxial with housing",
    )
    ctx.expect_origin_distance(
        inner_tube,
        housing,
        axes="yz",
        max_dist=1e-6,
        name="inner tube starts coaxial with housing",
    )
    ctx.expect_overlap(
        inner_tube,
        housing,
        axes="x",
        elem_a="tube_shell",
        elem_b="housing_shell",
        min_overlap=0.038,
        name="collapsed inner tube remains telescoped into housing",
    )

    rest_pos = ctx.part_world_position(inner_tube)
    tube_upper = tube_joint.motion_limits.upper if tube_joint.motion_limits is not None else None
    with ctx.pose({tube_joint: tube_upper}):
        ctx.expect_origin_distance(
            inner_tube,
            housing,
            axes="yz",
            max_dist=1e-6,
            name="extended inner tube stays coaxial with housing",
        )
        ctx.expect_overlap(
            inner_tube,
            housing,
            axes="x",
            elem_a="tube_shell",
            elem_b="housing_shell",
            min_overlap=0.018,
            name="extended inner tube retains insertion in housing",
        )
        extended_pos = ctx.part_world_position(inner_tube)

    ctx.check(
        "inner tube extends forward along optical axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
