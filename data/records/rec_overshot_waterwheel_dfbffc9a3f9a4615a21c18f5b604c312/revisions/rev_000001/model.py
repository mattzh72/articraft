from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_yz_timber(
    part,
    *,
    x: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    width_x: float,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    dy = y1 - y0
    dz = z1 - z0
    length = sqrt(dy * dy + dz * dz)
    angle = atan2(dy, dz)
    part.visual(
        Box((width_x, thickness, length)),
        origin=Origin(
            xyz=(x, 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
            rpy=(-angle, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mn, mx = aabb
    return tuple((mn[index] + mx[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_overshot_wheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.54, 0.40, 0.24, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.35, 0.25, 0.16, 1.0))
    damp_timber = model.material("damp_timber", rgba=(0.28, 0.21, 0.14, 1.0))
    forged_iron = model.material("forged_iron", rgba=(0.22, 0.22, 0.23, 1.0))

    wheel_radius = 1.42
    wheel_width = 0.58
    wheel_center_z = 1.72
    support_half_span_x = 0.44

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.20, 3.10, 3.30)),
        mass=620.0,
        origin=Origin(xyz=(0.0, -0.10, 1.65)),
    )

    frame.visual(
        Box((0.16, 2.60, 0.20)),
        origin=Origin(xyz=(-support_half_span_x, 0.0, 0.10)),
        material=damp_timber,
        name="left_sill",
    )
    frame.visual(
        Box((0.16, 2.60, 0.20)),
        origin=Origin(xyz=(support_half_span_x, 0.0, 0.10)),
        material=damp_timber,
        name="right_sill",
    )
    frame.visual(
        Box((1.04, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, 1.02, 0.18)),
        material=damp_timber,
        name="front_tie",
    )
    frame.visual(
        Box((1.04, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, -1.02, 0.18)),
        material=damp_timber,
        name="rear_tie",
    )

    for x_sign in (-1.0, 1.0):
        x = support_half_span_x * x_sign
        side_name = "left" if x_sign < 0.0 else "right"
        frame.visual(
            Box((0.12, 0.12, 2.34)),
            origin=Origin(xyz=(x, 1.02, 1.17)),
            material=weathered_oak,
            name=f"{side_name}_front_post",
        )
        frame.visual(
            Box((0.12, 0.12, 3.12)),
            origin=Origin(xyz=(x, -0.96, 1.56)),
            material=weathered_oak,
            name=f"{side_name}_rear_post",
        )
        frame.visual(
            Box((0.14, 0.18, 1.53)),
            origin=Origin(xyz=(x, 0.0, 0.765)),
            material=weathered_oak,
            name=f"{side_name}_axle_post",
        )
        frame.visual(
            Box((0.12, 2.10, 0.12)),
            origin=Origin(xyz=(x, 0.03, 2.46)),
            material=weathered_oak,
            name=f"{side_name}_top_rail",
        )
        frame.visual(
            Box((0.16, 0.22, 0.14)),
            origin=Origin(xyz=(x, 0.0, wheel_center_z - 0.125)),
            material=forged_iron,
            name=f"{side_name}_bearing_block",
        )
        _add_yz_timber(
            frame,
            x=x,
            y0=0.02,
            z0=1.42,
            y1=0.92,
            z1=2.02,
            width_x=0.10,
            thickness=0.10,
            material=weathered_oak,
            name=f"{side_name}_front_brace",
        )
        _add_yz_timber(
            frame,
            x=x,
            y0=-0.02,
            z0=1.44,
            y1=-0.90,
            z1=2.10,
            width_x=0.10,
            thickness=0.10,
            material=weathered_oak,
            name=f"{side_name}_rear_brace",
        )

    frame.visual(
        Box((1.04, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.96, 3.12)),
        material=weathered_oak,
        name="rear_crossbeam",
    )

    frame.visual(
        Box((0.90, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.96, 2.96)),
        material=weathered_oak,
        name="feed_support_beam",
    )
    frame.visual(
        Box((0.68, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, -0.85, 3.01)),
        material=damp_timber,
        name="feed_box_bottom",
    )
    frame.visual(
        Box((0.06, 0.34, 0.18)),
        origin=Origin(xyz=(-0.31, -0.85, 3.125)),
        material=weathered_oak,
        name="feed_box_left_wall",
    )
    frame.visual(
        Box((0.06, 0.34, 0.18)),
        origin=Origin(xyz=(0.31, -0.85, 3.125)),
        material=weathered_oak,
        name="feed_box_right_wall",
    )
    frame.visual(
        Box((0.68, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, -1.02, 3.125)),
        material=weathered_oak,
        name="feed_box_rear_wall",
    )
    frame.visual(
        Box((0.68, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, -0.685, 3.075)),
        material=weathered_oak,
        name="feed_box_front_lip",
    )
    frame.visual(
        Box((0.68, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, -1.02, 3.195)),
        material=dark_oak,
        name="feed_box_hinge_rail",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.46, length=wheel_width),
        mass=260.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.055, length=0.76),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_iron,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.28, length=0.52),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oak,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.34, length=0.05),
        origin=Origin(xyz=(-0.19, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oak,
        name="hub_left_flange",
    )
    wheel.visual(
        Cylinder(radius=0.34, length=0.05),
        origin=Origin(xyz=(0.19, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oak,
        name="hub_right_flange",
    )

    side_ring_x = wheel_width * 0.45
    ring_radius = 1.26
    bucket_radius = 1.35
    ring_segment_count = 16
    spoke_count = 16
    spoke_radius = 0.76

    for index in range(spoke_count):
        angle = (2.0 * pi * index) / spoke_count
        y = spoke_radius * sin(angle)
        z = spoke_radius * cos(angle)
        for side_x, side_label in ((-side_ring_x, "left"), (side_ring_x, "right")):
            wheel.visual(
                Box((0.05, 0.055, 1.00)),
                origin=Origin(xyz=(side_x, y, z), rpy=(-angle, 0.0, 0.0)),
                material=dark_oak,
                name=f"{side_label}_spoke_{index:02d}",
            )

    for index in range(ring_segment_count):
        angle = (2.0 * pi * index) / ring_segment_count
        y = ring_radius * sin(angle)
        z = ring_radius * cos(angle)
        for side_x, side_label in ((-side_ring_x, "left"), (side_ring_x, "right")):
            wheel.visual(
                Box((0.06, 0.54, 0.16)),
                origin=Origin(xyz=(side_x, y, z), rpy=(-angle, 0.0, 0.0)),
                material=weathered_oak,
                name=f"{side_label}_rim_{index:02d}",
            )
        wheel.visual(
            Box((0.58, 0.16, 0.22)),
            origin=Origin(
                xyz=(0.0, bucket_radius * sin(angle), bucket_radius * cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=weathered_oak,
            name=f"bucket_{index:02d}_outer",
        )

    cover = model.part("feed_box_cover")
    cover.inertial = Inertial.from_geometry(
        Box((0.56, 0.18, 0.05)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.09, -0.025)),
    )
    cover.visual(
        Box((0.56, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, 0.09, -0.015)),
        material=weathered_oak,
        name="cover_panel",
    )
    cover.visual(
        Box((0.05, 0.10, 0.05)),
        origin=Origin(xyz=(-0.19, 0.16, -0.04)),
        material=dark_oak,
        name="left_batten",
    )
    cover.visual(
        Box((0.05, 0.10, 0.05)),
        origin=Origin(xyz=(0.19, 0.16, -0.04)),
        material=dark_oak,
        name="right_batten",
    )
    cover.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, 0.12, -0.03), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_iron,
        name="cover_handle",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.5),
    )
    model.articulation(
        "feed_box_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(0.0, -0.99, 3.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    cover = object_model.get_part("feed_box_cover")

    wheel_spin = object_model.get_articulation("wheel_spin")
    cover_hinge = object_model.get_articulation("feed_box_cover_hinge")

    feed_box_bottom = frame.get_visual("feed_box_bottom")
    left_bearing_block = frame.get_visual("left_bearing_block")
    right_bearing_block = frame.get_visual("right_bearing_block")
    axle_shaft = wheel.get_visual("axle_shaft")
    cover_panel = cover.get_visual("cover_panel")
    wheel_bucket = wheel.get_visual("bucket_00_outer")

    wheel_limits = wheel_spin.motion_limits
    cover_limits = cover_hinge.motion_limits
    ctx.check(
        "wheel uses a horizontal continuous axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0)
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}, limits={wheel_limits}",
    )
    ctx.check(
        "feed-box cover uses a top hinge",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE
        and cover_hinge.axis == (1.0, 0.0, 0.0)
        and cover_limits is not None
        and cover_limits.lower == 0.0
        and cover_limits.upper is not None
        and cover_limits.upper >= 1.1,
        details=f"type={cover_hinge.articulation_type}, axis={cover_hinge.axis}, limits={cover_limits}",
    )

    ctx.expect_overlap(
        cover,
        frame,
        axes="x",
        min_overlap=0.50,
        elem_a=cover_panel,
        elem_b=feed_box_bottom,
        name="short cover spans the feed-box width",
    )
    ctx.expect_gap(
        cover,
        frame,
        axis="z",
        min_gap=0.12,
        max_gap=0.24,
        positive_elem=cover_panel,
        negative_elem=feed_box_bottom,
        name="cover rests above the feed box",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="xy",
        min_overlap=0.015,
        elem_a=axle_shaft,
        elem_b=left_bearing_block,
        name="left axle end sits over the left bearing block",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=axle_shaft,
        negative_elem=left_bearing_block,
        name="left bearing block supports the axle from below",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="xy",
        min_overlap=0.015,
        elem_a=axle_shaft,
        elem_b=right_bearing_block,
        name="right axle end sits over the right bearing block",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=axle_shaft,
        negative_elem=right_bearing_block,
        name="right bearing block supports the axle from below",
    )

    with ctx.pose({wheel_spin: 0.0}):
        rest_bucket_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=wheel_bucket))
    with ctx.pose({wheel_spin: pi / 2.0}):
        quarter_bucket_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=wheel_bucket))
    ctx.check(
        "wheel rotation carries a bucket around the rim",
        rest_bucket_center is not None
        and quarter_bucket_center is not None
        and quarter_bucket_center[1] < rest_bucket_center[1] - 1.1
        and quarter_bucket_center[2] < rest_bucket_center[2] - 1.1,
        details=f"rest={rest_bucket_center}, quarter={quarter_bucket_center}",
    )

    with ctx.pose({cover_hinge: 0.0}):
        closed_cover_center = _aabb_center(ctx.part_element_world_aabb(cover, elem=cover_panel))
    with ctx.pose({cover_hinge: 1.0}):
        open_cover_center = _aabb_center(ctx.part_element_world_aabb(cover, elem=cover_panel))
    ctx.check(
        "feed-box cover opens upward",
        closed_cover_center is not None
        and open_cover_center is not None
        and open_cover_center[2] > closed_cover_center[2] + 0.05
        and open_cover_center[1] < closed_cover_center[1] - 0.02,
        details=f"closed={closed_cover_center}, open={open_cover_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
