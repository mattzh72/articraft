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


def _add_sloped_box(part, name, p0, p1, size_x, size_y, material):
    """Add a rectangular member whose local +Z axis runs from p0 to p1."""
    cx = (p0[0] + p1[0]) * 0.5
    cy = (p0[1] + p1[1]) * 0.5
    cz = (p0[2] + p1[2]) * 0.5
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.hypot(dx, dz)
    pitch = math.atan2(dx, dz)
    part.visual(
        Box((size_x, size_y, length)),
        origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _line_point(foot_x, foot_z, t, y=0.0):
    return (foot_x * t, y, foot_z * t)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_position_convertible_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.74, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    graphite = model.material("graphite_hardware", rgba=(0.18, 0.18, 0.17, 1.0))
    yellow = model.material("yellow_lock_tabs", rgba=(1.0, 0.66, 0.05, 1.0))

    hinge_radius = 0.115
    hinge_rpy = (-math.pi / 2.0, 0.0, 0.0)
    front_foot_x, front_foot_z = 0.78, -2.35
    rear_foot_x, rear_foot_z = -0.68, -2.35
    rear_len = math.hypot(rear_foot_x, rear_foot_z)
    rear_up_axis = (-rear_foot_x / rear_len, 0.0, -rear_foot_z / rear_len)
    fly_plane_offset = (0.110 * rear_up_axis[2], 0.0, -0.110 * rear_up_axis[0])

    front = model.part("front_section")
    rear = model.part("rear_section")
    fly = model.part("fly_section")

    # Front A-frame ladder section: wide outer stiles, flat rungs, and rubber shoes.
    for side in (-1.0, 1.0):
        y = side * 0.36
        _add_sloped_box(
            front,
            f"front_stile_{'neg' if side < 0 else 'pos'}",
            (0.02, y, -0.04),
            (front_foot_x, y, front_foot_z),
            0.058,
            0.050,
            aluminum,
        )
        front.visual(
            Box((0.24, 0.17, 0.045)),
            origin=Origin(xyz=(front_foot_x + 0.035, y, front_foot_z - 0.006)),
            material=dark,
            name=f"front_shoe_{'neg' if side < 0 else 'pos'}",
        )
        front.visual(
            Box((0.155, 0.115, 0.095)),
            origin=Origin(xyz=(0.0, side * 0.3725, -0.006)),
            material=graphite,
            name=f"front_hinge_leaf_{'neg' if side < 0 else 'pos'}",
        )
        front.visual(
            Cylinder(radius=hinge_radius, length=0.032),
            origin=Origin(xyz=(0.0, side * 0.414, 0.0), rpy=hinge_rpy),
            material=graphite,
            name=f"front_knuckle_{'neg' if side < 0 else 'pos'}",
        )
        front.visual(
            Cylinder(radius=0.060, length=0.014),
            origin=Origin(xyz=(0.0, side * 0.435, 0.0), rpy=hinge_rpy),
            material=dark,
            name=f"pivot_cap_{'neg' if side < 0 else 'pos'}",
        )
        front.visual(
            Box((0.115, 0.016, 0.026)),
            origin=Origin(xyz=(0.096, side * 0.444, -0.062), rpy=(0.0, 0.0, side * 0.12)),
            material=yellow,
            name=f"lock_release_{'neg' if side < 0 else 'pos'}",
        )
        for i, deg in enumerate((-135, -95, -55, -15, 25)):
            ang = math.radians(deg)
            front.visual(
                Cylinder(radius=0.011, length=0.010),
                origin=Origin(
                    xyz=(0.074 * math.cos(ang), side * 0.434, 0.074 * math.sin(ang)),
                    rpy=hinge_rpy,
                ),
                material=dark,
                name=f"angle_hole_{'neg' if side < 0 else 'pos'}_{i}",
            )

    for i, t in enumerate((0.22, 0.38, 0.54, 0.70, 0.86)):
        x, _, z = _line_point(front_foot_x, front_foot_z, t)
        front.visual(
            Box((0.118, 0.790, 0.036)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"front_rung_{i}",
        )
        front.visual(
            Box((0.070, 0.640, 0.008)),
            origin=Origin(xyz=(x - 0.004, 0.0, z + 0.019)),
            material=dark,
            name=f"front_tread_grip_{i}",
        )

    # Rear A-frame section: slightly narrower stiles with guide clamps for the fly.
    for side in (-1.0, 1.0):
        y = side * 0.28
        _add_sloped_box(
            rear,
            f"rear_stile_{'neg' if side < 0 else 'pos'}",
            (-0.02, y, -0.04),
            (rear_foot_x, y, rear_foot_z),
            0.058,
            0.050,
            aluminum,
        )
        rear.visual(
            Box((0.23, 0.15, 0.045)),
            origin=Origin(xyz=(rear_foot_x - 0.030, y, rear_foot_z - 0.006)),
            material=dark,
            name=f"rear_shoe_{'neg' if side < 0 else 'pos'}",
        )
        rear.visual(
            Box((0.155, 0.090, 0.090)),
            origin=Origin(xyz=(0.0, side * 0.270, -0.006)),
            material=graphite,
            name=f"rear_hinge_leaf_{'neg' if side < 0 else 'pos'}",
        )
        rear.visual(
            Cylinder(radius=0.102, length=0.032),
            origin=Origin(xyz=(0.0, side * 0.230, 0.0), rpy=hinge_rpy),
            material=graphite,
            name=f"rear_knuckle_{'neg' if side < 0 else 'pos'}",
        )
        # Short guide collars mounted to the rear stiles. They nearly wrap the fly
        # stiles without intersecting them, so the sliding section reads as captured.
        for j, t in enumerate((0.28, 0.56, 0.84)):
            c0 = _line_point(rear_foot_x, rear_foot_z, t, side * 0.255)
            c = (c0[0] + fly_plane_offset[0] * 0.50, c0[1], c0[2] + fly_plane_offset[2] * 0.50)
            d = (rear_up_axis[0] * 0.070, 0.0, rear_up_axis[2] * 0.070)
            _add_sloped_box(
                rear,
                f"fly_guide_{'neg' if side < 0 else 'pos'}_{j}",
                (c[0] - d[0], c[1], c[2] - d[2]),
                (c[0] + d[0], c[1], c[2] + d[2]),
                0.130,
                0.075,
                graphite,
            )

    for i, t in enumerate((0.24, 0.40, 0.56, 0.72, 0.88)):
        x, _, z = _line_point(rear_foot_x, rear_foot_z, t)
        rear.visual(
            Box((0.105, 0.620, 0.035)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_rung_{i}",
        )
        rear.visual(
            Box((0.065, 0.500, 0.008)),
            origin=Origin(xyz=(x + 0.004, 0.0, z + 0.019)),
            material=dark,
            name=f"rear_tread_grip_{i}",
        )

    # Extension fly section nested inside the rear stile guide path.
    slide_base = _line_point(rear_foot_x, rear_foot_z, 0.20)
    slide_origin = (
        slide_base[0] + fly_plane_offset[0],
        slide_base[1] + fly_plane_offset[1],
        slide_base[2] + fly_plane_offset[2],
    )
    fly_top_t = 0.29
    fly_bottom_t = 0.94
    for side in (-1.0, 1.0):
        top_base = _line_point(rear_foot_x, rear_foot_z, fly_top_t, side * 0.190)
        bottom_base = _line_point(rear_foot_x, rear_foot_z, fly_bottom_t, side * 0.190)
        top = _sub((top_base[0] + fly_plane_offset[0], top_base[1], top_base[2] + fly_plane_offset[2]), slide_origin)
        bottom = _sub(
            (bottom_base[0] + fly_plane_offset[0], bottom_base[1], bottom_base[2] + fly_plane_offset[2]),
            slide_origin,
        )
        _add_sloped_box(
            fly,
            f"fly_stile_{'neg' if side < 0 else 'pos'}",
            top,
            bottom,
            0.046,
            0.040,
            aluminum,
        )
        fly.visual(
            Box((0.075, 0.060, 0.040)),
            origin=Origin(xyz=top),
            material=dark,
            name=f"fly_top_cap_{'neg' if side < 0 else 'pos'}",
        )
        pawl_base = _line_point(rear_foot_x, rear_foot_z, 0.48, side * 0.214)
        pawl = _sub((pawl_base[0] + fly_plane_offset[0], pawl_base[1], pawl_base[2] + fly_plane_offset[2]), slide_origin)
        fly.visual(
            Box((0.085, 0.030, 0.040)),
            origin=Origin(xyz=pawl),
            material=yellow,
            name=f"fly_lock_pawl_{'neg' if side < 0 else 'pos'}",
        )

    for i, t in enumerate((0.40, 0.55, 0.70, 0.85)):
        rung_base = _line_point(rear_foot_x, rear_foot_z, t)
        x, _, z = _sub((rung_base[0] + fly_plane_offset[0], 0.0, rung_base[2] + fly_plane_offset[2]), slide_origin)
        fly.visual(
            Box((0.086, 0.415, 0.032)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"fly_rung_{i}",
        )
        fly.visual(
            Box((0.055, 0.315, 0.008)),
            origin=Origin(xyz=(x + 0.003, 0.0, z + 0.018)),
            material=dark,
            name=f"fly_tread_grip_{i}",
        )

    model.articulation(
        "main_pivot",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.8, lower=-0.35, upper=0.55),
        meta={"detents_rad": [-0.35, -0.15, 0.0, 0.25, 0.55]},
    )
    model.articulation(
        "rear_to_fly",
        ArticulationType.PRISMATIC,
        parent=rear,
        child=fly,
        origin=Origin(xyz=slide_origin),
        axis=rear_up_axis,
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_section")
    rear = object_model.get_part("rear_section")
    fly = object_model.get_part("fly_section")
    pivot = object_model.get_articulation("main_pivot")
    slide = object_model.get_articulation("rear_to_fly")

    ctx.check(
        "main pivot has multiple lock detents",
        len(pivot.meta.get("detents_rad", ())) >= 5,
        details=f"detents={pivot.meta.get('detents_rad')}",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="z",
        min_overlap=1.4,
        name="front and rear sections form full height A-frame",
    )
    ctx.expect_within(
        fly,
        rear,
        axes="y",
        margin=0.015,
        name="fly section rides inside rear guide width",
    )
    ctx.expect_overlap(
        fly,
        rear,
        axes="xz",
        min_overlap=0.18,
        name="collapsed fly remains captured in rear guides",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 1.0}):
        extended_pos = ctx.part_world_position(fly)
        ctx.expect_within(
            fly,
            rear,
            axes="y",
            margin=0.015,
            name="extended fly stays centered in guides",
        )
        ctx.expect_overlap(
            fly,
            rear,
            axes="xz",
            min_overlap=0.16,
            name="extended fly retains insertion in rear guides",
        )

    ctx.check(
        "fly extends upward along rear stiles",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.85
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_rear = ctx.part_world_position(rear)
    rest_shoe = ctx.part_element_world_aabb(rear, elem="rear_shoe_pos")
    with ctx.pose({pivot: 0.45}):
        wide_rear = ctx.part_world_position(rear)
        wide_shoe = ctx.part_element_world_aabb(rear, elem="rear_shoe_pos")
        ctx.expect_overlap(
            rear,
            front,
            axes="z",
            min_overlap=1.2,
            name="wide pivot pose keeps both ladder sections upright",
        )
    ctx.check(
        "main pivot swings rear section",
        rest_rear is not None
        and wide_rear is not None
        and rest_shoe is not None
        and wide_shoe is not None
        and wide_shoe[1][0] < rest_shoe[0][0] - 0.30,
        details=f"rest={rest_rear}, wide={wide_rear}, rest_shoe={rest_shoe}, wide_shoe={wide_shoe}",
    )

    return ctx.report()


object_model = build_object_model()
