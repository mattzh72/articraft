from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            math.cos(2.0 * math.pi * i / segments) * radius,
            math.sin(2.0 * math.pi * i / segments) * radius,
        )
        for i in range(segments)
    ]


def _rectangle_profile(width: float, height: float) -> list[tuple[float, float]]:
    return [
        (-width / 2.0, -height / 2.0),
        (width / 2.0, -height / 2.0),
        (width / 2.0, height / 2.0),
        (-width / 2.0, height / 2.0),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_undershot_waterwheel")

    galvanized = model.material("galvanized_frame", color=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_bearing_steel", color=(0.16, 0.17, 0.18, 1.0))
    black_plastic = model.material("black_molded_rim", color=(0.05, 0.06, 0.055, 1.0))
    paddle_blue = model.material("blue_snap_paddles", color=(0.05, 0.25, 0.75, 1.0))
    fastener = model.material("zinc_fasteners", color=(0.78, 0.76, 0.68, 1.0))

    frame = model.part("frame")
    # Two identical extruded/skid base rails with stamped cross-ties keep the
    # support architecture obvious and low-cost.
    for idx, y in enumerate((-0.56, 0.56)):
        frame.visual(
            Box((1.38, 0.070, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.025)),
            material=galvanized,
            name=f"base_rail_{idx}",
        )

    for idx, x in enumerate((-0.58, 0.58)):
        frame.visual(
            Box((0.075, 1.20, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=galvanized,
            name=f"cross_tie_{idx}",
        )

    axle_z = 0.62
    for side_idx, y in enumerate((-0.50, 0.50)):
        # Simple A-frame made from constant-section stampings/extrusions.
        for post_idx, x in enumerate((-0.27, 0.27)):
            frame.visual(
                Box((0.060, 0.060, 0.96)),
                origin=Origin(xyz=(x, y, 0.53)),
                material=galvanized,
                name=f"upright_{side_idx}_{post_idx}",
            )

        for diag_idx, foot_x in enumerate((-0.50, 0.50)):
            end_x = -0.27 if foot_x < 0.0 else 0.27
            dx = end_x - foot_x
            dz = 0.85 - 0.10
            length = math.hypot(dx, dz)
            pitch = math.atan2(-dz, dx)
            frame.visual(
                Box((length, 0.050, 0.050)),
                origin=Origin(
                    xyz=((foot_x + end_x) / 2.0, y, (0.85 + 0.10) / 2.0),
                    rpy=(0.0, pitch, 0.0),
                ),
                material=galvanized,
                name=f"diagonal_{side_idx}_{diag_idx}",
            )

        frame.visual(
            Box((0.66, 0.070, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.94)),
            material=galvanized,
            name=f"top_tie_{side_idx}",
        )

    # One reusable split bearing saddle mesh, instanced on both side frames.
    bearing_profile = _rectangle_profile(0.22, 0.22)
    bearing_hole = _circle_profile(0.052, 56)
    bearing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            bearing_profile,
            [bearing_hole],
            0.110,
            center=True,
        ),
        "split_bearing_block",
    )
    for idx, y in enumerate((-0.46, 0.46)):
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, axle_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_{idx}",
        )
        for arm_idx, x in enumerate((-0.175, 0.175)):
            frame.visual(
                Box((0.190, 0.085, 0.055)),
                origin=Origin(xyz=(x, y, axle_z)),
                material=galvanized,
                name=f"bearing_arm_{idx}_{arm_idx}",
            )
        # Clamp strap, parting seam, and four through-bolts communicate the
        # snap/bolt assembly order without adding separate moving parts.
        frame.visual(
            Box((0.24, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, y, axle_z + 0.095)),
            material=dark_steel,
            name=f"bearing_strap_{idx}",
        )
        for bolt_idx, (x, z) in enumerate(
            ((-0.073, axle_z + 0.066), (0.073, axle_z + 0.066), (-0.073, axle_z - 0.066), (0.073, axle_z - 0.066))
        ):
            cap_y = y + (0.055 if y > 0.0 else -0.055)
            frame.visual(
                Cylinder(radius=0.014, length=0.014),
                origin=Origin(xyz=(x, cap_y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=fastener,
                name=f"bearing_bolt_{idx}_{bolt_idx}",
            )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.035, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.120, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="hub",
    )

    for idx, y in enumerate((-0.32, 0.32)):
        wheel.visual(
            mesh_from_geometry(TorusGeometry(0.420, 0.026, radial_segments=18, tubular_segments=36), f"side_ring_{idx}_mesh"),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_plastic,
            name=f"side_ring_{idx}",
        )

    # Radial ribs are identical straight bars spanning across both side rings,
    # which makes the wheel manufacturable as two simple molded halves snapped
    # around the axle/hub instead of many separate spoke parts.
    for idx in range(8):
        theta = 2.0 * math.pi * idx / 8.0
        radius = 0.265
        wheel.visual(
            Box((0.315, 0.680, 0.045)),
            origin=Origin(
                xyz=(math.cos(theta) * radius, 0.0, math.sin(theta) * radius),
                rpy=(0.0, -theta, 0.0),
            ),
            material=black_plastic,
            name=f"spoke_{idx}",
        )

    # Flat undershot paddles are simple constant-thickness snap-in plates.  Each
    # plate overlaps the molded rings at its inner edge so the wheel reads as a
    # supported rotating assembly rather than loose blades.
    for idx in range(12):
        theta = 2.0 * math.pi * idx / 12.0
        radius = 0.485
        wheel.visual(
            Box((0.225, 0.740, 0.130)),
            origin=Origin(
                xyz=(math.cos(theta) * radius, 0.0, math.sin(theta) * radius),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=paddle_blue,
            name=f"paddle_{idx}",
        )
        wheel.visual(
            Box((0.245, 0.035, 0.026)),
            origin=Origin(
                xyz=(math.cos(theta) * 0.425, 0.0, math.sin(theta) * 0.425),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=black_plastic,
            name=f"paddle_clip_{idx}",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_0",
        elem_b="axle",
        reason="The steel axle is intentionally captured inside the simplified split bearing bore.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_1",
        elem_b="axle",
        reason="The steel axle is intentionally captured inside the simplified split bearing bore.",
    )

    ctx.check(
        "waterwheel uses a continuous centered axle joint",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_0",
        margin=0.0,
        name="axle is centered through bearing 0",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_1",
        margin=0.0,
        name="axle is centered through bearing 1",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_0",
        min_overlap=0.08,
        name="axle passes through bearing 0",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_1",
        min_overlap=0.08,
        name="axle passes through bearing 1",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem="bearing_1",
        negative_elem="side_ring_1",
        min_gap=0.020,
        name="wheel clears positive side bearing",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="side_ring_0",
        negative_elem="bearing_0",
        min_gap=0.020,
        name="wheel clears negative side bearing",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem="bearing_0",
            margin=0.0,
            name="axle remains centered after rotation",
        )
    ctx.check(
        "wheel rotates without translating off the axle",
        rest_pos is not None and turned_pos is not None and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, rotated={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
