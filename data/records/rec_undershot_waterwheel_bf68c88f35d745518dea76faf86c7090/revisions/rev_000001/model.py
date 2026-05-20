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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", color=(0.48, 0.30, 0.16, 1.0))
    dark_wood = model.material("dark_wet_wood", color=(0.28, 0.15, 0.08, 1.0))
    stone = model.material("cut_stone", color=(0.48, 0.47, 0.43, 1.0))
    iron = model.material("dark_iron", color=(0.08, 0.075, 0.07, 1.0))
    water = model.material("shallow_water", color=(0.10, 0.36, 0.58, 0.55))

    support = model.part("support")

    # Broad raceway and trough.  The fixed structure deliberately spans much
    # farther than the wheel so the rotating assembly reads as the small moving
    # member carried between two side frames.
    support.visual(
        Box((1.80, 1.55, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=stone,
        name="foundation_slab",
    )
    support.visual(
        Box((1.36, 1.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.08, 0.089)),
        material=water,
        name="water_sheet",
    )
    for index, x in enumerate((-0.78, 0.78)):
        support.visual(
            Box((0.08, 1.55, 0.24)),
            origin=Origin(xyz=(x, 0.0, 0.20)),
            material=stone,
            name=f"channel_wall_{index}",
        )
    support.visual(
        Box((1.64, 0.09, 0.40)),
        origin=Origin(xyz=(0.0, -0.66, 0.28)),
        material=stone,
        name="upstream_trough_edge",
    )
    support.visual(
        Box((1.48, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.70, 0.18)),
        material=stone,
        name="tailrace_lip",
    )

    # Two timber A-frames with shallow bearing saddles.  The saddles stop just
    # below the rotating axle, while side cheeks flank it with clearance.
    frame_xs = (-0.60, 0.60)
    for side_index, x in enumerate(frame_xs):
        support.visual(
            Box((0.12, 0.94, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.13)),
            material=dark_wood,
            name=f"base_rail_{side_index}",
        )
        for brace_index, y_sign in enumerate((-1.0, 1.0)):
            dy = -0.30 * y_sign
            dz = 0.54
            length = math.hypot(dy, dz)
            angle = math.atan2(dz, dy)
            support.visual(
                Box((0.085, length, 0.065)),
                origin=Origin(
                    xyz=(x, 0.31 * y_sign, 0.43),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=weathered_wood,
                name=f"diagonal_brace_{side_index}_{brace_index}",
            )
        for post_index, y in enumerate((-0.105, 0.105)):
            support.visual(
                Box((0.085, 0.075, 0.54)),
                origin=Origin(xyz=(x, y, 0.39)),
                material=weathered_wood,
                name=f"bearing_post_{side_index}_{post_index}",
            )
        support.visual(
            Box((0.14, 0.20, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.69)),
            material=weathered_wood,
            name=f"bearing_saddle_{side_index}",
        )
        for cheek_index, y in enumerate((-0.07, 0.07)):
            support.visual(
                Box((0.14, 0.035, 0.20)),
                origin=Origin(xyz=(x, y, 0.78)),
                material=weathered_wood,
                name=f"bearing_cheek_{side_index}_{cheek_index}",
            )
        support.visual(
            Box((0.14, 0.25, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.905)),
            material=weathered_wood,
            name=f"bearing_cap_{side_index}",
        )
        support.visual(
            Box((0.11, 0.86, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.965)),
            material=weathered_wood,
            name=f"upper_tie_{side_index}",
        )

    support.visual(
        Box((1.32, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.46, 0.965)),
        material=weathered_wood,
        name="rear_cross_tie",
    )
    support.visual(
        Box((1.32, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.46, 0.965)),
        material=weathered_wood,
        name="front_cross_tie",
    )

    wheel = model.part("wheel")

    # Child frame sits at the axle center.  All wheel geometry is centered on
    # that frame so the CONTINUOUS joint turns it around local +X.
    wheel.visual(
        Cylinder(radius=0.030, length=1.16),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.078, length=0.30),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub",
    )

    for rim_index, x in enumerate((-0.13, 0.13)):
        rim_geometry = TorusGeometry(
            0.315,
            0.018,
            radial_segments=18,
            tubular_segments=64,
        ).rotate_y(math.pi / 2.0).translate(x, 0.0, 0.0)
        wheel.visual(
            mesh_from_geometry(rim_geometry, f"wheel_rim_mesh_{rim_index}"),
            material=weathered_wood,
            name=f"rim_{rim_index}",
        )

    spoke_length = 0.245
    spoke_center_radius = 0.195
    for side_index, x in enumerate((-0.105, 0.105)):
        for spoke_index in range(8):
            angle = spoke_index * math.tau / 8.0
            wheel.visual(
                Box((0.055, spoke_length, 0.028)),
                origin=Origin(
                    xyz=(
                        x,
                        spoke_center_radius * math.cos(angle),
                        spoke_center_radius * math.sin(angle),
                    ),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=weathered_wood,
                name=f"spoke_{side_index}_{spoke_index}",
            )

    # Broad flat boards catch the low stream: undershot paddles lie tangent to
    # the wheel and span across the two side rims.
    for paddle_index in range(12):
        angle = paddle_index * math.tau / 12.0
        wheel.visual(
            Box((0.34, 0.118, 0.026)),
            origin=Origin(
                xyz=(0.0, 0.367 * math.cos(angle), 0.367 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=weathered_wood,
            name=f"paddle_{paddle_index}",
        )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("axle_spin")

    ctx.check(
        "wheel uses continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "axle axis is horizontal",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )

    support_aabb = ctx.part_world_aabb(support)
    wheel_aabb = ctx.part_world_aabb(wheel)
    if support_aabb is not None and wheel_aabb is not None:
        support_min, support_max = support_aabb
        wheel_min, wheel_max = wheel_aabb
        support_size = tuple(support_max[i] - support_min[i] for i in range(3))
        wheel_size = tuple(wheel_max[i] - wheel_min[i] for i in range(3))
        ctx.check(
            "fixed support is broader than wheel",
            support_size[0] > wheel_size[0] * 1.35 and support_size[1] > wheel_size[1] * 1.45,
            details=f"support_size={support_size}, wheel_size={wheel_size}",
        )
    else:
        ctx.fail("fixed support is broader than wheel", "missing AABB")

    ctx.expect_within(
        wheel,
        support,
        axes="x",
        margin=0.0,
        name="axle lies between side frames",
    )
    for saddle_index in (0, 1):
        ctx.expect_overlap(
            wheel,
            support,
            axes="x",
            min_overlap=0.045,
            elem_a="axle",
            elem_b=f"bearing_saddle_{saddle_index}",
            name=f"axle crosses bearing saddle {saddle_index}",
        )
        ctx.expect_gap(
            wheel,
            support,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="axle",
            negative_elem=f"bearing_saddle_{saddle_index}",
            name=f"axle sits just above saddle {saddle_index}",
        )

    rest_position = ctx.part_world_position(wheel)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_position = ctx.part_world_position(wheel)
    ctx.check(
        "continuous spin keeps axle center fixed",
        rest_position is not None
        and spun_position is not None
        and all(abs(rest_position[i] - spun_position[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
