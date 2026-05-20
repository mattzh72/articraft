from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_rear_trailing_arm_suspension")

    painted_steel = model.material("satin_black_painted_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.12, 0.12, 0.12, 1.0))
    machined = model.material("machined_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))

    cross_member = model.part("cross_member")
    cross_member.visual(
        Box((0.16, 1.24, 0.070)),
        origin=Origin(),
        material=painted_steel,
        name="flat_transverse_beam",
    )

    bearing_ring_meshes = {}
    for index, sign in enumerate((-1.0, 1.0)):
        y = sign * 0.700
        # Small circular bearing rings at each end of the flat transverse member.
        # The ring hole is the actual horizontal trailing-arm pivot axis.
        ring_mesh = mesh_from_geometry(TorusGeometry(0.055, 0.010), f"trailing_arm_bearing_ring_{index}")
        bearing_ring_meshes[index] = ring_mesh
        cross_member.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pivot_ring_{index}",
        )

        # Side webs flank the bushing and weld the bearing ring back to the beam.
        # They sit on the inboard side of the ring so the trailing arm can pass
        # outboard without colliding with the support.
        for side_x, x in enumerate((-0.060, 0.060)):
            cross_member.visual(
                Box((0.022, 0.105, 0.130)),
                origin=Origin(xyz=(x, sign * 0.650, 0.0)),
                material=painted_steel,
                name=f"pivot_web_{index}_{side_x}",
            )

    # Shared wheel/tire geometry.  The wheel helper's spin axis is local X; the
    # visual origins below rotate local X to the suspension's lateral Y axle.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.158,
            0.070,
            rim=WheelRim(
                inner_radius=0.105,
                flange_height=0.012,
                flange_thickness=0.005,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.048,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.070, hole_diameter=0.008),
            ),
            face=WheelFace(dish_depth=0.012, front_inset=0.005, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.024),
        ),
        "rear_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.220,
            0.110,
            inner_radius=0.155,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.055),
            tread=TireTread(style="block", depth=0.009, count=24, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.012, radius=0.005),
        ),
        "rear_tire",
    )

    for index, sign in enumerate((-1.0, 1.0)):
        arm = model.part(f"trailing_arm_{index}")
        outboard_y = sign * 0.045
        arm.visual(
            Cylinder(radius=0.046, length=0.110),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="pivot_bushing",
        )
        arm.visual(
            Cylinder(radius=0.020, length=0.130),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=machined,
            name="pivot_sleeve",
        )
        arm.visual(
            Box((0.080, 0.026, 0.090)),
            origin=Origin(xyz=(0.055, outboard_y, -0.055)),
            material=dark_steel,
            name="bushing_drop_plate",
        )
        arm.visual(
            Box((0.700, 0.060, 0.046)),
            origin=Origin(xyz=(0.430, outboard_y, -0.085)),
            material=painted_steel,
            name="trailing_box_arm",
        )
        arm.visual(
            Box((0.120, 0.080, 0.185)),
            origin=Origin(xyz=(0.820, outboard_y, -0.085)),
            material=dark_steel,
            name="hub_carrier",
        )
        arm.visual(
            Cylinder(radius=0.026, length=0.070),
            origin=Origin(xyz=(0.860, sign * 0.055, -0.085), rpy=(pi / 2.0, 0.0, 0.0)),
            material=machined,
            name="stub_axle_seat",
        )

        model.articulation(
            f"cross_member_to_trailing_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=cross_member,
            child=arm,
            origin=Origin(xyz=(0.0, sign * 0.700, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1800.0, velocity=2.0, lower=-0.35, upper=0.50),
        )

        hub = model.part(f"wheel_hub_{index}")
        wheel_center_y = sign * 0.145
        hub.visual(
            tire_mesh,
            origin=Origin(xyz=(0.0, wheel_center_y, 0.0), rpy=(0.0, 0.0, pi / 2.0)),
            material=rubber,
            name="tire",
        )
        hub.visual(
            wheel_mesh,
            origin=Origin(xyz=(0.0, wheel_center_y, 0.0), rpy=(0.0, 0.0, pi / 2.0)),
            material=machined,
            name="rim_and_hub",
        )
        hub.visual(
            Cylinder(radius=0.045, length=0.030),
            origin=Origin(xyz=(0.0, sign * 0.105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=machined,
            name="inner_hub_boss",
        )

        model.articulation(
            f"trailing_arm_to_wheel_hub_{index}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=hub,
            origin=Origin(xyz=(0.860, 0.0, -0.085)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cross_member = object_model.get_part("cross_member")

    for index in range(2):
        sign = -1.0 if index == 0 else 1.0
        arm = object_model.get_part(f"trailing_arm_{index}")
        hub = object_model.get_part(f"wheel_hub_{index}")
        arm_joint = object_model.get_articulation(f"cross_member_to_trailing_arm_{index}")
        hub_joint = object_model.get_articulation(f"trailing_arm_to_wheel_hub_{index}")

        ctx.allow_overlap(
            cross_member,
            arm,
            elem_a=f"pivot_ring_{index}",
            elem_b="pivot_bushing",
            reason=(
                "The rubber pivot bushing is intentionally captured in the bearing ring; "
                "the ring mesh is treated as a bearing proxy around the horizontal mount axis."
            ),
        )
        ctx.expect_within(
            arm,
            cross_member,
            axes="xz",
            inner_elem="pivot_bushing",
            outer_elem=f"pivot_ring_{index}",
            margin=0.004,
            name=f"pivot bushing {index} stays concentric in bearing ring",
        )
        ctx.expect_overlap(
            arm,
            cross_member,
            axes="y",
            elem_a="pivot_bushing",
            elem_b=f"pivot_ring_{index}",
            min_overlap=0.015,
            name=f"pivot bushing {index} passes through bearing ring width",
        )

        ctx.expect_origin_gap(
            hub,
            cross_member,
            axis="x",
            min_gap=0.75,
            name=f"wheel hub {index} sits at the trailing arm tip",
        )
        if sign > 0.0:
            ctx.expect_gap(
                hub,
                arm,
                axis="y",
                positive_elem="tire",
                negative_elem="hub_carrier",
                min_gap=0.002,
                name=f"wheel tire {index} clears the outboard carrier face",
            )
        else:
            ctx.expect_gap(
                arm,
                hub,
                axis="y",
                positive_elem="hub_carrier",
                negative_elem="tire",
                min_gap=0.002,
                name=f"wheel tire {index} clears the outboard carrier face",
            )
        ctx.expect_contact(
            arm,
            hub,
            elem_a="stub_axle_seat",
            elem_b="inner_hub_boss",
            contact_tol=0.001,
            name=f"stub axle {index} seats against wheel hub boss",
        )

        rest_pos = ctx.part_world_position(hub)
        with ctx.pose({arm_joint: 0.45}):
            raised_pos = ctx.part_world_position(hub)
        ctx.check(
            f"trailing arm {index} pivots upward on positive travel",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[2] > rest_pos[2] + 0.25,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

        rest_axle_pos = ctx.part_world_position(hub)
        with ctx.pose({hub_joint: 1.25}):
            spun_axle_pos = ctx.part_world_position(hub)
        ctx.check(
            f"wheel hub {index} spins in place about its axle",
            rest_axle_pos is not None
            and spun_axle_pos is not None
            and max(abs(a - b) for a, b in zip(rest_axle_pos, spun_axle_pos)) < 1e-6,
            details=f"rest={rest_axle_pos}, spun={spun_axle_pos}",
        )

    return ctx.report()


object_model = build_object_model()
