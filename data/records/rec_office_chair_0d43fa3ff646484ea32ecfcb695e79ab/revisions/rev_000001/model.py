from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_office_chair")

    black_vinyl = model.material("black_vinyl", rgba=(0.02, 0.022, 0.025, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.055, 0.058, 0.065, 1.0))
    dark_metal = model.material("dark_powder_coat", rgba=(0.12, 0.13, 0.14, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    # Root: five-star base, lower gas sleeve, and drafting-height foot ring.
    base = model.part("wheeled_base")
    base.visual(
        Cylinder(radius=0.085, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_metal,
        name="center_hub",
    )
    base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.047, 0.0), (0.047, 0.52), (0.031, 0.52), (0.031, 0.0)],
                segments=56,
                closed=True,
            ),
            "outer_gas_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=chrome,
        name="outer_gas_sleeve",
    )
    base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.0235, 0.0), (0.040, 0.0), (0.040, 0.018), (0.0235, 0.018)],
                segments=56,
                closed=True,
            ),
            "upper_column_bushing",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.607)),
        material=dark_metal,
        name="upper_column_bushing",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.34, tube=0.012, radial_segments=18, tubular_segments=72), "foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=chrome,
        name="foot_ring",
    )

    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        c = math.cos(theta)
        s = math.sin(theta)
        base.visual(
            Box((0.43, 0.055, 0.035)),
            origin=Origin(xyz=(0.285 * c, 0.285 * s, 0.098), rpy=(0.0, 0.0, theta)),
            material=dark_metal,
            name=f"star_leg_{i}",
        )

    for i in range(4):
        theta = math.pi / 4.0 + i * math.pi / 2.0
        c = math.cos(theta)
        s = math.sin(theta)
        base.visual(
            Box((0.31, 0.022, 0.016)),
            origin=Origin(xyz=(0.18 * c, 0.18 * s, 0.425), rpy=(0.0, 0.0, theta)),
            material=chrome,
            name=f"foot_ring_spoke_{i}",
        )

    # Height-adjustable inner gas column.  It remains captured inside the hollow
    # lower sleeve at full extension.
    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.024, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=chrome,
        name="inner_gas_column",
    )
    lift_column.visual(
        Cylinder(radius=0.080, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        material=dark_metal,
        name="swivel_bearing",
    )
    model.articulation(
        "base_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.18, lower=0.0, upper=0.18),
    )

    # Swiveling seat carried by the center post.
    seat = model.part("seat")
    seat.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.50, 0.46, exponent=3.2, segments=72), 0.080),
            "rounded_seat_cushion",
        ),
        origin=Origin(xyz=(0.0, -0.035, 0.078)),
        material=black_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.34, 0.27, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.030)),
        material=dark_metal,
        name="underseat_plate",
    )
    seat.visual(
        Cylinder(radius=0.065, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="seat_bearing_socket",
    )
    seat.visual(
        Box((0.028, 0.25, 0.070)),
        origin=Origin(xyz=(0.278, -0.020, 0.092)),
        material=dark_metal,
        name="left_arm_hinge_mount",
    )
    seat.visual(
        Box((0.028, 0.25, 0.070)),
        origin=Origin(xyz=(-0.278, -0.020, 0.092)),
        material=dark_metal,
        name="right_arm_hinge_mount",
    )
    for side, x in (("left", 0.225), ("right", -0.225)):
        seat.visual(
            Box((0.140, 0.062, 0.036)),
            origin=Origin(xyz=(x, -0.020, 0.061)),
            material=dark_metal,
            name=f"{side}_arm_side_bracket",
        )
    seat.visual(
        Box((0.330, 0.115, 0.034)),
        origin=Origin(xyz=(0.0, 0.168, 0.061)),
        material=dark_metal,
        name="rear_back_bracket",
    )
    seat.visual(
        Box((0.36, 0.040, 0.052)),
        origin=Origin(xyz=(0.0, 0.230, 0.092)),
        material=dark_metal,
        name="back_hinge_mount",
    )
    model.articulation(
        "lift_column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )

    # Reclining back with a transverse hinge just behind the rear of the seat.
    back = model.part("back")
    back.visual(
        Cylinder(radius=0.018, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="back_hinge_barrel",
    )
    back.visual(
        Box((0.36, 0.042, 0.032)),
        origin=Origin(xyz=(0.0, 0.026, 0.018)),
        material=dark_metal,
        name="back_lower_bridge",
    )
    for x in (-0.145, 0.145):
        back.visual(
            Box((0.030, 0.036, 0.290)),
            origin=Origin(xyz=(x, 0.050, 0.145)),
            material=dark_metal,
            name=f"back_upright_{0 if x < 0 else 1}",
        )
    back.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.44, 0.56, exponent=3.0, segments=72), 0.065),
            "rounded_back_cushion",
        ),
        origin=Origin(xyz=(0.0, 0.090, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_vinyl,
        name="back_cushion",
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(0.0, 0.230, 0.120)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.08, upper=0.42),
    )

    # Flip-up armrests, each with its own side hinge.
    for side, sign, axis in (("left", 1.0, (0.0, -1.0, 0.0)), ("right", -1.0, (0.0, 1.0, 0.0))):
        armrest = model.part(f"{side}_armrest")
        armrest.visual(
            Cylinder(radius=0.015, length=0.235),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_pin",
        )
        armrest.visual(
            Box((0.105, 0.225, 0.014)),
            origin=Origin(xyz=(sign * 0.042, 0.0, 0.006)),
            material=dark_metal,
            name="hinge_leaf",
        )
        armrest.visual(
            Box((0.026, 0.052, 0.092)),
            origin=Origin(xyz=(sign * 0.082, -0.075, 0.046)),
            material=dark_metal,
            name="front_support",
        )
        armrest.visual(
            Box((0.026, 0.052, 0.092)),
            origin=Origin(xyz=(sign * 0.082, 0.095, 0.046)),
            material=dark_metal,
            name="rear_support",
        )
        armrest.visual(
            mesh_from_geometry(
                ExtrudeGeometry(superellipse_profile(0.105, 0.305, exponent=3.5, segments=48), 0.035),
                f"{side}_arm_pad",
            ),
            origin=Origin(xyz=(sign * 0.122, 0.010, 0.066)),
            material=charcoal,
            name="arm_pad",
        )
        model.articulation(
            f"seat_to_{side}_armrest",
            ArticulationType.REVOLUTE,
            parent=seat,
            child=armrest,
            origin=Origin(xyz=(sign * 0.278, -0.020, 0.132)),
            axis=axis,
            motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.65),
        )

    # Five swiveling caster forks, each with a freely spinning wheel on its axle.
    caster_wheel_mesh = mesh_from_geometry(
        TireGeometry(
            0.042,
            0.040,
            inner_radius=0.018,
            tread=TireTread(style="ribbed", depth=0.0025, count=16, land_ratio=0.58),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_wheel_tire",
    )
    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        c = math.cos(theta)
        s = math.sin(theta)

        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=dark_metal,
            name="swivel_cap",
        )
        fork.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=chrome,
            name="caster_stem",
        )
        fork.visual(
            Box((0.088, 0.032, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.061)),
            material=dark_metal,
            name="fork_crown",
        )
        fork.visual(
            Box((0.012, 0.030, 0.078)),
            origin=Origin(xyz=(-0.034, 0.0, -0.092)),
            material=dark_metal,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.012, 0.030, 0.078)),
            origin=Origin(xyz=(0.034, 0.0, -0.092)),
            material=dark_metal,
            name="fork_cheek_1",
        )
        model.articulation(
            f"base_to_caster_fork_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=(0.50 * c, 0.50 * s, 0.0805), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            caster_wheel_mesh,
            origin=Origin(),
            material=rubber,
            name="rubber_wheel",
        )
        wheel.visual(
            Cylinder(radius=0.019, length=0.083),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="axle_hub",
        )
        model.articulation(
            f"caster_fork_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.112)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("wheeled_base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")

    lift = object_model.get_articulation("base_to_lift_column")
    back_recline = object_model.get_articulation("seat_to_back")
    left_flip = object_model.get_articulation("seat_to_left_armrest")
    right_flip = object_model.get_articulation("seat_to_right_armrest")

    ctx.allow_overlap(
        back,
        seat,
        elem_a="back_hinge_barrel",
        elem_b="back_hinge_mount",
        reason="The back hinge barrel is intentionally seated through the rear hinge block.",
    )
    ctx.allow_overlap(
        left_armrest,
        seat,
        elem_a="hinge_pin",
        elem_b="left_arm_hinge_mount",
        reason="The flip-up armrest hinge pin is captured inside the side hinge mount.",
    )
    ctx.allow_overlap(
        right_armrest,
        seat,
        elem_a="hinge_pin",
        elem_b="right_arm_hinge_mount",
        reason="The flip-up armrest hinge pin is captured inside the side hinge mount.",
    )
    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="upper_column_bushing",
        elem_b="inner_gas_column",
        reason="The gas lift column slides through a tight upper bushing represented with local interference.",
    )
    for i in range(5):
        ctx.allow_overlap(
            f"caster_fork_{i}",
            f"caster_wheel_{i}",
            elem_a="fork_cheek_0",
            elem_b="axle_hub",
            reason="The wheel axle hub passes through the caster fork cheek as a captured axle.",
        )
        ctx.allow_overlap(
            f"caster_fork_{i}",
            f"caster_wheel_{i}",
            elem_a="fork_cheek_1",
            elem_b="axle_hub",
            reason="The wheel axle hub passes through the caster fork cheek as a captured axle.",
        )

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="inner_gas_column",
        outer_elem="outer_gas_sleeve",
        margin=0.003,
        name="gas column centered in sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="inner_gas_column",
        elem_b="outer_gas_sleeve",
        min_overlap=0.25,
        name="gas column retained in sleeve",
    )
    ctx.expect_gap(
        seat,
        lift_column,
        axis="z",
        positive_elem="seat_bearing_socket",
        negative_elem="swivel_bearing",
        max_gap=0.002,
        max_penetration=0.004,
        name="seat sits on swivel bearing",
    )
    ctx.expect_overlap(
        back,
        seat,
        axes="x",
        elem_a="back_hinge_barrel",
        elem_b="back_hinge_mount",
        min_overlap=0.30,
        name="back hinge spans rear seat mount",
    )
    ctx.expect_overlap(
        left_armrest,
        seat,
        axes="y",
        elem_a="hinge_pin",
        elem_b="left_arm_hinge_mount",
        min_overlap=0.18,
        name="left arm hinge pin captured",
    )
    ctx.expect_overlap(
        right_armrest,
        seat,
        axes="y",
        elem_a="hinge_pin",
        elem_b="right_arm_hinge_mount",
        min_overlap=0.18,
        name="right arm hinge pin captured",
    )

    rest_seat_pos = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.18}):
        raised_seat_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="inner_gas_column",
            elem_b="outer_gas_sleeve",
            min_overlap=0.20,
            name="raised gas column remains inserted",
        )
    ctx.check(
        "height adjustment raises seat",
        rest_seat_pos is not None
        and raised_seat_pos is not None
        and raised_seat_pos[2] > rest_seat_pos[2] + 0.15,
        details=f"rest={rest_seat_pos}, raised={raised_seat_pos}",
    )

    rest_back = ctx.part_world_aabb(back)
    with ctx.pose({back_recline: 0.38}):
        reclined_back = ctx.part_world_aabb(back)
    ctx.check(
        "back reclines rearward",
        rest_back is not None
        and reclined_back is not None
        and reclined_back[1][1] > rest_back[1][1] + 0.03,
        details=f"rest={rest_back}, reclined={reclined_back}",
    )

    rest_left = ctx.part_world_aabb(left_armrest)
    rest_right = ctx.part_world_aabb(right_armrest)
    with ctx.pose({left_flip: 1.55, right_flip: 1.55}):
        raised_left = ctx.part_world_aabb(left_armrest)
        raised_right = ctx.part_world_aabb(right_armrest)
    ctx.check(
        "armrests flip upward",
        rest_left is not None
        and rest_right is not None
        and raised_left is not None
        and raised_right is not None
        and raised_left[1][2] > rest_left[1][2] + 0.03
        and raised_right[1][2] > rest_right[1][2] + 0.03,
        details=f"left={rest_left}->{raised_left}, right={rest_right}->{raised_right}",
    )

    for i in range(5):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="fork_cheek_0",
            elem_b="axle_hub",
            min_overlap=0.006,
            name=f"caster {i} axle captured by fork cheek",
        )

    return ctx.report()


object_model = build_object_model()
