from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.76, 0.75, 1.0))
    dark_groove = model.material("shadowed_channel", rgba=(0.16, 0.18, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    orange = model.material("safety_orange", rgba=(1.0, 0.42, 0.07, 1.0))
    yellow = model.material("warning_yellow", rgba=(1.0, 0.80, 0.10, 1.0))
    rope_mat = model.material("braided_rope", rgba=(0.78, 0.68, 0.50, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.47, 0.50, 0.50, 1.0))

    base = model.part("base")
    fly = model.part("fly_section")

    rail_y = -0.042
    base_rail_x = 0.34
    base_length = 4.40
    fly_rail_x = 0.245
    fly_y = 0.0815
    fly_length = 4.36
    rung_x_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    # Base section: a pair of heavy extruded side rails, joined by round rungs.
    for side, x in enumerate((-base_rail_x, base_rail_x)):
        base.visual(
            Box((0.070, 0.080, base_length)),
            origin=Origin(xyz=(x, rail_y, base_length / 2.0)),
            material=aluminum,
            name=f"base_rail_{side}",
        )
        base.visual(
            Box((0.050, 0.010, base_length - 0.22)),
            origin=Origin(xyz=(x, rail_y + 0.044, base_length / 2.0 + 0.02)),
            material=dark_groove,
            name=f"base_channel_shadow_{side}",
        )
        base.visual(
            Box((0.100, 0.110, 0.060)),
            origin=Origin(xyz=(x, rail_y, base_length + 0.030)),
            material=aluminum,
            name=f"base_top_cap_{side}",
        )
        base.visual(
            Box((0.200, 0.170, 0.085)),
            origin=Origin(xyz=(x, rail_y - 0.006, 0.040)),
            material=rubber,
            name=f"base_foot_{side}",
        )

    base_rung_zs = [0.46 + 0.36 * i for i in range(11)]
    for i, z in enumerate(base_rung_zs):
        base.visual(
            Cylinder(radius=0.024, length=0.690),
            origin=Origin(xyz=(0.0, rail_y, z), rpy=rung_x_axis.rpy),
            material=aluminum,
            name=f"base_rung_{i}",
        )
        # Ribbed anti-slip pads on alternating rungs.
        if i % 2 == 0:
            base.visual(
                Box((0.330, 0.016, 0.018)),
                origin=Origin(xyz=(0.0, rail_y + 0.018, z + 0.018)),
                material=dark_groove,
                name=f"base_tread_{i}",
            )

    # Nylon guide blocks keep the fly section nested just in front of the base.
    for idx, z in enumerate((0.72, 3.78)):
        for side, x in enumerate((-fly_rail_x, fly_rail_x)):
            base.visual(
                Box((0.120, 0.036, 0.135)),
                origin=Origin(xyz=(x, 0.016, z)),
                material=steel,
                name=f"guide_bracket_{idx}_{side}",
            )
            base.visual(
                Box((0.078, 0.018, 0.115)),
                origin=Origin(xyz=(x, 0.035, z)),
                material=rubber,
                name=("guide_pad_0_0" if idx == 0 and side == 0 else f"guide_pad_{idx}_{side}"),
            )

    # A wide stabilizer shoe and diagonal-looking gussets give the base a real
    # utility-ladder stance instead of reading as two free sticks.
    base.visual(
        Box((0.900, 0.120, 0.050)),
        origin=Origin(xyz=(0.0, rail_y - 0.020, 0.090)),
        material=rubber,
        name="stabilizer_shoe",
    )
    for side, x in enumerate((-0.20, 0.20)):
        base.visual(
            Box((0.060, 0.055, 0.340)),
            origin=Origin(xyz=(x, rail_y - 0.015, 0.240), rpy=(0.0, 0.34 if x < 0 else -0.34, 0.0)),
            material=steel,
            name=f"shoe_gusset_{side}",
        )

    # Fly section: narrower rails ride in the base guide pads.  Its frame is at
    # the lower slide shoe, so positive prismatic motion extends upward.
    for side, x in enumerate((-fly_rail_x, fly_rail_x)):
        fly.visual(
            Box((0.058, 0.075, fly_length)),
            origin=Origin(xyz=(x, fly_y, fly_length / 2.0)),
            material=aluminum,
            name=("fly_rail_0" if side == 0 else "fly_rail_1"),
        )
        fly.visual(
            Box((0.040, 0.009, fly_length - 0.18)),
            origin=Origin(xyz=(x, fly_y + 0.040, fly_length / 2.0)),
            material=dark_groove,
            name=f"fly_channel_shadow_{side}",
        )
        fly.visual(
            Box((0.120, 0.030, 0.155)),
            origin=Origin(xyz=(x, fly_y + 0.047, 0.120)),
            material=rubber,
            name=f"lower_slide_shoe_{side}",
        )
        fly.visual(
            Box((0.100, 0.030, 0.155)),
            origin=Origin(xyz=(x, fly_y + 0.047, fly_length - 0.110)),
            material=rubber,
            name=f"upper_slide_shoe_{side}",
        )

    fly_rung_zs = [0.30 + 0.36 * i for i in range(11)]
    for i, z in enumerate(fly_rung_zs):
        fly.visual(
            Cylinder(radius=0.022, length=0.520),
            origin=Origin(xyz=(0.0, fly_y, z), rpy=rung_x_axis.rpy),
            material=aluminum,
            name=f"fly_rung_{i}",
        )
        if i % 2 == 1:
            fly.visual(
                Box((0.280, 0.014, 0.016)),
                origin=Origin(xyz=(0.0, fly_y + 0.017, z + 0.017)),
                material=dark_groove,
                name=f"fly_tread_{i}",
            )

    # Orange rung-lock hinge mounts on the fly section, supported back to the
    # rails by steel straps.  The pawls themselves are revolute children below.
    pawl_xs = (-0.158, 0.158)
    for idx, x in enumerate(pawl_xs):
        fly.visual(
            Box((0.120, 0.024, 0.090)),
            origin=Origin(xyz=(x, 0.102, 0.880)),
            material=steel,
            name=f"lock_bridge_{idx}",
        )
        for cheek, cx in enumerate((x - 0.048, x + 0.048)):
            fly.visual(
                Box((0.018, 0.052, 0.094)),
                origin=Origin(xyz=(cx, 0.137, 0.880)),
                material=orange,
                name=f"lock_cheek_{idx}_{cheek}",
            )

    # Top pulley bracket and rope cleat.  The bracket is deliberately connected
    # to the top rung via the lower bridge so it reads as load-bearing hardware.
    fly.visual(
        Box((0.140, 0.074, 0.060)),
        origin=Origin(xyz=(0.0, 0.104, 3.940)),
        material=steel,
        name="pulley_bridge",
    )
    for side, x in enumerate((-0.040, 0.040)):
        fly.visual(
            Box((0.012, 0.060, 0.220)),
            origin=Origin(xyz=(x, 0.162, 4.070)),
            material=steel,
            name=f"pulley_cheek_{side}",
        )
    fly.visual(
        Box((0.145, 0.090, 0.040)),
        origin=Origin(xyz=(0.0, 0.115, 0.305)),
        material=steel,
        name="rope_cleat_bridge",
    )
    fly.visual(
        Box((0.100, 0.040, 0.075)),
        origin=Origin(xyz=(0.0, 0.160, 0.330)),
        material=yellow,
        name="rope_cleat",
    )

    rope = tube_from_spline_points(
        [
            (-0.065, 0.170, 0.350),
            (-0.065, 0.170, 4.050),
            (0.000, 0.230, 4.225),
            (0.065, 0.170, 4.050),
            (0.065, 0.170, 0.350),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    fly.visual(
        mesh_from_geometry(rope, "haul_rope"),
        origin=Origin(),
        material=rope_mat,
        name="haul_rope",
    )

    slide = model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=2.35),
        motion_properties=MotionProperties(damping=12.0, friction=4.0),
    )
    slide.meta["description"] = "Fly section slides upward while retaining more than a meter of overlap in the base rails."

    for idx, x in enumerate(pawl_xs):
        pawl = model.part(f"lock_pawl_{idx}")
        pawl.visual(
            Cylinder(radius=0.017, length=0.078),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="hinge_barrel",
        )
        pawl.visual(
            Box((0.064, 0.022, 0.235)),
            origin=Origin(xyz=(0.0, 0.0, -0.118)),
            material=orange,
            name="pawl_plate",
        )
        pawl.visual(
            Box((0.086, 0.036, 0.050)),
            origin=Origin(xyz=(0.0, -0.018, -0.238), rpy=(0.0, 0.24 if x < 0 else -0.24, 0.0)),
            material=orange,
            name="rung_hook",
        )
        model.articulation(
            f"fly_to_lock_pawl_{idx}",
            ArticulationType.REVOLUTE,
            parent=fly,
            child=pawl,
            origin=Origin(xyz=(x, 0.140, 0.880)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.35, upper=0.95),
            motion_properties=MotionProperties(damping=1.0, friction=0.3),
        )

    pulley = model.part("pulley")
    pulley.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_groove,
        name="grooved_wheel",
    )
    pulley.visual(
        Cylinder(radius=0.025, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub_boss",
    )
    pulley.visual(
        Cylinder(radius=0.011, length=0.068),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_pin",
    )
    model.articulation(
        "fly_to_pulley",
        ArticulationType.CONTINUOUS,
        parent=fly,
        child=pulley,
        origin=Origin(xyz=(0.0, 0.162, 4.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fly = object_model.get_part("fly_section")
    slide = object_model.get_articulation("base_to_fly")

    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=3.0,
        name="collapsed sections are deeply nested",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="fly_rail_0",
        negative_elem="guide_pad_0_0",
        name="fly rail is closely guided without binding",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 2.35}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.0,
            name="extended fly remains retained in base rails",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "extension slide moves upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 2.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    for idx in range(2):
        joint = object_model.get_articulation(f"fly_to_lock_pawl_{idx}")
        limits = joint.motion_limits
        ctx.check(
            f"lock pawl {idx} has realistic swing limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0
            and limits.upper > 0.6,
            details=f"limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
