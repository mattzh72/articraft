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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_turntable")

    model.material("molded_black", rgba=(0.025, 0.028, 0.030, 1.0))
    model.material("matte_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("painted_deck", rgba=(0.24, 0.27, 0.28, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("dark_rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    model.material("warning_orange", rgba=(0.95, 0.34, 0.07, 1.0))
    model.material("screw_black", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("cue_red", rgba=(0.72, 0.04, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material="molded_black",
        name="plinth",
    )
    base.visual(
        Box((0.55, 0.39, 0.016)),
        origin=Origin(xyz=(-0.015, 0.0, 0.083)),
        material="painted_deck",
        name="top_deck",
    )

    # Thick utility bumpers and corner armor, slightly embedded into the plinth.
    for name, y in (("front_bumper", -0.242), ("rear_bumper", 0.242)):
        base.visual(
            Box((0.66, 0.030, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.041)),
            material="matte_graphite",
            name=name,
        )
    for name, x in (("side_bumper_0", -0.320), ("side_bumper_1", 0.320)):
        base.visual(
            Box((0.030, 0.42, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material="matte_graphite",
            name=name,
        )
    for ix, x in enumerate((-0.285, 0.285)):
        for iy, y in enumerate((-0.195, 0.195)):
            base.visual(
                Box((0.075, 0.060, 0.030)),
                origin=Origin(xyz=(x, y, 0.097)),
                material="molded_black",
                name=f"corner_armor_{ix}_{iy}",
            )

    # Exposed service handles: rails tied back into the molded front/rear ribs.
    for name, y in (("front_handle", -0.266), ("rear_handle", 0.266)):
        base.visual(
            Cylinder(radius=0.010, length=0.250),
            origin=Origin(xyz=(0.0, y, 0.047), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="molded_black",
            name=name,
        )
        for ix, x in enumerate((-0.145, 0.145)):
            base.visual(
                Box((0.030, 0.030, 0.050)),
                origin=Origin(xyz=(x, y * 0.965, 0.041)),
                material="molded_black",
                name=f"{name}_bracket_{ix}",
            )

    # Rubber feet press into the underside for a serviceable rugged stance.
    for ix, x in enumerate((-0.245, 0.245)):
        for iy, y in enumerate((-0.165, 0.165)):
            base.visual(
                Cylinder(radius=0.035, length=0.020),
                origin=Origin(xyz=(x, y, -0.009), rpy=(0.0, 0.0, 0.0)),
                material="dark_rubber",
                name=f"foot_{ix}_{iy}",
            )

    # Central bearing stack is fixed to the plinth; the platter rotates above it.
    base.visual(
        Cylinder(radius=0.083, length=0.020),
        origin=Origin(xyz=(-0.060, 0.0, 0.101)),
        material="brushed_steel",
        name="bearing_collar",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(xyz=(-0.060, 0.0, 0.122)),
        material="matte_graphite",
        name="bearing_hub",
    )
    for idx, ang in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(
                xyz=(-0.060 + 0.110 * math.cos(ang), 0.110 * math.sin(ang), 0.093),
            ),
            material="screw_black",
            name=f"bearing_bolt_{idx}",
        )

    # Top-deck fasteners and service slots.
    fasteners = [
        (-0.240, -0.145),
        (-0.240, 0.145),
        (0.085, -0.145),
        (0.085, 0.145),
        (0.225, -0.165),
        (0.225, 0.165),
    ]
    for idx, (x, y) in enumerate(fasteners):
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, 0.0925)),
            material="brushed_steel",
            name=f"deck_screw_{idx}",
        )
        base.visual(
            Box((0.014, 0.0022, 0.0015)),
            origin=Origin(xyz=(x, y, 0.095), rpy=(0.0, 0.0, (idx % 3) * 0.55)),
            material="screw_black",
            name=f"screw_slot_{idx}",
        )

    # Tonearm tower, guard ears, and arm-rest cradle are rigidly fixed to the base.
    base.visual(
        Cylinder(radius=0.034, length=0.021),
        origin=Origin(xyz=(0.190, 0.130, 0.1015)),
        material="brushed_steel",
        name="tonearm_base",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.093),
        origin=Origin(xyz=(0.190, 0.130, 0.1585)),
        material="matte_graphite",
        name="tonearm_column",
    )
    for ix, xoff in enumerate((-0.032, 0.032)):
        base.visual(
            Box((0.014, 0.030, 0.052)),
            origin=Origin(xyz=(0.190 + xoff, 0.130, 0.137)),
            material="brushed_steel",
            name=f"pivot_guard_{ix}",
        )
    base.visual(
        Cylinder(radius=0.015, length=0.079),
        origin=Origin(xyz=(0.195, 0.220, 0.1145)),
        material="matte_graphite",
        name="arm_rest_post",
    )
    base.visual(
        Box((0.055, 0.018, 0.018)),
        origin=Origin(xyz=(0.195, 0.220, 0.163)),
        material="brushed_steel",
        name="arm_rest_cradle",
    )

    # Rugged front controls: a guarded speed knob and a protected start button.
    base.visual(
        Cylinder(radius=0.034, length=0.007),
        origin=Origin(xyz=(0.210, -0.158, 0.0945)),
        material="brushed_steel",
        name="knob_boss",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.105, -0.165, 0.094)),
        material="screw_black",
        name="button_bezel",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.163, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_steel",
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.074, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="brushed_steel",
        name="lower_hub",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="dark_rubber",
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="brushed_steel",
        name="record_clamp_boss",
    )
    platter.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material="brushed_steel",
        name="spindle",
    )
    for idx in range(24):
        ang = idx * (2.0 * math.pi / 24.0)
        platter.visual(
            Box((0.012, 0.003, 0.005)),
            origin=Origin(
                xyz=(0.158 * math.cos(ang), 0.158 * math.sin(ang), 0.025),
                rpy=(0.0, 0.0, ang),
            ),
            material="screw_black",
            name=f"strobe_mark_{idx}",
        )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="brushed_steel",
        name="pivot_cap",
    )
    tonearm.visual(
        Box((0.060, 0.020, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, -0.002)),
        material="brushed_steel",
        name="arm_yoke",
    )
    tonearm.visual(
        Cylinder(radius=0.0065, length=0.255),
        origin=Origin(xyz=(-0.158, 0.0, -0.010), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="matte_graphite",
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.055, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_graphite",
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.028, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="weight_shaft",
    )
    tonearm.visual(
        Box((0.054, 0.032, 0.008)),
        origin=Origin(xyz=(-0.300, 0.0, -0.017)),
        material="brushed_steel",
        name="headshell",
    )
    tonearm.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(-0.318, 0.0, -0.026)),
        material="molded_black",
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(-0.326, 0.0, -0.033), rpy=(0.0, 0.22, 0.0)),
        material="screw_black",
        name="stylus",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="matte_graphite",
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.0035, 0.019, 0.003)),
        origin=Origin(xyz=(0.0, 0.006, 0.0195)),
        material="warning_orange",
        name="knob_pointer",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.020, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material="cue_red",
        name="button_cap",
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platter,
        origin=Origin(xyz=(-0.060, 0.0, 0.133)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "base_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tonearm,
        origin=Origin(xyz=(0.190, 0.130, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.30, upper=0.58, effort=2.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.05, friction=0.015),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.210, -0.158, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.9, upper=0.9, effort=0.6, velocity=2.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.05),
    )
    model.articulation(
        "base_to_start_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=start_button,
        origin=Origin(xyz=(0.105, -0.165, 0.097)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.006, effort=1.0, velocity=0.15),
        motion_properties=MotionProperties(damping=0.03, friction=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    speed_knob = object_model.get_part("speed_knob")
    start_button = object_model.get_part("start_button")

    platter_joint = object_model.get_articulation("base_to_platter")
    tonearm_joint = object_model.get_articulation("base_to_tonearm")
    button_joint = object_model.get_articulation("base_to_start_button")

    ctx.expect_gap(
        platter,
        base,
        axis="z",
        positive_elem="lower_hub",
        negative_elem="bearing_hub",
        max_penetration=0.0001,
        max_gap=0.0015,
        name="platter hub rides just above bearing hub",
    )
    ctx.expect_overlap(
        platter,
        base,
        axes="xy",
        elem_a="lower_hub",
        elem_b="bearing_collar",
        min_overlap=0.06,
        name="platter is centered over bearing collar",
    )
    ctx.expect_gap(
        tonearm,
        base,
        axis="z",
        positive_elem="pivot_cap",
        negative_elem="tonearm_column",
        max_penetration=0.0001,
        max_gap=0.001,
        name="tonearm cap is seated on pivot column",
    )
    ctx.expect_overlap(
        tonearm,
        base,
        axes="xy",
        elem_a="pivot_cap",
        elem_b="tonearm_column",
        min_overlap=0.018,
        name="tonearm pivot remains captured over column",
    )
    ctx.expect_gap(
        speed_knob,
        base,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="knob_boss",
        max_penetration=0.0001,
        max_gap=0.001,
        name="speed knob is supported by guard boss",
    )
    ctx.expect_gap(
        start_button,
        base,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_bezel",
        max_penetration=0.0001,
        max_gap=0.001,
        name="start button rests in protected bezel",
    )

    # Prove the continuous platter joint is a coaxial rotating stage.
    with ctx.pose({platter_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            platter,
            base,
            axes="xy",
            elem_a="lower_hub",
            elem_b="bearing_collar",
            min_overlap=0.06,
            name="rotated platter remains coaxial with bearing",
        )

    def _center_x_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    stylus_rest = _center_x_y(ctx.part_element_world_aabb(tonearm, elem="stylus"))
    with ctx.pose({tonearm_joint: 0.58}):
        stylus_inner = _center_x_y(ctx.part_element_world_aabb(tonearm, elem="stylus"))
    ctx.check(
        "tonearm sweeps inward across platter",
        stylus_rest is not None
        and stylus_inner is not None
        and stylus_inner[1] < stylus_rest[1] - 0.10,
        details=f"rest={stylus_rest}, inner={stylus_inner}",
    )

    button_rest = ctx.part_world_aabb(start_button)
    with ctx.pose({button_joint: 0.006}):
        button_pressed = ctx.part_world_aabb(start_button)
    ctx.check(
        "start button travels downward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0][2] < button_rest[0][2] - 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
