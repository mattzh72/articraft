from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

BASE_RADIUS = 0.110
BASE_THICKNESS = 0.028
HINGE_Z = 0.098
LOWER_ARM_LENGTH = 0.315
UPPER_ARM_LENGTH = 0.285
ARM_RAIL_RADIUS = 0.0055
ARM_RAIL_Y = 0.026
ARM_SPRING_Y = 0.034
ARM_SPRING_Z = 0.014
ROOT_BARREL_RADIUS = 0.009
ROOT_BARREL_LENGTH = 0.048
TIP_LUG_RADIUS = 0.008
TIP_LUG_LENGTH = 0.018
TIP_LUG_Y = 0.040


def _spring_mesh(
    *,
    start_x: float,
    end_x: float,
    center_y: float,
    center_z: float,
    coil_radius: float,
    turns: float,
    wire_radius: float,
) -> object:
    samples = max(64, int(turns * 22))
    helix_start_y = center_y + coil_radius
    points = [
        (start_x - 0.010, helix_start_y, center_z),
        (start_x, helix_start_y, center_z),
    ]
    for step in range(samples + 1):
        t = step / samples
        angle = turns * 2.0 * math.pi * t
        x = start_x + (end_x - start_x) * t
        y = center_y + coil_radius * math.cos(angle)
        z = center_z + coil_radius * math.sin(angle)
        points.append((x, y, z))
    points.extend(
        [
            (end_x, helix_start_y, center_z),
            (end_x + 0.010, helix_start_y, center_z),
        ]
    )
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=4,
        radial_segments=18,
        cap_ends=True,
    )


def _add_arm_frame(part, *, length: float, spring_turns: float) -> None:
    part.visual(
        Cylinder(radius=ROOT_BARREL_RADIUS, length=ROOT_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="root_barrel",
    )

    rail_length = length - 0.010
    for index, sign in enumerate((-1.0, 1.0)):
        rod_y = sign * ARM_RAIL_Y
        bridge_y = sign * 0.025
        spring_y = sign * ARM_SPRING_Y
        lug_y = sign * TIP_LUG_Y

        part.visual(
            Cylinder(radius=ARM_RAIL_RADIUS, length=rail_length),
            origin=Origin(
                xyz=(rail_length / 2.0, rod_y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            name=f"rail_{index}",
        )
        part.visual(
            Box((0.016, 0.010, 0.022)),
            origin=Origin(xyz=(0.017, bridge_y, 0.002)),
            name=f"root_link_{index}",
        )
        part.visual(
            Box((0.024, 0.016, 0.026)),
            origin=Origin(xyz=(length - 0.020, sign * 0.034, 0.004)),
            name=f"tip_link_{index}",
        )
        part.visual(
            Cylinder(radius=TIP_LUG_RADIUS, length=TIP_LUG_LENGTH),
            origin=Origin(
                xyz=(length, lug_y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            name=f"tip_lug_{index}",
        )
        part.visual(
            mesh_from_geometry(
                _spring_mesh(
                    start_x=0.018,
                    end_x=length - 0.018,
                    center_y=spring_y,
                    center_z=ARM_SPRING_Z,
                    coil_radius=0.006,
                    turns=spring_turns,
                    wire_radius=0.0015,
                ),
                f"{part.name}_spring_{index}",
            ),
            name=f"spring_{index}",
        )


def _build_shade_shell() -> object:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.040, 0.020),
            (0.066, 0.054),
            (0.082, 0.098),
            (0.083, 0.110),
        ],
        [
            (0.012, 0.004),
            (0.028, 0.022),
            (0.056, 0.054),
            (0.071, 0.096),
            (0.072, 0.102),
        ],
        segments=72,
    )
    shell.rotate_y(math.pi / 2.0)
    shell.translate(0.012, 0.0, -0.035)
    return shell


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_lamp")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_steel = model.material("arm_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.84, 0.85, 0.87, 1.0))
    switch_black = model.material("switch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_black,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=base_black,
        name="stem",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=base_black,
        name="shoulder",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.014, 0.018, 0.032)),
            origin=Origin(xyz=(-0.006, sign * 0.024, 0.068)),
            material=base_black,
            name=f"yoke_brace_{index}",
        )
        base.visual(
            Box((0.018, 0.024, 0.046)),
            origin=Origin(xyz=(-0.010, sign * 0.036, 0.076)),
            material=base_black,
            name=f"yoke_cheek_{index}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(
                xyz=(0.0, sign * 0.040, HINGE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=base_black,
            name=f"yoke_lug_{index}",
        )

    lower_arm = model.part("lower_arm")
    _add_arm_frame(lower_arm, length=LOWER_ARM_LENGTH, spring_turns=11.0)
    for name in ("root_barrel", "rail_0", "rail_1", "root_link_0", "root_link_1", "tip_link_0", "tip_link_1", "tip_lug_0", "tip_lug_1"):
        lower_arm.get_visual(name).material = arm_steel
    lower_arm.get_visual("spring_0").material = spring_steel
    lower_arm.get_visual("spring_1").material = spring_steel

    upper_arm = model.part("upper_arm")
    _add_arm_frame(upper_arm, length=UPPER_ARM_LENGTH, spring_turns=10.0)
    for name in ("root_barrel", "rail_0", "rail_1", "root_link_0", "root_link_1", "tip_link_0", "tip_link_1", "tip_lug_0", "tip_lug_1"):
        upper_arm.get_visual(name).material = arm_steel
    upper_arm.get_visual("spring_0").material = spring_steel
    upper_arm.get_visual("spring_1").material = spring_steel

    shade = model.part("shade")
    shade.visual(
        mesh_from_geometry(_build_shade_shell(), "desk_lamp_shade"),
        material=base_black,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.0075, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=base_black,
        name="root_barrel",
    )
    shade.visual(
        Box((0.032, 0.026, 0.028)),
        origin=Origin(xyz=(0.016, 0.0, -0.018)),
        material=base_black,
        name="bridge",
    )
    shade.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, -0.022)),
        material=base_black,
        name="switch_mount",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        shade.visual(
            Cylinder(radius=0.0035, length=0.006),
            origin=Origin(
                xyz=(-0.006, sign * 0.010, -0.022),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=base_black,
            name=f"switch_lug_{index}",
        )

    switch = model.part("switch")
    switch.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_black,
        name="barrel",
    )
    switch.visual(
        Box((0.012, 0.005, 0.010)),
        origin=Origin(xyz=(-0.008, 0.0, 0.007)),
        material=switch_black,
        name="stem",
    )
    switch.visual(
        Box((0.018, 0.006, 0.026)),
        origin=Origin(xyz=(-0.013, 0.0, 0.011), rpy=(0.0, 0.18, 0.0)),
        material=switch_black,
        name="toggle",
    )

    base_to_lower_arm = model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.95,
            upper=1.15,
            effort=16.0,
            velocity=1.4,
        ),
    )
    lower_arm_to_upper_arm = model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.45,
            upper=1.10,
            effort=12.0,
            velocity=1.6,
        ),
    )
    upper_arm_to_shade = model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.90,
            upper=0.65,
            effort=4.0,
            velocity=2.2,
        ),
    )
    shade_to_switch = model.articulation(
        "shade_to_switch",
        ArticulationType.REVOLUTE,
        parent=shade,
        child=switch,
        origin=Origin(xyz=(-0.006, 0.0, -0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=0.35,
            effort=0.5,
            velocity=4.0,
        ),
    )

    base_to_lower_arm.meta["qc_samples"] = [0.0, 0.55, 1.0]
    lower_arm_to_upper_arm.meta["qc_samples"] = [0.0, 0.7]
    upper_arm_to_shade.meta["qc_samples"] = [-0.5, 0.0, 0.45]
    shade_to_switch.meta["qc_samples"] = [-0.25, 0.25]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    switch = object_model.get_part("switch")

    base_hinge = object_model.get_articulation("base_to_lower_arm")
    elbow_hinge = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_hinge = object_model.get_articulation("upper_arm_to_shade")
    switch_hinge = object_model.get_articulation("shade_to_switch")

    ctx.allow_overlap(
        shade,
        switch,
        elem_a="switch_mount",
        elem_b="barrel",
        reason="The rear toggle's hinge barrel is intentionally captured inside the compact switch mount boss.",
    )
    ctx.allow_overlap(
        shade,
        switch,
        elem_a="switch_mount",
        elem_b="toggle",
        reason="The toggle lever intentionally roots inside the rear mount slot at its pivot.",
    )
    ctx.allow_overlap(
        shade,
        switch,
        elem_a="switch_mount",
        elem_b="stem",
        reason="The switch stem is intentionally retained inside the rear mount slot between the pivot barrel and the external toggle.",
    )

    rest_lower_tip = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="tip_lug_0"))
    with ctx.pose({base_hinge: 0.95}):
        raised_lower_tip = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="tip_lug_0"))
    ctx.check(
        "lower arm pitches upward from the base hinge",
        rest_lower_tip is not None
        and raised_lower_tip is not None
        and raised_lower_tip[2] > rest_lower_tip[2] + 0.15,
        details=f"rest={rest_lower_tip}, raised={raised_lower_tip}",
    )

    with ctx.pose({base_hinge: 0.55, elbow_hinge: 0.0}):
        neutral_upper_tip = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="tip_lug_0"))
    with ctx.pose({base_hinge: 0.55, elbow_hinge: 0.90}):
        raised_upper_tip = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="tip_lug_0"))
    ctx.check(
        "upper arm pitches upward at the elbow",
        neutral_upper_tip is not None
        and raised_upper_tip is not None
        and raised_upper_tip[2] > neutral_upper_tip[2] + 0.11,
        details=f"neutral={neutral_upper_tip}, raised={raised_upper_tip}",
    )

    with ctx.pose({base_hinge: 0.50, elbow_hinge: 0.55, shade_hinge: 0.45}):
        shade_up = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({base_hinge: 0.50, elbow_hinge: 0.55, shade_hinge: -0.70}):
        shade_down = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade tilts about the tip hinge",
        shade_up is not None and shade_down is not None and shade_down[2] < shade_up[2] - 0.03,
        details=f"up={shade_up}, down={shade_down}",
    )

    with ctx.pose(
        {
            base_hinge: 0.45,
            elbow_hinge: 0.35,
            shade_hinge: -0.15,
            switch_hinge: -0.30,
        }
    ):
        toggle_low = _aabb_center(ctx.part_element_world_aabb(switch, elem="toggle"))
    with ctx.pose(
        {
            base_hinge: 0.45,
            elbow_hinge: 0.35,
            shade_hinge: -0.15,
            switch_hinge: 0.30,
        }
    ):
        toggle_high = _aabb_center(ctx.part_element_world_aabb(switch, elem="toggle"))
    ctx.check(
        "rear toggle pivots on its local hinge",
        toggle_low is not None
        and toggle_high is not None
        and abs(toggle_high[2] - toggle_low[2]) > 0.005,
        details=f"low={toggle_low}, high={toggle_high}",
    )

    with ctx.pose({base_hinge: 0.85, elbow_hinge: 0.80, shade_hinge: -0.25}):
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.16,
            name="raised shade clears the round base",
        )

    return ctx.report()


object_model = build_object_model()
