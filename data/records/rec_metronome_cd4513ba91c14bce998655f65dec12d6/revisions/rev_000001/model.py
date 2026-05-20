from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_metronome")

    wood_dark = model.material("wood_dark", rgba=(0.28, 0.18, 0.10, 1.0))
    wood_mid = model.material("wood_mid", rgba=(0.36, 0.24, 0.14, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.63, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.86, 0.92, 0.22))

    shell_width = 0.175
    shell_depth = 0.115
    shell_height = 0.340
    wall_thickness = 0.010
    base_size = (0.240, 0.185, 0.022)

    body = model.part("body")
    body.visual(
        Box(base_size),
        origin=Origin(xyz=(0.0, 0.0, base_size[2] * 0.5)),
        material=wood_mid,
        name="base_plinth",
    )
    body.visual(
        Box((shell_width, shell_depth, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=wood_dark,
        name="interior_floor",
    )
    body.visual(
        Box((wall_thickness, shell_depth, shell_height)),
        origin=Origin(xyz=(shell_width * 0.5 - wall_thickness * 0.5, 0.0, 0.022 + shell_height * 0.5)),
        material=wood_dark,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, shell_depth, shell_height)),
        origin=Origin(xyz=(-shell_width * 0.5 + wall_thickness * 0.5, 0.0, 0.022 + shell_height * 0.5)),
        material=wood_dark,
        name="left_wall",
    )
    body.visual(
        Box((shell_width, wall_thickness, shell_height)),
        origin=Origin(xyz=(0.0, -shell_depth * 0.5 + wall_thickness * 0.5, 0.022 + shell_height * 0.5)),
        material=wood_dark,
        name="rear_wall",
    )
    body.visual(
        Box((shell_width, shell_depth, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.356)),
        material=wood_dark,
        name="roof",
    )
    body.visual(
        Box((shell_width, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.052, 0.035)),
        material=wood_dark,
        name="front_sill",
    )
    body.visual(
        Box((wall_thickness, 0.006, 0.288)),
        origin=Origin(xyz=(shell_width * 0.5 - wall_thickness * 0.5, 0.052, 0.188)),
        material=wood_dark,
        name="front_right_stile",
    )
    body.visual(
        Box((wall_thickness, 0.006, 0.288)),
        origin=Origin(xyz=(-shell_width * 0.5 + wall_thickness * 0.5, 0.052, 0.188)),
        material=wood_dark,
        name="front_left_stile",
    )
    body.visual(
        Box((shell_width, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, 0.052, 0.347)),
        material=wood_dark,
        name="front_header",
    )
    body.visual(
        Box((0.155, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.013, 0.323)),
        material=wood_mid,
        name="pivot_beam",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(shell_width * 0.5 + 0.006, -0.010, 0.140), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=wood_mid,
        name="winding_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.240, 0.185, 0.362)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
    )

    front_panel = model.part("front_panel")
    front_panel.visual(
        Cylinder(radius=0.004, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="hinge_barrel",
    )
    front_panel.visual(
        Box((0.152, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=wood_mid,
        name="bottom_rail",
    )
    front_panel.visual(
        Box((0.012, 0.006, 0.270)),
        origin=Origin(xyz=(-0.070, 0.0, 0.141)),
        material=wood_mid,
        name="left_rail",
    )
    front_panel.visual(
        Box((0.012, 0.006, 0.270)),
        origin=Origin(xyz=(0.070, 0.0, 0.141)),
        material=wood_mid,
        name="right_rail",
    )
    front_panel.visual(
        Box((0.152, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        material=wood_mid,
        name="top_rail",
    )
    front_panel.visual(
        Box((0.128, 0.003, 0.238)),
        origin=Origin(xyz=(0.0, 0.0005, 0.145)),
        material=glass,
        name="door_glass",
    )
    front_panel.inertial = Inertial.from_geometry(
        Box((0.152, 0.010, 0.288)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
    )

    gear = model.part("escapement_gear")
    gear.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="gear_wheel",
    )
    gear.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="gear_standoff",
    )
    gear.visual(
        Box((0.020, 0.006, 0.056)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=steel,
        name="gear_bracket",
    )
    for tooth_index in range(16):
        angle = 2.0 * math.pi * tooth_index / 16.0
        gear.visual(
            Box((0.006, 0.004, 0.007)),
            origin=Origin(
                xyz=(0.025 * math.cos(angle), 0.0, 0.025 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=brass,
            name=f"gear_tooth_{tooth_index}",
        )
    gear.inertial = Inertial.from_geometry(
        Box((0.060, 0.032, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    pendulum.visual(
        Box((0.014, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_steel,
        name="pivot_head",
    )
    pendulum.visual(
        Cylinder(radius=0.0020, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, -0.1325)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Box((0.020, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=steel,
        name="rod_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.250)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
    )

    weight = model.part("pendulum_weight")
    weight.visual(
        Box((0.006, 0.010, 0.050)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=brass,
        name="left_clamp_cheek",
    )
    weight.visual(
        Box((0.006, 0.010, 0.050)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=brass,
        name="right_clamp_cheek",
    )
    weight.visual(
        Box((0.022, 0.003, 0.050)),
        origin=Origin(xyz=(0.0, -0.0035, 0.0)),
        material=brass,
        name="rear_strap",
    )
    weight_block_mesh = mesh_from_geometry(
        LoftGeometry(
            [
                [(-0.010, 0.003, 0.024), (0.010, 0.003, 0.024), (0.008, 0.012, 0.024), (-0.008, 0.012, 0.024)],
                [(-0.014, 0.004, 0.000), (0.014, 0.004, 0.000), (0.012, 0.017, 0.000), (-0.012, 0.017, 0.000)],
                [(-0.018, 0.005, -0.024), (0.018, 0.005, -0.024), (0.014, 0.024, -0.024), (-0.014, 0.024, -0.024)],
            ],
            cap=True,
            closed=True,
        ),
        "metronome_slider_weight",
    )
    weight.visual(
        weight_block_mesh,
        material=brass,
        name="slider_block",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.036, 0.028, 0.054)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.010, 0.018, 0.008)),
        origin=Origin(xyz=(0.026, 0.010, 0.0)),
        material=brass,
        name="upper_wing",
    )
    winding_key.visual(
        Box((0.010, 0.018, 0.008)),
        origin=Origin(xyz=(0.026, -0.010, 0.0)),
        material=brass,
        name="lower_wing",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.038, 0.040, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_front_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_panel,
        origin=Origin(xyz=(0.0, 0.058, 0.044)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_gear",
        ArticulationType.FIXED,
        parent=body,
        child=gear,
        origin=Origin(xyz=(0.0, -0.022, 0.265)),
    )
    model.articulation(
        "body_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.001, 0.323)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.12,
            lower=-0.040,
            upper=0.070,
        ),
    )
    model.articulation(
        "body_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=winding_key,
        origin=Origin(xyz=(shell_width * 0.5 + 0.012, -0.010, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_panel = object_model.get_part("front_panel")
    gear = object_model.get_part("escapement_gear")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("pendulum_weight")
    key = object_model.get_part("winding_key")

    door_joint = object_model.get_articulation("body_to_front_panel")
    pendulum_joint = object_model.get_articulation("body_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_weight")

    ctx.expect_overlap(
        front_panel,
        body,
        axes="xz",
        min_overlap=0.12,
        name="front panel covers the housing opening",
    )
    ctx.expect_gap(
        front_panel,
        body,
        axis="y",
        positive_elem="door_glass",
        negative_elem="front_header",
        min_gap=0.0,
        max_gap=0.012,
        name="front panel sits just in front of the housing frame",
    )
    ctx.expect_overlap(
        gear,
        body,
        axes="xz",
        min_overlap=0.040,
        elem_a="gear_wheel",
        elem_b="rear_wall",
        name="escapement gear is mounted within the housing silhouette",
    )
    ctx.expect_gap(
        key,
        body,
        axis="x",
        positive_elem="key_shaft",
        negative_elem="winding_boss",
        max_gap=0.004,
        max_penetration=1e-5,
        name="winding key sits just outside the right-side boss",
    )

    closed_glass = ctx.part_element_world_aabb(front_panel, elem="door_glass")
    open_glass = None
    with ctx.pose({door_joint: 1.10}):
        open_glass = ctx.part_element_world_aabb(front_panel, elem="door_glass")
    glass_opens = (
        closed_glass is not None
        and open_glass is not None
        and open_glass[1][1] > closed_glass[1][1] + 0.080
        and open_glass[1][2] < closed_glass[1][2] - 0.080
    )
    ctx.check(
        "front panel opens downward from the bottom hinge",
        glass_opens,
        details=f"closed={closed_glass}, open={open_glass}",
    )

    left_swing = None
    right_swing = None
    with ctx.pose({pendulum_joint: -0.24}):
        left_swing = ctx.part_world_position(weight)
    with ctx.pose({pendulum_joint: 0.24}):
        right_swing = ctx.part_world_position(weight)
    ctx.check(
        "pendulum swings across the housing",
        left_swing is not None
        and right_swing is not None
        and left_swing[0] * right_swing[0] < 0.0
        and abs(left_swing[0] - right_swing[0]) > 0.060,
        details=f"left={left_swing}, right={right_swing}",
    )

    upper_weight = None
    lower_weight = None
    with ctx.pose({weight_joint: -0.035}):
        upper_weight = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: 0.065}):
        lower_weight = ctx.part_world_position(weight)
    ctx.check(
        "pendulum weight slides downward along the rod",
        upper_weight is not None
        and lower_weight is not None
        and lower_weight[2] < upper_weight[2] - 0.085,
        details=f"upper={upper_weight}, lower={lower_weight}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
