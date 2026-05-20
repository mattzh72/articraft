from __future__ import annotations

from math import pi

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


LINK_1_LENGTH = 0.28
LINK_2_LENGTH = 0.25
LINK_WIDTH = 0.026
LINK_THICKNESS = 0.014
HUB_RADIUS = 0.024
HUB_THICKNESS = 0.014

BRACKET_HUB_Z = 0.000
LINK_1_Z = 0.016
LINK_2_Z = 0.030
LED_HUB_Z = 0.044


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_led_folding_arm")

    wall_white = model.material("powder_coated_white", rgba=(0.86, 0.86, 0.82, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_hardware = model.material("dark_hinge_hardware", rgba=(0.06, 0.065, 0.07, 1.0))
    warm_diffuser = model.material("warm_led_diffuser", rgba=(1.0, 0.86, 0.46, 0.78))
    screw_shadow = model.material("recessed_screw_shadow", rgba=(0.025, 0.025, 0.025, 1.0))

    bracket = model.part("wall_bracket")
    bracket.visual(
        Box((0.105, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=wall_white,
        name="mount_plate",
    )
    bracket.visual(
        Box((0.034, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=wall_white,
        name="boss_standoff",
    )
    bracket.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_HUB_Z)),
        material=dark_hardware,
        name="pivot_boss",
    )
    for z, name in ((0.055, "upper_screw"), (-0.055, "lower_screw")):
        bracket.visual(
            Cylinder(radius=0.0085, length=0.002),
            origin=Origin(xyz=(0.0, -0.0272, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=screw_shadow,
            name=name,
        )

    link_1 = model.part("link_1")
    link_1.visual(
        Box((LINK_WIDTH, LINK_1_LENGTH, LINK_THICKNESS)),
        origin=Origin(xyz=(0.0, LINK_1_LENGTH / 2.0, LINK_1_Z)),
        material=aluminum,
        name="arm_blade",
    )
    link_1.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LINK_1_Z)),
        material=dark_hardware,
        name="base_hub",
    )
    link_1.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(xyz=(0.0, LINK_1_LENGTH, LINK_1_Z)),
        material=dark_hardware,
        name="distal_hub",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        Box((LINK_WIDTH, LINK_2_LENGTH, LINK_THICKNESS)),
        origin=Origin(xyz=(0.0, LINK_2_LENGTH / 2.0, LINK_2_Z)),
        material=aluminum,
        name="arm_blade",
    )
    link_2.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LINK_2_Z)),
        material=dark_hardware,
        name="proximal_hub",
    )
    link_2.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(xyz=(0.0, LINK_2_LENGTH, LINK_2_Z)),
        material=dark_hardware,
        name="terminal_hub",
    )

    led_bar = model.part("led_bar")
    led_bar.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LED_HUB_Z)),
        material=dark_hardware,
        name="pivot_cap",
    )
    led_bar.visual(
        Box((0.020, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.044)),
        material=dark_hardware,
        name="front_lug",
    )
    led_bar.visual(
        Box((0.020, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, 0.055, 0.018)),
        material=dark_hardware,
        name="drop_yoke",
    )
    led_bar.visual(
        Box((0.320, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.095, 0.006)),
        material=wall_white,
        name="light_housing",
    )
    led_bar.visual(
        Box((0.290, 0.025, 0.004)),
        origin=Origin(xyz=(0.0, 0.100, -0.0058)),
        material=warm_diffuser,
        name="led_diffuser",
    )
    for x in (-0.105, -0.0525, 0.0, 0.0525, 0.105):
        led_bar.visual(
            Box((0.018, 0.018, 0.002)),
            origin=Origin(xyz=(x, 0.100, -0.0087)),
            material=warm_diffuser,
            name=f"led_chip_{int((x + 0.105) / 0.0525)}",
        )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, LINK_1_LENGTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "bar_yaw",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=led_bar,
        origin=Origin(xyz=(0.0, LINK_2_LENGTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=-1.8, upper=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("wall_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    led_bar = object_model.get_part("led_bar")
    base_yaw = object_model.get_articulation("base_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    bar_yaw = object_model.get_articulation("bar_yaw")

    joints = (base_yaw, elbow_yaw, bar_yaw)
    ctx.check(
        "three visible pivots are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint_types={[j.articulation_type for j in joints]}",
    )
    ctx.check(
        "all pivots use vertical folding axes",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_gap(
        link_1,
        bracket,
        axis="z",
        positive_elem="base_hub",
        negative_elem="pivot_boss",
        min_gap=0.0,
        max_gap=0.0005,
        name="base hinge has a thin bearing clearance",
    )
    ctx.expect_overlap(
        link_1,
        bracket,
        axes="xy",
        elem_a="base_hub",
        elem_b="pivot_boss",
        min_overlap=0.026,
        name="base hinge hub is centered on bracket boss",
    )
    ctx.expect_gap(
        link_2,
        link_1,
        axis="z",
        positive_elem="proximal_hub",
        negative_elem="distal_hub",
        min_gap=0.0,
        max_gap=0.0005,
        name="elbow hinge plates are stacked with clearance",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="xy",
        elem_a="proximal_hub",
        elem_b="distal_hub",
        min_overlap=0.030,
        name="elbow pivot hubs share a common axis",
    )
    ctx.expect_gap(
        led_bar,
        link_2,
        axis="z",
        positive_elem="pivot_cap",
        negative_elem="terminal_hub",
        min_gap=0.0,
        max_gap=0.0005,
        name="terminal hinge cap clears the arm hub",
    )
    ctx.expect_overlap(
        led_bar,
        link_2,
        axes="xy",
        elem_a="pivot_cap",
        elem_b="terminal_hub",
        min_overlap=0.030,
        name="LED bar pivot cap is coaxial with terminal hub",
    )

    rest_link_2_pos = ctx.part_world_position(link_2)
    with ctx.pose({base_yaw: 0.75}):
        swung_link_2_pos = ctx.part_world_position(link_2)
    ctx.check(
        "first arm link swings the elbow outward",
        rest_link_2_pos is not None
        and swung_link_2_pos is not None
        and abs(swung_link_2_pos[0] - rest_link_2_pos[0]) > 0.15,
        details=f"rest={rest_link_2_pos}, swung={swung_link_2_pos}",
    )

    rest_led_pos = ctx.part_world_position(led_bar)
    with ctx.pose({elbow_yaw: 1.0}):
        folded_led_pos = ctx.part_world_position(led_bar)
    ctx.check(
        "elbow revolute joint folds the second link",
        rest_led_pos is not None
        and folded_led_pos is not None
        and abs(folded_led_pos[0] - rest_led_pos[0]) > 0.15,
        details=f"rest={rest_led_pos}, folded={folded_led_pos}",
    )

    def _aabb_center_x(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return (lo[0] + hi[0]) / 2.0

    rest_bar_x = _aabb_center_x(led_bar, "light_housing")
    with ctx.pose({bar_yaw: 0.85}):
        rotated_bar_x = _aabb_center_x(led_bar, "light_housing")
    ctx.check(
        "terminal joint rotates the flat LED strip bar",
        rest_bar_x is not None and rotated_bar_x is not None and abs(rotated_bar_x - rest_bar_x) > 0.045,
        details=f"rest_x={rest_bar_x}, rotated_x={rotated_bar_x}",
    )

    return ctx.report()


object_model = build_object_model()
