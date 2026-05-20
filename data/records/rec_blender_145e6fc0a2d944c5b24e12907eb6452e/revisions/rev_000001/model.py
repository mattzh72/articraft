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
    model = ArticulatedObject(name="immersion_blender_stand")

    warm_white = model.material("warm_white_plastic", rgba=(0.92, 0.91, 0.86, 1.0))
    satin_gray = model.material("satin_gray_plastic", rgba=(0.42, 0.44, 0.46, 1.0))
    dark_gray = model.material("dark_rubber", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    blade_steel = model.material("sharpened_steel", rgba=(0.84, 0.84, 0.80, 1.0))
    button_blue = model.material("soft_blue_button", rgba=(0.05, 0.22, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.075, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=warm_white,
        name="housing_tower",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_gray,
        name="weighted_foot",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.658)),
        material=dark_gray,
        name="top_cap",
    )
    base.visual(
        Box((0.055, 0.100, 0.410)),
        origin=Origin(xyz=(0.098, 0.0, 0.380)),
        material=warm_white,
        name="dock_spine",
    )
    base.visual(
        Box((0.090, 0.012, 0.410)),
        origin=Origin(xyz=(0.163, -0.039, 0.380)),
        material=satin_gray,
        name="dock_side_0",
    )
    base.visual(
        Box((0.090, 0.012, 0.410)),
        origin=Origin(xyz=(0.163, 0.039, 0.380)),
        material=satin_gray,
        name="dock_side_1",
    )
    base.visual(
        Box((0.090, 0.018, 0.020)),
        origin=Origin(xyz=(0.158, -0.044, 0.173)),
        material=satin_gray,
        name="dock_lower_lip_0",
    )
    base.visual(
        Box((0.090, 0.018, 0.020)),
        origin=Origin(xyz=(0.158, 0.044, 0.173)),
        material=satin_gray,
        name="dock_lower_lip_1",
    )
    base.visual(
        Box((0.078, 0.010, 0.018)),
        origin=Origin(xyz=(0.155, -0.052, 0.586)),
        material=dark_gray,
        name="dock_top_lip_0",
    )
    base.visual(
        Box((0.078, 0.010, 0.018)),
        origin=Origin(xyz=(0.155, 0.052, 0.586)),
        material=dark_gray,
        name="dock_top_lip_1",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.033, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=warm_white,
        name="grip_shell",
    )
    wand.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.398)),
        material=dark_gray,
        name="grip_cap",
    )
    wand.visual(
        Cylinder(radius=0.021, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=satin_gray,
        name="shaft_collar",
    )
    wand.visual(
        Cylinder(radius=0.012, length=0.236),
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        material=brushed_steel,
        name="metal_shaft",
    )
    wand.visual(
        Box((0.026, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=brushed_steel,
        name="tip_bridge",
    )
    wand.visual(
        Box((0.024, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.034, -0.178)),
        material=brushed_steel,
        name="tip_fork_0",
    )
    wand.visual(
        Box((0.024, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.034, -0.178)),
        material=brushed_steel,
        name="tip_fork_1",
    )
    wand.visual(
        Box((0.004, 0.032, 0.066)),
        origin=Origin(xyz=(0.0335, 0.0, 0.286)),
        material=satin_gray,
        name="button_recess",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.008, 0.026, 0.048)),
        origin=Origin(xyz=(0.0022, 0.0, 0.0)),
        material=button_blue,
        name="button_cap",
    )

    guard = model.part("blade_guard")
    guard.visual(
        Cylinder(radius=0.008, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    guard.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=brushed_steel,
        name="guard_neck",
    )
    guard.visual(
        Cylinder(radius=0.0045, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=brushed_steel,
        name="blade_spindle",
    )
    guard.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.038, tube=0.0035, radial_segments=16, tubular_segments=48),
            "guard_top_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=brushed_steel,
        name="guard_top_ring",
    )
    guard.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.045, tube=0.0035, radial_segments=16, tubular_segments=48),
            "guard_bottom_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=brushed_steel,
        name="guard_bottom_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        guard.visual(
            Box((0.042, 0.006, 0.006)),
            origin=Origin(
                xyz=(0.021 * math.cos(angle), 0.021 * math.sin(angle), -0.038),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"guard_spoke_{index}",
        )
    for index in range(8):
        angle = index * math.tau / 8.0
        guard.visual(
            Cylinder(radius=0.0025, length=0.047),
            origin=Origin(
                xyz=(0.0425 * math.cos(angle), 0.0425 * math.sin(angle), -0.061),
            ),
            material=brushed_steel,
            name=f"cage_rib_{index}",
        )
    guard.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=blade_steel,
        name="blade_hub",
    )
    guard.visual(
        Box((0.060, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.065), rpy=(0.0, 0.0, 0.25)),
        material=blade_steel,
        name="blade_0",
    )
    guard.visual(
        Box((0.052, 0.009, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.065), rpy=(0.0, 0.0, math.pi / 2.0 - 0.22)),
        material=blade_steel,
        name="blade_1",
    )

    model.articulation(
        "base_to_wand",
        ArticulationType.PRISMATIC,
        parent=base,
        child=wand,
        origin=Origin(xyz=(0.175, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.45, lower=0.0, upper=0.260),
    )
    model.articulation(
        "wand_to_power_button",
        ArticulationType.PRISMATIC,
        parent=wand,
        child=power_button,
        origin=Origin(xyz=(0.037, 0.0, 0.286)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.03, lower=0.0, upper=0.006),
    )
    model.articulation(
        "wand_to_blade_guard",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=guard,
        origin=Origin(xyz=(0.0, 0.0, -0.178)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wand = object_model.get_part("wand")
    guard = object_model.get_part("blade_guard")
    button = object_model.get_part("power_button")
    lift = object_model.get_articulation("base_to_wand")
    guard_hinge = object_model.get_articulation("wand_to_blade_guard")
    button_press = object_model.get_articulation("wand_to_power_button")

    ctx.expect_gap(
        wand,
        base,
        axis="x",
        positive_elem="grip_shell",
        negative_elem="dock_spine",
        min_gap=0.010,
        max_gap=0.030,
        name="wand clears the dock spine while seated",
    )
    ctx.expect_overlap(
        wand,
        base,
        axes="z",
        elem_a="grip_shell",
        elem_b="dock_side_0",
        min_overlap=0.16,
        name="docked wand is captured by the tall side cradle",
    )
    ctx.expect_gap(
        wand,
        guard,
        axis="y",
        positive_elem="tip_fork_1",
        negative_elem="hinge_barrel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="blade guard barrel is seated between the fork ears",
    )

    rest_wand_pos = ctx.part_world_position(wand)
    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({lift: 0.260, button_press: 0.006}):
        ctx.expect_gap(
            wand,
            base,
            axis="z",
            positive_elem="grip_shell",
            negative_elem="dock_side_0",
            min_gap=0.060,
            name="wand lifts clear of the dock rails",
        )
        lifted_wand_pos = ctx.part_world_position(wand)
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "prismatic lift moves the wand upward",
        rest_wand_pos is not None
        and lifted_wand_pos is not None
        and lifted_wand_pos[2] > rest_wand_pos[2] + 0.24,
        details=f"rest={rest_wand_pos}, lifted={lifted_wand_pos}",
    )
    ctx.check(
        "power button presses into the handle",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < rest_button_pos[0] - 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_guard_aabb = ctx.part_world_aabb(guard)
    with ctx.pose({guard_hinge: 1.0}):
        swung_guard_aabb = ctx.part_world_aabb(guard)
    rest_guard_center_x = None
    swung_guard_center_x = None
    if rest_guard_aabb is not None:
        rest_guard_center_x = 0.5 * (rest_guard_aabb[0][0] + rest_guard_aabb[1][0])
    if swung_guard_aabb is not None:
        swung_guard_center_x = 0.5 * (swung_guard_aabb[0][0] + swung_guard_aabb[1][0])
    ctx.check(
        "blade guard swings on the tip hinge",
        rest_guard_center_x is not None
        and swung_guard_center_x is not None
        and swung_guard_center_x < rest_guard_center_x - 0.030,
        details=f"rest_x={rest_guard_center_x}, swung_x={swung_guard_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
