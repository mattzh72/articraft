from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_education_laptop")

    blue = model.material("rugged_blue", rgba=(0.08, 0.19, 0.38, 1.0))
    blue_edge = model.material("blue_edge", rgba=(0.04, 0.11, 0.24, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.015, 0.017, 0.020, 1.0))
    dark = model.material("dark_gray", rgba=(0.075, 0.080, 0.086, 1.0))
    key_mat = model.material("matte_keys", rgba=(0.025, 0.028, 0.032, 1.0))
    screen_mat = model.material("soft_black_screen", rgba=(0.005, 0.012, 0.017, 1.0))
    rubber = model.material("rubber_black", rgba=(0.010, 0.010, 0.011, 1.0))
    accent = model.material("power_orange", rgba=(1.0, 0.39, 0.08, 1.0))

    base_w = 0.320
    base_d = 0.220
    base_h = 0.024
    hinge_x = 0.105
    hinge_z = 0.044

    chassis = model.part("lower_chassis")
    lower_shell = (
        cq.Workplane("XY")
        .box(base_d, base_w, base_h)
        .edges("|Z")
        .fillet(0.006)
    )
    chassis.visual(
        mesh_from_cadquery(lower_shell, "lower_shell", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
        material=blue,
        name="lower_shell",
    )
    chassis.visual(
        Box((0.122, 0.270, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.025)),
        material=dark,
        name="keyboard_well",
    )
    chassis.visual(
        Box((0.052, 0.092, 0.0015)),
        origin=Origin(xyz=(-0.076, 0.0, 0.0267)),
        material=charcoal,
        name="touchpad",
    )
    chassis.visual(
        Box((0.060, 0.010, 0.0012)),
        origin=Origin(xyz=(-0.108, 0.0, 0.012)),
        material=blue_edge,
        name="front_grip",
    )

    # The side wall carries a shallow guide slot and raised lips that visibly
    # retain the sliding power switch.
    slot_center = (-0.030, base_w / 2.0 + 0.0010, 0.014)
    chassis.visual(
        Box((0.078, 0.002, 0.016)),
        origin=Origin(xyz=slot_center),
        material=charcoal,
        name="side_slot_recess",
    )
    chassis.visual(
        Box((0.084, 0.004, 0.003)),
        origin=Origin(xyz=(slot_center[0], base_w / 2.0 + 0.002, slot_center[2] + 0.0095)),
        material=blue_edge,
        name="upper_slot_lip",
    )
    chassis.visual(
        Box((0.084, 0.004, 0.003)),
        origin=Origin(xyz=(slot_center[0], base_w / 2.0 + 0.002, slot_center[2] - 0.0095)),
        material=blue_edge,
        name="lower_slot_lip",
    )

    # Two hinge clevises are fixed to the rear deck.  The moving lid carries the
    # center knuckle in each clevis.
    hinge_centers = (-0.105, 0.105)
    hinge_0_inner_y = hinge_centers[0] - 0.017
    hinge_0_outer_y = hinge_centers[0] + 0.017
    hinge_1_inner_y = hinge_centers[1] - 0.017
    hinge_1_outer_y = hinge_centers[1] + 0.017
    chassis.visual(
        Box((0.025, 0.010, 0.021)),
        origin=Origin(xyz=(hinge_x, hinge_0_inner_y, 0.0345)),
        material=blue_edge,
        name="hinge_0_inner_stand",
    )
    chassis.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(hinge_x, hinge_0_inner_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_0_inner_barrel",
    )
    chassis.visual(
        Box((0.025, 0.010, 0.021)),
        origin=Origin(xyz=(hinge_x, hinge_0_outer_y, 0.0345)),
        material=blue_edge,
        name="hinge_0_outer_stand",
    )
    chassis.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(hinge_x, hinge_0_outer_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_0_outer_barrel",
    )
    chassis.visual(
        Box((0.025, 0.010, 0.021)),
        origin=Origin(xyz=(hinge_x, hinge_1_inner_y, 0.0345)),
        material=blue_edge,
        name="hinge_1_inner_stand",
    )
    chassis.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(hinge_x, hinge_1_inner_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_1_inner_barrel",
    )
    chassis.visual(
        Box((0.025, 0.010, 0.021)),
        origin=Origin(xyz=(hinge_x, hinge_1_outer_y, 0.0345)),
        material=blue_edge,
        name="hinge_1_outer_stand",
    )
    chassis.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(hinge_x, hinge_1_outer_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_1_outer_barrel",
    )

    # Individual prismatic key parts.  A compact school laptop reads better with
    # slightly oversized, separated keycaps rather than a dense ultrabook grid.
    key_z = 0.026
    key_x = 0.0145
    key_y = 0.019
    key_h = 0.0045
    key_step_y = 0.025
    regular_rows = [
        (0.052, 10, 0.0),
        (0.029, 10, 0.003),
        (0.006, 9, 0.0),
    ]
    for row, (x, count, offset) in enumerate(regular_rows):
        start_y = -0.5 * key_step_y * (count - 1) + offset
        for col in range(count):
            y = start_y + col * key_step_y
            key = model.part(f"key_{row}_{col}")
            key.visual(
                Box((key_x, key_y, key_h)),
                origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
                material=key_mat,
                name="keycap",
            )
            model.articulation(
                f"chassis_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key,
                origin=Origin(xyz=(x, y, key_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.08),
            )

    spacebar = model.part("spacebar")
    spacebar.visual(
        Box((0.0145, 0.115, key_h)),
        origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
        material=key_mat,
        name="keycap",
    )
    model.articulation(
        "chassis_to_spacebar",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=spacebar,
        origin=Origin(xyz=(-0.020, 0.0, key_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=3.0, velocity=0.08),
    )
    for col, y in enumerate((-0.112, -0.086, 0.086, 0.112)):
        key = model.part(f"key_3_{col}")
        key.visual(
            Box((key_x, key_y, key_h)),
            origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
            material=key_mat,
            name="keycap",
        )
        model.articulation(
            f"chassis_to_key_3_{col}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=key,
            origin=Origin(xyz=(-0.020, y, key_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.0025, effort=2.0, velocity=0.08),
        )

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.032, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rubber,
        name="slider_runner",
    )
    power_slider.visual(
        Box((0.012, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0035, 0.0)),
        material=rubber,
        name="slider_neck",
    )
    power_slider.visual(
        Box((0.018, 0.007, 0.010)),
        origin=Origin(xyz=(0.0, 0.0085, 0.0)),
        material=accent,
        name="thumb_tab",
    )
    model.articulation(
        "chassis_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=power_slider,
        origin=Origin(xyz=(-0.048, base_w / 2.0 + 0.0022, slot_center[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.026, effort=4.0, velocity=0.12),
    )

    lid = model.part("display_lid")
    lid_tilt = math.radians(20.0)
    lid_gap = 0.014
    lid_h = 0.190
    lid_w = 0.318
    lid_t = 0.014

    def lid_xyz(local_x: float, local_z: float) -> tuple[float, float, float]:
        return (
            math.cos(lid_tilt) * local_x + math.sin(lid_tilt) * local_z,
            0.0,
            -math.sin(lid_tilt) * local_x + math.cos(lid_tilt) * local_z,
        )

    lid_shell = (
        cq.Workplane("XY")
        .box(lid_t, lid_w, lid_h)
        .edges("|Z")
        .fillet(0.003)
    )
    lid.visual(
        mesh_from_cadquery(lid_shell, "lid_shell", tolerance=0.0008),
        origin=Origin(xyz=lid_xyz(0.0, lid_gap + lid_h / 2.0), rpy=(0.0, lid_tilt, 0.0)),
        material=blue,
        name="lid_shell",
    )
    screen_center_z = lid_gap + 0.095
    screen_w = 0.242
    screen_h = 0.126
    lid.visual(
        Box((0.0018, screen_w, screen_h)),
        origin=Origin(xyz=lid_xyz(-0.0075, screen_center_z), rpy=(0.0, lid_tilt, 0.0)),
        material=screen_mat,
        name="screen",
    )
    bezel_x = -0.0082
    bezel_center = lid_xyz(bezel_x, screen_center_z)
    lid.visual(
        Box((0.0025, 0.026, 0.164)),
        origin=Origin(
            xyz=(bezel_center[0], -(screen_w / 2.0 + 0.016), bezel_center[2]),
            rpy=(0.0, lid_tilt, 0.0),
        ),
        material=charcoal,
        name="side_bezel_0",
    )
    lid.visual(
        Box((0.0025, 0.026, 0.164)),
        origin=Origin(
            xyz=(bezel_center[0], screen_w / 2.0 + 0.016, bezel_center[2]),
            rpy=(0.0, lid_tilt, 0.0),
        ),
        material=charcoal,
        name="side_bezel_1",
    )
    lid.visual(
        Box((0.0025, 0.294, 0.027)),
        origin=Origin(
            xyz=lid_xyz(bezel_x, screen_center_z + screen_h / 2.0 + 0.016),
            rpy=(0.0, lid_tilt, 0.0),
        ),
        material=charcoal,
        name="top_bezel",
    )
    lid.visual(
        Box((0.0025, 0.294, 0.029)),
        origin=Origin(
            xyz=lid_xyz(bezel_x, screen_center_z - screen_h / 2.0 - 0.017),
            rpy=(0.0, lid_tilt, 0.0),
        ),
        material=charcoal,
        name="bottom_bezel",
    )
    lid.visual(
        Box((0.0028, 0.009, 0.005)),
        origin=Origin(
            xyz=lid_xyz(bezel_x - 0.001, screen_center_z + screen_h / 2.0 + 0.016),
            rpy=(0.0, lid_tilt, 0.0),
        ),
        material=rubber,
        name="camera_dot",
    )

    leaf_center = lid_xyz(-0.001, 0.010)
    lid.visual(
        Cylinder(radius=0.0054, length=0.024),
        origin=Origin(xyz=(0.0, hinge_centers[0], 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="lid_hinge_0_barrel",
    )
    lid.visual(
        Box((0.006, 0.018, 0.026)),
        origin=Origin(xyz=(leaf_center[0], hinge_centers[0], leaf_center[2]), rpy=(0.0, lid_tilt, 0.0)),
        material=dark,
        name="lid_hinge_0_leaf",
    )
    lid.visual(
        Cylinder(radius=0.0054, length=0.024),
        origin=Origin(xyz=(0.0, hinge_centers[1], 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="lid_hinge_1_barrel",
    )
    lid.visual(
        Box((0.006, 0.018, 0.026)),
        origin=Origin(xyz=(leaf_center[0], hinge_centers[1], leaf_center[2]), rpy=(0.0, lid_tilt, 0.0)),
        material=dark,
        name="lid_hinge_1_leaf",
    )

    model.articulation(
        "chassis_to_display_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.92, upper=0.30, effort=12.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("lower_chassis")
    lid = object_model.get_part("display_lid")
    slider = object_model.get_part("power_slider")
    key = object_model.get_part("key_0_0")
    spacebar = object_model.get_part("spacebar")
    lid_joint = object_model.get_articulation("chassis_to_display_lid")
    slider_joint = object_model.get_articulation("chassis_to_power_slider")
    key_joint = object_model.get_articulation("chassis_to_key_0_0")
    spacebar_joint = object_model.get_articulation("chassis_to_spacebar")

    ctx.allow_overlap(
        chassis,
        slider,
        elem_a="side_slot_recess",
        elem_b="slider_runner",
        reason=(
            "The runner is intentionally captured by a shallow side-wall guide "
            "slot proxy so the switch translates without separating."
        ),
    )
    ctx.expect_within(
        slider,
        chassis,
        axes="xz",
        inner_elem="slider_runner",
        outer_elem="side_slot_recess",
        margin=0.0005,
        name="slider runner stays inside slot at rest",
    )
    ctx.expect_overlap(
        slider,
        chassis,
        axes="x",
        elem_a="slider_runner",
        elem_b="side_slot_recess",
        min_overlap=0.030,
        name="slider remains lengthwise captured at rest",
    )
    ctx.expect_gap(
        slider,
        chassis,
        axis="y",
        positive_elem="slider_runner",
        negative_elem="side_slot_recess",
        max_penetration=0.0015,
        name="slider capture overlap is shallow",
    )

    with ctx.pose({slider_joint: 0.026}):
        ctx.expect_within(
            slider,
            chassis,
            axes="xz",
            inner_elem="slider_runner",
            outer_elem="side_slot_recess",
            margin=0.0005,
            name="slider runner stays inside slot at full travel",
        )
        ctx.expect_overlap(
            slider,
            chassis,
            axes="x",
            elem_a="slider_runner",
            elem_b="side_slot_recess",
            min_overlap=0.030,
            name="slider remains lengthwise captured at full travel",
        )

    rest_slider = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: 0.026}):
        moved_slider = ctx.part_world_position(slider)
    ctx.check(
        "power slider translates along side slot",
        rest_slider is not None
        and moved_slider is not None
        and moved_slider[0] > rest_slider[0] + 0.020
        and abs(moved_slider[1] - rest_slider[1]) < 1e-6
        and abs(moved_slider[2] - rest_slider[2]) < 1e-6,
        details=f"rest={rest_slider}, moved={moved_slider}",
    )

    ctx.expect_gap(
        key,
        chassis,
        axis="z",
        positive_elem="keycap",
        negative_elem="keyboard_well",
        max_penetration=0.0001,
        max_gap=0.0005,
        name="sample key rests on keyboard well",
    )
    rest_key = ctx.part_world_position(key)
    with ctx.pose({key_joint: 0.0025}):
        pressed_key = ctx.part_world_position(key)
    ctx.check(
        "sample key plunges downward",
        rest_key is not None
        and pressed_key is not None
        and pressed_key[2] < rest_key[2] - 0.002
        and abs(pressed_key[0] - rest_key[0]) < 1e-6
        and abs(pressed_key[1] - rest_key[1]) < 1e-6,
        details=f"rest={rest_key}, pressed={pressed_key}",
    )
    rest_space = ctx.part_world_position(spacebar)
    with ctx.pose({spacebar_joint: 0.0025}):
        pressed_space = ctx.part_world_position(spacebar)
    ctx.check(
        "spacebar uses the same short plunger travel",
        rest_space is not None
        and pressed_space is not None
        and pressed_space[2] < rest_space[2] - 0.002,
        details=f"rest={rest_space}, pressed={pressed_space}",
    )

    ctx.expect_gap(
        lid,
        chassis,
        axis="y",
        positive_elem="lid_hinge_0_barrel",
        negative_elem="hinge_0_inner_barrel",
        max_gap=0.0001,
        max_penetration=0.0001,
        name="first hinge center knuckle touches inner clevis ear",
    )
    ctx.expect_gap(
        chassis,
        lid,
        axis="y",
        positive_elem="hinge_0_outer_barrel",
        negative_elem="lid_hinge_0_barrel",
        max_gap=0.0001,
        max_penetration=0.0001,
        name="first hinge center knuckle touches outer clevis ear",
    )
    ctx.expect_overlap(
        lid,
        chassis,
        axes="y",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.28,
        name="display lid spans the chassis width",
    )

    open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: -1.55}):
        lowered_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "display lid rotates down toward the keyboard",
        open_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[0][0] < open_aabb[0][0] - 0.050
        and lowered_aabb[1][2] < open_aabb[1][2] - 0.050,
        details=f"open={open_aabb}, lowered={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
