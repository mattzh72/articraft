from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_box_fan")

    stand_finish = model.material("stand_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.83, 0.85, 0.87, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.68, 0.71, 0.74, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.15, 1.0))

    base_depth = 0.120
    base_width = 0.210
    base_height = 0.014
    arm_depth = 0.028
    arm_width = 0.012
    arm_height = 0.160
    arm_y = 0.148
    pivot_z = 0.172

    housing_depth = 0.105
    housing_width = 0.260
    housing_height = 0.260
    shell_wall = 0.016
    bezel_band = 0.020
    face_depth = 0.014
    grille_bar = 0.004

    stand = model.part("stand")
    stand.visual(
        Box((base_depth, base_width, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=stand_finish,
        name="base_plate",
    )
    stand.visual(
        Box((0.060, 0.140, 0.024)),
        origin=Origin(xyz=(-0.010, 0.0, 0.018)),
        material=stand_finish,
        name="base_riser",
    )
    stand.visual(
        Box((arm_depth, arm_width, arm_height)),
        origin=Origin(xyz=(0.0, arm_y, base_height + arm_height / 2.0)),
        material=stand_finish,
        name="arm_0",
    )
    stand.visual(
        Box((arm_depth, arm_width, arm_height)),
        origin=Origin(xyz=(0.0, -arm_y, base_height + arm_height / 2.0)),
        material=stand_finish,
        name="arm_1",
    )
    stand.visual(
        Box((0.048, 0.304, 0.022)),
        origin=Origin(xyz=(-0.036, 0.0, 0.025)),
        material=stand_finish,
        name="rear_bridge",
    )
    stand.visual(
        Cylinder(radius=0.012, length=arm_width),
        origin=Origin(xyz=(0.0, arm_y - arm_width / 2.0, pivot_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_cap_0",
    )
    stand.visual(
        Cylinder(radius=0.012, length=arm_width),
        origin=Origin(xyz=(0.0, -arm_y + arm_width / 2.0, pivot_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_cap_1",
    )
    for x_pos, y_pos, name in (
        (0.038, 0.070, "foot_0"),
        (0.038, -0.070, "foot_1"),
        (-0.038, 0.070, "foot_2"),
        (-0.038, -0.070, "foot_3"),
    ):
        stand.visual(
            Box((0.022, 0.022, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.002)),
            material=dark_trim,
            name=name,
        )

    housing = model.part("housing")
    housing.visual(
        Box((housing_depth, housing_width, shell_wall)),
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0 - shell_wall / 2.0)),
        material=housing_finish,
        name="top_shell",
    )
    housing.visual(
        Box((housing_depth, housing_width, shell_wall)),
        origin=Origin(xyz=(0.0, 0.0, -housing_height / 2.0 + shell_wall / 2.0)),
        material=housing_finish,
        name="bottom_shell",
    )
    housing.visual(
        Box((housing_depth, shell_wall, housing_height - 2.0 * shell_wall)),
        origin=Origin(xyz=(0.0, housing_width / 2.0 - shell_wall / 2.0, 0.0)),
        material=housing_finish,
        name="side_shell_0",
    )
    housing.visual(
        Box((housing_depth, shell_wall, housing_height - 2.0 * shell_wall)),
        origin=Origin(xyz=(0.0, -housing_width / 2.0 + shell_wall / 2.0, 0.0)),
        material=housing_finish,
        name="side_shell_1",
    )

    for x_pos, prefix in (
        (housing_depth / 2.0 - face_depth / 2.0, "front"),
        (-housing_depth / 2.0 + face_depth / 2.0, "rear"),
    ):
        housing.visual(
            Box((face_depth, housing_width, bezel_band)),
            origin=Origin(xyz=(x_pos, 0.0, housing_height / 2.0 - bezel_band / 2.0)),
            material=grille_finish,
            name=f"{prefix}_bezel_top",
        )
        housing.visual(
            Box((face_depth, housing_width, bezel_band)),
            origin=Origin(xyz=(x_pos, 0.0, -housing_height / 2.0 + bezel_band / 2.0)),
            material=grille_finish,
            name=f"{prefix}_bezel_bottom",
        )
        housing.visual(
            Box((face_depth, bezel_band, housing_height - 2.0 * bezel_band)),
            origin=Origin(xyz=(x_pos, housing_width / 2.0 - bezel_band / 2.0, 0.0)),
            material=grille_finish,
            name=f"{prefix}_bezel_side_0",
        )
        housing.visual(
            Box((face_depth, bezel_band, housing_height - 2.0 * bezel_band)),
            origin=Origin(xyz=(x_pos, -housing_width / 2.0 + bezel_band / 2.0, 0.0)),
            material=grille_finish,
            name=f"{prefix}_bezel_side_1",
        )

    opening_width = housing_width - 2.0 * bezel_band
    opening_height = housing_height - 2.0 * bezel_band
    front_grille_x = housing_depth / 2.0 - face_depth - grille_bar / 2.0
    rear_grille_x = -housing_depth / 2.0 + face_depth + grille_bar / 2.0

    for idx, y_pos in enumerate((-0.080, -0.040, 0.0, 0.040, 0.080)):
        housing.visual(
            Box((grille_bar, grille_bar, opening_height)),
            origin=Origin(xyz=(front_grille_x, y_pos, 0.0)),
            material=dark_trim,
            name=f"front_vertical_{idx}",
        )
        housing.visual(
            Box((grille_bar, grille_bar, opening_height)),
            origin=Origin(xyz=(rear_grille_x, y_pos, 0.0)),
            material=dark_trim,
            name=f"rear_vertical_{idx}",
        )
    for idx, z_pos in enumerate((-0.080, -0.040, 0.0, 0.040, 0.080)):
        housing.visual(
            Box((grille_bar, opening_width, grille_bar)),
            origin=Origin(xyz=(front_grille_x, 0.0, z_pos)),
            material=dark_trim,
            name=f"front_horizontal_{idx}",
        )
        housing.visual(
            Box((grille_bar, opening_width, grille_bar)),
            origin=Origin(xyz=(rear_grille_x, 0.0, z_pos)),
            material=dark_trim,
            name=f"rear_horizontal_{idx}",
        )

    housing.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(-0.034, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="motor_mount",
    )
    housing.visual(
        Cylinder(radius=0.0022, length=0.036),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="shaft",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, housing_width / 2.0 + 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_boss_0",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, -housing_width / 2.0 - 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_boss_1",
    )
    housing.visual(
        Box((0.060, 0.006, 0.086)),
        origin=Origin(xyz=(-0.026, housing_width / 2.0 + 0.003, 0.008)),
        material=grille_finish,
        name="control_plate",
    )
    housing.visual(
        Box((0.024, 0.004, 0.030)),
        origin=Origin(xyz=(-0.026, housing_width / 2.0 + 0.008, -0.024)),
        material=dark_trim,
        name="switch_bezel",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.094,
                0.024,
                5,
                thickness=0.012,
                blade_pitch_deg=26.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.14),
                hub=FanRotorHub(style="spinner", bore_diameter=0.005),
            ),
            "fan_blade",
        ),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="blade_rotor",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.016,
                body_style="skirted",
                top_diameter=0.020,
                skirt=KnobSkirt(0.029, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "fan_speed_knob",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_body",
    )

    switch = model.part("switch")
    switch.visual(
        Box((0.018, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, 0.004, -0.005)),
        material=dark_trim,
        name="switch_cap",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(-0.026, housing_width / 2.0 + 0.006, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    model.articulation(
        "housing_to_switch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=switch,
        origin=Origin(xyz=(-0.026, housing_width / 2.0 + 0.006, -0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=-0.26, upper=0.26),
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("knob")
    switch = object_model.get_part("switch")
    tilt = object_model.get_articulation("stand_to_housing")
    blade_spin = object_model.get_articulation("housing_to_blade")
    knob_spin = object_model.get_articulation("housing_to_knob")
    rocker = object_model.get_articulation("housing_to_switch")

    ctx.expect_origin_gap(
        housing,
        stand,
        axis="z",
        min_gap=0.160,
        max_gap=0.180,
        name="housing pivot sits above the stand base",
    )

    rest_top = ctx.part_element_world_aabb(housing, elem="front_horizontal_2")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_top = ctx.part_element_world_aabb(housing, elem="front_horizontal_2")
    ctx.check(
        "housing tilts upward at the positive limit",
        rest_top is not None
        and raised_top is not None
        and float(raised_top[1][2]) > float(rest_top[1][2]) + 0.020,
        details=f"rest={rest_top}, raised={raised_top}",
    )
    ctx.expect_origin_distance(
        blade,
        housing,
        axes="yz",
        max_dist=0.001,
        name="blade stays centered on the housing axis",
    )
    ctx.expect_origin_gap(
        knob,
        switch,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="speed knob sits above the rocker switch",
    )
    ctx.expect_origin_gap(
        knob,
        housing,
        axis="y",
        min_gap=0.134,
        max_gap=0.138,
        name="speed knob mounts on the control side",
    )
    ctx.check(
        "blade uses a continuous spin joint",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None,
        details=f"type={blade_spin.articulation_type}, limits={blade_spin.motion_limits}",
    )
    ctx.check(
        "knob uses a continuous rotary joint",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=f"type={knob_spin.articulation_type}, limits={knob_spin.motion_limits}",
    )
    ctx.check(
        "rocker switch uses a short revolute travel",
        rocker.articulation_type == ArticulationType.REVOLUTE
        and rocker.motion_limits is not None
        and rocker.motion_limits.lower is not None
        and rocker.motion_limits.upper is not None
        and abs(float(rocker.motion_limits.lower)) <= 0.30
        and abs(float(rocker.motion_limits.upper)) <= 0.30,
        details=f"type={rocker.articulation_type}, limits={rocker.motion_limits}",
    )

    switch_rest = ctx.part_element_world_aabb(switch, elem="switch_cap")
    with ctx.pose({rocker: rocker.motion_limits.upper}):
        switch_rocked = ctx.part_element_world_aabb(switch, elem="switch_cap")
    ctx.check(
        "rocker switch tips outward at one end",
        switch_rest is not None
        and switch_rocked is not None
        and float(switch_rocked[1][1]) > float(switch_rest[1][1]) + 0.0015,
        details=f"rest={switch_rest}, rocked={switch_rocked}",
    )

    return ctx.report()


object_model = build_object_model()
