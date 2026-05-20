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


HOUSING_OUTER = 0.52
HOUSING_DEPTH = 0.158
HOUSING_WALL = 0.028
INNER_SPAN = HOUSING_OUTER - 2.0 * HOUSING_WALL
HOUSING_FRONT_Y = 0.010 + HOUSING_DEPTH
PAN_PIVOT_Y = 0.118


def add_box(part, size, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    material,
    name,
    rpy=(0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mounted_box_fan")

    bracket_gray = model.material("bracket_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    blade_black = model.material("blade_black", rgba=(0.11, 0.12, 0.13, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    shaft_metal = model.material("shaft_metal", rgba=(0.76, 0.78, 0.80, 1.0))

    wall_bracket = model.part("wall_bracket")
    add_box(
        wall_bracket,
        (0.18, 0.014, 0.30),
        (0.0, -0.007, 0.0),
        material=bracket_gray,
        name="wall_plate",
    )
    add_box(
        wall_bracket,
        (0.065, 0.106, 0.18),
        (0.0, 0.053, 0.0),
        material=bracket_gray,
        name="arm_spine",
    )
    add_box(
        wall_bracket,
        (0.14, 0.045, 0.03),
        (0.0, 0.0225, 0.080),
        material=bracket_gray,
        name="brace_top",
    )
    add_box(
        wall_bracket,
        (0.14, 0.045, 0.03),
        (0.0, 0.0225, -0.080),
        material=bracket_gray,
        name="brace_bottom",
    )
    add_cylinder(
        wall_bracket,
        0.055,
        0.012,
        (0.0, 0.112, 0.0),
        material=bracket_gray,
        name="pivot_plate",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )

    housing = model.part("housing")
    add_cylinder(
        housing,
        0.050,
        0.010,
        (0.0, 0.005, 0.0),
        material=housing_dark,
        name="rear_pivot_plate",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    add_box(
        housing,
        (0.050, 0.020, 0.050),
        (0.0, 0.020, 0.0),
        material=housing_dark,
        name="rear_spine",
    )
    add_box(
        housing,
        (HOUSING_WALL, HOUSING_DEPTH, HOUSING_OUTER),
        ((HOUSING_OUTER - HOUSING_WALL) / 2.0, 0.089, 0.0),
        material=housing_dark,
        name="shell_right",
    )
    add_box(
        housing,
        (HOUSING_WALL, HOUSING_DEPTH, HOUSING_OUTER),
        (-(HOUSING_OUTER - HOUSING_WALL) / 2.0, 0.089, 0.0),
        material=housing_dark,
        name="shell_left",
    )
    add_box(
        housing,
        (INNER_SPAN, HOUSING_DEPTH, HOUSING_WALL),
        (0.0, 0.089, (HOUSING_OUTER - HOUSING_WALL) / 2.0),
        material=housing_dark,
        name="shell_top",
    )
    add_box(
        housing,
        (INNER_SPAN, HOUSING_DEPTH, HOUSING_WALL),
        (0.0, 0.089, -(HOUSING_OUTER - HOUSING_WALL) / 2.0),
        material=housing_dark,
        name="shell_bottom",
    )
    add_box(
        housing,
        (INNER_SPAN, 0.014, 0.020),
        (0.0, 0.055, 0.0),
        material=housing_dark,
        name="motor_support_horizontal",
    )
    add_box(
        housing,
        (0.020, 0.014, INNER_SPAN),
        (0.0, 0.055, 0.0),
        material=housing_dark,
        name="motor_support_vertical",
    )
    add_cylinder(
        housing,
        0.060,
        0.050,
        (0.0, 0.055, 0.0),
        material=housing_dark,
        name="motor_cover",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )

    grille_y = HOUSING_FRONT_Y - 0.003
    for index, x_pos in enumerate((-0.155, -0.078, 0.0, 0.078, 0.155)):
        add_box(
            housing,
            (0.010, 0.006, INNER_SPAN),
            (x_pos, grille_y, 0.0),
            material=housing_dark,
            name=f"grille_v_{index - 2}",
        )
    for index, z_pos in enumerate((-0.155, -0.078, 0.0, 0.078, 0.155)):
        add_box(
            housing,
            (INNER_SPAN, 0.006, 0.010),
            (0.0, grille_y, z_pos),
            material=housing_dark,
            name=f"grille_h_{index - 2}",
        )

    add_box(
        housing,
        (0.084, 0.018, 0.086),
        (0.218, HOUSING_FRONT_Y + 0.009, -0.182),
        material=housing_dark,
        name="front_pod",
    )
    add_box(
        housing,
        (0.020, 0.074, 0.072),
        (0.270, 0.058, 0.118),
        material=housing_dark,
        name="side_pod",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.165,
                0.038,
                5,
                thickness=0.018,
                blade_pitch_deg=28.0,
                blade_sweep_deg=22.0,
                blade=FanRotorBlade(
                    shape="broad",
                    tip_pitch_deg=12.0,
                    camber=0.12,
                ),
                hub=FanRotorHub(style="domed"),
            ),
            "box_fan_blade",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=blade_black,
        name="rotor",
    )
    add_cylinder(
        blade,
        0.020,
        0.016,
        (0.0, 0.008, 0.0),
        material=shaft_metal,
        name="hub_cap",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    add_cylinder(
        blade,
        0.007,
        0.016,
        (0.0, -0.008, 0.0),
        material=shaft_metal,
        name="drive_shaft",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )

    speed_knob = model.part("speed_knob")
    add_cylinder(
        speed_knob,
        0.005,
        0.010,
        (0.0, 0.005, 0.0),
        material=shaft_metal,
        name="speed_shaft",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.018,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.054, 0.005, flare=0.08),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0008,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="speed_cap",
    )

    oscillation_knob = model.part("oscillation_knob")
    add_cylinder(
        oscillation_knob,
        0.004,
        0.010,
        (0.005, 0.0, 0.0),
        material=shaft_metal,
        name="oscillation_shaft",
        rpy=(0.0, pi / 2.0, 0.0),
    )
    oscillation_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.015,
                body_style="cylindrical",
                grip=KnobGrip(
                    style="knurled",
                    count=24,
                    depth=0.0007,
                    helix_angle_deg=18.0,
                ),
                center=False,
            ),
            "oscillation_knob",
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_dark,
        name="oscillation_cap",
    )

    model.articulation(
        "wall_bracket_to_housing",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=housing,
        origin=Origin(xyz=(0.0, PAN_PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.096, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=35.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.218, HOUSING_FRONT_Y + 0.018, -0.182)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "housing_to_oscillation_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=oscillation_knob,
        origin=Origin(xyz=(0.280, 0.058, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    speed_knob = object_model.get_part("speed_knob")
    oscillation_knob = object_model.get_part("oscillation_knob")

    pan = object_model.get_articulation("wall_bracket_to_housing")
    blade_spin = object_model.get_articulation("housing_to_blade")
    speed_spin = object_model.get_articulation("housing_to_speed_knob")
    oscillation_spin = object_model.get_articulation("housing_to_oscillation_knob")

    ctx.check(
        "pan_joint_has_wall_fan_sweep",
        pan.motion_limits is not None
        and pan.motion_limits.lower is not None
        and pan.motion_limits.upper is not None
        and pan.motion_limits.lower <= -0.6
        and pan.motion_limits.upper >= 0.6,
        details=f"limits={pan.motion_limits!r}",
    )
    ctx.check(
        "rotor_and_knobs_are_continuous",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_spin.articulation_type == ArticulationType.CONTINUOUS
        and oscillation_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"blade={blade_spin.articulation_type!r}, "
            f"speed={speed_spin.articulation_type!r}, "
            f"oscillation={oscillation_spin.articulation_type!r}"
        ),
    )

    ctx.expect_contact(
        housing,
        wall_bracket,
        elem_a="rear_pivot_plate",
        elem_b="pivot_plate",
        name="housing seats on bracket pivot plate",
    )
    ctx.expect_contact(
        speed_knob,
        housing,
        elem_a="speed_shaft",
        elem_b="front_pod",
        name="front speed knob is mounted on a control pod",
    )
    ctx.expect_contact(
        oscillation_knob,
        housing,
        elem_a="oscillation_shaft",
        elem_b="side_pod",
        name="side oscillation knob is mounted on a side pod",
    )
    ctx.expect_contact(
        blade,
        housing,
        elem_a="drive_shaft",
        elem_b="motor_cover",
        name="blade shaft is supported by the motor cover",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="y",
        positive_elem="rotor",
        negative_elem="motor_cover",
        min_gap=0.004,
        name="rotor clears the rear motor cover",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="y",
        positive_elem="grille_v_0",
        negative_elem="rotor",
        min_gap=0.040,
        name="front grille stays ahead of the spinning blade",
    )
    ctx.expect_overlap(
        blade,
        housing,
        axes="xz",
        elem_a="rotor",
        min_overlap=0.30,
        name="blade stays centered in the housing opening footprint",
    )

    rest_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({pan: 0.70}):
        panned_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "housing_pan_moves_the_front_controls",
        rest_pos is not None
        and panned_pos is not None
        and abs(panned_pos[0] - rest_pos[0]) >= 0.15,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
