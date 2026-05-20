from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.120
BASE_DISK_HEIGHT = 0.020
BASE_CAP_RADIUS = 0.082
BASE_CAP_HEIGHT = 0.015
TURNTABLE_RADIUS = 0.052
TURNTABLE_HEIGHT = 0.010
BASE_HEIGHT = BASE_DISK_HEIGHT + BASE_CAP_HEIGHT + TURNTABLE_HEIGHT

TOWER_WIDTH = 0.108
TOWER_DEPTH = 0.092
TOWER_HEIGHT = 0.410
TOWER_CAVITY_WIDTH = 0.080
TOWER_CAVITY_DEPTH = 0.064
TOWER_CAVITY_BOTTOM = 0.030
TOWER_CAVITY_HEIGHT = 0.356
CONTROL_POCKET_WIDTH = 0.074
CONTROL_POCKET_DEPTH = 0.048
CONTROL_POCKET_RECESS = 0.012
CONTROL_FLOOR_Z = TOWER_HEIGHT - CONTROL_POCKET_RECESS - 0.0005

ROTOR_BOTTOM_Z = 0.055
ROTOR_RADIUS = 0.029
ROTOR_INNER_RADIUS = 0.012
ROTOR_HEIGHT = 0.285

SPEED_KNOB_POS = (-0.010, 0.010, CONTROL_FLOOR_Z)
TIMER_KNOB_POS = (-0.010, -0.010, CONTROL_FLOOR_Z)
BUTTON_POS = (0.018, 0.000, CONTROL_FLOOR_Z)


def _tower_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(TOWER_WIDTH, TOWER_DEPTH, TOWER_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.006)
    )

    shell = shell.cut(
        cq.Workplane("XY")
        .box(
            TOWER_CAVITY_WIDTH,
            TOWER_CAVITY_DEPTH,
            TOWER_CAVITY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TOWER_CAVITY_BOTTOM))
    )

    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.070, 0.026, 0.282)
        .translate((0.0, (TOWER_DEPTH * 0.5) - 0.013, 0.205))
    )

    for slot_x in (-0.024, -0.012, 0.0, 0.012, 0.024):
        shell = shell.cut(
            cq.Workplane("XY")
            .box(0.007, 0.022, 0.226)
            .translate((slot_x, -(TOWER_DEPTH * 0.5) + 0.011, 0.205))
        )

    shell = shell.cut(
        cq.Workplane("XY")
        .box(
            CONTROL_POCKET_WIDTH,
            CONTROL_POCKET_DEPTH,
            CONTROL_POCKET_RECESS + 0.001,
        )
        .translate((0.0, 0.0, TOWER_HEIGHT - (CONTROL_POCKET_RECESS * 0.5)))
    )

    for knob_x, knob_y, _ in (SPEED_KNOB_POS, TIMER_KNOB_POS):
        shell = shell.cut(
            cq.Workplane("XY")
            .circle(0.0052)
            .extrude(0.030)
            .translate((knob_x, knob_y, CONTROL_FLOOR_Z - 0.022))
        )

    shell = shell.cut(
        cq.Workplane("XY")
        .circle(0.0042)
        .extrude(0.030)
        .translate((BUTTON_POS[0], BUTTON_POS[1], CONTROL_FLOOR_Z - 0.022))
    )

    return shell


def _knob_mesh(mesh_name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.017,
            0.015,
            body_style="skirted",
            top_diameter=0.013,
            skirt=KnobSkirt(0.019, 0.0035, flare=0.06),
            grip=KnobGrip(style="fluted", count=14, depth=0.0007),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0005,
                angle_deg=0.0,
            ),
            center=False,
        ),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_tower_fan")

    base_finish = model.material("base_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.86, 0.87, 0.84, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.32, 0.34, 0.36, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.57, 0.60, 0.63, 1.0))
    control_finish = model.material("control_finish", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_DISK_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISK_HEIGHT * 0.5)),
        material=base_finish,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=BASE_CAP_RADIUS, length=BASE_CAP_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_DISK_HEIGHT + (BASE_CAP_HEIGHT * 0.5)),
        ),
        material=base_finish,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_DISK_HEIGHT + BASE_CAP_HEIGHT + (TURNTABLE_HEIGHT * 0.5)),
        ),
        material=base_finish,
        name="turntable",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_tower_shell_shape(), "tower_shell"),
        material=housing_finish,
        name="tower_shell",
    )
    tower.inertial = Inertial.from_geometry(
        Box((TOWER_WIDTH, TOWER_DEPTH, TOWER_HEIGHT)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT * 0.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                ROTOR_RADIUS,
                ROTOR_INNER_RADIUS,
                ROTOR_HEIGHT,
                20,
                blade_thickness=0.0026,
                blade_sweep_deg=24.0,
                backplate=True,
                shroud=True,
                center=False,
            ),
            "tower_fan_blower_wheel",
        ),
        material=rotor_finish,
        name="blower_wheel",
    )
    rotor.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=shaft_finish,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.004, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=shaft_finish,
        name="lower_shaft",
    )
    rotor.inertial = Inertial.from_geometry(
        Box((ROTOR_RADIUS * 2.0, ROTOR_RADIUS * 2.0, ROTOR_HEIGHT + 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, (ROTOR_HEIGHT + 0.020) * 0.5)),
    )

    knob_mesh = _knob_mesh("tower_fan_knob")

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        knob_mesh,
        material=control_finish,
        name="knob_shell",
    )
    speed_knob.visual(
        Cylinder(radius=0.0068, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=control_finish,
        name="base_flange",
    )
    speed_knob.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=shaft_finish,
        name="shaft",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.025)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        knob_mesh,
        material=control_finish,
        name="knob_shell",
    )
    timer_knob.visual(
        Cylinder(radius=0.0068, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=control_finish,
        name="base_flange",
    )
    timer_knob.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=shaft_finish,
        name="shaft",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.025)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Cylinder(radius=0.0055, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=control_finish,
        name="button_cap",
    )
    oscillation_button.visual(
        Cylinder(radius=0.0048, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=control_finish,
        name="button_flange",
    )
    oscillation_button.visual(
        Cylinder(radius=0.0038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=shaft_finish,
        name="button_stem",
    )
    oscillation_button.inertial = Inertial.from_geometry(
        Box((0.012, 0.012, 0.018)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.90,
            upper=0.90,
        ),
    )
    model.articulation(
        "tower_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, ROTOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=42.0),
    )
    model.articulation(
        "tower_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_knob,
        origin=Origin(xyz=SPEED_KNOB_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=6.0),
    )
    model.articulation(
        "tower_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=timer_knob,
        origin=Origin(xyz=TIMER_KNOB_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=6.0),
    )
    model.articulation(
        "tower_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=oscillation_button,
        origin=Origin(xyz=BUTTON_POS),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.10,
            lower=0.0,
            upper=0.0032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    rotor = object_model.get_part("rotor")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")
    oscillation_button = object_model.get_part("oscillation_button")

    oscillation = object_model.get_articulation("base_to_tower")
    rotor_spin = object_model.get_articulation("tower_to_rotor")
    speed_spin = object_model.get_articulation("tower_to_speed_knob")
    timer_spin = object_model.get_articulation("tower_to_timer_knob")
    button_press = object_model.get_articulation("tower_to_oscillation_button")

    ctx.allow_overlap(
        speed_knob,
        tower,
        elem_a="knob_shell",
        elem_b="tower_shell",
        reason="The speed knob is intentionally nested into the recessed top control pocket.",
    )
    ctx.allow_overlap(
        timer_knob,
        tower,
        elem_a="knob_shell",
        elem_b="tower_shell",
        reason="The timer knob is intentionally nested into the recessed top control pocket.",
    )
    ctx.allow_overlap(
        oscillation_button,
        tower,
        elem_a="button_cap",
        elem_b="tower_shell",
        reason="The oscillation button sits partly recessed inside the top deck button well.",
    )
    ctx.allow_overlap(
        rotor,
        tower,
        elem_a="lower_shaft",
        elem_b="tower_shell",
        reason="The blower rotor is carried by a captured lower bearing pin inside the tower body.",
    )

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        positive_elem="tower_shell",
        negative_elem="turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="tower rests on turntable",
    )
    ctx.expect_overlap(
        tower,
        base,
        axes="xy",
        elem_a="tower_shell",
        elem_b="turntable",
        min_overlap=0.075,
        name="tower remains centered over base",
    )
    ctx.expect_within(
        rotor,
        tower,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="tower_shell",
        margin=0.010,
        name="blower wheel stays within tower footprint",
    )
    ctx.expect_overlap(
        rotor,
        tower,
        axes="z",
        elem_a="blower_wheel",
        elem_b="tower_shell",
        min_overlap=0.260,
        name="blower wheel spans the tower outlet region",
    )

    ctx.check(
        "tower joint is oscillation revolute",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower is not None
        and oscillation.motion_limits.upper is not None
        and oscillation.motion_limits.lower < 0.0 < oscillation.motion_limits.upper,
        details=(
            f"type={oscillation.articulation_type!r}, "
            f"limits={oscillation.motion_limits!r}"
        ),
    )

    for name, joint in (
        ("rotor", rotor_spin),
        ("speed knob", speed_spin),
        ("timer knob", timer_spin),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{name} joint is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type!r}, limits={limits!r}",
        )

    ctx.check(
        "oscillation button uses prismatic press travel",
        button_press.articulation_type == ArticulationType.PRISMATIC
        and button_press.motion_limits is not None
        and button_press.motion_limits.lower == 0.0
        and button_press.motion_limits.upper is not None
        and 0.002 <= button_press.motion_limits.upper <= 0.005,
        details=(
            f"type={button_press.articulation_type!r}, "
            f"limits={button_press.motion_limits!r}"
        ),
    )

    rest_button_pos = ctx.part_world_position(oscillation_button)
    button_upper = (
        button_press.motion_limits.upper
        if button_press.motion_limits is not None and button_press.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({button_press: button_upper}):
        pressed_button_pos = ctx.part_world_position(oscillation_button)
    ctx.check(
        "oscillation button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.002,
        details=f"rest={rest_button_pos!r}, pressed={pressed_button_pos!r}",
    )

    rest_knob_pos = ctx.part_world_position(speed_knob)
    osc_upper = (
        oscillation.motion_limits.upper
        if oscillation.motion_limits is not None and oscillation.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({oscillation: osc_upper}):
        turned_knob_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "tower oscillation moves top controls around base axis",
        rest_knob_pos is not None
        and turned_knob_pos is not None
        and math.hypot(
            turned_knob_pos[0] - rest_knob_pos[0],
            turned_knob_pos[1] - rest_knob_pos[1],
        )
        > 0.012,
        details=f"rest={rest_knob_pos!r}, turned={turned_knob_pos!r}",
    )

    ctx.check(
        "independent top controls are present",
        all(part is not None for part in (speed_knob, timer_knob, oscillation_button)),
        details=(
            f"speed_knob={speed_knob!r}, "
            f"timer_knob={timer_knob!r}, "
            f"oscillation_button={oscillation_button!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
