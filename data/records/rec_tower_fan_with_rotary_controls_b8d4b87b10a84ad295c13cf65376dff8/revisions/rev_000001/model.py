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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_RADIUS = 0.086
BASE_HEIGHT = 0.022
TURNTABLE_RADIUS = 0.044
TURNTABLE_HEIGHT = 0.010
OSCILLATION_Z = BASE_HEIGHT + TURNTABLE_HEIGHT

TOWER_COLLAR_RADIUS = 0.034
TOWER_COLLAR_HEIGHT = 0.010
TOWER_BODY_DEPTH = 0.086
TOWER_BODY_WIDTH = 0.112
TOWER_BODY_HEIGHT = 0.320
TOP_CAP_DEPTH = 0.094
TOP_CAP_WIDTH = 0.120
TOP_CAP_HEIGHT = 0.040
TOP_CAP_OVERLAP = 0.010
TOP_CAP_Z0 = TOWER_COLLAR_HEIGHT + TOWER_BODY_HEIGHT - TOP_CAP_OVERLAP
TOWER_TOTAL_HEIGHT = TOP_CAP_Z0 + TOP_CAP_HEIGHT
BODY_WALL = 0.003

OUTLET_POCKET_DEPTH = 0.008
OUTLET_POCKET_WIDTH = 0.084
OUTLET_POCKET_HEIGHT = 0.274
OUTLET_POCKET_Z0 = 0.034
OUTLET_OPENING_DEPTH = 0.024
OUTLET_OPENING_WIDTH = 0.068
OUTLET_OPENING_HEIGHT = 0.248
OUTLET_OPENING_Z0 = 0.042
OUTLET_CENTER_Z = OUTLET_OPENING_Z0 + OUTLET_OPENING_HEIGHT / 2.0

GRILLE_WIDTH = 0.084
GRILLE_HEIGHT = 0.272
GRILLE_REAR_X = 0.033
GRILLE_RPY = (math.pi / 2.0, 0.0, math.pi / 2.0)

ROTOR_AXIS_X = 0.006
ROTOR_OUTER_RADIUS = 0.026
ROTOR_INNER_RADIUS = 0.016
ROTOR_HEIGHT = 0.236

CONTROL_RECESS_DEPTH = 0.011
CONTROL_RECESS_X = 0.010
CONTROL_RECESS_LENGTH = 0.046
CONTROL_RECESS_WIDTH = 0.078
CONTROL_FLOOR_Z = TOWER_TOTAL_HEIGHT - CONTROL_RECESS_DEPTH
AXLE_RADIUS = ROTOR_INNER_RADIUS
AXLE_LENGTH = ROTOR_HEIGHT + 0.016


def _build_tower_shell() -> object:
    body = (
        cq.Workplane("XY")
        .box(TOWER_BODY_DEPTH, TOWER_BODY_WIDTH, TOWER_BODY_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, TOWER_COLLAR_HEIGHT))
        .edges("|Z")
        .fillet(0.011)
    )
    cap = (
        cq.Workplane("XY")
        .box(TOP_CAP_DEPTH, TOP_CAP_WIDTH, TOP_CAP_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, TOP_CAP_Z0))
        .edges("|Z")
        .fillet(0.012)
    )
    shell = body.union(cap).faces("<Z").shell(-BODY_WALL)

    outlet_pocket = (
        cq.Workplane("XY")
        .box(
            OUTLET_POCKET_DEPTH,
            OUTLET_POCKET_WIDTH,
            OUTLET_POCKET_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                TOWER_BODY_DEPTH / 2.0 - OUTLET_POCKET_DEPTH / 2.0,
                0.0,
                OUTLET_POCKET_Z0,
            )
        )
    )
    outlet_opening = (
        cq.Workplane("XY")
        .box(
            OUTLET_OPENING_DEPTH,
            OUTLET_OPENING_WIDTH,
            OUTLET_OPENING_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                TOWER_BODY_DEPTH / 2.0 - OUTLET_OPENING_DEPTH / 2.0 + 0.002,
                0.0,
                OUTLET_OPENING_Z0,
            )
        )
    )
    control_recess = (
        cq.Workplane("XY")
        .box(
            CONTROL_RECESS_LENGTH,
            CONTROL_RECESS_WIDTH,
            CONTROL_RECESS_DEPTH,
            centered=(True, True, False),
        )
        .translate(
            (
                CONTROL_RECESS_X,
                0.0,
                TOWER_TOTAL_HEIGHT - CONTROL_RECESS_DEPTH,
            )
        )
    )

    shell = shell.cut(outlet_pocket).cut(outlet_opening)
    shell = (
        shell.faces(">Z")
        .workplane()
        .center(-0.020, 0.0)
        .slot2D(0.050, 0.018, 90)
        .cutBlind(-0.024)
    )
    shell = shell.cut(control_recess)

    collar = cq.Workplane("XY").circle(TOWER_COLLAR_RADIUS).extrude(TOWER_COLLAR_HEIGHT)
    transition = (
        cq.Workplane("XY")
        .box(0.084, 0.108, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, TOWER_COLLAR_HEIGHT - 0.002))
        .edges("|Z")
        .fillet(0.004)
    )

    return shell.union(collar).union(transition)


def _is_vertical_axis(axis: tuple[float, float, float] | None) -> bool:
    if axis is None:
        return False
    return all(abs(value - target) < 1e-6 for value, target in zip(axis, (0.0, 0.0, 1.0)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_tower_fan")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.89, 1.0))
    grille_grey = model.material("grille_grey", rgba=(0.52, 0.55, 0.58, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.14, 0.15, 0.16, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    tower_mesh = mesh_from_cadquery(_build_tower_shell(), "tower_fan_tower")
    blower_mesh = mesh_from_geometry(
        BlowerWheelGeometry(
            ROTOR_OUTER_RADIUS,
            ROTOR_INNER_RADIUS,
            ROTOR_HEIGHT,
            24,
            blade_thickness=0.0025,
            blade_sweep_deg=24.0,
            center=True,
        ),
        "tower_fan_blower",
    )
    grille_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (GRILLE_WIDTH, GRILLE_HEIGHT),
            frame=0.006,
            face_thickness=0.003,
            duct_depth=0.008,
            duct_wall=0.0025,
            slat_pitch=0.016,
            slat_width=0.008,
            slat_angle_deg=28.0,
            corner_radius=0.006,
            slats=VentGrilleSlats(
                profile="flat",
                direction="down",
                inset=0.0015,
                divider_count=1,
                divider_width=0.003,
            ),
            frame_profile=VentGrilleFrame(style="radiused", depth=0.0012),
            sleeve=VentGrilleSleeve(style="short", depth=0.008, wall=0.0025),
            center=False,
        ),
        "tower_fan_grille",
    )
    speed_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.026,
            0.012,
            body_style="skirted",
            top_diameter=0.020,
            edge_radius=0.0015,
            center=False,
        ),
        "tower_fan_speed_knob",
    )
    timer_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.022,
            0.011,
            body_style="cylindrical",
            edge_radius=0.0012,
            side_draft_deg=3.0,
            center=False,
        ),
        "tower_fan_timer_knob",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=base_dark,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + TURNTABLE_HEIGHT / 2.0)),
        material=trim_dark,
        name="turntable_ring",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT + TURNTABLE_HEIGHT),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + TURNTABLE_HEIGHT) / 2.0)),
    )

    tower = model.part("tower")
    tower.visual(tower_mesh, material=shell_white, name="tower_shell")
    tower.visual(
        Box((0.048, 0.010, 0.006)),
        origin=Origin(xyz=(-0.018, 0.0, OUTLET_CENTER_Z + AXLE_LENGTH / 2.0 + 0.003)),
        material=trim_dark,
        name="top_bracket",
    )
    tower.visual(
        Box((0.048, 0.010, 0.006)),
        origin=Origin(xyz=(-0.018, 0.0, OUTLET_CENTER_Z - AXLE_LENGTH / 2.0 - 0.003)),
        material=trim_dark,
        name="bottom_bracket",
    )
    tower.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(ROTOR_AXIS_X, 0.0, OUTLET_CENTER_Z + AXLE_LENGTH / 2.0 + 0.003)),
        material=trim_dark,
        name="top_bearing",
    )
    tower.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(ROTOR_AXIS_X, 0.0, OUTLET_CENTER_Z - AXLE_LENGTH / 2.0 - 0.003)),
        material=trim_dark,
        name="bottom_bearing",
    )
    tower.inertial = Inertial.from_geometry(
        Box((TOP_CAP_DEPTH, TOP_CAP_WIDTH, TOWER_TOTAL_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOTAL_HEIGHT / 2.0)),
    )

    grille = model.part("grille")
    grille.visual(
        grille_mesh,
        origin=Origin(rpy=GRILLE_RPY),
        material=grille_grey,
        name="vent_face",
    )
    grille.visual(
        Box((0.0015, 0.010, 0.030)),
        origin=Origin(xyz=(-0.00075, -0.032, 0.0)),
        material=grille_grey,
        name="mount_tab_0",
    )
    grille.visual(
        Box((0.0015, 0.010, 0.030)),
        origin=Origin(xyz=(-0.00075, 0.032, 0.0)),
        material=grille_grey,
        name="mount_tab_1",
    )

    rotor = model.part("rotor")
    rotor.visual(blower_mesh, material=rotor_black, name="blower_wheel")
    rotor.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        material=rotor_black,
        name="shaft_drum",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_OUTER_RADIUS, length=ROTOR_HEIGHT),
        mass=0.12,
    )

    knob_0 = model.part("knob_0")
    knob_0.visual(speed_knob_mesh, material=knob_dark, name="knob")
    knob_0.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=knob_dark,
        name="shaft",
    )
    knob_0.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.012),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    knob_1 = model.part("knob_1")
    knob_1.visual(timer_knob_mesh, material=knob_dark, name="knob")
    knob_1.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=knob_dark,
        name="shaft",
    )
    knob_1.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.011),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    tower_oscillation = model.articulation(
        "tower_oscillation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.70,
            upper=0.70,
        ),
    )
    model.articulation(
        "tower_to_grille",
        ArticulationType.FIXED,
        parent=tower,
        child=grille,
        origin=Origin(xyz=(0.036, 0.0, OUTLET_CENTER_Z)),
    )
    rotor_spin = model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(ROTOR_AXIS_X, 0.0, OUTLET_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    knob_0_spin = model.articulation(
        "knob_0_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=knob_0,
        origin=Origin(xyz=(0.012, -0.022, 0.346)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0),
    )
    knob_1_spin = model.articulation(
        "knob_1_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=knob_1,
        origin=Origin(xyz=(0.012, 0.022, 0.346)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0),
    )

    tower_oscillation.meta["qc_samples"] = [-0.55, 0.0, 0.55]
    rotor_spin.meta["qc_samples"] = [0.0, 2.0]
    knob_0_spin.meta["qc_samples"] = [0.0, 1.5]
    knob_1_spin.meta["qc_samples"] = [0.0, 1.5]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    grille = object_model.get_part("grille")
    rotor = object_model.get_part("rotor")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    tower_oscillation = object_model.get_articulation("tower_oscillation")
    rotor_spin = object_model.get_articulation("rotor_spin")
    knob_0_spin = object_model.get_articulation("knob_0_spin")
    knob_1_spin = object_model.get_articulation("knob_1_spin")

    ctx.allow_isolated_part(
        grille,
        reason="The outlet grille is represented as a shallow press-fit insert; its hidden snap tabs inside the bezel are simplified away.",
    )
    ctx.allow_isolated_part(
        knob_0,
        reason="The speed knob is represented as a visible cap over an omitted encoder shaft and bushing inside the recessed control pocket.",
    )
    ctx.allow_isolated_part(
        knob_1,
        reason="The timer knob is represented as a visible cap over an omitted encoder shaft and bushing inside the recessed control pocket.",
    )

    ctx.check(
        "oscillation joint is vertical revolute",
        tower_oscillation.articulation_type == ArticulationType.REVOLUTE
        and _is_vertical_axis(tower_oscillation.axis),
        details=f"type={tower_oscillation.articulation_type}, axis={tower_oscillation.axis}",
    )
    ctx.check(
        "blower wheel spin is continuous vertical",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS and _is_vertical_axis(rotor_spin.axis),
        details=f"type={rotor_spin.articulation_type}, axis={rotor_spin.axis}",
    )
    ctx.check(
        "control knobs spin continuously",
        knob_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and _is_vertical_axis(knob_0_spin.axis)
        and _is_vertical_axis(knob_1_spin.axis),
        details=(
            f"knob_0=({knob_0_spin.articulation_type}, {knob_0_spin.axis}), "
            f"knob_1=({knob_1_spin.articulation_type}, {knob_1_spin.axis})"
        ),
    )

    with ctx.pose({tower_oscillation: 0.0}):
        ctx.expect_gap(tower, base, axis="z", max_gap=0.001, max_penetration=0.0)
        ctx.expect_overlap(tower, base, axes="xy", min_overlap=0.060)
        ctx.expect_gap(
            grille,
            rotor,
            axis="x",
            positive_elem="vent_face",
            negative_elem="blower_wheel",
            min_gap=0.0005,
            max_gap=0.020,
            name="blower wheel sits just behind outlet grille",
        )
        ctx.expect_overlap(
            rotor,
            grille,
            axes="yz",
            elem_a="blower_wheel",
            elem_b="vent_face",
            min_overlap=0.050,
            name="blower wheel spans the outlet area",
        )

    rest_grille_position = ctx.part_world_position(grille)
    with ctx.pose({tower_oscillation: 0.60}):
        swung_grille_position = ctx.part_world_position(grille)

    ctx.check(
        "tower oscillation swings the outlet sideways",
        rest_grille_position is not None
        and swung_grille_position is not None
        and swung_grille_position[1] > rest_grille_position[1] + 0.010
        and swung_grille_position[0] < rest_grille_position[0] - 0.002,
        details=f"rest={rest_grille_position}, swung={swung_grille_position}",
    )

    return ctx.report()


object_model = build_object_model()
