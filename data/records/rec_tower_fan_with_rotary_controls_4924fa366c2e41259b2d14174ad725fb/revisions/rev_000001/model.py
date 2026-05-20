from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.18
BODY_DEPTH = 0.14
BODY_HEIGHT = 0.95
BODY_BOTTOM_Z = 0.05
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT
TOP_CAP_THICKNESS = 0.008
TOP_CAP_TOP_Z = BODY_TOP_Z + TOP_CAP_THICKNESS
DIAL_POSITIONS = ((-0.042, -0.004), (0.042, -0.004))


def _rounded_tower_shell() -> cq.Workplane:
    """Outer tower housing with a real front cavity and dial shaft holes."""
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BODY_BOTTOM_Z))
    )

    # A deep front outlet cut leaves side rails, a slim rear wall, and top/bottom
    # caps instead of a solid block behind the grille.
    outlet_cut = (
        cq.Workplane("XY")
        .box(0.140, 0.190, 0.790, centered=(True, True, False))
        .translate((0.0, -0.038, 0.135))
    )
    shell = shell.cut(outlet_cut)

    # Two vertical holes through the top cap for the rotary dial shafts.
    for x, y in DIAL_POSITIONS:
        shaft_hole = (
            cq.Workplane("XY")
            .circle(0.012)
            .extrude(0.13)
            .translate((x, y, BODY_TOP_Z - 0.045))
        )
        shell = shell.cut(shaft_hole)

    return shell


def _top_control_cap() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.162, 0.122, TOP_CAP_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
        .translate((0.0, 0.0, BODY_TOP_Z))
    )
    for x, y in DIAL_POSITIONS:
        hole = (
            cq.Workplane("XY")
            .circle(0.012)
            .extrude(0.030)
            .translate((x, y, BODY_TOP_Z - 0.010))
        )
        cap = cap.cut(hole)
    return cap


def _bushing_ring(x: float, y: float) -> cq.Workplane:
    # A low annular collar seated into the top cap; the dial shaft clears the
    # inner hole while the knob rests on the upper face.
    return (
        cq.Workplane("XY")
        .circle(0.028)
        .circle(0.012)
        .extrude(0.0045)
        .translate((x, y, TOP_CAP_TOP_Z - 0.0005))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_residential_tower_fan")

    satin_black = model.material("satin_black", rgba=(0.015, 0.015, 0.017, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.060, 1.0))
    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.55, 0.56, 0.55, 1.0))
    pale_marker = model.material("pale_marker", rgba=(0.95, 0.93, 0.84, 1.0))
    blower_blue = model.material("muted_blower_blue", rgba=(0.18, 0.42, 0.52, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="pedestal_disk",
    )
    base.visual(
        Cylinder(radius=0.116, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_plastic,
        name="raised_step",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_black,
        name="oscillation_hub",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_tower_shell(), "tower_shell", tolerance=0.0012),
        material=warm_white,
        name="tower_shell",
    )
    body.visual(
        Cylinder(radius=0.061, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=warm_white,
        name="lower_collar",
    )
    body.visual(
        mesh_from_cadquery(_top_control_cap(), "top_cap", tolerance=0.001),
        material=dark_plastic,
        name="top_cap",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.148, 0.760),
                0.004,
                slot_size=(0.112, 0.006),
                pitch=(0.134, 0.018),
                frame=0.012,
                corner_radius=0.010,
            ),
            "front_outlet_grille",
        ),
        origin=Origin(xyz=(0.0, -0.072, 0.525), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_grille",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.112, 0.520),
                0.003,
                slot_size=(0.045, 0.004),
                pitch=(0.062, 0.011),
                frame=0.010,
                corner_radius=0.008,
                stagger=True,
            ),
            "rear_intake_grille",
        ),
        origin=Origin(xyz=(0.0, 0.069, 0.560), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="rear_grille",
    )
    body.visual(
        Box((0.160, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.006, 0.155)),
        material=dark_plastic,
        name="lower_bearing_bridge",
    )
    body.visual(
        Box((0.160, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.006, 0.895)),
        material=dark_plastic,
        name="upper_bearing_bridge",
    )
    for index, (x, y) in enumerate(DIAL_POSITIONS):
        body.visual(
            mesh_from_cadquery(_bushing_ring(x, y), f"dial_bushing_{index}", tolerance=0.0007),
            material=soft_gray,
            name=f"dial_bushing_{index}",
        )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.95, upper=0.95),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.041,
                inner_radius=0.022,
                width=0.690,
                blade_count=34,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        material=blower_blue,
        name="cage_rotor",
    )
    blower_wheel.visual(
        Cylinder(radius=0.006, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_plastic,
        name="rotor_shaft",
    )
    blower_wheel.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blower_blue,
        name="shaft_hub",
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, -0.006, 0.525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=60.0),
        motion_properties=MotionProperties(damping=0.01),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.026,
            body_style="skirted",
            top_diameter=0.039,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "top_control_dial",
    )
    for index, (x, y) in enumerate(DIAL_POSITIONS):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=0.0065, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=dark_plastic,
            name="dial_shaft",
        )
        dial.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=soft_gray,
            name="dial_cap",
        )
        dial.visual(
            Box((0.028, 0.004, 0.0016)),
            origin=Origin(xyz=(0.006, 0.0, 0.0268)),
            material=pale_marker,
            name="pointer_line",
        )
        model.articulation(
            f"body_to_dial_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x, y, TOP_CAP_TOP_Z + 0.0040)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=6.0, lower=0.0, upper=2.0 * math.pi),
            motion_properties=MotionProperties(damping=0.015, friction=0.02),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower_wheel")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")

    oscillation = object_model.get_articulation("base_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    dial_spin_0 = object_model.get_articulation("body_to_dial_0")
    dial_spin_1 = object_model.get_articulation("body_to_dial_1")

    for bridge_name in ("lower_bearing_bridge", "upper_bearing_bridge"):
        ctx.allow_overlap(
            body,
            blower,
            elem_a=bridge_name,
            elem_b="rotor_shaft",
            reason="The rotating blower shaft is intentionally captured inside the stationary bearing bridge.",
        )
        ctx.expect_overlap(
            body,
            blower,
            axes="xy",
            elem_a=bridge_name,
            elem_b="rotor_shaft",
            min_overlap=0.010,
            name=f"{bridge_name} surrounds the rotor shaft in plan",
        )
        ctx.expect_overlap(
            body,
            blower,
            axes="z",
            elem_a=bridge_name,
            elem_b="rotor_shaft",
            min_overlap=0.006,
            name=f"{bridge_name} captures the rotor shaft vertically",
        )

    ctx.check(
        "floor appliance height",
        (ctx.part_world_aabb(body) is not None)
        and (ctx.part_world_aabb(body)[1][2] - ctx.part_world_aabb(base)[0][2] > 1.05),
        details="Tower fan should stand just over one metre including its base.",
    )
    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="oscillation_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillating body is seated on hub",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="cage_rotor",
        outer_elem="tower_shell",
        margin=0.0,
        name="blower wheel stays inside tower footprint",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        elem_a="cage_rotor",
        elem_b="front_grille",
        min_overlap=0.60,
        name="blower spans the tall outlet grille",
    )
    for index, dial in enumerate((dial_0, dial_1)):
        ctx.expect_gap(
            dial,
            body,
            axis="z",
            positive_elem="dial_cap",
            negative_elem=f"dial_bushing_{index}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"dial_{index} rests on its top bushing",
        )

    ctx.check(
        "blower has continuous vertical spin",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(blower_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blower_spin.articulation_type}, axis={blower_spin.axis}",
    )
    ctx.check(
        "oscillation has limited vertical yaw",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and tuple(oscillation.axis) == (0.0, 0.0, 1.0)
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower < -0.5
        and oscillation.motion_limits.upper > 0.5,
        details=f"type={oscillation.articulation_type}, axis={oscillation.axis}, limits={oscillation.motion_limits}",
    )
    ctx.check(
        "two separate top dial shafts",
        dial_spin_0.articulation_type == ArticulationType.REVOLUTE
        and dial_spin_1.articulation_type == ArticulationType.REVOLUTE
        and abs(dial_spin_0.origin.xyz[0] - dial_spin_1.origin.xyz[0]) > 0.070
        and abs(dial_spin_0.origin.xyz[2] - TOP_CAP_TOP_Z) < 0.010
        and abs(dial_spin_1.origin.xyz[2] - TOP_CAP_TOP_Z) < 0.010,
        details=f"origins={dial_spin_0.origin.xyz}, {dial_spin_1.origin.xyz}",
    )

    rest_aabb = ctx.part_world_aabb(body)
    with ctx.pose({oscillation: 0.75}):
        swung_aabb = ctx.part_world_aabb(body)
    rest_width = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    swung_width = None if swung_aabb is None else swung_aabb[1][0] - swung_aabb[0][0]
    ctx.check(
        "body visibly oscillates about base",
        rest_width is not None and swung_width is not None and swung_width > rest_width + 0.020,
        details=f"rest_width={rest_width}, swung_width={swung_width}",
    )

    return ctx.report()


object_model = build_object_model()
