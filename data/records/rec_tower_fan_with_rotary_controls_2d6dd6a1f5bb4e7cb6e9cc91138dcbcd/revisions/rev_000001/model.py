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
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_HEIGHT = 0.040
PEDESTAL_RADIUS = 0.032
PEDESTAL_HEIGHT = 0.028

BODY_WIDTH = 0.115
BODY_DEPTH = 0.078
BODY_HEIGHT = 0.780
BODY_CORNER_RADIUS = 0.024
BODY_WALL = 0.0035
BODY_TOP_THICKNESS = 0.014
BODY_BOTTOM_THICKNESS = 0.020
BODY_REAR_WALL = 0.006
BODY_BOTTOM_Z = 0.028

GRILLE_WIDTH = 0.088
GRILLE_HEIGHT = 0.704
TOP_DECK_WIDTH = 0.092
TOP_DECK_DEPTH = 0.064
TOP_DECK_HEIGHT = 0.010
TOP_DECK_Z = BODY_BOTTOM_Z + BODY_HEIGHT + TOP_DECK_HEIGHT / 2.0

WHEEL_RADIUS = 0.026
WHEEL_INNER_RADIUS = 0.011
WHEEL_HEIGHT = 0.662
WHEEL_BOTTOM_Z = 0.102


def _build_base_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT).edges("|Z").fillet(0.085)


def _build_housing_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).edges("|Z").fillet(BODY_CORNER_RADIUS)
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - BODY_TOP_THICKNESS - BODY_BOTTOM_THICKNESS,
        )
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS - BODY_WALL)
        .translate((0.0, 0.0, (BODY_BOTTOM_THICKNESS - BODY_TOP_THICKNESS) / 2.0))
    )
    shell = outer.cut(inner)
    front_opening = (
        cq.Workplane("XY")
        .box(GRILLE_WIDTH, BODY_DEPTH - BODY_REAR_WALL, GRILLE_HEIGHT)
        .translate((0.0, BODY_REAR_WALL / 2.0, 0.018))
    )
    return shell.cut(front_opening)


def _build_top_deck_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(TOP_DECK_WIDTH, TOP_DECK_DEPTH, TOP_DECK_HEIGHT).edges("|Z").fillet(0.018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    base_plastic = model.material("base_plastic", rgba=(0.86, 0.87, 0.88, 1.0))
    body_plastic = model.material("body_plastic", rgba=(0.93, 0.94, 0.95, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    button_satin = model.material("button_satin", rgba=(0.80, 0.81, 0.83, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "tower_fan_base"),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=base_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT / 2.0)),
        material=trim_gray,
        name="pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT + PEDESTAL_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + PEDESTAL_HEIGHT) / 2.0)),
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim_gray,
        name="turntable_ring",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=body_plastic,
        name="neck",
    )
    housing.visual(
        mesh_from_cadquery(_build_housing_shell_shape(), "tower_fan_housing_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=body_plastic,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_top_deck_shape(), "tower_fan_top_deck"),
        origin=Origin(xyz=(0.0, 0.0, TOP_DECK_Z)),
        material=body_plastic,
        name="top_deck",
    )
    housing.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (GRILLE_WIDTH, GRILLE_HEIGHT),
                frame=0.008,
                face_thickness=0.0035,
                duct_depth=0.010,
                duct_wall=0.0025,
                slat_pitch=0.016,
                slat_width=0.008,
                slat_angle_deg=28.0,
                corner_radius=0.008,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2, divider_width=0.004),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.0012),
                sleeve=VentGrilleSleeve(style="none"),
            ),
            "tower_fan_front_grille",
        ),
        origin=Origin(
            xyz=(0.0, BODY_DEPTH / 2.0 - 0.0015, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0 + 0.018),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_finish,
        name="front_grille",
    )
    housing.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.070, 0.560),
                0.0028,
                slot_size=(0.022, 0.0042),
                pitch=(0.030, 0.014),
                frame=0.006,
                corner_radius=0.006,
                slot_angle_deg=0.0,
                stagger=True,
            ),
            "tower_fan_rear_intake",
        ),
        origin=Origin(
            xyz=(0.0, -BODY_DEPTH / 2.0 + 0.0014, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0 + 0.010),
            rpy=(math.pi / 2.0, 0.0, math.pi),
        ),
        material=trim_gray,
        name="rear_intake",
    )
    housing.visual(
        Cylinder(radius=0.0215, length=0.004),
        origin=Origin(xyz=(0.0, 0.017, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.002)),
        material=trim_gray,
        name="front_dial_seat",
    )
    housing.visual(
        Cylinder(radius=0.0190, length=0.004),
        origin=Origin(xyz=(0.0, -0.018, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.002)),
        material=trim_gray,
        name="rear_dial_seat",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.030, 0.000, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.002)),
        material=trim_gray,
        name="button_guide",
    )
    housing.visual(
        Cylinder(radius=0.0075, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_BOTTOM_Z - 0.013)),
        material=trim_gray,
        name="lower_bearing",
    )
    housing.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, -0.022, WHEEL_BOTTOM_Z - 0.013)),
        material=trim_gray,
        name="lower_bearing_support",
    )
    housing.visual(
        Cylinder(radius=0.0075, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_BOTTOM_Z + WHEEL_HEIGHT + 0.013)),
        material=trim_gray,
        name="upper_bearing",
    )
    housing.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, -0.022, WHEEL_BOTTOM_Z + WHEEL_HEIGHT + 0.013)),
        material=trim_gray,
        name="upper_bearing_support",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_BOTTOM_Z + BODY_HEIGHT + TOP_DECK_HEIGHT + 0.010)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BOTTOM_Z + BODY_HEIGHT + TOP_DECK_HEIGHT + 0.010) / 2.0)),
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                WHEEL_RADIUS,
                WHEEL_INNER_RADIUS,
                WHEEL_HEIGHT,
                26,
                blade_thickness=0.0016,
                blade_sweep_deg=24.0,
            ),
            "tower_fan_blower_wheel",
        ),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_HEIGHT / 2.0)),
        material=wheel_gray,
        name="wheel",
    )
    blower_wheel.visual(
        Cylinder(radius=0.006, length=WHEEL_HEIGHT + 0.016),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_HEIGHT / 2.0)),
        material=trim_gray,
        name="axle",
    )
    blower_wheel.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_gray,
        name="lower_hub",
    )
    blower_wheel.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_HEIGHT - 0.002)),
        material=trim_gray,
        name="upper_hub",
    )
    blower_wheel.inertial = Inertial.from_geometry(
        Box((WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0, WHEEL_HEIGHT + 0.016)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_HEIGHT / 2.0)),
    )

    front_dial = model.part("front_dial")
    front_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="knurled", count=36, depth=0.0008, helix_angle_deg=20.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "tower_fan_front_dial",
        ),
        material=knob_dark,
        name="dial_cap",
    )
    front_dial.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="dial_stub",
    )
    front_dial.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    rear_dial = model.part("rear_dial")
    rear_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.016,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="knurled", count=32, depth=0.0007, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "tower_fan_rear_dial",
        ),
        material=knob_dark,
        name="dial_cap",
    )
    rear_dial.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="dial_stub",
    )
    rear_dial.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Cylinder(radius=0.0105, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=button_satin,
        name="button_cap",
    )
    oscillation_button.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_gray,
        name="button_stem",
    )
    oscillation_button.inertial = Inertial.from_geometry(
        Box((0.021, 0.021, 0.021)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.8,
            upper=0.8,
        ),
    )
    model.articulation(
        "housing_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
    )
    model.articulation(
        "housing_to_front_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=front_dial,
        origin=Origin(xyz=(0.0, 0.017, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0),
    )
    model.articulation(
        "housing_to_rear_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rear_dial,
        origin=Origin(xyz=(0.0, -0.018, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0),
    )
    model.articulation(
        "housing_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=oscillation_button,
        origin=Origin(xyz=(0.030, 0.000, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0 + 0.007)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    blower_wheel = object_model.get_part("blower_wheel")
    front_dial = object_model.get_part("front_dial")
    rear_dial = object_model.get_part("rear_dial")
    oscillation_button = object_model.get_part("oscillation_button")
    oscillation = object_model.get_articulation("base_to_housing")
    button_joint = object_model.get_articulation("housing_to_oscillation_button")

    ctx.allow_overlap(
        oscillation_button,
        housing,
        elem_a="button_stem",
        elem_b="button_guide",
        reason="The oscillation button stem is intentionally represented as sliding inside the guide sleeve proxy.",
    )
    ctx.allow_overlap(
        oscillation_button,
        housing,
        elem_a="button_stem",
        elem_b="top_deck",
        reason="The button stem passes through an implied deck opening that is simplified as a solid top deck.",
    )

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_ring",
        negative_elem="pedestal",
        name="housing turntable sits on pedestal",
    )
    ctx.expect_overlap(
        housing,
        base,
        axes="xy",
        min_overlap=0.05,
        elem_a="turntable_ring",
        elem_b="pedestal",
        name="housing stays centered over pedestal",
    )

    low_aabb = None
    high_aabb = None
    with ctx.pose({oscillation: -0.8}):
        low_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    with ctx.pose({oscillation: 0.8}):
        high_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    ctx.check(
        "housing oscillates left and right",
        low_aabb is not None
        and high_aabb is not None
        and float(low_aabb[0][0] + low_aabb[1][0]) > 0.03
        and float(high_aabb[0][0] + high_aabb[1][0]) < -0.03,
        details=f"low_aabb={low_aabb}, high_aabb={high_aabb}",
    )
    ctx.expect_within(
        blower_wheel,
        housing,
        axes="xy",
        margin=0.002,
        elem_a="wheel",
        elem_b="housing_shell",
        name="blower wheel stays inside housing cavity footprint",
    )
    ctx.expect_overlap(
        blower_wheel,
        housing,
        axes="z",
        min_overlap=0.55,
        elem_a="wheel",
        elem_b="housing_shell",
        name="blower wheel spans most of housing height",
    )
    ctx.expect_gap(
        blower_wheel,
        housing,
        axis="z",
        max_gap=0.0,
        max_penetration=1e-6,
        positive_elem="axle",
        negative_elem="lower_bearing",
        name="blower wheel axle sits on lower bearing",
    )
    ctx.expect_gap(
        housing,
        blower_wheel,
        axis="z",
        max_gap=0.0,
        max_penetration=1e-6,
        positive_elem="upper_bearing",
        negative_elem="axle",
        name="blower wheel axle reaches upper bearing",
    )
    ctx.expect_gap(
        front_dial,
        housing,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        positive_elem="dial_stub",
        negative_elem="front_dial_seat",
        name="front dial sits on its top seat",
    )
    ctx.expect_gap(
        rear_dial,
        housing,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        positive_elem="dial_stub",
        negative_elem="rear_dial_seat",
        name="rear dial sits on its top seat",
    )

    released_aabb = ctx.part_element_world_aabb(oscillation_button, elem="button_cap")
    pressed_aabb = None
    with ctx.pose({button_joint: 0.003}):
        pressed_aabb = ctx.part_element_world_aabb(oscillation_button, elem="button_cap")
        ctx.expect_gap(
            oscillation_button,
            housing,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem="button_cap",
            negative_elem="button_guide",
            name="pressed button cap seats at guide rim",
        )
    ctx.check(
        "oscillation button depresses downward",
        released_aabb is not None
        and pressed_aabb is not None
        and float(pressed_aabb[0][2]) < float(released_aabb[0][2]) - 0.002,
        details=f"released_aabb={released_aabb}, pressed_aabb={pressed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
