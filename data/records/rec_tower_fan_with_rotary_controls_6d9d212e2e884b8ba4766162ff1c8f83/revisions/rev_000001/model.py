from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HOUSING_BOTTOM_Z = 0.16
HOUSING_HEIGHT = 0.88
HOUSING_WIDTH = 0.26
HOUSING_DEPTH = 0.18
CONTROL_PANEL_Z = HOUSING_BOTTOM_Z + HOUSING_HEIGHT + 0.008
BLOWER_CENTER_Z = HOUSING_BOTTOM_Z + 0.45
BLOWER_SHAFT_LENGTH = 0.748


def _housing_shell_mesh() -> object:
    """Rounded, hollow tower housing with a large front grille opening."""

    wall = 0.014
    top_bottom_wall = 0.045
    window_width = 0.202
    window_height = 0.740
    window_bottom = 0.070

    outer = (
        cq.Workplane("XY")
        .box(HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.026)
    )
    outer = outer.edges(">Z").fillet(0.012)

    cavity = (
        cq.Workplane("XY")
        .box(
            HOUSING_WIDTH - 2.0 * wall,
            HOUSING_DEPTH - 2.0 * wall,
            HOUSING_HEIGHT - 2.0 * top_bottom_wall,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, top_bottom_wall))
    )

    front_window = (
        cq.Workplane("XY")
        .box(
            window_width,
            HOUSING_DEPTH + 0.040,
            window_height,
            centered=(True, True, False),
        )
        .translate((0.0, -HOUSING_DEPTH / 2.0, window_bottom))
    )

    shell = outer.cut(cavity).cut(front_window)
    return mesh_from_cadquery(shell, "housing_shell", tolerance=0.0012, angular_tolerance=0.14)


def _top_panel_mesh() -> object:
    return Box((0.180, 0.105, 0.008))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_tower_fan")

    satin_white = model.material("satin_white_plastic", rgba=(0.86, 0.88, 0.86, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.025, 0.028, 0.030, 1.0))
    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    translucent_shadow = model.material("smoked_shadow", rgba=(0.04, 0.05, 0.055, 0.82))
    light_mark = model.material("soft_white_markings", rgba=(0.92, 0.95, 0.90, 1.0))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.180, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=satin_white,
        name="pedestal_base",
    )
    housing.visual(
        Cylinder(radius=0.075, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=satin_white,
        name="pedestal_neck",
    )
    housing.visual(
        Cylinder(radius=0.108, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_white,
        name="lower_collar",
    )
    for name, bearing_z in (
        ("lower_bearing", BLOWER_CENTER_Z - BLOWER_SHAFT_LENGTH / 2.0 - 0.006),
        ("upper_bearing", BLOWER_CENTER_Z + BLOWER_SHAFT_LENGTH / 2.0 + 0.006),
    ):
        housing.visual(
            Box((0.040, 0.100, 0.012)),
            origin=Origin(xyz=(0.0, 0.043, bearing_z)),
            material=satin_white,
            name=f"{name}_arm",
        )
        housing.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, -0.004, bearing_z)),
            material=dark_graphite,
            name=name,
        )
    housing.visual(
        _housing_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_BOTTOM_Z)),
        material=satin_white,
        name="housing_shell",
    )
    housing.visual(
        Box((0.218, 0.004, 0.765)),
        origin=Origin(xyz=(0.0, -HOUSING_DEPTH / 2.0 - 0.0015, HOUSING_BOTTOM_Z + 0.450)),
        material=translucent_shadow,
        name="front_grille",
    )
    for i in range(13):
        x = -0.090 + i * 0.015
        housing.visual(
            Box((0.0042, 0.006, 0.735)),
            origin=Origin(xyz=(x, -HOUSING_DEPTH / 2.0 - 0.0040, HOUSING_BOTTOM_Z + 0.450)),
            material=dark_graphite,
            name=f"grille_slat_{i}",
        )
    for i, z in enumerate((HOUSING_BOTTOM_Z + 0.080, HOUSING_BOTTOM_Z + 0.820)):
        housing.visual(
            Box((0.218, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, -HOUSING_DEPTH / 2.0 - 0.0040, z)),
            material=dark_graphite,
            name=f"grille_rail_{i}",
        )
    housing.visual(
        _top_panel_mesh(),
        origin=Origin(xyz=(0.0, -0.010, HOUSING_BOTTOM_Z + HOUSING_HEIGHT + 0.004)),
        material=matte_black,
        name="control_panel",
    )

    # Small printed ticks around each rotary control; they are shallow ink/paint
    # pads slightly embedded in the top control panel so they read as markings.
    for dial_slot, dial_x in enumerate((-0.045, 0.045)):
        for i, angle in enumerate((-55.0, 0.0, 55.0)):
            theta = math.radians(angle)
            housing.visual(
                Box((0.004, 0.014, 0.0010)),
                origin=Origin(
                    xyz=(
                        dial_x + 0.028 * math.sin(theta),
                        -0.010 - 0.028 * math.cos(theta),
                        CONTROL_PANEL_Z - 0.0002,
                    ),
                    rpy=(0.0, 0.0, -theta),
                ),
                material=light_mark,
                name=f"tick_{dial_slot}_{i}",
            )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.062,
                0.032,
                0.700,
                30,
                blade_thickness=0.0022,
                blade_sweep_deg=22.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        origin=Origin(),
        material=translucent_shadow,
        name="blower_wheel",
    )
    blower_wheel.visual(
        Cylinder(radius=0.010, length=BLOWER_SHAFT_LENGTH),
        origin=Origin(),
        material=dark_graphite,
        name="axle",
    )
    for z in (-0.342, 0.342):
        for spoke_index in range(4):
            angle = spoke_index * math.tau / 4.0
            blower_wheel.visual(
                Box((0.060, 0.006, 0.006)),
                origin=Origin(
                    xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), z),
                    rpy=(0.0, 0.0, angle),
                ),
                material=dark_graphite,
                name=f"spoke_{'top' if z > 0 else 'bottom'}_{spoke_index}",
            )
    model.articulation(
        "housing_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, -0.004, BLOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=95.0),
    )

    for index, x in enumerate((-0.045, 0.045)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=0.019, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=satin_white,
            name="dial_cap",
        )
        dial.visual(
            Cylinder(radius=0.023, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=satin_white,
            name="dial_skirt",
        )
        for ridge_index in range(12):
            angle = ridge_index * math.tau / 12.0
            dial.visual(
                Box((0.0022, 0.0042, 0.013)),
                origin=Origin(
                    xyz=(0.0205 * math.cos(angle), 0.0205 * math.sin(angle), 0.009),
                    rpy=(0.0, 0.0, angle),
                ),
                material=satin_white,
                name=f"dial_rib_{ridge_index}",
            )
        dial.visual(
            Box((0.0035, 0.024, 0.0022)),
            origin=Origin(xyz=(0.0, -0.004, 0.0188)),
            material=dark_graphite,
            name="dial_indicator",
        )
        model.articulation(
            f"housing_to_dial_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(x, -0.010, CONTROL_PANEL_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=4.0, lower=0.0, upper=4.712),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blower = object_model.get_part("blower_wheel")
    blower_joint = object_model.get_articulation("housing_to_blower_wheel")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")
    dial_joint_0 = object_model.get_articulation("housing_to_dial_0")
    dial_joint_1 = object_model.get_articulation("housing_to_dial_1")

    ctx.check(
        "blower wheel uses a continuous vertical joint",
        blower_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in blower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={blower_joint.articulation_type}, axis={blower_joint.axis}",
    )
    ctx.check(
        "top dials use separate revolute joints",
        dial_joint_0.articulation_type == ArticulationType.REVOLUTE
        and dial_joint_1.articulation_type == ArticulationType.REVOLUTE
        and dial_joint_0.child != dial_joint_1.child,
        details=f"joints=({dial_joint_0}, {dial_joint_1})",
    )
    ctx.expect_within(
        blower,
        housing,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="housing_shell",
        margin=0.004,
        name="blower wheel fits inside the tower housing footprint",
    )
    ctx.expect_gap(
        blower,
        housing,
        axis="y",
        positive_elem="blower_wheel",
        negative_elem="front_grille",
        min_gap=0.010,
        name="blower wheel sits behind the front grille",
    )
    with ctx.pose({blower_joint: math.pi / 2.0}):
        ctx.expect_gap(
            blower,
            housing,
            axis="y",
            positive_elem="blower_wheel",
            negative_elem="front_grille",
            min_gap=0.010,
            name="rotating blower clears the front grille",
        )
    for dial, joint in ((dial_0, dial_joint_0), (dial_1, dial_joint_1)):
        ctx.expect_gap(
            dial,
            housing,
            axis="z",
            positive_elem="dial_cap",
            negative_elem="control_panel",
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"{dial.name} seats on the top control panel",
        )
        with ctx.pose({joint: 3.2}):
            ctx.expect_gap(
                dial,
                housing,
                axis="z",
                positive_elem="dial_cap",
                negative_elem="control_panel",
                max_gap=0.002,
                max_penetration=0.0005,
                name=f"{dial.name} remains seated when rotated",
            )

    return ctx.report()


object_model = build_object_model()
