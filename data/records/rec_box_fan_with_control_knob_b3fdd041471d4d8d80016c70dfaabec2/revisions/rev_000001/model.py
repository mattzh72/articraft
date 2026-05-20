from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_tower_fan")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.060, 0.066, 1.0))
    warm_white = model.material("warm_white", rgba=(0.88, 0.88, 0.82, 1.0))
    silver = model.material("brushed_silver", rgba=(0.58, 0.61, 0.62, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_gray = model.material("blue_gray", rgba=(0.19, 0.26, 0.31, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.190, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=satin_black,
        name="floor_disk",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=charcoal,
        name="turntable_socket",
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.136, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=warm_white,
        name="rotating_plate",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=warm_white,
        name="pedestal_neck",
    )
    housing.visual(
        Box((0.240, 0.164, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=warm_white,
        name="lower_cap",
    )
    housing.visual(
        Box((0.240, 0.164, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 1.030)),
        material=warm_white,
        name="upper_cap",
    )
    housing.visual(
        Box((0.022, 0.164, 0.910)),
        origin=Origin(xyz=(-0.109, 0.0, 0.575)),
        material=warm_white,
        name="side_rail_0",
    )
    housing.visual(
        Box((0.022, 0.164, 0.910)),
        origin=Origin(xyz=(0.109, 0.0, 0.575)),
        material=warm_white,
        name="side_rail_1",
    )
    housing.visual(
        Box((0.204, 0.014, 0.880)),
        origin=Origin(xyz=(0.0, 0.079, 0.575)),
        material=warm_white,
        name="rear_panel",
    )

    front_grille = VentGrilleGeometry(
        (0.205, 0.820),
        frame=0.014,
        face_thickness=0.004,
        duct_depth=0.018,
        duct_wall=0.0025,
        slat_pitch=0.020,
        slat_width=0.008,
        slat_angle_deg=12.0,
        corner_radius=0.008,
        slats=VentGrilleSlats(
            profile="airfoil",
            direction="down",
            inset=0.002,
            divider_count=2,
            divider_width=0.004,
        ),
        frame_profile=VentGrilleFrame(style="radiused", depth=0.0015),
        sleeve=VentGrilleSleeve(style="short", depth=0.018, wall=0.0025),
    )
    housing.visual(
        mesh_from_geometry(front_grille, "front_grille"),
        origin=Origin(xyz=(0.0, -0.085, 0.575), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_grille",
    )
    housing.visual(
        Box((0.014, 0.024, 0.840)),
        origin=Origin(xyz=(-0.103, -0.085, 0.575)),
        material=silver,
        name="grille_retainer_0",
    )
    housing.visual(
        Box((0.014, 0.024, 0.840)),
        origin=Origin(xyz=(0.103, -0.085, 0.575)),
        material=silver,
        name="grille_retainer_1",
    )
    housing.visual(
        Cylinder(radius=0.037, length=0.008),
        origin=Origin(xyz=(0.0, 0.089, 0.860), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="knob_plate",
    )

    impeller = model.part("impeller")
    blower_wheel = BlowerWheelGeometry(
        outer_radius=0.052,
        inner_radius=0.024,
        width=0.780,
        blade_count=26,
        blade_thickness=0.0026,
        blade_sweep_deg=28.0,
        backplate=True,
        shroud=True,
    )
    impeller.visual(
        mesh_from_geometry(blower_wheel, "blower_wheel"),
        material=blue_gray,
        name="blower_wheel",
    )
    impeller.visual(
        Cylinder(radius=0.006, length=0.860),
        material=dark_plastic,
        name="axle",
    )
    impeller.visual(
        Cylinder(radius=0.027, length=0.050),
        material=dark_plastic,
        name="center_hub",
    )

    speed_knob = model.part("speed_knob")
    knob = KnobGeometry(
        0.055,
        0.025,
        body_style="faceted",
        top_diameter=0.048,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0016),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    speed_knob.visual(
        mesh_from_geometry(knob, "speed_knob_body"),
        material=dark_plastic,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.006, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=warm_white,
        name="pointer_mark",
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.65, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "housing_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=impeller,
        origin=Origin(xyz=(0.0, -0.005, 0.575)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=60.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.0, 0.093, 0.860), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=4.712),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    impeller = object_model.get_part("impeller")
    speed_knob = object_model.get_part("speed_knob")
    oscillation = object_model.get_articulation("base_to_housing")
    impeller_axle = object_model.get_articulation("housing_to_impeller")
    knob_joint = object_model.get_articulation("housing_to_speed_knob")

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        positive_elem="rotating_plate",
        negative_elem="turntable_socket",
        max_gap=0.001,
        max_penetration=0.0005,
        name="rotating plate is seated on base socket",
    )
    ctx.expect_within(
        impeller,
        housing,
        axes="xy",
        inner_elem="blower_wheel",
        margin=0.004,
        name="drum impeller sits inside slim housing footprint",
    )
    ctx.expect_within(
        impeller,
        housing,
        axes="z",
        inner_elem="axle",
        margin=0.002,
        name="vertical impeller axle is captured within tower height",
    )
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="y",
        positive_elem="knob_body",
        negative_elem="knob_plate",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="rear speed knob sits on raised mounting plate",
    )

    ctx.check(
        "oscillation has realistic limited sweep",
        oscillation.motion_limits is not None
        and oscillation.motion_limits.lower <= -0.7
        and oscillation.motion_limits.upper >= 0.7,
        details=f"limits={oscillation.motion_limits}",
    )
    ctx.check(
        "impeller uses continuous revolute axle",
        impeller_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={impeller_axle.articulation_type}",
    )
    ctx.check(
        "speed knob has appliance dial travel",
        knob_joint.motion_limits is not None and knob_joint.motion_limits.upper >= 4.0,
        details=f"limits={knob_joint.motion_limits}",
    )

    rest_grille = ctx.part_element_world_aabb(housing, elem="front_grille")
    with ctx.pose({oscillation: oscillation.motion_limits.upper}):
        turned_grille = ctx.part_element_world_aabb(housing, elem="front_grille")
    ctx.check(
        "oscillation sweeps the tall housing sideways",
        rest_grille is not None
        and turned_grille is not None
        and abs(((turned_grille[0][0] + turned_grille[1][0]) * 0.5) - ((rest_grille[0][0] + rest_grille[1][0]) * 0.5)) > 0.03,
        details=f"rest={rest_grille}, turned={turned_grille}",
    )

    rest_mark = ctx.part_element_world_aabb(speed_knob, elem="pointer_mark")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(speed_knob, elem="pointer_mark")
    ctx.check(
        "speed selector pointer rotates with knob",
        rest_mark is not None
        and turned_mark is not None
        and (turned_mark[1][0] - turned_mark[0][0]) > (rest_mark[1][0] - rest_mark[0][0]) + 0.010,
        details=f"rest={rest_mark}, turned={turned_mark}",
    )

    return ctx.report()


object_model = build_object_model()
