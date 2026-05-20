from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_W = 0.86
HOUSING_D = 0.42
HOUSING_H = 0.070
TOP_Z = HOUSING_H
REAR_Y = HOUSING_D * 0.5


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_controller_laptop_stand")

    matte_black = _mat(model, "matte_black", (0.015, 0.016, 0.018, 1.0))
    dark_plastic = _mat(model, "dark_plastic", (0.045, 0.047, 0.052, 1.0))
    graphite = _mat(model, "graphite", (0.12, 0.13, 0.14, 1.0))
    soft_rubber = _mat(model, "soft_rubber", (0.01, 0.01, 0.012, 1.0))
    platter_metal = _mat(model, "brushed_platter", (0.55, 0.57, 0.60, 1.0))
    platter_dark = _mat(model, "platter_grooves", (0.08, 0.085, 0.09, 1.0))
    rail_black = _mat(model, "rail_black", (0.0, 0.0, 0.0, 1.0))
    white_mark = _mat(model, "white_mark", (0.90, 0.92, 0.88, 1.0))
    blue_led = _mat(model, "blue_led", (0.05, 0.26, 0.85, 1.0))
    red_led = _mat(model, "red_led", (0.90, 0.08, 0.04, 1.0))
    stand_metal = _mat(model, "stand_metal", (0.18, 0.19, 0.20, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H * 0.5)),
        material=matte_black,
        name="housing_body",
    )
    housing.visual(
        Box((0.80, 0.36, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.002)),
        material=dark_plastic,
        name="top_deck",
    )

    # Separate dark deck zones and a central mixer strip make the flat housing
    # read as a real two-deck DJ controller instead of a plain slab.
    for x, deck_name in [(-0.235, "deck_panel_0"), (0.235, "deck_panel_1")]:
        housing.visual(
            Box((0.285, 0.315, 0.004)),
            origin=Origin(xyz=(x, -0.005, TOP_Z + 0.006)),
            material=graphite,
            name=deck_name,
        )
    housing.visual(
        Box((0.170, 0.330, 0.005)),
        origin=Origin(xyz=(0.0, -0.005, TOP_Z + 0.007)),
        material=dark_plastic,
        name="mixer_strip",
    )

    # The crossfader rail is a visible slot on the central mixer strip.
    housing.visual(
        Box((0.255, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.145, TOP_Z + 0.010)),
        material=rail_black,
        name="center_rail_slot",
    )
    housing.visual(
        Box((0.010, 0.034, 0.006)),
        origin=Origin(xyz=(-0.132, -0.145, TOP_Z + 0.013)),
        material=white_mark,
        name="rail_stop_0",
    )
    housing.visual(
        Box((0.010, 0.034, 0.006)),
        origin=Origin(xyz=(0.132, -0.145, TOP_Z + 0.013)),
        material=white_mark,
        name="rail_stop_1",
    )

    # Printed cue/play zones and small meters are thin inlaid surfaces, not
    # separate loose parts.
    for side, x in enumerate((-0.235, 0.235)):
        for row, y in enumerate((-0.145, -0.182)):
            housing.visual(
                Box((0.054, 0.024, 0.003)),
                origin=Origin(xyz=(x - 0.065, y, TOP_Z + 0.006)),
                material=blue_led if row == 0 else red_led,
                name=f"deck_{side}_cue_{row}",
            )
            housing.visual(
                Box((0.054, 0.024, 0.003)),
                origin=Origin(xyz=(x + 0.065, y, TOP_Z + 0.006)),
                material=blue_led if row == 0 else red_led,
                name=f"deck_{side}_play_{row}",
            )
        for meter_i in range(4):
            housing.visual(
                Box((0.010, 0.038, 0.003)),
                origin=Origin(xyz=(x - 0.060 + meter_i * 0.018, 0.140, TOP_Z + 0.006)),
                material=blue_led,
                name=f"meter_{side}_{meter_i}",
            )

    # Rear hinge ears are part of the controller housing and visibly support the
    # fold-out laptop stand barrel.
    for x, name in [(-0.290, "stand_ear_0"), (0.290, "stand_ear_1")]:
        housing.visual(
            Box((0.040, 0.036, 0.052)),
            origin=Origin(xyz=(x, REAR_Y + 0.016, TOP_Z + 0.033)),
            material=stand_metal,
            name=name,
        )
    housing.visual(
        Box((0.660, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, REAR_Y + 0.004, TOP_Z + 0.010)),
        material=stand_metal,
        name="rear_hinge_base",
    )

    jog_positions = [(-0.235, -0.010), (0.235, -0.010)]
    for i, (x, y) in enumerate(jog_positions):
        platter = model.part(f"jog_platter_{i}")
        platter.visual(
            Cylinder(radius=0.126, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=platter_metal,
            name="platter_disk",
        )
        platter.visual(
            Cylinder(radius=0.116, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.0195)),
            material=platter_dark,
            name="outer_grip_ring",
        )
        platter.visual(
            Cylinder(radius=0.045, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.024)),
            material=graphite,
            name="center_cap",
        )
        platter.visual(
            Box((0.092, 0.006, 0.003)),
            origin=Origin(xyz=(0.046, 0.0, 0.028)),
            material=white_mark,
            name="rotation_mark",
        )
        for tick in range(12):
            a = 2.0 * pi * tick / 12.0
            platter.visual(
                Box((0.012, 0.004, 0.003)),
                origin=Origin(
                    xyz=(cos(a) * 0.098, sin(a) * 0.098, 0.0235),
                    rpy=(0.0, 0.0, a),
                ),
                material=white_mark,
                name=f"strobe_tick_{tick}",
            )
        model.articulation(
            f"jog_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=platter,
            origin=Origin(xyz=(x, y, TOP_Z + 0.008)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=16.0),
        )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.014, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=soft_rubber,
        name="fader_stem",
    )
    crossfader.visual(
        Box((0.064, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=graphite,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.006, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=white_mark,
        name="fader_center_line",
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.145, TOP_Z + 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.65, lower=-0.105, upper=0.105),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.035,
            0.022,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "ribbed_encoder_knob",
    )
    knob_positions = [
        (-0.060, 0.072),
        (0.000, 0.072),
        (0.060, 0.072),
        (-0.060, 0.018),
        (0.000, 0.018),
        (0.060, 0.018),
    ]
    for i, (x, y) in enumerate(knob_positions):
        knob = model.part(f"knob_{i}")
        knob.visual(knob_mesh, material=graphite, name="knob_cap")
        knob.visual(
            Box((0.018, 0.003, 0.003)),
            origin=Origin(xyz=(0.009, 0.0, 0.024)),
            material=white_mark,
            name="knob_pointer",
        )
        model.articulation(
            f"knob_{i}_turn",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, y, TOP_Z + 0.0095)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=5.0, lower=-2.35, upper=2.35),
        )

    stand_arm = model.part("stand_arm")
    stand_arm.visual(
        Cylinder(radius=0.012, length=0.540),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stand_metal,
        name="hinge_barrel",
    )
    for x, name in [(-0.225, "support_rail_0"), (0.225, "support_rail_1")]:
        stand_arm.visual(
            Box((0.030, 0.320, 0.014)),
            origin=Origin(xyz=(x, -0.160, 0.000)),
            material=stand_metal,
            name=name,
        )
    stand_arm.visual(
        Box((0.520, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, -0.315, 0.000)),
        material=stand_metal,
        name="front_lip",
    )
    stand_arm.visual(
        Box((0.440, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.190, 0.006)),
        material=stand_metal,
        name="brace_bar",
    )
    stand_arm.visual(
        Box((0.480, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.300, 0.012)),
        material=soft_rubber,
        name="rubber_lip_pad",
    )
    model.articulation(
        "stand_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stand_arm,
        origin=Origin(xyz=(0.0, REAR_Y + 0.016, TOP_Z + 0.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jog_0 = object_model.get_part("jog_platter_0")
    jog_1 = object_model.get_part("jog_platter_1")
    crossfader = object_model.get_part("crossfader")
    stand_arm = object_model.get_part("stand_arm")
    crossfader_slide = object_model.get_articulation("crossfader_slide")
    stand_hinge = object_model.get_articulation("stand_hinge")

    for i, jog in enumerate((jog_0, jog_1)):
        ctx.expect_gap(
            jog,
            housing,
            axis="z",
            positive_elem="platter_disk",
            negative_elem=f"deck_panel_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"jog {i} platter sits on deck",
        )
        ctx.expect_overlap(
            jog,
            housing,
            axes="xy",
            elem_a="platter_disk",
            elem_b="housing_body",
            min_overlap=0.20,
            name=f"jog {i} axle is over housing",
        )

    ctx.expect_within(
        crossfader,
        housing,
        axes="x",
        inner_elem="fader_stem",
        outer_elem="center_rail_slot",
        margin=0.002,
        name="crossfader stem starts inside rail",
    )
    rest_pos = ctx.part_world_position(crossfader)
    with ctx.pose({crossfader_slide: 0.105}):
        ctx.expect_within(
            crossfader,
            housing,
            axes="x",
            inner_elem="fader_stem",
            outer_elem="center_rail_slot",
            margin=0.002,
            name="crossfader stem remains retained at rail end",
        )
        extended_pos = ctx.part_world_position(crossfader)
    ctx.check(
        "crossfader slides laterally",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.09,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    folded_aabb = ctx.part_world_aabb(stand_arm)
    with ctx.pose({stand_hinge: 1.15}):
        raised_aabb = ctx.part_world_aabb(stand_arm)
    ctx.check(
        "laptop stand folds upward",
        folded_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > folded_aabb[1][2] + 0.20,
        details=f"folded={folded_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
