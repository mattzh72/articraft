from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.34, 0.22, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.36, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.43, 0.33, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_metal,
        name="top_plate",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            plinth.visual(
                Cylinder(radius=0.022, length=0.012),
                origin=Origin(xyz=(sx * 0.175, sy * 0.125, 0.006)),
                material=rubber,
                name=f"foot_{'r' if sx > 0 else 'l'}{'f' if sy < 0 else 'b'}",
            )
    plinth.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=dark_metal,
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.167, 0.095, 0.070)),
        material=dark_metal,
        name="tonearm_pod",
    )
    plinth.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.167, 0.095, 0.084)),
        material=aluminum,
        name="tonearm_column",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.193, 0.082, 0.078)),
        material=dark_metal,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.193, 0.082, 0.095)),
        material=dark_metal,
        name="arm_rest_clip",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.076)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.148, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.138, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.02925)),
        material=rubber,
        name="slip_mat",
    )
    platter.visual(
        Cylinder(radius=0.028, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.03175)),
        material=dark_metal,
        name="platter_hub",
    )
    platter.visual(
        Cylinder(radius=0.0036, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=aluminum,
        name="spindle",
    )
    platter.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark_metal,
        name="spindle_cap",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.148, length=0.028),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_metal,
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(-0.028, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.065, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.005, length=0.212),
        origin=Origin(xyz=(0.106, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.026, 0.020, 0.006)),
        origin=Origin(xyz=(0.225, 0.0, 0.006)),
        material=dark_metal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(0.232, 0.0, 0.001)),
        material=dark_metal,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(0.237, 0.0, -0.003)),
        material=dark_metal,
        name="stylus_tip",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.275, 0.028, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.080, 0.0, 0.010)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.167, 0.095, 0.094), rpy=(0.0, 0.0, -1.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disc",
        elem_b="plinth_body",
        min_overlap=0.20,
        name="platter is centered on the plinth footprint",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="top_plate",
        min_gap=0.0015,
        max_gap=0.0045,
        name="platter rides slightly above the top plate",
    )
    ctx.expect_origin_gap(
        tonearm,
        platter,
        axis="x",
        min_gap=0.14,
        max_gap=0.19,
        name="tonearm pivot sits off to the right side of the platter",
    )
    ctx.expect_origin_gap(
        tonearm,
        platter,
        axis="y",
        min_gap=0.08,
        max_gap=0.11,
        name="tonearm pivot sits behind the platter center",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="x",
        positive_elem="cartridge_body",
        negative_elem="platter_disc",
        min_gap=0.09,
        name="parked cartridge clears the platter edge",
    )

    with ctx.pose({platter_joint: 1.4}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_disc",
            negative_elem="top_plate",
            max_penetration=0.0,
            name="platter spin does not change vertical clearance",
        )
    with ctx.pose({tonearm_joint: 1.05}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="platter_disc",
            min_overlap=0.012,
            name="tonearm swing reaches the record playing area",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
