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

    walnut = model.material("walnut", rgba=(0.44, 0.30, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    silver = model.material("silver", rgba=(0.87, 0.87, 0.88, 1.0))
    off_white = model.material("off_white", rgba=(0.92, 0.91, 0.86, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.36, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.446, 0.346, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=satin_black,
        name="top_plate",
    )
    plinth.visual(
        Box((0.100, 0.090, 0.020)),
        origin=Origin(xyz=(0.155, -0.105, 0.079)),
        material=charcoal,
        name="tonearm_base_block",
    )
    plinth.visual(
        Cylinder(radius=0.036, length=0.056),
        origin=Origin(xyz=(0.155, -0.105, 0.097)),
        material=charcoal,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.060, 0.015, 0.072)),
        material=charcoal,
        name="bearing_collar",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.125)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.155, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.148, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.052, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=off_white,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
        material=silver,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.038),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="pivot_bearing",
    )
    tonearm.visual(
        Box((0.024, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.025, 0.012)),
        material=charcoal,
        name="gimbal_block",
    )
    tonearm.visual(
        Cylinder(radius=0.005, length=0.184),
        origin=Origin(xyz=(0.0, 0.132, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.020, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.223, -0.006)),
        material=silver,
        name="headshell",
    )
    tonearm.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.242, -0.014)),
        material=charcoal,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.0015, length=0.006),
        origin=Origin(xyz=(0.0, 0.247, -0.019)),
        material=off_white,
        name="stylus_tip",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.056),
        origin=Origin(xyz=(0.0, -0.028, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(xyz=(0.0, -0.069, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.040, 0.330, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.085, 0.002)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.060, 0.015, 0.069)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.155, -0.105, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-0.15,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("plinth_to_platter")
    arm_swing = object_model.get_articulation("plinth_to_tonearm")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_plate",
        min_gap=0.002,
        max_gap=0.0045,
        name="platter sits slightly above the top plate",
    )

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({arm_swing: 0.0, platter_spin: 0.0}):
        record_aabb = ctx.part_element_world_aabb(platter, elem="record_disc")
        parked_cartridge_aabb = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")
        parked_ok = (
            record_aabb is not None
            and parked_cartridge_aabb is not None
            and parked_cartridge_aabb[0][0] > record_aabb[1][0] + 0.010
        )
        ctx.check(
            "parked tonearm stays outside the record footprint",
            parked_ok,
            details=f"record={record_aabb}, parked_cartridge={parked_cartridge_aabb}",
        )

    with ctx.pose({arm_swing: 1.0, platter_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="stylus_tip",
            elem_b="record_disc",
            min_overlap=0.002,
            name="tonearm swings over the record",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="stylus_tip",
            negative_elem="record_disc",
            min_gap=0.0,
            max_gap=0.003,
            name="stylus hovers just above the record surface",
        )
        play_stylus = _center(ctx.part_element_world_aabb(tonearm, elem="stylus_tip"))

    with ctx.pose({arm_swing: 0.0}):
        parked_stylus = _center(ctx.part_element_world_aabb(tonearm, elem="stylus_tip"))

    ctx.check(
        "tonearm swings inward toward the platter",
        parked_stylus is not None
        and play_stylus is not None
        and play_stylus[0] < parked_stylus[0] - 0.12,
        details=f"parked_stylus={parked_stylus}, play_stylus={play_stylus}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
