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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))

    plinth = model.part("plinth")
    plinth_shell = _mesh(
        "plinth_shell",
        ExtrudeGeometry(
            rounded_rect_profile(0.50, 0.38, 0.018, corner_segments=8),
            0.045,
            center=True,
        ),
    )
    plinth.visual(
        plinth_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=plinth_black,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.44, 0.32, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=charcoal,
        name="top_deck",
    )
    plinth.visual(
        Box((0.020, 0.18, 0.080)),
        origin=Origin(xyz=(-0.215, 0.03, 0.077)),
        material=charcoal,
        name="left_stage_support",
    )
    plinth.visual(
        Box((0.020, 0.18, 0.080)),
        origin=Origin(xyz=(0.215, 0.03, 0.077)),
        material=charcoal,
        name="right_stage_support",
    )
    plinth.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=steel,
        name="spindle_bearing_housing",
    )
    plinth.visual(
        Cylinder(radius=0.034, length=0.034),
        origin=Origin(xyz=(0.198, -0.100, 0.0655)),
        material=charcoal,
        name="tonearm_podium",
    )
    for sx in (-0.19, 0.19):
        for sy in (-0.13, 0.13):
            plinth.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(sx, sy, 0.004)),
                material=satin_black,
                name=f"foot_{'l' if sx < 0 else 'r'}_{'f' if sy > 0 else 'b'}",
            )
    plinth.inertial = Inertial.from_geometry(
        Box((0.50, 0.38, 0.12)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    platter = model.part("platter")
    platter_shell = _mesh(
        "platter_shell",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.018, 0.000),
                (0.135, 0.004),
                (0.149, 0.010),
                (0.154, 0.019),
                (0.154, 0.023),
                (0.138, 0.024),
                (0.0, 0.024),
            ],
            segments=72,
        ),
    )
    platter.visual(
        platter_shell,
        material=aluminum,
        name="platter_shell",
    )
    platter.visual(
        Cylinder(radius=0.147, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=steel,
        name="spindle_tip",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.154, length=0.027),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
    )

    tonearm = model.part("tonearm")
    tonearm_tube = _mesh(
        "tonearm_tube",
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.000),
                (-0.004, 0.040, 0.001),
                (-0.010, 0.130, 0.004),
                (-0.018, 0.215, 0.006),
            ],
            radius=0.005,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="pivot_collar",
    )
    tonearm.visual(
        tonearm_tube,
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(xyz=(0.0, -0.016, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.0, -0.046, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.022, 0.030, 0.006)),
        origin=Origin(xyz=(-0.018, 0.223, 0.003)),
        material=steel,
        name="headshell",
    )
    tonearm.visual(
        Box((0.012, 0.018, 0.010)),
        origin=Origin(xyz=(-0.019, 0.233, 0.000)),
        material=satin_black,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.010),
        origin=Origin(xyz=(-0.019, 0.242, -0.005)),
        material=steel,
        name="stylus",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.060, 0.280, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(-0.008, 0.090, 0.004)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=15.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.198, -0.100, 0.0825)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.5,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_joint = object_model.get_articulation("tonearm_swing")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_shell",
        negative_elem="top_deck",
        min_gap=0.001,
        max_gap=0.003,
        name="platter floats just above the top deck",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="x",
        positive_elem="platter_shell",
        negative_elem="left_stage_support",
        min_gap=0.045,
        max_gap=0.060,
        name="left support frames the platter with side clearance",
    )
    ctx.expect_gap(
        plinth,
        platter,
        axis="x",
        positive_elem="right_stage_support",
        negative_elem="platter_shell",
        min_gap=0.045,
        max_gap=0.060,
        name="right support frames the platter with side clearance",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="x",
        positive_elem="headshell",
        negative_elem="record_disc",
        min_gap=0.015,
        name="parked tonearm clears the record footprint",
    )

    with ctx.pose({tonearm_joint: 1.08}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="record_disc",
            min_overlap=0.030,
            name="tonearm swing brings the headshell over the record",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="record_disc",
            min_gap=0.0005,
            max_gap=0.0025,
            name="cartridge body stays just above the record surface",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
