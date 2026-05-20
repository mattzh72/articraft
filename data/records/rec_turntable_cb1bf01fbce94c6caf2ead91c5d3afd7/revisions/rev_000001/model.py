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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))

    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.010, 0.000, 0.018),
                (0.055, 0.000, 0.020),
                (0.145, -0.002, 0.018),
                (0.225, 0.003, 0.015),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    plinth = model.part("plinth")
    plinth.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(-0.165, -0.132, 0.004)),
        material=rubber_black,
        name="foot_rear_left",
    )
    plinth.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.165, -0.132, 0.004)),
        material=rubber_black,
        name="foot_rear_right",
    )
    plinth.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(-0.165, 0.132, 0.004)),
        material=rubber_black,
        name="foot_front_left",
    )
    plinth.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.165, 0.132, 0.004)),
        material=rubber_black,
        name="foot_front_right",
    )

    for x in (-0.165, 0.165):
        for y in (-0.132, 0.132):
            plinth.visual(
                Cylinder(radius=0.010, length=0.054),
                origin=Origin(xyz=(x, y, 0.035)),
                material=brushed_aluminum,
                name=f"leg_{'right' if x > 0.0 else 'left'}_{'front' if y > 0.0 else 'rear'}",
            )

    plinth.visual(
        Box((0.020, 0.300, 0.016)),
        origin=Origin(xyz=(-0.165, 0.000, 0.066)),
        material=brushed_aluminum,
        name="left_side_rail",
    )
    plinth.visual(
        Box((0.020, 0.300, 0.016)),
        origin=Origin(xyz=(0.165, 0.000, 0.066)),
        material=brushed_aluminum,
        name="right_side_rail",
    )
    plinth.visual(
        Box((0.350, 0.020, 0.016)),
        origin=Origin(xyz=(0.000, -0.140, 0.066)),
        material=brushed_aluminum,
        name="rear_cross_rail",
    )
    plinth.visual(
        Box((0.350, 0.020, 0.016)),
        origin=Origin(xyz=(0.000, 0.140, 0.066)),
        material=brushed_aluminum,
        name="front_cross_rail",
    )
    plinth.visual(
        Box((0.074, 0.120, 0.016)),
        origin=Origin(xyz=(0.200, -0.080, 0.066)),
        material=brushed_aluminum,
        name="armboard",
    )
    plinth.visual(
        Box((0.332, 0.022, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=brushed_aluminum,
        name="bearing_brace_x",
    )
    plinth.visual(
        Box((0.022, 0.280, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=brushed_aluminum,
        name="bearing_brace_y",
    )
    plinth.visual(
        Cylinder(radius=0.045, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.068)),
        material=graphite,
        name="bearing_sleeve",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.084)),
        material=steel,
        name="bearing_cap",
    )
    plinth.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(xyz=(0.180, -0.080, 0.095)),
        material=graphite,
        name="tonearm_base_pod",
    )
    plinth.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(xyz=(0.212, -0.024, 0.095)),
        material=steel,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.020, 0.008, 0.008)),
        origin=Origin(xyz=(0.212, -0.018, 0.117)),
        material=rubber_black,
        name="arm_rest_clip",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.430, 0.320, 0.110)),
        mass=7.5,
        origin=Origin(xyz=(0.015, 0.000, 0.055)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.149, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=graphite,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.020, length=0.002),
        origin=Origin(xyz=(0.000, 0.000, -0.001)),
        material=steel,
        name="hub_skirt",
    )
    platter.visual(
        Cylinder(radius=0.142, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=rubber_black,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        material=steel,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.149, length=0.022),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=graphite,
        name="pivot_collar",
    )
    tonearm.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(0.010, 0.000, 0.018)),
        material=graphite,
        name="gimbal_block",
    )
    tonearm.visual(
        arm_tube_mesh,
        material=satin_black,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.028, 0.014, 0.004)),
        origin=Origin(xyz=(0.232, 0.000, 0.013)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.240, 0.000, 0.008)),
        material=graphite,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(xyz=(-0.018, 0.000, 0.018), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=steel,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.046, 0.000, 0.018), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=steel,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.012, 0.004, 0.018)),
        origin=Origin(xyz=(-0.002, 0.000, 0.024)),
        material=steel,
        name="anti_skate_fin",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.290, 0.050, 0.045)),
        mass=0.35,
        origin=Origin(xyz=(0.095, 0.000, 0.018)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.000, 0.000, 0.088)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.180, -0.080, 0.116), rpy=(0.000, 0.000, 1.15)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="bearing_cap",
        min_gap=0.0005,
        max_gap=0.003,
        name="platter rides just above exposed bearing cap",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="bearing_cap",
        min_overlap=0.050,
        name="platter remains centered over bearing stage",
    )

    rest_cart_aabb = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")
    mat_aabb = ctx.part_element_world_aabb(platter, elem="record_mat")
    rest_clear = (
        rest_cart_aabb is not None
        and mat_aabb is not None
        and rest_cart_aabb[0][0] > mat_aabb[1][0] + 0.015
    )
    ctx.check(
        "parked cartridge stays outboard of platter",
        rest_clear,
        details=f"cartridge_aabb={rest_cart_aabb}, mat_aabb={mat_aabb}",
    )

    with ctx.pose({tonearm_swing: 1.10}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge_body",
            elem_b="record_mat",
            min_overlap=0.010,
            name="tonearm can swing the cartridge over the record surface",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="record_mat",
            min_gap=0.006,
            max_gap=0.022,
            name="cartridge tracks above the platter surface without contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
