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

    walnut = model.material("walnut", rgba=(0.40, 0.26, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    smoked_metal = model.material("smoked_metal", rgba=(0.28, 0.29, 0.31, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.47, 0.37, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=walnut,
        name="plinth_body",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (0.19, 0.14),
            (0.19, -0.14),
            (-0.19, 0.14),
            (-0.19, -0.14),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.006)),
            material=dark_rubber,
            name=f"foot_{index}",
        )

    plinth.visual(
        Cylinder(radius=0.052, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=smoked_metal,
        name="platter_support_column",
    )
    plinth.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=satin_black,
        name="platter_bearing_cap",
    )
    plinth.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=satin_black,
        name="support_base_coller",
    )

    tonearm_pivot_xyz = (0.19, -0.085, 0.228)
    plinth.visual(
        Cylinder(radius=0.030, length=0.158),
        origin=Origin(
            xyz=(tonearm_pivot_xyz[0], tonearm_pivot_xyz[1], 0.141),
        ),
        material=smoked_metal,
        name="tonearm_tower",
    )
    plinth.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(
            xyz=(tonearm_pivot_xyz[0], tonearm_pivot_xyz[1], 0.070),
        ),
        material=satin_black,
        name="tonearm_tower_base",
    )
    plinth.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(tonearm_pivot_xyz[0], tonearm_pivot_xyz[1], 0.221)),
        material=satin_black,
        name="tonearm_pivot_housing",
    )
    plinth.visual(
        Box((0.012, 0.032, 0.060)),
        origin=Origin(xyz=(0.156, -0.062, 0.096)),
        material=satin_black,
        name="tonearm_rest_post",
    )
    plinth.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.171, -0.062, 0.127)),
        material=satin_black,
        name="tonearm_rest_cradle",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.47, 0.37, 0.235)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.155, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=dark_rubber,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.022),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.018, 0.0, 0.018),
                (-0.060, 0.004, 0.017),
                (-0.112, 0.014, 0.015),
                (-0.164, 0.026, 0.014),
                (-0.196, 0.034, 0.014),
            ],
            radius=0.005,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_black,
        name="pivot_collar",
    )
    tonearm.visual(
        Box((0.030, 0.020, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.018)),
        material=satin_black,
        name="gimbal_block",
    )
    tonearm.visual(
        arm_tube_mesh,
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.032, 0.014, 0.006)),
        origin=Origin(xyz=(-0.198, 0.036, 0.013)),
        material=aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(-0.206, 0.036, 0.008)),
        material=satin_black,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.056),
        origin=Origin(xyz=(0.022, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.041, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_metal,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.255, 0.060, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(-0.082, 0.016, 0.016)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=tonearm_pivot_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.2,
            lower=-0.42,
            upper=0.28,
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
        positive_elem="platter_disc",
        negative_elem="plinth_body",
        min_gap=0.12,
        max_gap=0.16,
        name="platter rides high above the plinth",
    )
    ctx.expect_origin_gap(
        tonearm,
        platter,
        axis="x",
        min_gap=0.15,
        name="tonearm pivot sits off to the platter side",
    )
    with ctx.pose({tonearm_swing: -0.32}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge_body",
            elem_b="record_mat",
            min_overlap=0.008,
            name="tonearm can swing the cartridge over the record area",
        )

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({tonearm_swing: -0.32}):
        inward = _center(ctx.part_element_world_aabb(tonearm, elem="cartridge_body"))
    with ctx.pose({tonearm_swing: 0.22}):
        outward = _center(ctx.part_element_world_aabb(tonearm, elem="cartridge_body"))

    ctx.check(
        "tonearm sweep moves the cartridge across the platter",
        inward is not None
        and outward is not None
        and math.hypot(inward[0], inward[1]) + 0.05 < math.hypot(outward[0], outward[1]),
        details=f"inward={inward}, outward={outward}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
