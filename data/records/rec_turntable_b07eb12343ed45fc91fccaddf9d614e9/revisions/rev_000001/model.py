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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    plinth = model.part("plinth")
    plinth_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.45, 0.36, 0.018, corner_segments=10),
            0.055,
        ),
        "plinth_shell",
    )
    plinth.visual(plinth_mesh, material=satin_black, name="plinth_shell")
    plinth.visual(
        Box((0.39, 0.30, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=graphite,
        name="top_inlay",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            plinth.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(0.175 * sx, 0.130 * sy, -0.006)),
                material=rubber,
                name=f"foot_{'r' if sx > 0 else 'l'}{'f' if sy < 0 else 'b'}",
            )
    plinth.visual(
        Cylinder(radius=0.048, length=0.004),
        origin=Origin(xyz=(-0.045, 0.0, 0.057)),
        material=graphite,
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.045),
        origin=Origin(xyz=(0.152, 0.102, 0.0775)),
        material=graphite,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.152, 0.102, 0.058)),
        material=graphite,
        name="tonearm_base_ring",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.177, 0.073, 0.075)),
        material=graphite,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.177, 0.073, 0.097)),
        material=graphite,
        name="arm_rest_cradle",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.45, 0.36, 0.067)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.148, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=brushed_aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.136, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=rubber,
        name="slip_mat",
    )
    platter.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="subplatter_hub",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=brushed_aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.148, length=0.032),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.045, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )

    tonearm = model.part("tonearm")
    tonearm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.010, 0.000, 0.008),
                (0.040, -0.002, 0.009),
                (0.105, -0.010, 0.009),
                (0.175, -0.015, 0.005),
                (0.212, -0.017, 0.002),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Box((0.020, 0.016, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, 0.008)),
        material=graphite,
        name="gimbal_block",
    )
    tonearm.visual(
        tonearm_tube_mesh,
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.030),
        origin=Origin(xyz=(-0.015, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(-0.036, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.028, 0.016, 0.004)),
        origin=Origin(xyz=(0.217, -0.017, 0.002), rpy=(0.0, 0.0, -0.20)),
        material=graphite,
        name="headshell",
    )
    tonearm.visual(
        Box((0.012, 0.010, 0.007)),
        origin=Origin(xyz=(0.226, -0.019, -0.003), rpy=(0.0, 0.0, -0.20)),
        material=rubber,
        name="cartridge",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.255, 0.040, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(0.090, -0.005, 0.007)),
    )

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.152, 0.102, 0.100), rpy=(0.0, 0.0, -2.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.5,
            lower=-0.30,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    platter_spin = object_model.get_articulation("plinth_to_platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_swing = object_model.get_articulation("plinth_to_tonearm")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="bearing_collar",
        min_gap=0.0,
        max_gap=0.0015,
        name="platter sits tightly on bearing collar",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="top_inlay",
        min_overlap=0.18,
        name="platter stays centered over the plinth top",
    )
    ctx.expect_contact(
        platter,
        plinth,
        elem_a="subplatter_hub",
        elem_b="bearing_collar",
        name="subplatter hub bears on the plinth collar",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_collar",
        elem_b="tonearm_pedestal",
        name="tonearm collar stays mounted on the pedestal",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="cartridge",
        negative_elem="slip_mat",
        min_gap=0.001,
        max_gap=0.020,
        name="cartridge clears the platter in the rest pose",
    )

    with ctx.pose({platter_spin: math.pi / 2.0}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_body",
            negative_elem="bearing_collar",
            min_gap=0.0,
            max_gap=0.0015,
            name="platter stays supported while rotating",
        )

    headshell_rest = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swing: tonearm_swing.motion_limits.upper}):
        headshell_inward = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge",
            negative_elem="slip_mat",
            min_gap=0.001,
            max_gap=0.020,
            name="cartridge keeps platter clearance while the arm swings inward",
        )
    ctx.check(
        "tonearm swings inward toward the record center",
        headshell_rest is not None
        and headshell_inward is not None
        and headshell_inward[0][0] < headshell_rest[0][0] - 0.01,
        details=f"rest={headshell_rest}, inward={headshell_inward}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
