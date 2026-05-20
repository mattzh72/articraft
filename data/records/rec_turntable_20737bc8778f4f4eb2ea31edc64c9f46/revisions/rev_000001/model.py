from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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

    walnut = model.material("walnut", color=(0.43, 0.28, 0.16))
    graphite = model.material("graphite", color=(0.12, 0.12, 0.13))
    matte_black = model.material("matte_black", color=(0.06, 0.06, 0.07))
    satin_aluminum = model.material("satin_aluminum", color=(0.72, 0.73, 0.75))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.36, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=walnut,
        name="main_body",
    )
    plinth.visual(
        Box((0.44, 0.34, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=graphite,
        name="top_deck",
    )
    for idx, (x, y) in enumerate(
        (
            (-0.17, -0.12),
            (-0.17, 0.12),
            (0.17, -0.12),
            (0.17, 0.12),
        ),
        start=1,
    ):
        plinth.visual(
            Cylinder(radius=0.023, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=matte_black,
            name=f"foot_{idx}",
        )

    plinth.visual(
        Cylinder(radius=0.055, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=graphite,
        name="bearing_housing",
    )
    for idx, x in enumerate((-0.08, 0.08), start=1):
        plinth.visual(
            Box((0.05, 0.14, 0.032)),
            origin=Origin(xyz=(x, 0.0, 0.078)),
            material=graphite,
            name=f"bearing_support_{idx}",
        )
    plinth.visual(
        Box((0.11, 0.09, 0.018)),
        origin=Origin(xyz=(0.19, -0.105, 0.071)),
        material=graphite,
        name="tonearm_board",
    )
    plinth.visual(
        Cylinder(radius=0.045, length=0.037),
        origin=Origin(xyz=(0.19, -0.105, 0.0985)),
        material=graphite,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.02, length=0.012),
        origin=Origin(xyz=(0.19, -0.105, 0.123)),
        material=satin_aluminum,
        name="tonearm_pivot_cap",
    )

    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.062)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.152, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.02925)),
        material=matte_black,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.0025, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=satin_aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.152, length=0.028),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.0065, length=0.215),
        origin=Origin(xyz=(0.1075, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.045),
        origin=Origin(xyz=(-0.018, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(-0.055, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.026, 0.012, 0.006)),
        origin=Origin(xyz=(0.223, 0.0, 0.010)),
        material=graphite,
        name="headshell",
    )
    tonearm.visual(
        Box((0.01, 0.008, 0.008)),
        origin=Origin(xyz=(0.231, 0.0, 0.004)),
        material=matte_black,
        name="cartridge",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.29, 0.05, 0.04)),
        mass=0.22,
        origin=Origin(xyz=(0.085, 0.0, 0.02)),
    )

    tonearm_yaw = atan2(0.105, -0.19) - 0.65
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.19, -0.105, 0.129), rpy=(0.0, 0.0, tonearm_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=1.5,
            lower=-0.1,
            upper=0.65,
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
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    with ctx.pose({platter_joint: 0.0}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_disc",
            negative_elem="bearing_housing",
            min_gap=0.0,
            max_gap=0.0005,
            name="platter seats directly on bearing housing",
        )
        ctx.expect_overlap(
            platter,
            plinth,
            axes="xy",
            elem_a="platter_disc",
            elem_b="bearing_housing",
            min_overlap=0.11,
            name="bearing housing stays centered under platter",
        )
        ctx.expect_gap(
            tonearm,
            plinth,
            axis="z",
            positive_elem="pivot_collar",
            negative_elem="tonearm_pivot_cap",
            min_gap=0.0,
            max_gap=0.0005,
            name="tonearm collar seats on pivot cap",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge",
            negative_elem="record_mat",
            min_gap=0.002,
            max_gap=0.02,
            name="cartridge remains slightly above the platter plane at rest",
        )

    platter_aabb = ctx.part_element_world_aabb(platter, elem="platter_disc")
    headshell_rest = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_joint: 0.45}):
        headshell_in = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge",
            elem_b="platter_disc",
            min_overlap=0.004,
            name="cartridge sweeps over the platter area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge",
            negative_elem="record_mat",
            min_gap=0.002,
            max_gap=0.02,
            name="cartridge stays above the record while swung inward",
        )

    platter_center = None
    rest_center = None
    in_center = None
    if platter_aabb is not None:
        platter_center = tuple(
            0.5 * (platter_aabb[0][idx] + platter_aabb[1][idx]) for idx in range(3)
        )
    if headshell_rest is not None:
        rest_center = tuple(
            0.5 * (headshell_rest[0][idx] + headshell_rest[1][idx]) for idx in range(3)
        )
    if headshell_in is not None:
        in_center = tuple(
            0.5 * (headshell_in[0][idx] + headshell_in[1][idx]) for idx in range(3)
        )
    ctx.check(
        "tonearm swings inward across the record",
        platter_center is not None
        and rest_center is not None
        and in_center is not None
        and (
            (in_center[0] - platter_center[0]) ** 2 + (in_center[1] - platter_center[1]) ** 2
        )
        < (
            (rest_center[0] - platter_center[0]) ** 2
            + (rest_center[1] - platter_center[1]) ** 2
            - 0.0001
        ),
        details=f"platter_center={platter_center}, rest_center={rest_center}, in_center={in_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
