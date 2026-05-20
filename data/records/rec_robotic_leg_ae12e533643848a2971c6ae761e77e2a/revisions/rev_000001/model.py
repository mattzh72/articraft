from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="walking_machine_leg_module")

    dark_anodized = Material("dark_anodized", rgba=(0.06, 0.065, 0.07, 1.0))
    graphite = Material("graphite_composite", rgba=(0.015, 0.018, 0.020, 1.0))
    machined = Material("machined_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    amber = Material("amber_service_covers", rgba=(0.95, 0.48, 0.08, 1.0))

    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    hip_housing = model.part("hip_housing")
    hip_housing.visual(
        Box((0.26, 0.20, 0.12)),
        origin=Origin(xyz=(0.02, 0.0, 0.105)),
        material=dark_anodized,
        name="upper_shell",
    )
    hip_housing.visual(
        Box((0.18, 0.16, 0.035)),
        origin=Origin(xyz=(-0.015, 0.0, 0.182)),
        material=machined,
        name="mounting_flange",
    )
    hip_housing.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="hip_side_boss_0",
    )
    hip_housing.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="hip_side_boss_1",
    )
    hip_housing.visual(
        Box((0.17, 0.19, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=dark_anodized,
        name="boss_bridge",
    )
    hip_housing.visual(
        Box((0.070, 0.012, 0.040)),
        origin=Origin(xyz=(0.095, 0.0, 0.118)),
        material=amber,
        name="service_window",
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Cylinder(radius=0.038, length=0.110),
        origin=cyl_y,
        material=machined,
        name="hip_hub",
    )
    thigh_link.visual(
        Box((0.055, 0.045, 0.530)),
        origin=Origin(xyz=(0.0, 0.0, -0.292)),
        material=graphite,
        name="thigh_spine",
    )
    thigh_link.visual(
        Box((0.076, 0.030, 0.430)),
        origin=Origin(xyz=(0.012, 0.0, -0.290)),
        material=dark_anodized,
        name="thigh_cover",
    )
    thigh_link.visual(
        Box((0.090, 0.170, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.548)),
        material=machined,
        name="knee_bridge",
    )
    thigh_link.visual(
        Cylinder(radius=0.052, length=0.042),
        origin=Origin(xyz=(0.0, -0.073, -0.620), rpy=cyl_y.rpy),
        material=machined,
        name="knee_side_boss_0",
    )
    thigh_link.visual(
        Cylinder(radius=0.052, length=0.042),
        origin=Origin(xyz=(0.0, 0.073, -0.620), rpy=cyl_y.rpy),
        material=machined,
        name="knee_side_boss_1",
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Cylinder(radius=0.044, length=0.104),
        origin=cyl_y,
        material=machined,
        name="knee_hub",
    )
    shank_link.visual(
        Box((0.045, 0.040, 0.450)),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=graphite,
        name="shank_spine",
    )
    shank_link.visual(
        Box((0.064, 0.028, 0.360)),
        origin=Origin(xyz=(-0.010, 0.0, -0.275)),
        material=dark_anodized,
        name="shank_cover",
    )
    shank_link.visual(
        Box((0.080, 0.150, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.463)),
        material=machined,
        name="ankle_bridge",
    )
    shank_link.visual(
        Cylinder(radius=0.043, length=0.035),
        origin=Origin(xyz=(0.0, -0.063, -0.520), rpy=cyl_y.rpy),
        material=machined,
        name="ankle_side_boss_0",
    )
    shank_link.visual(
        Cylinder(radius=0.043, length=0.035),
        origin=Origin(xyz=(0.0, 0.063, -0.520), rpy=cyl_y.rpy),
        material=machined,
        name="ankle_side_boss_1",
    )

    ankle_foot = model.part("ankle_foot")
    ankle_foot.visual(
        Cylinder(radius=0.035, length=0.091),
        origin=cyl_y,
        material=machined,
        name="ankle_hub",
    )
    ankle_foot.visual(
        Box((0.050, 0.045, 0.190)),
        origin=Origin(xyz=(0.020, 0.0, -0.095)),
        material=dark_anodized,
        name="ankle_neck",
    )
    ankle_foot.visual(
        Box((0.300, 0.130, 0.035)),
        origin=Origin(xyz=(0.060, 0.0, -0.200)),
        material=machined,
        name="foot_sole",
    )
    ankle_foot.visual(
        Box((0.250, 0.105, 0.015)),
        origin=Origin(xyz=(0.070, 0.0, -0.225)),
        material=black_rubber,
        name="rubber_tread",
    )
    ankle_foot.visual(
        Box((0.055, 0.090, 0.024)),
        origin=Origin(xyz=(0.185, 0.0, -0.180)),
        material=black_rubber,
        name="toe_bumper",
    )

    model.articulation(
        "hip_revolute",
        ArticulationType.REVOLUTE,
        parent=hip_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.0, lower=-0.70, upper=0.90),
    )
    model.articulation(
        "knee_revolute",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=4.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "ankle_revolute",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=5.0, lower=-0.60, upper=0.70),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    hip = object_model.get_articulation("hip_revolute")
    knee = object_model.get_articulation("knee_revolute")
    ankle = object_model.get_articulation("ankle_revolute")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")

    for joint in (hip, knee, ankle):
        ctx.check(
            f"{joint.name} is lateral revolute",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    thigh_origin = ctx.part_world_position(thigh)
    shank_origin = ctx.part_world_position(shank)
    foot_origin = ctx.part_world_position(foot)
    ctx.check(
        "joints are clearly separated",
        thigh_origin is not None
        and shank_origin is not None
        and foot_origin is not None
        and abs(thigh_origin[2] - shank_origin[2]) > 0.55
        and abs(shank_origin[2] - foot_origin[2]) > 0.45,
        details=f"thigh={thigh_origin}, shank={shank_origin}, foot={foot_origin}",
    )

    ctx.expect_contact(
        "hip_housing",
        "thigh_link",
        elem_a="hip_side_boss_1",
        elem_b="hip_hub",
        name="hip side bearing clears central thigh hub",
    )
    ctx.expect_overlap(
        "thigh_link",
        "hip_housing",
        axes="xz",
        min_overlap=0.025,
        elem_a="hip_hub",
        elem_b="hip_side_boss_1",
        name="hip hub aligns with lateral bearing",
    )
    ctx.expect_contact(
        "thigh_link",
        "shank_link",
        elem_a="knee_side_boss_1",
        elem_b="knee_hub",
        name="knee clevis clears central shank hub",
    )
    ctx.expect_contact(
        "shank_link",
        "ankle_foot",
        elem_a="ankle_side_boss_1",
        elem_b="ankle_hub",
        name="ankle clevis clears foot hub",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.35, knee: 0.80, ankle: -0.25}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "three joints move the distal foot in sagittal plane",
        rest_foot is not None
        and flexed_foot is not None
        and abs(flexed_foot[0] - rest_foot[0]) > 0.25
        and abs(flexed_foot[1] - rest_foot[1]) < 0.005,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
