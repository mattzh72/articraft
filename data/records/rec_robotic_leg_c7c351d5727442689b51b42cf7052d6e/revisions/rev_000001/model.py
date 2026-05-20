from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HIP_AXIS = (0.0, 1.0, 0.0)
KNEE_POS = (0.040, 0.0, -0.470)
ANKLE_POS = (-0.030, 0.0, -0.430)
CYL_Y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _rot_y_for_vector(dx: float, dz: float) -> float:
    """Pitch a local +Z cylinder/box axis onto an XZ-plane vector."""
    return math.atan2(dx, dz)


def _link_web_mesh(
    name: str,
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    *,
    start_half_width: float,
    end_half_width: float,
    thickness_y: float,
):
    """A tapered extruded web in the sagittal XZ plane, centered on local Y."""
    sx, sz = start_xz
    ex, ez = end_xz
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz)
    # Unit normal in the XZ sketch plane.
    nx = -dz / length
    nz = dx / length
    profile = [
        (sx + nx * start_half_width, sz + nz * start_half_width),
        (ex + nx * end_half_width, ez + nz * end_half_width),
        (ex - nx * end_half_width, ez - nz * end_half_width),
        (sx - nx * start_half_width, sz - nz * start_half_width),
    ]
    geom = ExtrudeGeometry.centered(profile, thickness_y, cap=True)
    # ExtrudeGeometry's depth is local Z. Rotate it onto world/local Y while
    # leaving the sketch's second coordinate as the leg's Z dimension.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="walking_robot_leg_module")

    gunmetal = model.material("dark_anodized_aluminum", color=(0.06, 0.07, 0.08, 1.0))
    graphite = model.material("graphite_composite", color=(0.12, 0.13, 0.14, 1.0))
    black = model.material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", color=(0.56, 0.58, 0.58, 1.0))
    orange = model.material("orange_service_covers", color=(0.95, 0.42, 0.08, 1.0))

    hip_housing = model.part("hip_housing")
    # Two fixed cheek plates leave a visible slot for the first moving member.
    hip_housing.visual(
        Box((0.270, 0.030, 0.245)),
        origin=Origin(xyz=(0.000, -0.080, 0.010)),
        material=gunmetal,
        name="cheek_0",
    )
    hip_housing.visual(
        Box((0.270, 0.030, 0.245)),
        origin=Origin(xyz=(0.000, 0.080, 0.010)),
        material=gunmetal,
        name="cheek_1",
    )
    hip_housing.visual(
        Box((0.265, 0.190, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.125)),
        material=gunmetal,
        name="upper_bridge",
    )
    hip_housing.visual(
        Box((0.120, 0.160, 0.075)),
        origin=Origin(xyz=(-0.075, 0.000, 0.182)),
        material=graphite,
        name="actuator_box",
    )
    hip_housing.visual(
        Cylinder(radius=0.070, length=0.024),
        origin=Origin(xyz=(0.000, -0.107, 0.000), rpy=CYL_Y.rpy),
        material=steel,
        name="outer_boss_0",
    )
    hip_housing.visual(
        Cylinder(radius=0.070, length=0.024),
        origin=Origin(xyz=(0.000, 0.107, 0.000), rpy=CYL_Y.rpy),
        material=steel,
        name="outer_boss_1",
    )

    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.055, length=0.130),
        origin=CYL_Y,
        material=steel,
        name="hip_hub",
    )
    thigh.visual(
        _link_web_mesh(
            "thigh_web",
            (0.004, -0.038),
            (0.034, -0.405),
            start_half_width=0.040,
            end_half_width=0.033,
            thickness_y=0.058,
        ),
        material=graphite,
        name="main_web",
    )
    thigh.visual(
        Box((0.030, 0.064, 0.270)),
        origin=Origin(
            xyz=(0.010, 0.000, -0.215),
            rpy=(0.0, _rot_y_for_vector(0.030, -0.367), 0.0),
        ),
        material=orange,
        name="service_cover",
    )
    thigh.visual(
        Box((0.092, 0.132, 0.030)),
        origin=Origin(xyz=(KNEE_POS[0], 0.000, KNEE_POS[2] + 0.060)),
        material=gunmetal,
        name="knee_bridge",
    )
    thigh.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(KNEE_POS[0], -0.058, KNEE_POS[2]), rpy=CYL_Y.rpy),
        material=steel,
        name="knee_lug_0",
    )
    thigh.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(KNEE_POS[0], 0.058, KNEE_POS[2]), rpy=CYL_Y.rpy),
        material=steel,
        name="knee_lug_1",
    )

    shank = model.part("shank")
    shank.visual(
        Cylinder(radius=0.040, length=0.091),
        origin=CYL_Y,
        material=steel,
        name="knee_hub",
    )
    shank.visual(
        _link_web_mesh(
            "shank_web",
            (-0.004, -0.035),
            (ANKLE_POS[0] + 0.003, ANKLE_POS[2] + 0.070),
            start_half_width=0.034,
            end_half_width=0.027,
            thickness_y=0.052,
        ),
        material=graphite,
        name="main_web",
    )
    shank.visual(
        Box((0.024, 0.056, 0.245)),
        origin=Origin(
            xyz=(-0.015, 0.000, -0.215),
            rpy=(0.0, _rot_y_for_vector(-0.026, -0.360), 0.0),
        ),
        material=orange,
        name="service_cover",
    )
    shank.visual(
        Box((0.070, 0.108, 0.026)),
        origin=Origin(xyz=(ANKLE_POS[0], 0.000, ANKLE_POS[2] + 0.070)),
        material=gunmetal,
        name="ankle_bridge",
    )
    shank.visual(
        Box((0.055, 0.022, 0.078)),
        origin=Origin(xyz=(ANKLE_POS[0], -0.046, ANKLE_POS[2] + 0.040)),
        material=gunmetal,
        name="ankle_cheek_0",
    )
    shank.visual(
        Box((0.055, 0.022, 0.078)),
        origin=Origin(xyz=(ANKLE_POS[0], 0.046, ANKLE_POS[2] + 0.040)),
        material=gunmetal,
        name="ankle_cheek_1",
    )
    shank.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(xyz=(ANKLE_POS[0], -0.046, ANKLE_POS[2]), rpy=CYL_Y.rpy),
        material=steel,
        name="ankle_lug_0",
    )
    shank.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(xyz=(ANKLE_POS[0], 0.046, ANKLE_POS[2]), rpy=CYL_Y.rpy),
        material=steel,
        name="ankle_lug_1",
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.031, length=0.070),
        origin=CYL_Y,
        material=steel,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.060, 0.052, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, -0.055)),
        material=gunmetal,
        name="ankle_block",
    )
    foot.visual(
        Box((0.285, 0.132, 0.038)),
        origin=Origin(xyz=(0.065, 0.000, -0.105)),
        material=black,
        name="sole",
    )
    foot.visual(
        Box((0.110, 0.138, 0.016)),
        origin=Origin(xyz=(0.165, 0.000, -0.132)),
        material=black,
        name="toe_pad",
    )
    foot.visual(
        Box((0.080, 0.138, 0.016)),
        origin=Origin(xyz=(-0.070, 0.000, -0.132)),
        material=black,
        name="heel_pad",
    )

    model.articulation(
        "hip",
        ArticulationType.REVOLUTE,
        parent=hip_housing,
        child=thigh,
        origin=Origin(),
        axis=HIP_AXIS,
        motion_limits=MotionLimits(effort=120.0, velocity=4.0, lower=-0.75, upper=0.80),
    )
    model.articulation(
        "knee",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=KNEE_POS),
        axis=HIP_AXIS,
        motion_limits=MotionLimits(effort=110.0, velocity=4.5, lower=0.0, upper=1.95),
    )
    model.articulation(
        "ankle",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=ANKLE_POS),
        axis=HIP_AXIS,
        motion_limits=MotionLimits(effort=70.0, velocity=5.0, lower=-0.55, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_housing = object_model.get_part("hip_housing")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip")
    knee = object_model.get_articulation("knee")
    ankle = object_model.get_articulation("ankle")

    for joint in (hip, knee, ankle):
        ctx.check(
            f"{joint.name} is lateral revolute",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == HIP_AXIS,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        hip_housing,
        thigh,
        axis="y",
        positive_elem="cheek_1",
        negative_elem="hip_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="hip hub clears positive cheek",
    )
    ctx.expect_gap(
        thigh,
        hip_housing,
        axis="y",
        positive_elem="hip_hub",
        negative_elem="cheek_0",
        min_gap=0.0,
        max_gap=0.001,
        name="hip hub clears negative cheek",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="y",
        positive_elem="knee_lug_1",
        negative_elem="knee_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="knee hub captured between lugs positive side",
    )
    ctx.expect_gap(
        shank,
        thigh,
        axis="y",
        positive_elem="knee_hub",
        negative_elem="knee_lug_0",
        min_gap=0.0,
        max_gap=0.001,
        name="knee hub captured between lugs negative side",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="y",
        positive_elem="ankle_lug_1",
        negative_elem="ankle_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="ankle hub clears positive lug",
    )
    ctx.expect_gap(
        foot,
        shank,
        axis="y",
        positive_elem="ankle_hub",
        negative_elem="ankle_lug_0",
        min_gap=0.0,
        max_gap=0.001,
        name="ankle hub clears negative lug",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.25, knee: 0.95, ankle: -0.30}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "three joints swing leg in sagittal plane",
        rest_foot is not None
        and flexed_foot is not None
        and abs(flexed_foot[1] - rest_foot[1]) < 1e-6
        and flexed_foot[0] < rest_foot[0] - 0.20,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
