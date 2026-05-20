from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="centered_walking_leg_module")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    gunmetal = Material("brushed_gunmetal", rgba=(0.34, 0.36, 0.37, 1.0))
    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.016, 1.0))
    rubber = Material("black_rubber", rgba=(0.02, 0.018, 0.015, 1.0))
    warning_yellow = Material("servo_warning_yellow", rgba=(0.95, 0.66, 0.12, 1.0))
    steel = Material("machined_steel", rgba=(0.72, 0.74, 0.74, 1.0))

    # Root frame is the hip rotation center.  The whole mechanism stays close to
    # the Y=0 center plane, with all user-facing joints rotating about +Y.
    upper_housing = model.part("upper_housing")
    upper_housing.visual(
        Cylinder(radius=0.105, length=0.220),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="hip_barrel",
    )
    upper_housing.visual(
        Box((0.270, 0.175, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=dark_anodized,
        name="upper_shell",
    )
    upper_housing.visual(
        Box((0.210, 0.150, 0.050)),
        origin=Origin(xyz=(-0.010, 0.0, 0.190)),
        material=gunmetal,
        name="top_mount",
    )
    upper_housing.visual(
        Box((0.180, 0.135, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
        material=matte_black,
        name="front_controller_cap",
    )
    upper_housing.visual(
        Box((0.035, 0.145, 0.095)),
        origin=Origin(xyz=(-0.145, 0.0, 0.070)),
        material=warning_yellow,
        name="service_cover",
    )
    upper_housing.visual(
        Cylinder(radius=0.018, length=0.132),
        origin=Origin(xyz=(-0.030, 0.0, 0.230), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="mount_cross_pin",
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Cylinder(radius=0.085, length=0.060),
        origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hip_lug_outer",
    )
    thigh_link.visual(
        Cylinder(radius=0.085, length=0.060),
        origin=Origin(xyz=(0.0, -0.140, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hip_lug_inner",
    )
    thigh_link.visual(
        Box((0.075, 0.052, 0.205)),
        origin=Origin(xyz=(0.0, 0.140, -0.080)),
        material=gunmetal,
        name="hip_cheek_outer",
    )
    thigh_link.visual(
        Box((0.075, 0.052, 0.205)),
        origin=Origin(xyz=(0.0, -0.140, -0.080)),
        material=gunmetal,
        name="hip_cheek_inner",
    )
    thigh_link.visual(
        Box((0.080, 0.315, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=gunmetal,
        name="hip_yoke_bridge",
    )
    thigh_link.visual(
        Box((0.092, 0.082, 0.380)),
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        material=dark_anodized,
        name="thigh_spine",
    )
    thigh_link.visual(
        Box((0.120, 0.052, 0.300)),
        origin=Origin(xyz=(0.030, 0.065, -0.335)),
        material=matte_black,
        name="thigh_side_web_outer",
    )
    thigh_link.visual(
        Box((0.120, 0.052, 0.300)),
        origin=Origin(xyz=(0.030, -0.065, -0.335)),
        material=matte_black,
        name="thigh_side_web_inner",
    )
    thigh_link.visual(
        Cylinder(radius=0.074, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.550), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="knee_barrel",
    )
    thigh_link.visual(
        Box((0.120, 0.130, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.505)),
        material=dark_anodized,
        name="knee_neck",
    )
    thigh_link.visual(
        Cylinder(radius=0.017, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.550), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knee_pin_face",
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.115, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="knee_lug_outer",
    )
    shank_link.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="knee_lug_inner",
    )
    shank_link.visual(
        Box((0.066, 0.044, 0.175)),
        origin=Origin(xyz=(0.0, 0.115, -0.068)),
        material=gunmetal,
        name="knee_cheek_outer",
    )
    shank_link.visual(
        Box((0.066, 0.044, 0.175)),
        origin=Origin(xyz=(0.0, -0.115, -0.068)),
        material=gunmetal,
        name="knee_cheek_inner",
    )
    shank_link.visual(
        Box((0.070, 0.258, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=gunmetal,
        name="knee_yoke_bridge",
    )
    shank_link.visual(
        Box((0.074, 0.070, 0.350)),
        origin=Origin(xyz=(0.0, 0.0, -0.305)),
        material=dark_anodized,
        name="shank_spine",
    )
    shank_link.visual(
        Box((0.105, 0.044, 0.260)),
        origin=Origin(xyz=(-0.022, 0.055, -0.310)),
        material=matte_black,
        name="shank_side_web_outer",
    )
    shank_link.visual(
        Box((0.105, 0.044, 0.260)),
        origin=Origin(xyz=(-0.022, -0.055, -0.310)),
        material=matte_black,
        name="shank_side_web_inner",
    )
    shank_link.visual(
        Cylinder(radius=0.056, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.500), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="ankle_barrel",
    )
    shank_link.visual(
        Box((0.095, 0.106, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.460)),
        material=dark_anodized,
        name="ankle_neck",
    )
    shank_link.visual(
        Cylinder(radius=0.014, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, -0.500), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="ankle_pin_face",
    )

    ankle_foot = model.part("ankle_foot")
    ankle_foot.visual(
        Cylinder(radius=0.056, length=0.040),
        origin=Origin(xyz=(0.0, 0.100, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="ankle_lug_outer",
    )
    ankle_foot.visual(
        Cylinder(radius=0.056, length=0.040),
        origin=Origin(xyz=(0.0, -0.100, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="ankle_lug_inner",
    )
    ankle_foot.visual(
        Box((0.058, 0.036, 0.120)),
        origin=Origin(xyz=(0.0, 0.100, -0.050)),
        material=gunmetal,
        name="ankle_cheek_outer",
    )
    ankle_foot.visual(
        Box((0.058, 0.036, 0.120)),
        origin=Origin(xyz=(0.0, -0.100, -0.050)),
        material=gunmetal,
        name="ankle_cheek_inner",
    )
    ankle_foot.visual(
        Box((0.085, 0.230, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=gunmetal,
        name="ankle_yoke_bridge",
    )
    ankle_foot.visual(
        Box((0.360, 0.165, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, -0.140)),
        material=rubber,
        name="rubber_sole",
    )
    ankle_foot.visual(
        Box((0.240, 0.130, 0.030)),
        origin=Origin(xyz=(0.060, 0.0, -0.100)),
        material=matte_black,
        name="foot_top_plate",
    )
    ankle_foot.visual(
        Box((0.070, 0.145, 0.040)),
        origin=Origin(xyz=(-0.110, 0.0, -0.105)),
        material=warning_yellow,
        name="heel_pad_marker",
    )

    model.articulation(
        "hip_joint",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=3.2, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "knee_joint",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=3.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "ankle_joint",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=4.0, lower=-0.55, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip = object_model.get_articulation("hip_joint")
    knee = object_model.get_articulation("knee_joint")
    ankle = object_model.get_articulation("ankle_joint")
    upper = object_model.get_part("upper_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")

    ctx.check(
        "three lateral revolute joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
            for joint in (hip, knee, ankle)
        ),
        details=f"axes={[joint.axis for joint in (hip, knee, ankle)]}",
    )
    ctx.expect_origin_gap(thigh, shank, axis="z", min_gap=0.53, max_gap=0.57, name="thigh length sets knee spacing")
    ctx.expect_origin_gap(shank, foot, axis="z", min_gap=0.48, max_gap=0.52, name="shank length sets ankle spacing")
    ctx.expect_origin_distance(upper, foot, axes="y", max_dist=0.001, name="moving chain remains on midline")
    ctx.expect_contact(upper, thigh, elem_a="hip_barrel", elem_b="hip_lug_outer", contact_tol=0.002, name="outer hip lug bears on housing")
    ctx.expect_contact(thigh, shank, elem_a="knee_barrel", elem_b="knee_lug_outer", contact_tol=0.002, name="outer knee lug bears on thigh barrel")
    ctx.expect_contact(shank, foot, elem_a="ankle_barrel", elem_b="ankle_lug_outer", contact_tol=0.002, name="outer ankle lug bears on shank barrel")

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.0}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion moves foot in sagittal plane",
        rest_foot is not None
        and flexed_foot is not None
        and abs(flexed_foot[1] - rest_foot[1]) < 0.002
        and abs(flexed_foot[0] - rest_foot[0]) > 0.20,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
