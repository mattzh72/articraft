from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            return "/"
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd
_safe_getcwd()

ASSET_ROOT = Path("/tmp/articraft_scanner_assets")
ASSET_ROOT.mkdir(parents=True, exist_ok=True)
ASSETS = AssetContext(ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_flatbed_scanner", assets=ASSETS)

    body_dark = model.material("body_dark", rgba=(0.21, 0.23, 0.25, 1.0))
    body_mid = model.material("body_mid", rgba=(0.34, 0.36, 0.39, 1.0))
    lid_light = model.material("lid_light", rgba=(0.86, 0.87, 0.88, 1.0))
    lid_inner = model.material("lid_inner", rgba=(0.95, 0.96, 0.97, 1.0))
    glass = model.material("glass", rgba=(0.54, 0.70, 0.80, 0.35))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    accent = model.material("accent", rgba=(0.17, 0.53, 0.77, 1.0))
    light_trim = model.material("light_trim", rgba=(0.82, 0.84, 0.86, 1.0))

    base_w = 0.72
    base_d = 0.55
    base_h = 0.082
    bezel_t = 0.006
    platen_w = 0.458
    platen_d = 0.340
    platen_y = -0.030
    hinge_y = 0.215
    hinge_z = 0.090

    body = model.part("body")
    body.visual(
        Box((base_w, base_d, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
        material=body_dark,
        name="lower_shell",
    )
    body.visual(
        Box((0.120, 0.455, bezel_t)),
        origin=Origin(xyz=(-0.300, platen_y, base_h + bezel_t / 2.0)),
        material=body_mid,
        name="left_frame",
    )
    body.visual(
        Box((0.120, 0.455, bezel_t)),
        origin=Origin(xyz=(0.300, platen_y, base_h + bezel_t / 2.0)),
        material=body_mid,
        name="right_frame",
    )
    body.visual(
        Box((0.600, 0.060, bezel_t)),
        origin=Origin(xyz=(0.0, -0.238, base_h + bezel_t / 2.0)),
        material=body_mid,
        name="front_frame",
    )
    body.visual(
        Box((0.580, 0.060, bezel_t)),
        origin=Origin(xyz=(0.0, 0.170, base_h + bezel_t / 2.0)),
        material=body_mid,
        name="rear_frame",
    )
    body.visual(
        Box((platen_w, platen_d, 0.004)),
        origin=Origin(xyz=(0.0, platen_y, base_h + 0.002)),
        material=glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.018, 0.320, 0.001)),
        origin=Origin(xyz=(-0.220, platen_y, base_h + 0.0045)),
        material=light_trim,
        name="calibration_strip",
    )
    body.visual(
        Box((0.240, 0.062, 0.008)),
        origin=Origin(xyz=(0.180, -0.236, base_h + 0.004)),
        material=accent,
        name="control_panel",
    )
    body.visual(
        Box((0.024, 0.014, 0.018)),
        origin=Origin(xyz=(-0.343, 0.225, 0.091)),
        material=body_mid,
        name="hinge_block_left",
    )
    body.visual(
        Box((0.024, 0.014, 0.018)),
        origin=Origin(xyz=(0.343, 0.225, 0.091)),
        material=body_mid,
        name="hinge_block_right",
    )
    body.visual(
        Box((0.580, 0.015, 0.002)),
        origin=Origin(xyz=(0.0, -0.238, 0.085)),
        material=rubber,
        name="front_seal",
    )
    body.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_h)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
    )

    lid = model.part("lid")
    lid_w = 0.66
    lid_d = 0.46
    lid.visual(
        Box((0.490, 0.350, 0.002)),
        origin=Origin(xyz=(0.0, -0.245, -0.001)),
        material=lid_inner,
        name="lid_pad",
    )
    lid.visual(
        Box((lid_w, lid_d, 0.018)),
        origin=Origin(xyz=(0.0, -0.240, 0.009)),
        material=lid_light,
        name="lid_shell",
    )
    lid.visual(
        Box((0.580, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, -0.468, -0.002)),
        material=rubber,
        name="front_bumper",
    )
    lid.visual(
        Box((0.024, 0.150, 0.060)),
        origin=Origin(xyz=(-0.322, -0.075, 0.049)),
        material=body_mid,
        name="adf_side_left",
    )
    lid.visual(
        Box((0.024, 0.150, 0.060)),
        origin=Origin(xyz=(0.322, -0.075, 0.049)),
        material=body_mid,
        name="adf_side_right",
    )
    lid.visual(
        Box((0.620, 0.120, 0.006)),
        origin=Origin(xyz=(0.0, -0.095, 0.021)),
        material=body_mid,
        name="adf_floor",
    )
    lid.visual(
        Box((0.620, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, -0.152, 0.038)),
        material=body_mid,
        name="adf_front_bridge",
    )
    lid.visual(
        Box((0.620, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.021, 0.038)),
        material=body_mid,
        name="adf_rear_spine",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.090)),
        mass=5.6,
        origin=Origin(xyz=(0.0, -0.180, 0.030)),
    )

    adf_cover = model.part("adf_cover")
    adf_cover.visual(
        Box((0.620, 0.112, 0.008)),
        origin=Origin(xyz=(0.0, -0.091, 0.003)),
        material=lid_light,
        name="cover_top",
    )
    adf_cover.visual(
        Box((0.620, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.146, -0.009)),
        material=lid_light,
        name="cover_front_lip",
    )
    adf_cover.visual(
        Box((0.620, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.040, -0.004)),
        material=body_mid,
        name="cover_rear_spine",
    )
    adf_cover.inertial = Inertial.from_geometry(
        Box((0.620, 0.155, 0.028)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.075, -0.008)),
    )

    feed_roller = model.part("feed_roller")
    feed_roller.visual(
        Cylinder(radius=0.016, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="roller_body",
    )
    feed_roller.visual(
        Cylinder(radius=0.0045, length=0.035),
        origin=Origin(xyz=(-0.2925, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_trim,
        name="shaft_left",
    )
    feed_roller.visual(
        Cylinder(radius=0.0045, length=0.035),
        origin=Origin(xyz=(0.2925, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_trim,
        name="shaft_right",
    )
    feed_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.550),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    pinch_roller = model.part("pinch_roller")
    pinch_roller.visual(
        Cylinder(radius=0.010, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="roller_body",
    )
    pinch_roller.visual(
        Cylinder(radius=0.0040, length=0.050),
        origin=Origin(xyz=(-0.285, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_trim,
        name="shaft_left",
    )
    pinch_roller.visual(
        Cylinder(radius=0.0040, length=0.050),
        origin=Origin(xyz=(0.285, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_trim,
        name="shaft_right",
    )
    pinch_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.520),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "lid_to_adf_cover",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=adf_cover,
        origin=Origin(xyz=(0.0, 0.030, 0.087)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "lid_to_feed_roller",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=feed_roller,
        origin=Origin(xyz=(0.0, -0.050, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "lid_to_pinch_roller",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=pinch_roller,
        origin=Origin(xyz=(0.0, -0.095, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    adf_cover = object_model.get_part("adf_cover")
    adf_cover_hinge = object_model.get_articulation("lid_to_adf_cover")
    feed_roller = object_model.get_part("feed_roller")
    feed_roller_joint = object_model.get_articulation("lid_to_feed_roller")
    pinch_roller = object_model.get_part("pinch_roller")
    pinch_roller_joint = object_model.get_articulation("lid_to_pinch_roller")

    platen_glass = body.get_visual("platen_glass")
    front_frame = body.get_visual("front_frame")
    rear_frame = body.get_visual("rear_frame")
    front_seal = body.get_visual("front_seal")

    lid_pad = lid.get_visual("lid_pad")
    lid_shell = lid.get_visual("lid_shell")
    front_bumper = lid.get_visual("front_bumper")
    adf_side_left = lid.get_visual("adf_side_left")
    adf_side_right = lid.get_visual("adf_side_right")
    adf_floor = lid.get_visual("adf_floor")
    adf_front_bridge = lid.get_visual("adf_front_bridge")
    cover_top = adf_cover.get_visual("cover_top")
    cover_front_lip = adf_cover.get_visual("cover_front_lip")
    feed_body = feed_roller.get_visual("roller_body")
    pinch_body = pinch_roller.get_visual("roller_body")
    feed_shaft_left = feed_roller.get_visual("shaft_left")
    feed_shaft_right = feed_roller.get_visual("shaft_right")
    pinch_shaft_left = pinch_roller.get_visual("shaft_left")
    pinch_shaft_right = pinch_roller.get_visual("shaft_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "lid_hinge_axis_and_limits",
        lid_hinge.axis == (-1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and 1.20 <= lid_hinge.motion_limits.upper <= 1.40,
        f"unexpected lid hinge configuration: axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "adf_cover_hinge_axis_and_limits",
        adf_cover_hinge.axis == (-1.0, 0.0, 0.0)
        and adf_cover_hinge.motion_limits is not None
        and adf_cover_hinge.motion_limits.lower == 0.0
        and adf_cover_hinge.motion_limits.upper is not None
        and 0.90 <= adf_cover_hinge.motion_limits.upper <= 1.20,
        f"unexpected adf hinge configuration: axis={adf_cover_hinge.axis}, limits={adf_cover_hinge.motion_limits}",
    )
    ctx.check(
        "roller_joint_types",
        feed_roller_joint.joint_type == ArticulationType.CONTINUOUS
        and pinch_roller_joint.joint_type == ArticulationType.CONTINUOUS
        and feed_roller_joint.axis == (1.0, 0.0, 0.0)
        and pinch_roller_joint.axis == (1.0, 0.0, 0.0),
        f"roller joints should be continuous x-axis rotations, got {feed_roller_joint.joint_type}/{pinch_roller_joint.joint_type}",
    )

    body_aabb = ctx.part_world_aabb(body)
    glass_aabb = ctx.part_element_world_aabb(body, elem=platen_glass)
    if body_aabb is not None:
        body_size = tuple(body_aabb[1][i] - body_aabb[0][i] for i in range(3))
        ctx.check(
            "scanner_body_large_format_size",
            0.68 <= body_size[0] <= 0.76 and 0.50 <= body_size[1] <= 0.58 and 0.08 <= body_size[2] <= 0.10,
            f"body size should read as large-format scanner, got {body_size}",
        )
    if glass_aabb is not None:
        glass_size = tuple(glass_aabb[1][i] - glass_aabb[0][i] for i in range(3))
        ctx.check(
            "a3_platen_footprint",
            glass_size[0] >= 0.42 and glass_size[1] >= 0.297,
            f"platen should fit A3 media, got {glass_size}",
        )

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.24)
    ctx.expect_within(
        body,
        lid,
        axes="xy",
        inner_elem=platen_glass,
        outer_elem=lid_pad,
    )
    ctx.expect_contact(lid, body, elem_a=front_bumper, elem_b=front_seal)
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=lid_shell,
        negative_elem=rear_frame,
    )
    ctx.expect_gap(
        adf_cover,
        lid,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem=cover_front_lip,
        negative_elem=adf_front_bridge,
    )
    ctx.expect_overlap(adf_cover, lid, axes="x", min_overlap=0.55, elem_a=cover_top, elem_b=adf_floor)
    ctx.expect_contact(
        feed_roller,
        lid,
        elem_a=feed_shaft_left,
        elem_b=adf_side_left,
    )
    ctx.expect_contact(
        feed_roller,
        lid,
        elem_a=feed_shaft_right,
        elem_b=adf_side_right,
    )
    ctx.expect_contact(
        pinch_roller,
        lid,
        elem_a=pinch_shaft_left,
        elem_b=adf_side_left,
    )
    ctx.expect_contact(
        pinch_roller,
        lid,
        elem_a=pinch_shaft_right,
        elem_b=adf_side_right,
    )
    ctx.expect_gap(
        feed_roller,
        lid,
        axis="z",
        min_gap=0.003,
        positive_elem=feed_body,
        negative_elem=adf_floor,
    )
    ctx.expect_gap(
        adf_cover,
        feed_roller,
        axis="z",
        min_gap=0.005,
        positive_elem=cover_front_lip,
        negative_elem=feed_body,
    )
    ctx.expect_gap(
        feed_roller,
        pinch_roller,
        axis="y",
        min_gap=0.010,
        positive_elem=feed_body,
        negative_elem=pinch_body,
    )

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.20,
            positive_elem=front_bumper,
            negative_elem=front_seal,
        )
        ctx.expect_gap(
            adf_cover,
            body,
            axis="z",
            min_gap=0.03,
            positive_elem=cover_front_lip,
            negative_elem=rear_frame,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_no_floating")
    with ctx.pose({adf_cover_hinge: adf_cover_hinge.motion_limits.upper}):
        ctx.expect_gap(
            adf_cover,
            lid,
            axis="z",
            min_gap=0.055,
            positive_elem=cover_front_lip,
            negative_elem=adf_front_bridge,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="adf_cover_open_no_overlap")
        ctx.fail_if_isolated_parts(name="adf_cover_open_no_floating")
    with ctx.pose({feed_roller_joint: math.pi * 0.75, pinch_roller_joint: math.pi * 1.25}):
        ctx.expect_contact(feed_roller, lid, elem_a=feed_shaft_left, elem_b=adf_side_left)
        ctx.expect_contact(feed_roller, lid, elem_a=feed_shaft_right, elem_b=adf_side_right)
        ctx.expect_contact(pinch_roller, lid, elem_a=pinch_shaft_left, elem_b=adf_side_left)
        ctx.expect_contact(pinch_roller, lid, elem_a=pinch_shaft_right, elem_b=adf_side_right)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
