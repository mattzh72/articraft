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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_ring_light")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.20, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.95, 0.96, 0.97, 0.92))
    cool_gray = model.material("cool_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.14, 0.15, 0.16, 1.0))

    ring_shell_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.128, tube=0.022, radial_segments=24, tubular_segments=96),
        "ring_light_shell_v4",
    )
    diffuser_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.128, tube=0.016, radial_segments=20, tubular_segments=96),
        "ring_light_diffuser_v4",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.130, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="weight_disc",
    )
    base.visual(
        Cylinder(radius=0.102, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_black,
        name="upper_cover",
    )
    base.visual(
        Box((0.008, 0.004, 0.260)),
        origin=Origin(xyz=(0.0, 0.012, 0.160)),
        material=satin_black,
        name="sleeve_front",
    )
    base.visual(
        Box((0.008, 0.004, 0.260)),
        origin=Origin(xyz=(0.0, -0.012, 0.160)),
        material=satin_black,
        name="sleeve_back",
    )
    base.visual(
        Box((0.004, 0.016, 0.260)),
        origin=Origin(xyz=(0.012, 0.0, 0.160)),
        material=satin_black,
        name="sleeve_right",
    )
    base.visual(
        Box((0.004, 0.016, 0.260)),
        origin=Origin(xyz=(-0.012, 0.0, 0.160)),
        material=satin_black,
        name="sleeve_left",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.026, 0.0, 0.044), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="height_lock_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.278)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.010, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cool_gray,
        name="telescoping_tube",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.520),
        mass=0.45,
        origin=Origin(),
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.180,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="mount_socket",
    )
    yoke.visual(
        Box((0.022, 0.016, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=satin_black,
        name="yoke_stem",
    )
    yoke.visual(
        Box((0.336, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_black,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.012, 0.016, 0.090)),
        origin=Origin(xyz=(-0.168, 0.0, 0.095)),
        material=satin_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.012, 0.016, 0.090)),
        origin=Origin(xyz=(0.168, 0.0, 0.095)),
        material=satin_black,
        name="right_arm",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.348, 0.016, 0.140)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    model.articulation(
        "mast_to_yoke",
        ArticulationType.FIXED,
        parent=upper_mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )

    light_head = model.part("light_head")
    light_head.visual(
        ring_shell_mesh,
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="ring_housing",
    )
    light_head.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    light_head.visual(
        Cylinder(radius=0.007, length=0.324),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cool_gray,
        name="pivot_axle",
    )
    light_head.visual(
        Box((0.016, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=satin_black,
        name="center_spine",
    )
    light_head.visual(
        Box((0.028, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=satin_black,
        name="clamp_mount_hub",
    )
    light_head.visual(
        Box((0.224, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=satin_black,
        name="left_right_spoke",
    )
    light_head.inertial = Inertial.from_geometry(
        Box((0.310, 0.120, 0.310)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.90,
            upper=1.00,
        ),
    )

    clamp_frame = model.part("clamp_frame")
    clamp_frame.visual(
        Box((0.018, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=satin_black,
        name="mount_block",
    )
    clamp_frame.visual(
        Box((0.010, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=satin_black,
        name="mount_tab",
    )
    clamp_frame.visual(
        Box((0.020, 0.012, 0.100)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=satin_black,
        name="back_plate",
    )
    clamp_frame.visual(
        Box((0.136, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, 0.044)),
        material=satin_black,
        name="top_rail",
    )
    clamp_frame.visual(
        Box((0.136, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, -0.044)),
        material=satin_black,
        name="bottom_rail",
    )
    clamp_frame.inertial = Inertial.from_geometry(
        Box((0.136, 0.040, 0.100)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    model.articulation(
        "head_to_clamp",
        ArticulationType.FIXED,
        parent=light_head,
        child=clamp_frame,
        origin=Origin(xyz=(0.0, 0.082, 0.0)),
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        Box((0.012, 0.008, 0.076)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=satin_black,
        name="jaw",
    )
    left_jaw.visual(
        Box((0.002, 0.006, 0.064)),
        origin=Origin(xyz=(0.007, 0.028, 0.0)),
        material=soft_pad,
        name="jaw_pad",
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((0.012, 0.008, 0.076)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        Box((0.012, 0.008, 0.076)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=satin_black,
        name="jaw",
    )
    right_jaw.visual(
        Box((0.002, 0.006, 0.064)),
        origin=Origin(xyz=(-0.007, 0.028, 0.0)),
        material=soft_pad,
        name="jaw_pad",
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((0.012, 0.008, 0.076)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
    )

    model.articulation(
        "left_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=clamp_frame,
        child=left_jaw,
        origin=Origin(xyz=(-0.042, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.10,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "right_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=clamp_frame,
        child=right_jaw,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.10,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    upper_mast = object_model.get_part("upper_mast")
    yoke = object_model.get_part("yoke")
    light_head = object_model.get_part("light_head")
    clamp_frame = object_model.get_part("clamp_frame")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")

    mast_slide = object_model.get_articulation("mast_slide")
    head_tilt = object_model.get_articulation("head_tilt")
    left_jaw_slide = object_model.get_articulation("left_jaw_slide")
    right_jaw_slide = object_model.get_articulation("right_jaw_slide")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        clamp_frame,
        light_head,
        reason="The phone clamp uses a keyed mounting tab captured inside the head's central hub.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper_mast, base, name="mast_contacts_sleeve")
    ctx.expect_contact(yoke, upper_mast, name="yoke_mounts_to_mast")
    ctx.expect_contact(light_head, yoke, name="head_supported_by_yoke")
    ctx.expect_contact(clamp_frame, light_head, name="clamp_mounts_to_head")
    ctx.expect_contact(left_jaw, clamp_frame, name="left_jaw_guided_by_frame")
    ctx.expect_contact(right_jaw, clamp_frame, name="right_jaw_guided_by_frame")

    ctx.expect_origin_distance(clamp_frame, light_head, axes="xz", max_dist=0.001)
    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.070,
        max_gap=0.074,
        name="rest_phone_gap",
    )
    ctx.expect_overlap(clamp_frame, light_head, axes="z", min_overlap=0.080)

    ring_aabb = ctx.part_element_world_aabb(light_head, elem="ring_housing")
    if ring_aabb is None:
        ctx.fail("ring_housing_present", "ring_housing visual AABB could not be resolved")
    else:
        ring_dx = ring_aabb[1][0] - ring_aabb[0][0]
        ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
        ctx.check(
            "ring_head_proportions",
            0.290 <= ring_dx <= 0.305 and 0.290 <= ring_dz <= 0.305,
            f"ring housing extents were ({ring_dx:.4f}, {ring_dz:.4f}) m; expected ~0.30 m outer diameter",
        )

    mast_rest = ctx.part_world_position(upper_mast)
    if mast_rest is None:
        ctx.fail("mast_rest_position", "upper mast rest position was unavailable")
    else:
        with ctx.pose({mast_slide: 0.180}):
            mast_extended = ctx.part_world_position(upper_mast)
            if mast_extended is None:
                ctx.fail("mast_extended_position", "upper mast extended position was unavailable")
            else:
                ctx.check(
                    "mast_slides_vertically",
                    abs(mast_extended[0] - mast_rest[0]) < 1e-6
                    and abs(mast_extended[1] - mast_rest[1]) < 1e-6
                    and mast_extended[2] > mast_rest[2] + 0.175,
                    f"mast rest={mast_rest}, extended={mast_extended}",
                )
            ctx.expect_contact(upper_mast, base, name="mast_extended_still_guided")

    ring_rest = ctx.part_element_world_aabb(light_head, elem="ring_housing")
    with ctx.pose({head_tilt: 0.85}):
        ring_tilted = ctx.part_element_world_aabb(light_head, elem="ring_housing")
        if ring_rest is None or ring_tilted is None:
            ctx.fail("tilt_pose_ring_aabb", "ring housing AABB unavailable for tilt test")
        else:
            rest_y = ring_rest[1][1] - ring_rest[0][1]
            tilted_y = ring_tilted[1][1] - ring_tilted[0][1]
            ctx.check(
                "head_tilts_about_horizontal_axis",
                tilted_y > rest_y + 0.180,
                f"ring y extent rest={rest_y:.4f}, tilted={tilted_y:.4f}",
            )
        ctx.expect_contact(light_head, yoke, name="head_tilt_still_supported")

    left_rest = ctx.part_world_position(left_jaw)
    right_rest = ctx.part_world_position(right_jaw)
    if left_rest is None or right_rest is None:
        ctx.fail("jaw_rest_positions", "jaw rest positions were unavailable")
    else:
        with ctx.pose({left_jaw_slide: 0.018, right_jaw_slide: 0.018}):
            left_open = ctx.part_world_position(left_jaw)
            right_open = ctx.part_world_position(right_jaw)
            if left_open is None or right_open is None:
                ctx.fail("jaw_open_positions", "jaw open positions were unavailable")
            else:
                ctx.check(
                    "jaws_open_symmetrically",
                    left_open[0] < left_rest[0] - 0.017 and right_open[0] > right_rest[0] + 0.017,
                    f"left rest/open={left_rest}/{left_open}, right rest/open={right_rest}/{right_open}",
                )
            ctx.expect_gap(
                right_jaw,
                left_jaw,
                axis="x",
                positive_elem="jaw",
                negative_elem="jaw",
                min_gap=0.106,
                max_gap=0.110,
                name="open_phone_gap",
            )
            ctx.expect_contact(left_jaw, clamp_frame, name="left_jaw_open_still_guided")
            ctx.expect_contact(right_jaw, clamp_frame, name="right_jaw_open_still_guided")

    for joint_name in ("mast_slide", "head_tilt", "left_jaw_slide", "right_jaw_slide"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
