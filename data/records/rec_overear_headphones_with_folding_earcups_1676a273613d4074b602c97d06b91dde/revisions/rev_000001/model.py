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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphones")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.17, 0.19, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.07, 0.07, 0.08, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.34, 0.35, 0.38, 1.0))
    dark_mesh = model.material("dark_mesh", rgba=(0.16, 0.16, 0.17, 1.0))

    headband_outer_path = [
        (-0.066, 0.0, 0.170),
        (-0.051, 0.0, 0.191),
        (-0.028, 0.0, 0.210),
        (0.0, 0.0, 0.222),
        (0.028, 0.0, 0.210),
        (0.051, 0.0, 0.191),
        (0.066, 0.0, 0.170),
    ]
    headband_pad_path = [
        (-0.050, 0.0, 0.175),
        (-0.036, 0.0, 0.190),
        (-0.015, 0.0, 0.201),
        (0.0, 0.0, 0.206),
        (0.015, 0.0, 0.201),
        (0.036, 0.0, 0.190),
        (0.050, 0.0, 0.175),
    ]

    headband_outer_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            headband_outer_path,
            profile=rounded_rect_profile(0.040, 0.016, radius=0.006, corner_segments=8),
            samples_per_segment=24,
            cap_profile=True,
        ),
        "headband_outer_v3",
    )
    headband_pad_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            headband_pad_path,
            profile=rounded_rect_profile(0.030, 0.012, radius=0.004, corner_segments=8),
            samples_per_segment=24,
            cap_profile=True,
        ),
        "headband_pad_v3",
    )

    earcup_shell_geom = (
        ExtrudeGeometry(
            rounded_rect_profile(0.092, 0.076, radius=0.015, corner_segments=8),
            0.046,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
    )
    earcup_pad_geom = (
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.074, 0.060, radius=0.015, corner_segments=8),
            [rounded_rect_profile(0.046, 0.034, radius=0.009, corner_segments=8)],
            0.020,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
    )
    earcup_grille_geom = (
        ExtrudeGeometry(
            rounded_rect_profile(0.046, 0.034, radius=0.009, corner_segments=8),
            0.003,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
    )
    earcup_backplate_geom = (
        ExtrudeGeometry(
            rounded_rect_profile(0.068, 0.054, radius=0.011, corner_segments=8),
            0.005,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
    )

    earcup_shell_mesh = mesh_from_geometry(earcup_shell_geom, "earcup_shell_v3")
    earcup_pad_mesh = mesh_from_geometry(earcup_pad_geom, "earcup_pad_v3")
    earcup_grille_mesh = mesh_from_geometry(earcup_grille_geom, "earcup_grille_v3")
    earcup_backplate_mesh = mesh_from_geometry(earcup_backplate_geom, "earcup_backplate_v3")

    headband = model.part("headband")
    headband.visual(headband_outer_mesh, material=matte_black, name="outer_band")
    headband.visual(headband_pad_mesh, material=cushion_black, name="inner_pad")
    headband.visual(
        Box((0.060, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.209)),
        material=cushion_black,
        name="pad_mount",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        hinge_x = side_sign * 0.082
        connector_x = side_sign * 0.078
        headband.visual(
            Box((0.028, 0.008, 0.014)),
            origin=Origin(xyz=(connector_x, -0.011, 0.166)),
            material=graphite,
            name=f"{side_name}_side_rail_0",
        )
        headband.visual(
            Box((0.028, 0.008, 0.014)),
            origin=Origin(xyz=(connector_x, 0.011, 0.166)),
            material=graphite,
            name=f"{side_name}_side_rail_1",
        )
        headband.visual(
            Box((0.018, 0.008, 0.022)),
            origin=Origin(xyz=(hinge_x, -0.011, 0.158)),
            material=graphite,
            name=f"{side_name}_hinge_web_0",
        )
        headband.visual(
            Box((0.018, 0.008, 0.022)),
            origin=Origin(xyz=(hinge_x, 0.011, 0.158)),
            material=graphite,
            name=f"{side_name}_hinge_web_1",
        )
        for idx, hinge_y in enumerate((-0.011, 0.011)):
            headband.visual(
                Cylinder(radius=0.0055, length=0.008),
                origin=Origin(
                    xyz=(hinge_x, hinge_y, 0.158),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=gunmetal,
                name=f"{side_name}_hinge_ear_{idx}",
            )
    headband.inertial = Inertial.from_geometry(
        Box((0.19, 0.05, 0.24)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    def add_yoke(name: str) -> None:
        yoke = model.part(name)
        yoke.visual(
            Cylinder(radius=0.0054, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.014, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=graphite,
            name="hinge_body",
        )
        yoke.visual(
            Box((0.010, 0.094, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=graphite,
            name="fork_crown",
        )
        yoke.visual(
            Box((0.010, 0.010, 0.070)),
            origin=Origin(xyz=(0.0, 0.050, -0.055)),
            material=graphite,
            name="front_arm",
        )
        yoke.visual(
            Box((0.010, 0.010, 0.070)),
            origin=Origin(xyz=(0.0, -0.050, -0.055)),
            material=graphite,
            name="rear_arm",
        )
        yoke.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(
                xyz=(0.0, 0.049, -0.074),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gunmetal,
            name="front_axle_block",
        )
        yoke.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(
                xyz=(0.0, -0.049, -0.074),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gunmetal,
            name="rear_axle_block",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.024, 0.12, 0.10)),
            mass=0.11,
            origin=Origin(xyz=(0.0, 0.0, -0.048)),
        )

    def add_earcup(name: str, sign: float) -> None:
        earcup = model.part(name)
        earcup.visual(earcup_shell_mesh, material=matte_black, name="shell")
        earcup.visual(
            earcup_backplate_mesh,
            origin=Origin(xyz=(-sign * 0.0205, 0.0, 0.0)),
            material=gunmetal,
            name="outer_backplate",
        )
        earcup.visual(
            earcup_pad_mesh,
            origin=Origin(xyz=(sign * 0.031, 0.0, 0.0)),
            material=cushion_black,
            name="pad_ring",
        )
        earcup.visual(
            earcup_grille_mesh,
            origin=Origin(xyz=(sign * 0.0215, 0.0, 0.0)),
            material=dark_mesh,
            name="speaker_grille",
        )
        earcup.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="outer_yoke_boss",
        )
        earcup.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="inner_yoke_boss",
        )
        earcup.visual(
            Cylinder(radius=0.004, length=0.014),
            origin=Origin(
                xyz=(sign * 0.010, 0.0, -0.046),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gunmetal,
            name="bottom_port",
        )
        earcup.inertial = Inertial.from_geometry(
            Box((0.05, 0.10, 0.095)),
            mass=0.20,
        )

    add_yoke("left_yoke")
    add_yoke("right_yoke")
    add_earcup("left_earcup", sign=1.0)
    add_earcup("right_earcup", sign=-1.0)

    left_yoke = model.get_part("left_yoke")
    right_yoke = model.get_part("right_yoke")
    left_earcup = model.get_part("left_earcup")
    right_earcup = model.get_part("right_earcup")

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.082, 0.0, 0.158)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.082, 0.0, 0.158)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "left_yoke_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-0.30,
            upper=0.60,
        ),
    )
    model.articulation(
        "right_yoke_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-0.30,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def aabb_dims(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_yoke_swivel = object_model.get_articulation("left_yoke_swivel")
    right_yoke_swivel = object_model.get_articulation("right_yoke_swivel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=32,
        name="articulation_clearance_sweep",
    )

    ctx.expect_contact(headband, left_yoke, name="left_fold_contact")
    ctx.expect_contact(headband, right_yoke, name="right_fold_contact")
    ctx.expect_contact(left_yoke, left_earcup, name="left_yoke_contact")
    ctx.expect_contact(right_yoke, right_earcup, name="right_yoke_contact")
    ctx.expect_origin_gap(
        right_earcup,
        left_earcup,
        axis="x",
        min_gap=0.14,
        max_gap=0.21,
        name="earcup_spacing",
    )

    headband_aabb = ctx.part_world_aabb(headband)
    left_earcup_aabb = ctx.part_world_aabb(left_earcup)
    right_earcup_aabb = ctx.part_world_aabb(right_earcup)
    left_yoke_aabb = ctx.part_world_aabb(left_yoke)
    if (
        headband_aabb is None
        or left_earcup_aabb is None
        or right_earcup_aabb is None
        or left_yoke_aabb is None
    ):
        ctx.fail("aabb_availability", "Expected part AABBs for headband, yoke, and earcups.")
        return ctx.report()

    left_dims = aabb_dims(left_earcup_aabb)
    yoke_dims = aabb_dims(left_yoke_aabb)
    overall_span = right_earcup_aabb[1][0] - left_earcup_aabb[0][0]
    band_to_cup_gap = headband_aabb[0][2] - max(left_earcup_aabb[1][2], right_earcup_aabb[1][2])

    ctx.check(
        "thick_earcup_proportions",
        0.040 <= left_dims[0] <= 0.070
        and 0.070 <= left_dims[1] <= 0.100
        and 0.085 <= left_dims[2] <= 0.110,
        details=f"left earcup dims were {left_dims}",
    )
    ctx.check(
        "broad_fork_yoke_proportions",
        yoke_dims[1] >= 0.070 and yoke_dims[2] >= 0.080,
        details=f"left yoke dims were {yoke_dims}",
    )
    ctx.check(
        "dj_headphone_overall_span",
        0.17 <= overall_span <= 0.25,
        details=f"overall headphone span was {overall_span:.4f} m",
    )
    ctx.check(
        "headband_above_earcups",
        0.002 <= band_to_cup_gap <= 0.030,
        details=f"headband-to-earcup top gap was {band_to_cup_gap:.4f} m",
    )

    left_rest = ctx.part_world_position(left_earcup)
    right_rest = ctx.part_world_position(right_earcup)
    if left_rest is None or right_rest is None:
        ctx.fail("rest_positions_available", "Expected earcup world positions in rest pose.")
        return ctx.report()

    left_port_rest_aabb = ctx.part_element_world_aabb(left_earcup, elem="bottom_port")
    right_port_rest_aabb = ctx.part_element_world_aabb(right_earcup, elem="bottom_port")
    if left_port_rest_aabb is None or right_port_rest_aabb is None:
        ctx.fail("bottom_port_aabbs", "Expected bottom_port element AABBs on both earcups.")
        return ctx.report()

    left_port_rest = aabb_center(left_port_rest_aabb)
    right_port_rest = aabb_center(right_port_rest_aabb)

    left_fold_limits = left_fold.motion_limits
    right_fold_limits = right_fold.motion_limits
    left_swivel_limits = left_yoke_swivel.motion_limits
    right_swivel_limits = right_yoke_swivel.motion_limits

    if left_fold_limits is not None and left_fold_limits.upper is not None:
        with ctx.pose({left_fold: left_fold_limits.upper}):
            left_folded = ctx.part_world_position(left_earcup)
            if left_folded is None:
                ctx.fail("left_fold_pose_position", "Missing left earcup position at fold upper limit.")
            else:
                ctx.check(
                    "left_fold_moves_inward_and_up",
                    left_folded[0] > left_rest[0] + 0.035 and left_folded[2] > left_rest[2] + 0.020,
                    details=f"left rest={left_rest}, folded={left_folded}",
                )
            ctx.expect_contact(headband, left_yoke, name="left_fold_contact_upper")
            ctx.expect_contact(left_yoke, left_earcup, name="left_earcup_contact_upper")

    if right_fold_limits is not None and right_fold_limits.upper is not None:
        with ctx.pose({right_fold: right_fold_limits.upper}):
            right_folded = ctx.part_world_position(right_earcup)
            if right_folded is None:
                ctx.fail("right_fold_pose_position", "Missing right earcup position at fold upper limit.")
            else:
                ctx.check(
                    "right_fold_moves_inward_and_up",
                    right_folded[0] < right_rest[0] - 0.035 and right_folded[2] > right_rest[2] + 0.020,
                    details=f"right rest={right_rest}, folded={right_folded}",
                )
            ctx.expect_contact(headband, right_yoke, name="right_fold_contact_upper")
            ctx.expect_contact(right_yoke, right_earcup, name="right_earcup_contact_upper")

    if left_swivel_limits is not None and left_swivel_limits.upper is not None:
        with ctx.pose({left_yoke_swivel: left_swivel_limits.upper}):
            left_port_pose_aabb = ctx.part_element_world_aabb(left_earcup, elem="bottom_port")
            if left_port_pose_aabb is None:
                ctx.fail("left_port_pose_aabb", "Missing left bottom_port AABB at swivel upper limit.")
            else:
                left_port_pose = aabb_center(left_port_pose_aabb)
                ctx.check(
                    "left_earcup_rotates_on_yoke_axis",
                    abs(left_port_pose[1] - left_port_rest[1]) <= 0.002
                    and abs(left_port_pose[0] - left_port_rest[0]) >= 0.020
                    and abs(left_port_pose[2] - left_port_rest[2]) >= 0.008,
                    details=f"left bottom_port rest={left_port_rest}, posed={left_port_pose}",
                )
            ctx.expect_contact(left_yoke, left_earcup, name="left_swivel_contact_upper")

    if right_swivel_limits is not None and right_swivel_limits.upper is not None:
        with ctx.pose({right_yoke_swivel: right_swivel_limits.upper}):
            right_port_pose_aabb = ctx.part_element_world_aabb(right_earcup, elem="bottom_port")
            if right_port_pose_aabb is None:
                ctx.fail("right_port_pose_aabb", "Missing right bottom_port AABB at swivel upper limit.")
            else:
                right_port_pose = aabb_center(right_port_pose_aabb)
                ctx.check(
                    "right_earcup_rotates_on_yoke_axis",
                    abs(right_port_pose[1] - right_port_rest[1]) <= 0.002
                    and abs(right_port_pose[0] - right_port_rest[0]) >= 0.020
                    and abs(right_port_pose[2] - right_port_rest[2]) >= 0.008,
                    details=f"right bottom_port rest={right_port_rest}, posed={right_port_pose}",
                )
            ctx.expect_contact(right_yoke, right_earcup, name="right_swivel_contact_upper")

    for articulation in (left_fold, right_fold, left_yoke_swivel, right_yoke_swivel):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
