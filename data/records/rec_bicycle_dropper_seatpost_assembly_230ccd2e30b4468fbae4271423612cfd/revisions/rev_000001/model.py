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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 64,
    centered: bool = True,
):
    if centered:
        z0 = -0.5 * length
        z1 = 0.5 * length
    else:
        z0 = 0.0
        z1 = length
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
    )
    return mesh_from_geometry(geom, name)


def _solid_cylinder_mesh(
    name: str,
    *,
    radius: float,
    length: float,
    segments: int = 48,
):
    geom = CylinderGeometry(radius, length, radial_segments=segments, closed=True)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost")

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.68, 0.70, 0.73, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.06, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.62, 0.63, 0.66, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    rail_black = model.material("rail_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body_radius = 0.0158
    body_inner_radius = 0.0142
    body_length = 0.285
    binder_length = 0.024
    binder_center_z = 0.236
    post_radius = 0.0130
    post_inner_radius = 0.0112
    post_length = 0.320
    post_center_local_z = -0.060
    drop_travel = 0.150
    head_mount_z = 0.100

    lower_body_shell = _shell_mesh(
        "lower_body_shell",
        outer_radius=body_radius,
        inner_radius=body_inner_radius,
        length=body_length,
        centered=True,
    )
    upper_bushing_shell = _shell_mesh(
        "upper_bushing_shell",
        outer_radius=body_inner_radius,
        inner_radius=post_radius,
        length=0.030,
        centered=True,
    )
    binder_ring_shell = _shell_mesh(
        "binder_ring_shell",
        outer_radius=0.0205,
        inner_radius=body_radius,
        length=binder_length,
        centered=True,
    )
    qr_pivot_pin_mesh = _solid_cylinder_mesh(
        "qr_pivot_pin",
        radius=0.0028,
        length=0.006,
    )
    stanchion_shell = _shell_mesh(
        "dropper_stanchion_shell",
        outer_radius=post_radius,
        inner_radius=post_inner_radius,
        length=post_length,
        centered=True,
    )
    tilt_axle_mesh = _solid_cylinder_mesh(
        "saddle_tilt_axle",
        radius=0.0045,
        length=0.016,
    )
    lever_barrel_shell = _shell_mesh(
        "qr_lever_barrel",
        outer_radius=0.0048,
        inner_radius=0.0028,
        length=0.006,
        centered=True,
    )
    tilt_barrel_shell = _shell_mesh(
        "saddle_tilt_barrel",
        outer_radius=0.0070,
        inner_radius=0.0045,
        length=0.016,
        centered=True,
    )

    lower_body = model.part("lower_body")
    lower_body.visual(
        lower_body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * body_length)),
        material=body_black,
        name="outer_tube",
    )
    lower_body.visual(
        upper_bushing_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=seal_black,
        name="upper_bushing",
    )
    lower_body.visual(
        Cylinder(radius=0.0174, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=seal_black,
        name="base_head",
    )
    lower_body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(
            xyz=(0.0, -body_radius - 0.0045, 0.040),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_steel,
        name="cable_actuator_stub",
    )
    lower_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0172, length=body_length),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * body_length)),
    )

    binder_ring = model.part("binder_ring")
    binder_ring.visual(
        binder_ring_shell,
        material=forged_alloy,
        name="collar_band",
    )
    binder_ring.visual(
        Box((0.016, 0.008, 0.020)),
        origin=Origin(xyz=(0.0255, 0.0060, 0.0)),
        material=forged_alloy,
        name="upper_ear",
    )
    binder_ring.visual(
        Box((0.016, 0.008, 0.020)),
        origin=Origin(xyz=(0.0255, -0.0060, 0.0)),
        material=forged_alloy,
        name="lower_ear",
    )
    binder_ring.visual(
        qr_pivot_pin_mesh,
        origin=Origin(xyz=(0.0320, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="pivot_pin",
    )
    binder_ring.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.021, 0.0, 0.0)),
        material=forged_alloy,
        name="threaded_bridge",
    )
    binder_ring.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, binder_length)),
        mass=0.08,
    )

    quick_release_lever = model.part("quick_release_lever")
    quick_release_lever.visual(
        lever_barrel_shell,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="lever_barrel",
    )
    quick_release_lever.visual(
        Box((0.018, 0.008, 0.046)),
        origin=Origin(xyz=(0.012, 0.0, -0.024)),
        material=forged_alloy,
        name="lever_blade",
    )
    quick_release_lever.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.007, 0.0, -0.008)),
        material=hardware_steel,
        name="cam_body",
    )
    quick_release_lever.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(
            xyz=(0.023, 0.0, -0.047),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=seal_black,
        name="lever_paddle",
    )
    quick_release_lever.inertial = Inertial.from_geometry(
        Box((0.040, 0.018, 0.060)),
        mass=0.05,
        origin=Origin(xyz=(0.016, 0.0, -0.024)),
    )

    telescoping_post = model.part("telescoping_post")
    telescoping_post.visual(
        stanchion_shell,
        origin=Origin(xyz=(0.0, 0.0, post_center_local_z)),
        material=hard_anodized,
        name="stanchion",
    )
    telescoping_post.inertial = Inertial.from_geometry(
        Cylinder(radius=post_radius, length=post_length),
        mass=0.44,
        origin=Origin(xyz=(0.0, 0.0, post_center_local_z)),
    )

    saddle_head_base = model.part("saddle_head_base")
    saddle_head_base.visual(
        Cylinder(radius=0.0128, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=forged_alloy,
        name="head_spigot",
    )
    saddle_head_base.visual(
        Box((0.018, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=forged_alloy,
        name="head_core",
    )
    saddle_head_base.visual(
        Box((0.022, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.011, 0.034)),
        material=forged_alloy,
        name="left_cheek",
    )
    saddle_head_base.visual(
        Box((0.022, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.011, 0.034)),
        material=forged_alloy,
        name="right_cheek",
    )
    saddle_head_base.visual(
        tilt_axle_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="tilt_axle",
    )
    saddle_head_base.inertial = Inertial.from_geometry(
        Box((0.028, 0.032, 0.050)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        tilt_barrel_shell,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="tilt_barrel",
    )
    saddle_clamp.visual(
        Box((0.044, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=forged_alloy,
        name="lower_plate",
    )
    saddle_clamp.visual(
        Box((0.038, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=forged_alloy,
        name="upper_plate",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0035, length=0.088),
        origin=Origin(xyz=(0.0, 0.022, 0.0165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_black,
        name="left_rail",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0035, length=0.088),
        origin=Origin(xyz=(0.0, -0.022, 0.0165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_black,
        name="right_rail",
    )
    for x_pos, stem_name in ((0.016, "front_bolt"), (-0.016, "rear_bolt")):
        saddle_clamp.visual(
            Cylinder(radius=0.0030, length=0.020),
            origin=Origin(xyz=(x_pos, 0.0, 0.018)),
            material=hardware_steel,
            name=stem_name,
        )
        saddle_clamp.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.029)),
            material=hardware_steel,
            name=f"{stem_name}_head",
        )
        saddle_clamp.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.007)),
            material=hardware_steel,
            name=f"{stem_name}_nut",
        )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.035)),
        mass=0.13,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    model.articulation(
        "body_to_binder_ring",
        ArticulationType.FIXED,
        parent=lower_body,
        child=binder_ring,
        origin=Origin(xyz=(0.0, 0.0, binder_center_z)),
    )
    model.articulation(
        "binder_quick_release",
        ArticulationType.REVOLUTE,
        parent=binder_ring,
        child=quick_release_lever,
        origin=Origin(xyz=(0.0320, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-1.65,
            upper=0.0,
        ),
    )
    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=telescoping_post,
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.25,
            lower=0.0,
            upper=drop_travel,
        ),
    )
    model.articulation(
        "post_to_head",
        ArticulationType.FIXED,
        parent=telescoping_post,
        child=saddle_head_base,
        origin=Origin(xyz=(0.0, 0.0, head_mount_z)),
    )
    model.articulation(
        "saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=saddle_head_base,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.26,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    lower_body = object_model.get_part("lower_body")
    binder_ring = object_model.get_part("binder_ring")
    quick_release_lever = object_model.get_part("quick_release_lever")
    telescoping_post = object_model.get_part("telescoping_post")
    saddle_head_base = object_model.get_part("saddle_head_base")
    saddle_clamp = object_model.get_part("saddle_clamp")

    ctx.allow_overlap(
        binder_ring,
        lower_body,
        elem_a="collar_band",
        elem_b="outer_tube",
        reason="Binder collar is modeled as a clamped cylindrical band directly wrapped onto the lower body.",
    )
    ctx.allow_overlap(
        quick_release_lever,
        binder_ring,
        elem_a="lever_barrel",
        elem_b="pivot_pin",
        reason="Quick-release lever rotates on a coaxial pivot pin with coincident bearing surfaces.",
    )
    ctx.allow_overlap(
        lower_body,
        telescoping_post,
        elem_a="upper_bushing",
        elem_b="stanchion",
        reason="Dropper stanchion rides inside the upper guide bushing with a coincident cylindrical sliding interface.",
    )
    ctx.allow_overlap(
        saddle_clamp,
        saddle_head_base,
        elem_a="tilt_barrel",
        elem_b="tilt_axle",
        reason="Saddle clamp tilts about a coaxial axle captured inside the lower clamp barrel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    binder_quick_release = object_model.get_articulation("binder_quick_release")
    dropper_travel = object_model.get_articulation("dropper_travel")
    saddle_tilt = object_model.get_articulation("saddle_tilt")

    ctx.check(
        "binder_quick_release_axis",
        tuple(round(v, 6) for v in binder_quick_release.axis) == (0.0, 1.0, 0.0),
        f"expected quick-release lever to pivot about +Y, got {binder_quick_release.axis}",
    )
    ctx.check(
        "dropper_travel_axis",
        tuple(round(v, 6) for v in dropper_travel.axis) == (0.0, 0.0, 1.0),
        f"expected dropper travel on +Z, got {dropper_travel.axis}",
    )
    ctx.check(
        "saddle_tilt_axis",
        tuple(round(v, 6) for v in saddle_tilt.axis) == (0.0, 1.0, 0.0),
        f"expected saddle tilt about +Y, got {saddle_tilt.axis}",
    )

    ctx.expect_contact(
        binder_ring,
        lower_body,
        elem_a="collar_band",
        elem_b="outer_tube",
        name="binder_ring_clamps_lower_body",
    )
    ctx.expect_contact(
        quick_release_lever,
        binder_ring,
        elem_a="lever_barrel",
        elem_b="pivot_pin",
        name="qr_lever_pivots_on_pin",
    )
    ctx.expect_contact(
        telescoping_post,
        lower_body,
        elem_a="stanchion",
        elem_b="upper_bushing",
        name="stanchion_guided_by_upper_bushing",
    )
    ctx.expect_contact(
        saddle_head_base,
        telescoping_post,
        elem_a="head_spigot",
        elem_b="stanchion",
        name="head_spigot_seated_on_stanchion",
    )
    ctx.expect_contact(
        saddle_clamp,
        saddle_head_base,
        elem_a="tilt_barrel",
        elem_b="tilt_axle",
        name="saddle_clamp_rotates_on_tilt_axle",
    )

    ctx.expect_within(
        lower_body,
        binder_ring,
        axes="xy",
        margin=0.0,
        inner_elem="outer_tube",
        outer_elem="collar_band",
        name="binder_ring_encircles_post_body",
    )
    ctx.expect_within(
        telescoping_post,
        lower_body,
        axes="xy",
        margin=0.0,
        inner_elem="stanchion",
        outer_elem="outer_tube",
        name="stanchion_coaxial_with_lower_body",
    )
    ctx.expect_origin_gap(
        binder_ring,
        lower_body,
        axis="z",
        min_gap=0.235,
        max_gap=0.237,
        name="binder_ring_at_top_of_body",
    )
    ctx.expect_origin_gap(
        saddle_head_base,
        telescoping_post,
        axis="z",
        min_gap=0.099,
        max_gap=0.101,
        name="head_base_at_post_tip",
    )
    ctx.expect_overlap(
        saddle_clamp,
        saddle_head_base,
        axes="xy",
        min_overlap=0.018,
        name="saddle_clamp_centered_over_head_base",
    )

    for joint in (binder_quick_release, dropper_travel, saddle_tilt):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_post_pos = ctx.part_world_position(telescoping_post)
    ctx.check("dropper_rest_position_available", rest_post_pos is not None, "missing rest post position")
    if rest_post_pos is not None:
        with ctx.pose({dropper_travel: 0.150}):
            raised_post_pos = ctx.part_world_position(telescoping_post)
            ctx.check(
                "dropper_moves_upward",
                raised_post_pos is not None and raised_post_pos[2] > rest_post_pos[2] + 0.149,
                f"expected ~0.150 m upward travel, got rest={rest_post_pos}, raised={raised_post_pos}",
            )
            head_pos = ctx.part_world_position(saddle_head_base)
            ctx.check(
                "dropper_extended_head_height",
                head_pos is not None and head_pos[2] > 0.49,
                f"expected extended head above 0.49 m, got {head_pos}",
            )
            ctx.expect_contact(
                telescoping_post,
                lower_body,
                elem_a="stanchion",
                elem_b="upper_bushing",
                name="stanchion_stays_guided_when_extended",
            )

    lever_rest_aabb = ctx.part_world_aabb(quick_release_lever)
    ctx.check("qr_lever_rest_aabb_available", lever_rest_aabb is not None, "missing lever rest aabb")
    if lever_rest_aabb is not None:
        with ctx.pose({binder_quick_release: -1.60}):
            lever_open_aabb = ctx.part_world_aabb(quick_release_lever)
            ctx.check(
                "qr_lever_open_pose_available",
                lever_open_aabb is not None,
                "missing lever open aabb",
            )
            if lever_open_aabb is not None:
                rest_center = aabb_center(lever_rest_aabb)
                open_center = aabb_center(lever_open_aabb)
                ctx.check(
                    "qr_lever_flips_upward",
                    open_center[2] > rest_center[2] + 0.030,
                    f"expected lever to swing upward; rest={rest_center}, open={open_center}",
                )
                ctx.expect_contact(
                    quick_release_lever,
                    binder_ring,
                    elem_a="lever_barrel",
                    elem_b="pivot_pin",
                    name="qr_lever_stays_on_pivot_when_open",
                )

    with ctx.pose({saddle_tilt: -0.26}):
        front_low = ctx.part_element_world_aabb(saddle_clamp, elem="front_bolt")
        rear_low = ctx.part_element_world_aabb(saddle_clamp, elem="rear_bolt")
    with ctx.pose({saddle_tilt: 0.26}):
        front_high = ctx.part_element_world_aabb(saddle_clamp, elem="front_bolt")
        rear_high = ctx.part_element_world_aabb(saddle_clamp, elem="rear_bolt")
    ctx.check(
        "saddle_tilt_element_aabbs_available",
        all(aabb is not None for aabb in (front_low, rear_low, front_high, rear_high)),
        "missing bolt element aabb for saddle tilt check",
    )
    if all(aabb is not None for aabb in (front_low, rear_low, front_high, rear_high)):
        front_low_center = aabb_center(front_low)
        rear_low_center = aabb_center(rear_low)
        front_high_center = aabb_center(front_high)
        rear_high_center = aabb_center(rear_high)
        ctx.check(
            "saddle_tilt_changes_front_rear_pitch",
            front_low_center[2] > rear_low_center[2] + 0.004
            and front_high_center[2] + 0.004 < rear_high_center[2],
            (
                "expected front and rear clamp bolts to swap relative height across tilt range; "
                f"low front/rear={front_low_center}/{rear_low_center}, "
                f"high front/rear={front_high_center}/{rear_high_center}"
            ),
        )
        with ctx.pose({saddle_tilt: 0.26}):
            ctx.expect_contact(
                saddle_clamp,
                saddle_head_base,
                elem_a="tilt_barrel",
                elem_b="tilt_axle",
                name="tilt_barrel_keeps_contact_at_positive_limit",
            )
        with ctx.pose({saddle_tilt: -0.26}):
            ctx.expect_contact(
                saddle_clamp,
                saddle_head_base,
                elem_a="tilt_barrel",
                elem_b="tilt_axle",
                name="tilt_barrel_keeps_contact_at_negative_limit",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
