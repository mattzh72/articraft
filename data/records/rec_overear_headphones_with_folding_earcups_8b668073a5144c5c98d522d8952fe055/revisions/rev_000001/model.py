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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _add_yoke_visuals(part, *, prefix: str, material) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="fold_barrel",
    )
    part.visual(
        Box((0.004, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=material,
        name="hinge_web",
    )
    part.visual(
        Box((0.004, 0.068, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=material,
        name="top_bridge",
    )
    for y_pos, stem_name in ((0.034, "front"), (-0.034, "rear")):
        part.visual(
            Box((0.004, 0.008, 0.046)),
            origin=Origin(xyz=(0.0, y_pos, -0.042)),
            material=material,
            name=f"{stem_name}_arm",
        )
        part.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(
                xyz=(0.0, y_pos, -0.064),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{stem_name}_pivot_block",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.018, 0.082, 0.078)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )


def _add_earcup_visuals(
    part,
    *,
    prefix: str,
    opening_sign: float,
    shell_material,
    cushion_material,
    fabric_material,
    accent_material,
    add_mic_mount: bool = False,
) -> None:
    shell_body = ExtrudeGeometry(
        rounded_rect_profile(0.086, 0.058, 0.016, corner_segments=8),
        0.020,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)
    part.visual(
        _mesh(f"{prefix}_shell_body", shell_body),
        origin=Origin(xyz=(-opening_sign * 0.011, 0.0, 0.0)),
        material=shell_material,
        name="shell_body",
    )

    outer_cap = ExtrudeGeometry(
        rounded_rect_profile(0.064, 0.038, 0.012, corner_segments=8),
        0.003,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)
    part.visual(
        _mesh(f"{prefix}_outer_cap", outer_cap),
        origin=Origin(xyz=(-opening_sign * 0.0225, 0.0, 0.002)),
        material=accent_material,
        name="outer_cap",
    )

    inner_baffle = ExtrudeGeometry(
        rounded_rect_profile(0.066, 0.040, 0.010, corner_segments=8),
        0.002,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)
    part.visual(
        _mesh(f"{prefix}_inner_baffle", inner_baffle),
        origin=Origin(xyz=(opening_sign * 0.001, 0.0, 0.0)),
        material=fabric_material,
        name="inner_baffle",
    )

    pad_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.080, 0.054, 0.018, corner_segments=8),
        [rounded_rect_profile(0.058, 0.030, 0.014, corner_segments=8)],
        0.014,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)
    part.visual(
        _mesh(f"{prefix}_pad_ring", pad_ring),
        origin=Origin(xyz=(opening_sign * 0.011, 0.0, 0.0)),
        material=cushion_material,
        name="pad_ring",
    )

    part.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_material,
        name="swivel_axle",
    )

    if add_mic_mount:
        part.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(
                xyz=(-opening_sign * 0.018, 0.020, -0.020),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=accent_material,
            name="mic_mount",
        )

    part.inertial = Inertial.from_geometry(
        Box((0.042, 0.062, 0.090)),
        mass=0.12,
        origin=Origin(),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_over_ear_headset")

    headband_black = model.material("headband_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shell_graphite = model.material("shell_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    yoke_metal = model.material("yoke_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.09, 0.09, 0.10, 1.0))
    fabric_dark = model.material("fabric_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    mic_dark = model.material("mic_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.30, 0.32, 0.35, 1.0))

    headband_frame = model.part("headband_frame")
    outer_band = sweep_profile_along_spline(
        [
            (-0.095, 0.0, 0.156),
            (-0.078, 0.0, 0.184),
            (-0.038, 0.0, 0.206),
            (0.0, 0.0, 0.214),
            (0.038, 0.0, 0.206),
            (0.078, 0.0, 0.184),
            (0.095, 0.0, 0.156),
        ],
        profile=rounded_rect_profile(0.030, 0.010, 0.004, corner_segments=6),
        samples_per_segment=16,
        cap_profile=True,
    )
    headband_frame.visual(
        _mesh("headband_outer_band", outer_band),
        material=headband_black,
        name="outer_band",
    )

    inner_pad = sweep_profile_along_spline(
        [
            (-0.068, 0.0, 0.149),
            (-0.040, 0.0, 0.173),
            (0.0, 0.0, 0.181),
            (0.040, 0.0, 0.173),
            (0.068, 0.0, 0.149),
        ],
        profile=rounded_rect_profile(0.018, 0.006, 0.0025, corner_segments=6),
        samples_per_segment=16,
        cap_profile=True,
    )
    headband_frame.visual(
        _mesh("headband_inner_pad", inner_pad),
        material=cushion_black,
        name="inner_pad",
    )
    headband_frame.visual(
        Box((0.184, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=headband_black,
        name="pad_carrier",
    )

    for side_name, x_pos in (("left", -0.095), ("right", 0.095)):
        headband_frame.visual(
            Box((0.014, 0.030, 0.022)),
            origin=Origin(xyz=(x_pos, 0.0, 0.157)),
            material=headband_black,
            name=f"{side_name}_arm_block",
        )
        headband_frame.visual(
            Box((0.010, 0.006, 0.018)),
            origin=Origin(xyz=(x_pos, 0.013, 0.144)),
            material=accent_gray,
            name=f"{side_name}_front_lug",
        )
        headband_frame.visual(
            Box((0.010, 0.006, 0.018)),
            origin=Origin(xyz=(x_pos, -0.013, 0.144)),
            material=accent_gray,
            name=f"{side_name}_rear_lug",
        )

    headband_frame.inertial = Inertial.from_geometry(
        Box((0.230, 0.040, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    _add_yoke_visuals(left_yoke, prefix="left", material=yoke_metal)
    _add_yoke_visuals(right_yoke, prefix="right", material=yoke_metal)

    left_cup = model.part("left_cup")
    right_cup = model.part("right_cup")
    _add_earcup_visuals(
        left_cup,
        prefix="left_cup",
        opening_sign=1.0,
        shell_material=shell_graphite,
        cushion_material=cushion_black,
        fabric_material=fabric_dark,
        accent_material=accent_gray,
        add_mic_mount=True,
    )
    _add_earcup_visuals(
        right_cup,
        prefix="right_cup",
        opening_sign=-1.0,
        shell_material=shell_graphite,
        cushion_material=cushion_black,
        fabric_material=fabric_dark,
        accent_material=accent_gray,
    )

    mic_boom = model.part("mic_boom")
    mic_boom.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_gray,
        name="pivot_barrel",
    )
    boom_arm = tube_from_spline_points(
        [
            (0.000, 0.000, 0.000),
            (-0.002, 0.026, -0.010),
            (-0.004, 0.074, -0.017),
            (-0.006, 0.116, -0.012),
        ],
        radius=0.0025,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    mic_boom.visual(
        _mesh("mic_boom_arm_v3", boom_arm),
        material=mic_dark,
        name="boom_arm",
    )
    mic_boom.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(
            xyz=(-0.006, 0.123, -0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=mic_dark,
        name="mic_capsule",
    )
    mic_boom.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(-0.006, 0.135, -0.012)),
        material=mic_dark,
        name="mic_windscreen",
    )
    mic_boom.inertial = Inertial.from_geometry(
        Box((0.040, 0.145, 0.028)),
        mass=0.03,
        origin=Origin(xyz=(0.020, 0.070, -0.012)),
    )

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband_frame,
        child=left_yoke,
        origin=Origin(xyz=(-0.095, 0.0, 0.144)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband_frame,
        child=right_yoke,
        origin=Origin(xyz=(0.095, 0.0, 0.144)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=math.radians(-28.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=math.radians(-28.0),
            upper=math.radians(28.0),
        ),
    )

    model.articulation(
        "mic_pivot",
        ArticulationType.REVOLUTE,
        parent=left_cup,
        child=mic_boom,
        origin=Origin(xyz=(-0.027, 0.020, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=math.radians(-15.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    part_names = (
        "headband_frame",
        "left_yoke",
        "right_yoke",
        "left_cup",
        "right_cup",
        "mic_boom",
    )
    joint_names = (
        "left_fold",
        "right_fold",
        "left_cup_swivel",
        "right_cup_swivel",
        "mic_pivot",
    )

    parts = {part.name: part for part in object_model.parts}
    joints = {joint.name: joint for joint in object_model.articulations}

    for name in part_names:
        ctx.check(f"has_{name}", name in parts, f"Missing part: {name}")
    for name in joint_names:
        ctx.check(f"has_{name}", name in joints, f"Missing articulation: {name}")

    if not all(name in parts for name in part_names) or not all(name in joints for name in joint_names):
        return ctx.report()

    headband_frame = parts["headband_frame"]
    left_yoke = parts["left_yoke"]
    right_yoke = parts["right_yoke"]
    left_cup = parts["left_cup"]
    right_cup = parts["right_cup"]
    mic_boom = parts["mic_boom"]

    left_fold = joints["left_fold"]
    right_fold = joints["right_fold"]
    left_cup_swivel = joints["left_cup_swivel"]
    right_cup_swivel = joints["right_cup_swivel"]
    mic_pivot = joints["mic_pivot"]

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("left_fold_axis", left_fold.axis == (0.0, -1.0, 0.0), f"{left_fold.axis}")
    ctx.check("right_fold_axis", right_fold.axis == (0.0, 1.0, 0.0), f"{right_fold.axis}")
    ctx.check("left_cup_swivel_axis", left_cup_swivel.axis == (0.0, 1.0, 0.0), f"{left_cup_swivel.axis}")
    ctx.check("right_cup_swivel_axis", right_cup_swivel.axis == (0.0, 1.0, 0.0), f"{right_cup_swivel.axis}")
    ctx.check("mic_pivot_axis", mic_pivot.axis == (1.0, 0.0, 0.0), f"{mic_pivot.axis}")
    ctx.check("mic_mount_parent", mic_pivot.parent == "left_cup", f"{mic_pivot.parent}")

    ctx.expect_contact(headband_frame, left_yoke, name="left_yoke_contacts_headband")
    ctx.expect_contact(headband_frame, right_yoke, name="right_yoke_contacts_headband")
    ctx.expect_contact(left_yoke, left_cup, name="left_cup_contacts_yoke")
    ctx.expect_contact(right_yoke, right_cup, name="right_cup_contacts_yoke")
    ctx.expect_contact(left_cup, mic_boom, name="mic_contacts_left_cup")

    ctx.expect_origin_distance(
        left_yoke,
        right_yoke,
        axes="x",
        min_dist=0.17,
        max_dist=0.21,
        name="yokes_spaced_under_headband",
    )
    ctx.expect_origin_distance(
        left_cup,
        right_cup,
        axes="x",
        min_dist=0.17,
        max_dist=0.21,
        name="earcups_have_head_width_spacing",
    )
    ctx.expect_origin_gap(
        mic_boom,
        left_cup,
        axis="y",
        min_gap=0.015,
        max_gap=0.030,
        name="microphone_pivot_sits_forward_of_left_cup",
    )

    left_pivot_rest = ctx.part_element_world_aabb(left_yoke, elem="front_pivot_block")
    right_pivot_rest = ctx.part_element_world_aabb(right_yoke, elem="front_pivot_block")
    left_cap_rest = ctx.part_element_world_aabb(left_cup, elem="outer_cap")
    right_cap_rest = ctx.part_element_world_aabb(right_cup, elem="outer_cap")
    mic_capsule_rest = ctx.part_element_world_aabb(mic_boom, elem="mic_capsule")

    assert left_pivot_rest is not None
    assert right_pivot_rest is not None
    assert left_cap_rest is not None
    assert right_cap_rest is not None
    assert mic_capsule_rest is not None

    left_pivot_rest_center = _aabb_center(left_pivot_rest)
    right_pivot_rest_center = _aabb_center(right_pivot_rest)
    left_cap_rest_center = _aabb_center(left_cap_rest)
    right_cap_rest_center = _aabb_center(right_cap_rest)
    mic_capsule_rest_center = _aabb_center(mic_capsule_rest)

    with ctx.pose({left_fold: math.radians(80.0), right_fold: math.radians(80.0)}):
        left_pivot_folded = ctx.part_element_world_aabb(left_yoke, elem="front_pivot_block")
        right_pivot_folded = ctx.part_element_world_aabb(right_yoke, elem="front_pivot_block")
        assert left_pivot_folded is not None
        assert right_pivot_folded is not None
        left_pivot_folded_center = _aabb_center(left_pivot_folded)
        right_pivot_folded_center = _aabb_center(right_pivot_folded)
        assert left_pivot_folded_center[0] > left_pivot_rest_center[0] + 0.04
        assert left_pivot_folded_center[2] > left_pivot_rest_center[2] + 0.02
        assert right_pivot_folded_center[0] < right_pivot_rest_center[0] - 0.04
        assert right_pivot_folded_center[2] > right_pivot_rest_center[2] + 0.02
        ctx.expect_contact(headband_frame, left_yoke)
        ctx.expect_contact(headband_frame, right_yoke)

    with ctx.pose({left_cup_swivel: math.radians(22.0), right_cup_swivel: math.radians(-22.0)}):
        left_cap_turned = ctx.part_element_world_aabb(left_cup, elem="outer_cap")
        right_cap_turned = ctx.part_element_world_aabb(right_cup, elem="outer_cap")
        assert left_cap_turned is not None
        assert right_cap_turned is not None
        left_cap_turned_center = _aabb_center(left_cap_turned)
        right_cap_turned_center = _aabb_center(right_cap_turned)
        assert left_cap_turned_center[2] > left_cap_rest_center[2] + 0.006
        assert right_cap_turned_center[2] > right_cap_rest_center[2] + 0.006
        ctx.expect_contact(left_yoke, left_cup)
        ctx.expect_contact(right_yoke, right_cup)

    with ctx.pose({mic_pivot: math.radians(60.0)}):
        mic_capsule_raised = ctx.part_element_world_aabb(mic_boom, elem="mic_capsule")
        assert mic_capsule_raised is not None
        mic_capsule_raised_center = _aabb_center(mic_capsule_raised)
        assert mic_capsule_raised_center[2] > mic_capsule_rest_center[2] + 0.08
        assert mic_capsule_raised_center[1] < mic_capsule_rest_center[1] - 0.02
        ctx.expect_contact(left_cup, mic_boom)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
