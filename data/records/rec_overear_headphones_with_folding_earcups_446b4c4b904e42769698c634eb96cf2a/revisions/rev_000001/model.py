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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _build_headband_band_mesh():
    path = [
        (-0.094, 0.0, 0.145),
        (-0.078, 0.0, 0.168),
        (-0.040, 0.0, 0.185),
        (0.000, 0.0, 0.191),
        (0.040, 0.0, 0.185),
        (0.078, 0.0, 0.168),
        (0.094, 0.0, 0.145),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            path,
            profile=rounded_rect_profile(0.030, 0.010, 0.004),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "travel_headphone_headband_band",
    )


def _build_headband_pad_mesh():
    path = [
        (-0.062, 0.0, 0.138),
        (-0.036, 0.0, 0.153),
        (0.000, 0.0, 0.160),
        (0.036, 0.0, 0.153),
        (0.062, 0.0, 0.138),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            path,
            profile=rounded_rect_profile(0.022, 0.006, 0.003),
            samples_per_segment=16,
            cap_profile=True,
        ),
        "travel_headphone_headband_pad",
    )


def _build_cup_shell_mesh():
    sections = [
        _yz_section(0.058, 0.068, 0.016, 0.000, z_center=-0.032),
        _yz_section(0.064, 0.076, 0.018, 0.010, z_center=-0.032),
        _yz_section(0.066, 0.080, 0.020, 0.018, z_center=-0.032),
    ]
    return mesh_from_geometry(
        section_loft(sections),
        "travel_headphone_cup_shell",
    )


def _build_pad_mesh():
    outer = rounded_rect_profile(0.072, 0.086, 0.018)
    inner = rounded_rect_profile(0.040, 0.054, 0.014)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            0.012,
            cap=True,
            center=True,
            closed=True,
        ),
        "travel_headphone_ear_pad",
    )


def _add_slider_geometry(part, shell_material, rail_material) -> None:
    part.visual(
        Box((0.008, 0.0035, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=rail_material,
        name="rail",
    )
    part.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=shell_material,
        name="housing",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.018, 0.010, 0.060)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )


def _add_yoke_geometry(part, *, sign: float, material) -> None:
    part.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=material,
        name="hinge_cap",
    )
    part.visual(
        Box((0.008, 0.010, 0.018)),
        origin=Origin(xyz=(sign * 0.009, 0.0, -0.011)),
        material=material,
        name="upper_arm",
    )
    part.visual(
        Box((0.006, 0.010, 0.016)),
        origin=Origin(xyz=(sign * 0.007, 0.0, -0.026)),
        material=material,
        name="lower_arm",
    )
    part.visual(
        Box((0.010, 0.012, 0.006)),
        origin=Origin(xyz=(sign * 0.006, 0.0, -0.033)),
        material=material,
        name="swivel_boss",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.040)),
        mass=0.025,
        origin=Origin(xyz=(sign * 0.006, 0.0, -0.018)),
    )


def _add_cup_geometry(
    part,
    *,
    depth_sign: float,
    shell_mesh,
    pad_mesh,
    shell_material,
    cushion_material,
    grille_material,
) -> None:
    shell_rpy = (0.0, 0.0, 0.0) if depth_sign > 0.0 else (0.0, 0.0, math.pi)
    pad_pitch = math.pi / 2.0 if depth_sign > 0.0 else -math.pi / 2.0
    shell_drop = -0.008

    part.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=shell_material,
        name="swivel_collar",
    )
    part.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(depth_sign * 0.004, 0.0, -0.006)),
        material=shell_material,
        name="pivot_block",
    )
    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, shell_drop), rpy=shell_rpy),
        material=shell_material,
        name="shell",
    )
    part.visual(
        Box((0.002, 0.042, 0.058)),
        origin=Origin(xyz=(depth_sign * 0.019, 0.0, -0.032 + shell_drop)),
        material=grille_material,
        name="baffle",
    )
    part.visual(
        pad_mesh,
        origin=Origin(
            xyz=(depth_sign * 0.026, 0.0, -0.032 + shell_drop),
            rpy=(0.0, pad_pitch, 0.0),
        ),
        material=cushion_material,
        name="cushion",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.034, 0.072, 0.080)),
        mass=0.100,
        origin=Origin(xyz=(depth_sign * 0.014, 0.0, -0.032 + shell_drop)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_over_ear_headphones")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.16, 0.16, 0.17, 1.0))
    cushion_gray = model.material("cushion_gray", rgba=(0.22, 0.22, 0.24, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    grille_black = model.material("grille_black", rgba=(0.08, 0.08, 0.09, 1.0))

    headband_band_mesh = _build_headband_band_mesh()
    headband_pad_mesh = _build_headband_pad_mesh()
    cup_shell_mesh = _build_cup_shell_mesh()
    cup_pad_mesh = _build_pad_mesh()

    headband = model.part("headband")
    headband.visual(headband_band_mesh, material=shell_black, name="band")
    headband.visual(headband_pad_mesh, material=cushion_gray, name="pad")
    for side_name, x_pos in (("left", -0.034), ("right", 0.034)):
        headband.visual(
            Box((0.010, 0.012, 0.026)),
            origin=Origin(xyz=(x_pos, 0.0, 0.168)),
            material=trim_black,
            name=f"{side_name}_pad_mount",
        )
    for side_name, x_pos in (("left", -0.094), ("right", 0.094)):
        headband.visual(
            Box((0.014, 0.002, 0.046)),
            origin=Origin(xyz=(x_pos, -0.004, 0.120)),
            material=trim_black,
            name=f"{side_name}_guide_front",
        )
        headband.visual(
            Box((0.014, 0.002, 0.046)),
            origin=Origin(xyz=(x_pos, 0.004, 0.120)),
            material=trim_black,
            name=f"{side_name}_guide_back",
        )
        headband.visual(
            Box((0.014, 0.010, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.146)),
            material=trim_black,
            name=f"{side_name}_guide_cap",
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.220, 0.040, 0.200)),
        mass=0.320,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    left_slider = model.part("left_slider")
    right_slider = model.part("right_slider")
    _add_slider_geometry(left_slider, trim_black, rail_steel)
    _add_slider_geometry(right_slider, trim_black, rail_steel)

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    _add_yoke_geometry(left_yoke, sign=-1.0, material=trim_black)
    _add_yoke_geometry(right_yoke, sign=1.0, material=trim_black)

    left_cup = model.part("left_cup")
    right_cup = model.part("right_cup")
    _add_cup_geometry(
        left_cup,
        depth_sign=1.0,
        shell_mesh=cup_shell_mesh,
        pad_mesh=cup_pad_mesh,
        shell_material=shell_black,
        cushion_material=cushion_gray,
        grille_material=grille_black,
    )
    _add_cup_geometry(
        right_cup,
        depth_sign=-1.0,
        shell_mesh=cup_shell_mesh,
        pad_mesh=cup_pad_mesh,
        shell_material=shell_black,
        cushion_material=cushion_gray,
        grille_material=grille_black,
    )

    model.articulation(
        "headband_to_left_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_slider,
        origin=Origin(xyz=(-0.094, 0.0, 0.143)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.080, lower=0.0, upper=0.028),
    )
    model.articulation(
        "headband_to_right_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_slider,
        origin=Origin(xyz=(0.094, 0.0, 0.143)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.080, lower=0.0, upper=0.028),
    )
    model.articulation(
        "left_slider_to_yoke",
        ArticulationType.REVOLUTE,
        parent=left_slider,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_slider_to_yoke",
        ArticulationType.REVOLUTE,
        parent=right_slider,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "left_yoke_to_cup",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(-0.006, 0.0, -0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "right_yoke_to_cup",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.006, 0.0, -0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.10,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    left_slider = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_slider_joint = object_model.get_articulation("headband_to_left_slider")
    right_slider_joint = object_model.get_articulation("headband_to_right_slider")
    left_fold_joint = object_model.get_articulation("left_slider_to_yoke")
    right_fold_joint = object_model.get_articulation("right_slider_to_yoke")
    left_swivel_joint = object_model.get_articulation("left_yoke_to_cup")
    right_swivel_joint = object_model.get_articulation("right_yoke_to_cup")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for name, joint, expected_axis in (
        ("left_slider_axis", left_slider_joint, (0.0, 0.0, -1.0)),
        ("right_slider_axis", right_slider_joint, (0.0, 0.0, -1.0)),
        ("left_fold_axis", left_fold_joint, (0.0, -1.0, 0.0)),
        ("right_fold_axis", right_fold_joint, (0.0, 1.0, 0.0)),
        ("left_swivel_axis", left_swivel_joint, (0.0, 0.0, 1.0)),
        ("right_swivel_axis", right_swivel_joint, (0.0, 0.0, 1.0)),
    ):
        ctx.check(name, tuple(joint.axis) == expected_axis, f"expected {expected_axis}, got {joint.axis}")

    ctx.expect_contact(headband, left_slider, contact_tol=0.0005)
    ctx.expect_contact(headband, right_slider, contact_tol=0.0005)
    ctx.expect_contact(left_slider, left_yoke, contact_tol=0.0005)
    ctx.expect_contact(right_slider, right_yoke, contact_tol=0.0005)
    ctx.expect_contact(left_yoke, left_cup, contact_tol=0.0005)
    ctx.expect_contact(right_yoke, right_cup, contact_tol=0.0005)

    ctx.expect_gap(headband, left_cup, axis="z", min_gap=0.025)
    ctx.expect_gap(headband, right_cup, axis="z", min_gap=0.025)
    ctx.expect_origin_distance(left_cup, right_cup, axes="x", min_dist=0.160, max_dist=0.220)

    left_slider_rest = ctx.part_world_position(left_slider)
    right_slider_rest = ctx.part_world_position(right_slider)
    left_cup_rest = ctx.part_world_position(left_cup)
    right_cup_rest = ctx.part_world_position(right_cup)
    left_cup_rest_aabb = ctx.part_world_aabb(left_cup)
    right_cup_rest_aabb = ctx.part_world_aabb(right_cup)

    assert left_slider_rest is not None
    assert right_slider_rest is not None
    assert left_cup_rest is not None
    assert right_cup_rest is not None
    assert left_cup_rest_aabb is not None
    assert right_cup_rest_aabb is not None

    def _center(aabb):
        return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))

    left_cup_rest_center = _center(left_cup_rest_aabb)
    right_cup_rest_center = _center(right_cup_rest_aabb)

    with ctx.pose({left_slider_joint: 0.028, right_slider_joint: 0.028}):
        left_slider_extended = ctx.part_world_position(left_slider)
        right_slider_extended = ctx.part_world_position(right_slider)
        assert left_slider_extended is not None
        assert right_slider_extended is not None
        ctx.check(
            "left_slider_extends_downward",
            left_slider_extended[2] < left_slider_rest[2] - 0.025,
            f"rest={left_slider_rest}, extended={left_slider_extended}",
        )
        ctx.check(
            "right_slider_extends_downward",
            right_slider_extended[2] < right_slider_rest[2] - 0.025,
            f"rest={right_slider_rest}, extended={right_slider_extended}",
        )

    with ctx.pose({left_fold_joint: 1.00, right_fold_joint: 1.00}):
        left_cup_folded = ctx.part_world_position(left_cup)
        right_cup_folded = ctx.part_world_position(right_cup)
        assert left_cup_folded is not None
        assert right_cup_folded is not None
        ctx.check(
            "left_fold_moves_cup_inward",
            left_cup_folded[0] > left_cup_rest[0] + 0.020 and left_cup_folded[2] > left_cup_rest[2] + 0.008,
            f"rest={left_cup_rest}, folded={left_cup_folded}",
        )
        ctx.check(
            "right_fold_moves_cup_inward",
            right_cup_folded[0] < right_cup_rest[0] - 0.020 and right_cup_folded[2] > right_cup_rest[2] + 0.008,
            f"rest={right_cup_rest}, folded={right_cup_folded}",
        )

    with ctx.pose({left_swivel_joint: 0.80, right_swivel_joint: 0.80}):
        left_cup_swiveled = ctx.part_world_aabb(left_cup)
        right_cup_swiveled = ctx.part_world_aabb(right_cup)
        assert left_cup_swiveled is not None
        assert right_cup_swiveled is not None
        left_cup_swiveled_center = _center(left_cup_swiveled)
        right_cup_swiveled_center = _center(right_cup_swiveled)
        ctx.check(
            "left_swivel_shifts_cup_yaw",
            left_cup_swiveled_center[1] > left_cup_rest_center[1] + 0.008,
            f"rest={left_cup_rest_center}, swiveled={left_cup_swiveled_center}",
        )
        ctx.check(
            "right_swivel_shifts_cup_yaw",
            right_cup_swiveled_center[1] < right_cup_rest_center[1] - 0.008,
            f"rest={right_cup_rest_center}, swiveled={right_cup_swiveled_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
