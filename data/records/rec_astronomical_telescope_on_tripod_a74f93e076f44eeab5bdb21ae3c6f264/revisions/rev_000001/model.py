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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


TRIPOD_PIVOT_Z = 0.62
LEG_SPLAY = math.radians(24.0)
UPPER_LEG_LENGTH = 0.42
LOWER_LEG_LENGTH = 0.30
LEG_SLIDER_ORIGIN = 0.26


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_objective_hood():
    hood = LatheGeometry.from_shell_profiles(
        outer_profile=((0.046, 0.0), (0.049, 0.035), (0.050, 0.090)),
        inner_profile=((0.040, 0.004), (0.043, 0.035), (0.044, 0.086)),
        segments=56,
        end_cap="flat",
    )
    hood.rotate_y(math.pi / 2.0)
    return hood


def _build_pan_handle():
    return tube_from_spline_points(
        [
            (-0.010, -0.012, 0.072),
            (-0.060, -0.085, 0.056),
            (-0.140, -0.180, 0.028),
            (-0.230, -0.255, 0.002),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
    )


def _rect_profile(width: float, height: float):
    hw = width * 0.5
    hh = height * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _rounded_plate_mesh(width: float, depth: float, height: float, radius: float, name: str):
    plate = ExtrudeGeometry(rounded_rect_profile(width, depth, radius), height, center=True)
    return _mesh(name, plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spotting_scope_tripod")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    carbon = model.material("carbon", rgba=(0.14, 0.15, 0.16, 1.0))
    olive = model.material("olive", rgba=(0.28, 0.34, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.64, 0.67, 1.0))
    glass = model.material("glass", rgba=(0.30, 0.45, 0.52, 0.45))

    head_base = model.part("head_base")
    head_base.visual(
        Cylinder(radius=0.020, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=graphite,
        name="center_column",
    )
    head_base.visual(
        Box((0.090, 0.090, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=graphite,
        name="crown_block",
    )
    head_base.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.686)),
        material=black,
        name="pan_seat",
    )
    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        head_base.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(
                xyz=(0.045 * math.cos(azimuth), 0.045 * math.sin(azimuth), 0.662),
                rpy=(math.pi / 2.0, 0.0, azimuth),
            ),
            material=metal,
            name=f"leg_{index}_hinge_pin",
        )
    head_base.inertial = Inertial.from_geometry(
        Box((0.180, 0.180, 0.180)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    pan_platform = model.part("pan_platform")
    pan_platform.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black,
        name="base_disk",
    )
    pan_platform.visual(
        Box((0.018, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, -0.021, 0.052)),
        material=graphite,
        name="left_yoke",
    )
    pan_platform.visual(
        Box((0.018, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, 0.021, 0.052)),
        material=graphite,
        name="right_yoke",
    )
    pan_platform.visual(
        Box((0.022, 0.050, 0.010)),
        origin=Origin(xyz=(-0.002, 0.0, 0.087)),
        material=graphite,
        name="yoke_bridge",
    )
    pan_platform.visual(
        _mesh("pan_handle", _build_pan_handle()),
        material=graphite,
        name="pan_handle",
    )
    pan_platform.inertial = Inertial.from_geometry(
        Box((0.180, 0.100, 0.160)),
        mass=1.2,
        origin=Origin(xyz=(0.035, 0.0, 0.070)),
    )

    tilt_platform = model.part("tilt_platform")
    tilt_platform.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="trunnion",
    )
    tilt_platform.visual(
        Box((0.030, 0.022, 0.020)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=graphite,
        name="cradle_block",
    )
    tilt_platform.visual(
        Box((0.090, 0.032, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.010)),
        material=graphite,
        name="plate_clamp",
    )
    tilt_platform.visual(
        Box((0.120, 0.045, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.021)),
        material=graphite,
        name="quick_plate",
    )
    tilt_platform.inertial = Inertial.from_geometry(
        Box((0.150, 0.070, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.045, 0.0, 0.010)),
    )

    scope_body = model.part("scope_body")
    scope_body.visual(
        Box((0.110, 0.030, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.006)),
        material=graphite,
        name="mounting_foot",
    )
    scope_body.visual(
        Box((0.032, 0.034, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, 0.037)),
        material=graphite,
        name="mount_pedestal",
    )
    scope_body.visual(
        Box((0.086, 0.070, 0.056)),
        origin=Origin(xyz=(-0.004, 0.0, 0.090)),
        material=olive,
        name="prism_housing",
    )
    scope_body.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.028, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="mounting_collar",
    )
    scope_body.visual(
        Cylinder(radius=0.040, length=0.250),
        origin=Origin(xyz=(0.140, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="main_tube",
    )
    scope_body.visual(
        Cylinder(radius=0.047, length=0.058),
        origin=Origin(xyz=(0.285, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="objective_bell",
    )
    scope_body.visual(
        _mesh("objective_hood", _build_objective_hood()),
        origin=Origin(xyz=(0.314, 0.0, 0.078)),
        material=black,
        name="objective_hood",
    )
    scope_body.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.314, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    scope_body.visual(
        Cylinder(radius=0.022, length=0.100),
        origin=Origin(xyz=(-0.046, 0.0, 0.134), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=black,
        name="eyepiece_tube",
    )
    scope_body.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(-0.091, 0.0, 0.179), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=rubber,
        name="eyecup",
    )
    scope_body.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(-0.106, 0.0, 0.194), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=glass,
        name="eyepiece_lens",
    )
    scope_body.inertial = Inertial.from_geometry(
        Box((0.470, 0.110, 0.220)),
        mass=1.9,
        origin=Origin(xyz=(0.145, 0.0, 0.090)),
    )

    model.articulation(
        "pan_joint",
        ArticulationType.CONTINUOUS,
        parent=head_base,
        child=pan_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_platform,
        child=tilt_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=math.radians(-25.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "scope_mount",
        ArticulationType.FIXED,
        parent=tilt_platform,
        child=scope_body,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        upper_leg = model.part(f"leg_{index}_upper")
        upper_sleeve = ExtrudeWithHolesGeometry(
            _rect_profile(0.018, 0.012),
            [_rect_profile(0.010, 0.004)],
            0.340,
            center=False,
        )
        upper_leg.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        upper_leg.visual(
            _mesh(f"upper_leg_sleeve_{index}", upper_sleeve),
            material=carbon,
            name="upper_sleeve",
        )
        upper_leg.inertial = Inertial.from_geometry(
            Box((0.020, 0.012, 0.380)),
            mass=0.48,
            origin=Origin(xyz=(0.0, 0.0, 0.190)),
        )

        lower_leg = model.part(f"leg_{index}_lower")
        lower_leg.visual(
            Box((0.010, 0.004, 0.340)),
            origin=Origin(xyz=(0.0, 0.0, 0.205)),
            material=carbon,
            name="slider_tube",
        )
        lower_leg.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.369)),
            material=rubber,
            name="foot_pivot",
        )
        lower_leg.visual(
            Box((0.026, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.383)),
            material=rubber,
            name="foot_pad",
        )
        lower_leg.inertial = Inertial.from_geometry(
            Box((0.030, 0.020, 0.360)),
            mass=0.34,
            origin=Origin(xyz=(0.0, 0.0, 0.215)),
        )

        pivot_x = 0.045 * math.cos(azimuth)
        pivot_y = 0.045 * math.sin(azimuth)
        model.articulation(
            f"leg_{index}_fold",
            ArticulationType.REVOLUTE,
            parent=head_base,
            child=upper_leg,
            origin=Origin(
                xyz=(pivot_x, pivot_y, 0.662),
                rpy=(0.0, math.pi - LEG_SPLAY, azimuth),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=0.8,
                lower=math.radians(-10.0),
                upper=math.radians(20.0),
            ),
        )
        model.articulation(
            f"leg_{index}_extend",
            ArticulationType.PRISMATIC,
            parent=upper_leg,
            child=lower_leg,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.10,
                lower=0.0,
                upper=0.12,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_base = object_model.get_part("head_base")
    pan_platform = object_model.get_part("pan_platform")
    tilt_platform = object_model.get_part("tilt_platform")
    scope_body = object_model.get_part("scope_body")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    upper_legs = [object_model.get_part(f"leg_{index}_upper") for index in range(3)]
    lower_legs = [object_model.get_part(f"leg_{index}_lower") for index in range(3)]
    fold_joints = [object_model.get_articulation(f"leg_{index}_fold") for index in range(3)]
    extend_joints = [object_model.get_articulation(f"leg_{index}_extend") for index in range(3)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    for index in range(3):
        ctx.allow_overlap(
            head_base,
            upper_legs[index],
            reason="Tripod leg hinge socket is simplified as a nested crown connection around the leg head.",
        )
        ctx.allow_overlap(
            upper_legs[index],
            head_base,
            reason="Tripod leg hinge barrel rotates around the crown hinge pin.",
            elem_a="hinge_barrel",
            elem_b=f"leg_{index}_hinge_pin",
        )
        ctx.allow_overlap(
            upper_legs[index],
            lower_legs[index],
            reason="Telescoping leg sections are modeled as solid nested members rather than hollow tubes.",
        )

    ctx.allow_overlap(
        tilt_platform,
        scope_body,
        reason="The quick-release clamp captures the scope foot with a simplified solid-on-solid mount.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        pan_platform,
        head_base,
        axis="z",
        max_gap=0.0008,
        max_penetration=1e-5,
        positive_elem="base_disk",
        negative_elem="pan_seat",
        name="pan_platform_seated_on_pan_base",
    )
    ctx.expect_overlap(
        pan_platform,
        head_base,
        axes="xy",
        min_overlap=0.060,
        elem_a="base_disk",
        elem_b="pan_seat",
        name="pan_platform_covers_pan_seat",
    )
    ctx.expect_contact(
        tilt_platform,
        scope_body,
        elem_a="quick_plate",
        elem_b="mounting_foot",
        name="scope_foot_on_quick_plate",
    )
    ctx.expect_contact(
        tilt_platform,
        pan_platform,
        elem_a="trunnion",
        elem_b="left_yoke",
        name="left_trunnion_supported_by_yoke",
    )
    ctx.expect_contact(
        tilt_platform,
        pan_platform,
        elem_a="trunnion",
        elem_b="right_yoke",
        name="right_trunnion_supported_by_yoke",
    )

    for index in range(3):
        ctx.expect_contact(
            upper_legs[index],
            head_base,
            elem_a="hinge_barrel",
            elem_b=f"leg_{index}_hinge_pin",
            name=f"leg_{index}_hinge_contact",
        )
        ctx.expect_contact(
            lower_legs[index],
            upper_legs[index],
            name=f"leg_{index}_telescoping_contact",
        )

    ctx.check(
        "pan_joint_axis_is_vertical",
        pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"Pan axis was {pan_joint.axis}",
    )
    ctx.check(
        "tilt_joint_axis_is_horizontal",
        tilt_joint.axis == (0.0, 1.0, 0.0),
        details=f"Tilt axis was {tilt_joint.axis}",
    )
    for index, joint in enumerate(fold_joints):
        ctx.check(
            f"leg_{index}_fold_axis_local_y",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"Fold axis was {joint.axis}",
        )
    for index, joint in enumerate(extend_joints):
        ctx.check(
            f"leg_{index}_extend_axis_local_z",
            joint.axis == (0.0, 0.0, 1.0),
            details=f"Extend axis was {joint.axis}",
        )

    obj_rest = _aabb_center(ctx.part_element_world_aabb(scope_body, elem="objective_lens"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        obj_pan = _aabb_center(ctx.part_element_world_aabb(scope_body, elem="objective_lens"))
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_joint_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="pan_joint_quarter_turn_no_floating")
        ctx.expect_gap(
            pan_platform,
            head_base,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="base_disk",
            negative_elem="pan_seat",
            name="pan_base_contact_at_ninety_degrees",
        )
    pan_ok = obj_rest is not None and obj_pan is not None and abs(obj_rest[0]) > 0.20 and abs(obj_pan[1]) > 0.20 and abs(obj_pan[0]) < 0.08
    ctx.check(
        "pan_rotates_scope_around_vertical_axis",
        pan_ok,
        details=f"objective rest={obj_rest}, pan90={obj_pan}",
    )

    tilt_limits = tilt_joint.motion_limits
    tilt_low = None
    tilt_high = None
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.lower}):
            tilt_low = _aabb_center(ctx.part_element_world_aabb(scope_body, elem="objective_lens"))
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            tilt_high = _aabb_center(ctx.part_element_world_aabb(scope_body, elem="objective_lens"))
    tilt_ok = tilt_low is not None and tilt_high is not None and abs(tilt_high[2] - tilt_low[2]) > 0.10
    ctx.check(
        "tilt_moves_objective_in_elevation",
        tilt_ok,
        details=f"tilt_low={tilt_low}, tilt_high={tilt_high}",
    )

    front_foot_low = None
    front_foot_high = None
    fold_limits = fold_joints[0].motion_limits
    if fold_limits is not None and fold_limits.lower is not None and fold_limits.upper is not None:
        with ctx.pose({fold_joints[0]: fold_limits.lower}):
            front_foot_low = _aabb_center(ctx.part_element_world_aabb(lower_legs[0], elem="foot_pad"))
        with ctx.pose({fold_joints[0]: fold_limits.upper}):
            front_foot_high = _aabb_center(ctx.part_element_world_aabb(lower_legs[0], elem="foot_pad"))
    fold_ok = (
        front_foot_low is not None
        and front_foot_high is not None
        and abs(math.hypot(front_foot_high[0], front_foot_high[1]) - math.hypot(front_foot_low[0], front_foot_low[1])) > 0.06
    )
    ctx.check(
        "front_leg_fold_changes_tripod_stance",
        fold_ok,
        details=f"fold_low={front_foot_low}, fold_high={front_foot_high}",
    )

    for index, joint in enumerate(extend_joints):
        limits = joint.motion_limits
        low_center = None
        high_center = None
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                low_center = _aabb_center(ctx.part_element_world_aabb(lower_legs[index], elem="foot_pad"))
            with ctx.pose({joint: limits.upper}):
                high_center = _aabb_center(ctx.part_element_world_aabb(lower_legs[index], elem="foot_pad"))
        ok = low_center is not None and high_center is not None and (low_center[2] - high_center[2]) > 0.07
        ctx.check(
            f"leg_{index}_extension_lowers_foot",
            ok,
            details=f"low={low_center}, high={high_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
