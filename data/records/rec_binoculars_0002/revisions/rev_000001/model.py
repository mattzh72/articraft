from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _rounded_section(
    center_x: float,
    y_pos: float,
    center_z: float,
    width: float,
    height: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    corner = min(half_w, half_h) * 0.36
    return [
        (center_x - half_w + corner, y_pos, center_z - half_h),
        (center_x + half_w - corner, y_pos, center_z - half_h),
        (center_x + half_w, y_pos, center_z - half_h + corner),
        (center_x + half_w, y_pos, center_z + half_h - corner),
        (center_x + half_w - corner, y_pos, center_z + half_h),
        (center_x - half_w + corner, y_pos, center_z + half_h),
        (center_x - half_w, y_pos, center_z + half_h - corner),
        (center_x - half_w, y_pos, center_z - half_h + corner),
    ]


def _build_barrel_shell(side: float):
    sections = [
        _rounded_section(side * 0.020, -0.028, 0.013, 0.020, 0.018),
        _rounded_section(side * 0.024, -0.014, 0.012, 0.032, 0.027),
        _rounded_section(side * 0.030, 0.001, 0.010, 0.043, 0.033),
        _rounded_section(side * 0.035, 0.017, 0.004, 0.038, 0.029),
        _rounded_section(side * 0.039, 0.031, -0.001, 0.028, 0.022),
    ]
    filename = "left_barrel_shell.obj" if side < 0.0 else "right_barrel_shell.obj"
    return _save_mesh(filename, section_loft(sections))


def _tube_shell_mesh(
    filename: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    lip: float = 0.0007,
):
    half = length * 0.5
    return _save_mesh(
        filename,
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -half),
                (outer_radius, half),
            ],
            [
                (inner_radius, -half + lip),
                (inner_radius, half - lip),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _objective_bell_mesh(filename: str):
    return _save_mesh(
        filename,
        LatheGeometry.from_shell_profiles(
            [
                (0.0138, -0.0030),
                (0.0149, 0.0004),
                (0.0156, 0.0036),
            ],
            [
                (0.0118, -0.0023),
                (0.0131, 0.0027),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _barrel_feature_origin(
    side: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    if side > 0.0:
        return Origin(
            xyz=(xyz[0], xyz[1] + 0.003, xyz[2] - 0.020),
            rpy=rpy,
        )
    return Origin(xyz=xyz, rpy=rpy)


def _add_barrel(
    model: ArticulatedObject,
    part_name: str,
    side: float,
    armor,
    trim,
    *,
    carries_bridge: bool = False,
) -> None:
    part = model.part(part_name)
    part.visual(
        _build_barrel_shell(side),
        origin=_barrel_feature_origin(side, (0.0, 0.0, 0.0)),
        material=armor,
        name="main_shell",
    )

    if side < 0.0:
        part.visual(
            Box((0.010, 0.016, 0.010)),
            origin=_barrel_feature_origin(side, (-0.010, -0.007, 0.024)),
            material=armor,
            name="inner_bridge_rib",
        )
        part.visual(
            Box((0.008, 0.011, 0.010)),
            origin=_barrel_feature_origin(side, (-0.008, -0.010, 0.013)),
            material=armor,
            name="lower_bridge_rib",
        )
        part.visual(
            Cylinder(radius=0.0058, length=0.006),
            origin=_barrel_feature_origin(side, (0.0, -0.003, 0.012)),
            material=trim,
            name="hinge_lower_knuckle",
        )
        part.visual(
            Cylinder(radius=0.0058, length=0.006),
            origin=_barrel_feature_origin(side, (0.0, -0.003, 0.028)),
            material=trim,
            name="hinge_upper_knuckle",
        )
        part.visual(
            Box((0.006, 0.010, 0.024)),
            origin=_barrel_feature_origin(side, (-0.006, -0.003, 0.020)),
            material=trim,
            name="hinge_side_plate",
        )
    else:
        part.visual(
            Box((0.008, 0.014, 0.008)),
            origin=_barrel_feature_origin(side, (0.013, -0.007, 0.023)),
            material=armor,
            name="inner_bridge_rib",
        )
        part.visual(
            Box((0.006, 0.010, 0.008)),
            origin=_barrel_feature_origin(side, (0.011, -0.010, 0.013)),
            material=armor,
            name="lower_bridge_rib",
        )
        part.visual(
            Cylinder(radius=0.0057, length=0.010),
            origin=_barrel_feature_origin(side, (0.0, -0.003, 0.020)),
            material=trim,
            name="hinge_center_knuckle",
        )
        part.visual(
            Box((0.006, 0.010, 0.016)),
            origin=_barrel_feature_origin(side, (0.006, -0.003, 0.020)),
            material=trim,
            name="hinge_side_plate",
        )

    part.visual(
        _tube_shell_mesh(
            f"{part_name}_objective_tube.obj",
            outer_radius=0.0134,
            inner_radius=0.0114,
            length=0.020,
        ),
        origin=_barrel_feature_origin(side, (side * 0.041, 0.039, 0.000), (-math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="objective_tube",
    )
    part.visual(
        _objective_bell_mesh(f"{part_name}_objective_bell.obj"),
        origin=_barrel_feature_origin(side, (side * 0.041, 0.051, 0.000), (-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="objective_bell",
    )
    part.visual(
        _tube_shell_mesh(
            f"{part_name}_eyepiece_tube.obj",
            outer_radius=0.0094,
            inner_radius=0.0076,
            length=0.016,
        ),
        origin=_barrel_feature_origin(side, (side * 0.021, -0.034, 0.014), (-math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="eyepiece_tube",
    )
    part.visual(
        Cylinder(radius=0.0122, length=0.008),
        origin=_barrel_feature_origin(side, (side * 0.021, -0.0457, 0.014), (-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="eyecup",
    )

    if carries_bridge:
        part.visual(
            Cylinder(radius=0.0044, length=0.024),
            origin=Origin(xyz=(0.0, -0.003, 0.020)),
            material=trim,
            name="hinge_axle",
        )
        part.visual(
            Box((0.006, 0.010, 0.004)),
            origin=Origin(xyz=(-0.006, -0.014, 0.026)),
            material=trim,
            name="focus_rail",
        )
        part.visual(
            Box((0.004, 0.016, 0.012)),
            origin=Origin(xyz=(-0.008, -0.021, 0.032)),
            material=trim,
            name="focus_support",
        )
        part.visual(
            Cylinder(radius=0.0026, length=0.012),
            origin=Origin(xyz=(0.0, -0.026, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim,
            name="focus_axle",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.050, 0.108, 0.040)),
        mass=0.11,
        origin=Origin(xyz=(side * 0.026, -0.001, 0.012)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_porro_binocular", assets=ASSETS)

    armor = model.material("armor", rgba=(0.12, 0.12, 0.13, 1.0))
    trim = model.material("trim", rgba=(0.24, 0.24, 0.26, 1.0))
    wheel_mat = model.material("focus_wheel", rgba=(0.16, 0.16, 0.17, 1.0))

    _add_barrel(model, "left_barrel", -1.0, armor, trim, carries_bridge=True)
    _add_barrel(model, "right_barrel", 1.0, armor, trim)

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.0088, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_mat,
        name="focus_drum",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Box((0.010, 0.018, 0.018)),
        mass=0.018,
        origin=Origin(),
    )

    left_barrel = model.get_part("left_barrel")
    hinge_origin = Origin(xyz=(0.0, -0.003, 0.020))
    focus_origin = Origin(xyz=(0.0, -0.026, 0.034))

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child="right_barrel",
        origin=hinge_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.12,
            upper=0.12,
        ),
    )
    model.articulation(
        "focus_wheel_joint",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=focus_wheel,
        origin=focus_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=6.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_wheel = object_model.get_part("focus_wheel")

    center_hinge = object_model.get_articulation("center_hinge")
    focus_joint = object_model.get_articulation("focus_wheel_joint")

    hinge_axle = left_barrel.get_visual("hinge_axle")
    focus_axle = left_barrel.get_visual("focus_axle")
    left_lower_knuckle = left_barrel.get_visual("hinge_lower_knuckle")
    left_upper_knuckle = left_barrel.get_visual("hinge_upper_knuckle")
    left_objective = left_barrel.get_visual("objective_tube")
    left_eyecup = left_barrel.get_visual("eyecup")

    right_center_knuckle = right_barrel.get_visual("hinge_center_knuckle")
    right_objective = right_barrel.get_visual("objective_tube")
    right_eyecup = right_barrel.get_visual("eyecup")

    focus_drum = focus_wheel.get_visual("focus_drum")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.012)
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        left_barrel,
        right_barrel,
        elem_a=hinge_axle,
        elem_b=right_center_knuckle,
        reason="The rotating barrel sleeve wraps around the fixed hinge axle carried by the left barrel.",
    )
    ctx.allow_overlap(
        left_barrel,
        focus_wheel,
        elem_a=focus_axle,
        elem_b=focus_drum,
        reason="The focus wheel drum is modeled around its spindle.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "center_hinge_axis_is_vertical",
        tuple(round(v, 6) for v in center_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"center hinge axis is {center_hinge.axis}, expected (0, 0, 1)",
    )
    ctx.check(
        "focus_wheel_axis_is_lateral",
        tuple(round(v, 6) for v in focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"focus wheel axis is {focus_joint.axis}, expected (1, 0, 0)",
    )

    ctx.expect_overlap(
        left_barrel,
        left_barrel,
        axes="xy",
        min_overlap=0.007,
        elem_a=hinge_axle,
        elem_b=left_lower_knuckle,
        name="left_lower_knuckle_is_coaxial_with_hinge_axle",
    )
    ctx.expect_overlap(
        left_barrel,
        left_barrel,
        axes="xy",
        min_overlap=0.007,
        elem_a=hinge_axle,
        elem_b=left_upper_knuckle,
        name="left_upper_knuckle_is_coaxial_with_hinge_axle",
    )
    ctx.expect_overlap(
        left_barrel,
        right_barrel,
        axes="xy",
        min_overlap=0.007,
        elem_a=hinge_axle,
        elem_b=right_center_knuckle,
        name="right_barrel_sleeve_is_coaxial_with_hinge_axle",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=right_center_knuckle,
        negative_elem=left_lower_knuckle,
        name="right_center_knuckle_sits_on_left_lower_knuckle_face",
    )
    ctx.expect_gap(
        left_barrel,
        right_barrel,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=left_upper_knuckle,
        negative_elem=right_center_knuckle,
        name="right_center_knuckle_sits_under_left_upper_knuckle_face",
    )

    ctx.expect_overlap(
        left_barrel,
        right_barrel,
        axes="yz",
        min_overlap=0.014,
        elem_a=left_objective,
        elem_b=right_objective,
        name="objective_tubes_share_foreaft_and_height_alignment",
    )
    ctx.expect_overlap(
        left_barrel,
        right_barrel,
        axes="yz",
        min_overlap=0.006,
        elem_a=left_eyecup,
        elem_b=right_eyecup,
        name="eyecups_form_a_matched_pair",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="x",
        min_gap=0.050,
        max_gap=0.065,
        positive_elem=right_objective,
        negative_elem=left_objective,
        name="objective_tubes_are_wider_than_the_eyepiece_spacing",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="x",
        min_gap=0.010,
        max_gap=0.020,
        positive_elem=right_eyecup,
        negative_elem=left_eyecup,
        name="eyecups_stay_close_for_compact_reverse_porro_layout",
    )
    ctx.expect_overlap(
        left_barrel,
        focus_wheel,
        axes="yz",
        min_overlap=0.005,
        elem_a=focus_axle,
        elem_b=focus_drum,
        name="focus_wheel_is_centered_on_its_spindle",
    )
    ctx.expect_gap(
        focus_wheel,
        left_barrel,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=focus_drum,
        negative_elem=left_eyecup,
        name="focus_wheel_sits_just_right_of_left_eyecup",
    )
    ctx.expect_gap(
        right_barrel,
        focus_wheel,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=right_eyecup,
        negative_elem=focus_drum,
        name="focus_wheel_sits_just_left_of_right_eyecup",
    )

    hinge_limits = center_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({center_hinge: hinge_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="center_hinge_lower_no_unexpected_overlap")
            ctx.fail_if_isolated_parts(name="center_hinge_lower_no_floating")
            ctx.expect_gap(
                right_barrel,
                left_barrel,
                axis="x",
                min_gap=0.006,
                max_gap=0.014,
                positive_elem=right_eyecup,
                negative_elem=left_eyecup,
                name="folded_pose_keeps_eyecups_clear",
            )
            ctx.expect_overlap(
                left_barrel,
                right_barrel,
                axes="xy",
                min_overlap=0.007,
                elem_a=hinge_axle,
                elem_b=right_center_knuckle,
                name="folded_pose_keeps_right_barrel_on_hinge_axis",
            )
        with ctx.pose({center_hinge: hinge_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="center_hinge_upper_no_unexpected_overlap")
            ctx.fail_if_isolated_parts(name="center_hinge_upper_no_floating")
            ctx.expect_gap(
                right_barrel,
                left_barrel,
                axis="x",
                min_gap=0.040,
                positive_elem=right_objective,
                negative_elem=left_objective,
                name="opened_pose_keeps_objective_bells_clear",
            )

    focus_limits = focus_joint.motion_limits
    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({focus_joint: focus_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_joint_lower_no_unexpected_overlap")
            ctx.fail_if_isolated_parts(name="focus_joint_lower_no_floating")
            ctx.expect_overlap(
                left_barrel,
                focus_wheel,
                axes="yz",
                min_overlap=0.005,
                elem_a=focus_axle,
                elem_b=focus_drum,
                name="focus_wheel_lower_pose_stays_on_spindle",
            )
        with ctx.pose({focus_joint: focus_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_joint_upper_no_unexpected_overlap")
            ctx.fail_if_isolated_parts(name="focus_joint_upper_no_floating")
            ctx.expect_overlap(
                left_barrel,
                focus_wheel,
                axes="yz",
                min_overlap=0.005,
                elem_a=focus_axle,
                elem_b=focus_drum,
                name="focus_wheel_upper_pose_stays_on_spindle",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
