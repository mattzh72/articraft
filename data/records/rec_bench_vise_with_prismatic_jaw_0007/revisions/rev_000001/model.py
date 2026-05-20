from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

JAW_WIDTH = 0.320
JAW_FACE_HEIGHT = 0.110
JAW_FACE_CENTER_Z = 0.165
LINER_HEIGHT = 0.104
GUIDE_BAR_RADIUS = 0.012
GUIDE_BAR_LENGTH = 0.460
GUIDE_BAR_CENTER_X = 0.270
TOP_GUIDE_Z = 0.090
BOTTOM_GUIDE_Z = 0.030
SCREW_Z = 0.060


def _x_axis_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _y_axis_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[idx] + upper[idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_woodworking_vise")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.29, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    hardwood = model.material("hardwood", rgba=(0.66, 0.50, 0.28, 1.0))
    lever_red = model.material("lever_red", rgba=(0.72, 0.12, 0.09, 1.0))
    phenolic_black = model.material("phenolic_black", rgba=(0.11, 0.11, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.020, JAW_WIDTH, JAW_FACE_HEIGHT)),
        origin=Origin(xyz=(0.010, 0.0, JAW_FACE_CENTER_Z)),
        material=cast_iron,
        name="fixed_jaw_plate",
    )
    base.visual(
        Box((0.018, JAW_WIDTH - 0.006, LINER_HEIGHT)),
        origin=Origin(xyz=(0.029, 0.0, JAW_FACE_CENTER_Z)),
        material=hardwood,
        name="fixed_jaw_liner",
    )
    base.visual(
        Box((0.040, 0.060, 0.110)),
        origin=Origin(xyz=(0.020, 0.060, 0.055)),
        material=cast_iron,
        name="right_jaw_post",
    )
    base.visual(
        Box((0.040, 0.060, 0.110)),
        origin=Origin(xyz=(0.020, -0.060, 0.055)),
        material=cast_iron,
        name="left_jaw_post",
    )
    base.visual(
        Box((0.050, 0.180, 0.024)),
        origin=Origin(xyz=(0.025, 0.0, TOP_GUIDE_Z)),
        material=cast_iron,
        name="top_bridge",
    )
    base.visual(
        Box((0.050, 0.180, 0.024)),
        origin=Origin(xyz=(0.025, 0.0, BOTTOM_GUIDE_Z)),
        material=cast_iron,
        name="bottom_bridge",
    )
    base.visual(
        Box((0.050, 0.120, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.078)),
        material=cast_iron,
        name="screw_housing_upper",
    )
    base.visual(
        Box((0.050, 0.120, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.042)),
        material=cast_iron,
        name="screw_housing_lower",
    )
    base.visual(
        Box((0.050, 0.014, 0.024)),
        origin=Origin(xyz=(0.025, 0.053, SCREW_Z)),
        material=cast_iron,
        name="screw_housing_right",
    )
    base.visual(
        Box((0.050, 0.014, 0.024)),
        origin=Origin(xyz=(0.025, -0.053, SCREW_Z)),
        material=cast_iron,
        name="screw_housing_left",
    )
    base.visual(
        Box((0.050, 0.020, 0.110)),
        origin=Origin(xyz=(0.025, 0.080, 0.055)),
        material=cast_iron,
        name="right_side_rib",
    )
    base.visual(
        Box((0.050, 0.020, 0.110)),
        origin=Origin(xyz=(0.025, -0.080, 0.055)),
        material=cast_iron,
        name="left_side_rib",
    )
    base.visual(
        Box((0.044, 0.220, 0.016)),
        origin=Origin(xyz=(0.012, 0.0, 0.008)),
        material=cast_iron,
        name="mount_flange",
    )
    base.visual(
        Box((0.036, 0.028, 0.044)),
        origin=Origin(xyz=(0.012, 0.104, SCREW_Z)),
        material=cast_iron,
        name="release_lug",
    )
    base.visual(
        Box((0.020, 0.012, 0.030)),
        origin=Origin(xyz=(0.020, 0.118, SCREW_Z)),
        material=cast_iron,
        name="release_neck",
    )
    top_bar_geom, top_bar_origin = _x_axis_cylinder(GUIDE_BAR_RADIUS, GUIDE_BAR_LENGTH, (GUIDE_BAR_CENTER_X, 0.0, TOP_GUIDE_Z))
    base.visual(top_bar_geom, origin=top_bar_origin, material=machined_steel, name="top_guide_bar")
    bottom_bar_geom, bottom_bar_origin = _x_axis_cylinder(GUIDE_BAR_RADIUS, GUIDE_BAR_LENGTH, (GUIDE_BAR_CENTER_X, 0.0, BOTTOM_GUIDE_Z))
    base.visual(bottom_bar_geom, origin=bottom_bar_origin, material=machined_steel, name="bottom_guide_bar")
    pivot_geom, pivot_origin = _y_axis_cylinder(0.008, 0.020, (0.020, 0.134, SCREW_Z))
    base.visual(pivot_geom, origin=pivot_origin, material=cast_iron, name="pivot_boss")
    base.inertial = Inertial.from_geometry(
        Box((0.220, 0.320, 0.220)),
        mass=15.0,
        origin=Origin(xyz=(0.055, 0.0, 0.110)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.024, JAW_WIDTH, JAW_FACE_HEIGHT)),
        origin=Origin(xyz=(0.012, 0.0, JAW_FACE_CENTER_Z)),
        material=cast_iron,
        name="front_jaw_plate",
    )
    moving_jaw.visual(
        Box((0.018, JAW_WIDTH - 0.006, LINER_HEIGHT)),
        origin=Origin(xyz=(0.009, 0.0, JAW_FACE_CENTER_Z)),
        material=hardwood,
        name="moving_jaw_liner",
    )
    moving_jaw.visual(
        Box((0.024, 0.030, 0.110)),
        origin=Origin(xyz=(0.024, 0.055, 0.055)),
        material=cast_iron,
        name="right_jaw_cheek",
    )
    moving_jaw.visual(
        Box((0.024, 0.030, 0.110)),
        origin=Origin(xyz=(0.024, -0.055, 0.055)),
        material=cast_iron,
        name="left_jaw_cheek",
    )
    moving_jaw.visual(
        Box((0.180, 0.020, 0.100)),
        origin=Origin(xyz=(0.114, 0.025, 0.060)),
        material=cast_iron,
        name="right_side_web",
    )
    moving_jaw.visual(
        Box((0.180, 0.020, 0.100)),
        origin=Origin(xyz=(0.114, -0.025, 0.060)),
        material=cast_iron,
        name="left_side_web",
    )
    moving_jaw.visual(
        Box((0.080, 0.050, 0.006)),
        origin=Origin(xyz=(0.090, 0.0, 0.073)),
        material=cast_iron,
        name="screw_boss_upper",
    )
    moving_jaw.visual(
        Box((0.080, 0.050, 0.006)),
        origin=Origin(xyz=(0.090, 0.0, 0.047)),
        material=cast_iron,
        name="screw_boss_lower",
    )
    moving_jaw.visual(
        Box((0.080, 0.006, 0.020)),
        origin=Origin(xyz=(0.090, 0.013, SCREW_Z)),
        material=cast_iron,
        name="screw_boss_right",
    )
    moving_jaw.visual(
        Box((0.080, 0.006, 0.020)),
        origin=Origin(xyz=(0.090, -0.013, SCREW_Z)),
        material=cast_iron,
        name="screw_boss_left",
    )
    for name, z_value, z_upper, z_lower in (
        ("top", TOP_GUIDE_Z, 0.105, 0.075),
        ("bottom", BOTTOM_GUIDE_Z, 0.045, 0.015),
    ):
        moving_jaw.visual(
            Box((0.110, 0.050, 0.006)),
            origin=Origin(xyz=(0.095, 0.0, z_upper)),
            material=cast_iron,
            name=f"{name}_sleeve_upper",
        )
        moving_jaw.visual(
            Box((0.110, 0.050, 0.006)),
            origin=Origin(xyz=(0.095, 0.0, z_lower)),
            material=cast_iron,
            name=f"{name}_sleeve_lower",
        )
        moving_jaw.visual(
            Box((0.110, 0.006, 0.024)),
            origin=Origin(xyz=(0.095, 0.022, z_value)),
            material=cast_iron,
            name=f"{name}_sleeve_right",
        )
        moving_jaw.visual(
            Box((0.110, 0.006, 0.024)),
            origin=Origin(xyz=(0.095, -0.022, z_value)),
            material=cast_iron,
            name=f"{name}_sleeve_left",
        )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.220, 0.320, 0.220)),
        mass=7.5,
        origin=Origin(xyz=(0.090, 0.0, 0.110)),
    )

    screw = model.part("screw")
    shaft_geom, shaft_origin = _x_axis_cylinder(0.010, 0.340, (-0.120, 0.0, 0.0))
    screw.visual(shaft_geom, origin=shaft_origin, material=machined_steel, name="shaft")
    nose_geom, nose_origin = _x_axis_cylinder(0.010, 0.120, (0.110, 0.0, 0.0))
    screw.visual(nose_geom, origin=nose_origin, material=machined_steel, name="nose")
    hub_geom, hub_origin = _x_axis_cylinder(0.014, 0.030, (0.180, 0.0, 0.0))
    screw.visual(hub_geom, origin=hub_origin, material=machined_steel, name="hub")
    spinner_geom, spinner_origin = _y_axis_cylinder(0.003, 0.012, (0.180, 0.006, 0.0))
    screw.visual(spinner_geom, origin=spinner_origin, material=machined_steel, name="spinner_arm")
    screw.visual(Sphere(radius=0.004), origin=Origin(xyz=(0.180, 0.012, 0.0)), material=phenolic_black, name="spinner_knob")
    screw.inertial = Inertial.from_geometry(
        Box((0.420, 0.080, 0.050)),
        mass=2.4,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
    )

    release_lever = model.part("release_lever")
    lever_hub_geom, lever_hub_origin = _y_axis_cylinder(0.008, 0.014, (0.0, 0.007, 0.0))
    release_lever.visual(lever_hub_geom, origin=lever_hub_origin, material=lever_red, name="lever_hub")
    release_lever.visual(
        Box((0.060, 0.012, 0.014)),
        origin=Origin(xyz=(-0.034, 0.014, 0.0)),
        material=lever_red,
        name="lever_arm",
    )
    release_lever.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(-0.068, 0.014, 0.0)),
        material=phenolic_black,
        name="lever_knob",
    )
    release_lever.visual(
        Box((0.018, 0.014, 0.012)),
        origin=Origin(xyz=(-0.022, 0.008, -0.013)),
        material=lever_red,
        name="release_cam",
    )
    release_lever.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(-0.030, 0.012, -0.004)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.28, lower=0.0, upper=0.250),
    )
    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw,
        origin=Origin(xyz=(0.090, 0.0, SCREW_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=7.0),
    )
    model.articulation(
        "release_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=release_lever,
        origin=Origin(xyz=(0.020, 0.144, SCREW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    base = object_model.get_part("base")
    moving_jaw = object_model.get_part("moving_jaw")
    screw = object_model.get_part("screw")
    release_lever = object_model.get_part("release_lever")

    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")
    release_pivot = object_model.get_articulation("release_pivot")

    fixed_jaw_liner = base.get_visual("fixed_jaw_liner")
    top_guide_bar = base.get_visual("top_guide_bar")
    bottom_guide_bar = base.get_visual("bottom_guide_bar")
    screw_housing_upper = base.get_visual("screw_housing_upper")
    pivot_boss = base.get_visual("pivot_boss")
    release_lug = base.get_visual("release_lug")

    moving_jaw_liner = moving_jaw.get_visual("moving_jaw_liner")
    top_sleeve_upper = moving_jaw.get_visual("top_sleeve_upper")
    bottom_sleeve_lower = moving_jaw.get_visual("bottom_sleeve_lower")
    screw_boss_upper = moving_jaw.get_visual("screw_boss_upper")

    screw_shaft = screw.get_visual("shaft")
    spinner_knob = screw.get_visual("spinner_knob")
    top_sleeve_lower = moving_jaw.get_visual("top_sleeve_lower")
    bottom_sleeve_upper = moving_jaw.get_visual("bottom_sleeve_upper")

    lever_hub = release_lever.get_visual("lever_hub")
    lever_knob = release_lever.get_visual("lever_knob")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_overlap(moving_jaw, base, axes="yz", elem_a=moving_jaw_liner, elem_b=fixed_jaw_liner, min_overlap=0.100)
    ctx.expect_gap(
        moving_jaw,
        base,
        axis="x",
        positive_elem=moving_jaw_liner,
        negative_elem=fixed_jaw_liner,
        min_gap=0.003,
        max_gap=0.005,
    )

    ctx.expect_contact(base, moving_jaw, elem_a=top_guide_bar, elem_b=top_sleeve_upper)
    ctx.expect_contact(base, moving_jaw, elem_a=top_guide_bar, elem_b=top_sleeve_lower)
    ctx.expect_contact(base, moving_jaw, elem_a=bottom_guide_bar, elem_b=bottom_sleeve_upper)
    ctx.expect_contact(base, moving_jaw, elem_a=bottom_guide_bar, elem_b=bottom_sleeve_lower)
    ctx.expect_overlap(base, moving_jaw, axes="x", elem_a=top_guide_bar, elem_b=top_sleeve_upper, min_overlap=0.100)
    ctx.expect_overlap(base, moving_jaw, axes="x", elem_a=bottom_guide_bar, elem_b=bottom_sleeve_lower, min_overlap=0.100)

    ctx.expect_contact(moving_jaw, screw, elem_a=screw_boss_upper, elem_b=screw_shaft)
    ctx.expect_overlap(base, screw, axes="xy", elem_a=screw_housing_upper, elem_b=screw_shaft, min_overlap=0.010)

    ctx.expect_contact(base, release_lever, elem_a=pivot_boss, elem_b=lever_hub)
    ctx.expect_gap(release_lever, base, axis="y", positive_elem=lever_knob, negative_elem=release_lug, min_gap=0.020)

    jaw_limits = jaw_slide.motion_limits
    if jaw_limits is not None and jaw_limits.lower is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_slide: jaw_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="jaw_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="jaw_slide_lower_no_floating")
        with ctx.pose({jaw_slide: jaw_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="jaw_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="jaw_slide_upper_no_floating")
            ctx.expect_gap(
                moving_jaw,
                base,
                axis="x",
                positive_elem=moving_jaw_liner,
                negative_elem=fixed_jaw_liner,
                min_gap=0.253,
                max_gap=0.255,
                name="jaw_slide_full_open_gap",
            )
            ctx.expect_contact(base, moving_jaw, elem_a=top_guide_bar, elem_b=top_sleeve_upper, name="jaw_slide_full_open_top_support")
            ctx.expect_contact(base, moving_jaw, elem_a=bottom_guide_bar, elem_b=bottom_sleeve_lower, name="jaw_slide_full_open_bottom_support")

    lever_limits = release_pivot.motion_limits
    if lever_limits is not None and lever_limits.lower is not None and lever_limits.upper is not None:
        with ctx.pose({release_pivot: lever_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="release_pivot_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="release_pivot_lower_no_floating")
        lever_rest_center = _aabb_center(ctx.part_element_world_aabb(release_lever, elem=lever_knob))
        with ctx.pose({release_pivot: lever_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="release_pivot_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="release_pivot_upper_no_floating")
            lever_open_center = _aabb_center(ctx.part_element_world_aabb(release_lever, elem=lever_knob))
            lever_motion_ok = (
                lever_rest_center is not None
                and lever_open_center is not None
                and abs(lever_open_center[1] - lever_rest_center[1]) < 0.003
                and lever_open_center[2] > lever_rest_center[2] + 0.045
            )
            ctx.check(
                "release_lever_swings_up",
                lever_motion_ok,
                details=f"rest={lever_rest_center}, open={lever_open_center}",
            )

    screw_rest_center = _aabb_center(ctx.part_element_world_aabb(screw, elem=spinner_knob))
    with ctx.pose({screw_spin: pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="screw_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="screw_quarter_turn_no_floating")
        ctx.expect_contact(moving_jaw, screw, elem_a=screw_boss_upper, elem_b=screw_shaft, name="screw_quarter_turn_supported")
        screw_quarter_center = _aabb_center(ctx.part_element_world_aabb(screw, elem=spinner_knob))
        screw_motion_ok = (
            screw_rest_center is not None
            and screw_quarter_center is not None
            and screw_rest_center[1] > 0.008
            and abs(screw_quarter_center[1]) < 0.005
            and screw_quarter_center[2] > screw_rest_center[2] + 0.007
        )
        ctx.check(
            "screw_spin_rotates_handle_bar",
            screw_motion_ok,
            details=f"rest={screw_rest_center}, quarter={screw_quarter_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
