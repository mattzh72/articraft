from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _rect_loop(size_x: float, size_y: float, z: float) -> list[tuple[float, float, float]]:
    half_x = size_x * 0.5
    half_y = size_y * 0.5
    return [
        (-half_x, -half_y, z),
        (half_x, -half_y, z),
        (half_x, half_y, z),
        (-half_x, half_y, z),
    ]


def _build_tapered_slide_mesh(
    *,
    bottom_x: float,
    bottom_y: float,
    top_x: float,
    top_y: float,
    height: float,
):
    return section_loft(
        [
            _rect_loop(bottom_x, bottom_y, 0.0),
            _rect_loop(top_x, top_y, height),
        ]
    )


def _build_handwheel_mesh(
    *,
    ring_radius: float,
    ring_tube: float,
    hub_radius: float,
    hub_length: float,
    spoke_width: float,
    spoke_thickness: float,
):
    wheel = TorusGeometry(
        radius=ring_radius,
        tube=ring_tube,
        radial_segments=18,
        tubular_segments=48,
    )
    wheel.merge(
        CylinderGeometry(
            radius=hub_radius,
            height=hub_length,
            radial_segments=24,
        )
    )

    spoke_span = (ring_radius - ring_tube * 0.55) * 2.0
    for angle in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0):
        spoke = BoxGeometry((spoke_span, spoke_width, spoke_thickness))
        spoke.rotate_z(angle)
        wheel.merge(spoke)

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_milling_vise")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.24, 0.28, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.26, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_block",
    )
    base.visual(
        Box((0.030, 0.160, 0.014)),
        origin=Origin(xyz=(0.115, 0.0, 0.057)),
        material=cast_iron,
        name="lower_fixed_jaw_plinth",
    )
    base.visual(
        Box((0.018, 0.136, 0.074)),
        origin=Origin(xyz=(0.114, 0.0, 0.087)),
        material=ground_steel,
        name="lower_fixed_jaw",
    )

    x_slide = model.part("x_slide")
    x_slide.visual(
        mesh_from_geometry(
            _build_tapered_slide_mesh(
                bottom_x=0.14,
                bottom_y=0.16,
                top_x=0.14,
                top_y=0.16,
                height=0.024,
            ),
            "x_slide_body_v5",
        ),
        origin=Origin(),
        material=cast_iron,
        name="lower_slide_body",
    )
    x_slide.visual(
        Box((0.116, 0.130, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=cast_iron,
        name="upper_slide_plinth",
    )
    x_slide.visual(
        Box((0.018, 0.136, 0.075)),
        origin=Origin(xyz=(0.078, 0.0, 0.0615)),
        material=ground_steel,
        name="lower_movable_jaw",
    )
    x_slide.visual(
        Box((0.110, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.071, 0.0595)),
        material=ground_steel,
        name="upper_fixed_jaw",
    )
    x_slide.visual(
        Box((0.032, 0.009, 0.014)),
        origin=Origin(xyz=(0.0, -0.0845, 0.024)),
        material=cast_iron,
        name="y_feed_bearing_housing",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_geometry(
            _build_tapered_slide_mesh(
                bottom_x=0.11,
                bottom_y=0.09,
                top_x=0.102,
                top_y=0.082,
                height=0.020,
            ),
            "y_slide_body_v2",
        ),
        origin=Origin(),
        material=cast_iron,
        name="upper_slide_body",
    )
    y_slide.visual(
        Box((0.100, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.045, 0.0475)),
        material=ground_steel,
        name="upper_movable_jaw",
    )

    x_handwheel = model.part("x_handwheel")
    x_handwheel.visual(
        mesh_from_geometry(
            _build_handwheel_mesh(
                ring_radius=0.0185,
                ring_tube=0.0035,
                hub_radius=0.0065,
                hub_length=0.008,
                spoke_width=0.0034,
                spoke_thickness=0.0030,
            ),
            "x_handwheel_v2",
        ),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="wheel_rim",
    )
    x_handwheel.visual(
        Cylinder(radius=0.008, length=0.009),
        origin=Origin(xyz=(0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ground_steel,
        name="wheel_shaft",
    )

    y_handwheel = model.part("y_handwheel")
    y_handwheel.visual(
        mesh_from_geometry(
            _build_handwheel_mesh(
                ring_radius=0.0155,
                ring_tube=0.0032,
                hub_radius=0.0058,
                hub_length=0.008,
                spoke_width=0.0030,
                spoke_thickness=0.0028,
            ),
            "y_handwheel_v2",
        ),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="wheel_rim",
    )
    y_handwheel.visual(
        Cylinder(radius=0.007, length=0.009),
        origin=Origin(xyz=(0.0, -0.0045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ground_steel,
        name="wheel_shaft",
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.08,
            lower=-0.045,
            upper=0.017,
        ),
    )
    model.articulation(
        "x_slide_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.06,
            lower=-0.030,
            upper=0.008,
        ),
    )
    model.articulation(
        "base_to_x_handwheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_handwheel,
        origin=Origin(xyz=(-0.138, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )
    model.articulation(
        "x_slide_to_y_handwheel",
        ArticulationType.CONTINUOUS,
        parent=x_slide,
        child=y_handwheel,
        origin=Origin(xyz=(0.0, -0.089, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_slide = object_model.get_part("y_slide")
    x_handwheel = object_model.get_part("x_handwheel")
    y_handwheel = object_model.get_part("y_handwheel")

    x_joint = object_model.get_articulation("base_to_x_slide")
    y_joint = object_model.get_articulation("x_slide_to_y_slide")
    x_wheel_joint = object_model.get_articulation("base_to_x_handwheel")
    y_wheel_joint = object_model.get_articulation("x_slide_to_y_handwheel")

    lower_fixed_jaw = base.get_visual("lower_fixed_jaw")
    lower_movable_jaw = x_slide.get_visual("lower_movable_jaw")
    upper_fixed_jaw = x_slide.get_visual("upper_fixed_jaw")
    upper_movable_jaw = y_slide.get_visual("upper_movable_jaw")
    lower_slide_body = x_slide.get_visual("lower_slide_body")
    y_feed_bearing_housing = x_slide.get_visual("y_feed_bearing_housing")
    base_block = base.get_visual("base_block")
    x_wheel_shaft = x_handwheel.get_visual("wheel_shaft")
    y_wheel_shaft = y_handwheel.get_visual("wheel_shaft")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "x_slide_is_prismatic_x",
        x_joint.joint_type == ArticulationType.PRISMATIC and tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        f"expected X-axis prismatic joint, got type={x_joint.joint_type} axis={x_joint.axis}",
    )
    ctx.check(
        "y_slide_is_prismatic_y",
        y_joint.joint_type == ArticulationType.PRISMATIC and tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        f"expected Y-axis prismatic joint, got type={y_joint.joint_type} axis={y_joint.axis}",
    )
    ctx.check(
        "x_handwheel_is_continuous",
        x_wheel_joint.joint_type == ArticulationType.CONTINUOUS and tuple(x_wheel_joint.axis) == (1.0, 0.0, 0.0),
        f"expected X handwheel continuous around X, got type={x_wheel_joint.joint_type} axis={x_wheel_joint.axis}",
    )
    ctx.check(
        "y_handwheel_is_continuous",
        y_wheel_joint.joint_type == ArticulationType.CONTINUOUS and tuple(y_wheel_joint.axis) == (0.0, 1.0, 0.0),
        f"expected Y handwheel continuous around Y, got type={y_wheel_joint.joint_type} axis={y_wheel_joint.axis}",
    )

    ctx.expect_contact(x_slide, base, name="x_slide_supported_on_base")
    ctx.expect_contact(y_slide, x_slide, name="y_slide_supported_on_x_slide")
    ctx.expect_contact(
        x_handwheel,
        base,
        elem_a=x_wheel_shaft,
        name="x_handwheel_shaft_mounted_to_base",
    )
    ctx.expect_contact(
        y_handwheel,
        x_slide,
        elem_a=y_wheel_shaft,
        elem_b=y_feed_bearing_housing,
        name="y_handwheel_shaft_mounted_to_x_slide",
    )

    ctx.expect_within(
        x_slide,
        base,
        axes="y",
        margin=0.0,
        inner_elem=lower_slide_body,
        outer_elem=base_block,
        name="x_slide_stays_on_base_width",
    )
    ctx.expect_within(y_slide, x_slide, axes="x", margin=0.0, name="y_slide_centered_over_x_slide")
    ctx.expect_overlap(
        base,
        x_slide,
        axes="xy",
        min_overlap=0.12,
        name="x_slide_has_broad_way_overlap",
    )
    ctx.expect_overlap(
        x_slide,
        y_slide,
        axes="xy",
        min_overlap=0.08,
        name="y_slide_has_broad_way_overlap",
    )

    ctx.expect_gap(
        base,
        x_slide,
        axis="x",
        min_gap=0.016,
        max_gap=0.0185,
        positive_elem=lower_fixed_jaw,
        negative_elem=lower_movable_jaw,
        name="lower_jaw_default_opening",
    )
    ctx.expect_overlap(
        base,
        x_slide,
        axes="yz",
        min_overlap=0.048,
        elem_a=lower_fixed_jaw,
        elem_b=lower_movable_jaw,
        name="lower_jaws_face_each_other",
    )
    ctx.expect_gap(
        x_slide,
        y_slide,
        axis="y",
        min_gap=0.007,
        max_gap=0.009,
        positive_elem=upper_fixed_jaw,
        negative_elem=upper_movable_jaw,
        name="upper_jaw_default_opening",
    )
    ctx.expect_overlap(
        x_slide,
        y_slide,
        axes="xz",
        min_overlap=0.034,
        elem_a=upper_fixed_jaw,
        elem_b=upper_movable_jaw,
        name="upper_jaws_face_each_other",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=48, name="articulation_clearance_sweep")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    x_limits = x_joint.motion_limits
    if x_limits is not None and x_limits.lower is not None and x_limits.upper is not None:
        with ctx.pose({x_joint: x_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="x_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="x_slide_lower_no_floating")
            ctx.expect_gap(
                base,
                x_slide,
                axis="x",
                min_gap=0.060,
                max_gap=0.064,
                positive_elem=lower_fixed_jaw,
                negative_elem=lower_movable_jaw,
                name="x_slide_lower_pose_opening",
            )
        with ctx.pose({x_joint: x_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="x_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="x_slide_upper_no_floating")
            ctx.expect_gap(
                base,
                x_slide,
                axis="x",
                max_gap=0.0015,
                max_penetration=1e-6,
                positive_elem=lower_fixed_jaw,
                negative_elem=lower_movable_jaw,
                name="x_slide_upper_pose_closing_gap",
            )

    y_limits = y_joint.motion_limits
    if y_limits is not None and y_limits.lower is not None and y_limits.upper is not None:
        with ctx.pose({y_joint: y_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="y_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="y_slide_lower_no_floating")
            ctx.expect_gap(
                x_slide,
                y_slide,
                axis="y",
                min_gap=0.037,
                max_gap=0.039,
                positive_elem=upper_fixed_jaw,
                negative_elem=upper_movable_jaw,
                name="y_slide_lower_pose_opening",
            )
        with ctx.pose({y_joint: y_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="y_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="y_slide_upper_no_floating")
            ctx.expect_gap(
                x_slide,
                y_slide,
                axis="y",
                max_gap=0.001,
                max_penetration=1e-6,
                positive_elem=upper_fixed_jaw,
                negative_elem=upper_movable_jaw,
                name="y_slide_upper_pose_closing_gap",
            )

    with ctx.pose({x_wheel_joint: 1.4, y_wheel_joint: -0.9}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handwheel_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="handwheel_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
