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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 72):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ).rotate_y(math.pi / 2.0),
        name,
    )


def _add_bolt_circle(
    part,
    *,
    prefix: str,
    center_x: float,
    circle_radius: float,
    bolt_radius: float,
    bolt_length: float,
    count: int,
    material,
) -> None:
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(
                    center_x,
                    circle_radius * math.cos(angle),
                    circle_radius * math.sin(angle),
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_floating_rear_axle")

    cast_steel = model.material("cast_steel", rgba=(0.33, 0.35, 0.37, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.08, 0.08, 0.09, 1.0))

    housing = model.part("axle_housing")

    banjo_shell = _x_shell_mesh(
        "banjo_shell",
        [
            (0.150, -0.220),
            (0.175, -0.170),
            (0.215, -0.110),
            (0.270, -0.055),
            (0.305, 0.000),
            (0.270, 0.055),
            (0.215, 0.110),
            (0.175, 0.170),
            (0.150, 0.220),
        ],
        [
            (0.110, -0.205),
            (0.132, -0.150),
            (0.155, -0.095),
            (0.185, -0.040),
            (0.200, 0.000),
            (0.185, 0.040),
            (0.155, 0.095),
            (0.132, 0.150),
            (0.110, 0.205),
        ],
    )
    tube_shell = _x_shell_mesh(
        "axle_tube_shell_v2",
        [
            (0.150, 0.000),
            (0.130, 0.045),
            (0.100, 0.090),
            (0.076, 0.130),
            (0.070, 0.480),
        ],
        [
            (0.050, 0.000),
            (0.050, 0.480),
        ],
    )
    spindle_shell = _x_shell_mesh(
        "spindle_shell_v3",
        [
            (0.088, 0.000),
            (0.088, 0.012),
            (0.080, 0.030),
            (0.072, 0.060),
            (0.068, 0.180),
            (0.080, 0.220),
        ],
        [
            (0.050, 0.000),
            (0.050, 0.220),
        ],
    )
    hub_body_shell = _x_shell_mesh(
        "hub_body_shell_v2",
        [
            (0.108, 0.000),
            (0.118, 0.030),
            (0.126, 0.110),
            (0.123, 0.180),
            (0.112, 0.220),
        ],
        [
            (0.088, 0.000),
            (0.088, 0.220),
        ],
    )
    wheel_flange_ring = _x_shell_mesh(
        "wheel_flange_ring_v2",
        [
            (0.160, 0.000),
            (0.160, 0.028),
        ],
        [
            (0.088, 0.000),
            (0.088, 0.028),
        ],
    )
    housing.visual(banjo_shell, material=cast_steel, name="banjo_shell")
    housing.visual(
        Cylinder(radius=0.205, length=0.016),
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="diff_cover_flange",
    )
    housing.visual(
        Cylinder(radius=0.176, length=0.060),
        origin=Origin(xyz=(0.0, -0.183, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="diff_cover",
    )
    housing.visual(
        Cylinder(radius=0.066, length=0.180),
        origin=Origin(xyz=(0.0, 0.182, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pinion_nose",
    )
    housing.visual(
        Cylinder(radius=0.086, length=0.026),
        origin=Origin(xyz=(0.0, 0.285, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pinion_yoke",
    )
    housing.visual(
        tube_shell,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material=cast_steel,
        name="right_tube_shell",
    )
    housing.visual(
        tube_shell,
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        material=cast_steel,
        name="left_tube_shell",
    )
    housing.visual(
        spindle_shell,
        origin=Origin(xyz=(0.700, 0.0, 0.0)),
        material=machined_steel,
        name="right_spindle_shell",
    )
    housing.visual(
        spindle_shell,
        origin=Origin(xyz=(-0.700, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        material=machined_steel,
        name="left_spindle_shell",
    )
    housing.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(-0.914, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_spindle_face",
    )
    housing.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.914, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_spindle_face",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((0.180, 0.110, 0.028)),
            origin=Origin(xyz=(side_sign * 0.450, 0.0, 0.084)),
            material=machined_steel,
            name=f"{side_name}_spring_pad",
        )

    left_hub = model.part("left_hub")
    left_hub.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="bearing_face",
    )
    left_hub.visual(
        Cylinder(radius=0.118, length=0.196),
        origin=Origin(xyz=(-0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_body",
    )
    left_hub.visual(
        Cylinder(radius=0.160, length=0.028),
        origin=Origin(xyz=(-0.214, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="wheel_flange",
    )
    _add_bolt_circle(
        left_hub,
        prefix="wheel_stud",
        center_x=-0.242,
        circle_radius=0.122,
        bolt_radius=0.009,
        bolt_length=0.028,
        count=8,
        material=fastener_black,
    )

    right_hub = model.part("right_hub")
    right_hub.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="bearing_face",
    )
    right_hub.visual(
        Cylinder(radius=0.118, length=0.196),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_body",
    )
    right_hub.visual(
        Cylinder(radius=0.160, length=0.028),
        origin=Origin(xyz=(0.214, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="wheel_flange",
    )
    _add_bolt_circle(
        right_hub,
        prefix="wheel_stud",
        center_x=0.242,
        circle_radius=0.122,
        bolt_radius=0.009,
        bolt_length=0.028,
        count=8,
        material=fastener_black,
    )

    left_shaft = model.part("left_half_shaft")
    left_shaft.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(-0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="shaft_body",
    )
    left_shaft.visual(
        Cylinder(radius=0.096, length=0.018),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="flange_plate",
    )
    _add_bolt_circle(
        left_shaft,
        prefix="flange_bolt",
        center_x=-0.026,
        circle_radius=0.074,
        bolt_radius=0.006,
        bolt_length=0.016,
        count=8,
        material=fastener_black,
    )

    right_shaft = model.part("right_half_shaft")
    right_shaft.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="shaft_body",
    )
    right_shaft.visual(
        Cylinder(radius=0.096, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="flange_plate",
    )
    _add_bolt_circle(
        right_shaft,
        prefix="flange_bolt",
        center_x=0.026,
        circle_radius=0.074,
        bolt_radius=0.006,
        bolt_length=0.016,
        count=8,
        material=fastener_black,
    )

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_hub,
        origin=Origin(xyz=(-0.920, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=20.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_hub,
        origin=Origin(xyz=(0.920, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=20.0),
    )
    model.articulation(
        "left_axle_withdrawal",
        ArticulationType.PRISMATIC,
        parent=left_hub,
        child=left_shaft,
        origin=Origin(xyz=(-0.228, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.15,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "right_axle_withdrawal",
        ArticulationType.PRISMATIC,
        parent=right_hub,
        child=right_shaft,
        origin=Origin(xyz=(0.228, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.15,
            lower=0.0,
            upper=0.220,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx = TestContext(object_model)
    housing = object_model.get_part("axle_housing")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_shaft = object_model.get_part("left_half_shaft")
    right_shaft = object_model.get_part("right_half_shaft")

    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")
    left_axle_withdrawal = object_model.get_articulation("left_axle_withdrawal")
    right_axle_withdrawal = object_model.get_articulation("right_axle_withdrawal")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_gap(
        housing,
        left_hub,
        axis="x",
        positive_elem="left_spindle_face",
        negative_elem="bearing_face",
        min_gap=0.0,
        max_gap=0.0,
        name="left_hub_bearing_face_seats_on_spindle_face",
    )
    ctx.expect_gap(
        right_hub,
        housing,
        axis="x",
        positive_elem="bearing_face",
        negative_elem="right_spindle_face",
        min_gap=0.0,
        max_gap=0.0,
        name="right_hub_bearing_face_seats_on_spindle_face",
    )
    ctx.expect_contact(left_shaft, left_hub, elem_a="flange_plate", elem_b="wheel_flange")
    ctx.expect_contact(right_shaft, right_hub, elem_a="flange_plate", elem_b="wheel_flange")
    ctx.expect_overlap(left_hub, housing, axes="yz", min_overlap=0.10)
    ctx.expect_overlap(right_hub, housing, axes="yz", min_overlap=0.10)
    ctx.expect_overlap(left_shaft, left_hub, axes="yz", min_overlap=0.07)
    ctx.expect_overlap(right_shaft, right_hub, axes="yz", min_overlap=0.07)

    left_stud_rest = ctx.part_element_world_aabb(left_hub, elem="wheel_stud_0")
    left_bolt_rest = ctx.part_element_world_aabb(left_shaft, elem="flange_bolt_0")
    assert left_stud_rest is not None
    assert left_bolt_rest is not None
    left_stud_rest_center = _aabb_center(left_stud_rest)
    left_bolt_rest_center = _aabb_center(left_bolt_rest)

    with ctx.pose({left_hub_spin: math.pi / 2.0, right_hub_spin: math.pi / 3.0}):
        ctx.expect_gap(
            housing,
            left_hub,
            axis="x",
            positive_elem="left_spindle_face",
            negative_elem="bearing_face",
            min_gap=0.0,
            max_gap=0.0,
            name="left_hub_bearing_face_seats_when_spun",
        )
        ctx.expect_gap(
            right_hub,
            housing,
            axis="x",
            positive_elem="bearing_face",
            negative_elem="right_spindle_face",
            min_gap=0.0,
            max_gap=0.0,
            name="right_hub_bearing_face_seats_when_spun",
        )

        left_stud_turn = ctx.part_element_world_aabb(left_hub, elem="wheel_stud_0")
        left_bolt_turn = ctx.part_element_world_aabb(left_shaft, elem="flange_bolt_0")
        assert left_stud_turn is not None
        assert left_bolt_turn is not None
        left_stud_turn_center = _aabb_center(left_stud_turn)
        left_bolt_turn_center = _aabb_center(left_bolt_turn)

        assert abs(left_stud_turn_center[1]) < 0.03
        assert abs(left_stud_turn_center[2] - left_stud_rest_center[1]) < 0.03
        assert abs(left_bolt_turn_center[1]) < 0.03
        assert abs(left_bolt_turn_center[2] - left_bolt_rest_center[1]) < 0.03

    withdrawal = left_axle_withdrawal.motion_limits
    assert withdrawal is not None
    assert withdrawal.upper is not None
    ctx.allow_isolated_part(
        left_shaft,
        reason="Service pose allows the left half-shaft to withdraw outward once its flange bolts are removed.",
    )
    ctx.allow_isolated_part(
        right_shaft,
        reason="Service pose allows the right half-shaft to withdraw outward once its flange bolts are removed.",
    )
    rest_left_pos = ctx.part_world_position(left_shaft)
    rest_right_pos = ctx.part_world_position(right_shaft)
    assert rest_left_pos is not None
    assert rest_right_pos is not None
    with ctx.pose(
        {
            left_axle_withdrawal: withdrawal.upper,
            right_axle_withdrawal: withdrawal.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="withdrawn_shafts_no_overlap")
        ctx.fail_if_isolated_parts(name="withdrawn_pose_only_service_shafts_may_disconnect")
        ctx.expect_gap(
            housing,
            left_hub,
            axis="x",
            positive_elem="left_spindle_face",
            negative_elem="bearing_face",
            min_gap=0.0,
            max_gap=0.0,
            name="left_hub_stays_seated_with_shaft_withdrawn",
        )
        ctx.expect_gap(
            right_hub,
            housing,
            axis="x",
            positive_elem="bearing_face",
            negative_elem="right_spindle_face",
            min_gap=0.0,
            max_gap=0.0,
            name="right_hub_stays_seated_with_shaft_withdrawn",
        )
        ctx.expect_gap(
            left_hub,
            left_shaft,
            axis="x",
            positive_elem="wheel_flange",
            negative_elem="flange_plate",
            min_gap=withdrawal.upper - 0.003,
            max_gap=withdrawal.upper + 0.003,
            name="left_shaft_withdrawal_gap",
        )
        ctx.expect_gap(
            right_shaft,
            right_hub,
            axis="x",
            positive_elem="flange_plate",
            negative_elem="wheel_flange",
            min_gap=withdrawal.upper - 0.003,
            max_gap=withdrawal.upper + 0.003,
            name="right_shaft_withdrawal_gap",
        )

        withdrawn_left_pos = ctx.part_world_position(left_shaft)
        withdrawn_right_pos = ctx.part_world_position(right_shaft)
        assert withdrawn_left_pos is not None
        assert withdrawn_right_pos is not None
        assert withdrawn_left_pos[0] < rest_left_pos[0] - 0.18
        assert withdrawn_right_pos[0] > rest_right_pos[0] + 0.18

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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
