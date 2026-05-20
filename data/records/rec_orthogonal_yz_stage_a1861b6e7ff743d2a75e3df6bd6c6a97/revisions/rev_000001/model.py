from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_Y = 0.22
SUPPORT_Z = 0.24
SUPPORT_T = 0.016

Y_RAIL_X = 0.022
Y_RAIL_Y = 0.16
Y_RAIL_Z = 0.036
Y_RAIL_ZC = 0.05

Y_BODY_X = 0.05
Y_BODY_Y = 0.076
Y_BODY_Z = 0.052
Y_TRAVEL = 0.03
Y_MOUNT_X = 0.073

Z_GUIDE_X = 0.022
Z_GUIDE_Y = 0.026
Z_GUIDE_Z = 0.18
Z_GUIDE_XC = 0.036
Z_GUIDE_ZC = -0.064

Z_SIDE_X = 0.016
Z_SIDE_Y = 0.008
Z_SIDE_Z = 0.152
Z_SIDE_XC = 0.043
Z_SIDE_ZC = -0.062
Z_SIDE_YC = 0.017

Z_CARRIAGE_SHOE_X = 0.034
Z_CARRIAGE_SHOE_Y = 0.03
Z_CARRIAGE_SHOE_Z = 0.034

Z_CARRIAGE_BODY_X = 0.04
Z_CARRIAGE_BODY_Y = 0.024
Z_CARRIAGE_BODY_Z = 0.052

Z_CARRIAGE_FOOT_X = 0.046
Z_CARRIAGE_FOOT_Y = 0.036
Z_CARRIAGE_FOOT_Z = 0.01

Z_TRAVEL = 0.09


def _support_shape() -> cq.Workplane:
    back_plate = (
        cq.Workplane("XY")
        .box(SUPPORT_T, SUPPORT_Y, SUPPORT_Z)
        .edges("|X")
        .fillet(0.006)
    )

    y_rail = (
        cq.Workplane("XY")
        .box(Y_RAIL_X, Y_RAIL_Y, Y_RAIL_Z)
        .translate((SUPPORT_T / 2 + Y_RAIL_X / 2, 0.0, Y_RAIL_ZC))
        .edges("|Y")
        .fillet(0.003)
    )

    upper_cap = (
        cq.Workplane("XY")
        .box(0.014, 0.028, 0.018)
        .translate((SUPPORT_T / 2 + 0.007, 0.0, Y_RAIL_ZC + 0.028))
        .edges("|Y")
        .fillet(0.002)
    )

    lower_gusset = (
        cq.Workplane("XZ")
        .moveTo(SUPPORT_T / 2, Y_RAIL_ZC - 0.010)
        .lineTo(SUPPORT_T / 2 + 0.040, Y_RAIL_ZC - 0.010)
        .lineTo(SUPPORT_T / 2 + 0.016, Y_RAIL_ZC - 0.050)
        .close()
        .extrude(0.048, both=True)
    )

    return back_plate.union(y_rail).union(upper_cap).union(lower_gusset)


def _y_slide_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(Y_BODY_X, Y_BODY_Y, Y_BODY_Z)
        .edges("|X")
        .fillet(0.004)
    )

    z_guide = (
        cq.Workplane("XY")
        .box(Z_GUIDE_X, Z_GUIDE_Y, Z_GUIDE_Z)
        .translate((Z_GUIDE_XC, 0.0, Z_GUIDE_ZC))
        .edges("|Z")
        .fillet(0.002)
    )

    side_way_pos = (
        cq.Workplane("XY")
        .box(Z_SIDE_X, Z_SIDE_Y, Z_SIDE_Z)
        .translate((Z_SIDE_XC, Z_SIDE_YC, Z_SIDE_ZC))
        .edges("|Z")
        .fillet(0.0015)
    )
    side_way_neg = (
        cq.Workplane("XY")
        .box(Z_SIDE_X, Z_SIDE_Y, Z_SIDE_Z)
        .translate((Z_SIDE_XC, -Z_SIDE_YC, Z_SIDE_ZC))
        .edges("|Z")
        .fillet(0.0015)
    )

    bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.046, 0.018)
        .translate((0.030, 0.0, -0.038))
        .edges("|Y")
        .fillet(0.0015)
    )

    return body.union(z_guide).union(side_way_pos).union(side_way_neg).union(bridge)


def _z_carriage_shape() -> cq.Workplane:
    shoe = (
        cq.Workplane("XY")
        .box(Z_CARRIAGE_SHOE_X, Z_CARRIAGE_SHOE_Y, Z_CARRIAGE_SHOE_Z)
        .translate((Z_CARRIAGE_SHOE_X / 2, 0.0, -Z_CARRIAGE_SHOE_Z / 2))
        .edges("|Z")
        .fillet(0.002)
    )

    body = (
        cq.Workplane("XY")
        .box(Z_CARRIAGE_BODY_X, Z_CARRIAGE_BODY_Y, Z_CARRIAGE_BODY_Z)
        .translate((0.022, 0.0, -0.060))
        .edges("|X")
        .fillet(0.002)
    )

    foot = (
        cq.Workplane("XY")
        .box(Z_CARRIAGE_FOOT_X, Z_CARRIAGE_FOOT_Y, Z_CARRIAGE_FOOT_Z)
        .translate((0.023, 0.0, -0.091))
        .edges("|Y")
        .fillet(0.0015)
    )

    nose = (
        cq.Workplane("XZ")
        .moveTo(0.006, -0.050)
        .lineTo(0.040, -0.050)
        .lineTo(0.034, -0.082)
        .lineTo(0.010, -0.082)
        .close()
        .extrude(0.020, both=True)
    )

    return shoe.union(body).union(foot).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_yz_stage")

    support_mat = model.material("support_gray", rgba=(0.39, 0.42, 0.46, 1.0))
    slide_mat = model.material("slide_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    carriage_mat = model.material("carriage_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    back_support = model.part("back_support")
    back_support.visual(
        mesh_from_cadquery(_support_shape(), "back_support"),
        origin=Origin(),
        material=support_mat,
        name="support_shell",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_y_slide_shape(), "y_slide"),
        origin=Origin(),
        material=slide_mat,
        name="slide_shell",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        mesh_from_cadquery(_z_carriage_shape(), "z_carriage"),
        origin=Origin(),
        material=carriage_mat,
        name="carriage_shell",
    )

    model.articulation(
        "support_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=back_support,
        child=y_slide,
        origin=Origin(
            xyz=(
                Y_MOUNT_X,
                0.0,
                Y_RAIL_ZC,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    model.articulation(
        "y_slide_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=z_carriage,
        origin=Origin(
            xyz=(
                Z_GUIDE_XC + Z_GUIDE_X / 2,
                0.0,
                -Y_BODY_Z / 2 - 0.002,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_support = object_model.get_part("back_support")
    y_slide = object_model.get_part("y_slide")
    z_carriage = object_model.get_part("z_carriage")
    y_joint = object_model.get_articulation("support_to_y_slide")
    z_joint = object_model.get_articulation("y_slide_to_z_carriage")

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
        "y_stage_is_prismatic_along_y",
        y_joint.joint_type == ArticulationType.PRISMATIC and tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected prismatic +Y stage, got type={y_joint.joint_type}, axis={y_joint.axis}",
    )
    ctx.check(
        "z_stage_is_prismatic_along_negative_z",
        z_joint.joint_type == ArticulationType.PRISMATIC and tuple(z_joint.axis) == (0.0, 0.0, -1.0),
        details=f"expected prismatic -Z stage, got type={z_joint.joint_type}, axis={z_joint.axis}",
    )
    ctx.check(
        "stages_are_orthogonal",
        sum(a * b for a, b in zip(y_joint.axis, z_joint.axis)) == 0.0,
        details=f"joint axes must be orthogonal, got {y_joint.axis} vs {z_joint.axis}",
    )

    ctx.expect_contact(
        y_slide,
        back_support,
        name="y_slide_is_supported_by_back_support",
    )
    ctx.expect_contact(
        z_carriage,
        y_slide,
        name="z_carriage_is_supported_by_y_slide",
    )
    ctx.expect_gap(
        z_carriage,
        back_support,
        axis="x",
        min_gap=0.06,
        name="z_carriage_hangs_in_front_of_support",
    )

    with ctx.pose({y_joint: Y_TRAVEL}):
        ctx.expect_contact(
            y_slide,
            back_support,
            name="y_slide_stays_supported_at_positive_y_limit",
        )

    with ctx.pose({y_joint: -Y_TRAVEL}):
        ctx.expect_contact(
            y_slide,
            back_support,
            name="y_slide_stays_supported_at_negative_y_limit",
        )

    with ctx.pose({z_joint: Z_TRAVEL}):
        ctx.expect_contact(
            z_carriage,
            y_slide,
            name="z_carriage_stays_supported_at_lower_z_limit",
        )

    y_rest = ctx.part_world_position(y_slide)
    z_rest = ctx.part_world_position(z_carriage)
    with ctx.pose({y_joint: Y_TRAVEL}):
        y_shifted = ctx.part_world_position(y_slide)
    with ctx.pose({z_joint: Z_TRAVEL}):
        z_lowered = ctx.part_world_position(z_carriage)

    y_ok = (
        y_rest is not None
        and y_shifted is not None
        and y_shifted[1] > y_rest[1] + 0.02
        and abs(y_shifted[0] - y_rest[0]) < 1e-6
        and abs(y_shifted[2] - y_rest[2]) < 1e-6
    )
    ctx.check(
        "y_stage_moves_laterally",
        y_ok,
        details=f"rest={y_rest}, shifted={y_shifted}",
    )

    z_ok = (
        z_rest is not None
        and z_lowered is not None
        and z_lowered[2] < z_rest[2] - 0.05
        and abs(z_lowered[0] - z_rest[0]) < 1e-6
        and abs(z_lowered[1] - z_rest[1]) < 1e-6
    )
    ctx.check(
        "z_stage_moves_vertically_downward",
        z_ok,
        details=f"rest={z_rest}, lowered={z_lowered}",
    )

    y_aabb = ctx.part_world_aabb(y_slide)
    z_aabb = ctx.part_world_aabb(z_carriage)
    smaller_ok = False
    if y_aabb is not None and z_aabb is not None:
        y_size = tuple(upper - lower for lower, upper in zip(y_aabb[0], y_aabb[1]))
        z_size = tuple(upper - lower for lower, upper in zip(z_aabb[0], z_aabb[1]))
        smaller_ok = (
            z_size[0] < y_size[0]
            and z_size[1] < y_size[1]
            and z_size[2] < y_size[2]
        )
    else:
        y_size = None
        z_size = None
    ctx.check(
        "z_carriage_is_visibly_smaller_than_y_slide",
        smaller_ok,
        details=f"y_slide_size={y_size}, z_carriage_size={z_size}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
