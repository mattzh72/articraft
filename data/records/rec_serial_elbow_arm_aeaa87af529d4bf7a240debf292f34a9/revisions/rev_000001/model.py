from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHOULDER_BARREL_RADIUS = 0.027
SHOULDER_BARREL_LENGTH = 0.032
ELBOW_BARREL_RADIUS = 0.022
ELBOW_BARREL_LENGTH = 0.026

PROXIMAL_LENGTH = 0.220
DISTAL_LENGTH = 0.145
FLANGE_RADIUS = 0.036
FLANGE_THICKNESS = 0.014


def _root_bracket_shape() -> cq.Workplane:
    back_plate = (
        cq.Workplane("XY")
        .box(0.018, 0.102, 0.164)
        .translate((-0.058, 0.0, 0.0))
    )

    upper_bridge = cq.Workplane("XY").box(0.036, 0.090, 0.024).translate((-0.040, 0.0, 0.054))
    lower_bridge = cq.Workplane("XY").box(0.044, 0.098, 0.030).translate((-0.044, 0.0, -0.056))

    left_ear = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.078)
        .translate((-0.025, 0.019, 0.0))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.078)
        .translate((-0.025, -0.019, 0.0))
    )

    wall_pad = (
        cq.Workplane("XY")
        .box(0.026, 0.102, 0.028)
        .translate((-0.061, 0.0, -0.078))
    )

    bracket = (
        back_plate.union(upper_bridge)
        .union(lower_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(wall_pad)
    )

    mount_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.048, -0.030),
                (0.048, -0.030),
                (-0.048, 0.030),
                (0.048, 0.030),
            ]
        )
        .circle(0.0065)
        .extrude(0.030, both=True)
        .translate((-0.056, 0.0, 0.0))
    )

    return bracket.cut(mount_holes)


def _proximal_link_shape() -> cq.Workplane:
    shoulder_housing = (
        cq.Workplane("XY")
        .box(0.050, 0.040, 0.068)
        .translate((0.025, 0.0, 0.0))
    )
    main_beam = (
        cq.Workplane("XY")
        .box(0.128, 0.046, 0.074)
        .translate((0.111, 0.0, 0.0))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(0.042, 0.028, 0.018)
        .translate((0.174, 0.0, 0.023))
    )
    lower_web = (
        cq.Workplane("XY")
        .box(0.042, 0.028, 0.018)
        .translate((0.174, 0.0, -0.023))
    )

    left_elbow_ear = (
        cq.Workplane("XY")
        .box(0.034, 0.012, 0.060)
        .translate((PROXIMAL_LENGTH - 0.017, 0.019, 0.0))
    )
    right_elbow_ear = (
        cq.Workplane("XY")
        .box(0.034, 0.012, 0.060)
        .translate((PROXIMAL_LENGTH - 0.017, -0.019, 0.0))
    )

    return (
        shoulder_housing.union(main_beam)
        .union(upper_web)
        .union(lower_web)
        .union(left_elbow_ear)
        .union(right_elbow_ear)
    )


def _distal_link_shape() -> cq.Workplane:
    rear_housing = (
        cq.Workplane("XY")
        .box(0.038, 0.034, 0.046)
        .translate((0.019, 0.0, 0.0))
    )
    arm_beam = (
        cq.Workplane("XY")
        .box(0.086, 0.036, 0.052)
        .translate((0.077, 0.0, 0.0))
    )
    nose_hub = (
        cq.Workplane("YZ")
        .circle(0.018)
        .extrude(0.030, both=True)
        .translate((0.124, 0.0, 0.0))
    )
    end_flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS, both=True)
        .translate((DISTAL_LENGTH, 0.0, 0.0))
    )
    flange_bore = (
        cq.Workplane("YZ")
        .circle(0.010)
        .extrude(FLANGE_THICKNESS + 0.010, both=True)
        .translate((DISTAL_LENGTH, 0.0, 0.0))
    )

    return (
        rear_housing.union(arm_beam)
        .union(nose_hub)
        .union(end_flange)
        .cut(flange_bore)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_arm")

    model.material("bracket_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("proximal_graphite", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("distal_silver", rgba=(0.73, 0.75, 0.78, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_root_bracket_shape(), "root_bracket"),
        material="bracket_dark",
        name="bracket_body",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.120, 0.102, 0.164)),
        mass=3.2,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(_proximal_link_shape(), "proximal_link"),
        material="proximal_graphite",
        name="proximal_body",
    )
    proximal_link.inertial = Inertial.from_geometry(
        Box((PROXIMAL_LENGTH, 0.050, 0.078)),
        mass=1.4,
        origin=Origin(xyz=(PROXIMAL_LENGTH * 0.5, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material="distal_silver",
        name="distal_body",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((DISTAL_LENGTH + FLANGE_THICKNESS, 0.072, 0.072)),
        mass=0.8,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.70,
            upper=1.20,
            effort=45.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.20,
            upper=1.25,
            effort=28.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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

    ctx.expect_contact(
        root_bracket,
        proximal_link,
        name="shoulder_joint_has_physical_mount_contact",
    )
    ctx.expect_contact(
        proximal_link,
        distal_link,
        name="elbow_joint_has_physical_mount_contact",
    )
    ctx.expect_overlap(
        root_bracket,
        proximal_link,
        axes="yz",
        min_overlap=0.030,
        name="shoulder_joint_aligned_in_yz",
    )
    ctx.expect_overlap(
        proximal_link,
        distal_link,
        axes="yz",
        min_overlap=0.024,
        name="elbow_joint_aligned_in_yz",
    )
    ctx.check(
        "joint_axes_define_single_pitch_plane",
        shoulder_joint.axis == (0.0, -1.0, 0.0) and elbow_joint.axis == (0.0, -1.0, 0.0),
        f"shoulder axis={shoulder_joint.axis}, elbow axis={elbow_joint.axis}",
    )

    rest_distal_aabb = ctx.part_world_aabb(distal_link)
    if rest_distal_aabb is None:
        ctx.fail("distal_link_has_world_aabb", "distal_link world AABB is unavailable in rest pose")
        return ctx.report()

    rest_distal_center_y = _aabb_center(rest_distal_aabb)[1]
    rest_distal_tip_z = rest_distal_aabb[1][2]

    with ctx.pose({shoulder_joint: 0.65, elbow_joint: 0.80}):
        raised_distal_aabb = ctx.part_world_aabb(distal_link)
        if raised_distal_aabb is None:
            ctx.fail("distal_link_has_world_aabb_in_raised_pose", "distal_link world AABB is unavailable in raised pose")
        else:
            raised_center = _aabb_center(raised_distal_aabb)
            ctx.check(
                "positive_joint_motion_raises_distal_link",
                raised_distal_aabb[1][2] > rest_distal_tip_z + 0.060,
                f"rest tip z={rest_distal_tip_z:.4f}, raised tip z={raised_distal_aabb[1][2]:.4f}",
            )
            ctx.check(
                "arm_motion_stays_in_one_plane",
                abs(raised_center[1] - rest_distal_center_y) < 1e-4,
                f"rest center y={rest_distal_center_y:.6f}, raised center y={raised_center[1]:.6f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
