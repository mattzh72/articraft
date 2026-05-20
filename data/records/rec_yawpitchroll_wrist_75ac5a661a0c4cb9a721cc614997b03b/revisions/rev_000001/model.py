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


def cylinder_x(radius: float, length: float, x_start: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x_start, 0.0, 0.0))
    )


def annulus_x(
    inner_radius: float,
    outer_radius: float,
    length: float,
    x_start: float,
) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length + 0.002)
        .translate((-0.001, 0.0, 0.0))
    )
    return outer.cut(inner).translate((x_start, 0.0, 0.0))


def cylinder_y(radius: float, length: float, y_start: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((0.0, y_start, 0.0))
    )


def annulus_y(
    inner_radius: float,
    outer_radius: float,
    length: float,
    y_start: float,
) -> cq.Workplane:
    outer = cq.Workplane("XZ").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XZ")
        .circle(inner_radius)
        .extrude(length + 0.002)
        .translate((0.0, -0.001, 0.0))
    )
    return outer.cut(inner).translate((0.0, y_start, 0.0))


def cylinder_z(radius: float, length: float, z_start: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z_start))
    )


def annulus_z(
    inner_radius: float,
    outer_radius: float,
    length: float,
    z_start: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner).translate((0.0, 0.0, z_start))


def box_centered(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def build_root_base() -> cq.Workplane:
    rear_flange = cylinder_x(0.086, 0.022, -0.210)
    motor_barrel = cylinder_x(0.074, 0.160, -0.188)
    shoulder_block = box_centered((0.080, 0.110, 0.100), (-0.110, 0.0, 0.0))
    upper_spine = box_centered((0.055, 0.070, 0.022), (-0.050, 0.0, 0.050))
    lower_spine = box_centered((0.055, 0.070, 0.022), (-0.050, 0.0, -0.050))
    side_left = box_centered((0.045, 0.024, 0.050), (-0.060, 0.060, 0.0))
    side_right = box_centered((0.045, 0.024, 0.050), (-0.060, -0.060, 0.0))
    yaw_face = annulus_x(0.108, 0.146, 0.008, -0.008)

    return (
        rear_flange.union(motor_barrel)
        .union(shoulder_block)
        .union(upper_spine)
        .union(lower_spine)
        .union(side_left)
        .union(side_right)
        .union(yaw_face)
    )


def build_yaw_ring() -> cq.Workplane:
    rear_face = annulus_x(0.108, 0.146, 0.008, 0.0)
    left_arm = box_centered((0.102, 0.018, 0.064), (0.055, 0.100, 0.0))
    right_arm = box_centered((0.102, 0.018, 0.064), (0.055, -0.100, 0.0))
    top_rib = box_centered((0.062, 0.164, 0.014), (0.070, 0.0, 0.104))
    bottom_rib = box_centered((0.062, 0.164, 0.014), (0.070, 0.0, -0.104))
    left_boss = annulus_y(0.0185, 0.032, 0.016, 0.076).translate((0.104, 0.0, 0.0))
    right_boss = annulus_y(0.0185, 0.032, 0.016, -0.092).translate((0.104, 0.0, 0.0))

    return (
        rear_face.union(left_arm)
        .union(right_arm)
        .union(top_rib)
        .union(bottom_rib)
        .union(left_boss)
        .union(right_boss)
    )


def build_pitch_cradle() -> cq.Workplane:
    left_trunnion = cylinder_y(0.018, 0.016, 0.076)
    right_trunnion = cylinder_y(0.018, 0.016, -0.092)
    left_cheek = box_centered((0.086, 0.016, 0.082), (0.062, 0.060, 0.0))
    right_cheek = box_centered((0.086, 0.016, 0.082), (0.062, -0.060, 0.0))
    top_bridge = box_centered((0.044, 0.116, 0.014), (0.102, 0.0, 0.048))
    bottom_bridge = box_centered((0.044, 0.116, 0.014), (0.102, 0.0, -0.048))
    rear_bearing = annulus_x(0.038, 0.050, 0.008, 0.078)
    front_retainer = annulus_x(0.038, 0.052, 0.008, 0.118)

    return (
        left_trunnion.union(right_trunnion)
        .union(left_cheek)
        .union(right_cheek)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(rear_bearing)
        .union(front_retainer)
    )


def build_roll_cartridge() -> cq.Workplane:
    rear_journal = cylinder_x(0.040, 0.030, -0.030)
    rear_flange = cylinder_x(0.038, 0.008, -0.008)
    drive_shaft = cylinder_x(0.014, 0.112, 0.000)
    rotor_drum = cylinder_x(0.034, 0.064, 0.112)
    front_flange = cylinder_x(0.038, 0.008, 0.176)
    nose_body = cylinder_x(0.031, 0.092, 0.184)
    nose_collar = cylinder_x(0.036, 0.018, 0.276)
    output_stub = cylinder_x(0.014, 0.028, 0.294)

    return (
        rear_journal.union(rear_flange)
        .union(drive_shaft)
        .union(rotor_drum)
        .union(front_flange)
        .union(nose_body)
        .union(nose_collar)
        .union(output_stub)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_robot_wrist")

    root_dark = model.material("root_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    ring_dark = model.material("ring_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    cradle_gray = model.material("cradle_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.69, 0.72, 1.0))

    root = model.part("root_base")
    root.visual(
        mesh_from_cadquery(build_root_base(), "root_base_mesh"),
        material=root_dark,
        name="root_body",
    )

    yaw_ring = model.part("yaw_ring")
    yaw_ring.visual(
        mesh_from_cadquery(build_yaw_ring(), "yaw_ring_mesh"),
        material=ring_dark,
        name="yaw_frame",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(build_pitch_cradle(), "pitch_cradle_mesh"),
        material=cradle_gray,
        name="cradle_frame",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        mesh_from_cadquery(build_roll_cartridge(), "roll_cartridge_mesh"),
        material=steel,
        name="cartridge_body",
    )

    model.articulation(
        "root_to_yaw",
        ArticulationType.REVOLUTE,
        parent=root,
        child=yaw_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-2.6,
            upper=2.6,
        ),
    )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_ring,
        child=pitch_cradle,
        origin=Origin(xyz=(0.104, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-0.95,
            upper=0.95,
        ),
    )

    model.articulation(
        "pitch_to_roll",
        ArticulationType.CONTINUOUS,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_base")
    yaw_ring = object_model.get_part("yaw_ring")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")
    yaw = object_model.get_articulation("root_to_yaw")
    pitch = object_model.get_articulation("yaw_to_pitch")
    roll = object_model.get_articulation("pitch_to_roll")

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
    ctx.allow_overlap(
        root,
        yaw_ring,
        reason="coaxial yaw bearing races are simplified as closed visual solids inside a dense wrist housing",
    )
    ctx.allow_overlap(
        yaw_ring,
        pitch_cradle,
        reason="the side-pivot bearing housings are modeled as closed shells around the captured cradle trunnions",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        reason="the roll cartridge is shown as a fully enclosed captured cartridge rather than exposing separate bearing voids",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint axes follow yaw-pitch-roll stack",
        yaw.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (0.0, 1.0, 0.0)
        and roll.axis == (1.0, 0.0, 0.0),
        details=(
            f"got axes yaw={yaw.axis}, pitch={pitch.axis}, roll={roll.axis}"
        ),
    )

    ctx.check(
        "joint limits match dense wrist travel",
        yaw.motion_limits is not None
        and pitch.motion_limits is not None
        and roll.motion_limits is not None
        and yaw.motion_limits.lower == -2.6
        and yaw.motion_limits.upper == 2.6
        and pitch.motion_limits.lower == -0.95
        and pitch.motion_limits.upper == 0.95
        and roll.motion_limits.lower is None
        and roll.motion_limits.upper is None,
        details="unexpected articulated travel limits",
    )

    ctx.expect_contact(root, yaw_ring, name="root base physically supports yaw ring")
    ctx.expect_contact(yaw_ring, pitch_cradle, name="yaw ring captures pitch cradle")
    ctx.expect_contact(
        pitch_cradle,
        roll_cartridge,
        name="pitch cradle retains roll cartridge",
    )

    ctx.expect_within(
        pitch_cradle,
        yaw_ring,
        axes="yz",
        margin=0.0,
        name="pitch cradle stays nested inside yaw ring envelope",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_cradle,
        axes="z",
        margin=0.0,
        name="roll cartridge stays centered in cradle height",
    )

    yaw_aabb = ctx.part_world_aabb(yaw_ring)
    cradle_aabb = ctx.part_world_aabb(pitch_cradle)
    cartridge_aabb = ctx.part_world_aabb(roll_cartridge)
    projects_forward = (
        yaw_aabb is not None
        and cradle_aabb is not None
        and cartridge_aabb is not None
        and cartridge_aabb[1][0] > yaw_aabb[1][0] + 0.14
        and cartridge_aabb[1][0] > cradle_aabb[1][0] + 0.10
    )
    ctx.check(
        "roll cartridge projects forward of cradle and ring",
        projects_forward,
        details=(
            f"yaw max x={None if yaw_aabb is None else yaw_aabb[1][0]:.3f}, "
            f"cradle max x={None if cradle_aabb is None else cradle_aabb[1][0]:.3f}, "
            f"cartridge max x={None if cartridge_aabb is None else cartridge_aabb[1][0]:.3f}"
        )
        if yaw_aabb is not None and cradle_aabb is not None and cartridge_aabb is not None
        else "one or more part AABBs were unavailable",
    )

    with ctx.pose({yaw: 0.4, pitch: 0.20, roll: 1.20}):
        ctx.expect_contact(
            root,
            yaw_ring,
            name="yaw ring stays seated on root in turned pose",
        )
        ctx.expect_contact(
            yaw_ring,
            pitch_cradle,
            name="pitch cradle stays captured on side pivots in turned pose",
        )
        ctx.expect_contact(
            pitch_cradle,
            roll_cartridge,
            name="roll cartridge stays retained in turned pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no part collisions in representative articulated pose"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
