from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POLE_RADIUS = 0.019
POLE_HEIGHT = 0.34
POLE_X = -0.042

ARM_1_LENGTH = 0.18
ARM_2_LENGTH = 0.16
HEAD_TILT_OFFSET = 0.038

ARM_1_BARREL_RADIUS = 0.014
ARM_1_BARREL_LENGTH = 0.014
ARM_1_EAR_THICKNESS = 0.004

ARM_2_BARREL_RADIUS = 0.013
ARM_2_BARREL_LENGTH = 0.014
ARM_2_EAR_THICKNESS = 0.004

HEAD_BARREL_RADIUS = 0.011
HEAD_BARREL_LENGTH = 0.012
HEAD_EAR_THICKNESS = 0.004

PLATE_TRUNNION_RADIUS = 0.008
PLATE_TRUNNION_LENGTH = 0.014
TILT_EAR_THICKNESS = 0.004


def _cylinder_z(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return _cylinder_z(radius, length).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        safe_radius = min(radius, 0.45 * min(size[0], size[1]))
        shape = shape.edges("|Z").fillet(safe_radius)
    return shape


def _make_mast_clamp_shape() -> cq.Workplane:
    collar = _rounded_box((0.034, 0.054, 0.082), 0.004).translate((POLE_X + 0.006, 0.0, 0.0))
    collar = collar.cut(_cylinder_z(POLE_RADIUS + 0.0015, 0.100).translate((POLE_X, 0.0, 0.0)))

    clevis_total_z = ARM_1_BARREL_LENGTH + 2.0 * ARM_1_EAR_THICKNESS
    clevis = _rounded_box((0.026, 0.022, clevis_total_z), 0.0015).translate((-0.013, 0.0, 0.0))
    clevis_slot = cq.Workplane("XY").box(0.018, 0.026, ARM_1_BARREL_LENGTH).translate((-0.004, 0.0, 0.0))

    return collar.union(clevis).cut(clevis_slot)


def _make_first_arm_shape() -> cq.Workplane:
    root_barrel = _cylinder_z(ARM_1_BARREL_RADIUS, ARM_1_BARREL_LENGTH)
    root_transition = _rounded_box((0.028, 0.024, 0.010), 0.003).translate((0.014, 0.0, 0.0))
    beam = _rounded_box((ARM_1_LENGTH - 0.026, 0.028, 0.010), 0.0035).translate(((ARM_1_LENGTH - 0.026) / 2.0, 0.0, 0.0))

    clevis_total_z = ARM_2_BARREL_LENGTH + 2.0 * ARM_2_EAR_THICKNESS
    distal_clevis = _rounded_box((0.030, 0.022, clevis_total_z), 0.0015).translate((ARM_1_LENGTH - 0.015, 0.0, 0.0))
    distal_slot = cq.Workplane("XY").box(0.020, 0.026, ARM_2_BARREL_LENGTH).translate((ARM_1_LENGTH - 0.005, 0.0, 0.0))

    return root_barrel.union(root_transition).union(beam).union(distal_clevis).cut(distal_slot)


def _make_second_arm_shape() -> cq.Workplane:
    root_barrel = _cylinder_z(ARM_2_BARREL_RADIUS, ARM_2_BARREL_LENGTH)
    root_transition = _rounded_box((0.024, 0.022, 0.010), 0.0025).translate((0.012, 0.0, 0.0))
    beam = _rounded_box((ARM_2_LENGTH - 0.026, 0.026, 0.010), 0.003).translate(((ARM_2_LENGTH - 0.026) / 2.0, 0.0, 0.0))

    clevis_total_z = HEAD_BARREL_LENGTH + 2.0 * HEAD_EAR_THICKNESS
    distal_clevis = _rounded_box((0.030, 0.020, clevis_total_z), 0.0014).translate((ARM_2_LENGTH - 0.015, 0.0, 0.0))
    distal_slot = cq.Workplane("XY").box(0.020, 0.024, HEAD_BARREL_LENGTH).translate((ARM_2_LENGTH - 0.005, 0.0, 0.0))

    return root_barrel.union(root_transition).union(beam).union(distal_clevis).cut(distal_slot)


def _make_head_swivel_shape() -> cq.Workplane:
    root_barrel = _cylinder_z(HEAD_BARREL_RADIUS, HEAD_BARREL_LENGTH)
    spine = _rounded_box((0.030, 0.014, 0.012), 0.002).translate((0.015, 0.0, 0.0))

    yoke_total_y = PLATE_TRUNNION_LENGTH + 2.0 * TILT_EAR_THICKNESS
    yoke = _rounded_box((0.014, yoke_total_y, 0.016), 0.0014).translate((HEAD_TILT_OFFSET - 0.007, 0.0, 0.0))
    yoke_slot = cq.Workplane("XY").box(0.010, PLATE_TRUNNION_LENGTH, 0.018).translate((HEAD_TILT_OFFSET - 0.003, 0.0, 0.0))

    return root_barrel.union(spine).union(yoke).cut(yoke_slot)


def _make_monitor_plate_shape() -> cq.Workplane:
    trunnion = _cylinder_y(PLATE_TRUNNION_RADIUS, PLATE_TRUNNION_LENGTH)
    hub = _rounded_box((0.012, 0.012, 0.020), 0.0015).translate((0.006, 0.0, 0.0))
    neck = _rounded_box((0.030, 0.018, 0.028), 0.0018).translate((0.024, 0.0, 0.0))

    plate = _rounded_box((0.004, 0.110, 0.110), 0.006).translate((0.039, 0.0, 0.0))
    plate = plate.faces(">X").workplane(centerOption="CenterOfMass").pushPoints(
        [(-0.0375, -0.0375), (-0.0375, 0.0375), (0.0375, -0.0375), (0.0375, 0.0375)]
    ).hole(0.0065)
    plate = plate.faces(">X").workplane(centerOption="CenterOfMass").rect(0.030, 0.018).cutBlind(-0.004)

    return trunnion.union(hub).union(neck).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pole_mounted_monitor_support")

    model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("satin_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("dark_polymer", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("mast_steel", rgba=(0.57, 0.60, 0.64, 1.0))

    mast_clamp = model.part("mast_clamp")
    mast_clamp.visual(
        Cylinder(radius=POLE_RADIUS, length=POLE_HEIGHT),
        origin=Origin(xyz=(POLE_X, 0.0, 0.0)),
        material="mast_steel",
        name="pole_segment",
    )
    mast_clamp.visual(
        mesh_from_cadquery(_make_mast_clamp_shape(), "mast_clamp_body"),
        material="powder_black",
        name="clamp_body",
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        mesh_from_cadquery(_make_first_arm_shape(), "first_arm_link"),
        material="satin_aluminum",
        name="arm_shell",
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        mesh_from_cadquery(_make_second_arm_shape(), "second_arm_link"),
        material="satin_aluminum",
        name="arm_shell",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(_make_head_swivel_shape(), "head_swivel_body"),
        material="dark_polymer",
        name="head_body",
    )

    monitor_plate = model.part("monitor_plate")
    monitor_plate.visual(
        mesh_from_cadquery(_make_monitor_plate_shape(), "monitor_plate"),
        material="powder_black",
        name="plate_panel",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=mast_clamp,
        child=first_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.3, upper=2.3, effort=25.0, velocity=1.4),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(ARM_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.4, upper=2.4, effort=18.0, velocity=1.7),
    )
    model.articulation(
        "head_swivel_yaw",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_swivel,
        origin=Origin(xyz=(ARM_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.6, upper=1.6, effort=6.0, velocity=2.0),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=monitor_plate,
        origin=Origin(xyz=(HEAD_TILT_OFFSET, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.55, effort=4.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_clamp = object_model.get_part("mast_clamp")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    head_swivel = object_model.get_part("head_swivel")
    monitor_plate = object_model.get_part("monitor_plate")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    head_swivel_yaw = object_model.get_articulation("head_swivel_yaw")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        mast_clamp,
        first_arm,
        reason="shoulder revolute uses a captured journal nested inside the clamp clevis",
    )
    ctx.allow_overlap(
        first_arm,
        second_arm,
        reason="elbow revolute uses a nested barrel inside the distal clevis of the first arm",
    )
    ctx.allow_overlap(
        second_arm,
        head_swivel,
        reason="head swivel journal is intentionally captured inside the second arm clevis",
    )
    ctx.allow_overlap(
        head_swivel,
        monitor_plate,
        reason="monitor tilt trunnion is intentionally nested inside the compact head yoke",
    )

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

    ctx.expect_contact(mast_clamp, first_arm, name="shoulder_joint_supported")
    ctx.expect_contact(first_arm, second_arm, name="elbow_joint_supported")
    ctx.expect_contact(second_arm, head_swivel, name="head_swivel_joint_supported")
    ctx.expect_contact(head_swivel, monitor_plate, name="head_tilt_joint_supported")

    with ctx.pose({shoulder_yaw: 0.70}):
        elbow_pos = ctx.part_world_position(second_arm)
        ctx.check(
            "shoulder_positive_yaw_swings_arm_left",
            elbow_pos is not None and elbow_pos[1] > 0.10,
            f"expected elbow origin y > 0.10 m after positive shoulder yaw, got {elbow_pos}",
        )

    with ctx.pose({elbow_yaw: 0.70}):
        head_pos = ctx.part_world_position(head_swivel)
        ctx.check(
            "elbow_positive_yaw_offsets_head_left",
            head_pos is not None and head_pos[1] > 0.09,
            f"expected head origin y > 0.09 m after positive elbow yaw, got {head_pos}",
        )

    with ctx.pose({head_swivel_yaw: 0.75}):
        tilt_axis_pos = ctx.part_world_position(monitor_plate)
        ctx.check(
            "head_swivel_reorients_monitor_mount",
            tilt_axis_pos is not None and tilt_axis_pos[1] > 0.02,
            f"expected monitor tilt axis to move to positive y after head swivel, got {tilt_axis_pos}",
        )

    neutral_plate_aabb = ctx.part_element_world_aabb(monitor_plate, elem="plate_panel")
    with ctx.pose({head_tilt: 0.40}):
        tilted_plate_aabb = ctx.part_element_world_aabb(monitor_plate, elem="plate_panel")
    ctx.check(
        "head_tilt_raises_top_edge",
        neutral_plate_aabb is not None
        and tilted_plate_aabb is not None
        and tilted_plate_aabb[1][2] > neutral_plate_aabb[1][2] + 0.010,
        f"expected tilted plate top edge to rise noticeably, neutral={neutral_plate_aabb}, tilted={tilted_plate_aabb}",
    )

    second_arm_aabb = ctx.part_world_aabb(second_arm)
    head_swivel_aabb = ctx.part_world_aabb(head_swivel)
    head_compact = False
    if second_arm_aabb is not None and head_swivel_aabb is not None:
        second_arm_y = second_arm_aabb[1][1] - second_arm_aabb[0][1]
        second_arm_z = second_arm_aabb[1][2] - second_arm_aabb[0][2]
        head_y = head_swivel_aabb[1][1] - head_swivel_aabb[0][1]
        head_z = head_swivel_aabb[1][2] - head_swivel_aabb[0][2]
        head_compact = head_y < second_arm_y and head_z < second_arm_z
    ctx.check(
        "head_hardware_stays_smaller_than_arm_link",
        head_compact,
        f"expected compact head hardware, second_arm={second_arm_aabb}, head_swivel={head_swivel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
