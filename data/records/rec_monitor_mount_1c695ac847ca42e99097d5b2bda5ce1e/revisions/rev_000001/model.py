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


SHOULDER_ORIGIN = (0.050, 0.0, 0.418)
LOWER_ARM_LENGTH = 0.270
UPPER_ARM_LENGTH = 0.220
SWIVEL_ORIGIN = (UPPER_ARM_LENGTH, 0.0, 0.032)
TILT_ORIGIN = (0.055, 0.0, 0.030)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center[0], center[1])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, center[2]))
    )


def _make_base_support() -> cq.Workplane:
    base_plate = _box((0.320, 0.260, 0.018), (0.0, 0.0, 0.009)).edges("|Z").fillet(0.016)
    anchor_boss = _box((0.118, 0.110, 0.026), (-0.020, 0.0, 0.031))
    column = _box((0.090, 0.082, 0.304), (-0.028, 0.0, 0.182)).edges("|Z").fillet(0.010)
    top_beam = _box((0.120, 0.064, 0.026), (-0.010, 0.0, 0.324))
    shoulder_upright = _box((0.020, 0.050, 0.128), (SHOULDER_ORIGIN[0] - 0.024, 0.0, 0.388))
    shoulder_block = _box((0.030, 0.056, 0.072), (SHOULDER_ORIGIN[0] - 0.034, 0.0, SHOULDER_ORIGIN[2]))
    shoulder_face = _box((0.024, 0.050, 0.048), (SHOULDER_ORIGIN[0] - 0.010, 0.0, SHOULDER_ORIGIN[2]))
    gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.060, 0.214)
        .lineTo(-0.014, 0.214)
        .lineTo(0.012, 0.330)
        .lineTo(-0.038, 0.330)
        .close()
        .extrude(0.068, both=True)
    )

    return (
        base_plate.union(anchor_boss)
        .union(column)
        .union(top_beam)
        .union(shoulder_upright)
        .union(shoulder_block)
        .union(shoulder_face)
        .union(gusset)
    )


def _make_lower_arm() -> cq.Workplane:
    root_block = _box((0.026, 0.050, 0.050), (0.013, 0.0, 0.0))
    root_web = _box((0.040, 0.036, 0.028), (0.032, 0.0, 0.0))
    beam = _box((0.230, 0.034, 0.024), (0.145, 0.0, 0.0))
    lower_rib = _box((0.150, 0.018, 0.016), (0.160, 0.0, -0.018))
    elbow_block = _box((0.036, 0.050, 0.046), (LOWER_ARM_LENGTH - 0.018, 0.0, 0.0))
    elbow_cap = _box((0.050, 0.040, 0.018), (LOWER_ARM_LENGTH - 0.028, 0.0, 0.022))

    return (
        root_block.union(root_web)
        .union(beam)
        .union(lower_rib)
        .union(elbow_block)
        .union(elbow_cap)
    )


def _make_upper_arm() -> cq.Workplane:
    root_block = _box((0.024, 0.046, 0.046), (0.012, 0.0, 0.0))
    root_web = _box((0.036, 0.034, 0.024), (0.026, 0.0, 0.0))
    beam = _box((0.184, 0.030, 0.022), (0.116, 0.0, 0.0))
    lower_rib = _box((0.114, 0.016, 0.014), (0.126, 0.0, -0.016))
    nose_block = _box((0.038, 0.040, 0.040), (UPPER_ARM_LENGTH - 0.019, 0.0, 0.006))
    swivel_pad = _box((0.020, 0.034, 0.014), (UPPER_ARM_LENGTH - 0.010, 0.0, SWIVEL_ORIGIN[2] - 0.003))

    return (
        root_block.union(root_web)
        .union(beam)
        .union(lower_rib)
        .union(nose_block)
        .union(swivel_pad)
    )


def _make_head_yoke() -> cq.Workplane:
    base_plate = _box((0.026, 0.040, 0.010), (0.013, 0.0, 0.005))
    neck = _box((0.032, 0.036, 0.036), (0.025, 0.0, 0.018))
    rear_bridge = _box((0.014, 0.120, 0.024), (TILT_ORIGIN[0] - 0.028, 0.0, TILT_ORIGIN[2] + 0.010))
    left_cheek = _box((0.012, 0.012, 0.070), (TILT_ORIGIN[0] - 0.016, 0.055, TILT_ORIGIN[2]))
    right_cheek = _box((0.012, 0.012, 0.070), (TILT_ORIGIN[0] - 0.016, -0.055, TILT_ORIGIN[2]))
    top_bridge = _box((0.014, 0.120, 0.014), (TILT_ORIGIN[0] - 0.016, 0.0, TILT_ORIGIN[2] + 0.030))

    return (
        base_plate.union(neck)
        .union(rear_bridge)
        .union(left_cheek)
        .union(right_cheek)
        .union(top_bridge)
    )


def _make_mounting_plate() -> cq.Workplane:
    trunnion_block = _box((0.022, 0.098, 0.018), (0.000, 0.0, 0.0))
    bridge = _box((0.098, 0.038, 0.038), (0.040, 0.0, 0.0))
    rear_rib = _box((0.060, 0.056, 0.082), (0.054, 0.0, 0.0))
    plate_blank = cq.Workplane("YZ").rect(0.160, 0.140).extrude(0.008).translate((0.088, 0.0, 0.0))
    vesa_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.050, -0.050), (0.050, -0.050), (-0.050, 0.050), (0.050, 0.050)])
        .circle(0.0032)
        .extrude(0.020)
        .translate((0.088, 0.0, 0.0))
    )
    cable_slot = cq.Workplane("YZ").rect(0.018, 0.048).extrude(0.020).translate((0.088, 0.0, 0.0))
    plate = plate_blank.cut(vesa_holes).cut(cable_slot)

    return trunnion_block.union(bridge).union(rear_rib).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_console_monitor_mount")

    model.material("powder_graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("machine_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("satin_aluminum", rgba=(0.75, 0.77, 0.79, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        mesh_from_cadquery(_make_base_support(), "base_support"),
        material="powder_graphite",
        name="base_shell",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_make_lower_arm(), "lower_arm"),
        material="machine_gray",
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm(), "upper_arm"),
        material="machine_gray",
        name="upper_arm_shell",
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        mesh_from_cadquery(_make_head_yoke(), "head_yoke"),
        material="powder_graphite",
        name="head_yoke_shell",
    )

    mounting_plate = model.part("mounting_plate")
    mounting_plate.visual(
        mesh_from_cadquery(_make_mounting_plate(), "mounting_plate"),
        material="satin_aluminum",
        name="plate_panel",
    )

    model.articulation(
        "lower_arm_pitch",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=lower_arm,
        origin=Origin(xyz=SHOULDER_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-0.55, upper=1.20),
    )
    model.articulation(
        "upper_arm_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-1.25, upper=1.20),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head_yoke,
        origin=Origin(xyz=SWIVEL_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "plate_tilt",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=mounting_plate,
        origin=Origin(xyz=TILT_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.65, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_support = object_model.get_part("base_support")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head_yoke = object_model.get_part("head_yoke")
    mounting_plate = object_model.get_part("mounting_plate")

    lower_arm_pitch = object_model.get_articulation("lower_arm_pitch")
    upper_arm_pitch = object_model.get_articulation("upper_arm_pitch")
    head_swivel = object_model.get_articulation("head_swivel")
    plate_tilt = object_model.get_articulation("plate_tilt")

    ctx.allow_overlap(
        head_yoke,
        mounting_plate,
        reason="captured tilt trunnion is intentionally nested within the compact yoke cheeks",
    )
    ctx.allow_overlap(
        base_support,
        lower_arm,
        reason="simplified shoulder casting captures the lower arm root around the hinge axis with intentional nested knuckle geometry",
    )

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

    ctx.expect_contact(lower_arm, base_support, contact_tol=1e-5, name="lower arm seated in shoulder clevis")
    ctx.expect_contact(upper_arm, lower_arm, contact_tol=1e-5, name="upper arm seated in elbow clevis")
    ctx.expect_contact(head_yoke, upper_arm, contact_tol=1e-5, name="swivel head seated on upper arm pad")
    ctx.expect_gap(
        mounting_plate,
        head_yoke,
        axis="x",
        max_gap=0.001,
        max_penetration=0.003,
        name="mounting plate trunnions seated in yoke",
    )
    ctx.expect_origin_gap(
        lower_arm,
        base_support,
        axis="z",
        min_gap=0.39,
        max_gap=0.45,
        name="lower arm hinge sits above grounded base",
    )

    with ctx.pose({lower_arm_pitch: 0.75}):
        ctx.expect_origin_gap(
            upper_arm,
            base_support,
            axis="z",
            min_gap=0.60,
            name="positive lower arm pitch raises elbow",
        )

    with ctx.pose({lower_arm_pitch: 0.40, upper_arm_pitch: 0.00}):
        neutral_head_z = ctx.part_world_position(head_yoke)[2]
    with ctx.pose({lower_arm_pitch: 0.40, upper_arm_pitch: 0.75}):
        raised_head_z = ctx.part_world_position(head_yoke)[2]
    ctx.check(
        "upper arm pitch lifts head assembly",
        raised_head_z > neutral_head_z + 0.09,
        f"head z only changed from {neutral_head_z:.3f} to {raised_head_z:.3f}",
    )

    with ctx.pose({lower_arm_pitch: 0.30, upper_arm_pitch: -0.15, head_swivel: 0.0}):
        centered_plate_pos = ctx.part_world_position(mounting_plate)
    with ctx.pose({lower_arm_pitch: 0.30, upper_arm_pitch: -0.15, head_swivel: 0.90}):
        swiveled_plate_pos = ctx.part_world_position(mounting_plate)
    ctx.check(
        "head swivel yaws mounting plate sideways",
        abs(swiveled_plate_pos[1]) > abs(centered_plate_pos[1]) + 0.035,
        f"plate y changed from {centered_plate_pos[1]:.3f} to {swiveled_plate_pos[1]:.3f}",
    )

    with ctx.pose({plate_tilt: 0.0}):
        neutral_aabb = ctx.part_element_world_aabb(mounting_plate, elem="plate_panel")
    with ctx.pose({plate_tilt: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(mounting_plate, elem="plate_panel")
    neutral_dx = neutral_aabb[1][0] - neutral_aabb[0][0]
    tilted_dx = tilted_aabb[1][0] - tilted_aabb[0][0]
    ctx.check(
        "plate tilt changes panel pitch",
        tilted_dx > neutral_dx + 0.045,
        f"panel x depth only changed from {neutral_dx:.3f} to {tilted_dx:.3f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
