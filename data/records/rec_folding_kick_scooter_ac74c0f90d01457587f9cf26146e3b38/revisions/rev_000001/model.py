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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_kick_scooter")

    deck_metal = model.material("deck_metal", rgba=(0.67, 0.69, 0.73, 1.0))
    stem_black = model.material("stem_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_black = model.material("grip_black", rgba=(0.05, 0.05, 0.05, 1.0))
    core_silver = model.material("core_silver", rgba=(0.80, 0.81, 0.84, 1.0))
    urethane_dark = model.material("urethane_dark", rgba=(0.14, 0.14, 0.16, 1.0))
    latch_red = model.material("latch_red", rgba=(0.70, 0.16, 0.14, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.58, 0.60, 0.64, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
        ]

    wheel_radius = 0.048
    wheel_width = 0.024
    hub_width = 0.032

    deck_frame = model.part("deck_frame")
    deck_frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.16, 0.22)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    deck_body_geom = ExtrudeGeometry(
        rounded_rect_profile(0.36, 0.12, 0.022, corner_segments=10),
        0.018,
        center=True,
    )
    deck_frame.visual(
        save_mesh("scooter_deck_body", deck_body_geom),
        origin=Origin(xyz=(0.010, 0.0, 0.058)),
        material=deck_metal,
        name="deck_body",
    )
    deck_frame.visual(
        Box((0.250, 0.095, 0.0015)),
        origin=Origin(xyz=(-0.005, 0.0, 0.06775)),
        material=grip_black,
        name="grip_tape",
    )

    neck_geom = section_loft(
        [
            yz_section(0.120, 0.052, 0.020, 0.007, 0.063),
            yz_section(0.155, 0.068, 0.038, 0.010, 0.072),
            yz_section(0.188, 0.076, 0.058, 0.012, 0.086),
            yz_section(0.214, 0.078, 0.082, 0.013, 0.102),
        ]
    )
    deck_frame.visual(
        save_mesh("scooter_neck_brace", neck_geom),
        material=deck_metal,
        name="neck_brace",
    )

    stem_pitch = -0.34
    headtube_center = (0.195, 0.0, 0.124)
    headtube_length = 0.112
    headtube_radius = 0.027
    deck_frame.visual(
        Cylinder(radius=headtube_radius, length=headtube_length),
        origin=Origin(xyz=headtube_center, rpy=(0.0, stem_pitch, 0.0)),
        material=stem_black,
        name="headtube_shell",
    )

    deck_frame.visual(
        Box((0.110, 0.012, 0.022)),
        origin=Origin(xyz=(-0.223, 0.024, 0.060)),
        material=deck_metal,
        name="left_rear_stay",
    )
    deck_frame.visual(
        Box((0.110, 0.012, 0.022)),
        origin=Origin(xyz=(-0.223, -0.024, 0.060)),
        material=deck_metal,
        name="right_rear_stay",
    )
    deck_frame.visual(
        Box((0.018, 0.008, 0.044)),
        origin=Origin(xyz=(-0.238, 0.020, wheel_radius)),
        material=stem_black,
        name="rear_left_dropout",
    )
    deck_frame.visual(
        Box((0.018, 0.008, 0.044)),
        origin=Origin(xyz=(-0.238, -0.020, wheel_radius)),
        material=stem_black,
        name="rear_right_dropout",
    )
    deck_frame.visual(
        Box((0.014, 0.012, 0.040)),
        origin=Origin(xyz=(-0.257, 0.024, 0.089)),
        material=deck_metal,
        name="left_fender_support",
    )
    deck_frame.visual(
        Box((0.014, 0.012, 0.040)),
        origin=Origin(xyz=(-0.257, -0.024, 0.089)),
        material=deck_metal,
        name="right_fender_support",
    )
    deck_frame.visual(
        Box((0.050, 0.060, 0.012)),
        origin=Origin(xyz=(-0.268, 0.0, 0.108)),
        material=stem_black,
        name="rear_fender",
    )

    deck_frame.visual(
        Box((0.018, 0.026, 0.032)),
        origin=Origin(xyz=(0.220, 0.038, 0.107)),
        material=stem_black,
        name="latch_mount_block",
    )
    deck_frame.visual(
        Box((0.042, 0.016, 0.006)),
        origin=Origin(xyz=(0.240, 0.038, 0.089)),
        material=stem_black,
        name="latch_stop_pad",
    )
    deck_frame.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.232, 0.038, 0.122)),
        material=stem_black,
        name="latch_upper_bridge",
    )

    stem_assembly = model.part("stem_assembly")
    stem_assembly.inertial = Inertial.from_geometry(
        Box((0.42, 0.60, 0.84)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )
    stem_assembly.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stem_black,
        name="stem_base_collar",
    )
    stem_assembly.visual(
        Cylinder(radius=0.022, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=stem_black,
        name="main_stem_tube",
    )
    stem_assembly.visual(
        Cylinder(radius=0.027, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.612)),
        material=stem_black,
        name="bar_clamp",
    )
    stem_assembly.visual(
        Box((0.038, 0.034, 0.028)),
        origin=Origin(xyz=(0.105, 0.0, -0.004)),
        material=stem_black,
        name="fork_crown",
    )
    stem_assembly.visual(
        Box((0.112, 0.024, 0.020)),
        origin=Origin(xyz=(0.052, 0.0, 0.025)),
        material=stem_black,
        name="fork_offset_bridge",
    )
    stem_assembly.visual(
        Box((0.040, 0.024, 0.024)),
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
        material=stem_black,
        name="fork_neck_connector",
    )
    stem_assembly.visual(
        Box((0.016, 0.008, 0.122)),
        origin=Origin(xyz=(0.105, 0.020, -0.061)),
        material=stem_black,
        name="left_fork_blade",
    )
    stem_assembly.visual(
        Box((0.016, 0.008, 0.122)),
        origin=Origin(xyz=(0.105, -0.020, -0.061)),
        material=stem_black,
        name="right_fork_blade",
    )
    stem_assembly.visual(
        Cylinder(radius=0.017, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.648), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stem_black,
        name="handlebar_crossbar",
    )
    stem_assembly.visual(
        Cylinder(radius=0.016, length=0.105),
        origin=Origin(xyz=(0.0, 0.322, 0.648), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    stem_assembly.visual(
        Cylinder(radius=0.016, length=0.105),
        origin=Origin(xyz=(0.0, -0.322, 0.648), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    stem_assembly.visual(
        Box((0.048, 0.030, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=stem_black,
        name="t_bar_junction",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=hub_width),
        mass=0.24,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    front_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=urethane_dark,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_silver,
        name="front_core",
    )
    front_wheel.visual(
        Cylinder(radius=0.010, length=hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="front_hub",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=hub_width),
        mass=0.24,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rear_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=urethane_dark,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_silver,
        name="rear_core",
    )
    rear_wheel.visual(
        Cylinder(radius=0.010, length=hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="rear_hub",
    )

    fold_latch = model.part("fold_latch")
    fold_latch.inertial = Inertial.from_geometry(
        Box((0.070, 0.014, 0.026)),
        mass=0.08,
        origin=Origin(xyz=(0.028, 0.0, -0.002)),
    )
    fold_latch.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="latch_barrel",
    )
    fold_latch.visual(
        Box((0.050, 0.010, 0.012)),
        origin=Origin(xyz=(0.026, 0.0, -0.010)),
        material=latch_red,
        name="latch_handle",
    )
    fold_latch.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(0.052, 0.0, 0.003)),
        material=latch_red,
        name="latch_paddle",
    )

    top_center_x = headtube_center[0] + (headtube_length * 0.5) * math.sin(stem_pitch)
    top_center_z = headtube_center[2] + (headtube_length * 0.5) * math.cos(stem_pitch)

    model.articulation(
        "deck_to_stem",
        ArticulationType.FIXED,
        parent=deck_frame,
        child=stem_assembly,
        origin=Origin(xyz=(top_center_x, 0.0, top_center_z), rpy=(0.0, stem_pitch, 0.0)),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.105, 0.0, -0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck_frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.238, 0.0, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )
    model.articulation(
        "deck_to_fold_latch",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=fold_latch,
        origin=Origin(xyz=(0.241, 0.038, 0.108)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.10,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_frame = object_model.get_part("deck_frame")
    stem_assembly = object_model.get_part("stem_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    fold_latch = object_model.get_part("fold_latch")

    front_axle = object_model.get_articulation("stem_to_front_wheel")
    rear_axle = object_model.get_articulation("deck_to_rear_wheel")
    latch_hinge = object_model.get_articulation("deck_to_fold_latch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        stem_assembly,
        deck_frame,
        elem_a="stem_base_collar",
        elem_b="headtube_shell",
        name="stem_seated_on_headtube",
    )
    ctx.expect_contact(
        front_wheel,
        stem_assembly,
        elem_a="front_hub",
        elem_b="left_fork_blade",
        name="front_wheel_captured_by_fork",
    )
    ctx.expect_contact(
        rear_wheel,
        deck_frame,
        elem_a="rear_hub",
        elem_b="rear_left_dropout",
        name="rear_wheel_captured_by_dropouts",
    )
    ctx.expect_contact(
        fold_latch,
        deck_frame,
        elem_a="latch_handle",
        elem_b="latch_stop_pad",
        name="fold_latch_closed_against_stop",
    )

    deck_body_aabb = ctx.part_element_world_aabb(deck_frame, elem="deck_body")
    if deck_body_aabb is None:
        ctx.fail("deck_body_present", "Deck body visual AABB is unavailable.")
    else:
        deck_length = deck_body_aabb[1][0] - deck_body_aabb[0][0]
        ctx.check(
            "compact_deck_length",
            0.33 <= deck_length <= 0.40,
            f"Expected compact deck length in [0.33, 0.40] m, got {deck_length:.3f} m.",
        )

    crossbar_aabb = ctx.part_element_world_aabb(stem_assembly, elem="handlebar_crossbar")
    if crossbar_aabb is None:
        ctx.fail("handlebar_crossbar_present", "Handlebar crossbar visual AABB is unavailable.")
    else:
        crossbar_center_z = 0.5 * (crossbar_aabb[0][2] + crossbar_aabb[1][2])
        ctx.check(
            "t_bar_height_realistic",
            0.78 <= crossbar_center_z <= 0.86,
            f"Expected T-bar height in [0.78, 0.86] m, got {crossbar_center_z:.3f} m.",
        )

    front_pos = ctx.part_world_position(front_wheel)
    rear_pos = ctx.part_world_position(rear_wheel)
    if front_pos is None or rear_pos is None:
        ctx.fail("wheel_positions_available", "Could not resolve world positions for both wheels.")
    else:
        wheelbase = front_pos[0] - rear_pos[0]
        ctx.check(
            "wheelbase_realistic",
            0.50 <= wheelbase <= 0.58,
            f"Expected wheelbase in [0.50, 0.58] m, got {wheelbase:.3f} m.",
        )

    ctx.check(
        "front_axle_axis_y",
        abs(front_axle.axis[0]) < 1e-9 and abs(front_axle.axis[1] - 1.0) < 1e-9 and abs(front_axle.axis[2]) < 1e-9,
        f"Front axle axis should be +Y, got {front_axle.axis}.",
    )
    ctx.check(
        "rear_axle_axis_y",
        abs(rear_axle.axis[0]) < 1e-9 and abs(rear_axle.axis[1] - 1.0) < 1e-9 and abs(rear_axle.axis[2]) < 1e-9,
        f"Rear axle axis should be +Y, got {rear_axle.axis}.",
    )
    ctx.check(
        "latch_hinge_axis_y",
        abs(latch_hinge.axis[0]) < 1e-9 and abs(abs(latch_hinge.axis[1]) - 1.0) < 1e-9 and abs(latch_hinge.axis[2]) < 1e-9,
        f"Fold latch hinge axis should run along Y, got {latch_hinge.axis}.",
    )
    ctx.check(
        "wheel_joints_are_continuous",
        front_axle.articulation_type == ArticulationType.CONTINUOUS
        and rear_axle.articulation_type == ArticulationType.CONTINUOUS,
        "Front and rear wheel axles should spin continuously.",
    )
    latch_limits = latch_hinge.motion_limits
    ctx.check(
        "latch_motion_limits_realistic",
        latch_limits is not None
        and latch_limits.lower is not None
        and latch_limits.upper is not None
        and latch_limits.lower <= 0.0
        and latch_limits.upper >= 0.9,
        f"Fold latch limits should span a closed and open position, got {latch_limits}.",
    )

    latch_rest = ctx.part_element_world_aabb(fold_latch, elem="latch_paddle")
    if latch_rest is None:
        ctx.fail("latch_paddle_present", "Latch paddle visual AABB is unavailable.")
    else:
        with ctx.pose({latch_hinge: 0.95}):
            latch_open = ctx.part_element_world_aabb(fold_latch, elem="latch_paddle")
            if latch_open is None:
                ctx.fail("latch_open_pose_available", "Latch paddle AABB is unavailable in the open pose.")
            else:
                ctx.check(
                    "latch_opens_upward",
                    latch_open[1][2] > latch_rest[1][2] + 0.020,
                    "Fold latch paddle should lift noticeably when the hinge is opened.",
                )

    with ctx.pose({front_axle: 1.4, rear_axle: -1.2}):
        ctx.expect_contact(
            front_wheel,
            stem_assembly,
            elem_a="front_hub",
            elem_b="left_fork_blade",
            name="front_wheel_stays_on_axle_when_spun",
        )
        ctx.expect_contact(
            rear_wheel,
            deck_frame,
            elem_a="rear_hub",
            elem_b="rear_left_dropout",
            name="rear_wheel_stays_on_axle_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
