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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _ring_shell_mesh(*, outer_radius: float, inner_radius: float, length: float, name: str):
    half = 0.5 * length
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _build_base_plate_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.48, 0.36, 0.055, corner_segments=10),
            0.03,
            cap=True,
            center=True,
        ),
        "stand_base_plate",
    )


def _build_stand_brace_mesh(y_sign: float):
    side = "left" if y_sign > 0.0 else "right"
    return mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (-0.15, 0.10 * y_sign, 0.055),
                (-0.12, 0.10 * y_sign, 0.16),
                (-0.06, 0.10 * y_sign, 0.31),
                (0.00, 0.10 * y_sign, 0.43),
            ],
            profile=rounded_rect_profile(0.024, 0.060, radius=0.006, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
        f"stand_brace_{side}",
    )


def _build_yoke_mesh():
    return mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, 0.195, 0.060),
                (0.01, 0.195, 0.110),
                (0.05, 0.195, 0.180),
                (0.045, 0.195, 0.270),
                (0.020, 0.105, 0.305),
                (0.000, 0.000, 0.315),
                (0.020, -0.105, 0.305),
                (0.045, -0.195, 0.270),
                (0.05, -0.195, 0.180),
                (0.01, -0.195, 0.110),
                (0.0, -0.195, 0.060),
            ],
            profile=rounded_rect_profile(0.038, 0.068, radius=0.010, corner_segments=7),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "field_yoke_frame",
    )


def _build_can_shell_mesh():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.120, -0.145),
                (0.123, -0.110),
                (0.126, -0.015),
                (0.129, 0.075),
                (0.135, 0.140),
                (0.140, 0.155),
            ],
            [
                (0.102, -0.140),
                (0.104, -0.020),
                (0.108, 0.090),
                (0.115, 0.145),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "spotlight_can_shell",
    )


def _build_handle_mesh():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.098, 0.0, 0.132),
                (-0.082, 0.0, 0.178),
                (-0.020, 0.0, 0.212),
                (0.070, 0.0, 0.208),
                (0.122, 0.0, 0.168),
                (0.138, 0.0, 0.136),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "service_handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_spotlight")

    powder_black = model.material("powder_black", rgba=(0.11, 0.12, 0.13, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    amber_bronze = model.material("amber_bronze", rgba=(0.60, 0.48, 0.28, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    stand = model.part("stand_assembly")
    stand.visual(
        _build_base_plate_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="base_plate",
    )
    stand.visual(
        Box((0.43, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.145, 0.0275)),
        material=machine_gray,
        name="left_skid",
    )
    stand.visual(
        Box((0.43, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.145, 0.0275)),
        material=machine_gray,
        name="right_skid",
    )
    stand.visual(
        Box((0.11, 0.16, 0.34)),
        origin=Origin(xyz=(-0.10, 0.0, 0.20)),
        material=machine_gray,
        name="lower_mast",
    )
    stand.visual(
        Box((0.09, 0.13, 0.15)),
        origin=Origin(xyz=(-0.07, 0.0, 0.445)),
        material=machine_gray,
        name="upper_mast",
    )
    stand.visual(
        Box((0.20, 0.44, 0.05)),
        origin=Origin(xyz=(0.00, 0.0, 0.455)),
        material=machine_gray,
        name="top_saddle",
    )
    stand.visual(_build_stand_brace_mesh(1.0), material=machine_gray, name="left_brace")
    stand.visual(_build_stand_brace_mesh(-1.0), material=machine_gray, name="right_brace")
    for index, (x_pos, y_pos) in enumerate(
        (
            (0.17, 0.12),
            (0.17, -0.12),
            (-0.17, 0.12),
            (-0.17, -0.12),
        )
    ):
        stand.visual(
            Box((0.06, 0.05, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=rubber,
            name=f"foot_pad_{index + 1}",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.48, 0.36, 0.52)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
    )

    yoke = model.part("yoke_frame")
    yoke.visual(
        Box((0.08, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, 0.195, 0.030)),
        material=machine_gray,
        name="left_foot",
    )
    yoke.visual(
        Box((0.08, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, -0.195, 0.030)),
        material=machine_gray,
        name="right_foot",
    )
    yoke.visual(
        Box((0.050, 0.030, 0.250)),
        origin=Origin(xyz=(0.0, 0.195, 0.155)),
        material=machine_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.050, 0.030, 0.250)),
        origin=Origin(xyz=(0.0, -0.195, 0.155)),
        material=machine_gray,
        name="right_arm",
    )
    yoke.visual(
        Box((0.216, 0.030, 0.030)),
        origin=Origin(xyz=(-0.107, 0.195, 0.285)),
        material=machine_gray,
        name="left_rear_strut",
    )
    yoke.visual(
        Box((0.216, 0.030, 0.030)),
        origin=Origin(xyz=(-0.107, -0.195, 0.285)),
        material=machine_gray,
        name="right_rear_strut",
    )
    yoke.visual(
        Box((0.030, 0.360, 0.040)),
        origin=Origin(xyz=(-0.215, 0.0, 0.290)),
        material=machine_gray,
        name="rear_bridge",
    )
    yoke.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.05, 0.186, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_clamp_disc",
    )
    yoke.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.05, -0.186, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_clamp_disc",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.05, 0.213, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_lock_hub",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.05, -0.213, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_lock_hub",
    )
    yoke.visual(
        Cylinder(radius=0.045, length=0.008),
        origin=Origin(xyz=(0.05, 0.232, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="left_lock_cap",
    )
    yoke.visual(
        Cylinder(radius=0.045, length=0.008),
        origin=Origin(xyz=(0.05, -0.232, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="right_lock_cap",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.12, 0.46, 0.34)),
        mass=4.5,
        origin=Origin(xyz=(0.02, 0.0, 0.17)),
    )

    head = model.part("spotlight_head")
    head.visual(
        _build_can_shell_mesh(),
        origin=Origin(xyz=(0.01, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="main_can_shell",
    )
    head.visual(
        _ring_shell_mesh(
            outer_radius=0.146,
            inner_radius=0.122,
            length=0.022,
            name="front_bezel_ring",
        ),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.104, length=0.150),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="optics_barrel",
    )
    head.visual(
        Box((0.11, 0.18, 0.090)),
        origin=Origin(xyz=(-0.10, 0.0, 0.120)),
        material=machine_gray,
        name="rear_service_cowl",
    )
    head.visual(
        Box((0.020, 0.150, 0.072)),
        origin=Origin(xyz=(-0.155, 0.0, 0.118)),
        material=dark_steel,
        name="service_panel",
    )
    head.visual(
        Box((0.012, 0.018, 0.065)),
        origin=Origin(xyz=(-0.145, 0.052, 0.118)),
        material=amber_bronze,
        name="left_panel_latch",
    )
    head.visual(
        Box((0.012, 0.018, 0.065)),
        origin=Origin(xyz=(-0.145, -0.052, 0.118)),
        material=amber_bronze,
        name="right_panel_latch",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.118, tube=0.005), "cooling_fin_rear"),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_cooling_fin",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.118, tube=0.005), "cooling_fin_mid"),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="mid_cooling_fin",
    )
    head.visual(_build_handle_mesh(), material=machine_gray, name="service_handle")
    head.visual(
        Cylinder(radius=0.015, length=0.348),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, 0.132, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_collar",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, -0.132, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_collar",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(xyz=(0.0, 0.163, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber_bronze,
        name="left_friction_washer",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(xyz=(0.0, -0.163, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber_bronze,
        name="right_friction_washer",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=0.36),
        mass=6.5,
        origin=Origin(xyz=(0.01, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.FIXED,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.02, 0.0, 0.48)),
    )
    model.articulation(
        "yoke_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.05, 0.0, 0.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=-0.50, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand_assembly")
    yoke = object_model.get_part("yoke_frame")
    head = object_model.get_part("spotlight_head")
    tilt = object_model.get_articulation("yoke_to_head_tilt")

    top_saddle = stand.get_visual("top_saddle")
    left_foot = yoke.get_visual("left_foot")
    right_foot = yoke.get_visual("right_foot")
    left_clamp = yoke.get_visual("left_clamp_disc")
    right_clamp = yoke.get_visual("right_clamp_disc")
    left_washer = head.get_visual("left_friction_washer")
    right_washer = head.get_visual("right_friction_washer")
    front_bezel = head.get_visual("front_bezel")

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

    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(yoke, stand, elem_a=left_foot, elem_b=top_saddle, name="left_yoke_foot_seated")
        ctx.expect_contact(yoke, stand, elem_a=right_foot, elem_b=top_saddle, name="right_yoke_foot_seated")
        ctx.expect_contact(
            head,
            yoke,
            elem_a=left_washer,
            elem_b=left_clamp,
            name="left_friction_stack_captured",
        )
        ctx.expect_contact(
            head,
            yoke,
            elem_a=right_washer,
            elem_b=right_clamp,
            name="right_friction_stack_captured",
        )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_down_tilt")

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_up_tilt")

    with ctx.pose({tilt: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(head, elem=front_bezel.name)
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(head, elem=front_bezel.name)
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        lowered_aabb = ctx.part_element_world_aabb(head, elem=front_bezel.name)

    closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2]) if closed_aabb is not None else None
    raised_center_z = 0.5 * (raised_aabb[0][2] + raised_aabb[1][2]) if raised_aabb is not None else None
    lowered_center_z = 0.5 * (lowered_aabb[0][2] + lowered_aabb[1][2]) if lowered_aabb is not None else None

    ctx.check(
        "tilt_raises_bezel_for_positive_angles",
        (
            closed_center_z is not None
            and raised_center_z is not None
            and raised_center_z > closed_center_z + 0.10
        ),
        details=(
            f"closed_z={closed_center_z}, raised_z={raised_center_z}; "
            "positive tilt should lift the front bezel clearly upward."
        ),
    )
    ctx.check(
        "tilt_lowers_bezel_for_negative_angles",
        (
            closed_center_z is not None
            and lowered_center_z is not None
            and lowered_center_z < closed_center_z - 0.06
        ),
        details=(
            f"closed_z={closed_center_z}, lowered_z={lowered_center_z}; "
            "negative tilt should aim the beam downward."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
