from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from functools import lru_cache

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


@lru_cache(maxsize=1)
def _launch_tube_mesh():
    outer_radius = 0.11
    wall = 0.008
    length = 1.44

    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=56)
    inner = CylinderGeometry(
        radius=outer_radius - wall,
        height=length + 0.04,
        radial_segments=56,
    )
    tube_shell = boolean_difference(outer, inner)
    tube_shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(tube_shell, ASSETS.mesh_path("launch_tube_shell.obj"))


@lru_cache(maxsize=1)
def _support_arm_mesh():
    profile = [
        (0.08, -0.18),
        (0.20, -0.22),
        (0.34, -0.10),
        (0.46, 0.08),
        (0.46, 0.46),
        (0.32, 0.60),
        (0.16, 0.54),
        (0.08, 0.22),
    ]
    arm = ExtrudeGeometry(profile, height=0.04, cap=True, center=True, closed=True)
    arm.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(arm, ASSETS.mesh_path("support_arm_plate.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="truck_bed_missile_launcher_module", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.34, 0.24, 1.0))
    tube_paint = model.material("tube_paint", rgba=(0.18, 0.25, 0.16, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.39, 0.40, 0.42, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((2.40, 1.60, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="floor_plate",
    )
    base_frame.visual(
        Box((2.40, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.75, 0.12)),
        material=frame_paint,
        name="left_rail",
    )
    base_frame.visual(
        Box((2.40, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.75, 0.12)),
        material=frame_paint,
        name="right_rail",
    )
    base_frame.visual(
        Box((0.10, 1.40, 0.12)),
        origin=Origin(xyz=(1.15, 0.0, 0.12)),
        material=frame_paint,
        name="front_rail",
    )
    base_frame.visual(
        Box((0.10, 1.40, 0.12)),
        origin=Origin(xyz=(-1.15, 0.0, 0.12)),
        material=frame_paint,
        name="rear_rail",
    )
    base_frame.visual(
        Box((0.08, 1.40, 0.10)),
        origin=Origin(xyz=(-0.55, 0.0, 0.11)),
        material=frame_paint,
        name="rear_crossmember",
    )
    base_frame.visual(
        Box((0.08, 1.40, 0.10)),
        origin=Origin(xyz=(0.55, 0.0, 0.11)),
        material=frame_paint,
        name="front_crossmember",
    )
    for name, x_sign, y_sign in (
        ("foot_fl", 1.0, 1.0),
        ("foot_fr", 1.0, -1.0),
        ("foot_rl", -1.0, 1.0),
        ("foot_rr", -1.0, -1.0),
    ):
        base_frame.visual(
            Box((0.22, 0.22, 0.04)),
            origin=Origin(xyz=(0.92 * x_sign, 0.58 * y_sign, -0.02)),
            material=dark_steel,
            name=name,
        )
    base_frame.visual(
        Cylinder(radius=0.46, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=bearing_steel,
        name="lower_ring",
    )

    slew_platform = model.part("slew_platform")
    slew_platform.visual(
        Cylinder(radius=0.43, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=bearing_steel,
        name="upper_ring",
    )
    slew_platform.visual(
        Box((0.95, 1.15, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=frame_paint,
        name="turret_deck",
    )
    slew_platform.visual(
        Box((0.40, 0.40, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=frame_paint,
        name="center_plinth",
    )
    slew_platform.visual(
        Box((0.20, 1.04, 0.22)),
        origin=Origin(xyz=(-0.20, 0.0, 0.25)),
        material=frame_paint,
        name="rear_bridge",
    )
    slew_platform.visual(
        Box((0.16, 1.04, 0.14)),
        origin=Origin(xyz=(0.30, 0.0, 0.24)),
        material=frame_paint,
        name="front_bridge",
    )
    slew_platform.visual(
        Box((0.58, 0.08, 0.78)),
        origin=Origin(xyz=(0.12, 0.56, 0.49)),
        material=frame_paint,
        name="left_cheek",
    )
    slew_platform.visual(
        Box((0.58, 0.08, 0.78)),
        origin=Origin(xyz=(0.12, -0.56, 0.49)),
        material=frame_paint,
        name="right_cheek",
    )
    slew_platform.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.12, 0.50, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="left_trunnion_sleeve",
    )
    slew_platform.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.12, -0.50, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="right_trunnion_sleeve",
    )

    launcher_bank = model.part("launcher_bank")
    launcher_bank.visual(
        Cylinder(radius=0.06, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_axle",
    )
    bundle_z = 0.24
    support_arm_mesh = _support_arm_mesh()
    launcher_bank.visual(
        support_arm_mesh,
        origin=Origin(xyz=(0.0, 0.43, 0.0)),
        material=frame_paint,
        name="left_side_plate",
    )
    launcher_bank.visual(
        support_arm_mesh,
        origin=Origin(xyz=(0.0, -0.43, 0.0)),
        material=frame_paint,
        name="right_side_plate",
    )

    tube_mesh = _launch_tube_mesh()
    y_positions = (-0.327, -0.109, 0.109, 0.327)
    z_positions = (-0.218, 0.0, 0.218)
    for row, z_pos in enumerate(z_positions):
        for col, y_pos in enumerate(y_positions):
            launcher_bank.visual(
                tube_mesh,
                origin=Origin(xyz=(0.76, y_pos, bundle_z + z_pos)),
                material=tube_paint,
                name=f"tube_r{row}_c{col}",
            )

    for name, z_pos in (("rear_top_strap", 0.347), ("rear_bottom_strap", -0.347)):
        launcher_bank.visual(
            Box((0.54, 0.912, 0.04)),
            origin=Origin(xyz=(0.31, 0.0, bundle_z + z_pos)),
            material=frame_paint,
            name=name,
        )
    launcher_bank.visual(
        Box((0.18, 0.76, 0.52)),
        origin=Origin(xyz=(0.40, 0.0, bundle_z + 0.02)),
        material=frame_paint,
        name="rear_bulkhead",
    )
    for name, z_pos in (("front_top_strap", 0.347), ("front_bottom_strap", -0.347)):
        launcher_bank.visual(
            Box((0.10, 0.912, 0.04)),
            origin=Origin(xyz=(1.43, 0.0, bundle_z + z_pos)),
            material=frame_paint,
            name=name,
        )
    for name, y_pos in (("front_left_strap", 0.456), ("front_right_strap", -0.456)):
        launcher_bank.visual(
            Box((0.10, 0.04, 0.70)),
            origin=Origin(xyz=(1.43, y_pos, bundle_z)),
            material=frame_paint,
            name=name,
        )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=slew_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=slew_platform,
        child=launcher_bank,
        origin=Origin(xyz=(0.12, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.9,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    slew_platform = object_model.get_part("slew_platform")
    launcher_bank = object_model.get_part("launcher_bank")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    lower_ring = base_frame.get_visual("lower_ring")
    upper_ring = slew_platform.get_visual("upper_ring")
    turret_deck = slew_platform.get_visual("turret_deck")
    left_sleeve = slew_platform.get_visual("left_trunnion_sleeve")
    right_sleeve = slew_platform.get_visual("right_trunnion_sleeve")
    pivot_axle = launcher_bank.get_visual("pivot_axle")
    lower_center_tube = launcher_bank.get_visual("tube_r0_c1")
    upper_center_tube = launcher_bank.get_visual("tube_r2_c1")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        launcher_bank,
        slew_platform,
        reason="Launcher trunnion axle is seated inside the left bearing sleeve.",
        elem_a=pivot_axle,
        elem_b=left_sleeve,
    )
    ctx.allow_overlap(
        launcher_bank,
        slew_platform,
        reason="Launcher trunnion axle is seated inside the right bearing sleeve.",
        elem_a=pivot_axle,
        elem_b=right_sleeve,
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "azimuth_axis_is_vertical",
        tuple(float(v) for v in azimuth.axis) == (0.0, 0.0, 1.0),
        f"Azimuth axis should be vertical, got {azimuth.axis!r}.",
    )
    ctx.check(
        "azimuth_is_continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        f"Expected continuous azimuth rotation, got {azimuth.articulation_type!r}.",
    )
    ctx.check(
        "elevation_axis_is_transverse",
        abs(float(elevation.axis[0])) < 1e-6
        and abs(abs(float(elevation.axis[1])) - 1.0) < 1e-6
        and abs(float(elevation.axis[2])) < 1e-6,
        f"Elevation axis should run transversely along ±Y, got {elevation.axis!r}.",
    )
    ctx.check(
        "elevation_limits_match_launcher_arc",
        abs(float(elevation.motion_limits.lower) - 0.0) < 1e-6
        and abs(float(elevation.motion_limits.upper) - math.radians(70.0)) < 1e-6,
        (
            "Elevation should travel from horizontal to about 70 degrees; "
            f"got {elevation.motion_limits!r}."
        ),
    )

    ctx.expect_contact(
        slew_platform,
        base_frame,
        elem_a=upper_ring,
        elem_b=lower_ring,
        name="slew_ring_seats_on_base_ring",
    )
    ctx.expect_gap(
        slew_platform,
        base_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_ring,
        negative_elem=lower_ring,
        name="slew_ring_has_no_vertical_float",
    )
    ctx.expect_overlap(
        slew_platform,
        base_frame,
        axes="xy",
        min_overlap=0.84,
        elem_a=upper_ring,
        elem_b=lower_ring,
        name="upper_and_lower_rings_share_bearing_footprint",
    )
    ctx.expect_overlap(
        launcher_bank,
        slew_platform,
        axes="xyz",
        min_overlap=0.03,
        elem_a=pivot_axle,
        elem_b=left_sleeve,
        name="left_trunnion_axle_engages_left_sleeve",
    )
    ctx.expect_overlap(
        launcher_bank,
        slew_platform,
        axes="xyz",
        min_overlap=0.03,
        elem_a=pivot_axle,
        elem_b=right_sleeve,
        name="right_trunnion_axle_engages_right_sleeve",
    )
    ctx.expect_gap(
        launcher_bank,
        slew_platform,
        axis="z",
        min_gap=0.10,
        positive_elem=lower_center_tube,
        negative_elem=turret_deck,
        name="launcher_bank_clears_turret_deck_at_rest",
    )
    ctx.expect_gap(
        launcher_bank,
        slew_platform,
        axis="z",
        min_gap=0.28,
        positive_elem=upper_center_tube,
        negative_elem=turret_deck,
        name="top_row_sits_well_above_turret_deck",
    )
    ctx.expect_within(
        launcher_bank,
        slew_platform,
        axes="y",
        margin=0.11,
        name="launcher_bank_stays_between_cheeks",
    )

    with ctx.pose({elevation: math.radians(55.0)}):
        ctx.expect_gap(
            launcher_bank,
            slew_platform,
            axis="z",
            min_gap=0.20,
            positive_elem=lower_center_tube,
            negative_elem=turret_deck,
            name="elevated_bank_still_clears_turret_deck",
        )
    with ctx.pose({azimuth: math.pi / 2.0, elevation: math.radians(55.0)}):
        ctx.expect_overlap(
            launcher_bank,
            slew_platform,
            axes="xyz",
            min_overlap=0.03,
            elem_a=pivot_axle,
            elem_b=left_sleeve,
            name="left_trunnion_remains_seated_when_slewed_and_elevated",
        )
        ctx.expect_overlap(
            launcher_bank,
            slew_platform,
            axes="xyz",
            min_overlap=0.03,
            elem_a=pivot_axle,
            elem_b=right_sleeve,
            name="right_trunnion_remains_seated_when_slewed_and_elevated",
        )

    base_aabb = ctx.part_world_aabb(base_frame)
    bank_aabb = ctx.part_world_aabb(launcher_bank)
    if base_aabb is not None:
        base_len = base_aabb[1][0] - base_aabb[0][0]
        base_width = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "base_frame_has_truck_bed_proportions",
            2.2 <= base_len <= 2.5 and 1.4 <= base_width <= 1.7,
            f"Base frame footprint should read like a truck-bed module, got {(base_len, base_width)!r}.",
        )
    if bank_aabb is not None:
        bank_len = bank_aabb[1][0] - bank_aabb[0][0]
        bank_height = bank_aabb[1][2] - bank_aabb[0][2]
        ctx.check(
            "tube_bank_is_longer_than_tall",
            1.45 <= bank_len <= 1.62 and 0.60 <= bank_height <= 0.85 and bank_len > bank_height,
            f"Tube bank proportions look off, got {(bank_len, bank_height)!r}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
