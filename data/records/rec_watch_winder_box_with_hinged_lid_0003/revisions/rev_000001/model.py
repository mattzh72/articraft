from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

BOX_WIDTH = 0.192
BOX_DEPTH = 0.168
BASE_HEIGHT = 0.110
BOTTOM_THICKNESS = 0.012
WALL_THICKNESS = 0.012

LID_HEIGHT = 0.048
LID_FRAME = 0.018
LID_GLASS = 0.003
LID_AXIS_Z = 0.006
HINGE_RADIUS = 0.005

CRADLE_AXIS_Z = 0.071
CRADLE_PILLOW_X = 0.080
CRADLE_PILLOW_Y = 0.060
CRADLE_PILLOW_Z = 0.048


def _x_axis_origin(*, xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _rounded_rect_section(
    width: float,
    height: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_watch_winder_box", assets=ASSETS)

    shell_matte = model.material("shell_matte", rgba=(0.12, 0.12, 0.13, 1.0))
    shell_satin = model.material("shell_satin", rgba=(0.19, 0.19, 0.20, 1.0))
    liner_matte = model.material("liner_matte", rgba=(0.22, 0.23, 0.24, 1.0))
    pillow_matte = model.material("pillow_matte", rgba=(0.30, 0.30, 0.31, 1.0))
    hardware_satin = model.material("hardware_satin", rgba=(0.69, 0.66, 0.60, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.56, 0.62, 0.67, 0.22))

    body_wall_height = BASE_HEIGHT - BOTTOM_THICKNESS
    body_wall_z = BOTTOM_THICKNESS + body_wall_height / 2.0

    base = model.part("base")
    base.visual(
        Box((BOX_WIDTH, BOX_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=shell_matte,
        name="bottom_panel",
    )
    base.visual(
        Box((WALL_THICKNESS, BOX_DEPTH, body_wall_height)),
        origin=Origin(
            xyz=(-BOX_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, body_wall_z)
        ),
        material=shell_matte,
        name="left_wall",
    )
    base.visual(
        Box((WALL_THICKNESS, BOX_DEPTH, body_wall_height)),
        origin=Origin(
            xyz=(BOX_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, body_wall_z)
        ),
        material=shell_matte,
        name="right_wall",
    )
    base.visual(
        Box((BOX_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, body_wall_height)),
        origin=Origin(
            xyz=(0.0, -BOX_DEPTH / 2.0 + WALL_THICKNESS / 2.0, body_wall_z)
        ),
        material=shell_matte,
        name="front_wall",
    )
    base.visual(
        Box((BOX_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, body_wall_height)),
        origin=Origin(
            xyz=(0.0, BOX_DEPTH / 2.0 - WALL_THICKNESS / 2.0, body_wall_z)
        ),
        material=shell_matte,
        name="rear_wall",
    )
    base.visual(
        Box(
            (
                BOX_WIDTH - 2.0 * WALL_THICKNESS - 0.006,
                BOX_DEPTH - 2.0 * WALL_THICKNESS - 0.006,
                0.006,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS + 0.003)),
        material=liner_matte,
        name="liner_floor",
    )
    base.visual(
        Box((0.128, 0.088, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=shell_satin,
        name="cradle_plinth",
    )

    pedestal_x = 0.071
    pedestal_size = (0.018, 0.040, 0.060)
    pedestal_z = 0.048
    base.visual(
        Box(pedestal_size),
        origin=Origin(xyz=(-pedestal_x, 0.0, pedestal_z)),
        material=liner_matte,
        name="left_pedestal",
    )
    base.visual(
        Box(pedestal_size),
        origin=Origin(xyz=(pedestal_x, 0.0, pedestal_z)),
        material=liner_matte,
        name="right_pedestal",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=_x_axis_origin(xyz=(-0.058, 0.0, CRADLE_AXIS_Z)),
        material=hardware_satin,
        name="left_bearing",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=_x_axis_origin(xyz=(0.058, 0.0, CRADLE_AXIS_Z)),
        material=hardware_satin,
        name="right_bearing",
    )

    trim_z = BASE_HEIGHT - 0.0015
    trim_h = 0.003
    base.visual(
        Box((BOX_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, trim_h)),
        origin=Origin(
            xyz=(0.0, -BOX_DEPTH / 2.0 + WALL_THICKNESS / 2.0, trim_z)
        ),
        material=hardware_satin,
        name="front_trim",
    )
    base.visual(
        Box((BOX_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, trim_h)),
        origin=Origin(
            xyz=(0.0, BOX_DEPTH / 2.0 - WALL_THICKNESS / 2.0, trim_z)
        ),
        material=hardware_satin,
        name="rear_trim",
    )
    base.visual(
        Box((WALL_THICKNESS, BOX_DEPTH, trim_h)),
        origin=Origin(
            xyz=(-BOX_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, trim_z)
        ),
        material=hardware_satin,
        name="left_trim",
    )
    base.visual(
        Box((WALL_THICKNESS, BOX_DEPTH, trim_h)),
        origin=Origin(
            xyz=(BOX_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, trim_z)
        ),
        material=hardware_satin,
        name="right_trim",
    )

    hinge_axis_world = (0.0, BOX_DEPTH / 2.0 + HINGE_RADIUS, BASE_HEIGHT + LID_AXIS_Z)
    base.visual(
        Cylinder(radius=0.0025, length=0.118),
        origin=_x_axis_origin(xyz=hinge_axis_world),
        material=hardware_satin,
        name="hinge_pin",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=_x_axis_origin(
            xyz=(-0.045, BOX_DEPTH / 2.0 + HINGE_RADIUS, BASE_HEIGHT + LID_AXIS_Z)
        ),
        material=hardware_satin,
        name="left_hinge_knuckle",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=_x_axis_origin(
            xyz=(0.045, BOX_DEPTH / 2.0 + HINGE_RADIUS, BASE_HEIGHT + LID_AXIS_Z)
        ),
        material=hardware_satin,
        name="right_hinge_knuckle",
    )
    base.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(
            xyz=(-0.045, BOX_DEPTH / 2.0 + 0.003, BASE_HEIGHT + 0.002)
        ),
        material=hardware_satin,
        name="left_hinge_leaf",
    )
    base.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(
            xyz=(0.045, BOX_DEPTH / 2.0 + 0.003, BASE_HEIGHT + 0.002)
        ),
        material=hardware_satin,
        name="right_hinge_leaf",
    )
    base.inertial = Inertial.from_geometry(
        Box((BOX_WIDTH, BOX_DEPTH, BASE_HEIGHT + 0.020)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + 0.020) / 2.0)),
    )

    lid = model.part("lid")
    lid_center_y = -(BOX_DEPTH / 2.0 + HINGE_RADIUS)
    lid_center_z = LID_HEIGHT / 2.0 - LID_AXIS_Z
    lid.visual(
        Box((LID_FRAME, BOX_DEPTH, LID_HEIGHT)),
        origin=Origin(
            xyz=(-BOX_WIDTH / 2.0 + LID_FRAME / 2.0, lid_center_y, lid_center_z)
        ),
        material=shell_matte,
        name="left_frame",
    )
    lid.visual(
        Box((LID_FRAME, BOX_DEPTH, LID_HEIGHT)),
        origin=Origin(
            xyz=(BOX_WIDTH / 2.0 - LID_FRAME / 2.0, lid_center_y, lid_center_z)
        ),
        material=shell_matte,
        name="right_frame",
    )
    lid.visual(
        Box((BOX_WIDTH - 2.0 * LID_FRAME, LID_FRAME, LID_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -BOX_DEPTH + LID_FRAME / 2.0 - HINGE_RADIUS,
                lid_center_z,
            )
        ),
        material=shell_matte,
        name="front_frame",
    )
    lid.visual(
        Box((BOX_WIDTH - 2.0 * LID_FRAME, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.012, 0.030)),
        material=shell_matte,
        name="rear_frame",
    )
    lid.visual(
        Box(
            (
                BOX_WIDTH - 2.0 * LID_FRAME + 0.0004,
                BOX_DEPTH - 2.0 * LID_FRAME + 0.0004,
                LID_GLASS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -(BOX_DEPTH / 2.0 + HINGE_RADIUS),
                LID_HEIGHT - LID_AXIS_Z - LID_GLASS / 2.0 - 0.003,
            )
        ),
        material=glass_smoke,
        name="glass_panel",
    )
    lid.visual(
        Box((BOX_WIDTH - 2.0 * LID_FRAME, 0.008, 0.008)),
        origin=Origin(
            xyz=(0.0, -BOX_DEPTH + 0.004 - HINGE_RADIUS, 0.008)
        ),
        material=hardware_satin,
        name="front_accent",
    )

    lip_thickness = 0.005
    lip_height = 0.018
    lip_center_z = -LID_AXIS_Z + lip_height / 2.0
    side_lip_depth = BOX_DEPTH - 0.040
    lid.visual(
        Box((lip_thickness, side_lip_depth, lip_height)),
        origin=Origin(
            xyz=(
                -BOX_WIDTH / 2.0 + LID_FRAME + lip_thickness / 2.0 - 0.0005,
                -(BOX_DEPTH / 2.0 + HINGE_RADIUS + 0.010),
                lip_center_z,
            )
        ),
        material=shell_satin,
        name="left_inner_lip",
    )
    lid.visual(
        Box((lip_thickness, side_lip_depth, lip_height)),
        origin=Origin(
            xyz=(
                BOX_WIDTH / 2.0 - LID_FRAME - lip_thickness / 2.0 + 0.0005,
                -(BOX_DEPTH / 2.0 + HINGE_RADIUS + 0.010),
                lip_center_z,
            )
        ),
        material=shell_satin,
        name="right_inner_lip",
    )
    lid.visual(
        Box((BOX_WIDTH - 2.0 * LID_FRAME + 0.001, lip_thickness, lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                -BOX_DEPTH + LID_FRAME - lip_thickness / 2.0 - HINGE_RADIUS + 0.0005,
                lip_center_z,
            )
        ),
        material=shell_satin,
        name="front_inner_lip",
    )
    lid.visual(
        Box((0.056, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, -0.007, 0.011)),
        material=shell_satin,
        name="hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.0052, length=0.066),
        origin=_x_axis_origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware_satin,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((BOX_WIDTH, BOX_DEPTH, LID_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z)),
    )

    cradle = model.part("cradle")
    pillow_mesh = mesh_from_geometry(
        section_loft(
            [
                _rounded_rect_section(0.070, 0.050, 0.010, -0.021),
                _rounded_rect_section(0.080, 0.060, 0.016, 0.0),
                _rounded_rect_section(0.072, 0.052, 0.012, 0.021),
            ]
        ),
        ASSETS.mesh_path("watch_winder_pillow.obj"),
    )
    cradle.visual(
        Cylinder(radius=0.005, length=0.108),
        origin=_x_axis_origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware_satin,
        name="axle",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.076),
        origin=_x_axis_origin(xyz=(0.0, 0.0, 0.0)),
        material=shell_satin,
        name="drum_body",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.008),
        origin=_x_axis_origin(xyz=(-0.030, 0.0, 0.0)),
        material=hardware_satin,
        name="left_drum_collar",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.008),
        origin=_x_axis_origin(xyz=(0.030, 0.0, 0.0)),
        material=hardware_satin,
        name="right_drum_collar",
    )
    cradle.visual(
        pillow_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pillow_matte,
        name="pillow_block",
    )
    cradle.visual(
        Box((0.050, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=liner_matte,
        name="pillow_crown",
    )
    cradle.visual(
        Box((0.068, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=shell_satin,
        name="rear_retainer",
    )
    cradle.visual(
        Box((0.068, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=shell_satin,
        name="front_retainer",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.108, 0.064, 0.060)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=hinge_axis_world),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "winder_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, CRADLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")

    lid_hinge = object_model.get_articulation("lid_hinge")
    winder_spin = object_model.get_articulation("winder_spin")

    hinge_pin = base.get_visual("hinge_pin")
    left_bearing = base.get_visual("left_bearing")
    right_bearing = base.get_visual("right_bearing")
    front_trim = base.get_visual("front_trim")
    glass_panel = lid.get_visual("glass_panel")
    front_frame = lid.get_visual("front_frame")
    hinge_barrel = lid.get_visual("hinge_barrel")
    axle = cradle.get_visual("axle")
    pillow_block = cradle.get_visual("pillow_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        base,
        lid,
        elem_a=hinge_pin,
        elem_b=hinge_barrel,
        reason="Captive rear hinge pin intentionally occupies the lid barrel volume.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        cradle,
        base,
        elem_a=axle,
        elem_b=left_bearing,
        name="left_bearing_contact",
    )
    ctx.expect_contact(
        cradle,
        base,
        elem_a=axle,
        elem_b=right_bearing,
        name="right_bearing_contact",
    )
    ctx.expect_within(cradle, base, axes="xy", margin=0.0, name="cradle_within_box_footprint")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            positive_elem=front_frame,
            negative_elem=front_trim,
            name="closed_front_seam_gap",
        )
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.14, name="closed_lid_overlap")
        ctx.expect_overlap(
            lid,
            cradle,
            axes="xy",
            min_overlap=0.06,
            elem_a=glass_panel,
            elem_b=pillow_block,
            name="window_centers_cradle",
        )
        ctx.expect_contact(lid, base, name="lid_supported_closed")

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None and lid_limits.lower is not None:
        with ctx.pose({lid_hinge: lid_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")

        closed_lid_aabb = ctx.part_world_aabb(lid)
        assert closed_lid_aabb is not None

        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
            open_lid_aabb = ctx.part_world_aabb(lid)
            assert open_lid_aabb is not None
            ctx.check(
                "lid_opens_above_box",
                open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.050,
                details=(
                    f"closed lid z_max={closed_lid_aabb[1][2]:.4f}, "
                    f"open lid z_max={open_lid_aabb[1][2]:.4f}"
                ),
            )
            ctx.check(
                "lid_swings_rearward",
                open_lid_aabb[1][1] > closed_lid_aabb[1][1] + 0.045,
                details=(
                    f"closed lid y_max={closed_lid_aabb[1][1]:.4f}, "
                    f"open lid y_max={open_lid_aabb[1][1]:.4f}"
                ),
            )

    with ctx.pose({winder_spin: 0.0}):
        cradle_rest_aabb = ctx.part_world_aabb(cradle)
        assert cradle_rest_aabb is not None
        pillow_rest_aabb = ctx.part_element_world_aabb(cradle, elem="pillow_block")
        assert pillow_rest_aabb is not None
    with ctx.pose({winder_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="cradle_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="cradle_quarter_turn_no_floating")
        ctx.expect_contact(
            cradle,
            base,
            elem_a=axle,
            elem_b=left_bearing,
            name="left_bearing_contact_quarter_turn",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a=axle,
            elem_b=right_bearing,
            name="right_bearing_contact_quarter_turn",
        )
        cradle_quarter_aabb = ctx.part_element_world_aabb(cradle, elem="pillow_block")
        assert cradle_quarter_aabb is not None
        rest_y = pillow_rest_aabb[1][1] - pillow_rest_aabb[0][1]
        quarter_y = cradle_quarter_aabb[1][1] - cradle_quarter_aabb[0][1]
        ctx.check(
            "cradle_profile_changes_when_spinning",
            quarter_y < rest_y - 0.008,
            details=f"rest y span={rest_y:.4f}, quarter-turn y span={quarter_y:.4f}",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_motion_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=48, name="sampled_motion_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
