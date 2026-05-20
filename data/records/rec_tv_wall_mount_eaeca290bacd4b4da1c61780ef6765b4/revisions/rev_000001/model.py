from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_W = 0.090
PLATE_H = 0.240
PLATE_T = 0.008

HINGE_R = 0.010
CENTER_BARREL_LEN = 0.016
SIDE_BARREL_LEN = 0.008
SIDE_BARREL_OFFSET = 0.012
HINGE_Y = 0.036

ARM_BAR_W = 0.030
ARM_BAR_T = 0.014
ARM_SHOULDER_W = 0.036
ARM_SHOULDER_T = 0.018
INNER_ARM_LEN = 0.180
OUTER_ARM_LEN = 0.180

SWIVEL_NECK_Y = 0.033
TILT_AXIS_Y = 0.060
TILT_R = 0.010

HEAD_PLATE_W = 0.160
HEAD_PLATE_H = 0.120
HEAD_PLATE_T = 0.006
HEAD_PLATE_Y = 0.050


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_cyl_z(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=center), material=material, name=name)


def _add_cyl_x(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cyl_y(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_wall_plate(part, structure_material: str, bolt_material: str) -> None:
    _add_box(part, (PLATE_W, PLATE_T, PLATE_H), (0.0, 0.5 * PLATE_T, 0.0), structure_material, name="plate")
    _add_box(part, (0.050, 0.014, 0.060), (0.0, 0.015, 0.0), structure_material, name="center_block")
    _add_box(part, (0.028, 0.024, 0.010), (0.0, 0.024, SIDE_BARREL_OFFSET), structure_material, name="upper_lug")
    _add_box(part, (0.028, 0.024, 0.010), (0.0, 0.024, -SIDE_BARREL_OFFSET), structure_material, name="lower_lug")
    _add_cyl_z(part, HINGE_R, SIDE_BARREL_LEN, (0.0, HINGE_Y, SIDE_BARREL_OFFSET), structure_material, name="upper_hinge")
    _add_cyl_z(part, HINGE_R, SIDE_BARREL_LEN, (0.0, HINGE_Y, -SIDE_BARREL_OFFSET), structure_material, name="lower_hinge")

    for x_pos in (-0.028, 0.028):
        for z_pos in (-0.075, 0.075):
            _add_cyl_y(part, 0.005, 0.004, (x_pos, PLATE_T + 0.002, z_pos), bolt_material)


def _build_arm_link(part, length: float, material: str) -> None:
    _add_cyl_z(part, HINGE_R, CENTER_BARREL_LEN, (0.0, 0.0, 0.0), material, name="base_barrel")
    _add_box(part, (ARM_SHOULDER_W, 0.028, ARM_SHOULDER_T), (0.0, 0.014, 0.0), material, name="base_shoulder")
    _add_box(part, (ARM_BAR_W, length - 0.066, ARM_BAR_T), (0.0, 0.083, 0.0), material, name="main_bar")

    _add_box(part, (0.018, 0.022, 0.008), (0.0, length - 0.034, 0.010), material, name="upper_rib")
    _add_box(part, (0.018, 0.022, 0.008), (0.0, length - 0.034, -0.010), material, name="lower_rib")
    _add_box(part, (0.020, 0.018, 0.008), (0.0, length - 0.017, SIDE_BARREL_OFFSET), material, name="upper_mount")
    _add_box(part, (0.020, 0.018, 0.008), (0.0, length - 0.017, -SIDE_BARREL_OFFSET), material, name="lower_mount")
    _add_cyl_z(part, HINGE_R, SIDE_BARREL_LEN, (0.0, length, SIDE_BARREL_OFFSET), material, name="upper_barrel")
    _add_cyl_z(part, HINGE_R, SIDE_BARREL_LEN, (0.0, length, -SIDE_BARREL_OFFSET), material, name="lower_barrel")


def _build_swivel_support(part, material: str) -> None:
    _add_cyl_z(part, HINGE_R, CENTER_BARREL_LEN, (0.0, 0.0, 0.0), material, name="swivel_barrel")
    _add_box(part, (0.034, 0.028, 0.018), (0.0, 0.014, 0.0), material, name="swivel_shoulder")
    _add_box(part, (0.028, 0.038, 0.014), (0.0, SWIVEL_NECK_Y, 0.0), material, name="swivel_neck")
    _add_box(part, (0.042, 0.010, 0.028), (0.0, 0.042, 0.0), material, name="tilt_crossmember")
    _add_box(part, (0.008, 0.024, 0.014), (SIDE_BARREL_OFFSET, 0.054, 0.0), material, name="right_cheek")
    _add_box(part, (0.008, 0.024, 0.014), (-SIDE_BARREL_OFFSET, 0.054, 0.0), material, name="left_cheek")
    _add_cyl_x(part, TILT_R, SIDE_BARREL_LEN, (SIDE_BARREL_OFFSET, TILT_AXIS_Y, 0.0), material, name="right_tilt_ear")
    _add_cyl_x(part, TILT_R, SIDE_BARREL_LEN, (-SIDE_BARREL_OFFSET, TILT_AXIS_Y, 0.0), material, name="left_tilt_ear")


def _build_display_head(part, structure_material: str, fastener_material: str) -> None:
    _add_cyl_x(part, TILT_R, CENTER_BARREL_LEN, (0.0, 0.0, 0.0), structure_material, name="tilt_barrel")
    _add_box(part, (0.014, 0.024, 0.040), (0.0, 0.012, 0.0), structure_material, name="head_neck")
    _add_box(part, (0.040, 0.036, 0.070), (0.0, 0.032, 0.0), structure_material, name="head_backplate")
    _add_box(part, (HEAD_PLATE_W, HEAD_PLATE_T, HEAD_PLATE_H), (0.0, HEAD_PLATE_Y, 0.0), structure_material, name="vesa_plate")

    for x_pos in (-0.050, 0.050):
        for z_pos in (-0.040, 0.040):
            _add_cyl_y(part, 0.004, 0.004, (x_pos, HEAD_PLATE_Y + 0.004, z_pos), fastener_material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hinge_tv_wall_mount")

    model.material("powder_black", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    model.material("soft_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("zinc", rgba=(0.72, 0.74, 0.77, 1.0))

    wall_plate = model.part("wall_plate")
    _build_wall_plate(wall_plate, "powder_black", "zinc")

    inner_arm = model.part("inner_arm")
    _build_arm_link(inner_arm, INNER_ARM_LEN, "graphite")

    outer_arm = model.part("outer_arm")
    _build_arm_link(outer_arm, OUTER_ARM_LEN, "graphite")

    swivel_support = model.part("swivel_support")
    _build_swivel_support(swivel_support, "powder_black")

    display_head = model.part("display_head")
    _build_display_head(display_head, "soft_black", "zinc")

    model.articulation(
        "wall_pivot",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=inner_arm,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.4, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(0.0, INNER_ARM_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-2.50, upper=2.50),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=swivel_support,
        origin=Origin(xyz=(0.0, OUTER_ARM_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_support,
        child=display_head,
        origin=Origin(xyz=(0.0, TILT_AXIS_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.50, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    swivel_support = object_model.get_part("swivel_support")
    display_head = object_model.get_part("display_head")

    wall_pivot = object_model.get_articulation("wall_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

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
        "revolute axes match wall-mount mechanism",
        wall_pivot.axis == (0.0, 0.0, 1.0)
        and elbow_pivot.axis == (0.0, 0.0, 1.0)
        and head_swivel.axis == (0.0, 0.0, 1.0)
        and head_tilt.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes were wall={wall_pivot.axis}, elbow={elbow_pivot.axis}, "
            f"swivel={head_swivel.axis}, tilt={head_tilt.axis}"
        ),
    )
    ctx.check(
        "revolute limits stay within realistic wall-mount ranges",
        wall_pivot.motion_limits is not None
        and wall_pivot.motion_limits.lower == -1.45
        and wall_pivot.motion_limits.upper == 1.45
        and elbow_pivot.motion_limits is not None
        and elbow_pivot.motion_limits.lower == -2.50
        and elbow_pivot.motion_limits.upper == 2.50
        and head_swivel.motion_limits is not None
        and head_swivel.motion_limits.lower == -1.20
        and head_swivel.motion_limits.upper == 1.20
        and head_tilt.motion_limits is not None
        and head_tilt.motion_limits.lower == -0.50
        and head_tilt.motion_limits.upper == 0.30,
        details="one or more articulation limits were changed from the intended mount envelope",
    )

    ctx.expect_contact(inner_arm, wall_plate, name="wall plate contacts inner arm hinge")
    ctx.expect_contact(outer_arm, inner_arm, name="inner arm contacts outer arm elbow")
    ctx.expect_contact(swivel_support, outer_arm, name="outer arm contacts swivel support")
    ctx.expect_contact(display_head, swivel_support, name="swivel support contacts display head")
    ctx.expect_gap(display_head, wall_plate, axis="y", min_gap=0.28, name="display head stands proud of wall plate")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple(0.5 * (aabb[0][i] + aabb[1][i]) for i in range(3))

    with ctx.pose({wall_pivot: 0.0, elbow_pivot: 0.0, head_swivel: 0.0, head_tilt: 0.0}):
        neutral_head_aabb = ctx.part_world_aabb(display_head)
        neutral_head_center = _aabb_center(neutral_head_aabb)

    with ctx.pose({wall_pivot: 0.85, elbow_pivot: 0.0, head_swivel: 0.0, head_tilt: 0.0}):
        swung_head_center = _aabb_center(ctx.part_world_aabb(display_head))
        if neutral_head_center is not None and swung_head_center is not None:
            ctx.check(
                "wall pivot swings the head laterally",
                swung_head_center[0] < neutral_head_center[0] - 0.10 and swung_head_center[1] > 0.12,
                details=f"neutral={neutral_head_center}, swung={swung_head_center}",
            )

    with ctx.pose({wall_pivot: 0.0, elbow_pivot: 1.20, head_swivel: 0.0, head_tilt: 0.0}):
        folded_head_center = _aabb_center(ctx.part_world_aabb(display_head))
        if neutral_head_center is not None and folded_head_center is not None:
            ctx.check(
                "elbow pivot folds the second link inward",
                folded_head_center[1] < neutral_head_center[1] - 0.09,
                details=f"neutral={neutral_head_center}, folded={folded_head_center}",
            )

    with ctx.pose({wall_pivot: 0.0, elbow_pivot: 0.0, head_swivel: 0.55, head_tilt: 0.0}):
        swiveled_head_center = _aabb_center(ctx.part_world_aabb(display_head))
        if neutral_head_center is not None and swiveled_head_center is not None:
            ctx.check(
                "head swivel yaws the display head independently of the arm",
                swiveled_head_center[0] < neutral_head_center[0] - 0.015,
                details=f"neutral={neutral_head_center}, swiveled={swiveled_head_center}",
            )

    with ctx.pose({wall_pivot: 0.0, elbow_pivot: 0.0, head_swivel: 0.0, head_tilt: 0.0}):
        neutral_head_aabb = ctx.part_world_aabb(display_head)
    with ctx.pose({wall_pivot: 0.0, elbow_pivot: 0.0, head_swivel: 0.0, head_tilt: 0.25}):
        tilted_head_aabb = ctx.part_world_aabb(display_head)
        if neutral_head_aabb is not None and tilted_head_aabb is not None:
            ctx.check(
                "positive head tilt raises the top of the display head",
                tilted_head_aabb[1][2] > neutral_head_aabb[1][2] + 0.010,
                details=f"neutral_top={neutral_head_aabb[1][2]:.4f}, tilted_top={tilted_head_aabb[1][2]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
