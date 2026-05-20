from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.20
BASE_HEIGHT = 0.025
GUIDE_LENGTH = 0.66
GUIDE_WIDTH = 0.10
GUIDE_HEIGHT = 0.06

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.16
CARRIAGE_HEIGHT = 0.09
CARRIAGE_CHANNEL_WIDTH = 0.108
CARRIAGE_CHANNEL_HEIGHT = 0.055
CARRIAGE_CHEEK_THICKNESS = (CARRIAGE_WIDTH - CARRIAGE_CHANNEL_WIDTH) / 2.0

SLIDE_TRAVEL = (GUIDE_LENGTH - CARRIAGE_LENGTH) / 2.0

HOUSING_RADIUS = 0.055
HOUSING_LENGTH = 0.10
HOUSING_BORE_RADIUS = 0.028
HOUSING_FOOT_HEIGHT = 0.03
HOUSING_FOOT_X = 0.11
HOUSING_FOOT_Y = 0.05
HOUSING_AXIS_Z_ON_CARRIAGE = CARRIAGE_HEIGHT + HOUSING_RADIUS


def _build_spindle_housing_shape() -> cq.Workplane:
    body = cq.Workplane("XZ").circle(HOUSING_RADIUS).extrude(HOUSING_LENGTH / 2.0, both=True)
    foot = cq.Workplane("XY").box(HOUSING_FOOT_X, HOUSING_FOOT_Y, HOUSING_FOOT_HEIGHT).translate(
        (0.0, 0.0, -0.04)
    )
    bore = cq.Workplane("XZ").circle(HOUSING_BORE_RADIUS).extrude((HOUSING_LENGTH + 0.03) / 2.0, both=True)
    return body.union(foot).cut(bore)


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple(upper - lower for lower, upper in zip(minimum, maximum))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_mounted_tool_slide")

    rail_finish = model.material("rail_finish", rgba=(0.24, 0.27, 0.31, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.79, 0.42, 0.14, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.56, 0.59, 0.63, 1.0))
    spindle_finish = model.material("spindle_finish", rgba=(0.83, 0.85, 0.87, 1.0))

    rail = model.part("rail_base")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=rail_finish,
        name="base_plate",
    )
    rail.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + GUIDE_HEIGHT / 2.0)),
        material=rail_finish,
        name="rail_guide",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT - CARRIAGE_CHANNEL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_CHANNEL_HEIGHT + (CARRIAGE_HEIGHT - CARRIAGE_CHANNEL_HEIGHT) / 2.0)),
        material=carriage_finish,
        name="carriage_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_THICKNESS, CARRIAGE_CHANNEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_CHANNEL_WIDTH / 2.0 + CARRIAGE_CHEEK_THICKNESS / 2.0,
                CARRIAGE_CHANNEL_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="left_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_THICKNESS, CARRIAGE_CHANNEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CARRIAGE_CHANNEL_WIDTH / 2.0 + CARRIAGE_CHEEK_THICKNESS / 2.0),
                CARRIAGE_CHANNEL_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="right_cheek",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.35,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    spindle_housing = model.part("spindle_housing")
    spindle_housing.visual(
        mesh_from_cadquery(_build_spindle_housing_shape(), "spindle_housing_shell"),
        material=housing_finish,
        name="housing_shell",
    )

    model.articulation(
        "carriage_to_housing",
        ArticulationType.FIXED,
        parent=carriage,
        child=spindle_housing,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_AXIS_Z_ON_CARRIAGE)),
    )

    spindle = model.part("spindle")
    spindle_axis = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    spindle.visual(
        Cylinder(radius=0.026, length=HOUSING_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="front_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.040, length=0.006),
        origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="rear_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=(0.0, 0.0785, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="tool_nose",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.112, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="tool_stub",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=spindle_axis.rpy),
        material=spindle_finish,
        name="rear_drive_hub",
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=spindle_housing,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=250.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail_base")
    carriage = object_model.get_part("carriage")
    spindle_housing = object_model.get_part("spindle_housing")
    spindle = object_model.get_part("spindle")
    carriage_slide = object_model.get_articulation("carriage_slide")
    housing_mount = object_model.get_articulation("carriage_to_housing")
    spindle_spin = object_model.get_articulation("spindle_spin")

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
        "carriage_slide_is_prismatic",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC carriage joint, got {carriage_slide.articulation_type!r}",
    )
    ctx.check(
        "carriage_slide_axis_is_rail_x",
        tuple(round(value, 6) for value in carriage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected slide axis (1, 0, 0), got {carriage_slide.axis!r}",
    )
    ctx.check(
        "housing_mount_is_fixed",
        housing_mount.articulation_type == ArticulationType.FIXED,
        details=f"expected FIXED housing mount, got {housing_mount.articulation_type!r}",
    )
    ctx.check(
        "spindle_joint_is_rotary_about_tool_axis",
        spindle_spin.articulation_type in (ArticulationType.REVOLUTE, ArticulationType.CONTINUOUS)
        and tuple(round(value, 6) for value in spindle_spin.axis) == (0.0, 1.0, 0.0),
        details=f"expected spindle axis (0, 1, 0), got type={spindle_spin.articulation_type!r}, axis={spindle_spin.axis!r}",
    )

    rail_dims = _aabb_dims(ctx.part_world_aabb(rail))
    carriage_dims = _aabb_dims(ctx.part_world_aabb(carriage))
    if rail_dims is not None:
        ctx.check(
            "rail_has_machine_scale",
            rail_dims[0] > 0.65 and rail_dims[1] > 0.18 and rail_dims[2] > 0.08,
            details=f"unexpected rail dimensions {rail_dims!r}",
        )
        ctx.check(
            "rail_is_long_and_slender",
            rail_dims[0] > 3.0 * rail_dims[1],
            details=f"rail should read as a rail, got dimensions {rail_dims!r}",
        )
    if rail_dims is not None and carriage_dims is not None:
        ctx.check(
            "carriage_is_shorter_than_rail",
            carriage_dims[0] < 0.4 * rail_dims[0] and carriage_dims[1] < rail_dims[1],
            details=f"carriage dims {carriage_dims!r} should be smaller than rail dims {rail_dims!r}",
        )

    ctx.expect_contact(rail, carriage, contact_tol=1e-6, name="carriage_supported_by_rail")
    ctx.expect_overlap(carriage, rail, axes="xy", min_overlap=0.10, name="carriage_overlaps_rail_footprint")
    ctx.expect_contact(carriage, spindle_housing, contact_tol=0.001, name="housing_seated_on_carriage")
    ctx.expect_overlap(spindle_housing, carriage, axes="xy", min_overlap=0.10, name="housing_centered_over_carriage")
    ctx.expect_contact(spindle, spindle_housing, contact_tol=0.001, name="spindle_bearing_faces_touch_housing")
    ctx.expect_overlap(spindle, spindle_housing, axes="xz", min_overlap=0.05, name="spindle_runs_through_housing_center")

    housing_shell_aabb = ctx.part_element_world_aabb(spindle_housing, elem="housing_shell")
    tool_nose_aabb = ctx.part_element_world_aabb(spindle, elem="tool_nose")
    if housing_shell_aabb is not None and tool_nose_aabb is not None:
        ctx.check(
            "tool_nose_projects_forward_of_housing",
            tool_nose_aabb[1][1] > housing_shell_aabb[1][1] + 0.03,
            details=f"tool nose max y {tool_nose_aabb[1][1]:.4f} should extend past housing max y {housing_shell_aabb[1][1]:.4f}",
        )

    slide_limits = carriage_slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({carriage_slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_slide_lower_no_floating")
            ctx.expect_contact(rail, carriage, contact_tol=1e-6, name="carriage_lower_contact")
            ctx.expect_contact(carriage, spindle_housing, contact_tol=0.001, name="housing_lower_contact")
            ctx.expect_contact(spindle, spindle_housing, contact_tol=0.001, name="spindle_lower_contact")
        with ctx.pose({carriage_slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_slide_upper_no_floating")
            ctx.expect_contact(rail, carriage, contact_tol=1e-6, name="carriage_upper_contact")
            ctx.expect_contact(carriage, spindle_housing, contact_tol=0.001, name="housing_upper_contact")
            ctx.expect_contact(spindle, spindle_housing, contact_tol=0.001, name="spindle_upper_contact")

    for index, angle in enumerate((0.0, pi / 2.0, pi)):
        with ctx.pose({spindle_spin: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"spindle_spin_pose_{index}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"spindle_spin_pose_{index}_no_floating")
            ctx.expect_contact(spindle, spindle_housing, contact_tol=0.001, name=f"spindle_spin_pose_{index}_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
