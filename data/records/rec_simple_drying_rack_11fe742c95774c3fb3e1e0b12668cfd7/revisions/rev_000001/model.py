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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder_x(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
):
    _add_cylinder(
        part,
        name,
        radius,
        length,
        xyz,
        material=material,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _add_cylinder_y(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
):
    _add_cylinder(
        part,
        name,
        radius,
        length,
        xyz,
        material=material,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )


def _add_cylinder_in_xz_plane(
    part,
    name: str,
    radius: float,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    material: str,
):
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    center = (
        0.5 * (start[0] + end[0]),
        start[1],
        0.5 * (start[2] + end[2]),
    )
    pitch = math.atan2(dx, dz)
    _add_cylinder(
        part,
        name,
        radius,
        length,
        center,
        material=material,
        rpy=(0.0, pitch, 0.0),
    )


def _add_base_frame(model: ArticulatedObject) -> None:
    base = model.part("base_frame")

    lower_y = 0.27
    upper_y = 0.18
    top_z = 0.955
    lower_z = 0.025
    foot_len = 1.12
    foot_width = 0.54
    upper_len = 1.07
    upper_end_width = 0.41
    post_height = top_z - lower_z
    end_x = 0.535
    top_end_x = 0.520

    _add_box(base, "left_foot_rail", (foot_len, 0.05, 0.05), (0.0, lower_y, lower_z), material="frame_dark")
    _add_box(base, "right_foot_rail", (foot_len, 0.05, 0.05), (0.0, -lower_y, lower_z), material="frame_dark")
    _add_box(base, "front_foot_crossbeam", (0.05, foot_width, 0.05), (end_x, 0.0, lower_z), material="frame_dark")
    _add_box(base, "rear_foot_crossbeam", (0.05, foot_width, 0.05), (-end_x, 0.0, lower_z), material="frame_dark")

    for x_sign, x_val in (("rear", -end_x), ("front", end_x)):
        for y_sign, y_val in (("right", -upper_y), ("left", upper_y)):
            _add_box(
                base,
                f"{x_sign}_{y_sign}_post",
                (0.05, 0.05, post_height),
                (x_val, y_val, lower_z + 0.5 * post_height),
                material="frame_dark",
            )
            _add_box(
                base,
                f"{x_sign}_{y_sign}_corner_gusset",
                (0.09, 0.04, 0.10),
                (x_val, y_val, top_z - 0.055),
                material="reinforcement_gray",
            )

    _add_box(base, "top_left_rail", (upper_len, 0.05, 0.05), (0.0, upper_y, top_z), material="frame_dark")
    _add_box(base, "top_right_rail", (upper_len, 0.05, 0.05), (0.0, -upper_y, top_z), material="frame_dark")
    _add_box(base, "front_top_crossbeam", (0.05, upper_end_width, 0.05), (top_end_x, 0.0, top_z), material="frame_dark")
    _add_box(base, "rear_top_crossbeam", (0.05, upper_end_width, 0.05), (-top_end_x, 0.0, top_z), material="frame_dark")

    for idx, y in enumerate((-0.12, -0.06, 0.0, 0.06, 0.12), start=1):
        _add_cylinder_x(
            base,
            f"center_hanging_rail_{idx}",
            radius=0.007,
            length=1.02,
            xyz=(0.0, y, top_z + 0.020),
            material="galvanized_steel",
        )

    brace_low_points = {
        "rear": -0.535,
        "front": 0.535,
    }
    brace_high_points = {
        "rear": -0.505,
        "front": 0.505,
    }
    for x_name in ("rear", "front"):
        for y_name, y_val in (("right", -upper_y), ("left", upper_y)):
            _add_cylinder_in_xz_plane(
                base,
                f"{x_name}_{y_name}_side_brace",
                radius=0.010,
                start=(brace_low_points[x_name], y_val, 0.05),
                end=(brace_high_points[x_name], y_val, 0.93),
                material="reinforcement_gray",
            )

    for x_name, x_val in (("rear", -end_x), ("front", end_x)):
        for y_name, y_val in (("right", -lower_y), ("left", lower_y)):
            _add_box(
                base,
                f"{x_name}_{y_name}_foot_pad",
                (0.09, 0.06, 0.012),
                (x_val, y_val, 0.006),
                material="rubber_black",
            )


def _add_hinge_link(model: ArticulatedObject, side_name: str, side_sign: float) -> None:
    hinge_link = model.part(f"{side_name}_hinge_link")

    _add_box(
        hinge_link,
        "back_plate",
        (0.86, 0.012, 0.12),
        (0.0, -side_sign * 0.035, -0.005),
        material="reinforcement_gray",
    )
    _add_box(
        hinge_link,
        "hinge_spine",
        (0.86, 0.028, 0.030),
        (0.0, -side_sign * 0.015, -0.020),
        material="reinforcement_gray",
    )
    for idx, x in enumerate((-0.24, 0.0, 0.24), start=1):
        _add_box(
            hinge_link,
            f"barrel_rib_{idx}",
            (0.06, 0.018, 0.032),
            (x, -side_sign * 0.006, -0.010),
            material="reinforcement_gray",
        )

    _add_cylinder_x(
        hinge_link,
        "hinge_pin_housing",
        radius=0.012,
        length=0.84,
        xyz=(0.0, 0.0, 0.0),
        material="galvanized_steel",
    )
    _add_box(
        hinge_link,
        "deploy_stop_arm",
        (0.14, 0.032, 0.020),
        (0.0, 0.0, -0.040),
        material="reinforcement_gray",
    )
    _add_box(
        hinge_link,
        "deploy_stop",
        (0.14, 0.016, 0.020),
        (0.0, side_sign * 0.024, -0.040),
        material="safety_yellow",
    )
    _add_box(
        hinge_link,
        "guard_brace",
        (0.86, 0.014, 0.070),
        (0.0, -side_sign * 0.004, 0.023),
        material="reinforcement_gray",
    )
    _add_box(
        hinge_link,
        "guard_side_plate",
        (0.86, 0.010, 0.050),
        (0.0, side_sign * 0.018, 0.080),
        material="safety_yellow",
    )
    _add_box(
        hinge_link,
        "guard_roof",
        (0.86, 0.050, 0.012),
        (0.0, 0.0, 0.061),
        material="safety_yellow",
    )
    _add_box(
        hinge_link,
        "lockout_bracket",
        (0.12, 0.024, 0.030),
        (0.0, -side_sign * 0.020, -0.030),
        material="reinforcement_gray",
    )
    _add_box(
        hinge_link,
        "lockout_plate",
        (0.18, 0.012, 0.11),
        (0.0, -side_sign * 0.008, -0.010),
        material="safety_yellow",
    )
    _add_cylinder_y(
        hinge_link,
        "lockout_knob",
        radius=0.014,
        length=0.014,
        xyz=(0.0, -side_sign * 0.001, -0.010),
        material="lockout_red",
    )

    for idx, x in enumerate((-0.30, -0.10, 0.10, 0.30), start=1):
        _add_cylinder_y(
            hinge_link,
            f"mount_bolt_head_{idx}",
            radius=0.008,
            length=0.014,
            xyz=(x, -side_sign * 0.022, -0.020),
            material="fastener_zinc",
        )

    base = model.get_part("base_frame")
    model.articulation(
        f"base_to_{side_name}_hinge_link",
        ArticulationType.FIXED,
        parent=base,
        child=hinge_link,
        origin=Origin(xyz=(0.0, side_sign * 0.246, 0.955)),
    )


def _add_wing(model: ArticulatedObject, side_name: str, side_sign: float) -> None:
    wing = model.part(f"{side_name}_wing")

    _add_box(
        wing,
        "hinge_leaf",
        (0.84, 0.012, 0.024),
        (0.0, side_sign * 0.018, 0.0),
        material="galvanized_steel",
    )
    _add_box(
        wing,
        "hinge_bridge",
        (0.84, 0.032, 0.018),
        (0.0, side_sign * 0.036, -0.012),
        material="reinforcement_gray",
    )
    _add_box(
        wing,
        "inner_reinforcement_rail",
        (1.00, 0.028, 0.030),
        (0.0, side_sign * 0.074, 0.0),
        material="frame_dark",
    )
    _add_box(
        wing,
        "outer_rail",
        (1.00, 0.028, 0.030),
        (0.0, side_sign * 0.336, 0.0),
        material="frame_dark",
    )
    _add_box(
        wing,
        "front_frame_rail",
        (0.028, 0.292, 0.030),
        (0.486, side_sign * 0.198, 0.0),
        material="frame_dark",
    )
    _add_box(
        wing,
        "rear_frame_rail",
        (0.028, 0.292, 0.030),
        (-0.486, side_sign * 0.198, 0.0),
        material="frame_dark",
    )
    _add_box(
        wing,
        "mid_tie_rail",
        (1.00, 0.018, 0.020),
        (0.0, side_sign * 0.205, -0.020),
        material="reinforcement_gray",
    )
    _add_box(
        wing,
        "stop_pad",
        (0.14, 0.020, 0.020),
        (0.0, side_sign * 0.042, -0.040),
        material="safety_yellow",
    )
    _add_box(
        wing,
        "stop_pad_arm",
        (0.06, 0.020, 0.030),
        (0.0, side_sign * 0.042, -0.015),
        material="reinforcement_gray",
    )
    _add_box(
        wing,
        "lockout_lug",
        (0.10, 0.012, 0.11),
        (0.0, side_sign * 0.056, -0.010),
        material="safety_yellow",
    )

    for idx, x in enumerate((-0.18, 0.18), start=1):
        _add_box(
            wing,
            f"hinge_ear_web_{idx}",
            (0.16, 0.024, 0.024),
            (x, side_sign * 0.032, 0.0),
            material="reinforcement_gray",
        )
        _add_cylinder_y(
            wing,
            f"ear_web_fastener_{idx}",
            radius=0.007,
            length=0.018,
            xyz=(x, side_sign * 0.068, -0.008),
            material="fastener_zinc",
        )

    for idx, y in enumerate((0.13, 0.20, 0.27), start=1):
        _add_cylinder_x(
            wing,
            f"hanging_rail_{idx}",
            radius=0.007,
            length=0.972,
            xyz=(0.0, side_sign * y, 0.018),
            material="galvanized_steel",
        )

    hinge_link = model.get_part(f"{side_name}_hinge_link")
    model.articulation(
        f"{side_name}_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_link,
        child=wing,
        origin=Origin(),
        axis=(-side_sign, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=0.0,
            upper=1.35,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_drying_rack")

    model.material("frame_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("reinforcement_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("galvanized_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("safety_yellow", rgba=(0.93, 0.77, 0.10, 1.0))
    model.material("lockout_red", rgba=(0.78, 0.15, 0.12, 1.0))
    model.material("fastener_zinc", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))

    _add_base_frame(model)
    _add_hinge_link(model, "left", 1.0)
    _add_hinge_link(model, "right", -1.0)
    _add_wing(model, "left", 1.0)
    _add_wing(model, "right", -1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    left_hinge_link = object_model.get_part("left_hinge_link")
    right_hinge_link = object_model.get_part("right_hinge_link")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

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

    ctx.expect_contact(left_hinge_link, base, elem_a="back_plate", elem_b="top_left_rail")
    ctx.expect_contact(right_hinge_link, base, elem_a="back_plate", elem_b="top_right_rail")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(left_wing, left_hinge_link, elem_a="stop_pad", elem_b="deploy_stop")
        ctx.expect_contact(right_wing, right_hinge_link, elem_a="stop_pad", elem_b="deploy_stop")
        ctx.expect_overlap(left_wing, left_hinge_link, axes="x", min_overlap=0.80)
        ctx.expect_overlap(right_wing, right_hinge_link, axes="x", min_overlap=0.80)

    with ctx.pose({left_hinge: 0.0}):
        left_open_aabb = ctx.part_world_aabb(left_wing)
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        left_stowed_aabb = ctx.part_world_aabb(left_wing)
    ctx.check(
        "left_wing_folds_down_to_stow",
        left_open_aabb is not None
        and left_stowed_aabb is not None
        and (left_open_aabb[1][1] - left_stowed_aabb[1][1]) > 0.12
        and (left_open_aabb[0][2] - left_stowed_aabb[0][2]) > 0.20
        and left_stowed_aabb[0][2] > 0.45,
        details=(
            f"open={left_open_aabb}, stowed={left_stowed_aabb}; "
            "expected the left wing to swing down and tuck inward while staying clear of the floor."
        ),
    )

    with ctx.pose({right_hinge: 0.0}):
        right_open_aabb = ctx.part_world_aabb(right_wing)
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        right_stowed_aabb = ctx.part_world_aabb(right_wing)
    ctx.check(
        "right_wing_folds_down_to_stow",
        right_open_aabb is not None
        and right_stowed_aabb is not None
        and (right_stowed_aabb[0][1] - right_open_aabb[0][1]) > 0.12
        and (right_open_aabb[0][2] - right_stowed_aabb[0][2]) > 0.20
        and right_stowed_aabb[0][2] > 0.45,
        details=(
            f"open={right_open_aabb}, stowed={right_stowed_aabb}; "
            "expected the right wing to swing down and tuck inward while staying clear of the floor."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
