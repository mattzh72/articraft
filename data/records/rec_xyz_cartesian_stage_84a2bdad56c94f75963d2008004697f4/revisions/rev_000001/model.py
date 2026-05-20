from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.02

X_RAIL_LENGTH = 0.34
X_RAIL_Y = 0.08
X_RAIL_SIZE = (X_RAIL_LENGTH, 0.016, 0.012)
X_RAIL_CENTER_Z = 0.042
X_STAGE_ORIGIN_Z = 0.055
X_TRAVEL = 0.08

Y_RAIL_LENGTH = 0.18
Y_RAIL_X = 0.048
Y_RAIL_SIZE = (0.018, Y_RAIL_LENGTH, 0.012)
Y_STAGE_ORIGIN_Z = 0.063
Y_TRAVEL = 0.035

Z_RAIL_HEIGHT = 0.22
Z_RAIL_Y = 0.025
Z_RAIL_SIZE = (0.012, 0.012, Z_RAIL_HEIGHT)
Z_STAGE_ORIGIN = (0.061, 0.0, 0.071)
Z_TRAVEL = 0.12


def _wp_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    *,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_mesh(part, shape: cq.Workplane, mesh_name: str, material: str, *, name: str) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=name)


def _base_body_shape() -> cq.Workplane:
    body = _wp_box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS), (0.0, 0.0, BASE_THICKNESS / 2.0))
    body = body.union(_wp_box((0.36, 0.040, 0.016), (0.0, X_RAIL_Y, 0.028)))
    body = body.union(_wp_box((0.36, 0.040, 0.016), (0.0, -X_RAIL_Y, 0.028)))
    body = body.union(_wp_box((0.30, 0.060, 0.018), (0.0, 0.0, 0.029)))
    body = body.union(_wp_box((0.030, 0.120, 0.032), (0.185, 0.0, 0.036)))
    body = body.union(_wp_box((0.030, 0.120, 0.032), (-0.185, 0.0, 0.036)))
    body = body.cut(_wp_box((0.24, 0.11, 0.010), (0.0, 0.0, 0.020)))
    return body


def _x_carriage_shape() -> cq.Workplane:
    body = _wp_box((0.18, 0.22, 0.018), (0.0, 0.0, 0.018))
    body = body.union(_wp_box((0.14, 0.042, 0.024), (0.0, X_RAIL_Y, 0.005)))
    body = body.union(_wp_box((0.14, 0.042, 0.024), (0.0, -X_RAIL_Y, 0.005)))
    body = body.union(_wp_box((0.030, 0.20, 0.018), (Y_RAIL_X, 0.0, 0.035)))
    body = body.union(_wp_box((0.030, 0.20, 0.018), (-Y_RAIL_X, 0.0, 0.035)))
    body = body.union(_wp_box((0.028, 0.16, 0.016), (0.0, 0.0, 0.035)))
    for x in (-0.055, 0.055):
        for y in (-0.105, 0.105):
            body = body.union(_wp_box((0.014, 0.010, 0.032), (x, y, 0.009)))
    body = body.cut(_wp_box((0.105, 0.105, 0.024), (0.0, 0.0, 0.000)))
    return body


def _y_slide_shape() -> cq.Workplane:
    body = _wp_box((0.12, 0.13, 0.018), (0.0, 0.0, 0.016))
    body = body.union(_wp_box((0.050, 0.065, 0.22), (-0.006, 0.0, 0.135)))
    body = body.union(_wp_box((0.016, 0.050, 0.22), (0.028, 0.0, 0.135)))
    body = body.union(_wp_box((0.028, 0.012, 0.11), (0.008, Z_RAIL_Y + 0.001, 0.080)))
    body = body.union(_wp_box((0.028, 0.012, 0.11), (0.008, -Z_RAIL_Y - 0.001, 0.080)))
    body = body.cut(_wp_box((0.020, 0.028, 0.140), (0.020, 0.0, 0.120)))
    return body


def _z_carriage_shape() -> cq.Workplane:
    body = _wp_box((0.044, 0.072, 0.096), (0.021, 0.0, 0.0))
    body = body.union(_wp_box((0.008, 0.082, 0.088), (0.048, 0.0, 0.0)))
    body = body.union(_wp_box((0.018, 0.050, 0.010), (0.028, 0.0, 0.053)))
    for z in (-0.022, 0.0, 0.022):
        body = body.union(_wp_box((0.006, 0.075, 0.006), (0.052, 0.0, z)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cartesian_manipulator")

    model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.59, 0.64, 1.0))
    model.material("cover_blue", rgba=(0.19, 0.33, 0.62, 1.0))
    model.material("polymer_black", rgba=(0.08, 0.09, 0.10, 1.0))

    base = model.part("base")
    _add_mesh(base, _base_body_shape(), "base_body", "graphite", name="body_shell")
    _add_box(base, X_RAIL_SIZE, (0.0, X_RAIL_Y, X_RAIL_CENTER_Z), "rail_steel", name="x_rail_left")
    _add_box(base, X_RAIL_SIZE, (0.0, -X_RAIL_Y, X_RAIL_CENTER_Z), "rail_steel", name="x_rail_right")
    _add_box(base, (0.006, 0.050, 0.014), (0.167, 0.0, 0.044), "polymer_black", name="x_stop_pos")
    _add_box(base, (0.006, 0.050, 0.014), (-0.167, 0.0, 0.044), "polymer_black", name="x_stop_neg")
    base.inertial = Inertial.from_geometry(
        Box((0.42, 0.28, 0.08)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    x_slide = model.part("x_slide")
    _add_mesh(x_slide, _x_carriage_shape(), "x_slide_body", "aluminum", name="body_shell")
    _add_box(x_slide, (0.120, 0.018, 0.014), (0.0, X_RAIL_Y, 0.0), "polymer_black", name="x_shoe_left")
    _add_box(x_slide, (0.120, 0.018, 0.014), (0.0, -X_RAIL_Y, 0.0), "polymer_black", name="x_shoe_right")
    _add_box(x_slide, Y_RAIL_SIZE, (Y_RAIL_X, 0.0, 0.050), "rail_steel", name="y_rail_left")
    _add_box(x_slide, Y_RAIL_SIZE, (-Y_RAIL_X, 0.0, 0.050), "rail_steel", name="y_rail_right")
    _add_box(x_slide, (0.030, 0.160, 0.008), (0.0, 0.0, 0.039), "cover_blue", name="y_cover_plate")
    _add_box(x_slide, (0.120, 0.008, 0.024), (0.0, 0.102, 0.047), "polymer_black", name="y_stop_pos")
    _add_box(x_slide, (0.120, 0.008, 0.024), (0.0, -0.102, 0.047), "polymer_black", name="y_stop_neg")
    _add_box(x_slide, (0.018, 0.040, 0.024), (0.065, 0.0, 0.062), "aluminum", name="x_bumper_pos_mount")
    _add_box(x_slide, (0.018, 0.040, 0.024), (-0.065, 0.0, 0.062), "aluminum", name="x_bumper_neg_mount")
    _add_box(x_slide, (0.010, 0.022, 0.028), (0.069, 0.0, 0.037), "aluminum", name="x_bumper_pos_stem")
    _add_box(x_slide, (0.010, 0.022, 0.028), (-0.069, 0.0, 0.037), "aluminum", name="x_bumper_neg_stem")
    _add_box(x_slide, (0.008, 0.050, 0.014), (0.073, 0.0, 0.018), "polymer_black", name="x_bumper_pos")
    _add_box(x_slide, (0.008, 0.050, 0.014), (-0.073, 0.0, 0.018), "polymer_black", name="x_bumper_neg")
    x_slide.inertial = Inertial.from_geometry(
        Box((0.18, 0.22, 0.08)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    y_slide = model.part("y_slide")
    _add_mesh(y_slide, _y_slide_shape(), "y_slide_body", "aluminum", name="body_shell")
    _add_box(y_slide, (0.018, 0.090, 0.014), (Y_RAIL_X, 0.0, 0.0), "polymer_black", name="y_shoe_left")
    _add_box(y_slide, (0.018, 0.090, 0.014), (-Y_RAIL_X, 0.0, 0.0), "polymer_black", name="y_shoe_right")
    _add_box(y_slide, Z_RAIL_SIZE, (0.048, Z_RAIL_Y, 0.135), "rail_steel", name="z_rail_left")
    _add_box(y_slide, Z_RAIL_SIZE, (0.048, -Z_RAIL_Y, 0.135), "rail_steel", name="z_rail_right")
    _add_box(y_slide, (0.008, 0.022, 0.200), (0.038, 0.0, 0.135), "cover_blue", name="z_cover_plate")
    _add_box(y_slide, (0.050, 0.050, 0.014), (0.020, 0.0, 0.015), "polymer_black", name="z_stop_bottom")
    _add_box(y_slide, (0.050, 0.050, 0.014), (0.020, 0.0, 0.259), "polymer_black", name="z_stop_top")
    _add_box(y_slide, (0.020, 0.030, 0.020), (0.036, 0.0, 0.020), "polymer_black", name="z_stop_bottom_mount")
    _add_box(y_slide, (0.020, 0.030, 0.020), (0.036, 0.0, 0.249), "polymer_black", name="z_stop_top_mount")
    _add_box(y_slide, (0.060, 0.008, 0.018), (0.0, 0.055, 0.016), "polymer_black", name="y_bumper_pos")
    _add_box(y_slide, (0.060, 0.008, 0.018), (0.0, -0.055, 0.016), "polymer_black", name="y_bumper_neg")
    y_slide.inertial = Inertial.from_geometry(
        Box((0.12, 0.13, 0.26)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    z_carriage = model.part("z_carriage")
    _add_mesh(z_carriage, _z_carriage_shape(), "z_carriage_body", "graphite", name="carriage_body")
    _add_box(z_carriage, (0.014, 0.018, 0.090), (0.0, Z_RAIL_Y, 0.0), "polymer_black", name="z_shoe_left")
    _add_box(z_carriage, (0.014, 0.018, 0.090), (0.0, -Z_RAIL_Y, 0.0), "polymer_black", name="z_shoe_right")
    _add_box(z_carriage, (0.055, 0.090, 0.006), (0.028, 0.0, 0.061), "aluminum", name="tool_plate")
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.07, 0.09, 0.12)),
        mass=1.0,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-X_TRAVEL, upper=X_TRAVEL, effort=300.0, velocity=0.35),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_TRAVEL, upper=Y_TRAVEL, effort=220.0, velocity=0.28),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=z_carriage,
        origin=Origin(xyz=Z_STAGE_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=180.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_slide = object_model.get_part("y_slide")
    z_carriage = object_model.get_part("z_carriage")

    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("y_to_z")

    x_rail_left = base.get_visual("x_rail_left")
    x_rail_right = base.get_visual("x_rail_right")
    x_stop_pos = base.get_visual("x_stop_pos")
    x_stop_neg = base.get_visual("x_stop_neg")

    x_shoe_left = x_slide.get_visual("x_shoe_left")
    x_shoe_right = x_slide.get_visual("x_shoe_right")
    x_bumper_pos = x_slide.get_visual("x_bumper_pos")
    x_bumper_neg = x_slide.get_visual("x_bumper_neg")
    y_rail_left = x_slide.get_visual("y_rail_left")
    y_rail_right = x_slide.get_visual("y_rail_right")
    y_stop_pos = x_slide.get_visual("y_stop_pos")
    y_stop_neg = x_slide.get_visual("y_stop_neg")

    y_shoe_left = y_slide.get_visual("y_shoe_left")
    y_shoe_right = y_slide.get_visual("y_shoe_right")
    y_bumper_pos = y_slide.get_visual("y_bumper_pos")
    y_bumper_neg = y_slide.get_visual("y_bumper_neg")
    z_rail_left = y_slide.get_visual("z_rail_left")
    z_rail_right = y_slide.get_visual("z_rail_right")
    z_stop_top = y_slide.get_visual("z_stop_top")
    z_stop_bottom = y_slide.get_visual("z_stop_bottom")

    z_shoe_left = z_carriage.get_visual("z_shoe_left")
    z_shoe_right = z_carriage.get_visual("z_shoe_right")
    carriage_body = z_carriage.get_visual("carriage_body")
    tool_plate = z_carriage.get_visual("tool_plate")

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

    ctx.check("x_axis_is_prismatic_x", tuple(x_joint.axis) == (1.0, 0.0, 0.0), f"axis={x_joint.axis}")
    ctx.check("y_axis_is_prismatic_y", tuple(y_joint.axis) == (0.0, 1.0, 0.0), f"axis={y_joint.axis}")
    ctx.check("z_axis_is_prismatic_z", tuple(z_joint.axis) == (0.0, 0.0, 1.0), f"axis={z_joint.axis}")
    ctx.check(
        "z_tool_plate_present",
        tool_plate.name == "tool_plate",
        "Uppermost carriage should carry the visible tool plate.",
    )

    ctx.expect_contact(x_slide, base, elem_a=x_shoe_left, elem_b=x_rail_left, name="x_left_shoe_contacts_left_rail")
    ctx.expect_contact(x_slide, base, elem_a=x_shoe_right, elem_b=x_rail_right, name="x_right_shoe_contacts_right_rail")
    ctx.expect_contact(y_slide, x_slide, elem_a=y_shoe_left, elem_b=y_rail_left, name="y_left_shoe_contacts_left_rail")
    ctx.expect_contact(y_slide, x_slide, elem_a=y_shoe_right, elem_b=y_rail_right, name="y_right_shoe_contacts_right_rail")
    ctx.expect_contact(z_carriage, y_slide, elem_a=z_shoe_left, elem_b=z_rail_left, name="z_left_shoe_contacts_left_rail")
    ctx.expect_contact(z_carriage, y_slide, elem_a=z_shoe_right, elem_b=z_rail_right, name="z_right_shoe_contacts_right_rail")

    with ctx.pose({x_joint: x_joint.motion_limits.upper, y_joint: y_joint.motion_limits.upper, z_joint: z_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_upper_stack_pose")
        ctx.expect_gap(
            base,
            x_slide,
            axis="x",
            positive_elem=x_stop_pos,
            negative_elem=x_bumper_pos,
            min_gap=0.006,
            max_gap=0.009,
            name="x_positive_endstop_clearance",
        )
        ctx.expect_gap(
            x_slide,
            y_slide,
            axis="y",
            positive_elem=y_stop_pos,
            negative_elem=y_bumper_pos,
            min_gap=0.0025,
            max_gap=0.0065,
            name="y_positive_endstop_clearance",
        )
        ctx.expect_gap(
            y_slide,
            z_carriage,
            axis="z",
            positive_elem=z_stop_top,
            negative_elem=carriage_body,
            min_gap=0.003,
            max_gap=0.007,
            name="z_top_endstop_clearance",
        )

    with ctx.pose({x_joint: x_joint.motion_limits.lower, y_joint: y_joint.motion_limits.lower, z_joint: z_joint.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_lower_stack_pose")
        ctx.expect_gap(
            x_slide,
            base,
            axis="x",
            positive_elem=x_bumper_neg,
            negative_elem=x_stop_neg,
            min_gap=0.006,
            max_gap=0.009,
            name="x_negative_endstop_clearance",
        )
        ctx.expect_gap(
            y_slide,
            x_slide,
            axis="y",
            positive_elem=y_bumper_neg,
            negative_elem=y_stop_neg,
            min_gap=0.0025,
            max_gap=0.0065,
            name="y_negative_endstop_clearance",
        )
        ctx.expect_gap(
            z_carriage,
            y_slide,
            axis="z",
            positive_elem=carriage_body,
            negative_elem=z_stop_bottom,
            min_gap=0.0,
            max_gap=0.004,
            name="z_bottom_endstop_clearance",
        )

    with ctx.pose({x_joint: x_joint.motion_limits.upper, y_joint: y_joint.motion_limits.lower, z_joint: z_joint.motion_limits.upper * 0.55}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_mixed_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
