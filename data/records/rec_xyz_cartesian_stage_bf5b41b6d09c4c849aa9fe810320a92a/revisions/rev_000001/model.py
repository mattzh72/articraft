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


BASE_L = 0.360
BASE_W = 0.200
BASE_PLATE_T = 0.018
BASE_SIDE_SKIRT_H = 0.012
BASE_SIDE_SKIRT_W = 0.016
X_RAIL_L = 0.300
X_RAIL_W = 0.022
X_RAIL_H = 0.022
X_RAIL_Y = 0.058
X_STAGE_HOME_Z = BASE_PLATE_T + X_RAIL_H

X_CARRIAGE_L = 0.110
X_CARRIAGE_W = 0.150
X_STAGE_SHOE_L = 0.096
X_STAGE_SHOE_W = 0.028
X_STAGE_SHOE_H = 0.012
X_STAGE_BRIDGE_H = 0.022
X_STAGE_DECK_T = 0.018
X_STAGE_DECK_Z = 0.028
X_STAGE_DECK_L = 0.124
X_STAGE_DECK_W = 0.220
Y_RAIL_L = 0.190
Y_RAIL_W = 0.016
Y_RAIL_H = 0.016
Y_RAIL_X = 0.034
Y_STAGE_HOME_Z = X_STAGE_DECK_Z + X_STAGE_DECK_T + Y_RAIL_H

Y_STAGE_SHOE_X = 0.034
Y_STAGE_SHOE_LX = 0.024
Y_STAGE_SHOE_LY = 0.062
Y_STAGE_SHOE_H = 0.010
Y_STAGE_SADDLE_LX = 0.098
Y_STAGE_SADDLE_LY = 0.072
Y_STAGE_SADDLE_H = 0.024
Y_STAGE_COLUMN_X = 0.036
Y_STAGE_COLUMN_Y = 0.036
Y_STAGE_COLUMN_H = 0.220
Z_GUIDE_T = 0.010
Z_GUIDE_W = 0.020
Z_GUIDE_H = 0.170
Z_GUIDE_BOTTOM_Z = 0.040
Y_TO_Z_X = (Y_STAGE_COLUMN_X / 2.0) + Z_GUIDE_T

Z_STAGE_GUIDE_H = 0.110
Z_STAGE_GUIDE_T = 0.012
Z_STAGE_GUIDE_W = 0.052
Z_STAGE_FACE_SIZE = 0.052
Z_STAGE_FACE_T = 0.010

X_TRAVEL = 0.090
Y_TRAVEL = 0.050
Z_TRAVEL = 0.120


def _box_on_floor(size: tuple[float, float, float], *, center=(0.0, 0.0), bottom=0.0) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, bottom + (sz / 2.0)))


def _base_body_shape() -> cq.Workplane:
    body = _box_on_floor((BASE_L, BASE_W, BASE_PLATE_T))
    body = body.union(
        _box_on_floor(
            (BASE_L - 0.040, BASE_SIDE_SKIRT_W, BASE_SIDE_SKIRT_H),
            center=(0.0, (BASE_W / 2.0) - (BASE_SIDE_SKIRT_W / 2.0)),
            bottom=0.0,
        )
    )
    body = body.union(
        _box_on_floor(
            (BASE_L - 0.040, BASE_SIDE_SKIRT_W, BASE_SIDE_SKIRT_H),
            center=(0.0, -(BASE_W / 2.0) + (BASE_SIDE_SKIRT_W / 2.0)),
            bottom=0.0,
        )
    )
    body = body.union(_box_on_floor((0.300, 0.040, 0.014), bottom=BASE_PLATE_T))
    body = body.union(_box_on_floor((0.028, 0.110, 0.030), center=(-0.166, 0.0), bottom=BASE_PLATE_T))
    body = body.union(_box_on_floor((0.028, 0.110, 0.030), center=(0.166, 0.0), bottom=BASE_PLATE_T))
    body = body.union(_box_on_floor((0.020, 0.062, 0.018), center=(-0.154, 0.0), bottom=BASE_PLATE_T + 0.030))
    return body


def _x_stage_shape() -> cq.Workplane:
    shape = _box_on_floor(
        (X_STAGE_SHOE_L, X_STAGE_SHOE_W, X_STAGE_SHOE_H),
        center=(0.0, X_RAIL_Y),
        bottom=0.0,
    )
    shape = shape.union(
        _box_on_floor(
            (X_STAGE_SHOE_L, X_STAGE_SHOE_W, X_STAGE_SHOE_H),
            center=(0.0, -X_RAIL_Y),
            bottom=0.0,
        )
    )
    shape = shape.union(_box_on_floor((X_CARRIAGE_L, 0.036, 0.028), center=(0.0, X_RAIL_Y), bottom=0.010))
    shape = shape.union(_box_on_floor((X_CARRIAGE_L, 0.036, 0.028), center=(0.0, -X_RAIL_Y), bottom=0.010))
    shape = shape.union(_box_on_floor((0.074, 0.086, 0.014), bottom=0.018))
    shape = shape.union(_box_on_floor((X_STAGE_DECK_L, X_STAGE_DECK_W, X_STAGE_DECK_T), bottom=X_STAGE_DECK_Z))
    shape = shape.union(_box_on_floor((0.074, 0.080, 0.012), bottom=X_STAGE_DECK_Z + X_STAGE_DECK_T))
    shape = shape.union(
        _box_on_floor((0.070, 0.012, 0.014), center=(0.0, 0.086), bottom=X_STAGE_DECK_Z + X_STAGE_DECK_T)
    )
    shape = shape.union(
        _box_on_floor((0.070, 0.012, 0.014), center=(0.0, -0.086), bottom=X_STAGE_DECK_Z + X_STAGE_DECK_T)
    )
    return shape


def _y_stage_shape() -> cq.Workplane:
    shape = _box_on_floor(
        (Y_STAGE_SHOE_LX, Y_STAGE_SHOE_LY, Y_STAGE_SHOE_H),
        center=(Y_STAGE_SHOE_X, 0.0),
        bottom=0.0,
    )
    shape = shape.union(
        _box_on_floor(
            (Y_STAGE_SHOE_LX, Y_STAGE_SHOE_LY, Y_STAGE_SHOE_H),
            center=(-Y_STAGE_SHOE_X, 0.0),
            bottom=0.0,
        )
    )
    shape = shape.union(_box_on_floor((Y_STAGE_SADDLE_LX, Y_STAGE_SADDLE_LY, Y_STAGE_SADDLE_H), bottom=0.008))
    shape = shape.union(_box_on_floor((Y_STAGE_COLUMN_X, Y_STAGE_COLUMN_Y, Y_STAGE_COLUMN_H), bottom=0.028))
    shape = shape.union(_box_on_floor((0.022, 0.050, 0.120), center=(-0.020, 0.0), bottom=0.038))
    shape = shape.union(_box_on_floor((0.048, 0.048, 0.012), bottom=0.236))
    return shape


def _z_stage_body_shape() -> cq.Workplane:
    shape = _box_on_floor((Z_STAGE_GUIDE_T, Z_STAGE_GUIDE_W, Z_STAGE_GUIDE_H), center=(0.006, 0.0), bottom=0.0)
    shape = shape.union(_box_on_floor((0.020, 0.040, 0.088), center=(0.022, 0.0), bottom=0.010))
    shape = shape.union(_box_on_floor((0.020, 0.030, 0.040), center=(0.030, 0.0), bottom=0.080))
    shape = shape.union(_box_on_floor((0.028, 0.050, 0.018), center=(0.018, 0.0), bottom=0.098))
    return shape


def _output_face_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(Z_STAGE_FACE_T, Z_STAGE_FACE_SIZE, Z_STAGE_FACE_SIZE).translate((0.045, 0.0, 0.100))
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.014, -0.014), (-0.014, 0.014), (0.014, -0.014), (0.014, 0.014)])
        .hole(0.004, depth=0.012)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_xyz_module")

    model.material("base_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("carriage_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("stage_gray", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("face_silver", rgba=(0.83, 0.85, 0.88, 1.0))

    base = model.part("base_axis")
    base.visual(mesh_from_cadquery(_base_body_shape(), "base_axis_body"), material="base_black", name="body")
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(xyz=(0.0, X_RAIL_Y, BASE_PLATE_T + (X_RAIL_H / 2.0))),
        material="rail_steel",
        name="x_rail_left",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(xyz=(0.0, -X_RAIL_Y, BASE_PLATE_T + (X_RAIL_H / 2.0))),
        material="rail_steel",
        name="x_rail_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_PLATE_T + 0.048)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_PLATE_T + 0.048) / 2.0)),
    )

    x_stage = model.part("x_stage")
    x_stage.visual(mesh_from_cadquery(_x_stage_shape(), "x_stage_body"), material="carriage_gray", name="body")
    x_stage.visual(
        Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)),
        origin=Origin(xyz=(Y_RAIL_X, 0.0, X_STAGE_DECK_Z + X_STAGE_DECK_T + (Y_RAIL_H / 2.0))),
        material="rail_steel",
        name="y_rail_front",
    )
    x_stage.visual(
        Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)),
        origin=Origin(xyz=(-Y_RAIL_X, 0.0, X_STAGE_DECK_Z + X_STAGE_DECK_T + (Y_RAIL_H / 2.0))),
        material="rail_steel",
        name="y_rail_rear",
    )
    x_stage.inertial = Inertial.from_geometry(
        Box((X_STAGE_DECK_L, X_STAGE_DECK_W, Y_STAGE_HOME_Z)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_HOME_Z / 2.0)),
    )

    y_stage = model.part("y_stage")
    y_stage.visual(mesh_from_cadquery(_y_stage_shape(), "y_stage_body"), material="stage_gray", name="body")
    y_stage.visual(
        Box((Z_GUIDE_T, Z_GUIDE_W, Z_GUIDE_H)),
        origin=Origin(
            xyz=(
                (Y_STAGE_COLUMN_X / 2.0) + (Z_GUIDE_T / 2.0),
                0.0,
                Z_GUIDE_BOTTOM_Z + (Z_GUIDE_H / 2.0),
            )
        ),
        material="rail_steel",
        name="z_guide",
    )
    y_stage.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.248)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
    )

    z_stage = model.part("z_stage")
    z_stage.visual(mesh_from_cadquery(_z_stage_body_shape(), "z_stage_body"), material="carriage_gray", name="body")
    z_stage.visual(mesh_from_cadquery(_output_face_shape(), "z_stage_output_face"), material="face_silver", name="output_face")
    z_stage.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.130)),
        mass=1.1,
        origin=Origin(xyz=(0.028, 0.0, 0.065)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-X_TRAVEL, upper=X_TRAVEL, effort=300.0, velocity=0.40),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_HOME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_TRAVEL, upper=Y_TRAVEL, effort=220.0, velocity=0.35),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(Y_TO_Z_X, 0.0, Z_GUIDE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=180.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_axis")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    z_stage = object_model.get_part("z_stage")

    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("y_to_z")

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

    ctx.expect_contact(
        x_stage,
        base,
        elem_a="body",
        elem_b="x_rail_left",
        name="x_stage_left_shoe_contacts_left_rail",
    )
    ctx.expect_contact(
        x_stage,
        base,
        elem_a="body",
        elem_b="x_rail_right",
        name="x_stage_right_shoe_contacts_right_rail",
    )
    ctx.expect_overlap(x_stage, base, axes="xy", min_overlap=0.10, name="x_stage_footprint_over_base")
    ctx.expect_contact(
        y_stage,
        x_stage,
        elem_a="body",
        elem_b="y_rail_front",
        name="y_stage_front_pad_contacts_front_y_rail",
    )
    ctx.expect_contact(
        y_stage,
        x_stage,
        elem_a="body",
        elem_b="y_rail_rear",
        name="y_stage_rear_pad_contacts_rear_y_rail",
    )
    ctx.expect_overlap(y_stage, x_stage, axes="xy", min_overlap=0.05, name="y_stage_footprint_over_x")
    ctx.expect_contact(
        z_stage,
        y_stage,
        elem_a="body",
        elem_b="z_guide",
        name="z_stage_contacts_vertical_guide",
    )
    ctx.expect_overlap(z_stage, y_stage, axes="yz", min_overlap=0.02, name="z_stage_tracks_on_vertical_guide")

    axes_ok = x_joint.axis == (1.0, 0.0, 0.0) and y_joint.axis == (0.0, 1.0, 0.0) and z_joint.axis == (0.0, 0.0, 1.0)
    ctx.check("orthogonal_prismatic_axes", axes_ok, details=f"x={x_joint.axis}, y={y_joint.axis}, z={z_joint.axis}")

    x_rest = ctx.part_world_position(x_stage)
    with ctx.pose({x_joint: X_TRAVEL}):
        x_moved = ctx.part_world_position(x_stage)
    x_ok = (
        x_rest is not None
        and x_moved is not None
        and (x_moved[0] - x_rest[0]) > 0.08
        and abs(x_moved[1] - x_rest[1]) < 1e-6
        and abs(x_moved[2] - x_rest[2]) < 1e-6
    )
    ctx.check("x_stage_moves_only_along_x", x_ok, details=f"rest={x_rest}, moved={x_moved}")

    y_rest = ctx.part_world_position(y_stage)
    with ctx.pose({y_joint: Y_TRAVEL}):
        y_moved = ctx.part_world_position(y_stage)
    y_ok = (
        y_rest is not None
        and y_moved is not None
        and (y_moved[1] - y_rest[1]) > 0.045
        and abs(y_moved[0] - y_rest[0]) < 1e-6
        and abs(y_moved[2] - y_rest[2]) < 1e-6
    )
    ctx.check("y_stage_moves_only_along_y", y_ok, details=f"rest={y_rest}, moved={y_moved}")

    z_rest = ctx.part_world_position(z_stage)
    with ctx.pose({z_joint: Z_TRAVEL}):
        z_moved = ctx.part_world_position(z_stage)
    z_ok = (
        z_rest is not None
        and z_moved is not None
        and (z_moved[2] - z_rest[2]) > 0.10
        and abs(z_moved[0] - z_rest[0]) < 1e-6
        and abs(z_moved[1] - z_rest[1]) < 1e-6
    )
    ctx.check("z_stage_moves_only_along_z", z_ok, details=f"rest={z_rest}, moved={z_moved}")

    face_aabb = ctx.part_element_world_aabb(z_stage, elem="output_face")
    if face_aabb is None:
        ctx.fail("square_output_face_present", "output_face visual AABB was not available")
    else:
        face_size = tuple(upper - lower for lower, upper in zip(face_aabb[0], face_aabb[1]))
        square_ok = (
            0.008 <= face_size[0] <= 0.012
            and 0.048 <= face_size[1] <= 0.056
            and 0.048 <= face_size[2] <= 0.056
            and abs(face_size[1] - face_size[2]) <= 0.002
        )
        ctx.check("square_output_face_present", square_ok, details=f"output_face_size={face_size}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
