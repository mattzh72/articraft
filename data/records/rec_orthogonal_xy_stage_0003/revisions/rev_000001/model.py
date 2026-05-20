from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

X_BASE_L = 0.42
X_BASE_W = 0.24
X_BASE_T = 0.016
X_GIRDER_L = 0.36
X_GIRDER_W = 0.034
X_GIRDER_H = 0.026
X_RAIL_L = 0.32
X_RAIL_W = 0.018
X_RAIL_H = 0.012
X_RAIL_Y = 0.08
X_TABLE_L = 0.20
X_TABLE_W = 0.19
X_TABLE_H = 0.048
X_TRAVEL = 0.16

Y_BASE_L = 0.26
Y_BASE_W = 0.19
Y_BASE_T = 0.014
Y_PEDESTAL_W = 0.028
Y_PEDESTAL_H = 0.020
Y_RAIL_L = 0.22
Y_RAIL_W = 0.018
Y_RAIL_H = 0.012
Y_RAIL_X = 0.055
Y_TABLE_L = 0.16
Y_TABLE_W = 0.18
Y_TABLE_H = 0.042
Y_TRAVEL = 0.14

COVER_T = 0.004


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((x, y, z + height / 2.0))


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    bbox: tuple[float, float, float],
    mass: float,
    material,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, f"{name}.obj", assets=ASSETS, tolerance=0.0008, angular_tolerance=0.08),
        material=material,
        name="body",
    )
    part.inertial = Inertial.from_geometry(Box(bbox), mass=mass, origin=Origin(xyz=(0.0, 0.0, bbox[2] / 2.0)))
    return part


def _linear_rail(length: float, width: float, height: float, hole_count: int, *, axis: str) -> cq.Workplane:
    rail = _box(length, width, height, z=0.0)
    pitch = length / (hole_count + 1)
    x_positions = [(-length / 2.0) + pitch * (i + 1) for i in range(hole_count)]
    pockets = (
        cq.Workplane("XY")
        .pushPoints([(x, 0.0) for x in x_positions])
        .circle(width * 0.22)
        .extrude(height * 0.45)
        .translate((0.0, 0.0, height * 0.55))
    )
    heads = (
        cq.Workplane("XY")
        .pushPoints([(x, 0.0) for x in x_positions])
        .circle(width * 0.17)
        .extrude(height * 0.20)
        .translate((0.0, 0.0, height))
    )
    rail = rail.cut(pockets).union(heads)
    if axis == "y":
        rail = rail.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)
    return rail


def _access_cover(length: float, height: float) -> cq.Workplane:
    cover = _box(length, COVER_T, height, z=0.0)
    cover = cover.cut(_box(length * 0.58, COVER_T * 2.0, height * 0.42, z=height * 0.30))
    cover = cover.cut(_box(length * 0.82, COVER_T * 2.0, height * 0.12, z=height * 0.76))
    ribs = _box(length * 0.78, COVER_T * 0.70, 0.004, z=height * 0.08)
    return cover.union(ribs)


def _cable_bracket(foot_x: float, foot_y: float, height: float, reach: float) -> cq.Workplane:
    bracket = _box(foot_x, foot_y, 0.006, z=0.0)
    bracket = bracket.union(_box(0.006, foot_y, height - 0.006, x=-(foot_x / 2.0) + 0.003, z=0.006))
    bracket = bracket.union(_box(reach, foot_y, 0.006, x=-(foot_x / 2.0) + (reach / 2.0), z=height - 0.006))
    bracket = bracket.union(_box(0.016, foot_y * 0.70, height * 0.45, x=-(foot_x / 2.0) + 0.012, z=0.006))
    bracket = bracket.cut(_box(reach * 0.42, foot_y * 0.45, 0.010, x=-(foot_x / 2.0) + (reach / 2.0), z=height - 0.010))
    return bracket


def _x_base_shape() -> cq.Workplane:
    shape = _box(X_BASE_L, X_BASE_W, X_BASE_T, z=0.0)
    shape = shape.union(_box(X_GIRDER_L, X_GIRDER_W, X_GIRDER_H, y=X_RAIL_Y, z=X_BASE_T))
    shape = shape.union(_box(X_GIRDER_L, X_GIRDER_W, X_GIRDER_H, y=-X_RAIL_Y, z=X_BASE_T))
    shape = shape.union(_box(0.026, 0.18, X_GIRDER_H, x=0.168, z=X_BASE_T))
    shape = shape.union(_box(0.026, 0.18, X_GIRDER_H, x=-0.168, z=X_BASE_T))
    shape = shape.union(_box(0.020, 0.125, 0.014, x=0.070, z=X_BASE_T))
    shape = shape.union(_box(0.020, 0.125, 0.014, x=-0.070, z=X_BASE_T))
    shape = shape.union(_box(0.012, 0.052, 0.018, x=0.182, z=X_BASE_T))
    shape = shape.union(_box(0.012, 0.052, 0.018, x=-0.182, z=X_BASE_T))
    shape = shape.cut(_box(0.22, 0.11, 0.010, z=0.006))
    for x_pos in (-0.09, 0.09):
        shape = shape.cut(_box(0.060, X_GIRDER_W * 1.25, 0.016, x=x_pos, y=X_RAIL_Y, z=X_BASE_T + 0.006))
        shape = shape.cut(_box(0.060, X_GIRDER_W * 1.25, 0.016, x=x_pos, y=-X_RAIL_Y, z=X_BASE_T + 0.006))
    return shape


def _x_table_shape() -> cq.Workplane:
    shape = _box(X_TABLE_L, X_TABLE_W, 0.014, z=0.034)
    shape = shape.cut(_box(0.110, 0.082, 0.020, z=0.031))
    shape = shape.union(_box(0.080, 0.026, 0.012, y=X_RAIL_Y, z=0.0))
    shape = shape.union(_box(0.080, 0.026, 0.012, y=-X_RAIL_Y, z=0.0))
    shape = shape.union(_box(0.080, 0.012, 0.022, y=X_RAIL_Y, z=0.012))
    shape = shape.union(_box(0.080, 0.012, 0.022, y=-X_RAIL_Y, z=0.012))
    shape = shape.union(_box(0.018, 0.150, 0.016, z=0.012))
    shape = shape.union(_box(0.036, 0.028, 0.008, x=0.048, z=0.048))
    shape = shape.union(_box(0.036, 0.028, 0.008, x=-0.048, z=0.048))
    shape = shape.union(_box(0.014, 0.050, 0.012, x=0.088, z=0.012))
    shape = shape.union(_box(0.014, 0.050, 0.012, x=-0.088, z=0.012))
    return shape


def _y_base_shape() -> cq.Workplane:
    shape = _box(Y_BASE_W, Y_BASE_L, Y_BASE_T, z=0.0)
    shape = shape.union(_box(Y_PEDESTAL_W, Y_RAIL_L, Y_PEDESTAL_H, x=Y_RAIL_X, z=Y_BASE_T))
    shape = shape.union(_box(Y_PEDESTAL_W, Y_RAIL_L, Y_PEDESTAL_H, x=-Y_RAIL_X, z=Y_BASE_T))
    shape = shape.union(_box(0.13, 0.028, 0.020, y=0.102, z=Y_BASE_T))
    shape = shape.union(_box(0.13, 0.028, 0.020, y=-0.102, z=Y_BASE_T))
    shape = shape.union(_box(0.100, 0.014, 0.014, y=0.060, z=Y_BASE_T))
    shape = shape.union(_box(0.100, 0.014, 0.014, y=-0.060, z=Y_BASE_T))
    shape = shape.union(_box(0.040, 0.012, 0.018, y=0.118, z=Y_BASE_T))
    shape = shape.union(_box(0.040, 0.012, 0.018, y=-0.118, z=Y_BASE_T))
    shape = shape.cut(_box(0.084, 0.140, 0.010, z=0.004))
    for y_pos in (-0.060, 0.060):
        shape = shape.cut(_box(0.018, 0.050, 0.012, x=Y_RAIL_X, y=y_pos, z=Y_BASE_T + 0.006))
        shape = shape.cut(_box(0.018, 0.050, 0.012, x=-Y_RAIL_X, y=y_pos, z=Y_BASE_T + 0.006))
    return shape


def _y_table_shape() -> cq.Workplane:
    shape = _box(Y_TABLE_W, Y_TABLE_L, 0.012, z=0.030)
    shape = shape.cut(_box(0.090, 0.060, 0.018, z=0.027))
    shape = shape.union(_box(0.026, 0.075, 0.012, x=Y_RAIL_X, z=0.0))
    shape = shape.union(_box(0.026, 0.075, 0.012, x=-Y_RAIL_X, z=0.0))
    shape = shape.union(_box(0.012, 0.075, 0.020, x=Y_RAIL_X, z=0.012))
    shape = shape.union(_box(0.012, 0.075, 0.020, x=-Y_RAIL_X, z=0.012))
    shape = shape.union(_box(0.140, 0.018, 0.014, z=0.012))
    shape = shape.union(_box(0.048, 0.028, 0.008, y=0.046, z=0.042))
    shape = shape.union(_box(0.048, 0.028, 0.008, y=-0.046, z=0.042))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xy_stage", assets=ASSETS)

    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.47, 0.50, 0.54, 1.0))
    blackened = model.material("blackened", rgba=(0.12, 0.13, 0.14, 1.0))

    x_base = _add_mesh_part(
        model,
        name="x_base",
        shape=_x_base_shape(),
        bbox=(X_BASE_L, X_BASE_W, X_BASE_T + X_GIRDER_H),
        mass=8.0,
        material=steel,
    )
    x_left_rail = _add_mesh_part(
        model,
        name="x_left_rail",
        shape=_linear_rail(X_RAIL_L, X_RAIL_W, X_RAIL_H, 6, axis="x"),
        bbox=(X_RAIL_L, X_RAIL_W, X_RAIL_H * 1.2),
        mass=0.9,
        material=blackened,
    )
    x_right_rail = _add_mesh_part(
        model,
        name="x_right_rail",
        shape=_linear_rail(X_RAIL_L, X_RAIL_W, X_RAIL_H, 6, axis="x"),
        bbox=(X_RAIL_L, X_RAIL_W, X_RAIL_H * 1.2),
        mass=0.9,
        material=blackened,
    )
    x_left_cover = _add_mesh_part(
        model,
        name="x_left_cover",
        shape=_access_cover(0.115, 0.048),
        bbox=(0.115, COVER_T, 0.048),
        mass=0.16,
        material=cover_gray,
    )
    x_right_cover = _add_mesh_part(
        model,
        name="x_right_cover",
        shape=_access_cover(0.115, 0.048),
        bbox=(0.115, COVER_T, 0.048),
        mass=0.16,
        material=cover_gray,
    )
    x_cable_bracket = _add_mesh_part(
        model,
        name="x_cable_bracket",
        shape=_cable_bracket(0.040, 0.022, 0.052, 0.024),
        bbox=(0.040, 0.022, 0.052),
        mass=0.14,
        material=dark_steel,
    )
    x_table = _add_mesh_part(
        model,
        name="x_table",
        shape=_x_table_shape(),
        bbox=(X_TABLE_L, X_TABLE_W, X_TABLE_H),
        mass=4.0,
        material=steel,
    )
    y_base = _add_mesh_part(
        model,
        name="y_base",
        shape=_y_base_shape(),
        bbox=(Y_BASE_W, Y_BASE_L, Y_BASE_T + Y_PEDESTAL_H),
        mass=3.6,
        material=steel,
    )
    y_left_rail = _add_mesh_part(
        model,
        name="y_left_rail",
        shape=_linear_rail(Y_RAIL_L, Y_RAIL_W, Y_RAIL_H, 5, axis="y"),
        bbox=(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H * 1.2),
        mass=0.7,
        material=blackened,
    )
    y_right_rail = _add_mesh_part(
        model,
        name="y_right_rail",
        shape=_linear_rail(Y_RAIL_L, Y_RAIL_W, Y_RAIL_H, 5, axis="y"),
        bbox=(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H * 1.2),
        mass=0.7,
        material=blackened,
    )
    y_front_cover = _add_mesh_part(
        model,
        name="y_front_cover",
        shape=_access_cover(0.118, 0.042).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0),
        bbox=(COVER_T, 0.118, 0.042),
        mass=0.14,
        material=cover_gray,
    )
    y_rear_cover = _add_mesh_part(
        model,
        name="y_rear_cover",
        shape=_access_cover(0.118, 0.042).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0),
        bbox=(COVER_T, 0.118, 0.042),
        mass=0.14,
        material=cover_gray,
    )
    y_table = _add_mesh_part(
        model,
        name="y_table",
        shape=_y_table_shape(),
        bbox=(Y_TABLE_W, Y_TABLE_L, Y_TABLE_H),
        mass=2.7,
        material=steel,
    )
    y_cable_bracket = _add_mesh_part(
        model,
        name="y_cable_bracket",
        shape=_cable_bracket(0.034, 0.022, 0.060, 0.028),
        bbox=(0.034, 0.022, 0.060),
        mass=0.12,
        material=dark_steel,
    )

    model.articulation(
        "x_left_rail_mount",
        ArticulationType.FIXED,
        parent=x_base,
        child=x_left_rail,
        origin=Origin(xyz=(0.0, X_RAIL_Y, X_BASE_T + X_GIRDER_H + 0.0002)),
    )
    model.articulation(
        "x_right_rail_mount",
        ArticulationType.FIXED,
        parent=x_base,
        child=x_right_rail,
        origin=Origin(xyz=(0.0, -X_RAIL_Y, X_BASE_T + X_GIRDER_H + 0.0002)),
    )
    model.articulation(
        "x_left_cover_mount",
        ArticulationType.FIXED,
        parent=x_base,
        child=x_left_cover,
        origin=Origin(xyz=(0.0, -(X_BASE_W / 2.0 + COVER_T / 2.0), X_BASE_T + 0.0002)),
    )
    model.articulation(
        "x_right_cover_mount",
        ArticulationType.FIXED,
        parent=x_base,
        child=x_right_cover,
        origin=Origin(xyz=(0.0, X_BASE_W / 2.0 + COVER_T / 2.0, X_BASE_T + 0.0002)),
    )
    model.articulation(
        "x_cable_bracket_mount",
        ArticulationType.FIXED,
        parent=x_base,
        child=x_cable_bracket,
        origin=Origin(xyz=(-0.172, -0.105, X_BASE_T + X_GIRDER_H + 0.0002)),
    )
    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=x_table,
        origin=Origin(xyz=(0.0, 0.0, X_BASE_T + X_GIRDER_H + X_RAIL_H + 0.0026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.30, lower=-(X_TRAVEL / 2.0), upper=X_TRAVEL / 2.0),
    )
    model.articulation(
        "y_base_mount",
        ArticulationType.FIXED,
        parent=x_table,
        child=y_base,
        origin=Origin(xyz=(0.0, 0.0, 0.0562)),
    )
    model.articulation(
        "y_left_rail_mount",
        ArticulationType.FIXED,
        parent=y_base,
        child=y_left_rail,
        origin=Origin(xyz=(-Y_RAIL_X, 0.0, Y_BASE_T + Y_PEDESTAL_H + 0.0002)),
    )
    model.articulation(
        "y_right_rail_mount",
        ArticulationType.FIXED,
        parent=y_base,
        child=y_right_rail,
        origin=Origin(xyz=(Y_RAIL_X, 0.0, Y_BASE_T + Y_PEDESTAL_H + 0.0002)),
    )
    model.articulation(
        "y_front_cover_mount",
        ArticulationType.FIXED,
        parent=y_base,
        child=y_front_cover,
        origin=Origin(xyz=(0.0, Y_BASE_L / 2.0 + 0.118 / 2.0 + 0.0002, Y_BASE_T + 0.0002)),
    )
    model.articulation(
        "y_rear_cover_mount",
        ArticulationType.FIXED,
        parent=y_base,
        child=y_rear_cover,
        origin=Origin(xyz=(0.0, -(Y_BASE_L / 2.0 + 0.118 / 2.0 + 0.0002), Y_BASE_T + 0.0002)),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=y_base,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, Y_BASE_T + Y_PEDESTAL_H + Y_RAIL_H + 0.0026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=-(Y_TRAVEL / 2.0), upper=Y_TRAVEL / 2.0),
    )
    model.articulation(
        "y_cable_bracket_mount",
        ArticulationType.FIXED,
        parent=y_table,
        child=y_cable_bracket,
        origin=Origin(xyz=(0.0, 0.068, 0.0502)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    x_base = object_model.get_part("x_base")
    x_left_rail = object_model.get_part("x_left_rail")
    x_right_rail = object_model.get_part("x_right_rail")
    x_left_cover = object_model.get_part("x_left_cover")
    x_right_cover = object_model.get_part("x_right_cover")
    x_cable_bracket = object_model.get_part("x_cable_bracket")
    x_table = object_model.get_part("x_table")
    y_base = object_model.get_part("y_base")
    y_left_rail = object_model.get_part("y_left_rail")
    y_right_rail = object_model.get_part("y_right_rail")
    y_front_cover = object_model.get_part("y_front_cover")
    y_rear_cover = object_model.get_part("y_rear_cover")
    y_table = object_model.get_part("y_table")
    y_cable_bracket = object_model.get_part("y_cable_bracket")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    for name, part in (
        ("x_base", x_base),
        ("x_left_rail", x_left_rail),
        ("x_right_rail", x_right_rail),
        ("x_left_cover", x_left_cover),
        ("x_right_cover", x_right_cover),
        ("x_cable_bracket", x_cable_bracket),
        ("x_table", x_table),
        ("y_base", y_base),
        ("y_left_rail", y_left_rail),
        ("y_right_rail", y_right_rail),
        ("y_front_cover", y_front_cover),
        ("y_rear_cover", y_rear_cover),
        ("y_table", y_table),
        ("y_cable_bracket", y_cable_bracket),
    ):
        ctx.check(f"{name}_present", part is not None, f"missing required part {name}")

    ctx.check("x_slide_present", x_slide is not None, "missing x prismatic joint")
    ctx.check("y_slide_present", y_slide is not None, "missing y prismatic joint")

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
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    ctx.expect_contact(x_left_rail, x_base, name="x_left_rail_mounted")
    ctx.expect_contact(x_right_rail, x_base, name="x_right_rail_mounted")
    ctx.expect_gap(x_left_rail, x_base, axis="z", max_gap=0.001, max_penetration=0.0, name="x_left_rail_seated")
    ctx.expect_gap(x_right_rail, x_base, axis="z", max_gap=0.001, max_penetration=0.0, name="x_right_rail_seated")
    ctx.expect_contact(x_left_cover, x_base, name="x_left_cover_attached")
    ctx.expect_contact(x_right_cover, x_base, name="x_right_cover_attached")
    ctx.expect_gap(x_base, x_left_cover, axis="y", max_gap=0.001, max_penetration=0.0, name="x_left_cover_flush")
    ctx.expect_gap(x_right_cover, x_base, axis="y", max_gap=0.001, max_penetration=0.0, name="x_right_cover_flush")
    ctx.expect_contact(x_cable_bracket, x_base, name="x_cable_bracket_attached")
    ctx.expect_contact(x_table, x_left_rail, name="x_table_left_bearing_contact")
    ctx.expect_contact(x_table, x_right_rail, name="x_table_right_bearing_contact")
    ctx.expect_gap(x_table, x_base, axis="z", min_gap=0.010, max_penetration=0.0, name="x_stage_stack_height")
    ctx.expect_overlap(x_table, x_base, axes="xy", min_overlap=0.18, name="x_table_supported_over_base")

    ctx.expect_contact(y_base, x_table, name="y_base_mounted_to_x_table")
    ctx.expect_contact(y_left_rail, y_base, name="y_left_rail_mounted")
    ctx.expect_contact(y_right_rail, y_base, name="y_right_rail_mounted")
    ctx.expect_gap(y_left_rail, y_base, axis="z", max_gap=0.001, max_penetration=0.0, name="y_left_rail_seated")
    ctx.expect_gap(y_right_rail, y_base, axis="z", max_gap=0.001, max_penetration=0.0, name="y_right_rail_seated")
    ctx.expect_contact(y_front_cover, y_base, name="y_front_cover_attached")
    ctx.expect_contact(y_rear_cover, y_base, name="y_rear_cover_attached")
    ctx.expect_gap(y_front_cover, y_base, axis="y", max_gap=0.001, max_penetration=0.0, name="y_front_cover_flush")
    ctx.expect_gap(y_base, y_rear_cover, axis="y", max_gap=0.001, max_penetration=0.0, name="y_rear_cover_flush")
    ctx.expect_contact(y_table, y_left_rail, name="y_table_left_bearing_contact")
    ctx.expect_contact(y_table, y_right_rail, name="y_table_right_bearing_contact")
    ctx.expect_gap(y_table, y_base, axis="z", min_gap=0.010, max_penetration=0.0, name="y_stage_stack_height")
    ctx.expect_overlap(y_table, y_base, axes="xy", min_overlap=0.14, name="y_table_supported_over_y_base")
    ctx.expect_contact(y_cable_bracket, y_table, name="y_cable_bracket_attached")

    with ctx.pose({x_slide: X_TRAVEL / 2.0}):
        ctx.expect_origin_gap(x_table, x_base, axis="x", min_gap=0.079, max_gap=0.081, name="x_slide_positive_travel")
        ctx.expect_origin_gap(y_base, x_base, axis="x", min_gap=0.079, max_gap=0.081, name="y_stack_follows_x_positive")
        ctx.expect_contact(x_table, x_left_rail, name="x_positive_left_contact")
        ctx.expect_contact(x_table, x_right_rail, name="x_positive_right_contact")
        ctx.expect_overlap(x_table, x_base, axes="xy", min_overlap=0.18, name="x_positive_support_overlap")

    with ctx.pose({x_slide: -(X_TRAVEL / 2.0)}):
        ctx.expect_origin_gap(x_base, x_table, axis="x", min_gap=0.079, max_gap=0.081, name="x_slide_negative_travel")
        ctx.expect_origin_gap(x_base, y_base, axis="x", min_gap=0.079, max_gap=0.081, name="y_stack_follows_x_negative")
        ctx.expect_contact(x_table, x_left_rail, name="x_negative_left_contact")
        ctx.expect_contact(x_table, x_right_rail, name="x_negative_right_contact")

    with ctx.pose({y_slide: Y_TRAVEL / 2.0}):
        ctx.expect_origin_gap(y_table, y_base, axis="y", min_gap=0.069, max_gap=0.071, name="y_slide_positive_travel")
        ctx.expect_contact(y_table, y_left_rail, name="y_positive_left_contact")
        ctx.expect_contact(y_table, y_right_rail, name="y_positive_right_contact")
        ctx.expect_overlap(y_table, y_base, axes="xy", min_overlap=0.14, name="y_positive_support_overlap")

    with ctx.pose({y_slide: -(Y_TRAVEL / 2.0)}):
        ctx.expect_origin_gap(y_base, y_table, axis="y", min_gap=0.069, max_gap=0.071, name="y_slide_negative_travel")
        ctx.expect_contact(y_table, y_left_rail, name="y_negative_left_contact")
        ctx.expect_contact(y_table, y_right_rail, name="y_negative_right_contact")

    with ctx.pose({x_slide: X_TRAVEL / 2.0, y_slide: Y_TRAVEL / 2.0}):
        ctx.expect_gap(y_table, x_table, axis="z", min_gap=0.040, max_penetration=0.0, name="stack_clearance_at_corner_pose")
        ctx.expect_overlap(y_table, y_base, axes="xy", min_overlap=0.14, name="corner_pose_y_support")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
