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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.46
BASE_W = 0.24
BASE_T = 0.026

X_RAIL_L = 0.38
X_RAIL_W = 0.018
X_RAIL_H = 0.012
X_RAIL_Y = 0.060

X_CARR_L = 0.17
X_CARR_W = 0.18
X_CARR_T = 0.020
X_BEARING_L = 0.048
X_BEARING_W = 0.032
X_BEARING_H = 0.016
X_BEARING_X = 0.052

X_STAGE_Z0 = BASE_T + X_RAIL_H + X_BEARING_H + (X_CARR_T / 2.0)

Z_RAIL_L = 0.24
Z_RAIL_W = 0.014
Z_RAIL_D = 0.012
Z_RAIL_X = 0.052

Z_STAGE_Y0 = -0.025
Z_STAGE_Z0 = 0.12


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS, tolerance=0.001, angular_tolerance=0.1)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cyl_x(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((center[0] - (length / 2.0), center[1], center[2]))


def _cover_mesh(length: float, width: float, thickness: float, bolt_pitch_x: float, bolt_pitch_y: float) -> cq.Workplane:
    cover = _cq_box((length, width, thickness), (0.0, 0.0, thickness / 2.0))
    for x in (-bolt_pitch_x / 2.0, bolt_pitch_x / 2.0):
        for y in (-bolt_pitch_y / 2.0, bolt_pitch_y / 2.0):
            head = cq.Workplane("XY").circle(0.0045).extrude(0.0025).translate((x, y, thickness))
            cover = cover.union(head)
    return cover


def _base_access_cover_mesh() -> cq.Workplane:
    cover = _cover_mesh(0.136, 0.066, 0.008, 0.108, 0.040)
    cover = cover.union(_cq_box((0.020, 0.020, 0.016), (-0.040, 0.050, -0.004)))
    cover = cover.union(_cq_box((0.020, 0.020, 0.016), (0.040, -0.050, -0.004)))
    return cover


def _make_base_bed() -> cq.Workplane:
    bed = _cq_box((BASE_L, BASE_W, BASE_T), (0.0, 0.0, BASE_T / 2.0))
    bed = bed.union(_cq_box((BASE_L - 0.04, 0.028, 0.050), (0.0, 0.095, 0.025)))
    bed = bed.union(_cq_box((BASE_L - 0.04, 0.028, 0.050), (0.0, -0.095, 0.025)))
    bed = bed.union(_cq_box((0.058, 0.150, 0.050), (0.170, 0.0, 0.025)))
    bed = bed.union(_cq_box((0.058, 0.150, 0.050), (-0.170, 0.0, 0.025)))

    bed = bed.cut(_cq_box((0.220, 0.100, 0.016), (0.0, 0.0, 0.018)))
    bed = bed.cut(_cq_box((0.320, 0.034, 0.012), (0.0, 0.0, 0.020)))

    for x in (-0.120, 0.0, 0.120):
        bed = bed.cut(_cq_box((0.070, 0.020, 0.026), (x, 0.095, 0.021)))
        bed = bed.cut(_cq_box((0.070, 0.020, 0.026), (x, -0.095, 0.021)))

    for x in (-0.185, 0.185):
        for y in (-0.095, 0.095):
            hole = cq.Workplane("XY").circle(0.0065).extrude(0.070).translate((x, y, -0.005))
            bed = bed.cut(hole)

    return bed


def _make_x_carriage_body() -> cq.Workplane:
    carriage = _cq_box((X_CARR_L, X_CARR_W, X_CARR_T), (0.0, 0.0, 0.0))
    carriage = carriage.union(_cq_box((0.150, 0.060, 0.018), (0.0, 0.040, 0.019)))
    carriage = carriage.cut(_cq_box((0.090, 0.075, 0.010), (0.0, 0.0, 0.004)))
    carriage = carriage.cut(_cq_box((0.050, 0.125, 0.022), (0.0, 0.0, -0.001)))
    carriage = carriage.cut(_cq_box((0.090, 0.022, 0.018), (0.0, -0.060, 0.001)))
    return carriage


def _make_column_body() -> cq.Workplane:
    column = _cq_box((0.160, 0.032, 0.310), (0.0, 0.032, 0.165))
    column = column.union(_cq_box((0.180, 0.026, 0.024), (0.0, 0.024, 0.302)))
    column = column.union(_cq_box((0.160, 0.050, 0.030), (0.0, 0.015, 0.025)))
    column = column.cut(_cq_box((0.092, 0.014, 0.150), (0.0, 0.043, 0.160)))
    column = column.cut(_cq_box((0.040, 0.060, 0.120), (0.0, 0.020, 0.160)))
    return column


def _make_gusset(offset_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline([(0.000, 0.010), (0.050, 0.010), (0.000, 0.130)])
        .close()
        .extrude(0.012)
        .translate((offset_x, 0.0, 0.0))
    )


def _make_z_carriage_body() -> cq.Workplane:
    carriage = _cq_box((0.150, 0.022, 0.150), (0.0, 0.0, 0.0))
    carriage = carriage.union(_cq_box((0.110, 0.018, 0.064), (0.0, -0.020, 0.0)))
    carriage = carriage.cut(_cq_box((0.046, 0.050, 0.052), (0.0, 0.0, 0.0)))
    carriage = carriage.cut(_cq_box((0.020, 0.030, 0.090), (-0.044, 0.0, 0.0)))
    carriage = carriage.cut(_cq_box((0.020, 0.030, 0.090), (0.044, 0.0, 0.0)))
    carriage = carriage.cut(_cq_box((0.074, 0.012, 0.092), (0.0, -0.006, 0.0)))
    return carriage


def _make_nut_block() -> cq.Workplane:
    nut = _cq_box((0.055, 0.026, 0.040), (0.0, 0.025, 0.0))
    bore = cq.Workplane("XY").circle(0.009).extrude(0.060).translate((0.0, 0.025, -0.030))
    return nut.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xz_stage", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.36, 0.39, 0.42, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    polymer = model.material("polymer", rgba=(0.08, 0.09, 0.10, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.56, 0.58, 0.60, 1.0))

    base = model.part("base")
    base.visual(_mesh(_make_base_bed(), "base_bed.obj"), material=painted_steel, name="base_bed")
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(xyz=(0.0, X_RAIL_Y, BASE_T + (X_RAIL_H / 2.0))),
        material=ground_steel,
        name="x_rail_left",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(xyz=(0.0, -X_RAIL_Y, BASE_T + (X_RAIL_H / 2.0))),
        material=ground_steel,
        name="x_rail_right",
    )
    base.visual(
        Box((0.040, 0.060, 0.048)),
        origin=Origin(xyz=(0.195, 0.0, 0.050)),
        material=painted_steel,
        name="x_screw_fixed_support",
    )
    base.visual(
        Box((0.040, 0.060, 0.048)),
        origin=Origin(xyz=(-0.195, 0.0, 0.050)),
        material=painted_steel,
        name="x_screw_free_support",
    )
    base.visual(
        _mesh(_cq_cyl_x(0.410, 0.006, (0.0, 0.0, 0.050)), "x_screw.obj"),
        material=ground_steel,
        name="x_screw",
    )
    base.visual(
        Box((0.016, 0.022, 0.028)),
        origin=Origin(xyz=(0.155, -0.084, 0.040)),
        material=black_oxide,
        name="x_stop_pos",
    )
    base.visual(
        Box((0.016, 0.022, 0.028)),
        origin=Origin(xyz=(-0.155, -0.084, 0.040)),
        material=black_oxide,
        name="x_stop_neg",
    )
    base.visual(
        _mesh(_base_access_cover_mesh().translate((0.0, 0.0, BASE_T)), "base_access_cover.obj"),
        material=cover_gray,
        name="base_access_cover",
    )
    base.inertial = Inertial.from_geometry(Box((BASE_L, BASE_W, 0.076)), mass=28.0, origin=Origin(xyz=(0.0, 0.0, 0.038)))

    x_stage = model.part("x_stage")
    x_stage.visual(_mesh(_make_x_carriage_body(), "x_carriage_body.obj"), material=painted_steel, name="x_carriage_body")
    x_stage.visual(
        Box((X_BEARING_L, X_BEARING_W, X_BEARING_H)),
        origin=Origin(xyz=(-X_BEARING_X, X_RAIL_Y, -0.018)),
        material=black_oxide,
        name="x_bearing_lf",
    )
    x_stage.visual(
        Box((X_BEARING_L, X_BEARING_W, X_BEARING_H)),
        origin=Origin(xyz=(X_BEARING_X, X_RAIL_Y, -0.018)),
        material=black_oxide,
        name="x_bearing_lr",
    )
    x_stage.visual(
        Box((X_BEARING_L, X_BEARING_W, X_BEARING_H)),
        origin=Origin(xyz=(-X_BEARING_X, -X_RAIL_Y, -0.018)),
        material=black_oxide,
        name="x_bearing_rf",
    )
    x_stage.visual(
        Box((X_BEARING_L, X_BEARING_W, X_BEARING_H)),
        origin=Origin(xyz=(X_BEARING_X, -X_RAIL_Y, -0.018)),
        material=black_oxide,
        name="x_bearing_rr",
    )
    x_stage.visual(_mesh(_make_column_body(), "column_body.obj"), material=painted_steel, name="column_body")
    x_stage.visual(_mesh(_make_gusset(-0.086), "column_gusset_left.obj"), material=painted_steel, name="column_gusset_left")
    x_stage.visual(_mesh(_make_gusset(0.074), "column_gusset_right.obj"), material=painted_steel, name="column_gusset_right")
    x_stage.visual(
        Box((Z_RAIL_W, Z_RAIL_D, Z_RAIL_L)),
        origin=Origin(xyz=(-Z_RAIL_X, 0.010, 0.160)),
        material=ground_steel,
        name="z_rail_left",
    )
    x_stage.visual(
        Box((Z_RAIL_W, Z_RAIL_D, Z_RAIL_L)),
        origin=Origin(xyz=(Z_RAIL_X, 0.010, 0.160)),
        material=ground_steel,
        name="z_rail_right",
    )
    x_stage.visual(
        Box((0.050, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.045)),
        material=painted_steel,
        name="z_screw_bottom_support",
    )
    x_stage.visual(
        Box((0.050, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.275)),
        material=painted_steel,
        name="z_screw_top_support",
    )
    x_stage.visual(
        Cylinder(radius=0.0065, length=0.260),
        origin=Origin(xyz=(0.0, -0.001, 0.160)),
        material=ground_steel,
        name="z_screw",
    )
    x_stage.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.078, -0.082, 0.002)),
        material=black_oxide,
        name="x_bumper_pos",
    )
    x_stage.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.078, -0.082, 0.002)),
        material=black_oxide,
        name="x_bumper_neg",
    )
    x_stage.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.072, 0.056, 0.034)),
        material=black_oxide,
        name="z_stop_bottom",
    )
    x_stage.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.072, 0.056, 0.296)),
        material=black_oxide,
        name="z_stop_top",
    )
    x_stage.visual(
        Box((0.116, 0.008, 0.176)),
        origin=Origin(xyz=(0.0, 0.052, 0.114)),
        material=cover_gray,
        name="rear_access_cover",
    )
    x_stage.inertial = Inertial.from_geometry(Box((0.20, 0.20, 0.34)), mass=16.0, origin=Origin(xyz=(0.0, 0.02, 0.15)))

    z_stage = model.part("z_stage")
    z_stage.visual(_mesh(_make_z_carriage_body(), "z_carriage_body.obj"), material=painted_steel, name="z_carriage_body")
    z_stage.visual(
        Box((0.028, 0.018, 0.036)),
        origin=Origin(xyz=(-Z_RAIL_X, 0.020, -0.050)),
        material=black_oxide,
        name="z_bearing_ll",
    )
    z_stage.visual(
        Box((0.028, 0.018, 0.036)),
        origin=Origin(xyz=(-Z_RAIL_X, 0.020, 0.050)),
        material=black_oxide,
        name="z_bearing_lu",
    )
    z_stage.visual(
        Box((0.028, 0.018, 0.036)),
        origin=Origin(xyz=(Z_RAIL_X, 0.020, -0.050)),
        material=black_oxide,
        name="z_bearing_rl",
    )
    z_stage.visual(
        Box((0.028, 0.018, 0.036)),
        origin=Origin(xyz=(Z_RAIL_X, 0.020, 0.050)),
        material=black_oxide,
        name="z_bearing_ru",
    )
    z_stage.visual(
        _mesh(_make_nut_block().translate((0.0, -0.003, 0.0)), "z_nut_block.obj"),
        material=black_oxide,
        name="z_nut_block",
    )
    z_stage.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(-Z_RAIL_X, 0.028, -0.050)),
        material=polymer,
        name="z_wiper_ll",
    )
    z_stage.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(-Z_RAIL_X, 0.028, 0.050)),
        material=polymer,
        name="z_wiper_lu",
    )
    z_stage.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(Z_RAIL_X, 0.028, -0.050)),
        material=polymer,
        name="z_wiper_rl",
    )
    z_stage.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(Z_RAIL_X, 0.028, 0.050)),
        material=polymer,
        name="z_wiper_ru",
    )
    z_stage.visual(
        Box((0.026, 0.014, 0.010)),
        origin=Origin(xyz=(0.072, 0.000, -0.072)),
        material=black_oxide,
        name="z_bumper_bottom",
    )
    z_stage.visual(
        Box((0.026, 0.014, 0.010)),
        origin=Origin(xyz=(0.072, 0.000, 0.072)),
        material=black_oxide,
        name="z_bumper_top",
    )
    z_stage.inertial = Inertial.from_geometry(Box((0.16, 0.05, 0.17)), mass=8.0, origin=Origin(xyz=(0.0, 0.0, 0.0)))

    model.articulation(
        "base_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.25, lower=-0.06, upper=0.06),
    )
    model.articulation(
        "x_stage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, Z_STAGE_Y0, Z_STAGE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.09),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    z_stage = object_model.get_part("z_stage")
    x_axis = object_model.get_articulation("base_to_x_stage")
    z_axis = object_model.get_articulation("x_stage_to_z_stage")

    def visual(part, visual_name):
        try:
            value = part.get_visual(visual_name)
        except Exception as exc:  # pragma: no cover - defensive test plumbing
            ctx.fail(f"visual_present_{part.name}_{visual_name}", str(exc))
            return None
        ctx.check(f"visual_present_{part.name}_{visual_name}", value is not None, f"Missing visual {visual_name} on {part.name}")
        return value

    x_rail_left = visual(base, "x_rail_left")
    x_rail_right = visual(base, "x_rail_right")
    x_stop_pos = visual(base, "x_stop_pos")
    x_stop_neg = visual(base, "x_stop_neg")
    x_bearing_lf = visual(x_stage, "x_bearing_lf")
    x_bearing_lr = visual(x_stage, "x_bearing_lr")
    x_bearing_rf = visual(x_stage, "x_bearing_rf")
    x_bearing_rr = visual(x_stage, "x_bearing_rr")
    x_bumper_pos = visual(x_stage, "x_bumper_pos")
    x_bumper_neg = visual(x_stage, "x_bumper_neg")
    column_body = visual(x_stage, "column_body")
    z_rail_left = visual(x_stage, "z_rail_left")
    z_rail_right = visual(x_stage, "z_rail_right")
    z_stop_bottom = visual(x_stage, "z_stop_bottom")
    z_stop_top = visual(x_stage, "z_stop_top")
    z_carriage_body = visual(z_stage, "z_carriage_body")
    z_bearing_ll = visual(z_stage, "z_bearing_ll")
    z_bearing_lu = visual(z_stage, "z_bearing_lu")
    z_bearing_rl = visual(z_stage, "z_bearing_rl")
    z_bearing_ru = visual(z_stage, "z_bearing_ru")
    z_bumper_bottom = visual(z_stage, "z_bumper_bottom")
    z_bumper_top = visual(z_stage, "z_bumper_top")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=20, ignore_adjacent=False, ignore_fixed=True)

    ctx.check("parts_present", all(part is not None for part in (base, x_stage, z_stage)), "Expected three stage parts")
    ctx.check("joints_present", all(joint is not None for joint in (x_axis, z_axis)), "Expected X and Z prismatic joints")

    if all(v is not None for v in (x_bearing_lf, x_rail_left)):
        ctx.expect_contact(x_stage, base, elem_a=x_bearing_lf, elem_b=x_rail_left, name="x_left_front_bearing_on_left_rail")
    if all(v is not None for v in (x_bearing_lr, x_rail_left)):
        ctx.expect_contact(x_stage, base, elem_a=x_bearing_lr, elem_b=x_rail_left, name="x_left_rear_bearing_on_left_rail")
    if all(v is not None for v in (x_bearing_rf, x_rail_right)):
        ctx.expect_contact(x_stage, base, elem_a=x_bearing_rf, elem_b=x_rail_right, name="x_right_front_bearing_on_right_rail")
    if all(v is not None for v in (x_bearing_rr, x_rail_right)):
        ctx.expect_contact(x_stage, base, elem_a=x_bearing_rr, elem_b=x_rail_right, name="x_right_rear_bearing_on_right_rail")

    if all(v is not None for v in (z_bearing_ll, z_rail_left)):
        ctx.expect_contact(z_stage, x_stage, elem_a=z_bearing_ll, elem_b=z_rail_left, name="z_lower_left_bearing_on_left_rail")
    if all(v is not None for v in (z_bearing_lu, z_rail_left)):
        ctx.expect_contact(z_stage, x_stage, elem_a=z_bearing_lu, elem_b=z_rail_left, name="z_upper_left_bearing_on_left_rail")
    if all(v is not None for v in (z_bearing_rl, z_rail_right)):
        ctx.expect_contact(z_stage, x_stage, elem_a=z_bearing_rl, elem_b=z_rail_right, name="z_lower_right_bearing_on_right_rail")
    if all(v is not None for v in (z_bearing_ru, z_rail_right)):
        ctx.expect_contact(z_stage, x_stage, elem_a=z_bearing_ru, elem_b=z_rail_right, name="z_upper_right_bearing_on_right_rail")

    if all(v is not None for v in (z_carriage_body, column_body)):
        ctx.expect_overlap(z_stage, x_stage, axes="x", min_overlap=0.14, elem_a=z_carriage_body, elem_b=column_body, name="z_carriage_centered_on_column")
    if all(v is not None for v in (z_carriage_body, column_body)):
        ctx.expect_within(z_stage, x_stage, axes="x", margin=0.006, inner_elem=z_carriage_body, outer_elem=column_body, name="z_carriage_within_column_width")

    with ctx.pose({x_axis: 0.09}):
        ctx.expect_origin_distance(x_stage, base, axes="x", min_dist=0.089, max_dist=0.091, name="x_axis_positive_travel_amount")
    with ctx.pose({x_axis: -0.09}):
        ctx.expect_origin_distance(x_stage, base, axes="x", min_dist=0.089, max_dist=0.091, name="x_axis_negative_travel_amount")
    with ctx.pose({z_axis: 0.00}):
        ctx.expect_origin_gap(z_stage, x_stage, axis="z", min_gap=0.119, max_gap=0.121, name="z_axis_home_height")
    with ctx.pose({z_axis: 0.08}):
        ctx.expect_origin_gap(z_stage, x_stage, axis="z", min_gap=0.199, max_gap=0.201, name="z_axis_raised_height")

    with ctx.pose({x_axis: 0.11}):
        if all(v is not None for v in (x_stop_pos, x_bumper_pos)):
            ctx.expect_gap(base, x_stage, axis="x", positive_elem=x_stop_pos, negative_elem=x_bumper_pos, min_gap=0.0035, max_gap=0.0065, name="x_positive_stop_clearance")
    with ctx.pose({x_axis: -0.11}):
        if all(v is not None for v in (x_stop_neg, x_bumper_neg)):
            ctx.expect_gap(x_stage, base, axis="x", positive_elem=x_bumper_neg, negative_elem=x_stop_neg, min_gap=0.0035, max_gap=0.0065, name="x_negative_stop_clearance")
    with ctx.pose({z_axis: 0.00}):
        if all(v is not None for v in (z_stop_bottom, z_bumper_bottom)):
            ctx.expect_gap(z_stage, x_stage, axis="z", positive_elem=z_bumper_bottom, negative_elem=z_stop_bottom, min_gap=0.0035, max_gap=0.0065, name="z_bottom_stop_clearance")
    with ctx.pose({z_axis: 0.09}):
        if all(v is not None for v in (z_stop_top, z_bumper_top)):
            ctx.expect_gap(x_stage, z_stage, axis="z", positive_elem=z_stop_top, negative_elem=z_bumper_top, min_gap=0.0035, max_gap=0.0065, name="z_top_stop_clearance")

    with ctx.pose({x_axis: 0.07, z_axis: 0.05}):
        if all(v is not None for v in (x_bearing_lf, x_rail_left)):
            ctx.expect_contact(x_stage, base, elem_a=x_bearing_lf, elem_b=x_rail_left, name="x_bearing_contact_in_combined_pose")
        if all(v is not None for v in (z_bearing_ru, z_rail_right)):
            ctx.expect_contact(z_stage, x_stage, elem_a=z_bearing_ru, elem_b=z_rail_right, name="z_bearing_contact_in_combined_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
