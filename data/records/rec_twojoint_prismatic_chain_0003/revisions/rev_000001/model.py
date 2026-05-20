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


BASE_L = 0.300
BASE_W = 0.180
BASE_T = 0.018

X_RAIL_L = 0.220
X_RAIL_W = 0.018
X_RAIL_H = 0.012
X_RAIL_Y = 0.055

X_BLOCK_L = 0.040
X_BLOCK_W = 0.024
X_BLOCK_H = 0.018

X_CARR_L = 0.150
X_CARR_W = 0.135
X_CARR_H = 0.024
X_CARR_Z = BASE_T + X_RAIL_H + X_BLOCK_H
X_TRAVEL = 0.050

X_COVER_T = 0.003
X_COVER_H = 0.026
X_COVER_L = 0.118

Y_RAIL_L = 0.135
Y_RAIL_W = 0.016
Y_RAIL_H = 0.010
Y_RAIL_X = 0.032
Y_RAIL_MOUNT_Z = X_CARR_H

Y_BLOCK_L = 0.022
Y_BLOCK_W = 0.036
Y_BLOCK_H = 0.014

Y_CARR_L = 0.108
Y_CARR_W = 0.112
Y_CARR_H = 0.020
Y_CARR_Z = Y_RAIL_MOUNT_Z + Y_RAIL_H + Y_BLOCK_H
Y_TRAVEL = 0.030

Y_COVER_L = 0.072
Y_COVER_W = 0.050
Y_COVER_T = 0.003


def _mount_points(span: float, count: int) -> list[tuple[float, float]]:
    if count <= 1:
        return [(0.0, 0.0)]
    pitch = span / (count - 1)
    return [(-span / 2 + i * pitch, 0.0) for i in range(count)]


def _base_frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    pocket = (
        cq.Workplane("XY")
        .box(BASE_L - 0.050, BASE_W - 0.046, BASE_T - 0.007, centered=(True, True, False))
        .translate((0.0, 0.0, 0.007))
    )
    side_slot = cq.Workplane("XY").box(0.120, 0.016, BASE_T, centered=(True, True, False))
    rail_land_left = cq.Workplane("XY").box(0.236, 0.030, BASE_T, centered=(True, True, False)).translate((0.0, X_RAIL_Y, 0.0))
    rail_land_right = cq.Workplane("XY").box(0.236, 0.030, BASE_T, centered=(True, True, False)).translate((0.0, -X_RAIL_Y, 0.0))
    base = base.cut(pocket)
    base = base.cut(side_slot.translate((0.0, 0.052, 0.0)))
    base = base.cut(side_slot.translate((0.0, -0.052, 0.0)))
    base = base.union(rail_land_left).union(rail_land_right)
    return (
        base.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(
            [
                (-0.115, -0.070),
                (-0.115, 0.070),
                (0.115, -0.070),
                (0.115, 0.070),
                (-0.070, 0.0),
                (0.070, 0.0),
            ]
        )
        .hole(0.005)
    )


def _rail_shape(length: float, width: float, height: float, axis: str, hole_count: int) -> cq.Workplane:
    rail = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    pts = _mount_points(length - 0.040, hole_count) if axis == "x" else [(0.0, y) for (y, _) in _mount_points(length - 0.030, hole_count)]
    return rail.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(pts).hole(0.0042)


def _guide_block_shape(length: float, width: float, height: float, bolt_pitch: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    side_relief = (
        cq.Workplane("XY")
        .box(length * 0.55, width * 0.18, height * 0.50, centered=(True, True, False))
        .translate((0.0, width * 0.38, height * 0.18))
    )
    body = body.cut(side_relief).cut(side_relief.translate((0.0, -2.0 * width * 0.38, 0.0)))
    return (
        body.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-bolt_pitch / 2, 0.0), (bolt_pitch / 2, 0.0)])
        .cboreHole(0.0038, 0.0068, 0.0035)
    )


def _end_bracket_shape(length: float, width: float, height: float, window_l: float, window_w: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    window = (
        cq.Workplane("XY")
        .box(window_l, window_w, height * 0.52, centered=(True, True, False))
        .translate((0.0, 0.0, height * 0.10))
    )
    return (
        body.cut(window)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-length * 0.20, 0.0), (length * 0.20, 0.0)])
        .hole(0.0036)
    )


def _alignment_plate_shape(length: float, width: float, height: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-length * 0.28, 0.0), (length * 0.28, 0.0)])
        .cboreHole(0.0036, 0.0065, height * 0.55)
    )


def _x_carriage_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(X_CARR_L, X_CARR_W, 0.010, centered=(True, True, False))
    upper = (
        cq.Workplane("XY")
        .box(0.118, 0.100, X_CARR_H - 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    body = lower.union(upper)
    top_pocket = (
        cq.Workplane("XY")
        .box(0.080, 0.060, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    side_window = (
        cq.Workplane("XY")
        .box(0.060, 0.020, 0.010, centered=(True, True, False))
        .translate((0.0, 0.040, 0.004))
    )
    return (
        body.cut(top_pocket)
        .cut(side_window)
        .cut(side_window.translate((0.0, -0.080, 0.0)))
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(
            [
                (-Y_RAIL_X, -0.036),
                (-Y_RAIL_X, 0.036),
                (Y_RAIL_X, -0.036),
                (Y_RAIL_X, 0.036),
            ]
        )
        .hole(0.0036)
    )


def _cover_plate_shape(length: float, width: float, height: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-length * 0.34, 0.0), (length * 0.34, 0.0)])
        .cboreHole(0.0028, 0.0052, height * 0.60)
    )


def _y_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(Y_CARR_L, Y_CARR_W, Y_CARR_H, centered=(True, True, False))
    top_pocket = (
        cq.Workplane("XY")
        .box(0.060, 0.074, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    side_relief = (
        cq.Workplane("XY")
        .box(0.020, 0.048, 0.008, centered=(True, True, False))
        .translate((0.032, 0.0, 0.006))
    )
    return (
        body.cut(top_pocket)
        .cut(side_relief)
        .cut(side_relief.translate((-0.064, 0.0, 0.0)))
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-0.026, -0.026), (-0.026, 0.026), (0.026, -0.026), (0.026, 0.026)])
        .hole(0.0034)
    )


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    size: tuple[float, float, float],
    mass: float,
    material: str,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, f"{name}.obj", assets=ASSETS),
        material=material,
        name="shell",
    )
    part.inertial = Inertial.from_geometry(
        Box(size),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, size[2] / 2)),
    )
    return part


def _fixed(model: ArticulatedObject, name: str, parent, child, xyz: tuple[float, float, float]) -> None:
    model.articulation(
        name,
        ArticulationType.FIXED,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_prismatic_chain", assets=ASSETS)

    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.46, 0.47, 0.50, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.18, 0.19, 0.20, 1.0))

    base_frame = _add_mesh_part(
        model,
        name="base_frame",
        shape=_base_frame_shape(),
        size=(BASE_L, BASE_W, BASE_T),
        mass=7.5,
        material=dark_steel,
    )

    x_rail_left = _add_mesh_part(
        model,
        name="x_rail_left",
        shape=_rail_shape(X_RAIL_L, X_RAIL_W, X_RAIL_H, "x", 5),
        size=(X_RAIL_L, X_RAIL_W, X_RAIL_H),
        mass=0.65,
        material=steel,
    )
    x_rail_right = _add_mesh_part(
        model,
        name="x_rail_right",
        shape=_rail_shape(X_RAIL_L, X_RAIL_W, X_RAIL_H, "x", 5),
        size=(X_RAIL_L, X_RAIL_W, X_RAIL_H),
        mass=0.65,
        material=steel,
    )
    x_end_bracket_left = _add_mesh_part(
        model,
        name="x_end_bracket_left",
        shape=_end_bracket_shape(0.018, 0.148, 0.028, 0.010, 0.090),
        size=(0.018, 0.148, 0.028),
        mass=0.38,
        material=dark_steel,
    )
    x_end_bracket_right = _add_mesh_part(
        model,
        name="x_end_bracket_right",
        shape=_end_bracket_shape(0.018, 0.148, 0.028, 0.010, 0.090),
        size=(0.018, 0.148, 0.028),
        mass=0.38,
        material=dark_steel,
    )
    base_alignment_plate = _add_mesh_part(
        model,
        name="base_alignment_plate",
        shape=_alignment_plate_shape(0.090, 0.026, 0.008),
        size=(0.090, 0.026, 0.008),
        mass=0.12,
        material=cover_gray,
    )

    x_carriage = _add_mesh_part(
        model,
        name="x_carriage",
        shape=_x_carriage_shape(),
        size=(X_CARR_L, X_CARR_W, X_CARR_H),
        mass=1.8,
        material=dark_steel,
    )

    x_block_lf = _add_mesh_part(
        model,
        name="x_block_lf",
        shape=_guide_block_shape(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H, 0.020),
        size=(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H),
        mass=0.22,
        material=black_oxide,
    )
    x_block_lr = _add_mesh_part(
        model,
        name="x_block_lr",
        shape=_guide_block_shape(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H, 0.020),
        size=(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H),
        mass=0.22,
        material=black_oxide,
    )
    x_block_rf = _add_mesh_part(
        model,
        name="x_block_rf",
        shape=_guide_block_shape(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H, 0.020),
        size=(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H),
        mass=0.22,
        material=black_oxide,
    )
    x_block_rr = _add_mesh_part(
        model,
        name="x_block_rr",
        shape=_guide_block_shape(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H, 0.020),
        size=(X_BLOCK_L, X_BLOCK_W, X_BLOCK_H),
        mass=0.22,
        material=black_oxide,
    )
    x_cover_left = _add_mesh_part(
        model,
        name="x_cover_left",
        shape=_cover_plate_shape(X_COVER_L, X_COVER_T, X_COVER_H),
        size=(X_COVER_L, X_COVER_T, X_COVER_H),
        mass=0.09,
        material=cover_gray,
    )
    x_cover_right = _add_mesh_part(
        model,
        name="x_cover_right",
        shape=_cover_plate_shape(X_COVER_L, X_COVER_T, X_COVER_H),
        size=(X_COVER_L, X_COVER_T, X_COVER_H),
        mass=0.09,
        material=cover_gray,
    )

    y_rail_front = _add_mesh_part(
        model,
        name="y_rail_front",
        shape=_rail_shape(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, "y", 4),
        size=(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H),
        mass=0.32,
        material=steel,
    )
    y_rail_rear = _add_mesh_part(
        model,
        name="y_rail_rear",
        shape=_rail_shape(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, "y", 4),
        size=(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H),
        mass=0.32,
        material=steel,
    )
    y_end_plate_front = _add_mesh_part(
        model,
        name="y_end_plate_front",
        shape=_end_bracket_shape(0.082, 0.012, 0.020, 0.042, 0.006),
        size=(0.082, 0.012, 0.020),
        mass=0.11,
        material=cover_gray,
    )
    y_end_plate_rear = _add_mesh_part(
        model,
        name="y_end_plate_rear",
        shape=_end_bracket_shape(0.082, 0.012, 0.020, 0.042, 0.006),
        size=(0.082, 0.012, 0.020),
        mass=0.11,
        material=cover_gray,
    )

    y_carriage = _add_mesh_part(
        model,
        name="y_carriage",
        shape=_y_carriage_shape(),
        size=(Y_CARR_L, Y_CARR_W, Y_CARR_H),
        mass=1.0,
        material=dark_steel,
    )

    y_block_ff = _add_mesh_part(
        model,
        name="y_block_ff",
        shape=_guide_block_shape(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H, 0.014),
        size=(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H),
        mass=0.12,
        material=black_oxide,
    )
    y_block_fr = _add_mesh_part(
        model,
        name="y_block_fr",
        shape=_guide_block_shape(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H, 0.014),
        size=(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H),
        mass=0.12,
        material=black_oxide,
    )
    y_block_rf = _add_mesh_part(
        model,
        name="y_block_rf",
        shape=_guide_block_shape(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H, 0.014),
        size=(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H),
        mass=0.12,
        material=black_oxide,
    )
    y_block_rr = _add_mesh_part(
        model,
        name="y_block_rr",
        shape=_guide_block_shape(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H, 0.014),
        size=(Y_BLOCK_L, Y_BLOCK_W, Y_BLOCK_H),
        mass=0.12,
        material=black_oxide,
    )
    y_cover = _add_mesh_part(
        model,
        name="y_cover",
        shape=_cover_plate_shape(Y_COVER_L, Y_COVER_W, Y_COVER_T),
        size=(Y_COVER_L, Y_COVER_W, Y_COVER_T),
        mass=0.06,
        material=cover_gray,
    )

    _fixed(model, "base_to_x_rail_left", base_frame, x_rail_left, (0.0, X_RAIL_Y, BASE_T))
    _fixed(model, "base_to_x_rail_right", base_frame, x_rail_right, (0.0, -X_RAIL_Y, BASE_T))
    _fixed(model, "base_to_x_end_bracket_left", base_frame, x_end_bracket_left, (-0.141, 0.0, BASE_T))
    _fixed(model, "base_to_x_end_bracket_right", base_frame, x_end_bracket_right, (0.141, 0.0, BASE_T))
    _fixed(model, "base_to_alignment_plate", base_frame, base_alignment_plate, (0.0, 0.0, BASE_T))

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_CARR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.20,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )

    _fixed(model, "x_carriage_to_x_block_lf", x_carriage, x_block_lf, (-0.045, X_RAIL_Y, -X_BLOCK_H))
    _fixed(model, "x_carriage_to_x_block_lr", x_carriage, x_block_lr, (0.045, X_RAIL_Y, -X_BLOCK_H))
    _fixed(model, "x_carriage_to_x_block_rf", x_carriage, x_block_rf, (-0.045, -X_RAIL_Y, -X_BLOCK_H))
    _fixed(model, "x_carriage_to_x_block_rr", x_carriage, x_block_rr, (0.045, -X_RAIL_Y, -X_BLOCK_H))
    _fixed(
        model,
        "x_carriage_to_x_cover_left",
        x_carriage,
        x_cover_left,
        (0.0, X_CARR_W / 2 + X_COVER_T / 2, 0.0),
    )
    _fixed(
        model,
        "x_carriage_to_x_cover_right",
        x_carriage,
        x_cover_right,
        (0.0, -(X_CARR_W / 2 + X_COVER_T / 2), 0.0),
    )
    _fixed(model, "x_carriage_to_y_rail_front", x_carriage, y_rail_front, (Y_RAIL_X, 0.0, Y_RAIL_MOUNT_Z))
    _fixed(model, "x_carriage_to_y_rail_rear", x_carriage, y_rail_rear, (-Y_RAIL_X, 0.0, Y_RAIL_MOUNT_Z))
    _fixed(model, "x_carriage_to_y_end_plate_front", x_carriage, y_end_plate_front, (0.0, 0.0615, Y_RAIL_MOUNT_Z))
    _fixed(model, "x_carriage_to_y_end_plate_rear", x_carriage, y_end_plate_rear, (0.0, -0.0615, Y_RAIL_MOUNT_Z))

    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, Y_CARR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.18,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    _fixed(model, "y_carriage_to_y_block_ff", y_carriage, y_block_ff, (Y_RAIL_X, -0.030, -Y_BLOCK_H))
    _fixed(model, "y_carriage_to_y_block_fr", y_carriage, y_block_fr, (Y_RAIL_X, 0.030, -Y_BLOCK_H))
    _fixed(model, "y_carriage_to_y_block_rf", y_carriage, y_block_rf, (-Y_RAIL_X, -0.030, -Y_BLOCK_H))
    _fixed(model, "y_carriage_to_y_block_rr", y_carriage, y_block_rr, (-Y_RAIL_X, 0.030, -Y_BLOCK_H))
    _fixed(model, "y_carriage_to_y_cover", y_carriage, y_cover, (0.0, 0.0, Y_CARR_H))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    part_names = [
        "base_frame",
        "x_rail_left",
        "x_rail_right",
        "x_end_bracket_left",
        "x_end_bracket_right",
        "base_alignment_plate",
        "x_carriage",
        "x_block_lf",
        "x_block_lr",
        "x_block_rf",
        "x_block_rr",
        "x_cover_left",
        "x_cover_right",
        "y_rail_front",
        "y_rail_rear",
        "y_end_plate_front",
        "y_end_plate_rear",
        "y_carriage",
        "y_block_ff",
        "y_block_fr",
        "y_block_rf",
        "y_block_rr",
        "y_cover",
    ]
    parts = {name: object_model.get_part(name) for name in part_names}
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    for name, part in parts.items():
        ctx.check(f"has_{name}", part is not None, f"missing part {name}")

    ctx.check(
        "x_slide_axis_is_x",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        f"unexpected x_slide axis {x_slide.axis}",
    )
    ctx.check(
        "y_slide_axis_is_y",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        f"unexpected y_slide axis {y_slide.axis}",
    )
    ctx.check(
        "x_slide_limits",
        abs(x_slide.motion_limits.lower + X_TRAVEL) < 1e-9 and abs(x_slide.motion_limits.upper - X_TRAVEL) < 1e-9,
        "x_slide travel limits do not match intended compact stroke",
    )
    ctx.check(
        "y_slide_limits",
        abs(y_slide.motion_limits.lower + Y_TRAVEL) < 1e-9 and abs(y_slide.motion_limits.upper - Y_TRAVEL) < 1e-9,
        "y_slide travel limits do not match intended compact stroke",
    )

    base = parts["base_frame"]
    for mounted_name in [
        "x_rail_left",
        "x_rail_right",
        "x_end_bracket_left",
        "x_end_bracket_right",
        "base_alignment_plate",
    ]:
        mounted = parts[mounted_name]
        ctx.expect_gap(mounted, base, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{mounted_name}_seats_on_base")
        ctx.expect_overlap(mounted, base, axes="xy", min_overlap=0.010, name=f"{mounted_name}_footprint_on_base")

    x_carriage = parts["x_carriage"]
    for block_name in ["x_block_lf", "x_block_lr", "x_block_rf", "x_block_rr"]:
        block = parts[block_name]
        ctx.expect_gap(x_carriage, block, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{block_name}_seats_under_x_carriage")
        ctx.expect_overlap(x_carriage, block, axes="xy", min_overlap=0.016, name=f"{block_name}_mount_overlap_with_x_carriage")

    ctx.expect_contact(parts["x_cover_left"], x_carriage, name="x_cover_left_contacts_x_carriage")
    ctx.expect_contact(parts["x_cover_right"], x_carriage, name="x_cover_right_contacts_x_carriage")
    ctx.expect_overlap(parts["x_cover_left"], x_carriage, axes="xz", min_overlap=0.020, name="x_cover_left_has_side_face_overlap")
    ctx.expect_overlap(parts["x_cover_right"], x_carriage, axes="xz", min_overlap=0.020, name="x_cover_right_has_side_face_overlap")

    for mounted_name in ["y_rail_front", "y_rail_rear", "y_end_plate_front", "y_end_plate_rear"]:
        mounted = parts[mounted_name]
        ctx.expect_gap(mounted, x_carriage, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{mounted_name}_seats_on_x_carriage")
        ctx.expect_overlap(mounted, x_carriage, axes="xy", min_overlap=0.010, name=f"{mounted_name}_footprint_on_x_carriage")

    y_carriage = parts["y_carriage"]
    for block_name in ["y_block_ff", "y_block_fr", "y_block_rf", "y_block_rr"]:
        block = parts[block_name]
        ctx.expect_gap(y_carriage, block, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{block_name}_seats_under_y_carriage")
        ctx.expect_overlap(y_carriage, block, axes="xy", min_overlap=0.014, name=f"{block_name}_mount_overlap_with_y_carriage")

    ctx.expect_gap(parts["y_cover"], y_carriage, axis="z", max_gap=0.0, max_penetration=0.0, name="y_cover_seats_on_y_carriage")
    ctx.expect_overlap(parts["y_cover"], y_carriage, axes="xy", min_overlap=0.040, name="y_cover_has_service_overlap")

    with ctx.pose({x_slide: 0.0, y_slide: 0.0}):
        for rail_name, block_names in {
            "x_rail_left": ["x_block_lf", "x_block_lr"],
            "x_rail_right": ["x_block_rf", "x_block_rr"],
        }.items():
            rail = parts[rail_name]
            for block_name in block_names:
                block = parts[block_name]
                ctx.expect_gap(block, rail, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{block_name}_runs_flush_on_{rail_name}_at_rest")
                ctx.expect_overlap(block, rail, axes="xy", min_overlap=0.017, name=f"{block_name}_retains_support_on_{rail_name}_at_rest")
        for rail_name, block_names in {
            "y_rail_front": ["y_block_ff", "y_block_fr"],
            "y_rail_rear": ["y_block_rf", "y_block_rr"],
        }.items():
            rail = parts[rail_name]
            for block_name in block_names:
                block = parts[block_name]
                ctx.expect_gap(block, rail, axis="z", max_gap=0.0, max_penetration=0.0, name=f"{block_name}_runs_flush_on_{rail_name}_at_rest")
                ctx.expect_overlap(block, rail, axes="xy", min_overlap=0.014, name=f"{block_name}_retains_support_on_{rail_name}_at_rest")

    for x_pose in (-0.046, 0.046):
        with ctx.pose({x_slide: x_pose}):
            for rail_name, block_names in {
                "x_rail_left": ["x_block_lf", "x_block_lr"],
                "x_rail_right": ["x_block_rf", "x_block_rr"],
            }.items():
                rail = parts[rail_name]
                for block_name in block_names:
                    block = parts[block_name]
                    ctx.expect_overlap(block, rail, axes="xy", min_overlap=0.014, name=f"{block_name}_keeps_x_support_at_{x_pose:+.3f}")

    for y_pose in (-0.028, 0.028):
        with ctx.pose({y_slide: y_pose}):
            for rail_name, block_names in {
                "y_rail_front": ["y_block_ff", "y_block_fr"],
                "y_rail_rear": ["y_block_rf", "y_block_rr"],
            }.items():
                rail = parts[rail_name]
                for block_name in block_names:
                    block = parts[block_name]
                    ctx.expect_overlap(block, rail, axes="xy", min_overlap=0.012, name=f"{block_name}_keeps_y_support_at_{y_pose:+.3f}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
