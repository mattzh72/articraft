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


BASE_LENGTH = 0.52
BASE_WIDTH = 0.34
BASE_SKIRT_HEIGHT = 0.034
BASE_DECK_THICKNESS = 0.018
BASE_TOP_Z = BASE_SKIRT_HEIGHT + BASE_DECK_THICKNESS

X_RAIL_LENGTH = 0.42
X_RAIL_WIDTH = 0.032
X_RAIL_HEIGHT = 0.014
X_RAIL_Y = 0.096
FEATURE_SINK = 0.001
X_RAIL_BOTTOM_Z = BASE_TOP_Z - FEATURE_SINK
X_RAIL_TOP_Z = X_RAIL_BOTTOM_Z + X_RAIL_HEIGHT

X_PAD_LENGTH = 0.17
X_PAD_WIDTH = 0.050
X_PAD_HEIGHT = 0.022
X_BODY_LENGTH = 0.22
X_BODY_WIDTH = 0.24
X_BODY_THICKNESS = 0.018
X_BODY_BOTTOM_Z = X_PAD_HEIGHT - FEATURE_SINK
X_BODY_TOP_Z = X_BODY_BOTTOM_Z + X_BODY_THICKNESS
Y_RAIL_LENGTH = 0.24
Y_RAIL_WIDTH = 0.024
Y_RAIL_HEIGHT = 0.012
Y_RAIL_X = 0.052
Y_RAIL_BOTTOM_Z = X_BODY_TOP_Z - FEATURE_SINK
Y_RAIL_TOP_Z = Y_RAIL_BOTTOM_Z + Y_RAIL_HEIGHT

Y_PAD_WIDTH_X = 0.040
Y_PAD_LENGTH_Y = 0.13
Y_PAD_HEIGHT = 0.018
Y_BODY_LENGTH = 0.16
Y_BODY_WIDTH = 0.15
Y_BODY_THICKNESS = 0.014
Y_BODY_BOTTOM_Z = Y_PAD_HEIGHT - FEATURE_SINK
Y_BODY_TOP_Z = Y_BODY_BOTTOM_Z + Y_BODY_THICKNESS
TOWER_LENGTH = 0.050
TOWER_WIDTH = 0.108
TOWER_HEIGHT = 0.195
TOWER_CENTER_X = 0.005
TOWER_FRONT_X = TOWER_CENTER_X + TOWER_LENGTH / 2.0
Z_RAIL_DEPTH = 0.008
Z_RAIL_WIDTH = 0.018
Z_RAIL_HEIGHT = 0.150
Z_RAIL_Y = 0.034
Z_RAIL_BOTTOM_Z = 0.050
Z_RAIL_BOTTOM_X = TOWER_FRONT_X - FEATURE_SINK
Z_RAIL_FRONT_X = Z_RAIL_BOTTOM_X + Z_RAIL_DEPTH
Z_RAIL_CENTER_X = Z_RAIL_BOTTOM_X + Z_RAIL_DEPTH / 2.0

Z_SHOE_DEPTH = 0.018
Z_SHOE_WIDTH = 0.030
Z_SHOE_HEIGHT = 0.055
Z_BODY_LENGTH = 0.058
Z_BODY_WIDTH = 0.100
Z_BODY_HEIGHT = 0.078
Z_BODY_START_X = 0.016
Z_BODY_START_Z = 0.008
TOOL_PLATE_LENGTH = 0.118
TOOL_PLATE_WIDTH = 0.094
TOOL_PLATE_THICKNESS = 0.012
TOOL_PLATE_START_X = 0.002
TOOL_PLATE_BOTTOM_Z = Z_BODY_START_Z + Z_BODY_HEIGHT - FEATURE_SINK

X_TRAVEL = 0.10
Y_TRAVEL = 0.06
Z_TRAVEL = 0.095


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _set_proxy_inertial(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    mass: float,
):
    part.inertial = Inertial.from_geometry(Box(size), mass=mass, origin=Origin(xyz=center))


def _base_body_shape():
    deck = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_DECK_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_SKIRT_HEIGHT))
    )
    left_skirt = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.92, 0.036, BASE_SKIRT_HEIGHT, centered=(True, True, False))
        .translate((0.0, BASE_WIDTH * 0.36, 0.0))
    )
    right_skirt = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.92, 0.036, BASE_SKIRT_HEIGHT, centered=(True, True, False))
        .translate((0.0, -BASE_WIDTH * 0.36, 0.0))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.22, BASE_WIDTH * 0.62, BASE_SKIRT_HEIGHT * 0.85, centered=(True, True, False))
        .translate((-BASE_LENGTH * 0.24, 0.0, 0.0))
    )
    center_rib = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.40, 0.055, BASE_SKIRT_HEIGHT * 0.55, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )

    body = deck.union(left_skirt).union(right_skirt).union(rear_rib).union(center_rib)
    pocket = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.42, BASE_WIDTH * 0.14, BASE_DECK_THICKNESS * 0.72, centered=(True, True, False))
        .translate((0.08, 0.0, BASE_TOP_Z - BASE_DECK_THICKNESS * 0.42))
    )
    body = body.cut(pocket)
    return body


def _x_carriage_body_shape():
    bridge = (
        cq.Workplane("XY")
        .box(X_BODY_LENGTH, X_BODY_WIDTH, X_BODY_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, X_BODY_BOTTOM_Z))
    )
    front_rib = (
        cq.Workplane("XY")
        .box(X_BODY_LENGTH * 0.78, 0.022, 0.030, centered=(True, True, False))
        .translate((0.0, X_BODY_WIDTH * 0.36, X_BODY_BOTTOM_Z))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(X_BODY_LENGTH * 0.78, 0.022, 0.030, centered=(True, True, False))
        .translate((0.0, -X_BODY_WIDTH * 0.36, X_BODY_BOTTOM_Z))
    )
    center_spine = (
        cq.Workplane("XY")
        .box(0.11, 0.050, 0.024, centered=(True, True, False))
        .translate((0.0, 0.0, X_BODY_BOTTOM_Z + 0.004))
    )
    body = bridge.union(front_rib).union(rear_rib).union(center_spine)
    lightening = (
        cq.Workplane("XY")
        .box(0.090, 0.110, 0.013, centered=(True, True, False))
        .translate((0.0, 0.0, X_BODY_BOTTOM_Z + 0.004))
    )
    body = body.cut(lightening)
    return body


def _y_carriage_body_shape():
    saddle = (
        cq.Workplane("XY")
        .box(Y_BODY_LENGTH, Y_BODY_WIDTH, Y_BODY_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, Y_BODY_BOTTOM_Z))
    )
    tower = (
        cq.Workplane("XY")
        .box(TOWER_LENGTH, TOWER_WIDTH, TOWER_HEIGHT, centered=(True, True, False))
        .translate((TOWER_CENTER_X, 0.0, Y_BODY_TOP_Z - FEATURE_SINK))
    )
    left_buttress = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.082, centered=(True, True, False))
        .translate((0.020, 0.040, Y_BODY_TOP_Z - FEATURE_SINK))
    )
    right_buttress = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.082, centered=(True, True, False))
        .translate((0.020, -0.040, Y_BODY_TOP_Z - FEATURE_SINK))
    )
    body = saddle.union(tower).union(left_buttress).union(right_buttress)
    tower_window = (
        cq.Workplane("XY")
        .box(0.028, 0.052, 0.100, centered=(False, True, False))
        .translate((0.008, 0.0, Y_BODY_TOP_Z + 0.050))
    )
    body = body.cut(tower_window)
    return body


def _z_slide_body_shape():
    core = (
        cq.Workplane("XY")
        .box(Z_BODY_LENGTH, Z_BODY_WIDTH, Z_BODY_HEIGHT, centered=(False, True, False))
        .translate((Z_BODY_START_X, 0.0, Z_BODY_START_Z))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.020, 0.058, 0.040, centered=(False, True, False))
        .translate((Z_BODY_START_X + Z_BODY_LENGTH - 0.004, 0.0, 0.018))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(0.034, 0.076, 0.018, centered=(False, True, False))
        .translate((0.014, 0.0, 0.060))
    )
    body = core.union(nose).union(upper_web)
    front_pocket = (
        cq.Workplane("XY")
        .box(0.016, 0.042, 0.030, centered=(False, True, False))
        .translate((Z_BODY_START_X + Z_BODY_LENGTH + 0.006, 0.0, 0.026))
    )
    body = body.cut(front_pocket)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    model.material("base_graphite", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("anodized_black", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.60, 0.65, 1.0))
    model.material("tool_blue", rgba=(0.22, 0.40, 0.72, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_body_shape(), "base_body"), material="base_graphite", name="base_body")
    _add_box_visual(
        base,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, X_RAIL_Y, X_RAIL_BOTTOM_Z + X_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "x_rail_left",
    )
    _add_box_visual(
        base,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, -X_RAIL_Y, X_RAIL_BOTTOM_Z + X_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "x_rail_right",
    )
    _set_proxy_inertial(base, (BASE_LENGTH, BASE_WIDTH, X_RAIL_TOP_Z), (0.0, 0.0, X_RAIL_TOP_Z / 2.0), 11.0)

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_body_shape(), "x_carriage_body"),
        material="machined_aluminum",
        name="x_carriage_body",
    )
    _add_box_visual(
        x_carriage,
        (X_PAD_LENGTH, X_PAD_WIDTH, X_PAD_HEIGHT),
        (0.0, X_RAIL_Y, X_PAD_HEIGHT / 2.0),
        "rail_steel",
        "x_pad_left",
    )
    _add_box_visual(
        x_carriage,
        (X_PAD_LENGTH, X_PAD_WIDTH, X_PAD_HEIGHT),
        (0.0, -X_RAIL_Y, X_PAD_HEIGHT / 2.0),
        "rail_steel",
        "x_pad_right",
    )
    _add_box_visual(
        x_carriage,
        (Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT),
        (Y_RAIL_X, 0.0, Y_RAIL_BOTTOM_Z + Y_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "y_rail_left",
    )
    _add_box_visual(
        x_carriage,
        (Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT),
        (-Y_RAIL_X, 0.0, Y_RAIL_BOTTOM_Z + Y_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "y_rail_right",
    )
    _set_proxy_inertial(x_carriage, (X_BODY_LENGTH, X_BODY_WIDTH, Y_RAIL_TOP_Z), (0.0, 0.0, Y_RAIL_TOP_Z / 2.0), 3.6)

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_body_shape(), "y_carriage_body"),
        material="anodized_black",
        name="y_carriage_body",
    )
    _add_box_visual(
        y_carriage,
        (Y_PAD_WIDTH_X, Y_PAD_LENGTH_Y, Y_PAD_HEIGHT),
        (Y_RAIL_X, 0.0, Y_PAD_HEIGHT / 2.0),
        "rail_steel",
        "y_pad_left",
    )
    _add_box_visual(
        y_carriage,
        (Y_PAD_WIDTH_X, Y_PAD_LENGTH_Y, Y_PAD_HEIGHT),
        (-Y_RAIL_X, 0.0, Y_PAD_HEIGHT / 2.0),
        "rail_steel",
        "y_pad_right",
    )
    _add_box_visual(
        y_carriage,
        (Z_RAIL_DEPTH, Z_RAIL_WIDTH, Z_RAIL_HEIGHT),
        (Z_RAIL_CENTER_X, Z_RAIL_Y, Z_RAIL_BOTTOM_Z + Z_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "z_rail_left",
    )
    _add_box_visual(
        y_carriage,
        (Z_RAIL_DEPTH, Z_RAIL_WIDTH, Z_RAIL_HEIGHT),
        (Z_RAIL_CENTER_X, -Z_RAIL_Y, Z_RAIL_BOTTOM_Z + Z_RAIL_HEIGHT / 2.0),
        "rail_steel",
        "z_rail_right",
    )
    _set_proxy_inertial(y_carriage, (0.17, 0.16, 0.23), (0.01, 0.0, 0.115), 2.8)

    z_slide = model.part("z_slide")
    z_slide.visual(
        mesh_from_cadquery(_z_slide_body_shape(), "z_slide_body"),
        material="machined_aluminum",
        name="z_slide_body",
    )
    _add_box_visual(
        z_slide,
        (Z_SHOE_DEPTH, Z_SHOE_WIDTH, Z_SHOE_HEIGHT),
        (Z_SHOE_DEPTH / 2.0, Z_RAIL_Y, Z_SHOE_HEIGHT / 2.0),
        "rail_steel",
        "z_shoe_left",
    )
    _add_box_visual(
        z_slide,
        (Z_SHOE_DEPTH, Z_SHOE_WIDTH, Z_SHOE_HEIGHT),
        (Z_SHOE_DEPTH / 2.0, -Z_RAIL_Y, Z_SHOE_HEIGHT / 2.0),
        "rail_steel",
        "z_shoe_right",
    )
    _add_box_visual(
        z_slide,
        (TOOL_PLATE_LENGTH, TOOL_PLATE_WIDTH, TOOL_PLATE_THICKNESS),
        (
            TOOL_PLATE_START_X + TOOL_PLATE_LENGTH / 2.0,
            0.0,
            TOOL_PLATE_BOTTOM_Z + TOOL_PLATE_THICKNESS / 2.0,
        ),
        "tool_blue",
        "tool_plate",
    )
    _set_proxy_inertial(z_slide, (0.12, 0.10, 0.10), (0.05, 0.0, 0.05), 1.6)

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-X_TRAVEL, upper=X_TRAVEL, effort=180.0, velocity=0.35),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_TRAVEL, upper=Y_TRAVEL, effort=140.0, velocity=0.28),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(Z_RAIL_FRONT_X, 0.0, Z_RAIL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=100.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")
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

    ctx.expect_contact(x_carriage, base, elem_a="x_pad_left", elem_b="x_rail_left", contact_tol=0.0015)
    ctx.expect_contact(x_carriage, base, elem_a="x_pad_right", elem_b="x_rail_right", contact_tol=0.0015)
    ctx.expect_overlap(x_carriage, base, axes="xy", elem_a="x_pad_left", elem_b="x_rail_left", min_overlap=0.03)
    ctx.expect_overlap(x_carriage, base, axes="xy", elem_a="x_pad_right", elem_b="x_rail_right", min_overlap=0.03)

    ctx.expect_contact(y_carriage, x_carriage, elem_a="y_pad_left", elem_b="y_rail_left", contact_tol=0.0015)
    ctx.expect_contact(y_carriage, x_carriage, elem_a="y_pad_right", elem_b="y_rail_right", contact_tol=0.0015)
    ctx.expect_overlap(y_carriage, x_carriage, axes="xy", elem_a="y_pad_left", elem_b="y_rail_left", min_overlap=0.02)
    ctx.expect_overlap(y_carriage, x_carriage, axes="xy", elem_a="y_pad_right", elem_b="y_rail_right", min_overlap=0.02)

    ctx.expect_contact(z_slide, y_carriage, elem_a="z_shoe_left", elem_b="z_rail_left", contact_tol=0.0015)
    ctx.expect_contact(z_slide, y_carriage, elem_a="z_shoe_right", elem_b="z_rail_right", contact_tol=0.0015)
    ctx.expect_overlap(z_slide, y_carriage, axes="yz", elem_a="z_shoe_left", elem_b="z_rail_left", min_overlap=0.015)
    ctx.expect_overlap(z_slide, y_carriage, axes="yz", elem_a="z_shoe_right", elem_b="z_rail_right", min_overlap=0.015)

    def _axis_delta(part, joint, q: float, axis_index: int) -> float | None:
        with ctx.pose({joint: 0.0}):
            home = ctx.part_world_position(part)
        with ctx.pose({joint: q}):
            moved = ctx.part_world_position(part)
        if home is None or moved is None:
            return None
        return moved[axis_index] - home[axis_index]

    x_delta = _axis_delta(x_carriage, x_joint, 0.08, 0)
    y_delta = _axis_delta(y_carriage, y_joint, 0.04, 1)
    z_delta = _axis_delta(z_slide, z_joint, 0.05, 2)

    ctx.check(
        "x_joint_moves_along_positive_x",
        x_delta is not None and abs(x_delta - 0.08) <= 1e-6,
        f"expected +0.08 m X motion, got {x_delta!r}",
    )
    ctx.check(
        "y_joint_moves_along_positive_y",
        y_delta is not None and abs(y_delta - 0.04) <= 1e-6,
        f"expected +0.04 m Y motion, got {y_delta!r}",
    )
    ctx.check(
        "z_joint_moves_along_positive_z",
        z_delta is not None and abs(z_delta - 0.05) <= 1e-6,
        f"expected +0.05 m Z motion, got {z_delta!r}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
