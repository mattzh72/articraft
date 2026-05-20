from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.42
BASE_WIDTH = 0.22
BASE_THICK = 0.02

X_RAIL_SEP = 0.116
X_RAIL_LEN = 0.34
X_RAIL_WIDTH = 0.024
X_RAIL_HEIGHT = 0.016
X_RAIL_Z = -0.008

X_BEARING_X = 0.032
X_BEARING_LEN = 0.044
X_BEARING_WIDTH = 0.030
X_BEARING_HEIGHT = 0.022
X_RIB_LEN = 0.108
X_RIB_WIDTH = 0.018
X_RIB_HEIGHT = 0.020
X_TOP_LEN = 0.14
X_TOP_WIDTH = 0.18
X_TOP_THICK = 0.014
X_TOP_Z = 0.035

Y_RAIL_SEP = 0.080
Y_RAIL_LEN = 0.18
Y_RAIL_WIDTH = 0.018
Y_RAIL_HEIGHT = 0.014
Y_RAIL_Z = 0.049
Y_JOINT_Z = Y_RAIL_Z + Y_RAIL_HEIGHT / 2.0

Y_BEARING_Y = 0.024
Y_BEARING_LEN = 0.028
Y_BEARING_WIDTH = 0.034
Y_BEARING_HEIGHT = 0.020
Y_BODY_LEN = 0.12
Y_BODY_WIDTH = 0.070
Y_BODY_THICK = 0.014
Y_BODY_Y = -0.015
Y_BODY_Z = 0.031
UPRIGHT_WIDTH = 0.10
UPRIGHT_THICK = 0.012
UPRIGHT_HEIGHT = 0.16
UPRIGHT_Z = 0.10

Z_RAIL_SEP = 0.044
Z_RAIL_WIDTH = 0.016
Z_RAIL_DEPTH = 0.020
Z_RAIL_HEIGHT = 0.12
Z_RAIL_Y = 0.016
Z_RAIL_Z = 0.09
Z_JOINT_Y = Z_RAIL_Y + Z_RAIL_DEPTH / 2.0
Z_JOINT_Z = Z_RAIL_Z

Z_SLIDE_WIDTH = 0.074
Z_SLIDE_DEPTH = 0.028
Z_SLIDE_HEIGHT = 0.060
Z_PAD_WIDTH = 0.024
Z_PAD_DEPTH = 0.018
Z_PAD_HEIGHT = 0.060
TOP_PLATE_LEN = 0.11
TOP_PLATE_WIDTH = 0.09
TOP_PLATE_THICK = 0.010
TOP_PLATE_Y = 0.045
TOP_PLATE_Z = 0.035

X_TRAVEL = 0.10
Y_TRAVEL = 0.08
Z_TRAVEL = 0.08


def _box(x: float, y: float, z: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z).translate(center_xyz)


def _rail_x(length: float, width: float, height: float, *, y: float, z: float) -> cq.Workplane:
    return _box(length, width, height, (0.0, y, z))


def _rail_y(length: float, width: float, height: float, *, x: float, z: float) -> cq.Workplane:
    return _box(width, length, height, (x, 0.0, z))


def _rail_z(width: float, depth: float, height: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return _box(width, depth, height, (x, y, z))


def _make_base_shape() -> cq.Workplane:
    plate = _box(BASE_LEN, BASE_WIDTH, BASE_THICK, (0.0, 0.0, -0.035))
    left_rail = _rail_x(
        X_RAIL_LEN,
        X_RAIL_WIDTH,
        X_RAIL_HEIGHT,
        y=-X_RAIL_SEP / 2.0,
        z=X_RAIL_Z,
    )
    right_rail = _rail_x(
        X_RAIL_LEN,
        X_RAIL_WIDTH,
        X_RAIL_HEIGHT,
        y=X_RAIL_SEP / 2.0,
        z=X_RAIL_Z,
    )
    rail_feet = (
        _box(0.050, BASE_WIDTH - 0.020, 0.014, (-0.150, 0.0, -0.018))
        .union(_box(0.050, BASE_WIDTH - 0.020, 0.014, (0.150, 0.0, -0.018)))
    )
    base = plate.union(left_rail).union(right_rail).union(rail_feet)
    return base


def _make_x_carriage_shape() -> cq.Workplane:
    shape = _box(
        X_BEARING_LEN,
        X_BEARING_WIDTH,
        X_BEARING_HEIGHT,
        (-X_BEARING_X, -X_RAIL_SEP / 2.0, X_BEARING_HEIGHT / 2.0),
    )
    for x in (-X_BEARING_X, X_BEARING_X):
        for y in (-X_RAIL_SEP / 2.0, X_RAIL_SEP / 2.0):
            if x == -X_BEARING_X and y == -X_RAIL_SEP / 2.0:
                continue
            shape = shape.union(
                _box(
                    X_BEARING_LEN,
                    X_BEARING_WIDTH,
                    X_BEARING_HEIGHT,
                    (x, y, X_BEARING_HEIGHT / 2.0),
                )
            )
    left_rib = _box(
        X_RIB_LEN,
        X_RIB_WIDTH,
        X_RIB_HEIGHT,
        (0.0, -X_RAIL_SEP / 2.0, 0.022),
    )
    right_rib = _box(
        X_RIB_LEN,
        X_RIB_WIDTH,
        X_RIB_HEIGHT,
        (0.0, X_RAIL_SEP / 2.0, 0.022),
    )
    deck = _box(X_TOP_LEN, X_TOP_WIDTH, X_TOP_THICK, (0.0, 0.0, X_TOP_Z))
    left_y_rail = _rail_y(
        Y_RAIL_LEN,
        Y_RAIL_WIDTH,
        Y_RAIL_HEIGHT,
        x=-Y_RAIL_SEP / 2.0,
        z=Y_RAIL_Z,
    )
    right_y_rail = _rail_y(
        Y_RAIL_LEN,
        Y_RAIL_WIDTH,
        Y_RAIL_HEIGHT,
        x=Y_RAIL_SEP / 2.0,
        z=Y_RAIL_Z,
    )
    shape = shape.union(left_rib).union(right_rib).union(deck).union(left_y_rail).union(right_y_rail)
    return shape


def _make_y_carriage_shape() -> cq.Workplane:
    shape = _box(
        Y_BEARING_LEN,
        Y_BEARING_WIDTH,
        Y_BEARING_HEIGHT,
        (-Y_RAIL_SEP / 2.0, -Y_BEARING_Y, Y_BEARING_HEIGHT / 2.0),
    )
    for x in (-Y_RAIL_SEP / 2.0, Y_RAIL_SEP / 2.0):
        for y in (-Y_BEARING_Y, Y_BEARING_Y):
            if x == -Y_RAIL_SEP / 2.0 and y == -Y_BEARING_Y:
                continue
            shape = shape.union(
                _box(
                    Y_BEARING_LEN,
                    Y_BEARING_WIDTH,
                    Y_BEARING_HEIGHT,
                    (x, y, Y_BEARING_HEIGHT / 2.0),
                )
            )
    bearing_bridge = _box(0.108, 0.082, 0.008, (0.0, 0.0, 0.018))
    body = _box(Y_BODY_LEN, Y_BODY_WIDTH, Y_BODY_THICK, (0.0, Y_BODY_Y, Y_BODY_Z))
    spine = _box(0.050, 0.024, 0.058, (0.0, -0.008, 0.045))
    upright = _box(UPRIGHT_WIDTH, UPRIGHT_THICK, UPRIGHT_HEIGHT, (0.0, 0.0, UPRIGHT_Z))
    left_z_rail = _rail_z(
        Z_RAIL_WIDTH,
        Z_RAIL_DEPTH,
        Z_RAIL_HEIGHT,
        x=-Z_RAIL_SEP / 2.0,
        y=Z_RAIL_Y,
        z=Z_RAIL_Z,
    )
    right_z_rail = _rail_z(
        Z_RAIL_WIDTH,
        Z_RAIL_DEPTH,
        Z_RAIL_HEIGHT,
        x=Z_RAIL_SEP / 2.0,
        y=Z_RAIL_Y,
        z=Z_RAIL_Z,
    )
    shape = (
        shape.union(bearing_bridge)
        .union(body)
        .union(spine)
        .union(upright)
        .union(left_z_rail)
        .union(right_z_rail)
    )
    return shape


def _make_z_slide_shape() -> cq.Workplane:
    left_pad = _box(
        Z_PAD_WIDTH,
        Z_PAD_DEPTH,
        Z_PAD_HEIGHT,
        (-Z_RAIL_SEP / 2.0, Z_PAD_DEPTH / 2.0, 0.0),
    )
    right_pad = _box(
        Z_PAD_WIDTH,
        Z_PAD_DEPTH,
        Z_PAD_HEIGHT,
        (Z_RAIL_SEP / 2.0, Z_PAD_DEPTH / 2.0, 0.0),
    )
    block = _box(Z_SLIDE_WIDTH, Z_SLIDE_DEPTH, Z_SLIDE_HEIGHT, (0.0, 0.032, 0.0))
    top_plate = _box(
        TOP_PLATE_LEN,
        TOP_PLATE_WIDTH,
        TOP_PLATE_THICK,
        (0.0, TOP_PLATE_Y, TOP_PLATE_Z),
    )
    shape = left_pad.union(right_pad).union(block).union(top_plate)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_stack_xyz_stage")

    anodized_black = model.material("anodized_black", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.63, 0.66, 0.69, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base"),
        material=anodized_black,
        name="base_shell",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_make_x_carriage_shape(), "x_carriage"),
        material=satin_gray,
        name="x_carriage_shell",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_make_y_carriage_shape(), "y_carriage"),
        material=bright_steel,
        name="y_carriage_shell",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        mesh_from_cadquery(_make_z_slide_shape(), "z_slide"),
        material=plate_gray,
        name="z_slide_shell",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )

    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.16,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_JOINT_Y, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")

    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("y_to_z")

    ctx.check(
        "x axis is prismatic +X",
        tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected +X axis, got {x_joint.axis}",
    )
    ctx.check(
        "y axis is prismatic +Y",
        tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected +Y axis, got {y_joint.axis}",
    )
    ctx.check(
        "z axis is prismatic +Z",
        tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected +Z axis, got {z_joint.axis}",
    )

    ctx.expect_contact(x_carriage, base, name="x carriage contacts grounded x guides")
    ctx.expect_contact(y_carriage, x_carriage, name="y carriage contacts x carriage guides")
    ctx.expect_contact(z_slide, y_carriage, name="z slide contacts y carriage guides")

    ctx.expect_origin_gap(
        y_carriage,
        x_carriage,
        axis="z",
        min_gap=Y_JOINT_Z - 0.001,
        max_gap=Y_JOINT_Z + 0.001,
        name="y carriage is mounted above x carriage",
    )
    ctx.expect_origin_gap(
        z_slide,
        y_carriage,
        axis="z",
        min_gap=Z_JOINT_Z - 0.001,
        max_gap=Z_JOINT_Z + 0.001,
        name="z slide is mounted above y carriage",
    )

    with ctx.pose(base_to_x=0.08):
        ctx.expect_origin_gap(
            x_carriage,
            base,
            axis="x",
            min_gap=0.079,
            max_gap=0.081,
            name="x carriage moves along +X",
        )

    with ctx.pose(x_to_y=0.06):
        ctx.expect_origin_gap(
            y_carriage,
            x_carriage,
            axis="y",
            min_gap=0.059,
            max_gap=0.061,
            name="y carriage moves along +Y",
        )

    with ctx.pose(y_to_z=0.06):
        ctx.expect_origin_gap(
            z_slide,
            y_carriage,
            axis="z",
            min_gap=Z_JOINT_Z + 0.059,
            max_gap=Z_JOINT_Z + 0.061,
            name="z slide moves along +Z",
        )

    with ctx.pose(base_to_x=X_TRAVEL, x_to_y=Y_TRAVEL, y_to_z=Z_TRAVEL):
        ctx.expect_contact(
            x_carriage,
            base,
            name="x carriage stays supported at full travel",
        )
        ctx.expect_contact(
            y_carriage,
            x_carriage,
            name="y carriage stays supported at full travel",
        )
        ctx.expect_contact(
            z_slide,
            y_carriage,
            name="z slide stays supported at full travel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at combined travel pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
