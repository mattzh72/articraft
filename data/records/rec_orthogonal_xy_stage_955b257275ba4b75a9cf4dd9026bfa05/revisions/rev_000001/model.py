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


BASE_L = 0.320
BASE_W = 0.220
BASE_T = 0.012
BASE_CORNER_R = 0.014
MOUNT_SLOT_L = 0.028
MOUNT_SLOT_W = 0.009
MOUNT_SLOT_X = 0.118
MOUNT_SLOT_Y = 0.078

X_RAIL_L = 0.240
X_RAIL_W = 0.016
X_RAIL_H = 0.008
X_RAIL_SPAN = 0.114

X_STAGE_L = 0.200
X_STAGE_W = 0.140
X_SHOE_L = 0.084
X_SHOE_W = 0.026
X_SHOE_H = 0.006
X_POST_L = 0.028
X_POST_W = 0.018
X_BRIDGE_Z = 0.012
X_BRIDGE_T = 0.008
X_POST_X = 0.050

Y_RAIL_L = 0.132
Y_RAIL_W = 0.014
Y_RAIL_H = 0.006
Y_RAIL_SPAN = 0.086

Y_STAGE_X = 0.138
Y_STAGE_Y = 0.096
Y_SHOE_X = 0.026
Y_SHOE_Y = 0.060
Y_SHOE_H = 0.005
Y_POST_X = 0.016
Y_POST_Y = 0.020
Y_TOP_Z = 0.011
Y_TOP_T = 0.009

X_TRAVEL = 0.050
Y_TRAVEL = 0.018


def _box_on_plane(
    length: float,
    width: float,
    height: float,
    *,
    z0: float = 0.0,
    corner_radius: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(True, True, False),
    )
    if corner_radius > 0.0:
        shape = shape.edges("|Z").fillet(corner_radius)
    return shape.translate((0.0, 0.0, z0))


def _base_shape() -> cq.Workplane:
    plate = _box_on_plane(
        BASE_L,
        BASE_W,
        BASE_T,
        z0=-BASE_T,
        corner_radius=BASE_CORNER_R,
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-MOUNT_SLOT_X, -MOUNT_SLOT_Y),
                (-MOUNT_SLOT_X, MOUNT_SLOT_Y),
                (MOUNT_SLOT_X, -MOUNT_SLOT_Y),
                (MOUNT_SLOT_X, MOUNT_SLOT_Y),
            ]
        )
        .slot2D(MOUNT_SLOT_L, MOUNT_SLOT_W, angle=0.0)
        .cutThruAll()
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .rect(0.168, 0.042)
        .cutBlind(-0.0025)
    )

    rail = _box_on_plane(
        X_RAIL_L,
        X_RAIL_W,
        X_RAIL_H,
        z0=0.0,
        corner_radius=0.0015,
    )
    body = plate
    for y_pos in (-X_RAIL_SPAN / 2.0, X_RAIL_SPAN / 2.0):
        body = body.union(rail.translate((0.0, y_pos, 0.0)))
    return body


def _x_stage_shape() -> cq.Workplane:
    bridge = _box_on_plane(
        X_STAGE_L,
        X_STAGE_W,
        X_BRIDGE_T,
        z0=X_BRIDGE_Z,
        corner_radius=0.006,
    )
    bridge = (
        bridge.faces(">Z")
        .workplane()
        .rect(0.094, 0.050)
        .cutBlind(-0.003)
    )

    shoe = _box_on_plane(
        X_SHOE_L,
        X_SHOE_W,
        X_SHOE_H,
        z0=0.0,
        corner_radius=0.002,
    )
    post = _box_on_plane(
        X_POST_L,
        X_POST_W,
        X_BRIDGE_Z - X_SHOE_H,
        z0=X_SHOE_H,
        corner_radius=0.001,
    )
    y_rail = _box_on_plane(
        Y_RAIL_W,
        Y_RAIL_L,
        Y_RAIL_H,
        z0=X_BRIDGE_Z + X_BRIDGE_T,
        corner_radius=0.001,
    )

    body = bridge
    for y_pos in (-X_RAIL_SPAN / 2.0, X_RAIL_SPAN / 2.0):
        body = body.union(shoe.translate((0.0, y_pos, 0.0)))
        for x_pos in (-X_POST_X, X_POST_X):
            body = body.union(post.translate((x_pos, y_pos, 0.0)))
    for x_pos in (-Y_RAIL_SPAN / 2.0, Y_RAIL_SPAN / 2.0):
        body = body.union(y_rail.translate((x_pos, 0.0, 0.0)))
    return body


def _y_stage_shape() -> cq.Workplane:
    top_plate = _box_on_plane(
        Y_STAGE_X,
        Y_STAGE_Y,
        Y_TOP_T,
        z0=Y_TOP_Z,
        corner_radius=0.005,
    )
    top_plate = (
        top_plate.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.038, -0.024),
                (-0.038, 0.024),
                (0.038, -0.024),
                (0.038, 0.024),
            ]
        )
        .hole(0.005)
    )

    shoe = _box_on_plane(
        Y_SHOE_X,
        Y_SHOE_Y,
        Y_SHOE_H,
        z0=0.0,
        corner_radius=0.0015,
    )
    post = _box_on_plane(
        Y_POST_X,
        Y_POST_X,
        Y_TOP_Z - Y_SHOE_H,
        z0=Y_SHOE_H,
        corner_radius=0.001,
    )

    body = top_plate
    for x_pos in (-Y_RAIL_SPAN / 2.0, Y_RAIL_SPAN / 2.0):
        body = body.union(shoe.translate((x_pos, 0.0, 0.0)))
        for y_pos in (-Y_POST_Y, Y_POST_Y):
            body = body.union(post.translate((x_pos, y_pos, 0.0)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_xy_positioning_module")

    model.material("anodized_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("stage_blue", rgba=(0.45, 0.52, 0.63, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "xy_base"),
        material="anodized_black",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + X_RAIL_H)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, (X_RAIL_H - BASE_T) / 2.0)),
    )

    x_stage = model.part("x_stage")
    x_stage.visual(
        mesh_from_cadquery(_x_stage_shape(), "xy_x_stage"),
        material="machined_gray",
        name="x_stage_body",
    )
    x_stage.inertial = Inertial.from_geometry(
        Box((X_STAGE_L, X_STAGE_W, X_BRIDGE_Z + X_BRIDGE_T + Y_RAIL_H)),
        mass=1.35,
        origin=Origin(
            xyz=(0.0, 0.0, (X_BRIDGE_Z + X_BRIDGE_T + Y_RAIL_H) / 2.0)
        ),
    )

    y_stage = model.part("y_stage")
    y_stage.visual(
        mesh_from_cadquery(_y_stage_shape(), "xy_y_stage"),
        material="stage_blue",
        name="y_stage_body",
    )
    y_stage.inertial = Inertial.from_geometry(
        Box((Y_STAGE_X, Y_STAGE_Y, Y_TOP_Z + Y_TOP_T)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, (Y_TOP_Z + Y_TOP_T) / 2.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_H)),
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
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, X_BRIDGE_Z + X_BRIDGE_T + Y_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")

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
        "x stage axis is horizontal x",
        tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        f"axis={x_joint.axis}",
    )
    ctx.check(
        "y stage axis is horizontal y",
        tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        f"axis={y_joint.axis}",
    )
    ctx.expect_contact(base, x_stage, name="lower slide is supported on base rails")
    ctx.expect_contact(x_stage, y_stage, name="upper cross-slide is supported on x stage")
    ctx.expect_within(
        x_stage,
        base,
        axes="xy",
        margin=0.001,
        name="lower slide footprint stays on mounting plate at rest",
    )
    ctx.expect_within(
        y_stage,
        x_stage,
        axes="xy",
        margin=0.001,
        name="upper slide footprint is smaller than lower slide at rest",
    )

    with ctx.pose({x_joint: X_TRAVEL}):
        ctx.expect_origin_gap(
            x_stage,
            base,
            axis="x",
            min_gap=X_TRAVEL - 0.001,
            max_gap=X_TRAVEL + 0.001,
            name="x stage translates along x",
        )
        ctx.expect_within(
            x_stage,
            base,
            axes="xy",
            margin=0.001,
            name="lower slide remains over plate at x travel limit",
        )

    with ctx.pose({y_joint: Y_TRAVEL}):
        ctx.expect_origin_gap(
            y_stage,
            x_stage,
            axis="y",
            min_gap=Y_TRAVEL - 0.001,
            max_gap=Y_TRAVEL + 0.001,
            name="y stage translates along y",
        )
        ctx.expect_within(
            y_stage,
            x_stage,
            axes="xy",
            margin=0.001,
            name="upper cross-slide remains within lower stage at y travel limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
