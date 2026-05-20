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


BASE_LENGTH = 0.68
BASE_WIDTH = 0.24
FOOT_BAR_LENGTH = 0.56
FOOT_BAR_WIDTH = 0.034
FOOT_BAR_HEIGHT = 0.012
SLAB_THICKNESS = 0.018
SLAB_Z0 = FOOT_BAR_HEIGHT

RAIL_LENGTH = 0.58
RAIL_BASE_WIDTH = 0.022
RAIL_CROWN_WIDTH = 0.014
RAIL_BASE_HEIGHT = 0.010
RAIL_CROWN_HEIGHT = 0.014
RAIL_CENTER_Y = 0.070
RAIL_Z0 = FOOT_BAR_HEIGHT + SLAB_THICKNESS
RAIL_TOP_Z = RAIL_Z0 + RAIL_BASE_HEIGHT + RAIL_CROWN_HEIGHT

X_TRAVEL_LOWER = -0.135
X_TRAVEL_UPPER = 0.135
X_TRUCK_LENGTH = 0.068
X_TRUCK_WIDTH = 0.046
X_TRUCK_HEIGHT = 0.026
X_TRUCK_CENTER_X = 0.096
X_TRUCK_CENTER_Y = RAIL_CENTER_Y
X_BRIDGE_LENGTH = 0.30
X_BRIDGE_WIDTH = 0.20
X_BRIDGE_THICKNESS = 0.016
X_BRIDGE_Z0 = X_TRUCK_HEIGHT
Y_BASE_X = 0.16
Y_BASE_Y = 0.18
Y_BASE_THICKNESS = 0.022
Y_BASE_Z0 = X_BRIDGE_Z0 + X_BRIDGE_THICKNESS
Y_GUIDE_X = 0.040
Y_GUIDE_WIDTH = 0.024
Y_GUIDE_LENGTH = 0.18
Y_GUIDE_HEIGHT = 0.012
Y_GUIDE_Z0 = Y_BASE_Z0 + Y_BASE_THICKNESS
Y_GUIDE_TOP_Z = Y_GUIDE_Z0 + Y_GUIDE_HEIGHT

Y_TRAVEL_LOWER = -0.045
Y_TRAVEL_UPPER = 0.045
Y_SHOE_X = Y_GUIDE_X
Y_SHOE_WIDTH = 0.028
Y_SHOE_LENGTH = 0.080
Y_SHOE_HEIGHT = 0.014
Y_BRIDGE_X = 0.110
Y_BRIDGE_Y = 0.100
Y_BRIDGE_THICKNESS = 0.014
Y_BRIDGE_Z0 = Y_SHOE_HEIGHT
Y_PEDESTAL_X = 0.080
Y_PEDESTAL_Y = 0.070
Y_PEDESTAL_THICKNESS = 0.020
Y_PEDESTAL_Z0 = Y_BRIDGE_Z0 + Y_BRIDGE_THICKNESS
TOP_PAD_X = 0.140
TOP_PAD_Y = 0.140
TOP_PAD_THICKNESS = 0.010
TOP_PAD_Z0 = Y_PEDESTAL_Z0 + Y_PEDESTAL_THICKNESS


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z0))
    )


def _base_shape() -> cq.Workplane:
    left_foot = _box(
        FOOT_BAR_LENGTH,
        FOOT_BAR_WIDTH,
        FOOT_BAR_HEIGHT,
        y=-0.082,
        z0=0.0,
    )
    right_foot = _box(
        FOOT_BAR_LENGTH,
        FOOT_BAR_WIDTH,
        FOOT_BAR_HEIGHT,
        y=0.082,
        z0=0.0,
    )
    slab = _box(BASE_LENGTH, BASE_WIDTH, SLAB_THICKNESS, z0=SLAB_Z0)

    rail_base_left = _box(
        RAIL_LENGTH,
        RAIL_BASE_WIDTH,
        RAIL_BASE_HEIGHT,
        y=-RAIL_CENTER_Y,
        z0=RAIL_Z0,
    )
    rail_base_right = _box(
        RAIL_LENGTH,
        RAIL_BASE_WIDTH,
        RAIL_BASE_HEIGHT,
        y=RAIL_CENTER_Y,
        z0=RAIL_Z0,
    )
    rail_crown_left = _box(
        RAIL_LENGTH,
        RAIL_CROWN_WIDTH,
        RAIL_CROWN_HEIGHT,
        y=-RAIL_CENTER_Y,
        z0=RAIL_Z0 + RAIL_BASE_HEIGHT,
    )
    rail_crown_right = _box(
        RAIL_LENGTH,
        RAIL_CROWN_WIDTH,
        RAIL_CROWN_HEIGHT,
        y=RAIL_CENTER_Y,
        z0=RAIL_Z0 + RAIL_BASE_HEIGHT,
    )

    nose_left = _box(0.045, 0.050, 0.016, x=-0.285, y=-RAIL_CENTER_Y, z0=RAIL_Z0)
    nose_right = _box(0.045, 0.050, 0.016, x=-0.285, y=RAIL_CENTER_Y, z0=RAIL_Z0)
    tail_left = _box(0.045, 0.050, 0.016, x=0.285, y=-RAIL_CENTER_Y, z0=RAIL_Z0)
    tail_right = _box(0.045, 0.050, 0.016, x=0.285, y=RAIL_CENTER_Y, z0=RAIL_Z0)

    base = left_foot.union(right_foot)
    for piece in (
        slab,
        rail_base_left,
        rail_base_right,
        rail_crown_left,
        rail_crown_right,
        nose_left,
        nose_right,
        tail_left,
        tail_right,
    ):
        base = base.union(piece)
    return base


def _x_carriage_shape() -> cq.Workplane:
    truck_positions = (
        (-X_TRUCK_CENTER_X, -X_TRUCK_CENTER_Y),
        (-X_TRUCK_CENTER_X, X_TRUCK_CENTER_Y),
        (X_TRUCK_CENTER_X, -X_TRUCK_CENTER_Y),
        (X_TRUCK_CENTER_X, X_TRUCK_CENTER_Y),
    )
    shape = _box(
        X_TRUCK_LENGTH,
        X_TRUCK_WIDTH,
        X_TRUCK_HEIGHT,
        x=truck_positions[0][0],
        y=truck_positions[0][1],
        z0=0.0,
    )
    for x_pos, y_pos in truck_positions[1:]:
        shape = shape.union(
            _box(
                X_TRUCK_LENGTH,
                X_TRUCK_WIDTH,
                X_TRUCK_HEIGHT,
                x=x_pos,
                y=y_pos,
                z0=0.0,
            )
        )

    bridge = _box(X_BRIDGE_LENGTH, X_BRIDGE_WIDTH, X_BRIDGE_THICKNESS, z0=X_BRIDGE_Z0)
    y_stage_base = _box(Y_BASE_X, Y_BASE_Y, Y_BASE_THICKNESS, z0=Y_BASE_Z0)
    guide_left = _box(
        Y_GUIDE_WIDTH,
        Y_GUIDE_LENGTH,
        Y_GUIDE_HEIGHT,
        x=-Y_GUIDE_X,
        z0=Y_GUIDE_Z0,
    )
    guide_right = _box(
        Y_GUIDE_WIDTH,
        Y_GUIDE_LENGTH,
        Y_GUIDE_HEIGHT,
        x=Y_GUIDE_X,
        z0=Y_GUIDE_Z0,
    )
    end_rib_front = _box(0.125, 0.022, 0.020, y=-0.079, z0=Y_BASE_Z0)
    end_rib_back = _box(0.125, 0.022, 0.020, y=0.079, z0=Y_BASE_Z0)

    for piece in (bridge, y_stage_base, guide_left, guide_right, end_rib_front, end_rib_back):
        shape = shape.union(piece)
    return shape


def _y_slide_shape() -> cq.Workplane:
    left_shoe = _box(
        Y_SHOE_WIDTH,
        Y_SHOE_LENGTH,
        Y_SHOE_HEIGHT,
        x=-Y_SHOE_X,
        z0=0.0,
    )
    right_shoe = _box(
        Y_SHOE_WIDTH,
        Y_SHOE_LENGTH,
        Y_SHOE_HEIGHT,
        x=Y_SHOE_X,
        z0=0.0,
    )
    bridge = _box(Y_BRIDGE_X, Y_BRIDGE_Y, Y_BRIDGE_THICKNESS, z0=Y_BRIDGE_Z0)
    pedestal = _box(Y_PEDESTAL_X, Y_PEDESTAL_Y, Y_PEDESTAL_THICKNESS, z0=Y_PEDESTAL_Z0)
    top_pad = _box(TOP_PAD_X, TOP_PAD_Y, TOP_PAD_THICKNESS, z0=TOP_PAD_Z0)

    slide = left_shoe.union(right_shoe)
    for piece in (bridge, pedestal, top_pad):
        slide = slide.union(piece)
    return slide


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xy_service_fixture_stage")

    model.material("base_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("machined_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("anodized_silver", rgba=(0.79, 0.81, 0.84, 1.0))

    base = model.part("base_rail_set")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_rail_set"),
        material="base_gray",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, RAIL_TOP_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_shape(), "x_carriage"),
        material="machined_steel",
        name="x_carriage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_BRIDGE_LENGTH, X_BRIDGE_WIDTH, Y_GUIDE_TOP_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, Y_GUIDE_TOP_Z / 2.0)),
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_y_slide_shape(), "y_slide"),
        material="anodized_silver",
        name="y_slide_body",
    )
    y_slide.inertial = Inertial.from_geometry(
        Box((TOP_PAD_X, TOP_PAD_Y, TOP_PAD_Z0 + TOP_PAD_THICKNESS)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, (TOP_PAD_Z0 + TOP_PAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.35,
            lower=X_TRAVEL_LOWER,
            upper=X_TRAVEL_UPPER,
        ),
    )
    model.articulation(
        "x_carriage_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, Y_GUIDE_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.25,
            lower=Y_TRAVEL_LOWER,
            upper=Y_TRAVEL_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail_set")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    x_joint = object_model.get_articulation("base_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_slide")

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
        "x prismatic axis is stage longitudinal",
        tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {x_joint.axis}",
    )
    ctx.check(
        "y prismatic axis is cross-slide lateral",
        tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {y_joint.axis}",
    )

    with ctx.pose({x_joint: 0.0, y_joint: 0.0}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="x carriage seats on base rails",
        )
        ctx.expect_gap(
            y_slide,
            x_carriage,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="y slide seats on carriage guides",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="xy",
            min_overlap=0.12,
            name="x carriage overlaps base rail footprint",
        )
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="xy",
            min_overlap=0.06,
            name="y slide overlaps carriage guide footprint",
        )

    with ctx.pose({x_joint: X_TRAVEL_UPPER, y_joint: Y_TRAVEL_LOWER}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="x carriage stays seated at combined travel",
        )
        ctx.expect_gap(
            y_slide,
            x_carriage,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="y slide stays seated at combined travel",
        )

    with ctx.pose({x_joint: 0.0, y_joint: 0.0}):
        x_home = ctx.part_world_position(x_carriage)
        y_home = ctx.part_world_position(y_slide)
    with ctx.pose({x_joint: X_TRAVEL_UPPER, y_joint: 0.0}):
        x_forward = ctx.part_world_position(x_carriage)
    with ctx.pose({x_joint: 0.0, y_joint: Y_TRAVEL_UPPER}):
        y_forward = ctx.part_world_position(y_slide)

    ctx.check(
        "positive x travel moves carriage in +x",
        x_home is not None and x_forward is not None and x_forward[0] > x_home[0] + 0.10,
        details=f"home={x_home}, forward={x_forward}",
    )
    ctx.check(
        "positive y travel moves top slide in +y",
        y_home is not None and y_forward is not None and y_forward[1] > y_home[1] + 0.03,
        details=f"home={y_home}, forward={y_forward}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
