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


BASE_LENGTH = 0.40
BASE_WIDTH = 0.19
BASE_THICKNESS = 0.018
WAY_LENGTH = 0.36
WAY_WIDTH = 0.148
WAY_HEIGHT = 0.010
RAIL_LENGTH = 0.35
RAIL_WIDTH = 0.026
RAIL_HEIGHT = 0.006
RAIL_OFFSET_Y = 0.045
GUIDE_TOP_Z = BASE_THICKNESS + WAY_HEIGHT + RAIL_HEIGHT

CARRIAGE_LENGTH = 0.112
CARRIAGE_WIDTH = 0.142
CARRIAGE_BASE_HEIGHT = 0.024
CARRIAGE_CAP_LENGTH = 0.088
CARRIAGE_CAP_WIDTH = 0.104
CARRIAGE_CAP_HEIGHT = 0.016
GUIDE_CHEEK_LENGTH = 0.072
GUIDE_CHEEK_THICKNESS = 0.016
GUIDE_CHEEK_HEIGHT = 0.028
GUIDE_INNER_WIDTH = 0.058
GUIDE_PLATFORM_Z = CARRIAGE_BASE_HEIGHT + CARRIAGE_CAP_HEIGHT
CARRIAGE_HEIGHT = GUIDE_PLATFORM_Z + GUIDE_CHEEK_HEIGHT

Z_SHOE_LENGTH = 0.068
Z_SHOE_WIDTH = GUIDE_INNER_WIDTH
Z_SHOE_HEIGHT = 0.020
Z_COLUMN_LENGTH = 0.056
Z_COLUMN_WIDTH = 0.046
Z_COLUMN_HEIGHT = 0.108
TOOL_FACE_LENGTH = 0.106
TOOL_FACE_WIDTH = 0.082
TOOL_FACE_THICKNESS = 0.012
Z_STAGE_HEIGHT = Z_SHOE_HEIGHT + Z_COLUMN_HEIGHT + TOOL_FACE_THICKNESS

X_TRAVEL = 0.10
Z_TRAVEL = 0.12


def _soft_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _base_guideway_shape() -> cq.Workplane:
    plate = _soft_box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, 0.003)
    way = _soft_box(WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT, 0.002).translate((0.0, 0.0, BASE_THICKNESS))
    rails = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -RAIL_OFFSET_Y), (0.0, RAIL_OFFSET_Y)])
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS + WAY_HEIGHT))
    )
    end_pads = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.155, -0.060),
                (-0.155, 0.060),
                (0.155, -0.060),
                (0.155, 0.060),
            ]
        )
        .box(0.024, 0.030, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    return _fuse_all([plate, way, rails, end_pads]).translate((0.0, 0.0, -GUIDE_TOP_Z))


def _x_carriage_shape() -> cq.Workplane:
    lower = _soft_box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BASE_HEIGHT, 0.0025)
    cap = _soft_box(CARRIAGE_CAP_LENGTH, CARRIAGE_CAP_WIDTH, CARRIAGE_CAP_HEIGHT, 0.002).translate(
        (0.0, 0.0, CARRIAGE_BASE_HEIGHT)
    )
    cheek_y = (GUIDE_INNER_WIDTH / 2.0) + (GUIDE_CHEEK_THICKNESS / 2.0)
    cheeks = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -cheek_y), (0.0, cheek_y)])
        .box(
            GUIDE_CHEEK_LENGTH,
            GUIDE_CHEEK_THICKNESS,
            GUIDE_CHEEK_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, GUIDE_PLATFORM_Z))
    )
    rear_boss = _soft_box(0.034, GUIDE_INNER_WIDTH + (2.0 * GUIDE_CHEEK_THICKNESS), 0.016, 0.0015).translate(
        (-0.018, 0.0, GUIDE_PLATFORM_Z)
    )
    return _fuse_all([lower, cap, cheeks, rear_boss])


def _z_stage_shape() -> cq.Workplane:
    shoe = _soft_box(Z_SHOE_LENGTH, Z_SHOE_WIDTH, Z_SHOE_HEIGHT, 0.0015)
    column = _soft_box(Z_COLUMN_LENGTH, Z_COLUMN_WIDTH, Z_COLUMN_HEIGHT, 0.0018).translate(
        (0.0, 0.0, Z_SHOE_HEIGHT)
    )
    tool_face = (
        cq.Workplane("XY")
        .box(
            TOOL_FACE_LENGTH,
            TOOL_FACE_WIDTH,
            TOOL_FACE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, Z_SHOE_HEIGHT + Z_COLUMN_HEIGHT))
    )
    tool_face = (
        tool_face.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.032, -0.022),
                (-0.032, 0.022),
                (0.032, -0.022),
                (0.032, 0.022),
            ]
        )
        .hole(0.008, depth=0.006)
    )
    return _fuse_all([shoe, column, tool_face])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_slide_stack")

    model.material("base_steel", rgba=(0.24, 0.27, 0.31, 1.0))
    model.material("carriage_gray", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("ram_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    base_guideway = model.part("base_guideway")
    base_guideway.visual(
        mesh_from_cadquery(_base_guideway_shape(), "base_guideway"),
        material="base_steel",
        name="guideway_body",
    )
    base_guideway.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, GUIDE_TOP_Z)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, -GUIDE_TOP_Z / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_shape(), "x_carriage"),
        material="carriage_gray",
        name="carriage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    z_stage = model.part("z_stage")
    z_stage.visual(
        mesh_from_cadquery(_z_stage_shape(), "z_stage"),
        material="ram_aluminum",
        name="z_stage_body",
    )
    z_stage.inertial = Inertial.from_geometry(
        Box((TOOL_FACE_LENGTH, TOOL_FACE_WIDTH, Z_STAGE_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, Z_STAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base_guideway,
        child=x_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=450.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_stage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=300.0,
            velocity=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_guideway = object_model.get_part("base_guideway")
    x_carriage = object_model.get_part("x_carriage")
    z_stage = object_model.get_part("z_stage")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

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
        x_carriage,
        base_guideway,
        name="x_carriage_is_supported_by_lower_guideway",
    )
    ctx.expect_contact(
        z_stage,
        x_carriage,
        name="z_stage_is_supported_by_moving_carriage",
    )
    ctx.expect_overlap(
        x_carriage,
        base_guideway,
        axes="xy",
        min_overlap=0.10,
        name="x_carriage_sits_broadly_on_guideway",
    )
    ctx.expect_gap(
        z_stage,
        base_guideway,
        axis="z",
        min_gap=0.030,
        name="z_stage_clears_the_base_guideway",
    )

    with ctx.pose({x_axis: -X_TRAVEL}):
        left_pos = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: X_TRAVEL}):
        right_pos = ctx.part_world_position(x_carriage)
    x_ok = (
        left_pos is not None
        and right_pos is not None
        and abs((right_pos[0] - left_pos[0]) - (2.0 * X_TRAVEL)) <= 0.002
        and abs(right_pos[1] - left_pos[1]) <= 1e-6
        and abs(right_pos[2] - left_pos[2]) <= 1e-6
    )
    ctx.check(
        "x_axis_translates_the_lower_block_horizontally",
        x_ok,
        details=f"left={left_pos}, right={right_pos}",
    )

    with ctx.pose({x_axis: 0.060, z_axis: 0.0}):
        z_home = ctx.part_world_position(z_stage)
    with ctx.pose({x_axis: 0.060, z_axis: Z_TRAVEL}):
        z_up = ctx.part_world_position(z_stage)
    z_ok = (
        z_home is not None
        and z_up is not None
        and abs((z_up[2] - z_home[2]) - Z_TRAVEL) <= 0.002
        and abs(z_up[0] - z_home[0]) <= 1e-6
        and abs(z_up[1] - z_home[1]) <= 1e-6
    )
    ctx.check(
        "z_axis_lifts_the_upper_guide_vertically",
        z_ok,
        details=f"home={z_home}, up={z_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
