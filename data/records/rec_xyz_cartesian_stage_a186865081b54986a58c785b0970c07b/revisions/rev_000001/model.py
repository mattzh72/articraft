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


BASE_LENGTH = 0.48
BASE_WIDTH = 0.20
BASE_HEIGHT = 0.022
X_RAIL_LENGTH = 0.42
X_RAIL_WIDTH = 0.028
X_RAIL_HEIGHT = 0.018
X_RAIL_OFFSET_Y = 0.055
X_SCREW_COVER_WIDTH = 0.018
X_SCREW_COVER_HEIGHT = 0.012

X_CARRIAGE_LENGTH = 0.145
X_CARRIAGE_WIDTH = 0.180
X_CARRIAGE_HEIGHT = 0.040
X_CARRIAGE_SLOT_WIDTH = 0.074
X_CARRIAGE_SLOT_HEIGHT = 0.018
Y_TABLE_LENGTH = 0.118
Y_TABLE_WIDTH = 0.150
Y_TABLE_HEIGHT = 0.010
Y_RAIL_LENGTH = 0.148
Y_RAIL_WIDTH = 0.020
Y_RAIL_HEIGHT = 0.012
Y_RAIL_OFFSET_X = 0.030
Y_SCREW_COVER_WIDTH = 0.014
Y_SCREW_COVER_HEIGHT = 0.014

Y_CARRIAGE_WIDTH = 0.100
Y_CARRIAGE_DEPTH = 0.100
Y_CARRIAGE_HEIGHT = 0.036
Y_CARRIAGE_SLOT_WIDTH = 0.040
Y_CARRIAGE_SLOT_DEPTH = 0.078
Y_CARRIAGE_SLOT_HEIGHT = 0.016
Z_COLUMN_BASE_WIDTH = 0.090
Z_COLUMN_BASE_DEPTH = 0.068
Z_COLUMN_BASE_HEIGHT = 0.016
Z_COLUMN_WIDTH = 0.094
Z_COLUMN_DEPTH = 0.036
Z_COLUMN_HEIGHT = 0.200
Z_RAIL_WIDTH = 0.012
Z_RAIL_DEPTH = 0.008
Z_RAIL_HEIGHT = 0.165
Z_RAIL_OFFSET_X = 0.024
Z_RAIL_CENTER_Y = 0.022
Z_SCREW_COVER_WIDTH = 0.014
Z_SCREW_COVER_DEPTH = 0.010
Z_SCREW_COVER_HEIGHT = 0.145

Z_SLIDE_BODY_WIDTH = 0.100
Z_SLIDE_BODY_DEPTH = 0.038
Z_SLIDE_BODY_HEIGHT = 0.078
Z_SLIDE_BODY_CENTER_Y = 0.029
Z_SLIDE_PAD_WIDTH = 0.014
Z_SLIDE_PAD_DEPTH = 0.010
Z_SLIDE_PAD_HEIGHT = 0.070
Z_SLIDE_PAD_CENTER_Y = 0.005
TOOL_BOSS_WIDTH = 0.030
TOOL_BOSS_DEPTH = 0.022
TOOL_BOSS_HEIGHT = 0.022
TOOL_BOSS_CENTER_Y = 0.059
TOOL_BOSS_BOTTOM_Z = 0.028
TOOL_PLATE_SIZE = 0.090
TOOL_PLATE_THICKNESS = 0.010
TOOL_PLATE_CENTER_Y = 0.075
TOOL_PLATE_BOTTOM_Z = 0.004

X_RAIL_TOP_Z = BASE_HEIGHT + X_RAIL_HEIGHT
Y_RAIL_TOP_Z = X_CARRIAGE_HEIGHT + Y_TABLE_HEIGHT + Y_RAIL_HEIGHT
Z_RAIL_BOTTOM_Z = Y_CARRIAGE_HEIGHT + Z_COLUMN_BASE_HEIGHT
Z_RAIL_FRONT_Y = Z_RAIL_CENTER_Y + Z_RAIL_DEPTH / 2.0


def box_at(size: tuple[float, float, float], center_xy: tuple[float, float], bottom_z: float) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy = center_xy
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate((cx, cy, bottom_z))


def make_base_shape() -> cq.Workplane:
    base = box_at((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT), (0.0, 0.0), 0.0)
    base = base.union(
        box_at((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT), (0.0, X_RAIL_OFFSET_Y), BASE_HEIGHT)
    )
    base = base.union(
        box_at((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT), (0.0, -X_RAIL_OFFSET_Y), BASE_HEIGHT)
    )
    base = base.union(
        box_at((X_RAIL_LENGTH * 0.82, X_SCREW_COVER_WIDTH, X_SCREW_COVER_HEIGHT), (0.0, 0.0), BASE_HEIGHT)
    )
    return base


def make_x_carriage_shape() -> cq.Workplane:
    carriage = box_at((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_CARRIAGE_HEIGHT), (0.0, 0.0), 0.0)
    carriage = carriage.cut(
        box_at(
            (X_CARRIAGE_LENGTH * 0.72, X_CARRIAGE_SLOT_WIDTH, X_CARRIAGE_SLOT_HEIGHT),
            (0.0, 0.0),
            0.0,
        )
    )
    carriage = carriage.union(box_at((Y_TABLE_LENGTH, Y_TABLE_WIDTH, Y_TABLE_HEIGHT), (0.0, 0.0), X_CARRIAGE_HEIGHT))
    carriage = carriage.union(
        box_at((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT), (Y_RAIL_OFFSET_X, 0.0), X_RAIL_TOP_Z + 0.010)
    )
    carriage = carriage.union(
        box_at((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT), (-Y_RAIL_OFFSET_X, 0.0), X_RAIL_TOP_Z + 0.010)
    )
    carriage = carriage.union(
        box_at(
            (Y_SCREW_COVER_WIDTH, Y_RAIL_LENGTH * 0.92, Y_SCREW_COVER_HEIGHT),
            (0.0, 0.0),
            X_RAIL_TOP_Z + 0.010,
        )
    )
    return carriage


def make_y_carriage_shape() -> cq.Workplane:
    carriage = box_at((Y_CARRIAGE_WIDTH, Y_CARRIAGE_DEPTH, Y_CARRIAGE_HEIGHT), (0.0, 0.0), 0.0)
    carriage = carriage.cut(
        box_at(
            (Y_CARRIAGE_SLOT_WIDTH, Y_CARRIAGE_SLOT_DEPTH, Y_CARRIAGE_SLOT_HEIGHT),
            (0.0, 0.0),
            0.0,
        )
    )
    carriage = carriage.union(
        box_at((Z_COLUMN_BASE_WIDTH, Z_COLUMN_BASE_DEPTH, Z_COLUMN_BASE_HEIGHT), (0.0, 0.0), Y_CARRIAGE_HEIGHT)
    )
    carriage = carriage.union(
        box_at((Z_COLUMN_WIDTH, Z_COLUMN_DEPTH, Z_COLUMN_HEIGHT), (0.0, 0.0), Z_RAIL_BOTTOM_Z)
    )
    carriage = carriage.union(
        box_at((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT), (Z_RAIL_OFFSET_X, Z_RAIL_CENTER_Y), Z_RAIL_BOTTOM_Z)
    )
    carriage = carriage.union(
        box_at((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT), (-Z_RAIL_OFFSET_X, Z_RAIL_CENTER_Y), Z_RAIL_BOTTOM_Z)
    )
    carriage = carriage.union(
        box_at(
            (Z_SCREW_COVER_WIDTH, Z_SCREW_COVER_DEPTH, Z_SCREW_COVER_HEIGHT),
            (0.0, Z_RAIL_CENTER_Y),
            Z_RAIL_BOTTOM_Z + 0.008,
        )
    )
    return carriage


def make_tool_plate_shape() -> cq.Workplane:
    plate = box_at(
        (TOOL_PLATE_SIZE, TOOL_PLATE_THICKNESS, TOOL_PLATE_SIZE),
        (0.0, TOOL_PLATE_CENTER_Y),
        TOOL_PLATE_BOTTOM_Z,
    )
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .hole(0.016)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.025, -0.025),
                (-0.025, 0.025),
                (0.025, -0.025),
                (0.025, 0.025),
            ]
        )
        .hole(0.005)
    )
    return plate


def make_z_slide_shape() -> cq.Workplane:
    slide = box_at(
        (Z_SLIDE_BODY_WIDTH, Z_SLIDE_BODY_DEPTH, Z_SLIDE_BODY_HEIGHT),
        (0.0, Z_SLIDE_BODY_CENTER_Y),
        0.0,
    )
    slide = slide.union(
        box_at(
            (Z_SLIDE_PAD_WIDTH, Z_SLIDE_PAD_DEPTH, Z_SLIDE_PAD_HEIGHT),
            (Z_RAIL_OFFSET_X, Z_SLIDE_PAD_CENTER_Y),
            0.0,
        )
    )
    slide = slide.union(
        box_at(
            (Z_SLIDE_PAD_WIDTH, Z_SLIDE_PAD_DEPTH, Z_SLIDE_PAD_HEIGHT),
            (-Z_RAIL_OFFSET_X, Z_SLIDE_PAD_CENTER_Y),
            0.0,
        )
    )
    slide = slide.union(
        box_at((TOOL_BOSS_WIDTH, TOOL_BOSS_DEPTH, TOOL_BOSS_HEIGHT), (0.0, TOOL_BOSS_CENTER_Y), TOOL_BOSS_BOTTOM_Z)
    )
    slide = slide.union(make_tool_plate_shape())
    return slide


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_positioning_stage")

    base_mat = model.material("base_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    x_mat = model.material("x_carriage_gray", rgba=(0.52, 0.55, 0.58, 1.0))
    y_mat = model.material("y_carriage_light", rgba=(0.70, 0.73, 0.76, 1.0))
    z_mat = model.material("z_slide_bluegray", rgba=(0.55, 0.63, 0.72, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(make_base_shape(), "stage_base"), material=base_mat, name="base_visual")

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(make_x_carriage_shape(), "stage_x_carriage"),
        material=x_mat,
        name="x_carriage_visual",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(make_y_carriage_shape(), "stage_y_carriage"),
        material=y_mat,
        name="y_carriage_visual",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        mesh_from_cadquery(make_z_slide_shape(), "stage_z_slide"),
        material=z_mat,
        name="z_slide_visual",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=-0.022, upper=0.022),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_RAIL_FRONT_Y, Z_RAIL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.15, lower=0.0, upper=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")
    y_carriage_visual = y_carriage.get_visual("y_carriage_visual")
    z_slide_visual = z_slide.get_visual("z_slide_visual")

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
    ctx.allow_overlap(
        y_carriage,
        z_slide,
        elem_a=y_carriage_visual,
        elem_b=z_slide_visual,
        reason="The Z-axis carriage is modeled as a captured linear-guide block that intentionally nests around the vertical guide stack.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "x axis is prismatic along world x",
        x_axis.articulation_type == ArticulationType.PRISMATIC and tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        f"Expected prismatic +X axis, got type={x_axis.articulation_type} axis={x_axis.axis}",
    )
    ctx.check(
        "y axis is prismatic along world y",
        y_axis.articulation_type == ArticulationType.PRISMATIC and tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        f"Expected prismatic +Y axis, got type={y_axis.articulation_type} axis={y_axis.axis}",
    )
    ctx.check(
        "z axis is prismatic along world z",
        z_axis.articulation_type == ArticulationType.PRISMATIC and tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        f"Expected prismatic +Z axis, got type={z_axis.articulation_type} axis={z_axis.axis}",
    )

    ctx.expect_contact(base, x_carriage, name="x carriage seated on base rails")
    ctx.expect_overlap(base, x_carriage, axes="xy", min_overlap=0.12, name="x carriage overlaps x base footprint")

    ctx.expect_contact(x_carriage, y_carriage, name="y carriage seated on cross rails")
    ctx.expect_overlap(
        x_carriage,
        y_carriage,
        axes="xy",
        min_overlap=0.08,
        name="y carriage overlaps x carriage footprint",
    )

    ctx.expect_contact(y_carriage, z_slide, name="z slide mounted to vertical rails")
    ctx.expect_overlap(y_carriage, z_slide, axes="xz", min_overlap=0.06, name="z slide overlaps column in xz")

    rest_x = ctx.part_world_position(x_carriage)
    rest_y = ctx.part_world_position(y_carriage)
    rest_z = ctx.part_world_position(z_slide)
    with ctx.pose(x_axis=0.10):
        moved_x = ctx.part_world_position(x_carriage)
    with ctx.pose(y_axis=0.018):
        moved_y = ctx.part_world_position(y_carriage)
    with ctx.pose(z_axis=0.05):
        moved_z = ctx.part_world_position(z_slide)

    ctx.check(
        "x carriage translates only along x",
        rest_x is not None
        and moved_x is not None
        and abs((moved_x[0] - rest_x[0]) - 0.10) < 1e-5
        and abs(moved_x[1] - rest_x[1]) < 1e-5
        and abs(moved_x[2] - rest_x[2]) < 1e-5,
        f"rest={rest_x}, moved={moved_x}",
    )
    ctx.check(
        "y carriage translates only along y",
        rest_y is not None
        and moved_y is not None
        and abs((moved_y[1] - rest_y[1]) - 0.018) < 1e-5
        and abs(moved_y[0] - rest_y[0]) < 1e-5
        and abs(moved_y[2] - rest_y[2]) < 1e-5,
        f"rest={rest_y}, moved={moved_y}",
    )
    ctx.check(
        "z slide translates only along z",
        rest_z is not None
        and moved_z is not None
        and abs((moved_z[2] - rest_z[2]) - 0.05) < 1e-5
        and abs(moved_z[0] - rest_z[0]) < 1e-5
        and abs(moved_z[1] - rest_z[1]) < 1e-5,
        f"rest={rest_z}, moved={moved_z}",
    )

    with ctx.pose(x_axis=0.10, y_axis=-0.018, z_axis=0.05):
        ctx.expect_contact(base, x_carriage, name="x carriage remains supported in offset pose")
        ctx.expect_contact(x_carriage, y_carriage, name="y carriage remains supported in offset pose")
        ctx.expect_contact(y_carriage, z_slide, name="z slide remains guided in offset pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
