from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_BASE_LENGTH = 0.68
RAIL_BASE_DEPTH = 0.11
RAIL_BASE_HEIGHT = 0.024
RAIL_BASE_CENTER_Z = -0.068

RAIL_TRACK_LENGTH = 0.62
RAIL_TRACK_WIDTH = 0.040
RAIL_TRACK_HEIGHT = 0.028
RAIL_TRACK_CENTER_Z = RAIL_BASE_CENTER_Z + (RAIL_BASE_HEIGHT / 2.0) + (RAIL_TRACK_HEIGHT / 2.0)

END_SUPPORT_LENGTH = 0.06
END_SUPPORT_DEPTH = 0.11
END_SUPPORT_HEIGHT = 0.090
END_SUPPORT_CENTER_Z = -0.033
END_SUPPORT_OFFSET_X = (RAIL_BASE_LENGTH / 2.0) - (END_SUPPORT_LENGTH / 2.0)

CARRIAGE_SHOE_LENGTH = 0.14
CARRIAGE_SHOE_WIDTH = RAIL_TRACK_WIDTH
CARRIAGE_SHOE_HEIGHT = 0.028
CARRIAGE_SHOE_CENTER_Z = (
    RAIL_TRACK_CENTER_Z + (RAIL_TRACK_HEIGHT / 2.0) + (CARRIAGE_SHOE_HEIGHT / 2.0)
)

CARRIAGE_BRIDGE_LENGTH = 0.14
CARRIAGE_BRIDGE_DEPTH = 0.080
CARRIAGE_BRIDGE_HEIGHT = 0.050
CARRIAGE_BRIDGE_CENTER_Y = 0.020
CARRIAGE_BRIDGE_CENTER_Z = 0.020

Z_COLUMN_WIDTH = 0.060
Z_COLUMN_DEPTH = 0.032
Z_COLUMN_HEIGHT = 0.210
Z_COLUMN_CENTER_Y = 0.060
Z_COLUMN_CENTER_Z = 0.122
Z_COLUMN_FRONT_FACE_Y = Z_COLUMN_CENTER_Y + (Z_COLUMN_DEPTH / 2.0)

Z_COLUMN_CAP_WIDTH = 0.086
Z_COLUMN_CAP_DEPTH = 0.050
Z_COLUMN_CAP_HEIGHT = 0.022
Z_COLUMN_CAP_CENTER_Y = 0.048
Z_COLUMN_CAP_CENTER_Z = 0.224

TOOL_SLIDER_WIDTH = Z_COLUMN_WIDTH
TOOL_SLIDER_DEPTH = 0.024
TOOL_SLIDER_HEIGHT = 0.090
TOOL_SLIDER_CENTER_Y = TOOL_SLIDER_DEPTH / 2.0

TOOL_FRONT_PLATE_WIDTH = 0.142
TOOL_FRONT_PLATE_DEPTH = 0.012
TOOL_FRONT_PLATE_HEIGHT = 0.150
TOOL_FRONT_PLATE_CENTER_Y = TOOL_SLIDER_DEPTH + (TOOL_FRONT_PLATE_DEPTH / 2.0)
TOOL_FRONT_PLATE_CENTER_Z = -0.005

TOOL_NOSE_BLOCK_WIDTH = 0.050
TOOL_NOSE_BLOCK_DEPTH = 0.030
TOOL_NOSE_BLOCK_HEIGHT = 0.034
TOOL_NOSE_BLOCK_CENTER_Y = (
    TOOL_SLIDER_DEPTH + TOOL_FRONT_PLATE_DEPTH + (TOOL_NOSE_BLOCK_DEPTH / 2.0)
)
TOOL_NOSE_BLOCK_CENTER_Z = -0.055

TOOL_HOLDER_RADIUS = 0.012
TOOL_HOLDER_LENGTH = 0.050
TOOL_HOLDER_CENTER_Y = TOOL_NOSE_BLOCK_CENTER_Y + 0.030
TOOL_HOLDER_CENTER_Z = TOOL_NOSE_BLOCK_CENTER_Z

X_TRAVEL = 0.15
Z_HOME = 0.125
Z_TRAVEL_LOWER = -0.07
Z_TRAVEL_UPPER = 0.04


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_xz_positioning_head")

    model.material("frame_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_gray", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("tool_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("guide_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_BASE_LENGTH, RAIL_BASE_DEPTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_CENTER_Z)),
        material="frame_black",
        name="rail_body",
    )
    rail.visual(
        Box((RAIL_TRACK_LENGTH, RAIL_TRACK_WIDTH, RAIL_TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_TRACK_CENTER_Z)),
        material="guide_steel",
        name="rail_track",
    )
    rail.visual(
        Box((END_SUPPORT_LENGTH, END_SUPPORT_DEPTH, END_SUPPORT_HEIGHT)),
        origin=Origin(xyz=(-END_SUPPORT_OFFSET_X, 0.0, END_SUPPORT_CENTER_Z)),
        material="frame_black",
        name="left_support",
    )
    rail.visual(
        Box((END_SUPPORT_LENGTH, END_SUPPORT_DEPTH, END_SUPPORT_HEIGHT)),
        origin=Origin(xyz=(END_SUPPORT_OFFSET_X, 0.0, END_SUPPORT_CENTER_Z)),
        material="frame_black",
        name="right_support",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_BASE_LENGTH, RAIL_BASE_DEPTH, END_SUPPORT_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_SHOE_CENTER_Z)),
        material="carriage_gray",
        name="carriage_shoe",
    )
    carriage.visual(
        Box((CARRIAGE_BRIDGE_LENGTH, CARRIAGE_BRIDGE_DEPTH, CARRIAGE_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CARRIAGE_BRIDGE_CENTER_Y, CARRIAGE_BRIDGE_CENTER_Z),
        ),
        material="carriage_gray",
        name="carriage_body",
    )
    carriage.visual(
        Box((Z_COLUMN_WIDTH, Z_COLUMN_DEPTH, Z_COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, Z_COLUMN_CENTER_Y, Z_COLUMN_CENTER_Z)),
        material="frame_black",
        name="z_column",
    )
    carriage.visual(
        Box((Z_COLUMN_CAP_WIDTH, Z_COLUMN_CAP_DEPTH, Z_COLUMN_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, Z_COLUMN_CAP_CENTER_Y, Z_COLUMN_CAP_CENTER_Z)),
        material="frame_black",
        name="column_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.16, 0.09, 0.26)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.04, 0.11)),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((TOOL_SLIDER_WIDTH, TOOL_SLIDER_DEPTH, TOOL_SLIDER_HEIGHT)),
        origin=Origin(xyz=(0.0, TOOL_SLIDER_CENTER_Y, 0.0)),
        material="tool_black",
        name="z_slider",
    )
    tool_plate.visual(
        Box((TOOL_FRONT_PLATE_WIDTH, TOOL_FRONT_PLATE_DEPTH, TOOL_FRONT_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, TOOL_FRONT_PLATE_CENTER_Y, TOOL_FRONT_PLATE_CENTER_Z),
        ),
        material="tool_black",
        name="tool_plate_body",
    )
    tool_plate.visual(
        Box((TOOL_NOSE_BLOCK_WIDTH, TOOL_NOSE_BLOCK_DEPTH, TOOL_NOSE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, TOOL_NOSE_BLOCK_CENTER_Y, TOOL_NOSE_BLOCK_CENTER_Z),
        ),
        material="tool_black",
        name="nose_block",
    )
    tool_plate.visual(
        Cylinder(radius=TOOL_HOLDER_RADIUS, length=TOOL_HOLDER_LENGTH),
        origin=Origin(
            xyz=(0.0, TOOL_HOLDER_CENTER_Y, TOOL_HOLDER_CENTER_Z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="guide_steel",
        name="tool_holder",
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((0.15, 0.07, 0.16)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.03, -0.01)),
    )

    model.articulation(
        "x_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=320.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "z_tool_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_plate,
        origin=Origin(xyz=(0.0, Z_COLUMN_FRONT_FACE_Y, Z_HOME)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
            effort=220.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    tool_plate = object_model.get_part("tool_plate")
    x_slide = object_model.get_articulation("x_carriage_slide")
    z_slide = object_model.get_articulation("z_tool_slide")

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="carriage_shoe",
        elem_b="rail_track",
        name="carriage shoe sits on the horizontal rail track",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="x",
        elem_a="carriage_shoe",
        elem_b="rail_track",
        min_overlap=0.13,
        name="carriage remains engaged on the horizontal rail at home",
    )
    ctx.expect_contact(
        tool_plate,
        carriage,
        elem_a="z_slider",
        elem_b="z_column",
        name="tool slider bears against the vertical column",
    )
    ctx.expect_overlap(
        tool_plate,
        carriage,
        axes="z",
        elem_a="z_slider",
        elem_b="z_column",
        min_overlap=0.08,
        name="tool plate remains captured on the vertical column at home",
    )

    carriage_home = ctx.part_world_position(carriage)
    tool_home = ctx.part_world_position(tool_plate)

    with ctx.pose({x_slide: X_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            rail,
            axes="x",
            elem_a="carriage_shoe",
            elem_b="rail_track",
            min_overlap=0.13,
            name="carriage still overlaps the rail at full X travel",
        )
        ctx.expect_contact(
            carriage,
            rail,
            elem_a="carriage_shoe",
            elem_b="rail_track",
            name="carriage shoe remains seated on the rail track at full X travel",
        )
        carriage_extended = ctx.part_world_position(carriage)

    with ctx.pose({z_slide: Z_TRAVEL_UPPER}):
        ctx.expect_overlap(
            tool_plate,
            carriage,
            axes="z",
            elem_a="z_slider",
            elem_b="z_column",
            min_overlap=0.08,
            name="tool plate retains vertical guide engagement at full lift",
        )
        ctx.expect_contact(
            tool_plate,
            carriage,
            elem_a="z_slider",
            elem_b="z_column",
            name="tool slider remains seated on the column at full lift",
        )
        tool_raised = ctx.part_world_position(tool_plate)

    with ctx.pose({x_slide: X_TRAVEL, z_slide: Z_TRAVEL_UPPER}):
        combined_pose_tool = ctx.part_world_position(tool_plate)

    ctx.check(
        "carriage translates along +X",
        carriage_home is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_home[0] + 0.12
        and abs(carriage_extended[1] - carriage_home[1]) < 1e-4
        and abs(carriage_extended[2] - carriage_home[2]) < 1e-4,
        details=f"home={carriage_home}, extended={carriage_extended}",
    )
    ctx.check(
        "tool plate translates upward along +Z",
        tool_home is not None
        and tool_raised is not None
        and tool_raised[2] > tool_home[2] + 0.03
        and abs(tool_raised[0] - tool_home[0]) < 1e-4,
        details=f"home={tool_home}, raised={tool_raised}",
    )
    ctx.check(
        "tool plate follows carriage in X while keeping its lifted Z position",
        tool_home is not None
        and combined_pose_tool is not None
        and combined_pose_tool[0] > tool_home[0] + 0.12
        and combined_pose_tool[2] > tool_home[2] + 0.03,
        details=f"home={tool_home}, combined={combined_pose_tool}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
