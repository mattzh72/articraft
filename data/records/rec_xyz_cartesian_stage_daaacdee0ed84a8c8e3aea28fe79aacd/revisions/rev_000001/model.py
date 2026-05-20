from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


EMBED = 0.001

BASE_L = 0.90
BASE_W = 0.42
BASE_T = 0.035
BASE_END_BLOCK_L = 0.10
BASE_END_BLOCK_W = 0.30
BASE_END_BLOCK_H = 0.030
BASE_COVER_L = 0.62
BASE_COVER_W = 0.08
BASE_COVER_H = 0.024

X_RAIL_L = 0.82
X_RAIL_W = 0.05
X_RAIL_H = 0.04
X_RAIL_Y = 0.12

X_TRAVEL = 0.18
X_FRAME_Z = 0.1055
X_DECK_L = 0.60
X_DECK_W = 0.31
X_DECK_T = 0.026
X_CENTER_PAD_L = 0.24
X_CENTER_PAD_W = 0.18
X_CENTER_PAD_H = 0.016
X_SHOE_L = 0.18
X_SHOE_W = 0.068
X_SHOE_H = 0.019
X_SHOE_Y = X_RAIL_Y
Y_RAIL_X = 0.075
Y_RAIL_L = 0.26
Y_RAIL_W = 0.036
Y_RAIL_H = 0.016

Y_TRAVEL = 0.07
Y_FRAME_Z = 0.0545
Y_SADDLE_LX = 0.19
Y_SADDLE_LY = 0.29
Y_SADDLE_T = 0.024
Y_SHOE_LX = 0.054
Y_SHOE_LY = 0.11
Y_SHOE_H = 0.015
Y_COLUMN_X = 0.11
Y_COLUMN_Y = 0.045
Y_COLUMN_H = 0.275
Y_GUSSET_X = 0.03
Y_GUSSET_Y = 0.055
Y_GUSSET_H = 0.10
Z_RAIL_X = 0.032
Z_RAIL_WX = 0.018
Z_RAIL_WY = 0.012
Z_RAIL_H = 0.220
Z_RAIL_Y = 0.028
Z_RAIL_Z = 0.140

Z_TRAVEL = 0.08
Z_FRAME_Y = 0.054
Z_FRAME_Z = 0.105
Z_PLATE_X = 0.094
Z_PLATE_Y = 0.016
Z_PLATE_H = 0.120
Z_SHOE_X = 0.020
Z_SHOE_Y = 0.012
Z_SHOE_H = 0.110
Z_SHOE_X_POS = Z_RAIL_X
Z_SHOE_Y_POS = -0.014
Z_TOOL_X = 0.070
Z_TOOL_Y = 0.042
Z_TOOL_H = 0.040
Z_TOOL_Y_POS = 0.021
Z_TOOL_Z_POS = -0.036
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_xyz_gantry_lite_stage")

    model.material("powder_charcoal", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("anodized_gray", rgba=(0.50, 0.53, 0.57, 1.0))
    model.material("rail_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("carriage_black", rgba=(0.12, 0.13, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="powder_charcoal",
        name="elem_base_body",
    )
    base.visual(
        Box((BASE_END_BLOCK_L, BASE_END_BLOCK_W, BASE_END_BLOCK_H)),
        origin=Origin(
            xyz=(BASE_L * 0.36, 0.0, BASE_T + BASE_END_BLOCK_H / 2.0 - EMBED / 2.0),
        ),
        material="powder_charcoal",
        name="elem_base_end_front",
    )
    base.visual(
        Box((BASE_END_BLOCK_L, BASE_END_BLOCK_W, BASE_END_BLOCK_H)),
        origin=Origin(
            xyz=(-BASE_L * 0.36, 0.0, BASE_T + BASE_END_BLOCK_H / 2.0 - EMBED / 2.0),
        ),
        material="powder_charcoal",
        name="elem_base_end_rear",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(
            xyz=(0.0, X_RAIL_Y, BASE_T + X_RAIL_H / 2.0 - EMBED / 2.0),
        ),
        material="rail_steel",
        name="elem_x_rail_left",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(
            xyz=(0.0, -X_RAIL_Y, BASE_T + X_RAIL_H / 2.0 - EMBED / 2.0),
        ),
        material="rail_steel",
        name="elem_x_rail_right",
    )
    base.visual(
        Box((BASE_COVER_L, BASE_COVER_W, BASE_COVER_H)),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_T + BASE_COVER_H / 2.0 - EMBED / 2.0),
        ),
        material="anodized_gray",
        name="elem_x_drive_cover",
    )

    x_slide = model.part("x_slide")
    x_slide.visual(
        Box((X_DECK_L, X_DECK_W, X_DECK_T)),
        material="machined_aluminum",
        name="elem_x_slide_body",
    )
    x_slide.visual(
        Box((X_CENTER_PAD_L, X_CENTER_PAD_W, X_CENTER_PAD_H)),
        origin=Origin(xyz=(0.0, 0.0, X_DECK_T / 2.0 + X_CENTER_PAD_H / 2.0 - EMBED / 2.0)),
        material="machined_aluminum",
        name="elem_x_center_pad",
    )
    x_slide.visual(
        Box((X_SHOE_L, X_SHOE_W, X_SHOE_H)),
        origin=Origin(xyz=(0.0, X_SHOE_Y, -0.0215)),
        material="carriage_black",
        name="elem_x_shoe_left",
    )
    x_slide.visual(
        Box((X_SHOE_L, X_SHOE_W, X_SHOE_H)),
        origin=Origin(xyz=(0.0, -X_SHOE_Y, -0.0215)),
        material="carriage_black",
        name="elem_x_shoe_right",
    )
    x_slide.visual(
        Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)),
        origin=Origin(
            xyz=(Y_RAIL_X, 0.0, X_DECK_T / 2.0 + Y_RAIL_H / 2.0 - EMBED / 2.0),
        ),
        material="rail_steel",
        name="elem_y_rail_pos_x",
    )
    x_slide.visual(
        Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)),
        origin=Origin(
            xyz=(-Y_RAIL_X, 0.0, X_DECK_T / 2.0 + Y_RAIL_H / 2.0 - EMBED / 2.0),
        ),
        material="rail_steel",
        name="elem_y_rail_neg_x",
    )

    y_runner = model.part("y_runner")
    y_runner.visual(
        Box((Y_SADDLE_LX, Y_SADDLE_LY, Y_SADDLE_T)),
        material="anodized_gray",
        name="elem_y_runner_body",
    )
    y_runner.visual(
        Box((Y_COLUMN_X, Y_COLUMN_Y, Y_COLUMN_H)),
        origin=Origin(
            xyz=(0.0, 0.0, Y_SADDLE_T / 2.0 + Y_COLUMN_H / 2.0 - EMBED / 2.0),
        ),
        material="anodized_gray",
        name="elem_y_column",
    )
    y_runner.visual(
        Box((Y_GUSSET_X, Y_GUSSET_Y, Y_GUSSET_H)),
        origin=Origin(
            xyz=(-0.035, 0.0, Y_SADDLE_T / 2.0 + Y_GUSSET_H / 2.0 - EMBED / 2.0),
        ),
        material="anodized_gray",
        name="elem_y_gusset_left",
    )
    y_runner.visual(
        Box((Y_GUSSET_X, Y_GUSSET_Y, Y_GUSSET_H)),
        origin=Origin(
            xyz=(0.035, 0.0, Y_SADDLE_T / 2.0 + Y_GUSSET_H / 2.0 - EMBED / 2.0),
        ),
        material="anodized_gray",
        name="elem_y_gusset_right",
    )
    y_runner.visual(
        Box((Y_SHOE_LX, Y_SHOE_LY, Y_SHOE_H)),
        origin=Origin(xyz=(Y_RAIL_X, 0.0, -0.0195)),
        material="carriage_black",
        name="elem_y_shoe_pos_x",
    )
    y_runner.visual(
        Box((Y_SHOE_LX, Y_SHOE_LY, Y_SHOE_H)),
        origin=Origin(xyz=(-Y_RAIL_X, 0.0, -0.0195)),
        material="carriage_black",
        name="elem_y_shoe_neg_x",
    )
    y_runner.visual(
        Box((Z_RAIL_WX, Z_RAIL_WY, Z_RAIL_H)),
        origin=Origin(xyz=(Z_RAIL_X, Z_RAIL_Y, Z_RAIL_Z)),
        material="rail_steel",
        name="elem_z_rail_left",
    )
    y_runner.visual(
        Box((Z_RAIL_WX, Z_RAIL_WY, Z_RAIL_H)),
        origin=Origin(xyz=(-Z_RAIL_X, Z_RAIL_Y, Z_RAIL_Z)),
        material="rail_steel",
        name="elem_z_rail_right",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((Z_PLATE_X, Z_PLATE_Y, Z_PLATE_H)),
        material="machined_aluminum",
        name="elem_z_slide_body",
    )
    z_slide.visual(
        Box((Z_TOOL_X, Z_TOOL_Y, Z_TOOL_H)),
        origin=Origin(xyz=(0.0, Z_TOOL_Y_POS, Z_TOOL_Z_POS)),
        material="machined_aluminum",
        name="elem_z_tool_block",
    )
    z_slide.visual(
        Box((0.050, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.028, -0.072)),
        material="machined_aluminum",
        name="elem_z_nose",
    )
    z_slide.visual(
        Box((Z_SHOE_X, Z_SHOE_Y, Z_SHOE_H)),
        origin=Origin(xyz=(Z_SHOE_X_POS, Z_SHOE_Y_POS, 0.0)),
        material="carriage_black",
        name="elem_z_shoe_left",
    )
    z_slide.visual(
        Box((Z_SHOE_X, Z_SHOE_Y, Z_SHOE_H)),
        origin=Origin(xyz=(-Z_SHOE_X_POS, Z_SHOE_Y_POS, 0.0)),
        material="carriage_black",
        name="elem_z_shoe_right",
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, X_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=500.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "x_slide_to_y_runner",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_runner,
        origin=Origin(xyz=(0.0, 0.0, Y_FRAME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=250.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "y_runner_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=y_runner,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_FRAME_Y, Z_FRAME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=180.0,
            velocity=0.15,
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

    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_runner = object_model.get_part("y_runner")
    z_slide = object_model.get_part("z_slide")

    x_joint = object_model.get_articulation("base_to_x_slide")
    y_joint = object_model.get_articulation("x_slide_to_y_runner")
    z_joint = object_model.get_articulation("y_runner_to_z_slide")

    ctx.check(
        "primary stage parts exist",
        all(part is not None for part in (base, x_slide, y_runner, z_slide)),
    )

    ctx.expect_contact(
        x_slide,
        base,
        elem_a="elem_x_shoe_left",
        elem_b="elem_x_rail_left",
        contact_tol=0.001,
        name="left X shoe is seated on the left X rail",
    )
    ctx.expect_contact(
        x_slide,
        base,
        elem_a="elem_x_shoe_right",
        elem_b="elem_x_rail_right",
        contact_tol=0.001,
        name="right X shoe is seated on the right X rail",
    )
    ctx.expect_contact(
        y_runner,
        x_slide,
        elem_a="elem_y_shoe_pos_x",
        elem_b="elem_y_rail_pos_x",
        contact_tol=0.001,
        name="positive-X Y shoe is seated on the positive-X Y rail",
    )
    ctx.expect_contact(
        y_runner,
        x_slide,
        elem_a="elem_y_shoe_neg_x",
        elem_b="elem_y_rail_neg_x",
        contact_tol=0.001,
        name="negative-X Y shoe is seated on the negative-X Y rail",
    )
    ctx.expect_contact(
        z_slide,
        y_runner,
        elem_a="elem_z_shoe_left",
        elem_b="elem_z_rail_left",
        contact_tol=0.001,
        name="left Z shoe is seated on the left Z guide rail",
    )
    ctx.expect_contact(
        z_slide,
        y_runner,
        elem_a="elem_z_shoe_right",
        elem_b="elem_z_rail_right",
        contact_tol=0.001,
        name="right Z shoe is seated on the right Z guide rail",
    )

    rest_x = ctx.part_world_position(x_slide)
    with ctx.pose({x_joint: X_TRAVEL}):
        ctx.expect_overlap(
            x_slide,
            base,
            axes="x",
            elem_a="elem_x_shoe_left",
            elem_b="elem_x_rail_left",
            min_overlap=0.12,
            name="X slide retains rail engagement at max +X travel",
        )
        moved_x = ctx.part_world_position(x_slide)
    ctx.check(
        "X axis extends along +X",
        rest_x is not None
        and moved_x is not None
        and moved_x[0] > rest_x[0] + 0.15
        and abs(moved_x[1] - rest_x[1]) < 0.002
        and abs(moved_x[2] - rest_x[2]) < 0.002,
        details=f"rest={rest_x}, moved={moved_x}",
    )

    rest_y = ctx.part_world_position(y_runner)
    with ctx.pose({y_joint: Y_TRAVEL}):
        ctx.expect_overlap(
            y_runner,
            x_slide,
            axes="y",
            elem_a="elem_y_shoe_pos_x",
            elem_b="elem_y_rail_pos_x",
            min_overlap=0.09,
            name="Y runner retains rail engagement at max +Y travel",
        )
        moved_y = ctx.part_world_position(y_runner)
    ctx.check(
        "Y axis extends along +Y",
        rest_y is not None
        and moved_y is not None
        and moved_y[1] > rest_y[1] + 0.05
        and abs(moved_y[0] - rest_y[0]) < 0.002
        and abs(moved_y[2] - rest_y[2]) < 0.002,
        details=f"rest={rest_y}, moved={moved_y}",
    )

    rest_z = ctx.part_world_position(z_slide)
    with ctx.pose({z_joint: Z_TRAVEL}):
        ctx.expect_overlap(
            z_slide,
            y_runner,
            axes="z",
            elem_a="elem_z_shoe_left",
            elem_b="elem_z_rail_left",
            min_overlap=0.03,
            name="Z carriage retains guide engagement at max upward travel",
        )
        moved_z = ctx.part_world_position(z_slide)
    ctx.check(
        "Z axis rises along +Z",
        rest_z is not None
        and moved_z is not None
        and moved_z[2] > rest_z[2] + 0.06
        and abs(moved_z[0] - rest_z[0]) < 0.002
        and abs(moved_z[1] - rest_z[1]) < 0.002,
        details=f"rest={rest_z}, moved={moved_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
