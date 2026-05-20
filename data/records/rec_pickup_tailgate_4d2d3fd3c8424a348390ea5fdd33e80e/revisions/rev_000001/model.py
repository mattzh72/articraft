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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_pickup_tailgate")

    painted_metal = model.material("painted_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    liner_black = model.material("liner_black", rgba=(0.13, 0.13, 0.14, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))

    opening_width = 1.48
    bed_depth = 0.40
    wall_thickness = 0.05
    floor_thickness = 0.035
    side_height = 0.56

    tailgate_width = 1.46
    tailgate_height = 0.46
    skin_thickness = 0.028
    inner_depth = 0.020
    hinge_axis_z = -0.015

    bed_body = model.part("bed_body")
    bed_body.visual(
        Box((opening_width + 2.0 * wall_thickness, bed_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.23, -floor_thickness / 2.0)),
        material=liner_black,
        name="bed_floor",
    )
    bed_body.visual(
        Box((wall_thickness, 0.34, side_height)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + wall_thickness / 2.0),
                0.19,
                side_height / 2.0,
            )
        ),
        material=painted_metal,
        name="left_bedside",
    )
    bed_body.visual(
        Box((wall_thickness, 0.34, side_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + wall_thickness / 2.0,
                0.19,
                side_height / 2.0,
            )
        ),
        material=painted_metal,
        name="right_bedside",
    )
    bed_body.visual(
        Box((opening_width + 2.0 * wall_thickness, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.07, -0.05)),
        material=liner_black,
        name="rear_crossmember",
    )
    for side_name, x_pos in (("left", -0.55), ("right", 0.55)):
        bed_body.visual(
            Box((0.12, 0.024, 0.05)),
            origin=Origin(xyz=(x_pos, 0.03, -0.010)),
            material=liner_black,
            name=f"hinge_bracket_{side_name}",
        )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((tailgate_width, skin_thickness, tailgate_height)),
        origin=Origin(xyz=(0.0, -skin_thickness / 2.0, 0.245)),
        material=painted_metal,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.34, inner_depth, 0.05)),
        origin=Origin(xyz=(0.0, inner_depth / 2.0, 0.435)),
        material=liner_black,
        name="inner_top_beam",
    )
    tailgate.visual(
        Box((1.24, inner_depth, 0.06)),
        origin=Origin(xyz=(0.0, inner_depth / 2.0, 0.045)),
        material=liner_black,
        name="inner_bottom_beam",
    )
    tailgate.visual(
        Box((0.08, inner_depth, 0.34)),
        origin=Origin(xyz=(-0.69, inner_depth / 2.0, 0.235)),
        material=liner_black,
        name="inner_left_beam",
    )
    tailgate.visual(
        Box((0.08, inner_depth, 0.34)),
        origin=Origin(xyz=(0.69, inner_depth / 2.0, 0.235)),
        material=liner_black,
        name="inner_right_beam",
    )
    tailgate.visual(
        Box((0.16, 0.018, 0.28)),
        origin=Origin(xyz=(0.0, 0.009, 0.205)),
        material=liner_black,
        name="center_reinforcement",
    )
    for side_name, x_pos in (("left", -0.55), ("right", 0.55)):
        tailgate.visual(
            Box((0.08, 0.028, 0.06)),
            origin=Origin(xyz=(x_pos, -0.014, 0.015)),
            material=liner_black,
            name=f"hinge_strap_{side_name}",
        )
        tailgate.visual(
            Cylinder(radius=0.016, length=0.07),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=liner_black,
            name=f"hinge_barrel_{side_name}",
        )

    latch_center_z = 0.40
    latch_y = -0.041
    tailgate.visual(
        Box((0.18, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, latch_y, latch_center_z + 0.027)),
        material=trim_black,
        name="latch_bezel_top",
    )
    tailgate.visual(
        Box((0.18, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, latch_y, latch_center_z - 0.027)),
        material=trim_black,
        name="latch_bezel_bottom",
    )
    tailgate.visual(
        Box((0.012, 0.026, 0.042)),
        origin=Origin(xyz=(-0.084, latch_y, latch_center_z)),
        material=trim_black,
        name="latch_bezel_left",
    )
    tailgate.visual(
        Box((0.012, 0.026, 0.042)),
        origin=Origin(xyz=(0.084, latch_y, latch_center_z)),
        material=trim_black,
        name="latch_bezel_right",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.156, 0.014, 0.024)),
        material=trim_black,
        name="button_cap",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_body,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "tailgate_to_button",
        ArticulationType.PRISMATIC,
        parent=tailgate,
        child=release_button,
        origin=Origin(xyz=(0.0, -0.050, latch_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
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

    bed_body = object_model.get_part("bed_body")
    tailgate = object_model.get_part("tailgate")
    release_button = object_model.get_part("release_button")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    button_slide = object_model.get_articulation("tailgate_to_button")

    tailgate_upper = tailgate_hinge.motion_limits.upper if tailgate_hinge.motion_limits else None
    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits else None

    ctx.check("bed body present", bed_body is not None)
    ctx.check("tailgate present", tailgate is not None)
    ctx.check("release button present", release_button is not None)

    rest_tailgate_box = ctx.part_element_world_aabb(tailgate, elem="outer_skin")
    rest_button_pos = ctx.part_world_position(release_button)

    ctx.expect_contact(
        release_button,
        tailgate,
        elem_a="button_cap",
        elem_b="latch_bezel_left",
        name="release button is guided by the left latch rail",
    )
    ctx.expect_contact(
        release_button,
        tailgate,
        elem_a="button_cap",
        elem_b="latch_bezel_right",
        name="release button is guided by the right latch rail",
    )

    with ctx.pose({tailgate_hinge: tailgate_upper}):
        open_tailgate_box = ctx.part_element_world_aabb(tailgate, elem="outer_skin")

    ctx.check(
        "tailgate drops downward and rearward when opened",
        rest_tailgate_box is not None
        and open_tailgate_box is not None
        and tailgate_upper is not None
        and open_tailgate_box[0][1] < rest_tailgate_box[0][1] - 0.28
        and open_tailgate_box[1][2] < rest_tailgate_box[1][2] - 0.18,
        details=f"rest={rest_tailgate_box}, open={open_tailgate_box}, q={tailgate_upper}",
    )

    with ctx.pose({button_slide: button_upper}):
        pressed_button_pos = ctx.part_world_position(release_button)
        ctx.expect_gap(
            tailgate,
            release_button,
            axis="y",
            positive_elem="outer_skin",
            negative_elem="button_cap",
            min_gap=0.0045,
            max_gap=0.0055,
            name="pressed button stays clear of the tailgate skin",
        )

    ctx.check(
        "release button travels inward into the latch housing",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and button_upper is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.008,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}, q={button_upper}",
    )

    if rest_tailgate_box is not None and rest_button_pos is not None:
        tailgate_center_x = 0.5 * (rest_tailgate_box[0][0] + rest_tailgate_box[1][0])
        tailgate_top_z = rest_tailgate_box[1][2]
        tailgate_bottom_z = rest_tailgate_box[0][2]
        ctx.check(
            "release button sits near the top center of the tailgate",
            abs(rest_button_pos[0] - tailgate_center_x) < 0.02
            and rest_button_pos[2] > tailgate_bottom_z + 0.72 * (tailgate_top_z - tailgate_bottom_z),
            details=f"tailgate_box={rest_tailgate_box}, button_pos={rest_button_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
