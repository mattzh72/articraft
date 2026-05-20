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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_press")

    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    painted_green = model.material("painted_green", rgba=(0.12, 0.42, 0.31, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.22, 0.13, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="base_body",
    )
    base.visual(
        Box((0.070, 0.010, 0.160)),
        origin=Origin(xyz=(-0.050, -0.028, 0.135)),
        material=painted_green,
        name="left_guide",
    )
    base.visual(
        Box((0.070, 0.010, 0.160)),
        origin=Origin(xyz=(-0.050, 0.028, 0.135)),
        material=painted_green,
        name="right_guide",
    )
    base.visual(
        Box((0.010, 0.046, 0.160)),
        origin=Origin(xyz=(-0.080, 0.0, 0.135)),
        material=painted_green,
        name="rear_guide",
    )

    column = model.part("column")
    column.visual(
        Box((0.036, 0.028, 0.430)),
        origin=Origin(xyz=(-0.050, 0.0, 0.065)),
        material=steel,
        name="mast",
    )
    column.visual(
        Box((0.004, 0.020, 0.320)),
        origin=Origin(xyz=(-0.030, 0.0, 0.110)),
        material=dark_steel,
        name="rack_strip",
    )
    column.visual(
        Box((0.007, 0.020, 0.160)),
        origin=Origin(xyz=(-0.0715, 0.0, -0.070)),
        material=dark_steel,
        name="rear_wear_pad",
    )

    head = model.part("head")
    head.visual(
        Box((0.030, 0.080, 0.180)),
        origin=Origin(xyz=(-0.017, 0.0, 0.090)),
        material=painted_green,
        name="carriage_block",
    )
    head.visual(
        Box((0.120, 0.092, 0.090)),
        origin=Origin(xyz=(0.058, 0.0, 0.160)),
        material=painted_green,
        name="motor_housing",
    )
    head.visual(
        Box((0.110, 0.080, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, 0.225)),
        material=painted_green,
        name="gearbox_cap",
    )
    quill_sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.034, 0.020), (0.034, 0.115)],
            [(0.026, 0.020), (0.026, 0.115)],
            segments=48,
        ),
        "quill_sleeve",
    )
    head.visual(
        quill_sleeve_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=steel,
        name="quill_sleeve",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.020, 0.056, 0.110), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_hub",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.100),
        origin=Origin(xyz=(0.020, 0.101, 0.110), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_handle",
    )
    head.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.020, 0.151, 0.110)),
        material=black,
        name="feed_knob",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.0245, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=steel,
        name="quill_barrel",
    )
    quill.visual(
        Cylinder(radius=0.013, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.1975)),
        material=dark_steel,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=black,
        name="drill_bit",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.15,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.078, 0.0, 0.115)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.050,
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
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")
    column_slide = object_model.get_articulation("base_to_column")
    quill_slide = object_model.get_articulation("head_to_quill")

    ctx.expect_contact(
        head,
        column,
        elem_a="carriage_block",
        elem_b="mast",
        name="head carriage mounts directly on the mast",
    )

    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="mast",
        elem_b="left_guide",
        min_overlap=0.145,
        name="column is deeply inserted in the guide at rest",
    )
    ctx.expect_within(
        quill,
        head,
        axes="xy",
        inner_elem="quill_barrel",
        outer_elem="quill_sleeve",
        margin=0.010,
        name="quill stays centered inside the head sleeve",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        elem_a="quill_barrel",
        elem_b="quill_sleeve",
        min_overlap=0.090,
        name="quill remains retained in the sleeve at rest",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({column_slide: 0.10}):
        extended_column_pos = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="mast",
            elem_b="left_guide",
            min_overlap=0.045,
            name="column still retains insertion at maximum lift",
        )

    ctx.check(
        "column lifts upward from the magnetic base",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.09,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({quill_slide: 0.050}):
        dropped_quill_pos = ctx.part_world_position(quill)
        ctx.expect_within(
            quill,
            head,
            axes="xy",
            inner_elem="quill_barrel",
            outer_elem="quill_sleeve",
            margin=0.010,
            name="quill stays centered when fully lowered",
        )
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a="quill_barrel",
            elem_b="quill_sleeve",
            min_overlap=0.040,
            name="quill retains insertion when fully lowered",
        )
        ctx.expect_gap(
            quill,
            base,
            axis="z",
            positive_elem="drill_bit",
            negative_elem="base_body",
            min_gap=0.002,
            max_gap=0.050,
            name="lowered bit stops just above the magnetic base",
        )

    ctx.check(
        "quill drops downward out of the head",
        rest_quill_pos is not None
        and dropped_quill_pos is not None
        and dropped_quill_pos[2] < rest_quill_pos[2] - 0.040,
        details=f"rest={rest_quill_pos}, dropped={dropped_quill_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
