from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


HOUSING_WIDTH = 0.34
HOUSING_DEPTH = 0.23
BODY_HEIGHT = 0.038
TOP_PLATE_HEIGHT = 0.006
TOP_Z = BODY_HEIGHT + TOP_PLATE_HEIGHT

KNOB_RADIUS = 0.023
KNOB_POSITIONS = (
    (-0.115, -0.060),
    (-0.055, -0.060),
    (0.005, -0.060),
    (0.065, -0.060),
)


def _add_encoder_knob(
    model: ArticulatedObject,
    housing,
    name: str,
    origin_xyz: tuple[float, float, float],
) -> None:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="panel_gray",
        name="encoder_shaft",
    )
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="knob_dark",
        name="encoder_skirt",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="panel_gray",
        name="encoder_cap",
    )
    knob.visual(
        Box((0.003, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.013, 0.039)),
        material="screen_accent",
        name="encoder_indicator",
    )

    model.articulation(
        f"housing_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=origin_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_sampler")

    model.material("chassis_black", color=(0.11, 0.11, 0.12))
    model.material("panel_gray", color=(0.19, 0.19, 0.21))
    model.material("knob_dark", color=(0.14, 0.14, 0.15))
    model.material("rubber_pad", color=(0.24, 0.24, 0.25))
    model.material("screen_bezel", color=(0.10, 0.10, 0.11))
    model.material("screen_glow", color=(0.16, 0.44, 0.58))
    model.material("screen_accent", color=(0.78, 0.78, 0.80))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material="chassis_black",
        name="body_shell",
    )
    housing.visual(
        Box((0.33, 0.22, TOP_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + TOP_PLATE_HEIGHT / 2.0)),
        material="panel_gray",
        name="top_plate",
    )
    housing.visual(
        Box((0.21, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.072, 0.047)),
        material="panel_gray",
        name="display_riser",
    )
    housing.visual(
        Box((0.018, 0.035, 0.024)),
        origin=Origin(xyz=(-0.101, 0.071, 0.050)),
        material="panel_gray",
        name="left_display_cheek",
    )
    housing.visual(
        Box((0.018, 0.035, 0.024)),
        origin=Origin(xyz=(0.101, 0.071, 0.050)),
        material="panel_gray",
        name="right_display_cheek",
    )

    for index, (x_pos, y_pos) in enumerate(KNOB_POSITIONS, start=1):
        housing.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, TOP_Z + 0.002)),
            material="panel_gray",
            name=f"encoder_collar_{index}",
        )

    pad_columns = (0.102, 0.138)
    pad_rows = (-0.072, -0.036, 0.0, 0.036)
    for row_index, y_pos in enumerate(pad_rows, start=1):
        for col_index, x_pos in enumerate(pad_columns, start=1):
            housing.visual(
                Box((0.028, 0.028, 0.004)),
                origin=Origin(xyz=(x_pos, y_pos, TOP_Z + 0.002)),
                material="rubber_pad",
                name=f"pad_{row_index}_{col_index}",
            )

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(
        Box((0.170, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="panel_gray",
        name="hinge_bar",
    )
    lcd_panel.visual(
        Box((0.182, 0.016, 0.105)),
        origin=Origin(xyz=(0.0, -0.006, 0.052), rpy=(-0.72, 0.0, 0.0)),
        material="screen_bezel",
        name="lcd_bezel",
    )
    lcd_panel.visual(
        Box((0.050, 0.020, 0.062)),
        origin=Origin(xyz=(0.0, -0.006, 0.031), rpy=(-0.72, 0.0, 0.0)),
        material="panel_gray",
        name="lcd_back_spine",
    )
    lcd_panel.visual(
        Box((0.050, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="panel_gray",
        name="lcd_hinge_bridge",
    )
    lcd_panel.visual(
        Box((0.148, 0.005, 0.076)),
        origin=Origin(xyz=(0.0, -0.0125, 0.054), rpy=(-0.72, 0.0, 0.0)),
        material="screen_glow",
        name="lcd_screen",
    )
    lcd_panel.visual(
        Box((0.182, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.017), rpy=(-0.72, 0.0, 0.0)),
        material="panel_gray",
        name="lcd_chin",
    )

    model.articulation(
        "housing_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lcd_panel,
        origin=Origin(xyz=(0.0, 0.047, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=0.62),
    )

    for index, (x_pos, y_pos) in enumerate(KNOB_POSITIONS, start=1):
        _add_encoder_knob(
            model,
            housing,
            f"encoder_{index}",
            (x_pos, y_pos, TOP_Z),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lcd_panel = object_model.get_part("lcd_panel")
    lcd_joint = object_model.get_articulation("housing_to_lcd_panel")

    encoder_parts = [
        object_model.get_part(f"encoder_{index}")
        for index in range(1, 5)
    ]
    encoder_joints = [
        object_model.get_articulation(f"housing_to_encoder_{index}")
        for index in range(1, 5)
    ]

    ctx.check(
        "lcd hinge is a bounded tilt joint",
        lcd_joint.articulation_type == ArticulationType.REVOLUTE
        and lcd_joint.motion_limits is not None
        and lcd_joint.motion_limits.lower == 0.0
        and lcd_joint.motion_limits.upper is not None
        and lcd_joint.motion_limits.upper >= 0.5,
        details=(
            f"type={lcd_joint.articulation_type}, "
            f"limits={lcd_joint.motion_limits}"
        ),
    )

    for index, joint in enumerate(encoder_joints, start=1):
        ctx.check(
            f"encoder {index} rotates continuously",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None,
            details=(
                f"type={joint.articulation_type}, "
                f"limits={joint.motion_limits}"
            ),
        )

    with ctx.pose({lcd_joint: 0.0}):
        ctx.expect_contact(
            lcd_panel,
            housing,
            elem_a="hinge_bar",
            elem_b="display_riser",
            name="lcd hinge bar seats on the riser",
        )
        ctx.expect_overlap(
            lcd_panel,
            housing,
            axes="x",
            min_overlap=0.16,
            name="lcd panel stays centered over the housing",
        )
        closed_lcd_aabb = ctx.part_world_aabb(lcd_panel)

        for index, knob in enumerate(encoder_parts, start=1):
            ctx.expect_contact(
                knob,
                housing,
                elem_a="encoder_shaft",
                elem_b="top_plate",
                name=f"encoder {index} seats on the top panel",
            )
            ctx.expect_overlap(
                knob,
                housing,
                axes="xy",
                min_overlap=0.035,
                name=f"encoder {index} overlaps the top-panel footprint",
            )

    with ctx.pose({lcd_joint: 0.62}):
        raised_lcd_aabb = ctx.part_world_aabb(lcd_panel)

    ctx.check(
        "lcd panel tilts toward the operator when opened",
        closed_lcd_aabb is not None
        and raised_lcd_aabb is not None
        and raised_lcd_aabb[0][1] < closed_lcd_aabb[0][1] - 0.03,
        details=f"closed={closed_lcd_aabb}, raised={raised_lcd_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
