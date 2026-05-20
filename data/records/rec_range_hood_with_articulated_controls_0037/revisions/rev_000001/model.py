from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


BODY_WIDTH = 0.90
BODY_DEPTH = 0.46
BODY_HEIGHT = 0.18

CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.24
CHIMNEY_HEIGHT = 0.62

PANEL_WIDTH = 0.48
PANEL_HEIGHT = 0.14
PANEL_THICKNESS = 0.012

BUTTON_RADIUS = 0.010
BUTTON_TRAVEL = 0.004
BUTTON_POSITIONS = (
    (-0.152, 0.024),
    (-0.076, 0.012),
    (0.000, 0.000),
    (0.076, -0.012),
    (0.152, -0.024),
)
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.75, 0.77, 0.79, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    slat_dark = model.material("slat_dark", rgba=(0.26, 0.27, 0.29, 1.0))
    lamp_lens = model.material("lamp_lens", rgba=(0.94, 0.90, 0.74, 0.55))
    button_black = model.material("button_black", rgba=(0.08, 0.08, 0.09, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.010)),
        material=brushed_steel,
        name="canopy_top",
    )
    hood_body.visual(
        Box((BODY_WIDTH, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH * 0.5) - 0.010, 0.010)),
        material=brushed_steel,
        name="front_lower_lip",
    )
    hood_body.visual(
        Box((PANEL_WIDTH, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH * 0.5) - 0.010, BODY_HEIGHT - 0.010)),
        material=brushed_steel,
        name="front_upper_lip",
    )
    hood_body.visual(
        Box((0.210, 0.020, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(-0.345, (BODY_DEPTH * 0.5) - 0.010, (BODY_HEIGHT * 0.5) - 0.010)),
        material=brushed_steel,
        name="front_left_cheek",
    )
    hood_body.visual(
        Box((0.210, 0.020, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.345, (BODY_DEPTH * 0.5) - 0.010, (BODY_HEIGHT * 0.5) - 0.010)),
        material=brushed_steel,
        name="front_right_cheek",
    )
    hood_body.visual(
        Box((BODY_WIDTH, 0.020, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5) + 0.010, (BODY_HEIGHT * 0.5) - 0.010)),
        material=brushed_steel,
        name="rear_skirt",
    )
    hood_body.visual(
        Box((0.020, BODY_DEPTH - 0.040, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(-(BODY_WIDTH * 0.5) + 0.010, 0.0, (BODY_HEIGHT * 0.5) - 0.010)),
        material=brushed_steel,
        name="left_skirt",
    )
    hood_body.visual(
        Box((0.020, BODY_DEPTH - 0.040, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=((BODY_WIDTH * 0.5) - 0.010, 0.0, (BODY_HEIGHT * 0.5) - 0.010)),
        material=brushed_steel,
        name="right_skirt",
    )
    hood_body.visual(
        Box((0.82, 0.36, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=filter_dark,
        name="grease_filter",
    )
    hood_body.visual(
        Box((0.020, 0.36, 0.012)),
        origin=Origin(xyz=(-0.420, 0.0, 0.012)),
        material=slat_dark,
        name="left_filter_rail",
    )
    hood_body.visual(
        Box((0.020, 0.36, 0.012)),
        origin=Origin(xyz=(0.420, 0.0, 0.012)),
        material=slat_dark,
        name="right_filter_rail",
    )
    for slat_index, y_pos in enumerate((-0.12, -0.04, 0.04, 0.12), start=1):
        hood_body.visual(
            Box((0.74, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, y_pos, 0.0175)),
            material=slat_dark,
            name=f"filter_slat_{slat_index}",
        )
    hood_body.visual(
        Box((0.060, 0.032, 0.003)),
        origin=Origin(xyz=(-0.255, 0.110, 0.0175)),
        material=lamp_lens,
        name="left_light",
    )
    hood_body.visual(
        Box((0.060, 0.032, 0.003)),
        origin=Origin(xyz=(0.255, 0.110, 0.0175)),
        material=lamp_lens,
        name="right_light",
    )
    hood_body.visual(
        Box((0.620, 0.122, 0.005)),
        origin=Origin(xyz=(0.0, 0.159, 0.0205)),
        material=slat_dark,
        name="lighting_plenum",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_cover.visual(
        Box((CHIMNEY_WIDTH, 0.015, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, (CHIMNEY_DEPTH * 0.5) - 0.0075, CHIMNEY_HEIGHT * 0.5)),
        material=satin_steel,
        name="chimney_front",
    )
    chimney_cover.visual(
        Box((CHIMNEY_WIDTH, 0.015, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(CHIMNEY_DEPTH * 0.5) + 0.0075, CHIMNEY_HEIGHT * 0.5)),
        material=satin_steel,
        name="chimney_back",
    )
    chimney_cover.visual(
        Box((0.015, CHIMNEY_DEPTH - 0.030, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(-(CHIMNEY_WIDTH * 0.5) + 0.0075, 0.0, CHIMNEY_HEIGHT * 0.5)),
        material=satin_steel,
        name="chimney_left",
    )
    chimney_cover.visual(
        Box((0.015, CHIMNEY_DEPTH - 0.030, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=((CHIMNEY_WIDTH * 0.5) - 0.0075, 0.0, CHIMNEY_HEIGHT * 0.5)),
        material=satin_steel,
        name="chimney_right",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT * 0.5)),
    )
    model.articulation(
        "hood_to_chimney",
        ArticulationType.FIXED,
        parent=hood_body,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, -0.070, BODY_HEIGHT)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((PANEL_WIDTH, 0.003, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=satin_steel,
        name="panel_backing",
    )
    control_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=satin_steel,
        name="panel_top_rail",
    )
    control_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=satin_steel,
        name="panel_bottom_rail",
    )
    control_panel.visual(
        Box((0.036, PANEL_THICKNESS, PANEL_HEIGHT - 0.048)),
        origin=Origin(xyz=(-0.222, 0.0, 0.0)),
        material=satin_steel,
        name="panel_left_rail",
    )
    control_panel.visual(
        Box((0.036, PANEL_THICKNESS, PANEL_HEIGHT - 0.048)),
        origin=Origin(xyz=(0.222, 0.0, 0.0)),
        material=satin_steel,
        name="panel_right_rail",
    )
    for index, (x_pos, z_pos) in enumerate(BUTTON_POSITIONS, start=1):
        control_panel.visual(
            Box((0.036, PANEL_THICKNESS, 0.005)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos + 0.015)),
            material=satin_steel,
            name=f"button_{index}_bezel_top",
        )
        control_panel.visual(
            Box((0.036, PANEL_THICKNESS, 0.005)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos - 0.015)),
            material=satin_steel,
            name=f"button_{index}_bezel_bottom",
        )
        control_panel.visual(
            Box((0.005, PANEL_THICKNESS, 0.030)),
            origin=Origin(xyz=(x_pos - 0.015, 0.0, z_pos)),
            material=satin_steel,
            name=f"button_{index}_bezel_left",
        )
        control_panel.visual(
            Box((0.005, PANEL_THICKNESS, 0.030)),
            origin=Origin(xyz=(x_pos + 0.015, 0.0, z_pos)),
            material=satin_steel,
            name=f"button_{index}_bezel_right",
        )
    control_panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        mass=0.8,
    )
    model.articulation(
        "hood_to_control_panel",
        ArticulationType.FIXED,
        parent=hood_body,
        child=control_panel,
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH * 0.5) - (PANEL_THICKNESS * 0.5),
                BODY_HEIGHT * 0.5,
            )
        ),
    )

    for index, (x_pos, z_pos) in enumerate(BUTTON_POSITIONS, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=0.006),
            origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=0.008),
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_black,
            name="button_guide",
        )
        button.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_black,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=BUTTON_RADIUS, length=0.020),
            mass=0.03,
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(x_pos, PANEL_THICKNESS * 0.5, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    chimney_cover = object_model.get_part("chimney_cover")
    control_panel = object_model.get_part("control_panel")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    button_joints = [
        object_model.get_articulation(f"control_panel_to_button_{index}")
        for index in range(1, 6)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(chimney_cover, hood_body, name="chimney_seated_on_canopy")
    ctx.expect_contact(control_panel, hood_body, name="front_panel_seated")
    ctx.expect_within(control_panel, hood_body, axes="xz", margin=0.0, name="front_panel_within_canopy")

    hood_aabb = ctx.part_world_aabb(hood_body)
    chimney_aabb = ctx.part_world_aabb(chimney_cover)
    panel_aabb = ctx.part_world_aabb(control_panel)
    assert hood_aabb is not None
    assert chimney_aabb is not None
    assert panel_aabb is not None

    hood_width = hood_aabb[1][0] - hood_aabb[0][0]
    hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
    hood_height = hood_aabb[1][2] - hood_aabb[0][2]
    chimney_width = chimney_aabb[1][0] - chimney_aabb[0][0]
    chimney_height = chimney_aabb[1][2] - chimney_aabb[0][2]
    panel_front = panel_aabb[1][1]
    hood_front = hood_aabb[1][1]

    ctx.check(
        "hood_canopy_realistic_size",
        0.84 <= hood_width <= 0.96 and 0.42 <= hood_depth <= 0.50 and 0.16 <= hood_height <= 0.20,
        details=(
            f"expected a broad canopy near 0.90 x 0.46 x 0.18 m, got "
            f"{hood_width:.3f} x {hood_depth:.3f} x {hood_height:.3f} m"
        ),
    )
    ctx.check(
        "chimney_taller_and_narrower_than_canopy",
        chimney_height > 0.55 and chimney_width < hood_width * 0.45,
        details=(
            f"expected chimney about 0.62 m tall and much narrower than canopy, got "
            f"height={chimney_height:.3f}, width={chimney_width:.3f}"
        ),
    )
    ctx.check(
        "front_panel_flush_with_body_face",
        abs(panel_front - hood_front) <= 0.0015,
        details=f"expected panel front {panel_front:.4f} to be flush with hood front {hood_front:.4f}",
    )

    rest_positions = []
    for index, (button, joint) in enumerate(zip(buttons, button_joints), start=1):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"{joint.name} should be prismatic, got {joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis_normal_to_panel",
            tuple(float(value) for value in joint.axis) == (0.0, -1.0, 0.0),
            details=f"{joint.name} axis should be (0, -1, 0), got {joint.axis}",
        )

        rest_position = ctx.part_world_position(button)
        assert rest_position is not None
        rest_positions.append(rest_position)

        ctx.expect_contact(button, control_panel, name=f"button_{index}_mounted_to_panel")
        ctx.expect_within(button, control_panel, axes="xz", margin=0.0, name=f"button_{index}_within_panel_bounds")

        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")

        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.check(
                f"{joint.name}_presses_inward",
                (
                    pressed_position[1] < rest_position[1] - 0.0032
                    and pressed_position[1] > rest_position[1] - 0.0048
                ),
                details=(
                    f"expected button {index} to move inward about 4 mm along -y, "
                    f"rest={rest_position[1]:.4f}, pressed={pressed_position[1]:.4f}"
                ),
            )
            ctx.check(
                f"{joint.name}_keeps_stair_step_alignment",
                abs(pressed_position[0] - rest_position[0]) <= 1e-6
                and abs(pressed_position[2] - rest_position[2]) <= 1e-6,
                details=(
                    f"button {index} should only translate normal to the panel, "
                    f"rest={rest_position}, pressed={pressed_position}"
                ),
            )
            ctx.expect_contact(button, control_panel, name=f"{joint.name}_pressed_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    ctx.check(
        "buttons_step_down_left_to_right",
        all(rest_positions[i + 1][2] < rest_positions[i][2] - 0.006 for i in range(4)),
        details=f"expected descending z positions for buttons, got {rest_positions}",
    )
    ctx.check(
        "buttons_progress_left_to_right",
        all(rest_positions[i + 1][0] > rest_positions[i][0] + 0.05 for i in range(4)),
        details=f"expected increasing x positions for buttons, got {rest_positions}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
