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

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.22
CANOPY_WALL = 0.012

CHIMNEY_WALL = 0.008
CHIMNEY_LOWER_WIDTH = 0.34
CHIMNEY_LOWER_DEPTH = 0.26
CHIMNEY_LOWER_HEIGHT = 0.56
CHIMNEY_LOWER_CENTER_Y = 0.12

CHIMNEY_UPPER_WIDTH = 0.28
CHIMNEY_UPPER_DEPTH = 0.20
CHIMNEY_UPPER_HEIGHT = 0.52
CHIMNEY_UPPER_INSERT_Z = 0.18
CHIMNEY_UPPER_CENTER_Y = 0.126

CONTROL_WIDTH = 0.19
CONTROL_HEIGHT = 0.15
CONTROL_DEPTH = 0.035
CONTROL_WALL = 0.006
CONTROL_CENTER_X = 0.032
CONTROL_CENTER_Z = 0.122
CONTROL_FRONT_Y = -CANOPY_DEPTH / 2.0 + 0.002

KNOB_RADIUS = 0.022
KNOB_BODY_LENGTH = 0.025
KNOB_SHAFT_RADIUS = 0.004
KNOB_SHAFT_LENGTH = 0.032
KNOB_LOCAL_X = -CONTROL_CENTER_X
KNOB_LOCAL_Z = 0.008

BUTTON_CAP_WIDTH = 0.018
BUTTON_CAP_DEPTH = 0.008
BUTTON_CAP_HEIGHT = 0.012
BUTTON_STEM_WIDTH = 0.010
BUTTON_STEM_DEPTH = 0.026
BUTTON_STEM_HEIGHT = 0.006
BUTTON_TRAVEL = 0.006
BUTTON_LOCAL_X = 0.053
BUTTON_LOCAL_ZS = (0.036, 0.006, -0.024, -0.054)


def _add_open_sleeve(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    material,
    name_prefix: str,
) -> None:
    half_width = width / 2.0
    half_depth = depth / 2.0

    part.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-half_width + wall / 2.0, 0.0, height / 2.0)),
        material=material,
        name=f"{name_prefix}_left_wall",
    )
    part.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(half_width - wall / 2.0, 0.0, height / 2.0)),
        material=material,
        name=f"{name_prefix}_right_wall",
    )
    part.visual(
        Box((width - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, -half_depth + wall / 2.0, height / 2.0)),
        material=material,
        name=f"{name_prefix}_front_wall",
    )
    part.visual(
        Box((width - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, half_depth - wall / 2.0, height / 2.0)),
        material=material,
        name=f"{name_prefix}_back_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    filter_black = model.material("filter_black", rgba=(0.14, 0.15, 0.16, 1.0))

    canopy = model.part("canopy")

    opening_left = CONTROL_CENTER_X - CONTROL_WIDTH / 2.0
    opening_right = CONTROL_CENTER_X + CONTROL_WIDTH / 2.0
    opening_bottom = CONTROL_CENTER_Z - CONTROL_HEIGHT / 2.0
    opening_top = CONTROL_CENTER_Z + CONTROL_HEIGHT / 2.0
    front_wall_y = -CANOPY_DEPTH / 2.0 + CANOPY_WALL / 2.0

    canopy.visual(
        Box((CANOPY_WALL, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(-CANOPY_WIDTH / 2.0 + CANOPY_WALL / 2.0, 0.0, CANOPY_HEIGHT / 2.0)
        ),
        material=stainless,
        name="left_side",
    )
    canopy.visual(
        Box((CANOPY_WALL, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(CANOPY_WIDTH / 2.0 - CANOPY_WALL / 2.0, 0.0, CANOPY_HEIGHT / 2.0)
        ),
        material=stainless,
        name="right_side",
    )
    canopy.visual(
        Box((CANOPY_WIDTH - 2.0 * CANOPY_WALL, CANOPY_WALL, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CANOPY_DEPTH / 2.0 - CANOPY_WALL / 2.0, CANOPY_HEIGHT / 2.0)
        ),
        material=stainless,
        name="back_panel",
    )
    canopy.visual(
        Box((CANOPY_WIDTH - 2.0 * CANOPY_WALL, CANOPY_DEPTH, CANOPY_WALL)),
        origin=Origin(
            xyz=(0.0, 0.0, CANOPY_HEIGHT - CANOPY_WALL / 2.0)
        ),
        material=stainless,
        name="top_panel",
    )

    left_front_width = opening_left + CANOPY_WIDTH / 2.0 - CANOPY_WALL
    right_front_width = CANOPY_WIDTH / 2.0 - opening_right - CANOPY_WALL
    if left_front_width > 0.0:
        canopy.visual(
            Box((left_front_width, CANOPY_WALL, CANOPY_HEIGHT)),
            origin=Origin(
                xyz=(
                    -CANOPY_WIDTH / 2.0 + CANOPY_WALL + left_front_width / 2.0,
                    front_wall_y,
                    CANOPY_HEIGHT / 2.0,
                )
            ),
            material=stainless,
            name="front_left_panel",
        )
    if right_front_width > 0.0:
        canopy.visual(
            Box((right_front_width, CANOPY_WALL, CANOPY_HEIGHT)),
            origin=Origin(
                xyz=(
                    opening_right + right_front_width / 2.0,
                    front_wall_y,
                    CANOPY_HEIGHT / 2.0,
                )
            ),
            material=stainless,
            name="front_right_panel",
        )

    top_strip_height = CANOPY_HEIGHT - opening_top
    bottom_strip_height = opening_bottom
    canopy.visual(
        Box((CONTROL_WIDTH, CANOPY_WALL, top_strip_height)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X,
                front_wall_y,
                opening_top + top_strip_height / 2.0,
            )
        ),
        material=stainless,
        name="front_top_strip",
    )
    canopy.visual(
        Box((CONTROL_WIDTH, CANOPY_WALL, bottom_strip_height)),
        origin=Origin(
            xyz=(CONTROL_CENTER_X, front_wall_y, bottom_strip_height / 2.0)
        ),
        material=stainless,
        name="front_bottom_strip",
    )

    canopy.visual(
        Box((0.27, 0.20, 0.004)),
        origin=Origin(xyz=(-0.145, -0.045, CANOPY_HEIGHT - CANOPY_WALL - 0.002)),
        material=filter_black,
        name="left_filter",
    )
    canopy.visual(
        Box((0.27, 0.20, 0.004)),
        origin=Origin(xyz=(0.145, -0.045, CANOPY_HEIGHT - CANOPY_WALL - 0.002)),
        material=filter_black,
        name="right_filter",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT / 2.0)),
    )

    chimney_lower = model.part("chimney_lower")
    _add_open_sleeve(
        chimney_lower,
        width=CHIMNEY_LOWER_WIDTH,
        depth=CHIMNEY_LOWER_DEPTH,
        height=CHIMNEY_LOWER_HEIGHT,
        wall=CHIMNEY_WALL,
        material=stainless,
        name_prefix="lower_chimney",
    )
    chimney_lower.inertial = Inertial.from_geometry(
        Box((CHIMNEY_LOWER_WIDTH, CHIMNEY_LOWER_DEPTH, CHIMNEY_LOWER_HEIGHT)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_LOWER_HEIGHT / 2.0)),
    )
    model.articulation(
        "canopy_to_chimney_lower",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney_lower,
        origin=Origin(xyz=(0.0, CHIMNEY_LOWER_CENTER_Y, CANOPY_HEIGHT)),
    )

    chimney_upper = model.part("chimney_upper")
    _add_open_sleeve(
        chimney_upper,
        width=CHIMNEY_UPPER_WIDTH,
        depth=CHIMNEY_UPPER_DEPTH,
        height=CHIMNEY_UPPER_HEIGHT,
        wall=CHIMNEY_WALL,
        material=stainless,
        name_prefix="upper_chimney",
    )
    chimney_upper.visual(
        Box((0.16, 0.016, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_UPPER_DEPTH / 2.0 + 0.008,
                0.024,
            )
        ),
        material=stainless,
        name="rear_mounting_bridge",
    )
    chimney_upper.inertial = Inertial.from_geometry(
        Box((CHIMNEY_UPPER_WIDTH, CHIMNEY_UPPER_DEPTH, CHIMNEY_UPPER_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_UPPER_HEIGHT / 2.0)),
    )
    model.articulation(
        "chimney_lower_to_chimney_upper",
        ArticulationType.FIXED,
        parent=chimney_lower,
        child=chimney_upper,
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_UPPER_CENTER_Y - CHIMNEY_LOWER_CENTER_Y,
                CHIMNEY_UPPER_INSERT_Z,
            )
        ),
    )

    control_module = model.part("control_module")
    control_module.visual(
        Box((CONTROL_WIDTH, 0.003, CONTROL_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_DEPTH - 0.0015, 0.0)),
        material=control_black,
        name="back_plate",
    )
    control_module.visual(
        Box((CONTROL_WALL, CONTROL_DEPTH, CONTROL_HEIGHT)),
        origin=Origin(
            xyz=(-CONTROL_WIDTH / 2.0 + CONTROL_WALL / 2.0, CONTROL_DEPTH / 2.0, 0.0)
        ),
        material=control_black,
        name="left_wall",
    )
    control_module.visual(
        Box((CONTROL_WALL, CONTROL_DEPTH, CONTROL_HEIGHT)),
        origin=Origin(
            xyz=(CONTROL_WIDTH / 2.0 - CONTROL_WALL / 2.0, CONTROL_DEPTH / 2.0, 0.0)
        ),
        material=control_black,
        name="right_wall",
    )
    control_module.visual(
        Box((CONTROL_WIDTH, CONTROL_DEPTH, CONTROL_WALL)),
        origin=Origin(
            xyz=(0.0, CONTROL_DEPTH / 2.0, CONTROL_HEIGHT / 2.0 - CONTROL_WALL / 2.0)
        ),
        material=control_black,
        name="top_wall",
    )
    control_module.visual(
        Box((CONTROL_WIDTH, CONTROL_DEPTH, CONTROL_WALL)),
        origin=Origin(
            xyz=(0.0, CONTROL_DEPTH / 2.0, -CONTROL_HEIGHT / 2.0 + CONTROL_WALL / 2.0)
        ),
        material=control_black,
        name="bottom_wall",
    )

    guide_rail_thickness = 0.002
    guide_depth = 0.032
    for index, local_z in enumerate(BUTTON_LOCAL_ZS, start=1):
        control_module.visual(
            Box(
                (
                    guide_rail_thickness,
                    guide_depth,
                    BUTTON_STEM_HEIGHT + 2.0 * guide_rail_thickness,
                )
            ),
            origin=Origin(
                xyz=(
                    BUTTON_LOCAL_X - BUTTON_STEM_WIDTH / 2.0 - guide_rail_thickness / 2.0,
                    guide_depth / 2.0,
                    local_z,
                )
            ),
            material=satin_dark,
            name=f"button_{index}_left_guide",
        )
        control_module.visual(
            Box(
                (
                    guide_rail_thickness,
                    guide_depth,
                    BUTTON_STEM_HEIGHT + 2.0 * guide_rail_thickness,
                )
            ),
            origin=Origin(
                xyz=(
                    BUTTON_LOCAL_X + BUTTON_STEM_WIDTH / 2.0 + guide_rail_thickness / 2.0,
                    guide_depth / 2.0,
                    local_z,
                )
            ),
            material=satin_dark,
            name=f"button_{index}_right_guide",
        )
        control_module.visual(
            Box(
                (
                    BUTTON_STEM_WIDTH + 2.0 * guide_rail_thickness,
                    guide_depth,
                    guide_rail_thickness,
                )
            ),
            origin=Origin(
                xyz=(
                    BUTTON_LOCAL_X,
                    guide_depth / 2.0,
                    local_z + BUTTON_STEM_HEIGHT / 2.0 + guide_rail_thickness / 2.0,
                )
            ),
            material=satin_dark,
            name=f"button_{index}_top_guide",
        )
        control_module.visual(
            Box(
                (
                    BUTTON_STEM_WIDTH + 2.0 * guide_rail_thickness,
                    guide_depth,
                    guide_rail_thickness,
                )
            ),
            origin=Origin(
                xyz=(
                    BUTTON_LOCAL_X,
                    guide_depth / 2.0,
                    local_z - BUTTON_STEM_HEIGHT / 2.0 - guide_rail_thickness / 2.0,
                )
            ),
            material=satin_dark,
            name=f"button_{index}_bottom_guide",
        )
    control_module.inertial = Inertial.from_geometry(
        Box((CONTROL_WIDTH, CONTROL_DEPTH, CONTROL_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(0.0, CONTROL_DEPTH / 2.0, 0.0)),
    )
    model.articulation(
        "canopy_to_control_module",
        ArticulationType.FIXED,
        parent=canopy,
        child=control_module,
        origin=Origin(xyz=(CONTROL_CENTER_X, CONTROL_FRONT_Y, CONTROL_CENTER_Z)),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_BODY_LENGTH),
        origin=Origin(
            xyz=(0.0, -KNOB_BODY_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_dark,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, KNOB_SHAFT_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_dark,
        name="knob_shaft",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.050, KNOB_BODY_LENGTH + KNOB_SHAFT_LENGTH, 0.050)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
    )
    model.articulation(
        "control_module_to_knob",
        ArticulationType.CONTINUOUS,
        parent=control_module,
        child=knob,
        origin=Origin(xyz=(KNOB_LOCAL_X, 0.0, KNOB_LOCAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    for index, local_z in enumerate(BUTTON_LOCAL_ZS, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            origin=Origin(xyz=(0.0, -BUTTON_CAP_DEPTH / 2.0, 0.0)),
            material=satin_dark,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, BUTTON_STEM_DEPTH / 2.0, 0.0)),
            material=satin_dark,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH + BUTTON_STEM_DEPTH, BUTTON_CAP_HEIGHT)),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.009, 0.0)),
        )
        model.articulation(
            f"control_module_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_module,
            child=button,
            origin=Origin(xyz=(BUTTON_LOCAL_X, 0.0, local_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy")
    chimney_lower = object_model.get_part("chimney_lower")
    chimney_upper = object_model.get_part("chimney_upper")
    control_module = object_model.get_part("control_module")
    knob = object_model.get_part("control_knob")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 5)]

    knob_joint = object_model.get_articulation("control_module_to_knob")
    button_joints = [
        object_model.get_articulation(f"control_module_to_button_{index}")
        for index in range(1, 5)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(chimney_lower, canopy, name="chimney_lower_seated_on_canopy")
    ctx.expect_contact(chimney_upper, chimney_lower, name="chimney_upper_supported_by_lower")
    ctx.expect_contact(control_module, canopy, name="control_module_mounted_in_opening")
    ctx.expect_contact(knob, control_module, name="knob_mounted_to_control_module")
    ctx.expect_within(
        chimney_upper,
        chimney_lower,
        axes="xy",
        margin=0.0,
        name="upper_chimney_nested_within_lower",
    )

    canopy_aabb = ctx.part_world_aabb(canopy)
    upper_aabb = ctx.part_world_aabb(chimney_upper)
    assert canopy_aabb is not None
    assert upper_aabb is not None
    canopy_width = canopy_aabb[1][0] - canopy_aabb[0][0]
    canopy_depth = canopy_aabb[1][1] - canopy_aabb[0][1]
    total_height = upper_aabb[1][2] - canopy_aabb[0][2]
    ctx.check(
        "canopy_width_realistic",
        0.85 <= canopy_width <= 0.95,
        f"canopy width was {canopy_width:.3f} m",
    )
    ctx.check(
        "canopy_depth_realistic",
        0.46 <= canopy_depth <= 0.54,
        f"canopy depth was {canopy_depth:.3f} m",
    )
    ctx.check(
        "total_height_realistic",
        0.90 <= total_height <= 1.05,
        f"overall height was {total_height:.3f} m",
    )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"knob articulation type was {knob_joint.articulation_type!r}",
    )
    ctx.check(
        "knob_rotates_about_front_normal",
        tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
        f"knob axis was {knob_joint.axis!r}",
    )
    ctx.check(
        "knob_has_unbounded_continuous_limits",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        f"knob limits were {knob_limits!r}",
    )

    with ctx.pose({knob_joint: math.pi}):
        ctx.expect_contact(knob, control_module, name="knob_contact_when_rotated")
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotated_no_floating")

    for index, (button, button_joint) in enumerate(zip(buttons, button_joints, strict=True), start=1):
        limits = button_joint.motion_limits
        ctx.expect_contact(
            button,
            control_module,
            name=f"button_{index}_mounted_to_guides",
        )
        ctx.expect_within(
            button,
            control_module,
            axes="xz",
            margin=0.001,
            name=f"button_{index}_stays_in_control_module_footprint",
        )
        ctx.check(
            f"button_{index}_is_prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            f"button articulation type was {button_joint.articulation_type!r}",
        )
        ctx.check(
            f"button_{index}_moves_inward",
            tuple(round(value, 6) for value in button_joint.axis) == (0.0, 1.0, 0.0),
            f"button axis was {button_joint.axis!r}",
        )
        ctx.check(
            f"button_{index}_travel_realistic",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.004 <= limits.upper <= 0.008,
            f"button limits were {limits!r}",
        )

        rest_position = ctx.part_world_position(button)
        assert rest_position is not None
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        with ctx.pose({button_joint: limits.lower}):
            ctx.expect_contact(
                button,
                control_module,
                name=f"button_{index}_lower_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"button_{index}_lower_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"button_{index}_lower_no_floating")

        with ctx.pose({button_joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.check(
                f"button_{index}_advances_inward_at_upper_limit",
                pressed_position[1] > rest_position[1] + 0.004,
                f"rest y={rest_position[1]:.4f}, pressed y={pressed_position[1]:.4f}",
            )
            ctx.expect_contact(
                button,
                control_module,
                name=f"button_{index}_upper_contact",
            )
            ctx.expect_within(
                button,
                control_module,
                axes="xz",
                margin=0.001,
                name=f"button_{index}_upper_within_guides",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"button_{index}_upper_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"button_{index}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
