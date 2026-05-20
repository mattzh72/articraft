from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    mesh_from_geometry,
    section_loft,
)

BUTTON_GUIDE_HALF_OPENING = 0.0225
BUTTON_GUIDE_WALL_THICKNESS = 0.006
BUTTON_GUIDE_HEIGHT = 0.080
BUTTON_GUIDE_OUTER_HALF = BUTTON_GUIDE_HALF_OPENING + BUTTON_GUIDE_WALL_THICKNESS
BUTTON_STEM_RADIUS = 0.019
BUTTON_GUIDE_CENTER_Z = 0.950


def _prism_from_yz_profile(
    width: float,
    yz_profile: list[tuple[float, float]],
    mesh_name: str,
):
    sections = []
    for x in (-0.5 * width, 0.5 * width):
        sections.append([(x, y, z) for y, z in yz_profile])
    return mesh_from_geometry(section_loft(sections), mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casino_cabinet")

    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.06, 0.07, 0.08, 1.0))
    trim = model.material("trim", rgba=(0.32, 0.34, 0.38, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.30, 0.34, 0.35))
    display_glow = model.material("display_glow", rgba=(0.10, 0.22, 0.34, 0.55))
    chrome = model.material("chrome", rgba=(0.72, 0.74, 0.78, 1.0))
    reel_white = model.material("reel_white", rgba=(0.92, 0.92, 0.90, 1.0))
    reel_band = model.material("reel_band", rgba=(0.82, 0.10, 0.14, 1.0))
    button_red = model.material("button_red", rgba=(0.86, 0.16, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.84, 0.68, 0.10)),
        origin=Origin(xyz=(0.0, -0.01, 0.05)),
        material=charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.04, 0.62, 1.68)),
        origin=Origin(xyz=(-0.40, -0.01, 0.94)),
        material=charcoal,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((0.04, 0.62, 1.68)),
        origin=Origin(xyz=(0.40, -0.01, 0.94)),
        material=charcoal,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((0.76, 0.46, 0.03)),
        origin=Origin(xyz=(0.0, -0.05, 0.115)),
        material=satin_black,
        name="interior_floor",
    )
    cabinet.visual(
        Box((0.76, 0.04, 0.82)),
        origin=Origin(xyz=(0.0, -0.285, 1.45)),
        material=charcoal,
        name="rear_upper_wall",
    )
    cabinet.visual(
        Box((0.76, 0.03, 0.15)),
        origin=Origin(xyz=(0.0, -0.285, 0.21)),
        material=charcoal,
        name="rear_opening_sill",
    )
    cabinet.visual(
        Box((0.76, 0.03, 0.18)),
        origin=Origin(xyz=(0.0, -0.285, 1.00)),
        material=charcoal,
        name="rear_opening_lintel",
    )
    cabinet.visual(
        Box((0.08, 0.03, 0.62)),
        origin=Origin(xyz=(-0.34, -0.285, 0.60)),
        material=charcoal,
        name="back_left_jamb",
    )
    cabinet.visual(
        Box((0.08, 0.03, 0.62)),
        origin=Origin(xyz=(0.34, -0.285, 0.60)),
        material=charcoal,
        name="back_right_jamb",
    )
    cabinet.visual(
        Box((0.76, 0.04, 0.42)),
        origin=Origin(xyz=(0.0, 0.015, 1.12)),
        material=satin_black,
        name="reel_backboard",
    )
    cabinet.visual(
        Box((0.76, 0.03, 0.52)),
        origin=Origin(xyz=(0.0, -0.18, 1.56)),
        material=satin_black,
        name="display_backboard",
    )
    cabinet.visual(
        Box((0.76, 0.05, 0.22)),
        origin=Origin(xyz=(0.0, 0.18, 0.67)),
        material=charcoal,
        name="front_lower_fascia",
    )
    cabinet.visual(
        Box((0.05, 0.08, 0.46)),
        origin=Origin(xyz=(-0.355, 0.20, 1.12)),
        material=trim,
        name="reel_bezel_left",
    )
    cabinet.visual(
        Box((0.05, 0.08, 0.46)),
        origin=Origin(xyz=(0.355, 0.20, 1.12)),
        material=trim,
        name="reel_bezel_right",
    )
    cabinet.visual(
        Box((0.66, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.20, 1.32)),
        material=trim,
        name="reel_bezel_top",
    )
    cabinet.visual(
        Box((0.66, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.20, 0.92)),
        material=trim,
        name="reel_bezel_bottom",
    )
    cabinet.visual(
        Box((0.06, 0.08, 0.54)),
        origin=Origin(xyz=(-0.37, 0.10, 1.56)),
        material=trim,
        name="screen_bezel_left",
    )
    cabinet.visual(
        Box((0.06, 0.08, 0.54)),
        origin=Origin(xyz=(0.37, 0.10, 1.56)),
        material=trim,
        name="screen_bezel_right",
    )
    cabinet.visual(
        Box((0.68, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 1.82)),
        material=trim,
        name="screen_bezel_top",
    )
    cabinet.visual(
        Box((0.68, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 1.34)),
        material=trim,
        name="screen_bezel_bottom",
    )
    cabinet.visual(
        _prism_from_yz_profile(
            0.50,
            [
                (-0.12, -0.06),
                (0.09, -0.06),
                (0.16, 0.01),
                (0.10, 0.09),
                (-0.10, 0.11),
                (-0.14, 0.03),
            ],
            "control_deck_left_shell",
        ),
        origin=Origin(xyz=(-0.14, 0.18, 0.90)),
        material=charcoal,
        name="control_deck_left",
    )
    cabinet.visual(
        _prism_from_yz_profile(
            0.138,
            [
                (-0.12, -0.06),
                (0.09, -0.06),
                (0.16, 0.01),
                (0.10, 0.09),
                (-0.10, 0.11),
                (-0.14, 0.03),
            ],
            "control_deck_right_shell",
        ),
        origin=Origin(xyz=(0.331, 0.18, 0.90)),
        material=charcoal,
        name="control_deck_right",
    )
    cabinet.visual(
        Box((BUTTON_GUIDE_WALL_THICKNESS, 2.0 * BUTTON_GUIDE_OUTER_HALF, BUTTON_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.18 - BUTTON_GUIDE_HALF_OPENING - 0.5 * BUTTON_GUIDE_WALL_THICKNESS,
                0.27,
                BUTTON_GUIDE_CENTER_Z,
            )
        ),
        material=chrome,
        name="button_sleeve_left",
    )
    cabinet.visual(
        Box((BUTTON_GUIDE_WALL_THICKNESS, 2.0 * BUTTON_GUIDE_OUTER_HALF, BUTTON_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.18 + BUTTON_GUIDE_HALF_OPENING + 0.5 * BUTTON_GUIDE_WALL_THICKNESS,
                0.27,
                BUTTON_GUIDE_CENTER_Z,
            )
        ),
        material=chrome,
        name="button_sleeve_right",
    )
    cabinet.visual(
        Box((2.0 * BUTTON_GUIDE_OUTER_HALF, BUTTON_GUIDE_WALL_THICKNESS, BUTTON_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.18,
                0.27 - BUTTON_GUIDE_HALF_OPENING - 0.5 * BUTTON_GUIDE_WALL_THICKNESS,
                BUTTON_GUIDE_CENTER_Z,
            )
        ),
        material=chrome,
        name="button_sleeve_rear",
    )
    cabinet.visual(
        Box((2.0 * BUTTON_GUIDE_OUTER_HALF, BUTTON_GUIDE_WALL_THICKNESS, BUTTON_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.18,
                0.27 + BUTTON_GUIDE_HALF_OPENING + 0.5 * BUTTON_GUIDE_WALL_THICKNESS,
                BUTTON_GUIDE_CENTER_Z,
            )
        ),
        material=chrome,
        name="button_sleeve_front",
    )
    cabinet.visual(
        Box((0.064, 0.072, 0.050)),
        origin=Origin(xyz=(0.240, 0.270, 0.950)),
        material=satin_black,
        name="button_support_bridge",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.84, 0.68, 1.86)),
        mass=120.0,
        origin=Origin(xyz=(0.0, -0.01, 0.93)),
    )

    reel_window = model.part("reel_window")
    reel_window.visual(
        Box((0.70, 0.01, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=smoked_glass,
        name="window_glass",
    )
    reel_window.inertial = Inertial.from_geometry(
        Box((0.70, 0.01, 0.20)),
        mass=2.0,
    )
    model.articulation(
        "cabinet_to_reel_window",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_window,
        origin=Origin(xyz=(0.0, 0.245, 1.12)),
    )

    upper_display = model.part("upper_display")
    upper_display.visual(
        Box((0.66, 0.01, 0.40)),
        material=display_glow,
        name="display_panel",
    )
    upper_display.inertial = Inertial.from_geometry(
        Box((0.66, 0.01, 0.40)),
        mass=2.2,
    )
    model.articulation(
        "cabinet_to_upper_display",
        ArticulationType.FIXED,
        parent=cabinet,
        child=upper_display,
        origin=Origin(xyz=(0.0, 0.145, 1.56)),
    )

    reel_x_positions = (-0.28, -0.14, 0.0, 0.14, 0.28)
    for index, x_pos in enumerate(reel_x_positions, start=1):
        cabinet.visual(
            Box((0.010, 0.106, 0.045)),
            origin=Origin(xyz=(x_pos - 0.068, 0.078, 1.12)),
            material=trim,
            name=f"reel_{index}_left_bearing",
        )
        cabinet.visual(
            Box((0.010, 0.106, 0.045)),
            origin=Origin(xyz=(x_pos + 0.068, 0.078, 1.12)),
            material=trim,
            name=f"reel_{index}_right_bearing",
        )
        reel = model.part(f"reel_{index}")
        reel.visual(
            Cylinder(radius=0.092, length=0.104),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_white,
            name="reel_drum",
        )
        reel.visual(
            Cylinder(radius=0.014, length=0.126),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        reel.visual(
            Cylinder(radius=0.095, length=0.010),
            origin=Origin(xyz=(-0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_band,
            name="left_flange",
        )
        reel.visual(
            Cylinder(radius=0.095, length=0.010),
            origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_band,
            name="right_flange",
        )
        reel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.095, length=0.126),
            mass=1.2,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"cabinet_to_reel_{index}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x_pos, 0.140, 1.12)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    spin_button = model.part("spin_button")
    spin_button.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=button_red,
        name="button_cap",
    )
    spin_button.visual(
        Cylinder(radius=BUTTON_GUIDE_HALF_OPENING, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="button_collar",
    )
    spin_button.visual(
        Cylinder(radius=BUTTON_STEM_RADIUS, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=chrome,
        name="button_stem",
    )
    spin_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.115),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )
    model.articulation(
        "cabinet_to_spin_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=spin_button,
        origin=Origin(xyz=(0.18, 0.27, 0.98)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.15,
            lower=0.0,
            upper=0.012,
        ),
    )

    maintenance_panel = model.part("maintenance_panel")
    maintenance_panel.visual(
        Box((0.598, 0.018, 0.618)),
        origin=Origin(xyz=(-0.299, 0.0, 0.0)),
        material=satin_black,
        name="panel_leaf",
    )
    maintenance_panel.visual(
        Box((0.040, 0.028, 0.110)),
        origin=Origin(xyz=(-0.530, -0.023, 0.0)),
        material=trim,
        name="panel_handle",
    )
    maintenance_panel.inertial = Inertial.from_geometry(
        Box((0.62, 0.03, 0.62)),
        mass=8.0,
        origin=Origin(xyz=(-0.31, 0.0, 0.0)),
    )
    model.articulation(
        "cabinet_to_maintenance_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=maintenance_panel,
        origin=Origin(xyz=(0.30, -0.309, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
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

    cabinet = object_model.get_part("cabinet")
    reel_window = object_model.get_part("reel_window")
    spin_button = object_model.get_part("spin_button")
    maintenance_panel = object_model.get_part("maintenance_panel")
    spin_joint = object_model.get_articulation("cabinet_to_spin_button")
    panel_joint = object_model.get_articulation("cabinet_to_maintenance_panel")

    reel_parts = [object_model.get_part(f"reel_{index}") for index in range(1, 6)]
    reel_joints = [
        object_model.get_articulation(f"cabinet_to_reel_{index}") for index in range(1, 6)
    ]

    def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(axis, expected))

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))

    for index, joint in enumerate(reel_joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"reel_{index} uses a continuous shaft articulation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )
        ctx.check(
            f"reel_{index} shaft runs horizontally",
            _axis_matches(joint.axis, (1.0, 0.0, 0.0)),
            details=f"axis={joint.axis}",
        )

    reel_positions = [ctx.part_world_position(reel) for reel in reel_parts]
    reels_aligned = (
        all(position is not None for position in reel_positions)
        and all(abs(reel_positions[0][1] - position[1]) < 1e-6 for position in reel_positions[1:])
        and all(abs(reel_positions[0][2] - position[2]) < 1e-6 for position in reel_positions[1:])
        and all(
            reel_positions[index][0] < reel_positions[index + 1][0]
            for index in range(len(reel_positions) - 1)
        )
    )
    ctx.check(
        "five reel centers line up across one horizontal shaft row",
        reels_aligned,
        details=f"positions={reel_positions}",
    )

    ctx.expect_gap(
        reel_window,
        object_model.get_part("reel_3"),
        axis="y",
        min_gap=0.0075,
        max_gap=0.025,
        positive_elem="window_glass",
        negative_elem="reel_drum",
        name="center reel sits just behind the front glass",
    )

    ctx.check(
        "spin button travels downward in the deck",
        _axis_matches(spin_joint.axis, (0.0, 0.0, -1.0)),
        details=f"axis={spin_joint.axis}",
    )
    ctx.expect_gap(
        spin_button,
        cabinet,
        axis="x",
        min_gap=0.0025,
        max_gap=0.0045,
        positive_elem="button_stem",
        negative_elem="button_sleeve_left",
        name="spin button stem clears the left side of its guide sleeve at rest",
    )
    ctx.expect_gap(
        cabinet,
        spin_button,
        axis="x",
        min_gap=0.0025,
        max_gap=0.0045,
        positive_elem="button_sleeve_right",
        negative_elem="button_stem",
        name="spin button stem clears the right side of its guide sleeve at rest",
    )
    ctx.expect_gap(
        spin_button,
        cabinet,
        axis="y",
        min_gap=0.0025,
        max_gap=0.0045,
        positive_elem="button_stem",
        negative_elem="button_sleeve_rear",
        name="spin button stem clears the rear of its guide sleeve at rest",
    )
    ctx.expect_gap(
        cabinet,
        spin_button,
        axis="y",
        min_gap=0.0025,
        max_gap=0.0045,
        positive_elem="button_sleeve_front",
        negative_elem="button_stem",
        name="spin button stem clears the front of its guide sleeve at rest",
    )
    ctx.expect_overlap(
        spin_button,
        cabinet,
        axes="z",
        elem_a="button_stem",
        elem_b="button_sleeve_right",
        min_overlap=0.045,
        name="spin button stem remains retained in the sleeve at rest",
    )
    ctx.expect_contact(
        cabinet,
        spin_button,
        elem_a="button_sleeve_right",
        elem_b="button_collar",
        name="spin button collar is guided by the sleeve at rest",
    )

    rest_cap = _center_from_aabb(ctx.part_element_world_aabb(spin_button, elem="button_cap"))
    with ctx.pose({spin_joint: spin_joint.motion_limits.upper}):
        ctx.expect_gap(
            spin_button,
            cabinet,
            axis="x",
            min_gap=0.0025,
            max_gap=0.0045,
            positive_elem="button_stem",
            negative_elem="button_sleeve_left",
            name="spin button stem clears the left side of its guide sleeve when pressed",
        )
        ctx.expect_gap(
            cabinet,
            spin_button,
            axis="x",
            min_gap=0.0025,
            max_gap=0.0045,
            positive_elem="button_sleeve_right",
            negative_elem="button_stem",
            name="spin button stem clears the right side of its guide sleeve when pressed",
        )
        ctx.expect_gap(
            spin_button,
            cabinet,
            axis="y",
            min_gap=0.0025,
            max_gap=0.0045,
            positive_elem="button_stem",
            negative_elem="button_sleeve_rear",
            name="spin button stem clears the rear of its guide sleeve when pressed",
        )
        ctx.expect_gap(
            cabinet,
            spin_button,
            axis="y",
            min_gap=0.0025,
            max_gap=0.0045,
            positive_elem="button_sleeve_front",
            negative_elem="button_stem",
            name="spin button stem clears the front of its guide sleeve when pressed",
        )
        ctx.expect_overlap(
            spin_button,
            cabinet,
            axes="z",
            elem_a="button_stem",
            elem_b="button_sleeve_right",
            min_overlap=0.040,
            name="spin button stem remains retained in the sleeve when pressed",
        )
        ctx.expect_contact(
            cabinet,
            spin_button,
            elem_a="button_sleeve_right",
            elem_b="button_collar",
            name="spin button collar stays guided by the sleeve when pressed",
        )
        pressed_cap = _center_from_aabb(
            ctx.part_element_world_aabb(spin_button, elem="button_cap")
        )

    ctx.check(
        "spin button cap moves downward when pressed",
        rest_cap is not None
        and pressed_cap is not None
        and pressed_cap[2] < rest_cap[2] - 0.009,
        details=f"rest_cap={rest_cap}, pressed_cap={pressed_cap}",
    )

    ctx.check(
        "maintenance panel hinge is vertical",
        _axis_matches(panel_joint.axis, (0.0, 0.0, 1.0)),
        details=f"axis={panel_joint.axis}",
    )
    closed_panel_center = _center_from_aabb(
        ctx.part_element_world_aabb(maintenance_panel, elem="panel_leaf")
    )
    with ctx.pose({panel_joint: 1.10}):
        open_panel_center = _center_from_aabb(
            ctx.part_element_world_aabb(maintenance_panel, elem="panel_leaf")
        )

    ctx.check(
        "rear maintenance panel swings outward from the cabinet back",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[1] < closed_panel_center[1] - 0.10,
        details=f"closed={closed_panel_center}, open={open_panel_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
