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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.160
SHELL_THICKNESS = 0.018
TOP_THICKNESS = 0.018
FRONT_FACE_Y = CANOPY_DEPTH * 0.5

CHIMNEY_WIDTH = 0.320
CHIMNEY_DEPTH = 0.260
CHIMNEY_HEIGHT = 0.720
CHIMNEY_WALL = 0.012

FILTER_PANEL_WIDTH = 0.330
FILTER_PANEL_DEPTH = 0.220
FILTER_PANEL_THICKNESS = 0.008

CONTROL_PANEL_WIDTH = 0.320
CONTROL_PANEL_HEIGHT = 0.040
CONTROL_PANEL_THICKNESS = 0.004
CONTROL_PANEL_Z = 0.110

BUTTON_COUNT = 5
BUTTON_PITCH = 0.048
BUTTON_CAP_WIDTH = 0.034
BUTTON_CAP_HEIGHT = 0.018
BUTTON_CAP_DEPTH = 0.008
BUTTON_STEM_WIDTH = 0.020
BUTTON_STEM_HEIGHT = 0.010
BUTTON_STEM_DEPTH = 0.012
BUTTON_RETAINER_WIDTH = 0.044


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom
BUTTON_RETAINER_HEIGHT = 0.024
BUTTON_RETAINER_DEPTH = 0.004
BUTTON_TRAVEL = 0.004


def _shift_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _control_panel_mesh():
    outer = rounded_rect_profile(
        CONTROL_PANEL_WIDTH,
        CONTROL_PANEL_HEIGHT,
        radius=0.006,
        corner_segments=8,
    )
    hole_profile = rounded_rect_profile(0.040, 0.021, radius=0.0025, corner_segments=6)
    x0 = -BUTTON_PITCH * (BUTTON_COUNT - 1) * 0.5
    holes = [
        _shift_profile(hole_profile, x0 + index * BUTTON_PITCH, 0.0)
        for index in range(BUTTON_COUNT)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile=outer,
            hole_profiles=holes,
            height=CONTROL_PANEL_THICKNESS,
            center=True,
        ),
        ASSETS.mesh_path("range_hood_control_panel.obj"),
    )


def _filter_panel_mesh():
    return mesh_from_geometry(
        _grille_panel_geometry(
            panel_size=(FILTER_PANEL_WIDTH, FILTER_PANEL_DEPTH),
            thickness=FILTER_PANEL_THICKNESS,
            frame=0.010,
            slat_pitch=0.026,
            slat_width=0.012,
            slat_angle_deg=34.0,
            corner_radius=0.006,
            center=True,
        ),
        ASSETS.mesh_path("range_hood_filter_panel.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.82, 1.0))
    brushed_shadow = model.material("brushed_shadow", rgba=(0.68, 0.70, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    button_black = model.material("button_black", rgba=(0.12, 0.12, 0.13, 1.0))

    control_panel_mesh = _control_panel_mesh()
    filter_mesh = _filter_panel_mesh()

    canopy = model.part("canopy_body")
    canopy.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - TOP_THICKNESS * 0.5)),
        material=stainless,
        name="top_shell",
    )
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, 0.150)),
        origin=Origin(xyz=(-CANOPY_WIDTH * 0.5 + SHELL_THICKNESS * 0.5, 0.0, 0.075)),
        material=stainless,
        name="left_shell",
    )
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, 0.150)),
        origin=Origin(xyz=(CANOPY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5, 0.0, 0.075)),
        material=stainless,
        name="right_shell",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, 0.150)),
        origin=Origin(xyz=(0.0, -CANOPY_DEPTH * 0.5 + SHELL_THICKNESS * 0.5, 0.075)),
        material=brushed_shadow,
        name="rear_shell",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, 0.022)),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - SHELL_THICKNESS * 0.5, 0.139)),
        material=stainless,
        name="front_top_brow",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, 0.016)),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - SHELL_THICKNESS * 0.5, 0.084)),
        material=stainless,
        name="front_lower_rail",
    )
    canopy.visual(
        Box((0.304, SHELL_THICKNESS, 0.036)),
        origin=Origin(xyz=(-0.298, FRONT_FACE_Y - SHELL_THICKNESS * 0.5, 0.110)),
        material=stainless,
        name="front_left_cheek",
    )
    canopy.visual(
        Box((0.304, SHELL_THICKNESS, 0.036)),
        origin=Origin(xyz=(0.298, FRONT_FACE_Y - SHELL_THICKNESS * 0.5, 0.110)),
        material=stainless,
        name="front_right_cheek",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, 0.045, 0.028)),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - 0.045 * 0.5, 0.014)),
        material=brushed_shadow,
        name="front_apron",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
    )

    chimney = model.part("chimney_cover")
    chimney.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_WALL, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, CHIMNEY_DEPTH * 0.5 - CHIMNEY_WALL * 0.5, CHIMNEY_HEIGHT * 0.5)),
        material=stainless,
        name="front_shell",
    )
    chimney.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_WALL, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, -CHIMNEY_DEPTH * 0.5 + CHIMNEY_WALL * 0.5, CHIMNEY_HEIGHT * 0.5)),
        material=brushed_shadow,
        name="rear_shell",
    )
    chimney.visual(
        Box((CHIMNEY_WALL, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(-CHIMNEY_WIDTH * 0.5 + CHIMNEY_WALL * 0.5, 0.0, CHIMNEY_HEIGHT * 0.5)),
        material=stainless,
        name="left_shell",
    )
    chimney.visual(
        Box((CHIMNEY_WALL, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(CHIMNEY_WIDTH * 0.5 - CHIMNEY_WALL * 0.5, 0.0, CHIMNEY_HEIGHT * 0.5)),
        material=stainless,
        name="right_shell",
    )
    chimney.inertial = Inertial.from_geometry(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT * 0.5)),
    )
    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT)),
    )

    filter_bank = model.part("filter_bank")
    filter_bank.visual(
        filter_mesh,
        origin=Origin(xyz=(-0.172, 0.0, -0.004)),
        material=filter_metal,
        name="left_filter",
    )
    filter_bank.visual(
        filter_mesh,
        origin=Origin(xyz=(0.172, 0.0, -0.004)),
        material=filter_metal,
        name="right_filter",
    )
    filter_bank.visual(
        Box((0.780, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, FILTER_PANEL_DEPTH * 0.5, 0.005)),
        material=charcoal,
        name="front_bridge",
    )
    filter_bank.visual(
        Box((0.780, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -FILTER_PANEL_DEPTH * 0.5, 0.005)),
        material=charcoal,
        name="rear_bridge",
    )
    filter_bank.visual(
        Box((0.024, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
        name="center_brace",
    )
    filter_bank.inertial = Inertial.from_geometry(
        Box((0.780, 0.240, 0.022)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "canopy_to_filter_bank",
        ArticulationType.FIXED,
        parent=canopy,
        child=filter_bank,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        control_panel_mesh,
        origin=Origin(
            xyz=(0.0, CONTROL_PANEL_THICKNESS * 0.5, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="panel_frame",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        mass=0.35,
        origin=Origin(xyz=(0.0, CONTROL_PANEL_THICKNESS * 0.5, 0.0)),
    )
    model.articulation(
        "canopy_to_control_panel",
        ArticulationType.FIXED,
        parent=canopy,
        child=control_panel,
        origin=Origin(xyz=(0.0, FRONT_FACE_Y, CONTROL_PANEL_Z)),
    )

    button_x0 = -BUTTON_PITCH * (BUTTON_COUNT - 1) * 0.5
    for index in range(BUTTON_COUNT):
        button = model.part(f"button_{index + 1}")
        button.visual(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
            material=button_black,
            name="cap",
        )
        button.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=button_black,
            name="stem",
        )
        button.visual(
            Box((BUTTON_RETAINER_WIDTH, BUTTON_RETAINER_DEPTH, BUTTON_RETAINER_HEIGHT)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=button_black,
            name="retainer",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_RETAINER_WIDTH, 0.020, BUTTON_RETAINER_HEIGHT)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
        )
        model.articulation(
            f"control_panel_to_button_{index + 1}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(button_x0 + index * BUTTON_PITCH, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy_body")
    chimney = object_model.get_part("chimney_cover")
    filter_bank = object_model.get_part("filter_bank")
    control_panel = object_model.get_part("control_panel")
    button_parts = [object_model.get_part(f"button_{index + 1}") for index in range(BUTTON_COUNT)]
    button_joints = [
        object_model.get_articulation(f"control_panel_to_button_{index + 1}")
        for index in range(BUTTON_COUNT)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(chimney, canopy, name="chimney cover sits on canopy top")
    ctx.expect_contact(filter_bank, canopy, name="filter bank hangs from canopy shell")
    ctx.expect_contact(control_panel, canopy, name="control panel seats in front opening")
    ctx.expect_within(filter_bank, canopy, axes="xy", margin=0.02, name="filters stay within canopy footprint")
    ctx.expect_within(control_panel, canopy, axes="x", margin=0.01, name="control panel centered in canopy width")
    ctx.expect_overlap(control_panel, canopy, axes="x", min_overlap=0.25, name="control panel spans front center")

    canopy_aabb = ctx.part_world_aabb(canopy)
    chimney_aabb = ctx.part_world_aabb(chimney)
    ctx.check(
        "major parts have measurable bounds",
        canopy_aabb is not None and chimney_aabb is not None,
        details="Expected canopy and chimney to produce world-space AABBs.",
    )
    if canopy_aabb is not None and chimney_aabb is not None:
        canopy_width = canopy_aabb[1][0] - canopy_aabb[0][0]
        canopy_height = canopy_aabb[1][2] - canopy_aabb[0][2]
        chimney_width = chimney_aabb[1][0] - chimney_aabb[0][0]
        chimney_height = chimney_aabb[1][2] - chimney_aabb[0][2]
        ctx.check(
            "chimney taller than canopy",
            chimney_height > canopy_height * 3.5,
            details=f"chimney_height={chimney_height:.3f}, canopy_height={canopy_height:.3f}",
        )
        ctx.check(
            "canopy wider than chimney",
            canopy_width > chimney_width + 0.45,
            details=f"canopy_width={canopy_width:.3f}, chimney_width={chimney_width:.3f}",
        )

    rest_positions = [ctx.part_world_position(button) for button in button_parts]
    ctx.check(
        "button row positions measurable",
        all(position is not None for position in rest_positions),
        details="Expected all buttons to have measurable world positions at rest.",
    )
    if all(position is not None for position in rest_positions):
        button_positions = [position for position in rest_positions if position is not None]
        pitches = [
            button_positions[index + 1][0] - button_positions[index][0]
            for index in range(len(button_positions) - 1)
        ]
        evenly_spaced = all(0.044 <= pitch <= 0.052 for pitch in pitches)
        centered_row = abs(button_positions[0][0] + button_positions[-1][0]) <= 0.005
        ctx.check(
            "buttons form an even front row",
            evenly_spaced and centered_row,
            details=f"pitches={[round(p, 4) for p in pitches]}, end_sum={button_positions[0][0] + button_positions[-1][0]:.4f}",
        )

    for index, button in enumerate(button_parts):
        ctx.expect_contact(
            button,
            control_panel,
            name=f"button {index + 1} seats against control panel at rest",
        )
        ctx.expect_within(
            button,
            control_panel,
            axes="xz",
            margin=0.01,
            name=f"button {index + 1} stays within panel opening band",
        )
        ctx.expect_overlap(
            button,
            control_panel,
            axes="xz",
            min_overlap=0.014,
            name=f"button {index + 1} overlaps control panel area",
        )

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        rest_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index + 1} rest position measurable",
            rest_pos is not None,
            details="Expected button position at rest.",
        )
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"button {index + 1} pressed position measurable",
                pressed_pos is not None,
                details="Expected button position when fully pressed.",
            )
            if rest_pos is not None and pressed_pos is not None:
                ctx.check(
                    f"button {index + 1} plunges inward",
                    pressed_pos[1] < rest_pos[1] - 0.0035,
                    details=f"rest_y={rest_pos[1]:.4f}, pressed_y={pressed_pos[1]:.4f}",
                )
            ctx.expect_within(
                button,
                control_panel,
                axes="xz",
                margin=0.01,
                name=f"button {index + 1} remains aligned when pressed",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
