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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

CABINET_WIDTH = 1.28
CABINET_DEPTH = 0.62
CABINET_HEIGHT = 0.88
SIDE_THICKNESS = 0.025
BACK_THICKNESS = 0.020

TOP_WIDTH = 1.42
TOP_DEPTH = 0.74
TOP_THICKNESS = 0.040

CONTROL_STRIP_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
CONTROL_STRIP_HEIGHT = 0.11
CONTROL_STRIP_DEPTH = 0.040
CONTROL_STRIP_CENTER_Y = CABINET_DEPTH * 0.5 - CONTROL_STRIP_DEPTH * 0.5
CONTROL_STRIP_CENTER_Z = CABINET_HEIGHT - CONTROL_STRIP_HEIGHT * 0.5 - 0.010

DOOR_WIDTH = 1.18
DOOR_HEIGHT = 0.64
DOOR_THICKNESS = 0.022
DOOR_HINGE_Z = 0.10
DOOR_OPEN_ANGLE = math.radians(108.0)
HINGE_BARREL_RADIUS = 0.010
HINGE_BARREL_LENGTH = 0.050

BUTTON_X_OFFSET = 0.070
BUTTON_CENTER_Z_LOCAL = 0.017
BUTTON_STEM_WIDTH = 0.060
BUTTON_STEM_HEIGHT = 0.022
BUTTON_STEM_DEPTH = 0.055
BUTTON_CAP_WIDTH = 0.106
BUTTON_CAP_HEIGHT = 0.036
BUTTON_CAP_DEPTH = 0.012
BUTTON_CAP_REST_GAP = 0.0
BUTTON_TRAVEL = 0.004

KNOB_RADIUS = 0.016
KNOB_LENGTH = 0.018
KNOB_CENTER_Z_LOCAL = -0.030

COOKTOP_GLASS_WIDTH = 0.90
COOKTOP_GLASS_DEPTH = 0.52
COOKTOP_GLASS_THICKNESS = 0.006
COOKTOP_BODY_WIDTH = 0.84
COOKTOP_BODY_DEPTH = 0.46
COOKTOP_BODY_HEIGHT = 0.12
COOKTOP_CUTOUT_WIDTH = 0.85
COOKTOP_CUTOUT_DEPTH = 0.47


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _offset_profile(
    profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_island_top_mesh():
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(TOP_WIDTH, TOP_DEPTH, radius=0.028, corner_segments=10),
        [
            rounded_rect_profile(
                COOKTOP_CUTOUT_WIDTH,
                COOKTOP_CUTOUT_DEPTH,
                radius=0.012,
                corner_segments=8,
            )
        ],
        TOP_THICKNESS,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("island_top_with_cooktop_cutout.obj"))


def _control_strip_vertical_clear_height() -> float:
    return CONTROL_STRIP_HEIGHT - BUTTON_CENTER_Z_LOCAL - BUTTON_STEM_HEIGHT * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_zone_cooktop_island", assets=ASSETS)

    stone = model.material("stone", rgba=(0.74, 0.73, 0.71, 1.0))
    cabinet_paint = model.material("cabinet_paint", rgba=(0.26, 0.28, 0.30, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.09, 0.09, 0.10, 0.95))
    cooktop_zone = model.material("cooktop_zone", rgba=(0.20, 0.22, 0.24, 0.42))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.14, 0.15, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.63, 0.49, 0.34, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))

    cabinet_shell = model.part("cabinet_shell")
    cabinet_shell.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH * 0.5 + SIDE_THICKNESS * 0.5, 0.0, CABINET_HEIGHT * 0.5)),
        material=cabinet_paint,
        name="left_side",
    )
    cabinet_shell.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH * 0.5 - SIDE_THICKNESS * 0.5, 0.0, CABINET_HEIGHT * 0.5)),
        material=cabinet_paint,
        name="right_side",
    )
    cabinet_shell.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet_shell.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.60, 0.03)),
        origin=Origin(xyz=(0.0, 0.01, 0.015)),
        material=cabinet_paint,
        name="bottom_panel",
    )
    cabinet_shell.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.26, CABINET_HEIGHT - 0.025)),
        material=cabinet_paint,
        name="top_back_rail",
    )
    cabinet_shell.visual(
        Box((1.16, 0.09, 0.10)),
        origin=Origin(xyz=(0.0, 0.20, 0.05)),
        material=dark_trim,
        name="toe_kick",
    )
    cabinet_shell.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(-CABINET_WIDTH * 0.5 + HINGE_BARREL_LENGTH * 0.5, CABINET_DEPTH * 0.5, DOOR_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="left_hinge_barrel",
    )
    cabinet_shell.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(CABINET_WIDTH * 0.5 - HINGE_BARREL_LENGTH * 0.5, CABINET_DEPTH * 0.5, DOOR_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="right_hinge_barrel",
    )
    cabinet_shell.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    island_top = model.part("island_top")
    island_top.visual(_build_island_top_mesh(), material=stone, name="stone_top")
    island_top.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        mass=38.0,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_island_top",
        ArticulationType.FIXED,
        parent=cabinet_shell,
        child=island_top,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT + TOP_THICKNESS * 0.5)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((COOKTOP_GLASS_WIDTH, COOKTOP_GLASS_DEPTH, COOKTOP_GLASS_THICKNESS)),
        material=cooktop_glass,
        name="glass_surface",
    )
    cooktop.visual(
        Box((COOKTOP_BODY_WIDTH, COOKTOP_BODY_DEPTH, COOKTOP_BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, -(COOKTOP_GLASS_THICKNESS + COOKTOP_BODY_HEIGHT) * 0.5)
        ),
        material=control_black,
        name="cooktop_body",
    )
    for zone_name, zone_x, zone_y, zone_radius in (
        ("front_left_zone", -0.23, 0.11, 0.110),
        ("rear_left_zone", -0.25, -0.12, 0.085),
        ("center_zone", 0.00, -0.03, 0.080),
        ("front_right_zone", 0.23, 0.11, 0.110),
        ("rear_right_zone", 0.25, -0.12, 0.085),
    ):
        cooktop.visual(
            Cylinder(radius=zone_radius, length=0.0008),
            origin=Origin(
                xyz=(
                    zone_x,
                    zone_y,
                    COOKTOP_GLASS_THICKNESS * 0.5 + 0.0004,
                )
            ),
            material=cooktop_zone,
            name=zone_name,
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_GLASS_WIDTH, COOKTOP_GLASS_DEPTH, COOKTOP_BODY_HEIGHT + COOKTOP_GLASS_THICKNESS)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )
    model.articulation(
        "top_to_cooktop",
        ArticulationType.FIXED,
        parent=island_top,
        child=cooktop,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                TOP_THICKNESS * 0.5 + COOKTOP_GLASS_THICKNESS * 0.5,
            )
        ),
    )

    control_strip = model.part("control_strip")
    opening_bottom_z = BUTTON_CENTER_Z_LOCAL - BUTTON_STEM_HEIGHT * 0.5
    opening_top_z = BUTTON_CENTER_Z_LOCAL + BUTTON_STEM_HEIGHT * 0.5
    top_bound = CONTROL_STRIP_HEIGHT * 0.5
    bottom_bound = -CONTROL_STRIP_HEIGHT * 0.5
    bottom_rail_height = opening_bottom_z - bottom_bound
    top_rail_height = top_bound - opening_top_z
    side_segment_width = CONTROL_STRIP_WIDTH * 0.5 - BUTTON_X_OFFSET - BUTTON_STEM_WIDTH * 0.5
    center_segment_width = 2.0 * BUTTON_X_OFFSET - BUTTON_STEM_WIDTH
    control_strip.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, bottom_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                bottom_bound + bottom_rail_height * 0.5,
            )
        ),
        material=control_black,
        name="bottom_rail",
    )
    control_strip.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                opening_top_z + top_rail_height * 0.5,
            )
        ),
        material=control_black,
        name="top_rail",
    )
    control_strip.visual(
        Box((side_segment_width, CONTROL_STRIP_DEPTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(
            xyz=(
                -CONTROL_STRIP_WIDTH * 0.25 - BUTTON_X_OFFSET * 0.5 - BUTTON_STEM_WIDTH * 0.25,
                0.0,
                BUTTON_CENTER_Z_LOCAL,
            )
        ),
        material=control_black,
        name="left_frame",
    )
    control_strip.visual(
        Box((center_segment_width, CONTROL_STRIP_DEPTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_CENTER_Z_LOCAL)),
        material=control_black,
        name="center_frame",
    )
    control_strip.visual(
        Box((side_segment_width, CONTROL_STRIP_DEPTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_STRIP_WIDTH * 0.25 + BUTTON_X_OFFSET * 0.5 + BUTTON_STEM_WIDTH * 0.25,
                0.0,
                BUTTON_CENTER_Z_LOCAL,
            )
        ),
        material=control_black,
        name="right_frame",
    )
    control_strip.inertial = Inertial.from_geometry(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT)),
        mass=3.2,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_control_strip",
        ArticulationType.FIXED,
        parent=cabinet_shell,
        child=control_strip,
        origin=Origin(
            xyz=(0.0, CONTROL_STRIP_CENTER_Y, CONTROL_STRIP_CENTER_Z),
        ),
    )

    def add_button(part_name: str, joint_name: str, x_offset: float) -> None:
        button = model.part(part_name)
        button.visual(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            origin=Origin(
                xyz=(0.0, BUTTON_CAP_REST_GAP + BUTTON_CAP_DEPTH * 0.5, 0.0)
            ),
            material=stainless,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
            origin=Origin(
                xyz=(0.0, BUTTON_CAP_REST_GAP - BUTTON_STEM_DEPTH * 0.5, 0.0)
            ),
            material=dark_trim,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_CAP_WIDTH, BUTTON_STEM_DEPTH + BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            mass=0.14,
            origin=Origin(xyz=(0.0, -0.018, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(
                xyz=(
                    x_offset,
                    CONTROL_STRIP_DEPTH * 0.5,
                    BUTTON_CENTER_Z_LOCAL,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=0.10,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    add_button("left_button", "control_strip_to_left_button", -BUTTON_X_OFFSET)
    add_button("right_button", "control_strip_to_right_button", BUTTON_X_OFFSET)

    def add_knob(part_name: str, joint_name: str, x_offset: float) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            origin=Origin(
                xyz=(0.0, KNOB_LENGTH * 0.5, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, KNOB_LENGTH * 0.9, 0.010)),
            origin=Origin(
                xyz=(0.0, KNOB_LENGTH * 0.55, KNOB_RADIUS * 0.65),
            ),
            material=stainless,
            name="knob_pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((KNOB_RADIUS * 2.2, KNOB_LENGTH, KNOB_RADIUS * 2.2)),
            mass=0.06,
            origin=Origin(xyz=(0.0, KNOB_LENGTH * 0.5, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=control_strip,
            child=knob,
            origin=Origin(
                xyz=(
                    x_offset,
                    CONTROL_STRIP_DEPTH * 0.5,
                    KNOB_CENTER_Z_LOCAL,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=10.0,
            ),
        )

    add_knob("left_knob", "control_strip_to_left_knob", -BUTTON_X_OFFSET)
    add_knob("right_knob", "control_strip_to_right_knob", BUTTON_X_OFFSET)

    door = model.part("flip_down_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
        material=warm_wood,
        name="door_panel",
    )
    door.visual(
        Box((0.46, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.030, DOOR_HEIGHT - 0.040)),
        material=dark_trim,
        name="door_pull",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(-DOOR_WIDTH * 0.5 + HINGE_BARREL_LENGTH * 0.5, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="left_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(DOOR_WIDTH * 0.5 - HINGE_BARREL_LENGTH * 0.5, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="right_hinge_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + 0.01, DOOR_HEIGHT)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.016, DOOR_HEIGHT * 0.5)),
    )
    model.articulation(
        "shell_to_flip_down_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_shell,
        child=door,
        origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet_shell = object_model.get_part("cabinet_shell")
    island_top = object_model.get_part("island_top")
    cooktop = object_model.get_part("cooktop")
    control_strip = object_model.get_part("control_strip")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    door = object_model.get_part("flip_down_door")

    glass_surface = cooktop.get_visual("glass_surface")
    cooktop.get_visual("front_left_zone")
    cooktop.get_visual("rear_left_zone")
    cooktop.get_visual("center_zone")
    cooktop.get_visual("front_right_zone")
    cooktop.get_visual("rear_right_zone")
    left_button.get_visual("button_cap")
    right_button.get_visual("button_cap")
    left_knob.get_visual("knob_body")
    right_knob.get_visual("knob_body")
    door.get_visual("door_panel")

    left_button_slide = object_model.get_articulation("control_strip_to_left_button")
    right_button_slide = object_model.get_articulation("control_strip_to_right_button")
    left_knob_spin = object_model.get_articulation("control_strip_to_left_knob")
    right_knob_spin = object_model.get_articulation("control_strip_to_right_knob")
    door_hinge = object_model.get_articulation("shell_to_flip_down_door")

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

    ctx.check(
        "button_joints_are_prismatic",
        left_button_slide.articulation_type == ArticulationType.PRISMATIC
        and right_button_slide.articulation_type == ArticulationType.PRISMATIC,
        "Both push-buttons must use prismatic joints.",
    )
    ctx.check(
        "button_joints_press_inward",
        tuple(left_button_slide.axis) == (0.0, -1.0, 0.0)
        and tuple(right_button_slide.axis) == (0.0, -1.0, 0.0),
        "Both button axes should point inward along -Y.",
    )
    ctx.check(
        "knob_joints_are_continuous",
        left_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        "Both knobs must be continuous rotary joints.",
    )
    ctx.check(
        "knob_axes_front_to_back",
        tuple(left_knob_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_knob_spin.axis) == (0.0, 1.0, 0.0),
        "Knob axes should run front-to-back along Y.",
    )
    ctx.check(
        "door_joint_rotates_downward",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (-1.0, 0.0, 0.0),
        "The door should hinge downward about the bottom edge.",
    )
    ctx.check(
        "cooktop_has_five_zone_markings",
        len(cooktop.visuals) == 7,
        "Cooktop should contain glass, body, and five zone markings.",
    )

    ctx.expect_contact(island_top, cabinet_shell)
    ctx.expect_contact(cooktop, island_top)
    ctx.expect_contact(control_strip, cabinet_shell)
    ctx.expect_contact(left_button, control_strip)
    ctx.expect_contact(right_button, control_strip)
    ctx.expect_contact(left_knob, control_strip)
    ctx.expect_contact(right_knob, control_strip)
    ctx.expect_contact(door, cabinet_shell)

    ctx.expect_origin_distance(
        cooktop,
        island_top,
        axes="xy",
        min_dist=0.0,
        max_dist=0.02,
        name="cooktop_centered_in_top",
    )
    ctx.expect_within(cooktop, island_top, axes="xy", margin=0.02)
    ctx.expect_origin_distance(
        left_button,
        right_button,
        axes="x",
        min_dist=0.12,
        max_dist=0.16,
        name="button_pair_spacing",
    )
    ctx.expect_origin_distance(
        left_knob,
        right_knob,
        axes="x",
        min_dist=0.12,
        max_dist=0.16,
        name="knob_pair_spacing",
    )
    ctx.expect_origin_gap(
        left_button,
        left_knob,
        axis="z",
        min_gap=0.040,
        max_gap=0.060,
        name="left_knob_below_left_button",
    )
    ctx.expect_origin_gap(
        right_button,
        right_knob,
        axis="z",
        min_gap=0.040,
        max_gap=0.060,
        name="right_knob_below_right_button",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            control_strip,
            door,
            axis="z",
            min_gap=0.015,
            max_gap=0.030,
            name="closed_door_gap_below_controls",
        )
        ctx.expect_overlap(door, cabinet_shell, axes="x", min_overlap=1.10)

    left_button_rest = ctx.part_world_position(left_button)
    right_button_rest = ctx.part_world_position(right_button)
    assert left_button_rest is not None and right_button_rest is not None
    with ctx.pose({left_button_slide: BUTTON_TRAVEL, right_button_slide: BUTTON_TRAVEL}):
        left_button_pressed = ctx.part_world_position(left_button)
        right_button_pressed = ctx.part_world_position(right_button)
        assert left_button_pressed is not None and right_button_pressed is not None
        ctx.check(
            "buttons_press_inward_by_short_travel",
            left_button_pressed[1] < left_button_rest[1] - BUTTON_TRAVEL * 0.9
            and right_button_pressed[1] < right_button_rest[1] - BUTTON_TRAVEL * 0.9,
            "Pressed buttons should move inward a few millimeters.",
        )
        ctx.expect_contact(left_button, control_strip)
        ctx.expect_contact(right_button, control_strip)
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_pressed_no_floating")

    left_knob_rest = ctx.part_world_position(left_knob)
    right_knob_rest = ctx.part_world_position(right_knob)
    assert left_knob_rest is not None and right_knob_rest is not None
    with ctx.pose({left_knob_spin: math.pi / 2.0, right_knob_spin: -math.pi / 2.0}):
        left_knob_turned = ctx.part_world_position(left_knob)
        right_knob_turned = ctx.part_world_position(right_knob)
        assert left_knob_turned is not None and right_knob_turned is not None
        ctx.check(
            "knobs_rotate_in_place",
            max(
                abs(left_knob_turned[index] - left_knob_rest[index]) for index in range(3)
            )
            < 1e-9
            and max(
                abs(right_knob_turned[index] - right_knob_rest[index]) for index in range(3)
            )
            < 1e-9,
            "Knob origins should stay fixed while the knobs spin.",
        )
        ctx.expect_contact(left_knob, control_strip)
        ctx.expect_contact(right_knob, control_strip)
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_turned_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_turned_no_floating")

    door_closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_closed_aabb is not None
    door_closed_center = tuple(
        (door_closed_aabb[0][index] + door_closed_aabb[1][index]) * 0.5 for index in range(3)
    )
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        assert door_open_aabb is not None
        door_open_center = tuple(
            (door_open_aabb[0][index] + door_open_aabb[1][index]) * 0.5 for index in range(3)
        )
        ctx.check(
            "door_swings_out_and_down",
            door_open_center[2] < door_closed_center[2] - 0.30
            and door_open_center[1] > door_closed_center[1] + 0.20,
            "The flip-down door should drop and swing forward when opened.",
        )
        ctx.expect_contact(door, cabinet_shell)
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
