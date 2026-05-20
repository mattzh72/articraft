from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
)

ASSETS = AssetContext.from_script(__file__)


CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.060
SHEET = 0.002

CHIMNEY_WIDTH = 0.28
CHIMNEY_DEPTH = 0.22
CHIMNEY_HEIGHT = 0.65

BUTTON_WIDTH = 0.032
BUTTON_HEIGHT = 0.013
BUTTON_DEPTH = 0.014
BUTTON_TRAVEL = 0.008

KNOB_RADIUS = 0.020
KNOB_DEPTH = 0.018
KNOB_SHAFT_RADIUS = 0.007
KNOB_SHAFT_DEPTH = 0.018

FRONT_PANEL_WIDTH = CANOPY_WIDTH - 2.0 * SHEET
FRONT_PANEL_HEIGHT = CANOPY_HEIGHT - SHEET
FRONT_PANEL_CENTER_Y = (CANOPY_DEPTH * 0.5) - (SHEET * 0.5)
FRONT_PANEL_CENTER_Z = FRONT_PANEL_HEIGHT * 0.5

UPPER_BUTTON_X = -0.285
LOWER_BUTTON_X = -0.285
BUTTON_UPPER_Z = 0.039
BUTTON_LOWER_Z = 0.018
KNOB_X = 0.295
KNOB_Z = 0.029


def _rect_profile(
    width: float,
    height: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * index) / segments),
            cy + radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _front_panel_mesh():
    outer = _rect_profile(FRONT_PANEL_WIDTH, FRONT_PANEL_HEIGHT)
    hole_profiles = [
        _rect_profile(
            BUTTON_WIDTH,
            BUTTON_HEIGHT,
            center=(
                UPPER_BUTTON_X,
                -(BUTTON_UPPER_Z - FRONT_PANEL_CENTER_Z),
            ),
        ),
        _rect_profile(
            BUTTON_WIDTH,
            BUTTON_HEIGHT,
            center=(
                LOWER_BUTTON_X,
                -(BUTTON_LOWER_Z - FRONT_PANEL_CENTER_Z),
            ),
        ),
        _circle_profile(
            KNOB_SHAFT_RADIUS,
            center=(
                KNOB_X,
                -(KNOB_Z - FRONT_PANEL_CENTER_Z),
            ),
        ),
    ]
    geom = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=SHEET,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(-pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("range_hood_front_panel.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - (SHEET * 0.5))),
        material=stainless,
        name="canopy_top",
    )
    housing.visual(
        Box((SHEET, CANOPY_DEPTH, FRONT_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CANOPY_WIDTH * 0.5) + (SHEET * 0.5),
                0.0,
                FRONT_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="canopy_left_side",
    )
    housing.visual(
        Box((SHEET, CANOPY_DEPTH, FRONT_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                (CANOPY_WIDTH * 0.5) - (SHEET * 0.5),
                0.0,
                FRONT_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="canopy_right_side",
    )
    housing.visual(
        Box((FRONT_PANEL_WIDTH, SHEET, FRONT_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CANOPY_DEPTH * 0.5) + (SHEET * 0.5),
                FRONT_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="canopy_rear_panel",
    )
    housing.visual(
        _front_panel_mesh(),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_PANEL_CENTER_Y,
                FRONT_PANEL_CENTER_Z,
            )
        ),
        material=stainless,
        name="canopy_front_panel",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, SHEET)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CANOPY_HEIGHT + CHIMNEY_HEIGHT - (SHEET * 0.5),
            )
        ),
        material=stainless,
        name="chimney_cap",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH - (2.0 * SHEET), SHEET, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CHIMNEY_DEPTH * 0.5) - (SHEET * 0.5),
                CANOPY_HEIGHT + (CHIMNEY_HEIGHT * 0.5),
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH - (2.0 * SHEET), SHEET, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CHIMNEY_DEPTH * 0.5) + (SHEET * 0.5),
                CANOPY_HEIGHT + (CHIMNEY_HEIGHT * 0.5),
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    housing.visual(
        Box((SHEET, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CHIMNEY_WIDTH * 0.5) + (SHEET * 0.5),
                0.0,
                CANOPY_HEIGHT + (CHIMNEY_HEIGHT * 0.5),
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    housing.visual(
        Box((SHEET, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                (CHIMNEY_WIDTH * 0.5) - (SHEET * 0.5),
                0.0,
                CANOPY_HEIGHT + (CHIMNEY_HEIGHT * 0.5),
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH - 0.030, CHIMNEY_DEPTH - 0.030, SHEET)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CANOPY_HEIGHT + (SHEET * 0.5),
            )
        ),
        material=charcoal,
        name="chimney_transition_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT + CHIMNEY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (CANOPY_HEIGHT + CHIMNEY_HEIGHT) * 0.5)),
    )

    upper_button = model.part("upper_button")
    upper_button.visual(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=satin_dark,
        name="button_body",
    )
    upper_button.visual(
        Box((BUTTON_WIDTH - 0.006, 0.002, BUTTON_HEIGHT - 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=charcoal,
        name="button_face",
    )
    upper_button.inertial = Inertial.from_geometry(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
    )

    lower_button = model.part("lower_button")
    lower_button.visual(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=satin_dark,
        name="button_body",
    )
    lower_button.visual(
        Box((BUTTON_WIDTH - 0.006, 0.002, BUTTON_HEIGHT - 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=charcoal,
        name="button_face",
    )
    lower_button.inertial = Inertial.from_geometry(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(xyz=(0.0, KNOB_DEPTH * 0.5, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_dark,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_DEPTH),
        origin=Origin(
            xyz=(0.0, -(KNOB_SHAFT_DEPTH * 0.5), 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="knob_shaft",
    )
    knob.visual(
        Box((0.004, 0.004, 0.013)),
        origin=Origin(xyz=(0.0, KNOB_DEPTH - 0.002, KNOB_RADIUS * 0.55)),
        material=stainless,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        mass=0.08,
        origin=Origin(xyz=(0.0, KNOB_DEPTH * 0.5, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "upper_button_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=upper_button,
        origin=Origin(xyz=(UPPER_BUTTON_X, CANOPY_DEPTH * 0.5, BUTTON_UPPER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "lower_button_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=lower_button,
        origin=Origin(xyz=(LOWER_BUTTON_X, CANOPY_DEPTH * 0.5, BUTTON_LOWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "knob_rotate",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(KNOB_X, CANOPY_DEPTH * 0.5, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")
    knob = object_model.get_part("knob")

    front_panel = housing.get_visual("canopy_front_panel")
    upper_button_joint = object_model.get_articulation("upper_button_press")
    lower_button_joint = object_model.get_articulation("lower_button_press")
    knob_joint = object_model.get_articulation("knob_rotate")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "expected_articulation_count",
        len(object_model.articulations) == 3,
        f"expected 3 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "upper_button_axis",
        tuple(upper_button_joint.axis) == (0.0, -1.0, 0.0),
        f"upper button axis was {upper_button_joint.axis}",
    )
    ctx.check(
        "lower_button_axis",
        tuple(lower_button_joint.axis) == (0.0, -1.0, 0.0),
        f"lower button axis was {lower_button_joint.axis}",
    )
    ctx.check(
        "knob_axis",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        f"knob axis was {knob_joint.axis}",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        "knob should be a continuous rotation control without finite bounds",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is None:
        ctx.fail("housing_aabb_exists", "housing AABB was not available")
    else:
        mins, maxs = housing_aabb
        dims = (
            maxs[0] - mins[0],
            maxs[1] - mins[1],
            maxs[2] - mins[2],
        )
        ctx.check(
            "hood_width_realistic",
            0.88 <= dims[0] <= 0.92,
            f"hood width was {dims[0]:.4f} m",
        )
        ctx.check(
            "hood_depth_realistic",
            0.49 <= dims[1] <= 0.52,
            f"hood depth was {dims[1]:.4f} m",
        )
        ctx.check(
            "hood_height_realistic",
            0.70 <= dims[2] <= 0.73,
            f"hood height was {dims[2]:.4f} m",
        )

    upper_origin = ctx.part_world_position(upper_button)
    lower_origin = ctx.part_world_position(lower_button)
    knob_origin = ctx.part_world_position(knob)
    if upper_origin is None or lower_origin is None or knob_origin is None:
        ctx.fail("control_origins_exist", "one or more control positions were unavailable")
    else:
        ctx.check(
            "buttons_on_left",
            upper_origin[0] < -0.20 and lower_origin[0] < -0.20,
            f"button x positions were {upper_origin[0]:.4f} and {lower_origin[0]:.4f}",
        )
        ctx.check(
            "knob_on_right",
            knob_origin[0] > 0.20,
            f"knob x position was {knob_origin[0]:.4f}",
        )
        ctx.check(
            "controls_on_front_face",
            upper_origin[1] > 0.24 and lower_origin[1] > 0.24 and knob_origin[1] > 0.24,
            f"control y positions were {upper_origin[1]:.4f}, {lower_origin[1]:.4f}, {knob_origin[1]:.4f}",
        )

    ctx.expect_origin_gap(
        upper_button,
        lower_button,
        axis="z",
        min_gap=0.018,
        max_gap=0.024,
        name="buttons_are_vertically_stacked",
    )

    with ctx.pose({upper_button_joint: 0.0, lower_button_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_contact(
            upper_button,
            housing,
            elem_b=front_panel,
            name="upper_button_contacts_front_panel_rest",
        )
        ctx.expect_contact(
            lower_button,
            housing,
            elem_b=front_panel,
            name="lower_button_contacts_front_panel_rest",
        )
        ctx.expect_contact(
            knob,
            housing,
            elem_b=front_panel,
            name="knob_contacts_front_panel_rest",
        )
        ctx.expect_within(
            upper_button,
            housing,
            axes="xz",
            margin=0.0,
            name="upper_button_within_front_span",
        )
        ctx.expect_within(
            lower_button,
            housing,
            axes="xz",
            margin=0.0,
            name="lower_button_within_front_span",
        )
        ctx.expect_within(
            knob,
            housing,
            axes="xz",
            margin=0.0,
            name="knob_within_front_span",
        )

    upper_limits = upper_button_joint.motion_limits
    lower_limits = lower_button_joint.motion_limits
    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_button_joint: upper_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="upper_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="upper_button_pressed_no_floating")
            ctx.expect_contact(
                upper_button,
                housing,
                elem_b=front_panel,
                name="upper_button_pressed_contact",
            )

    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({lower_button_joint: lower_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lower_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="lower_button_pressed_no_floating")
            ctx.expect_contact(
                lower_button,
                housing,
                elem_b=front_panel,
                name="lower_button_pressed_contact",
            )

    if upper_limits is not None and upper_limits.upper is not None and lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose(
            {
                upper_button_joint: upper_limits.upper,
                lower_button_joint: lower_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_buttons_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="both_buttons_pressed_no_floating")

    rest_upper_y = None
    pressed_upper_y = None
    rest_lower_y = None
    pressed_lower_y = None
    with ctx.pose({upper_button_joint: 0.0, lower_button_joint: 0.0}):
        upper_pos = ctx.part_world_position(upper_button)
        lower_pos = ctx.part_world_position(lower_button)
        if upper_pos is not None:
            rest_upper_y = upper_pos[1]
        if lower_pos is not None:
            rest_lower_y = lower_pos[1]
    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_button_joint: upper_limits.upper}):
            upper_pos = ctx.part_world_position(upper_button)
            if upper_pos is not None:
                pressed_upper_y = upper_pos[1]
    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({lower_button_joint: lower_limits.upper}):
            lower_pos = ctx.part_world_position(lower_button)
            if lower_pos is not None:
                pressed_lower_y = lower_pos[1]
    if rest_upper_y is None or pressed_upper_y is None:
        ctx.fail("upper_button_motion_measured", "could not measure upper button travel")
    else:
        ctx.check(
            "upper_button_moves_inward",
            pressed_upper_y < rest_upper_y - 0.007,
            f"upper button y moved from {rest_upper_y:.4f} to {pressed_upper_y:.4f}",
        )
    if rest_lower_y is None or pressed_lower_y is None:
        ctx.fail("lower_button_motion_measured", "could not measure lower button travel")
    else:
        ctx.check(
            "lower_button_moves_inward",
            pressed_lower_y < rest_lower_y - 0.007,
            f"lower button y moved from {rest_lower_y:.4f} to {pressed_lower_y:.4f}",
        )

    with ctx.pose({knob_joint: 1.75}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotated_no_floating")
        ctx.expect_contact(
            knob,
            housing,
            elem_b=front_panel,
            name="knob_rotated_contact",
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
