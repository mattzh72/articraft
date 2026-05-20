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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.90
BODY_DEPTH = 0.50
CANOPY_HEIGHT = 0.26
TOP_WIDTH = 0.34
TOP_DEPTH = 0.24
SHEET = 0.012
FRONT_STRIP_HEIGHT = 0.075

CHIMNEY_OUTER_WIDTH = 0.30
CHIMNEY_OUTER_DEPTH = 0.22
CHIMNEY_WALL = 0.012
CHIMNEY_HEIGHT = 0.50

KNOB_RADIUS = 0.022
KNOB_DEPTH = 0.020
KNOB_X = BODY_WIDTH / 2.0 - 0.064

BUTTON_WIDTH = 0.020
BUTTON_HEIGHT = 0.028
BUTTON_PROUD = 0.006
BUTTON_TRAVEL = 0.006
BUTTON_CENTER_OFFSET = 0.014
CONTROL_Z = FRONT_STRIP_HEIGHT / 2.0


def _centered_rect(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _upright_rect(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    return [
        (-half_w, 0.0),
        (half_w, 0.0),
        (half_w, height),
        (-half_w, height),
    ]


def _upright_trapezoid(
    bottom_width: float,
    top_width: float,
    height: float,
) -> list[tuple[float, float]]:
    return [
        (-bottom_width / 2.0, 0.0),
        (bottom_width / 2.0, 0.0),
        (top_width / 2.0, height),
        (-top_width / 2.0, height),
    ]


def _mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_front_strip_mesh():
    outer = _upright_rect(BODY_WIDTH, FRONT_STRIP_HEIGHT)
    hole_left = _upright_rect(BUTTON_WIDTH, BUTTON_HEIGHT)
    hole_right = _upright_rect(BUTTON_WIDTH, BUTTON_HEIGHT)
    hole_left = [(x - BUTTON_CENTER_OFFSET, y + CONTROL_Z - BUTTON_HEIGHT / 2.0) for x, y in hole_left]
    hole_right = [(x + BUTTON_CENTER_OFFSET, y + CONTROL_Z - BUTTON_HEIGHT / 2.0) for x, y in hole_right]
    geom = ExtrudeWithHolesGeometry(
        outer,
        [hole_left, hole_right],
        SHEET,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return geom


def _build_front_upper_mesh():
    front_inset = (BODY_DEPTH - TOP_DEPTH) / 2.0
    rise = CANOPY_HEIGHT - SHEET / 2.0 - FRONT_STRIP_HEIGHT
    slope_len = math.hypot(front_inset, rise)
    geom = ExtrudeGeometry(
        _upright_trapezoid(BODY_WIDTH, TOP_WIDTH, slope_len),
        SHEET,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi - math.atan2(rise, front_inset))
    return geom


def _build_back_upper_mesh():
    back_inset = (BODY_DEPTH - TOP_DEPTH) / 2.0
    rise = CANOPY_HEIGHT - SHEET / 2.0
    slope_len = math.hypot(back_inset, rise)
    geom = ExtrudeGeometry(
        _upright_trapezoid(BODY_WIDTH, TOP_WIDTH, slope_len),
        SHEET,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.atan2(rise, back_inset))
    return geom


def _build_side_mesh(*, left: bool):
    width_inset = (BODY_WIDTH - TOP_WIDTH) / 2.0
    rise = CANOPY_HEIGHT - SHEET / 2.0
    slope_len = math.hypot(width_inset, rise)
    geom = ExtrudeGeometry(
        _upright_trapezoid(BODY_DEPTH, TOP_DEPTH, slope_len),
        SHEET,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_z(math.pi / 2.0)
    geom.rotate_y(math.pi - math.atan2(rise, width_inset) if left else math.atan2(rise, width_inset))
    return geom


def _build_top_ring_mesh():
    chimney_inner_width = CHIMNEY_OUTER_WIDTH - 2.0 * CHIMNEY_WALL
    chimney_inner_depth = CHIMNEY_OUTER_DEPTH - 2.0 * CHIMNEY_WALL
    return ExtrudeWithHolesGeometry(
        _centered_rect(TOP_WIDTH, TOP_DEPTH),
        [_centered_rect(chimney_inner_width, chimney_inner_depth)],
        SHEET,
        cap=True,
        center=True,
        closed=True,
    )


def _build_chimney_shell_mesh():
    return ExtrudeWithHolesGeometry(
        _centered_rect(CHIMNEY_OUTER_WIDTH, CHIMNEY_OUTER_DEPTH),
        [_centered_rect(CHIMNEY_OUTER_WIDTH - 2.0 * CHIMNEY_WALL, CHIMNEY_OUTER_DEPTH - 2.0 * CHIMNEY_WALL)],
        CHIMNEY_HEIGHT,
        cap=False,
        center=False,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_grey = model.material("button_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    indicator_silver = model.material("indicator_silver", rgba=(0.90, 0.91, 0.93, 1.0))

    canopy = model.part("canopy")
    left_hole_left = -BUTTON_CENTER_OFFSET - BUTTON_WIDTH / 2.0
    left_hole_right = -BUTTON_CENTER_OFFSET + BUTTON_WIDTH / 2.0
    right_hole_left = BUTTON_CENTER_OFFSET - BUTTON_WIDTH / 2.0
    right_hole_right = BUTTON_CENTER_OFFSET + BUTTON_WIDTH / 2.0
    bottom_rail_height = CONTROL_Z - BUTTON_HEIGHT / 2.0
    top_rail_height = FRONT_STRIP_HEIGHT - (CONTROL_Z + BUTTON_HEIGHT / 2.0)
    left_web_width = left_hole_left + BODY_WIDTH / 2.0
    center_web_width = right_hole_left - left_hole_right
    right_web_width = BODY_WIDTH / 2.0 - right_hole_right
    front_face_y = BODY_DEPTH / 2.0 - SHEET / 2.0

    canopy.visual(
        Box((BODY_WIDTH, SHEET, bottom_rail_height)),
        origin=Origin(xyz=(0.0, front_face_y, bottom_rail_height / 2.0)),
        material=stainless,
        name="front_strip_lower",
    )
    canopy.visual(
        Box((BODY_WIDTH, SHEET, top_rail_height)),
        origin=Origin(
            xyz=(0.0, front_face_y, FRONT_STRIP_HEIGHT - top_rail_height / 2.0),
        ),
        material=stainless,
        name="front_strip_upper",
    )
    canopy.visual(
        Box((left_web_width, SHEET, BUTTON_HEIGHT)),
        origin=Origin(
            xyz=((-BODY_WIDTH / 2.0 + left_hole_left) / 2.0, front_face_y, CONTROL_Z),
        ),
        material=stainless,
        name="front_strip_left",
    )
    canopy.visual(
        Box((center_web_width, SHEET, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, front_face_y, CONTROL_Z)),
        material=stainless,
        name="front_strip_center",
    )
    canopy.visual(
        Box((right_web_width, SHEET, BUTTON_HEIGHT)),
        origin=Origin(
            xyz=((BODY_WIDTH / 2.0 + right_hole_right) / 2.0, front_face_y, CONTROL_Z),
        ),
        material=stainless,
        name="front_strip_right",
    )
    canopy.visual(
        _mesh(_build_front_upper_mesh(), "range_hood_front_upper.obj"),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - SHEET / 2.0, FRONT_STRIP_HEIGHT)),
        material=stainless,
        name="front_upper",
    )
    canopy.visual(
        _mesh(_build_back_upper_mesh(), "range_hood_back_upper.obj"),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + SHEET / 2.0, 0.0)),
        material=stainless,
        name="back_upper",
    )
    canopy.visual(
        _mesh(_build_side_mesh(left=True), "range_hood_left_side.obj"),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + SHEET / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_side",
    )
    canopy.visual(
        _mesh(_build_side_mesh(left=False), "range_hood_right_side.obj"),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - SHEET / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_side",
    )
    canopy.visual(
        _mesh(_build_top_ring_mesh(), "range_hood_top_ring.obj"),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - SHEET / 2.0)),
        material=shadow_steel,
        name="top_ring",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, CANOPY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT / 2.0)),
    )

    chimney = model.part("chimney")
    chimney.visual(
        _mesh(_build_chimney_shell_mesh(), "range_hood_chimney_shell.obj"),
        material=stainless,
        name="chimney_shell",
    )
    chimney.inertial = Inertial.from_geometry(
        Box((CHIMNEY_OUTER_WIDTH, CHIMNEY_OUTER_DEPTH, CHIMNEY_HEIGHT)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT / 2.0)),
    )
    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT)),
    )

    def add_knob(name: str, x_pos: float) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
            origin=Origin(xyz=(0.0, KNOB_DEPTH / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=control_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.006, 0.002, 0.012)),
            origin=Origin(xyz=(0.0, KNOB_DEPTH - 0.001, KNOB_RADIUS - 0.006)),
            material=indicator_silver,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
            mass=0.12,
            origin=Origin(xyz=(0.0, KNOB_DEPTH / 2.0, 0.0)),
        )
        model.articulation(
            f"canopy_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=canopy,
            child=knob,
            origin=Origin(xyz=(x_pos, BODY_DEPTH / 2.0, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        )

    def add_button(name: str, x_pos: float) -> None:
        button = model.part(name)
        button_depth = SHEET
        button.visual(
            Box((BUTTON_WIDTH, button_depth, BUTTON_HEIGHT)),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
            ),
            material=button_grey,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_WIDTH, button_depth, BUTTON_HEIGHT)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"canopy_to_{name}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_DEPTH / 2.0, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    add_knob("left_knob", -KNOB_X)
    add_knob("right_knob", KNOB_X)
    add_button("left_button", -BUTTON_CENTER_OFFSET)
    add_button("right_button", BUTTON_CENTER_OFFSET)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    top_ring = canopy.get_visual("top_ring")
    chimney_shell = chimney.get_visual("chimney_shell")

    canopy_to_chimney = object_model.get_articulation("canopy_to_chimney")
    left_knob_joint = object_model.get_articulation("canopy_to_left_knob")
    right_knob_joint = object_model.get_articulation("canopy_to_right_knob")
    left_button_joint = object_model.get_articulation("canopy_to_left_button")
    right_button_joint = object_model.get_articulation("canopy_to_right_button")

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
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="pose_sweep_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_fixed=False,
    )

    ctx.expect_contact(chimney, canopy, elem_a=chimney_shell, elem_b=top_ring, name="chimney_seats_on_top_ring")
    ctx.expect_contact(left_knob, canopy, name="left_knob_mounted_to_front_strip")
    ctx.expect_contact(right_knob, canopy, name="right_knob_mounted_to_front_strip")
    ctx.expect_contact(left_button, canopy, name="left_button_guided_by_front_strip")
    ctx.expect_contact(right_button, canopy, name="right_button_guided_by_front_strip")

    ctx.expect_origin_distance(left_knob, right_knob, axes="x", min_dist=0.72, max_dist=0.80, name="corner_knob_spacing")
    ctx.expect_origin_distance(left_button, right_button, axes="x", min_dist=0.025, max_dist=0.035, name="center_button_spacing")
    ctx.expect_origin_distance(left_knob, left_button, axes="x", min_dist=0.34, max_dist=0.40, name="left_controls_separated")
    ctx.expect_origin_gap(chimney, canopy, axis="z", min_gap=CANOPY_HEIGHT - 0.001, max_gap=CANOPY_HEIGHT + 0.001, name="chimney_origin_above_canopy")

    ctx.check(
        "chimney_joint_fixed",
        canopy_to_chimney.articulation_type == ArticulationType.FIXED,
        f"Expected fixed chimney mount, got {canopy_to_chimney.articulation_type}.",
    )
    ctx.check(
        "knob_joints_continuous",
        left_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        "Corner controls should be continuous rotary knobs.",
    )
    ctx.check(
        "button_joints_prismatic",
        left_button_joint.articulation_type == ArticulationType.PRISMATIC
        and right_button_joint.articulation_type == ArticulationType.PRISMATIC,
        "Center controls should be prismatic push-buttons.",
    )
    ctx.check(
        "knob_axes_face_forward",
        left_knob_joint.axis == (0.0, 1.0, 0.0) and right_knob_joint.axis == (0.0, 1.0, 0.0),
        f"Knob axes should point outward along +Y, got {left_knob_joint.axis} and {right_knob_joint.axis}.",
    )
    ctx.check(
        "button_axes_press_inward",
        left_button_joint.axis == (0.0, -1.0, 0.0) and right_button_joint.axis == (0.0, -1.0, 0.0),
        f"Button axes should press inward along -Y, got {left_button_joint.axis} and {right_button_joint.axis}.",
    )

    left_knob_limits = left_knob_joint.motion_limits
    right_knob_limits = right_knob_joint.motion_limits
    left_button_limits = left_button_joint.motion_limits
    right_button_limits = right_button_joint.motion_limits
    ctx.check(
        "continuous_knobs_unbounded",
        left_knob_limits is not None
        and right_knob_limits is not None
        and left_knob_limits.lower is None
        and left_knob_limits.upper is None
        and right_knob_limits.lower is None
        and right_knob_limits.upper is None,
        "Continuous knob joints must not set lower or upper limits.",
    )
    ctx.check(
        "button_travel_short",
        left_button_limits is not None
        and right_button_limits is not None
        and left_button_limits.upper is not None
        and right_button_limits.upper is not None
        and 0.003 <= left_button_limits.upper <= 0.010
        and 0.003 <= right_button_limits.upper <= 0.010,
        "Buttons should have short inward travel.",
    )

    canopy_aabb = ctx.part_world_aabb(canopy)
    chimney_aabb = ctx.part_world_aabb(chimney)
    ctx.check(
        "canopy_realistic_width",
        canopy_aabb is not None and 0.86 <= canopy_aabb[1][0] - canopy_aabb[0][0] <= 0.94,
        f"Canopy width should read as a full-size range hood, got {canopy_aabb}.",
    )
    ctx.check(
        "overall_realistic_height",
        chimney_aabb is not None and 0.72 <= chimney_aabb[1][2] <= 0.85,
        f"Overall height should include a tall chimney shell, got {chimney_aabb}.",
    )

    with ctx.pose({left_knob_joint: 1.6, right_knob_joint: -1.2}):
        ctx.expect_contact(left_knob, canopy, name="left_knob_contact_when_rotated")
        ctx.expect_contact(right_knob, canopy, name="right_knob_contact_when_rotated")

    for button, joint, label in (
        (left_button, left_button_joint, "left_button"),
        (right_button, right_button_joint, "right_button"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            rest_position = ctx.part_world_position(button)
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
                ctx.expect_contact(button, canopy, name=f"{label}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                pressed_position = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")
                ctx.expect_contact(button, canopy, name=f"{label}_upper_contact")
                ctx.check(
                    f"{label}_moves_inward",
                    rest_position is not None
                    and pressed_position is not None
                    and pressed_position[1] < rest_position[1] - 0.003,
                    f"{label} should shift inward on press, got rest={rest_position}, pressed={pressed_position}.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
