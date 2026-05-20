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

COUNTER_WIDTH = 1.28
COUNTER_DEPTH = 0.74
COUNTER_THICKNESS = 0.032

GLASS_WIDTH = 0.90
GLASS_DEPTH = 0.52
GLASS_THICKNESS = 0.006
GLASS_CENTER_Y = 0.02
GLASS_CENTER_Z = COUNTER_THICKNESS + GLASS_THICKNESS * 0.5

CUTOUT_WIDTH = 0.79
CUTOUT_DEPTH = 0.44
CUTOUT_CENTER_Y = GLASS_CENTER_Y

PAN_WIDTH = 0.75
PAN_DEPTH = 0.40
PAN_TOP_Z = COUNTER_THICKNESS
PAN_BOTTOM_Z = -0.082
PAN_HEIGHT = PAN_TOP_Z - PAN_BOTTOM_Z
PAN_CENTER_Z = (PAN_TOP_Z + PAN_BOTTOM_Z) * 0.5
PAN_WALL_THICKNESS = 0.004
PAN_BOTTOM_THICKNESS = 0.004
PAN_CENTER_Y = GLASS_CENTER_Y

FASCIA_WIDTH = 0.88
FASCIA_DEPTH = 0.018
FASCIA_HEIGHT = 0.034
FASCIA_CENTER_Y = -0.238
FASCIA_CENTER_Z = -0.028
FASCIA_FRONT_FACE_Y = FASCIA_CENTER_Y - FASCIA_DEPTH * 0.5
FASCIA_BACK_FACE_Y = FASCIA_CENTER_Y + FASCIA_DEPTH * 0.5
FASCIA_RAIL_HEIGHT = 0.007
FASCIA_OPENING_HEIGHT = FASCIA_HEIGHT - 2.0 * FASCIA_RAIL_HEIGHT

CONTROL_BRIDGE_DEPTH = 0.049
CONTROL_BRIDGE_CENTER_Y = FASCIA_BACK_FACE_Y + CONTROL_BRIDGE_DEPTH * 0.5

KNOB_X = 0.365
KNOB_HOLE_RADIUS = 0.008
BUTTON_XS = (-0.060, 0.0, 0.060)
BUTTON_SLOT_WIDTH = 0.010
BUTTON_SLOT_HEIGHT = 0.010


def _build_counter_slab_mesh():
    slab_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(COUNTER_WIDTH, COUNTER_DEPTH, radius=0.028),
        [rounded_rect_profile(CUTOUT_WIDTH, CUTOUT_DEPTH, radius=0.014)],
        COUNTER_THICKNESS,
        center=True,
    )
    slab_geom.translate(0.0, 0.0, COUNTER_THICKNESS * 0.5)
    return mesh_from_geometry(slab_geom, ASSETS.mesh_path("counter_slab.obj"))


def _add_zone_marking(
    glass_part,
    *,
    name: str,
    center_x: float,
    center_y: float,
    outer_radius: float,
    inner_radius: float,
    ring_material,
    mask_material,
) -> None:
    glass_part.visual(
        Cylinder(radius=outer_radius, length=0.0010),
        origin=Origin(xyz=(center_x, center_y, GLASS_CENTER_Z + 0.0022)),
        material=ring_material,
        name=f"{name}_outer",
    )
    glass_part.visual(
        Cylinder(radius=inner_radius, length=0.0013),
        origin=Origin(xyz=(center_x, center_y, GLASS_CENTER_Z + 0.0023)),
        material=mask_material,
        name=f"{name}_inner",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_zone_glass_cooktop", assets=ASSETS)

    stone = model.material("stone_quartz", rgba=(0.80, 0.80, 0.78, 1.0))
    glass_black = model.material("glass_black", rgba=(0.07, 0.08, 0.10, 0.94))
    burner_mark = model.material("burner_mark", rgba=(0.58, 0.60, 0.64, 0.82))
    burner_mask = model.material("burner_mask", rgba=(0.05, 0.05, 0.06, 0.96))
    chassis_steel = model.material("chassis_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.13, 0.14, 0.16, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    knob_indicator = model.material("knob_indicator", rgba=(0.83, 0.83, 0.82, 1.0))
    button_finish = model.material("button_finish", rgba=(0.22, 0.23, 0.25, 1.0))

    slab = model.part("counter_slab")
    slab.visual(_build_counter_slab_mesh(), material=stone, name="slab_shell")
    slab.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_THICKNESS * 0.5)),
    )

    chassis = model.part("cooktop_chassis")
    chassis.visual(
        Box((PAN_WIDTH - 2.0 * PAN_WALL_THICKNESS, PAN_DEPTH - 2.0 * PAN_WALL_THICKNESS, PAN_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                PAN_CENTER_Y,
                PAN_BOTTOM_Z - PAN_BOTTOM_THICKNESS * 0.5,
            )
        ),
        material=chassis_steel,
        name="pan_bottom",
    )
    chassis.visual(
        Box((PAN_WIDTH, PAN_WALL_THICKNESS, PAN_HEIGHT)),
        origin=Origin(xyz=(0.0, PAN_CENTER_Y - PAN_DEPTH * 0.5 + PAN_WALL_THICKNESS * 0.5, PAN_CENTER_Z)),
        material=chassis_steel,
        name="pan_front_wall",
    )
    chassis.visual(
        Box((PAN_WIDTH, PAN_WALL_THICKNESS, PAN_HEIGHT)),
        origin=Origin(xyz=(0.0, PAN_CENTER_Y + PAN_DEPTH * 0.5 - PAN_WALL_THICKNESS * 0.5, PAN_CENTER_Z)),
        material=chassis_steel,
        name="pan_back_wall",
    )
    chassis.visual(
        Box((PAN_WALL_THICKNESS, PAN_DEPTH - 2.0 * PAN_WALL_THICKNESS, PAN_HEIGHT)),
        origin=Origin(xyz=(-PAN_WIDTH * 0.5 + PAN_WALL_THICKNESS * 0.5, PAN_CENTER_Y, PAN_CENTER_Z)),
        material=chassis_steel,
        name="pan_left_wall",
    )
    chassis.visual(
        Box((PAN_WALL_THICKNESS, PAN_DEPTH - 2.0 * PAN_WALL_THICKNESS, PAN_HEIGHT)),
        origin=Origin(xyz=(PAN_WIDTH * 0.5 - PAN_WALL_THICKNESS * 0.5, PAN_CENTER_Y, PAN_CENTER_Z)),
        material=chassis_steel,
        name="pan_right_wall",
    )
    chassis.visual(
        Box((FASCIA_WIDTH, FASCIA_DEPTH, FASCIA_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, FASCIA_CENTER_Y, FASCIA_CENTER_Z + (FASCIA_HEIGHT - FASCIA_RAIL_HEIGHT) * 0.5)),
        material=fascia_black,
        name="front_lip",
    )
    chassis.visual(
        Box((FASCIA_WIDTH, FASCIA_DEPTH, FASCIA_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, FASCIA_CENTER_Y, FASCIA_CENTER_Z - (FASCIA_HEIGHT - FASCIA_RAIL_HEIGHT) * 0.5)),
        material=fascia_black,
        name="front_lip_lower_rail",
    )
    for index, (x0, x1) in enumerate(
        (
            (-FASCIA_WIDTH * 0.5, -KNOB_X - 0.014),
            (-KNOB_X + 0.014, BUTTON_XS[0] - 0.008),
            (BUTTON_XS[0] + 0.008, BUTTON_XS[1] - 0.008),
            (BUTTON_XS[1] + 0.008, BUTTON_XS[2] - 0.008),
            (BUTTON_XS[2] + 0.008, KNOB_X - 0.014),
            (KNOB_X + 0.014, FASCIA_WIDTH * 0.5),
        )
    ):
        chassis.visual(
            Box((x1 - x0, FASCIA_DEPTH, FASCIA_OPENING_HEIGHT)),
            origin=Origin(xyz=((x0 + x1) * 0.5, FASCIA_CENTER_Y, FASCIA_CENTER_Z)),
            material=fascia_black,
            name=f"front_lip_post_{index}",
        )
    chassis.visual(
        Box((FASCIA_WIDTH, CONTROL_BRIDGE_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, CONTROL_BRIDGE_CENTER_Y, -0.003)),
        material=fascia_black,
        name="control_top_bridge",
    )
    chassis.visual(
        Box((FASCIA_WIDTH, CONTROL_BRIDGE_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, CONTROL_BRIDGE_CENTER_Y, -0.033)),
        material=fascia_black,
        name="control_bottom_bridge",
    )
    chassis.visual(
        Box((0.030, CONTROL_BRIDGE_DEPTH, FASCIA_HEIGHT)),
        origin=Origin(xyz=(-0.425, CONTROL_BRIDGE_CENTER_Y, FASCIA_CENTER_Z)),
        material=fascia_black,
        name="control_left_cheek",
    )
    chassis.visual(
        Box((0.030, CONTROL_BRIDGE_DEPTH, FASCIA_HEIGHT)),
        origin=Origin(xyz=(0.425, CONTROL_BRIDGE_CENTER_Y, FASCIA_CENTER_Z)),
        material=fascia_black,
        name="control_right_cheek",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((GLASS_WIDTH, GLASS_DEPTH * 0.88, 0.13)),
        mass=11.5,
        origin=Origin(xyz=(0.0, GLASS_CENTER_Y, -0.028)),
    )

    glass = model.part("glass_top")
    glass.visual(
        Box((GLASS_WIDTH, GLASS_DEPTH, GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, GLASS_CENTER_Y, GLASS_CENTER_Z)),
        material=glass_black,
        name="top_glass",
    )
    _add_zone_marking(
        glass,
        name="zone_back_left",
        center_x=-0.225,
        center_y=0.125,
        outer_radius=0.095,
        inner_radius=0.078,
        ring_material=burner_mark,
        mask_material=burner_mask,
    )
    _add_zone_marking(
        glass,
        name="zone_front_left",
        center_x=-0.235,
        center_y=-0.080,
        outer_radius=0.112,
        inner_radius=0.093,
        ring_material=burner_mark,
        mask_material=burner_mask,
    )
    _add_zone_marking(
        glass,
        name="zone_center",
        center_x=0.0,
        center_y=0.018,
        outer_radius=0.126,
        inner_radius=0.103,
        ring_material=burner_mark,
        mask_material=burner_mask,
    )
    _add_zone_marking(
        glass,
        name="zone_back_right",
        center_x=0.225,
        center_y=0.125,
        outer_radius=0.095,
        inner_radius=0.078,
        ring_material=burner_mark,
        mask_material=burner_mask,
    )
    _add_zone_marking(
        glass,
        name="zone_front_right",
        center_x=0.235,
        center_y=-0.080,
        outer_radius=0.112,
        inner_radius=0.093,
        ring_material=burner_mark,
        mask_material=burner_mask,
    )
    glass.inertial = Inertial.from_geometry(
        Box((GLASS_WIDTH, GLASS_DEPTH, GLASS_THICKNESS)),
        mass=7.2,
        origin=Origin(xyz=(0.0, GLASS_CENTER_Y, GLASS_CENTER_Z)),
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=KNOB_HOLE_RADIUS, length=FASCIA_DEPTH),
        origin=Origin(xyz=(0.0, FASCIA_DEPTH * 0.5 - 0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="spindle",
    )
    left_knob.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    left_knob.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_skirt",
    )
    left_knob.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, 0.012)),
        material=knob_indicator,
        name="indicator_mark",
    )
    left_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.028),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=KNOB_HOLE_RADIUS, length=FASCIA_DEPTH),
        origin=Origin(xyz=(0.0, FASCIA_DEPTH * 0.5 - 0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="spindle",
    )
    right_knob.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    right_knob.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_skirt",
    )
    right_knob.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, 0.012)),
        material=knob_indicator,
        name="indicator_mark",
    )
    right_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.028),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    button_names = ("left_button", "center_button", "right_button")
    button_parts = []
    for button_name in button_names:
        button = model.part(button_name)
        button.visual(
            Box((BUTTON_SLOT_WIDTH, FASCIA_DEPTH, BUTTON_SLOT_HEIGHT)),
            origin=Origin(xyz=(0.0, FASCIA_DEPTH * 0.5, 0.0)),
            material=button_finish,
            name="button_stem",
        )
        button.visual(
            Box((0.026, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.026, 0.028, 0.012)),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
        )
        button_parts.append(button)

    model.articulation(
        "slab_to_chassis",
        ArticulationType.FIXED,
        parent=slab,
        child=chassis,
        origin=Origin(),
    )
    model.articulation(
        "chassis_to_glass",
        ArticulationType.FIXED,
        parent=chassis,
        child=glass,
        origin=Origin(),
    )
    model.articulation(
        "chassis_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=left_knob,
        origin=Origin(xyz=(-KNOB_X, FASCIA_FRONT_FACE_Y, FASCIA_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "chassis_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=right_knob,
        origin=Origin(xyz=(KNOB_X, FASCIA_FRONT_FACE_Y, FASCIA_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    for button_name, button_part, button_x in zip(button_names, button_parts, BUTTON_XS):
        model.articulation(
            f"chassis_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=button_part,
            origin=Origin(xyz=(button_x, FASCIA_FRONT_FACE_Y, FASCIA_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    slab = object_model.get_part("counter_slab")
    chassis = object_model.get_part("cooktop_chassis")
    glass = object_model.get_part("glass_top")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    left_button = object_model.get_part("left_button")
    center_button = object_model.get_part("center_button")
    right_button = object_model.get_part("right_button")

    top_glass = glass.get_visual("top_glass")
    slab_shell = slab.get_visual("slab_shell")
    front_lip = chassis.get_visual("front_lip")
    zone_center = glass.get_visual("zone_center_outer")
    zone_front_left = glass.get_visual("zone_front_left_outer")
    zone_front_right = glass.get_visual("zone_front_right_outer")

    left_knob_joint = object_model.get_articulation("chassis_to_left_knob")
    right_knob_joint = object_model.get_articulation("chassis_to_right_knob")
    left_button_joint = object_model.get_articulation("chassis_to_left_button")
    center_button_joint = object_model.get_articulation("chassis_to_center_button")
    right_button_joint = object_model.get_articulation("chassis_to_right_button")

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
        "controls_are_only_articulated_user_inputs",
        len(object_model.articulations) == 7,
        details=f"Expected 7 articulations total (2 fixed, 2 knobs, 3 buttons); got {len(object_model.articulations)}.",
    )
    ctx.check(
        "left_knob_is_continuous",
        left_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Left knob should rotate continuously.",
    )
    ctx.check(
        "right_knob_is_continuous",
        right_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Right knob should rotate continuously.",
    )
    ctx.check(
        "knob_axes_front_to_back",
        tuple(left_knob_joint.axis) == (0.0, 1.0, 0.0) and tuple(right_knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Knob axes should be (0, 1, 0), got {left_knob_joint.axis} and {right_knob_joint.axis}.",
    )
    for name, joint in (
        ("left_button", left_button_joint),
        ("center_button", center_button_joint),
        ("right_button", right_button_joint),
    ):
        limits = joint.motion_limits
        ok = (
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0035 <= limits.upper <= 0.0045
        )
        ctx.check(
            f"{name}_is_short_inward_prismatic",
            ok,
            details=f"{name} should be a short inward prismatic button; joint={joint.articulation_type}, axis={joint.axis}, limits={limits}.",
        )

    ctx.expect_contact(chassis, slab, name="chassis_bears_on_slab")
    ctx.expect_contact(glass, slab, elem_a=top_glass, elem_b=slab_shell, name="glass_rests_on_slab")
    ctx.expect_contact(glass, chassis, elem_a=top_glass, name="glass_seated_on_chassis")
    ctx.expect_gap(
        glass,
        slab,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=top_glass,
        negative_elem=slab_shell,
        name="glass_flush_on_slab",
    )
    ctx.expect_overlap(glass, slab, axes="xy", min_overlap=0.45, elem_a=top_glass, elem_b=slab_shell)
    ctx.expect_within(chassis, glass, axes="xy", margin=0.08, outer_elem=top_glass)
    ctx.expect_within(glass, slab, axes="xy", margin=0.0, inner_elem=top_glass, outer_elem=slab_shell)

    ctx.expect_within(glass, glass, axes="xy", margin=0.0, inner_elem=zone_center, outer_elem=top_glass)
    ctx.expect_within(glass, glass, axes="xy", margin=0.0, inner_elem=zone_front_left, outer_elem=top_glass)
    ctx.expect_within(glass, glass, axes="xy", margin=0.0, inner_elem=zone_front_right, outer_elem=top_glass)

    for control, label in (
        (left_knob, "left_knob"),
        (right_knob, "right_knob"),
        (left_button, "left_button"),
        (center_button, "center_button"),
        (right_button, "right_button"),
    ):
        ctx.expect_contact(control, chassis, name=f"{label}_mounted_to_front_lip")
        ctx.expect_overlap(control, chassis, axes="z", min_overlap=0.010, name=f"{label}_shares_front_lip_height_band")

    left_indicator_rest = ctx.part_element_world_aabb(left_knob, elem="indicator_mark")
    ctx.check(
        "left_indicator_exists",
        left_indicator_rest is not None,
        details="Left knob indicator visual should be measurable.",
    )
    with ctx.pose({left_knob_joint: math.pi / 2.0, right_knob_joint: -math.pi / 3.0}):
        ctx.expect_contact(left_knob, chassis, name="left_knob_turned_contact")
        ctx.expect_contact(right_knob, chassis, name="right_knob_turned_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_turned_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_turned_no_floating")
        left_indicator_turn = ctx.part_element_world_aabb(left_knob, elem="indicator_mark")
        rest_center_xz = None
        turn_center_xz = None
        if left_indicator_rest is not None:
            rest_center_xz = (
                (left_indicator_rest[0][0] + left_indicator_rest[1][0]) * 0.5,
                (left_indicator_rest[0][2] + left_indicator_rest[1][2]) * 0.5,
            )
        if left_indicator_turn is not None:
            turn_center_xz = (
                (left_indicator_turn[0][0] + left_indicator_turn[1][0]) * 0.5,
                (left_indicator_turn[0][2] + left_indicator_turn[1][2]) * 0.5,
            )
        indicator_rotates = (
            rest_center_xz is not None
            and turn_center_xz is not None
            and math.hypot(
                turn_center_xz[0] - rest_center_xz[0],
                turn_center_xz[1] - rest_center_xz[1],
            )
            > 0.008
        )
        ctx.check(
            "left_knob_indicator_moves_when_rotated",
            indicator_rotates,
            details=f"Indicator center should move around the knob axis when rotated. rest={left_indicator_rest}, turned={left_indicator_turn}",
        )

    for label, button_part, joint in (
        ("left_button", left_button, left_button_joint),
        ("center_button", center_button, center_button_joint),
        ("right_button", right_button, right_button_joint),
    ):
        rest_pos = ctx.part_world_position(button_part)
        limits = joint.motion_limits
        assert limits is not None and limits.upper is not None
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
            moves_inward = (
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] > rest_pos[1] + 0.0035
            )
            ctx.check(
                f"{label}_moves_inward_when_pressed",
                moves_inward,
                details=f"{label} should move toward +Y when pressed. rest={rest_pos}, pressed={pressed_pos}",
            )
            ctx.expect_contact(button_part, chassis, name=f"{label}_pressed_stays_guided")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_pressed_no_floating")

    lip_aabb = ctx.part_element_world_aabb(chassis, elem=front_lip)
    glass_aabb = ctx.part_element_world_aabb(glass, elem=top_glass)
    ctx.check(
        "front_lip_sits_below_glass",
        lip_aabb is not None and glass_aabb is not None and lip_aabb[1][2] < glass_aabb[0][2],
        details=f"Front lip should hang below the glass panel. lip={lip_aabb}, glass={glass_aabb}",
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
