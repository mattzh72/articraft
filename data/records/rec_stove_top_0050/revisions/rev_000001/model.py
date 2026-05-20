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

STOVE_WIDTH = 0.76
STOVE_DEPTH = 0.64
STOVE_HEIGHT = 0.90
HINGE_RADIUS = 0.005

BURNER_SPECS = (
    ("front_left", -0.18, -0.13),
    ("front_right", 0.18, -0.13),
    ("rear_left", -0.18, 0.13),
    ("rear_right", 0.18, 0.13),
)

KNOB_SPECS = (
    ("left_outer_burner_knob", -0.25, 0.785, 0.026, 0.032),
    ("left_inner_burner_knob", -0.13, 0.785, 0.026, 0.032),
    ("oven_selector_knob", 0.00, 0.785, 0.040, 0.042),
    ("right_inner_burner_knob", 0.13, 0.785, 0.026, 0.032),
    ("right_outer_burner_knob", 0.25, 0.785, 0.026, 0.032),
)


def _origin_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _build_knob(model: ArticulatedObject, name: str, radius: float, depth: float):
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=radius * 1.12, length=0.006),
        origin=_origin_y((0.0, -0.003, 0.0)),
        material="chrome_trim",
        name="bezel",
    )
    knob.visual(
        Cylinder(radius=radius, length=depth),
        origin=_origin_y((0.0, -depth / 2.0, 0.0)),
        material="bakelite_black",
        name="body",
    )
    knob.visual(
        Box((radius * 0.24, 0.010, radius * 0.52)),
        origin=Origin(xyz=(0.0, -depth - 0.005, radius * 0.52)),
        material="chrome_trim",
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((radius * 2.3, depth + 0.014, radius * 2.3)),
        mass=0.18 if radius > 0.03 else 0.09,
        origin=Origin(xyz=(0.0, -(depth + 0.004) / 2.0, 0.0)),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_enamel_stove", assets=ASSETS)

    model.material("enamel_cream", rgba=(0.94, 0.90, 0.82, 1.0))
    model.material("chrome_trim", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("cast_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("cavity_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("glass_smoke", rgba=(0.24, 0.31, 0.35, 0.42))
    model.material("bakelite_black", rgba=(0.12, 0.11, 0.10, 1.0))
    model.material("indicator_amber", rgba=(0.79, 0.51, 0.14, 1.0))

    stove_body = model.part("stove_body")
    stove_body.inertial = Inertial.from_geometry(
        Box((STOVE_WIDTH, STOVE_DEPTH, 1.00)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    # Outer enamel body.
    stove_body.visual(
        Box((0.020, STOVE_DEPTH, STOVE_HEIGHT)),
        origin=Origin(xyz=(-0.370, 0.0, STOVE_HEIGHT / 2.0)),
        material="enamel_cream",
        name="left_side_wall",
    )
    stove_body.visual(
        Box((0.020, STOVE_DEPTH, STOVE_HEIGHT)),
        origin=Origin(xyz=(0.370, 0.0, STOVE_HEIGHT / 2.0)),
        material="enamel_cream",
        name="right_side_wall",
    )
    stove_body.visual(
        Box((0.720, 0.020, STOVE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.310, STOVE_HEIGHT / 2.0)),
        material="enamel_cream",
        name="rear_wall",
    )
    stove_body.visual(
        Box((0.720, 0.600, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="enamel_cream",
        name="base_plate",
    )
    stove_body.visual(
        Box((0.720, 0.620, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.885)),
        material="enamel_cream",
        name="top_deck",
    )
    stove_body.visual(
        Cylinder(radius=0.025, length=STOVE_HEIGHT),
        origin=Origin(xyz=(-0.345, -0.295, STOVE_HEIGHT / 2.0)),
        material="enamel_cream",
        name="left_front_round",
    )
    stove_body.visual(
        Cylinder(radius=0.025, length=STOVE_HEIGHT),
        origin=Origin(xyz=(0.345, -0.295, STOVE_HEIGHT / 2.0)),
        material="enamel_cream",
        name="right_front_round",
    )
    stove_body.visual(
        Box((0.720, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, -0.245, 0.0375)),
        material="enamel_cream",
        name="toe_kick",
    )
    stove_body.visual(
        Box((0.080, 0.010, 0.620)),
        origin=Origin(xyz=(-0.320, -0.284, 0.390)),
        material="enamel_cream",
        name="left_door_frame",
    )
    stove_body.visual(
        Box((0.080, 0.010, 0.620)),
        origin=Origin(xyz=(0.320, -0.284, 0.390)),
        material="enamel_cream",
        name="right_door_frame",
    )
    stove_body.visual(
        Box((0.720, 0.030, 0.170)),
        origin=Origin(xyz=(0.0, -0.305, 0.785)),
        material="enamel_cream",
        name="control_fascia",
    )
    stove_body.visual(
        Box((0.720, 0.025, 0.100)),
        origin=Origin(xyz=(0.0, 0.2975, 0.950)),
        material="enamel_cream",
        name="backsplash",
    )
    stove_body.visual(
        Cylinder(radius=0.015, length=0.720),
        origin=_origin_x((0.0, -0.300, 0.900)),
        material="chrome_trim",
        name="stovetop_front_trim",
    )
    stove_body.visual(
        Box((0.720, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.284, 0.685)),
        material="chrome_trim",
        name="door_header_trim",
    )

    # Oven cavity so the door opens onto a real recess.
    stove_body.visual(
        Box((0.520, 0.520, 0.020)),
        origin=Origin(xyz=(0.0, -0.040, 0.110)),
        material="cavity_dark",
        name="oven_floor",
    )
    stove_body.visual(
        Box((0.520, 0.520, 0.020)),
        origin=Origin(xyz=(0.0, -0.040, 0.690)),
        material="cavity_dark",
        name="oven_ceiling",
    )
    stove_body.visual(
        Box((0.020, 0.460, 0.560)),
        origin=Origin(xyz=(-0.270, -0.010, 0.400)),
        material="cavity_dark",
        name="oven_left_wall",
    )
    stove_body.visual(
        Box((0.020, 0.460, 0.560)),
        origin=Origin(xyz=(0.270, -0.010, 0.400)),
        material="cavity_dark",
        name="oven_right_wall",
    )
    stove_body.visual(
        Box((0.520, 0.020, 0.560)),
        origin=Origin(xyz=(0.0, 0.210, 0.400)),
        material="cavity_dark",
        name="oven_back_liner",
    )
    stove_body.visual(
        Box((0.520, 0.460, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.340)),
        material="chrome_trim",
        name="oven_rack",
    )
    stove_body.visual(
        Box((0.060, 0.018, 0.030)),
        origin=Origin(xyz=(-0.290, -0.286, 0.085)),
        material="chrome_trim",
        name="left_hinge_mount",
    )
    stove_body.visual(
        Box((0.060, 0.018, 0.030)),
        origin=Origin(xyz=(0.290, -0.286, 0.085)),
        material="chrome_trim",
        name="right_hinge_mount",
    )
    stove_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.600),
        origin=_origin_x((0.0, -0.295, 0.080)),
        material="chrome_trim",
        name="oven_hinge_pin",
    )

    # Indicator light and trim on the control panel.
    stove_body.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=_origin_y((0.315, -0.324, 0.820)),
        material="indicator_amber",
        name="indicator_light",
    )

    # Four burner layout on the stovetop.
    for burner_name, x_pos, y_pos in BURNER_SPECS:
        stove_body.visual(
            Cylinder(radius=0.095, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.903)),
            material="chrome_trim",
            name=f"{burner_name}_drip_pan",
        )
        stove_body.visual(
            Cylinder(radius=0.055, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.912)),
            material="cast_iron",
            name=f"{burner_name}_burner_base",
        )
        stove_body.visual(
            Cylinder(radius=0.040, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.925)),
            material="cast_iron",
            name=f"{burner_name}_burner_cap",
        )
        stove_body.visual(
            Box((0.145, 0.014, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.936)),
            material="cast_iron",
            name=f"{burner_name}_grate_cross_x",
        )
        stove_body.visual(
            Box((0.014, 0.145, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.936)),
            material="cast_iron",
            name=f"{burner_name}_grate_cross_y",
        )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Box((0.500, 0.020, 0.470)),
        origin=Origin(xyz=(0.0, -0.010, 0.325)),
        material="cavity_dark",
        name="inner_liner",
    )
    oven_door.visual(
        Box((0.070, 0.040, 0.500)),
        origin=Origin(xyz=(-0.245, -0.020, 0.350)),
        material="enamel_cream",
        name="left_stile",
    )
    oven_door.visual(
        Box((0.070, 0.040, 0.500)),
        origin=Origin(xyz=(0.245, -0.020, 0.350)),
        material="enamel_cream",
        name="right_stile",
    )
    oven_door.visual(
        Box((0.560, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, -0.020, 0.560)),
        material="enamel_cream",
        name="top_rail",
    )
    oven_door.visual(
        Box((0.560, 0.046, 0.060)),
        origin=Origin(xyz=(0.0, -0.023, 0.120)),
        material="enamel_cream",
        name="bottom_rail",
    )
    oven_door.visual(
        Box((0.400, 0.028, 0.100)),
        origin=Origin(xyz=(0.0, -0.014, 0.205)),
        material="enamel_cream",
        name="lower_panel",
    )
    oven_door.visual(
        Box((0.360, 0.006, 0.210)),
        origin=Origin(xyz=(0.0, -0.018, 0.390)),
        material="glass_smoke",
        name="door_window",
    )
    oven_door.visual(
        Box((0.390, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.020, 0.503)),
        material="chrome_trim",
        name="window_trim_top",
    )
    oven_door.visual(
        Box((0.390, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.020, 0.277)),
        material="chrome_trim",
        name="window_trim_bottom",
    )
    oven_door.visual(
        Box((0.016, 0.008, 0.226)),
        origin=Origin(xyz=(-0.187, -0.020, 0.390)),
        material="chrome_trim",
        name="window_trim_left",
    )
    oven_door.visual(
        Box((0.016, 0.008, 0.226)),
        origin=Origin(xyz=(0.187, -0.020, 0.390)),
        material="chrome_trim",
        name="window_trim_right",
    )
    oven_door.visual(
        Cylinder(radius=0.010, length=0.360),
        origin=_origin_x((0.0, -0.090, 0.565)),
        material="chrome_trim",
        name="door_handle",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=_origin_y((-0.145, -0.055, 0.565)),
        material="chrome_trim",
        name="handle_post_left",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=_origin_y((0.145, -0.055, 0.565)),
        material="chrome_trim",
        name="handle_post_right",
    )
    oven_door.visual(
        Box((0.050, 0.018, 0.094)),
        origin=Origin(xyz=(-0.190, -0.020, 0.053)),
        material="enamel_cream",
        name="hinge_arm_left",
    )
    oven_door.visual(
        Box((0.060, 0.018, 0.094)),
        origin=Origin(xyz=(0.000, -0.020, 0.053)),
        material="enamel_cream",
        name="hinge_arm_center",
    )
    oven_door.visual(
        Box((0.050, 0.018, 0.094)),
        origin=Origin(xyz=(0.190, -0.020, 0.053)),
        material="enamel_cream",
        name="hinge_arm_right",
    )
    oven_door.visual(
        Box((0.090, 0.022, 0.020)),
        origin=Origin(xyz=(-0.190, -0.018, 0.008)),
        material="chrome_trim",
        name="hinge_sleeve_left",
    )
    oven_door.visual(
        Box((0.100, 0.022, 0.020)),
        origin=Origin(xyz=(0.000, -0.018, 0.008)),
        material="chrome_trim",
        name="hinge_sleeve_center",
    )
    oven_door.visual(
        Box((0.090, 0.022, 0.020)),
        origin=Origin(xyz=(0.190, -0.018, 0.008)),
        material="chrome_trim",
        name="hinge_sleeve_right",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((0.560, 0.100, 0.560)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.035, 0.300)),
    )

    model.articulation(
        "stove_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=stove_body,
        child=oven_door,
        origin=Origin(xyz=(0.0, -0.290, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    for knob_name, x_pos, z_pos, radius, depth in KNOB_SPECS:
        knob = _build_knob(model, knob_name, radius, depth)
        model.articulation(
            f"stove_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=stove_body,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.320, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    stove_body = object_model.get_part("stove_body")
    oven_door = object_model.get_part("oven_door")
    knob_parts = [object_model.get_part(name) for name, *_ in KNOB_SPECS]
    door_hinge = object_model.get_articulation("stove_to_oven_door")
    knob_joints = [object_model.get_articulation(f"stove_to_{name}") for name, *_ in KNOB_SPECS]

    hinge_pin = stove_body.get_visual("oven_hinge_pin")
    hinge_sleeves = [
        oven_door.get_visual("hinge_sleeve_left"),
        oven_door.get_visual("hinge_sleeve_center"),
        oven_door.get_visual("hinge_sleeve_right"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.check(
        "oven_door_axis_and_limits",
        door_hinge.joint_type == ArticulationType.REVOLUTE
        and door_hinge.axis == (1.0, 0.0, 0.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.2 <= door_hinge.motion_limits.upper <= 1.5,
        details="Oven door should hinge downward on a lower horizontal x-axis with a realistic opening range.",
    )
    for knob_name, *_ in KNOB_SPECS:
        joint = object_model.get_articulation(f"stove_to_{knob_name}")
        limits = joint.motion_limits
        ctx.check(
            f"{knob_name}_continuous_joint",
            joint.joint_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"{knob_name} should spin continuously about the front-to-back y-axis.",
        )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    for sleeve in hinge_sleeves:
        ctx.allow_overlap(
            stove_body,
            oven_door,
            elem_a=hinge_pin,
            elem_b=sleeve,
            reason="Oven door hinge sleeves wrap the hinge pin as a real mechanical joint.",
        )
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

    for knob in knob_parts:
        ctx.expect_contact(knob, stove_body, name=f"{knob.name}_mounted")

    for sleeve in hinge_sleeves:
        ctx.expect_contact(
            oven_door,
            stove_body,
            elem_a=sleeve,
            elem_b=hinge_pin,
            name=f"{sleeve.name}_contacts_hinge_pin",
        )

    closed_door_aabb = ctx.part_world_aabb(oven_door)
    door_window_aabb = ctx.part_element_world_aabb(oven_door, elem=oven_door.get_visual("door_window"))
    assert closed_door_aabb is not None
    assert door_window_aabb is not None
    ctx.check(
        "door_window_in_upper_half",
        door_window_aabb[0][2] > closed_door_aabb[0][2] + 0.20
        and door_window_aabb[1][2] < closed_door_aabb[1][2] - 0.08,
        details="The oven door window should sit in the upper portion of the door.",
    )

    backsplash_aabb = ctx.part_element_world_aabb(stove_body, elem=stove_body.get_visual("backsplash"))
    top_deck_aabb = ctx.part_element_world_aabb(stove_body, elem=stove_body.get_visual("top_deck"))
    assert backsplash_aabb is not None
    assert top_deck_aabb is not None
    ctx.check(
        "backsplash_is_shallow_and_rear",
        backsplash_aabb[1][2] > top_deck_aabb[1][2] + 0.08
        and backsplash_aabb[0][1] > 0.26
        and (backsplash_aabb[1][1] - backsplash_aabb[0][1]) < 0.04,
        details="The backsplash should be shallow, rear-mounted, and rise above the stovetop.",
    )

    burner_aabbs = {
        name: ctx.part_element_world_aabb(stove_body, elem=stove_body.get_visual(f"{name}_burner_cap"))
        for name, _, _ in BURNER_SPECS
    }
    for aabb in burner_aabbs.values():
        assert aabb is not None
    burner_centers = {
        name: (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][1] + aabb[1][1]) / 2.0,
        )
        for name, aabb in burner_aabbs.items()
    }
    ctx.check(
        "burners_form_four_burner_grid",
        burner_centers["front_left"][0] < -0.10
        and burner_centers["front_right"][0] > 0.10
        and burner_centers["rear_left"][1] > burner_centers["front_left"][1] + 0.20
        and burner_centers["rear_right"][1] > burner_centers["front_right"][1] + 0.20,
        details="Four burners should read as a left/right by front/rear grid.",
    )

    rotated_knob_pose = {
        knob_joints[0]: math.pi / 3.0,
        knob_joints[1]: -math.pi / 2.0,
        knob_joints[2]: math.pi,
        knob_joints[3]: -2.0 * math.pi / 3.0,
        knob_joints[4]: math.pi / 2.0,
    }
    with ctx.pose(rotated_knob_pose):
        for knob in knob_parts:
            ctx.expect_contact(knob, stove_body, name=f"{knob.name}_mounted_rotated")
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")

    door_limits = door_hinge.motion_limits
    assert door_limits is not None and door_limits.upper is not None
    with ctx.pose({door_hinge: door_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(oven_door)
        open_top_rail_aabb = ctx.part_element_world_aabb(oven_door, elem=oven_door.get_visual("top_rail"))
        assert open_door_aabb is not None
        assert open_top_rail_aabb is not None
        for sleeve in hinge_sleeves:
            ctx.expect_contact(
                oven_door,
                stove_body,
                elem_a=sleeve,
                elem_b=hinge_pin,
                contact_tol=0.002,
                name=f"{sleeve.name}_open_pose_contact",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
        ctx.check(
            "oven_door_swings_downward",
            open_top_rail_aabb[1][2] < closed_door_aabb[1][2] - 0.45
            and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.20,
            details="When opened, the oven door should drop and swing forward.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
