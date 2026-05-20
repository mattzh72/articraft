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

COUNTER_WIDTH = 0.92
COUNTER_DEPTH = 0.64
COUNTER_THICKNESS = 0.04
COUNTER_TOP_Z = 0.90
LEG_SIZE = 0.04

HOB_WIDTH = 0.76
HOB_DEPTH = 0.52
HOB_PANEL_THICKNESS = 0.005
HOB_CORNER_RADIUS = 0.015
HOB_PANEL_CENTER_Z = COUNTER_TOP_Z - (HOB_PANEL_THICKNESS * 0.5)
HOB_PANEL_BOTTOM_Z = COUNTER_TOP_Z - HOB_PANEL_THICKNESS

BURNER_SPECS = (
    ("burner_left_rear", (-0.23, 0.11), 0.048, 0.135),
    ("burner_left_front", (-0.22, -0.09), 0.045, 0.128),
    ("burner_center", (0.00, 0.015), 0.060, 0.182),
    ("burner_right_rear", (0.24, 0.11), 0.045, 0.128),
    ("burner_right_front", (0.23, -0.09), 0.048, 0.135),
)

KNOB_SPECS = (
    ("knob_left_large", "hob_to_knob_left_large", -0.30, 0.028, 0.036),
    ("knob_center_left", "hob_to_knob_center_left", -0.06, 0.021, 0.032),
    ("knob_center_right", "hob_to_knob_center_right", 0.04, 0.021, 0.032),
    ("knob_right_inner", "hob_to_knob_right_inner", 0.23, 0.021, 0.032),
    ("knob_right_outer", "hob_to_knob_right_outer", 0.32, 0.021, 0.032),
)

def _add_burner(
    hob_part,
    *,
    prefix: str,
    center_xy: tuple[float, float],
    dish_radius: float,
    grate_span: float,
    burner_material,
    grate_material,
) -> None:
    x, y = center_xy
    panel_top_z = COUNTER_TOP_Z

    hob_part.visual(
        Cylinder(radius=dish_radius, length=0.006),
        origin=Origin(xyz=(x, y, panel_top_z + 0.003)),
        material=burner_material,
        name=f"{prefix}_dish",
    )
    hob_part.visual(
        Cylinder(radius=dish_radius * 0.42, length=0.010),
        origin=Origin(xyz=(x, y, panel_top_z + 0.011)),
        material=burner_material,
        name=f"{prefix}_cap",
    )

    grate_bar_height = 0.006
    grate_bar_width = 0.012
    grate_bar_z = panel_top_z + 0.021
    foot_size = 0.010
    foot_height = 0.018
    foot_z = panel_top_z + (foot_height * 0.5)
    foot_offset = (grate_span * 0.5) - 0.008

    hob_part.visual(
        Box((grate_span, grate_bar_width, grate_bar_height)),
        origin=Origin(xyz=(x, y, grate_bar_z)),
        material=grate_material,
        name=f"{prefix}_grate_cross_x",
    )
    hob_part.visual(
        Box((grate_bar_width, grate_span, grate_bar_height)),
        origin=Origin(xyz=(x, y, grate_bar_z)),
        material=grate_material,
        name=f"{prefix}_grate_cross_y",
    )

    for suffix, foot_x, foot_y in (
        ("foot_left", x - foot_offset, y),
        ("foot_right", x + foot_offset, y),
        ("foot_front", x, y - foot_offset),
        ("foot_back", x, y + foot_offset),
    ):
        hob_part.visual(
            Box((foot_size, foot_size, foot_height)),
            origin=Origin(xyz=(foot_x, foot_y, foot_z)),
            material=grate_material,
            name=f"{prefix}_{suffix}",
        )


def _add_knob(
    model: ArticulatedObject,
    hob_part,
    *,
    part_name: str,
    joint_name: str,
    x: float,
    radius: float,
    length: float,
    knob_material,
    mark_material,
):
    knob_part = model.part(part_name)
    knob_part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_shell",
    )
    knob_part.visual(
        Box((radius * 0.28, length * 0.62, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0, radius * 0.88)),
        material=mark_material,
        name="pointer_mark",
    )
    knob_part.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=length),
        mass=0.24 if radius > 0.025 else 0.14,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    knob_y = -(HOB_DEPTH * 0.5) + 0.004 + (length * 0.5)
    control_fascia_top_z = 0.895
    knob_z = control_fascia_top_z + radius

    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=hob_part,
        child=knob_part,
        origin=Origin(xyz=(x, knob_y, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6 if radius > 0.025 else 0.35,
            velocity=8.0,
        ),
    )
    return knob_part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_shelf_gas_hob", assets=ASSETS)

    frame_black = model.material("frame_black", rgba=(0.18, 0.18, 0.19, 1.0))
    stone = model.material("stone", rgba=(0.80, 0.80, 0.78, 1.0))
    shelf_wood = model.material("shelf_wood", rgba=(0.58, 0.43, 0.28, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.71, 0.72, 1.0))
    burner_black = model.material("burner_black", rgba=(0.18, 0.18, 0.20, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.78, 0.78, 0.79, 1.0))

    counter_frame = model.part("counter_frame")

    leg_x = (COUNTER_WIDTH * 0.5) - (LEG_SIZE * 0.5)
    leg_y = (COUNTER_DEPTH * 0.5) - (LEG_SIZE * 0.5)
    leg_height = COUNTER_TOP_Z - COUNTER_THICKNESS
    leg_center_z = leg_height * 0.5
    for sx in (-leg_x, leg_x):
        for sy in (-leg_y, leg_y):
            counter_frame.visual(
                Box((LEG_SIZE, LEG_SIZE, leg_height)),
                origin=Origin(xyz=(sx, sy, leg_center_z)),
                material=frame_black,
            )

    top_rail_z = leg_height - (LEG_SIZE * 0.5)
    counter_frame.visual(
        Box((COUNTER_WIDTH - (LEG_SIZE * 2.0), LEG_SIZE, LEG_SIZE)),
        origin=Origin(xyz=(0.0, leg_y, top_rail_z)),
        material=frame_black,
        name="top_back_rail",
    )
    counter_frame.visual(
        Box((LEG_SIZE, COUNTER_DEPTH - (LEG_SIZE * 2.0), LEG_SIZE)),
        origin=Origin(xyz=(-leg_x, 0.0, top_rail_z)),
        material=frame_black,
        name="top_left_rail",
    )
    counter_frame.visual(
        Box((LEG_SIZE, COUNTER_DEPTH - (LEG_SIZE * 2.0), LEG_SIZE)),
        origin=Origin(xyz=(leg_x, 0.0, top_rail_z)),
        material=frame_black,
        name="top_right_rail",
    )
    counter_frame.visual(
        Box((COUNTER_WIDTH - (LEG_SIZE * 2.0), LEG_SIZE, LEG_SIZE)),
        origin=Origin(xyz=(0.0, -0.19, top_rail_z)),
        material=frame_black,
        name="top_mid_front_support",
    )
    counter_frame.visual(
        Box((COUNTER_WIDTH - (LEG_SIZE * 2.0), LEG_SIZE, 0.12)),
        origin=Origin(xyz=(0.0, -leg_y, 0.72)),
        material=frame_black,
        name="front_apron",
    )

    lower_rail_z = 0.16
    lower_span_x = COUNTER_WIDTH - (LEG_SIZE * 2.0)
    lower_span_y = COUNTER_DEPTH - (LEG_SIZE * 2.0)
    counter_frame.visual(
        Box((lower_span_x, LEG_SIZE, LEG_SIZE)),
        origin=Origin(xyz=(0.0, -leg_y, lower_rail_z)),
        material=frame_black,
        name="lower_front_rail",
    )
    counter_frame.visual(
        Box((lower_span_x, LEG_SIZE, LEG_SIZE)),
        origin=Origin(xyz=(0.0, leg_y, lower_rail_z)),
        material=frame_black,
        name="lower_back_rail",
    )
    counter_frame.visual(
        Box((LEG_SIZE, lower_span_y, LEG_SIZE)),
        origin=Origin(xyz=(-leg_x, 0.0, lower_rail_z)),
        material=frame_black,
        name="lower_left_rail",
    )
    counter_frame.visual(
        Box((LEG_SIZE, lower_span_y, LEG_SIZE)),
        origin=Origin(xyz=(leg_x, 0.0, lower_rail_z)),
        material=frame_black,
        name="lower_right_rail",
    )
    counter_frame.visual(
        Box((0.84, 0.60, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=shelf_wood,
        name="lower_shelf",
    )

    countertop_center_z = COUNTER_TOP_Z - (COUNTER_THICKNESS * 0.5)
    counter_frame.visual(
        Box((COUNTER_WIDTH, 0.06, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.29, countertop_center_z)),
        material=stone,
        name="countertop_front",
    )
    counter_frame.visual(
        Box((COUNTER_WIDTH, 0.06, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.29, countertop_center_z)),
        material=stone,
        name="countertop_back",
    )
    counter_frame.visual(
        Box((0.08, HOB_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=(-0.42, 0.0, countertop_center_z)),
        material=stone,
        name="countertop_left",
    )
    counter_frame.visual(
        Box((0.08, HOB_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.42, 0.0, countertop_center_z)),
        material=stone,
        name="countertop_right",
    )
    counter_frame.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_TOP_Z)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_TOP_Z * 0.5)),
    )

    hob = model.part("hob")
    hob.visual(
        Box((HOB_WIDTH, HOB_DEPTH, HOB_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, HOB_PANEL_CENTER_Z)),
        material=stainless,
        name="hob_panel",
    )
    hob.visual(
        Box((HOB_WIDTH, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, -0.242, 0.886)),
        material=stainless,
        name="control_fascia",
    )

    for burner_name, center_xy, dish_radius, grate_span in BURNER_SPECS:
        _add_burner(
            hob,
            prefix=burner_name,
            center_xy=center_xy,
            dish_radius=dish_radius,
            grate_span=grate_span,
            burner_material=burner_black,
            grate_material=cast_iron,
        )

    hob.inertial = Inertial.from_geometry(
        Box((HOB_WIDTH, HOB_DEPTH, 0.070)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
    )

    model.articulation(
        "frame_to_hob",
        ArticulationType.FIXED,
        parent=counter_frame,
        child=hob,
        origin=Origin(),
    )

    for part_name, joint_name, x, radius, length in KNOB_SPECS:
        _add_knob(
            model,
            hob,
            part_name=part_name,
            joint_name=joint_name,
            x=x,
            radius=radius,
            length=length,
            knob_material=knob_black,
            mark_material=knob_mark,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    counter_frame = object_model.get_part("counter_frame")
    hob = object_model.get_part("hob")

    knob_parts = {
        part_name: object_model.get_part(part_name)
        for part_name, _, _, _, _ in KNOB_SPECS
    }
    knob_joints = {
        joint_name: object_model.get_articulation(joint_name)
        for _, joint_name, _, _, _ in KNOB_SPECS
    }
    hob_mount = object_model.get_articulation("frame_to_hob")

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
    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="sampled_knob_clearance")

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

    ctx.check(
        "single_root_part",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "counter_frame",
        f"Unexpected roots: {[part.name for part in object_model.root_parts()]}",
    )
    ctx.check(
        "hob_mount_is_fixed",
        hob_mount.articulation_type == ArticulationType.FIXED,
        f"Expected fixed hob mount, got {hob_mount.articulation_type}",
    )

    for _, joint_name, _, _, _ in KNOB_SPECS:
        joint = knob_joints[joint_name]
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"Expected continuous joint, got {joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name}_front_to_back_axis",
            all(abs(a - b) < 1e-9 for a, b in zip(joint.axis, (0.0, 1.0, 0.0))),
            f"Expected axis (0, 1, 0), got {joint.axis}",
        )
        ctx.check(
            f"{joint_name}_unbounded",
            limits is not None and limits.lower is None and limits.upper is None,
            f"Continuous knob limits should be unbounded, got {limits}",
        )

    ctx.expect_contact(
        hob,
        counter_frame,
        elem_a="hob_panel",
        elem_b="countertop_front",
        contact_tol=0.0005,
        name="hob_panel_touches_front_counter_edge",
    )
    ctx.expect_gap(
        hob,
        counter_frame,
        axis="x",
        positive_elem="hob_panel",
        negative_elem="countertop_left",
        max_gap=0.0005,
        max_penetration=0.0,
        name="hob_panel_touches_left_counter_edge",
    )
    ctx.expect_gap(
        counter_frame,
        hob,
        axis="x",
        positive_elem="countertop_right",
        negative_elem="hob_panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="hob_panel_touches_right_counter_edge",
    )

    for part_name, _, _, _, _ in KNOB_SPECS:
        ctx.expect_contact(
            knob_parts[part_name],
            hob,
            elem_a="knob_shell",
            elem_b="control_fascia",
            contact_tol=0.0005,
            name=f"{part_name}_mounted_to_fascia",
        )

    ctx.expect_origin_gap(
        knob_parts["knob_center_left"],
        knob_parts["knob_left_large"],
        axis="x",
        min_gap=0.18,
        max_gap=0.30,
        name="large_left_knob_is_far_left",
    )
    ctx.expect_origin_gap(
        knob_parts["knob_center_right"],
        knob_parts["knob_center_left"],
        axis="x",
        min_gap=0.08,
        max_gap=0.14,
        name="center_pair_grouped_together",
    )
    ctx.expect_origin_gap(
        knob_parts["knob_right_inner"],
        knob_parts["knob_center_right"],
        axis="x",
        min_gap=0.14,
        max_gap=0.24,
        name="right_pair_separated_from_center_group",
    )
    ctx.expect_origin_gap(
        knob_parts["knob_right_outer"],
        knob_parts["knob_right_inner"],
        axis="x",
        min_gap=0.07,
        max_gap=0.12,
        name="right_pair_grouped_together",
    )

    left_knob_radius = knob_parts["knob_left_large"].get_visual("knob_shell").geometry.radius
    center_knob_radius = knob_parts["knob_center_left"].get_visual("knob_shell").geometry.radius
    ctx.check(
        "left_knob_is_largest",
        left_knob_radius > center_knob_radius,
        f"Expected left knob radius > center knob radius, got {left_knob_radius} <= {center_knob_radius}",
    )

    missing_burners: list[str] = []
    for burner_name, _, _, _ in BURNER_SPECS:
        try:
            hob.get_visual(f"{burner_name}_dish")
        except Exception:
            missing_burners.append(burner_name)
    ctx.check(
        "five_burners_present",
        not missing_burners,
        f"Missing burner visuals for: {missing_burners}",
    )

    frame_aabb = ctx.part_world_aabb(counter_frame)
    if frame_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "counter_height_realistic",
            0.88 <= frame_height <= 0.93,
            f"Expected overall height near kitchen counter height, got {frame_height:.4f} m",
        )
        ctx.check(
            "counter_width_realistic",
            0.88 <= frame_width <= 0.96,
            f"Expected width near a 90 cm hob station, got {frame_width:.4f} m",
        )

    hob_aabb = ctx.part_world_aabb(hob)
    if hob_aabb is not None:
        hob_height = hob_aabb[1][2] - hob_aabb[0][2]
        ctx.check(
            "hob_remains_flat_without_stove_body",
            hob_height <= 0.08,
            f"Expected shallow hob assembly without enclosed oven body, got {hob_height:.4f} m tall",
        )

    with ctx.pose(
        {
            knob_joints["hob_to_knob_left_large"]: math.pi / 2.0,
            knob_joints["hob_to_knob_center_left"]: math.pi,
            knob_joints["hob_to_knob_center_right"]: -math.pi / 2.0,
            knob_joints["hob_to_knob_right_inner"]: 2.1,
            knob_joints["hob_to_knob_right_outer"]: -1.4,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="turned_knobs_no_floating")
        for part_name, _, _, _, _ in KNOB_SPECS:
            ctx.expect_contact(
                knob_parts[part_name],
                hob,
                elem_a="knob_shell",
                elem_b="control_fascia",
                contact_tol=0.0005,
                name=f"{part_name}_stays_mounted_when_turned",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
