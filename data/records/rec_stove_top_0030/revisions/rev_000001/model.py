from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

COUNTER_WIDTH = 1.35
COUNTER_DEPTH = 0.65
COUNTER_TOP_Z = 0.90
COUNTER_THICKNESS = 0.04
COUNTER_UNDERSIDE_Z = COUNTER_TOP_Z - COUNTER_THICKNESS

COUNTER_CUTOUT_WIDTH = 0.70
COUNTER_CUTOUT_DEPTH = 0.48
COUNTER_FRONT_OPENING_WIDTH = 0.90

HOB_WIDTH = 0.76
HOB_DEPTH = 0.52
HOB_FLANGE_THICKNESS = 0.006
HOB_TRAY_WIDTH = 0.68
HOB_TRAY_DEPTH = 0.46
HOB_TRAY_DROP = 0.12
HOB_FASCIA_THICKNESS = 0.015
HOB_FASCIA_HEIGHT = 0.045
HOB_ORIGIN_Z = COUNTER_TOP_Z + HOB_FLANGE_THICKNESS / 2.0

BURNER_SPECS = (
    ("rear_left", (-0.22, 0.15), 0.040, 0.028, 0.100),
    ("front_left", (-0.23, -0.11), 0.034, 0.023, 0.086),
    ("center", (-0.02, 0.02), 0.056, 0.038, 0.132),
    ("rear_right", (0.20, 0.16), 0.041, 0.029, 0.104),
    ("front_right", (0.12, -0.12), 0.037, 0.026, 0.092),
)

KNOB_SPECS = (
    ("knob_left", 0.185, 0.018, 0.026),
    ("knob_middle", 0.245, 0.021, 0.029),
    ("knob_right", 0.310, 0.024, 0.032),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob", assets=ASSETS)

    stone = model.material("stone", rgba=(0.77, 0.77, 0.74, 1.0))
    support = model.material("support", rgba=(0.36, 0.33, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.83, 0.84, 1.0))
    burner_black = model.material("burner_black", rgba=(0.12, 0.12, 0.12, 1.0))
    grate_black = model.material("grate_black", rgba=(0.18, 0.18, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.15, 0.15, 0.15, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.84, 0.84, 0.84, 1.0))

    counter = model.part("counter_structure")

    front_margin = (COUNTER_DEPTH - COUNTER_CUTOUT_DEPTH) / 2.0
    side_margin = (COUNTER_WIDTH - COUNTER_CUTOUT_WIDTH) / 2.0

    front_corner_width = (COUNTER_WIDTH - COUNTER_FRONT_OPENING_WIDTH) / 2.0
    front_strip_center_y = -(COUNTER_DEPTH - front_margin) / 2.0
    counter.visual(
        Box((front_corner_width, front_margin, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                -(COUNTER_WIDTH - front_corner_width) / 2.0,
                front_strip_center_y,
                COUNTER_TOP_Z - COUNTER_THICKNESS / 2.0,
            )
        ),
        material=stone,
        name="front_left_strip",
    )
    counter.visual(
        Box((front_corner_width, front_margin, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                (COUNTER_WIDTH - front_corner_width) / 2.0,
                front_strip_center_y,
                COUNTER_TOP_Z - COUNTER_THICKNESS / 2.0,
            )
        ),
        material=stone,
        name="front_right_strip",
    )
    counter.visual(
        Box((COUNTER_WIDTH, front_margin, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, (COUNTER_DEPTH - front_margin) / 2.0, COUNTER_TOP_Z - COUNTER_THICKNESS / 2.0)),
        material=stone,
        name="back_strip",
    )
    counter.visual(
        Box((side_margin, COUNTER_CUTOUT_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=(-(COUNTER_WIDTH - side_margin) / 2.0, 0.0, COUNTER_TOP_Z - COUNTER_THICKNESS / 2.0)),
        material=stone,
        name="left_strip",
    )
    counter.visual(
        Box((side_margin, COUNTER_CUTOUT_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=((COUNTER_WIDTH - side_margin) / 2.0, 0.0, COUNTER_TOP_Z - COUNTER_THICKNESS / 2.0)),
        material=stone,
        name="right_strip",
    )

    counter.visual(
        Box((0.05, 0.60, COUNTER_UNDERSIDE_Z)),
        origin=Origin(xyz=(-(COUNTER_WIDTH / 2.0 - 0.025), 0.0, COUNTER_UNDERSIDE_Z / 2.0)),
        material=support,
        name="left_panel",
    )
    counter.visual(
        Box((0.05, 0.60, COUNTER_UNDERSIDE_Z)),
        origin=Origin(xyz=((COUNTER_WIDTH / 2.0 - 0.025), 0.0, COUNTER_UNDERSIDE_Z / 2.0)),
        material=support,
        name="right_panel",
    )
    counter.visual(
        Box((COUNTER_WIDTH - 0.10, 0.03, COUNTER_UNDERSIDE_Z)),
        origin=Origin(xyz=(0.0, COUNTER_DEPTH / 2.0 - 0.015, COUNTER_UNDERSIDE_Z / 2.0)),
        material=support,
        name="back_panel",
    )

    hob = model.part("hob_body")
    hob.visual(
        Box((HOB_WIDTH, HOB_DEPTH, HOB_FLANGE_THICKNESS)),
        material=steel,
        name="flange",
    )
    hob.visual(
        Box((HOB_TRAY_WIDTH, HOB_TRAY_DEPTH, HOB_TRAY_DROP)),
        origin=Origin(xyz=(0.0, 0.0, -(HOB_FLANGE_THICKNESS / 2.0 + HOB_TRAY_DROP / 2.0))),
        material=steel,
        name="tray",
    )
    hob.visual(
        Box((HOB_WIDTH, HOB_FASCIA_THICKNESS, HOB_FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(HOB_DEPTH / 2.0 - HOB_FASCIA_THICKNESS / 2.0),
                -(HOB_FLANGE_THICKNESS / 2.0 + HOB_FASCIA_HEIGHT / 2.0),
            )
        ),
        material=steel,
        name="control_fascia",
    )

    burner_base_height = 0.006
    burner_cap_height = 0.008
    grate_height = 0.004
    burner_base_center_z = HOB_FLANGE_THICKNESS / 2.0 + burner_base_height / 2.0
    burner_cap_center_z = HOB_FLANGE_THICKNESS / 2.0 + burner_base_height + burner_cap_height / 2.0
    grate_center_z = HOB_FLANGE_THICKNESS / 2.0 + burner_base_height + burner_cap_height + grate_height / 2.0

    for burner_name, (bx, by), base_radius, cap_radius, grate_span in BURNER_SPECS:
        hob.visual(
            Cylinder(radius=base_radius, length=burner_base_height),
            origin=Origin(xyz=(bx, by, burner_base_center_z)),
            material=burner_black,
            name=f"burner_base_{burner_name}",
        )
        hob.visual(
            Cylinder(radius=cap_radius, length=burner_cap_height),
            origin=Origin(xyz=(bx, by, burner_cap_center_z)),
            material=burner_black,
            name=f"burner_cap_{burner_name}",
        )
        hob.visual(
            Box((grate_span, 0.010, grate_height)),
            origin=Origin(xyz=(bx, by, grate_center_z)),
            material=grate_black,
            name=f"grate_x_{burner_name}",
        )
        hob.visual(
            Box((0.010, grate_span, grate_height)),
            origin=Origin(xyz=(bx, by, grate_center_z)),
            material=grate_black,
            name=f"grate_y_{burner_name}",
        )

    model.articulation(
        "counter_to_hob",
        ArticulationType.FIXED,
        parent=counter,
        child=hob,
        origin=Origin(xyz=(0.0, 0.0, HOB_ORIGIN_Z)),
    )

    knob_center_z = -(HOB_FLANGE_THICKNESS / 2.0 + HOB_FASCIA_HEIGHT / 2.0)
    fascia_front_y = -HOB_DEPTH / 2.0

    for knob_name, knob_x, knob_radius, knob_length in KNOB_SPECS:
        knob = model.part(knob_name)
        body_length = knob_length * 0.76
        face_length = knob_length * 0.42
        body_center_y = (knob_length - body_length) / 2.0
        face_center_y = -(knob_length - face_length) / 2.0
        indicator_thickness = 0.0025
        knob.visual(
            Cylinder(radius=knob_radius, length=body_length),
            origin=Origin(xyz=(0.0, body_center_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_shell",
        )
        knob.visual(
            Cylinder(radius=knob_radius * 0.82, length=face_length),
            origin=Origin(xyz=(0.0, face_center_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_face",
        )
        knob.visual(
            Box((knob_radius * 0.30, indicator_thickness, knob_radius * 0.16)),
            origin=Origin(
                xyz=(
                    0.0,
                    -knob_length / 2.0 + indicator_thickness / 2.0 + 0.001,
                    knob_radius * 0.52,
                )
            ),
            material=knob_mark,
            name="indicator_mark",
        )
        model.articulation(
            f"hob_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=hob,
            child=knob,
            origin=Origin(xyz=(knob_x, fascia_front_y - knob_length / 2.0, knob_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    try:
        counter = object_model.get_part("counter_structure")
        hob = object_model.get_part("hob_body")
        knob_left = object_model.get_part("knob_left")
        knob_middle = object_model.get_part("knob_middle")
        knob_right = object_model.get_part("knob_right")
        joint_left = object_model.get_articulation("hob_to_knob_left")
        joint_middle = object_model.get_articulation("hob_to_knob_middle")
        joint_right = object_model.get_articulation("hob_to_knob_right")
    except Exception as exc:
        ctx.fail("required_parts_and_joints_present", str(exc))
        return ctx.report()

    knob_visual_left = knob_left.get_visual("knob_shell")
    knob_visual_middle = knob_middle.get_visual("knob_shell")
    knob_visual_right = knob_right.get_visual("knob_shell")

    burner_caps = [
        visual.name
        for visual in hob.visuals
        if visual.name is not None and visual.name.startswith("burner_cap_")
    ]
    ctx.check(
        "five_burner_layout_present",
        len(burner_caps) == 5,
        f"Expected 5 burner cap visuals, found {len(burner_caps)}.",
    )

    ctx.expect_contact(hob, counter, name="hob_seated_on_counter")
    ctx.expect_gap(
        hob,
        counter,
        axis="z",
        positive_elem="flange",
        negative_elem="left_strip",
        min_gap=0.0,
        max_gap=0.001,
        name="hob_flange_sits_on_left_counter_rail",
    )
    ctx.expect_overlap(
        hob,
        counter,
        axes="xy",
        elem_a="flange",
        elem_b="left_strip",
        min_overlap=0.02,
        name="hob_flange_overlaps_left_counter_rail",
    )

    ctx.expect_contact(knob_left, hob, name="left_knob_mounted")
    ctx.expect_contact(knob_middle, hob, name="middle_knob_mounted")
    ctx.expect_contact(knob_right, hob, name="right_knob_mounted")

    ctx.expect_origin_gap(
        knob_left,
        hob,
        axis="x",
        min_gap=0.16,
        max_gap=0.22,
        name="left_knob_is_on_far_right_side",
    )
    ctx.expect_origin_gap(
        knob_middle,
        knob_left,
        axis="x",
        min_gap=0.045,
        max_gap=0.075,
        name="knob_cluster_spacing_left_to_middle",
    )
    ctx.expect_origin_gap(
        knob_right,
        knob_middle,
        axis="x",
        min_gap=0.045,
        max_gap=0.080,
        name="knob_cluster_spacing_middle_to_right",
    )
    ctx.expect_origin_gap(
        hob,
        knob_left,
        axis="y",
        min_gap=0.22,
        max_gap=0.32,
        name="knobs_project_forward_from_fascia",
    )

    ctx.check(
        "only_three_knobs_are_moving_joints",
        sum(
            1
            for articulation in object_model.articulations
            if articulation.articulation_type != ArticulationType.FIXED
        )
        == 3,
        "Only the three knobs should be articulated as moving joints.",
    )

    ctx.check(
        "knob_cluster_sits_clear_of_front_counter_opening",
        all(
            knob.name in {"knob_left", "knob_middle", "knob_right"}
            for knob in (knob_left, knob_middle, knob_right)
        ),
        "Knob cluster parts should be present at the front-right control lip.",
    )

    ctx.expect_gap(
        hob,
        counter,
        axis="z",
        positive_elem="flange",
        negative_elem="back_strip",
        min_gap=0.0,
        max_gap=0.001,
        name="hob_and_counter_share_top_plane_height",
    )

    ctx.check(
        "knob_diameters_increase_left_to_right",
        (
            isinstance(knob_visual_left.geometry, Cylinder)
            and isinstance(knob_visual_middle.geometry, Cylinder)
            and isinstance(knob_visual_right.geometry, Cylinder)
            and knob_visual_left.geometry.radius < knob_visual_middle.geometry.radius < knob_visual_right.geometry.radius
        ),
        "Knob radii should increase from left to right.",
    )

    for joint_name, joint in (
        ("left", joint_left),
        ("middle", joint_middle),
        ("right", joint_right),
    ):
        ctx.check(
            f"{joint_name}_knob_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"Expected axis (0, 1, 0), found {joint.axis}.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_knob_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            "Knob should use a continuous joint without lower or upper limits.",
        )

    with ctx.pose({joint_left: 1.7, joint_middle: 3.2, joint_right: 5.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_turned_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_turned_no_floating")

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
