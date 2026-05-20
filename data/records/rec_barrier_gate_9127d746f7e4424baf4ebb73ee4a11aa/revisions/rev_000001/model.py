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
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_lift_barrier")

    panel_width = 3.10
    panel_height = 1.90
    panel_thickness = 0.05
    panel_travel = 1.70
    frame_tube = 0.06
    cable_radius = 0.006
    pulley_radius = 0.09

    sill_height = 0.08
    column_height = 4.00
    header_bottom_z = sill_height + column_height
    panel_center_z = sill_height + panel_height / 2.0

    dark_frame = model.material("powder_coat_frame", rgba=(0.22, 0.23, 0.24, 1.0))
    bright_steel = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    cable_black = model.material("cable_black", rgba=(0.12, 0.12, 0.13, 1.0))
    weight_finish = model.material("counterweight_gray", rgba=(0.36, 0.37, 0.39, 1.0))

    threshold = model.part("threshold")
    threshold.visual(
        Box((3.46, 0.16, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, sill_height / 2.0)),
        material=dark_frame,
        name="sill",
    )
    threshold.visual(
        Box((0.30, 0.22, 0.02)),
        origin=Origin(xyz=(-1.58, 0.0, 0.01)),
        material=dark_frame,
        name="left_base_plate",
    )
    threshold.visual(
        Box((0.30, 0.22, 0.02)),
        origin=Origin(xyz=(1.58, 0.0, 0.01)),
        material=dark_frame,
        name="right_base_plate",
    )

    left_guide = model.part("left_guide")
    left_guide.visual(
        Box((0.025, 0.14, column_height)),
        origin=Origin(xyz=(-0.0575, 0.0, column_height / 2.0)),
        material=dark_frame,
        name="web",
    )
    left_guide.visual(
        Box((0.14, 0.02, column_height)),
        origin=Origin(xyz=(0.0, 0.06, column_height / 2.0)),
        material=dark_frame,
        name="front_flange",
    )
    left_guide.visual(
        Box((0.14, 0.02, column_height)),
        origin=Origin(xyz=(0.0, -0.06, column_height / 2.0)),
        material=dark_frame,
        name="rear_flange",
    )
    left_guide.visual(
        Box((0.14, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, column_height - 0.04)),
        material=dark_frame,
        name="top_cap",
    )

    right_guide = model.part("right_guide")
    right_guide.visual(
        Box((0.025, 0.14, column_height)),
        origin=Origin(xyz=(0.0575, 0.0, column_height / 2.0)),
        material=dark_frame,
        name="web",
    )
    right_guide.visual(
        Box((0.14, 0.02, column_height)),
        origin=Origin(xyz=(0.0, 0.06, column_height / 2.0)),
        material=dark_frame,
        name="front_flange",
    )
    right_guide.visual(
        Box((0.14, 0.02, column_height)),
        origin=Origin(xyz=(0.0, -0.06, column_height / 2.0)),
        material=dark_frame,
        name="rear_flange",
    )
    right_guide.visual(
        Box((0.14, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, column_height - 0.04)),
        material=dark_frame,
        name="top_cap",
    )

    header = model.part("header")
    header.visual(
        Box((3.50, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_frame,
        name="crossbeam",
    )
    header.visual(
        Box((0.18, 0.34, 0.16)),
        origin=Origin(xyz=(1.68, -0.17, 0.08)),
        material=dark_frame,
        name="rear_transfer_arm",
    )

    rear_column = model.part("rear_column")
    rear_column.visual(
        Box((0.18, 0.02, column_height)),
        origin=Origin(xyz=(0.0, -0.06, column_height / 2.0)),
        material=dark_frame,
        name="back_plate",
    )
    rear_column.visual(
        Box((0.03, 0.14, column_height)),
        origin=Origin(xyz=(-0.075, 0.0, column_height / 2.0)),
        material=dark_frame,
        name="left_rail",
    )
    rear_column.visual(
        Box((0.03, 0.14, column_height)),
        origin=Origin(xyz=(0.075, 0.0, column_height / 2.0)),
        material=dark_frame,
        name="right_rail",
    )
    rear_column.visual(
        Box((0.18, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, -0.03, column_height - 0.04)),
        material=dark_frame,
        name="top_cap",
    )
    rear_column.visual(
        Box((0.18, 0.20, 0.10)),
        origin=Origin(xyz=(0.0, -0.03, 0.05)),
        material=dark_frame,
        name="bottom_cap",
    )

    pulley_head = model.part("pulley_head")
    pulley_head.visual(
        Box((0.16, 0.12, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=dark_frame,
        name="mount_block",
    )
    pulley_head.visual(
        Cylinder(radius=pulley_radius, length=0.05),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="wheel",
    )
    pulley_head.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="axle",
    )
    pulley_head.visual(
        Box((0.02, 0.08, 0.18)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=dark_frame,
        name="left_cheek",
    )
    pulley_head.visual(
        Box((0.02, 0.08, 0.18)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=dark_frame,
        name="right_cheek",
    )
    cable_points = [(-0.08, 0.07, -1.91), (-0.08, 0.07, 0.0)]
    for step in range(13):
        theta = math.pi * step / 12.0
        cable_points.append((0.0, pulley_radius * math.cos(theta), pulley_radius * math.sin(theta)))
    cable_points.extend([(0.006, -0.06, -0.08), (0.006, -0.06, -0.42)])
    pulley_head.visual(
        mesh_from_geometry(
            wire_from_points(
                cable_points,
                radius=cable_radius,
                radial_segments=14,
                closed_path=False,
                cap_ends=True,
                corner_mode="miter",
            ),
            "pulley_cable",
        ),
        material=cable_black,
        name="cable",
    )

    panel = model.part("panel")
    panel.meta["role"] = "lift_gate_panel"
    panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness, panel_height)),
        mass=95.0,
        origin=Origin(),
    )
    rail_z = panel_height / 2.0 - frame_tube / 2.0
    stile_x = panel_width / 2.0 - frame_tube / 2.0
    inner_width = panel_width - 2.0 * frame_tube
    inner_height = panel_height - 2.0 * frame_tube
    panel.visual(
        Box((panel_width, panel_thickness, frame_tube)),
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=bright_steel,
        name="top_rail",
    )
    panel.visual(
        Box((panel_width, panel_thickness, frame_tube)),
        origin=Origin(xyz=(0.0, 0.0, -rail_z)),
        material=bright_steel,
        name="bottom_rail",
    )
    panel.visual(
        Box((frame_tube, panel_thickness, panel_height)),
        origin=Origin(xyz=(-stile_x, 0.0, 0.0)),
        material=bright_steel,
        name="left_stile",
    )
    panel.visual(
        Box((frame_tube, panel_thickness, panel_height)),
        origin=Origin(xyz=(stile_x, 0.0, 0.0)),
        material=bright_steel,
        name="right_stile",
    )
    panel.visual(
        Box((0.04, 0.08, 0.25)),
        origin=Origin(xyz=(1.50, -0.04, 0.80)),
        material=bright_steel,
        name="cable_bracket",
    )
    vertical_bar_positions = [
        -1.30,
        -1.10,
        -0.90,
        -0.70,
        -0.50,
        -0.30,
        -0.10,
        0.10,
        0.30,
        0.50,
        0.70,
        0.90,
        1.10,
        1.30,
    ]
    for index, x in enumerate(vertical_bar_positions, start=1):
        panel.visual(
            Cylinder(radius=0.006, length=inner_height),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=bright_steel,
            name=f"mesh_vertical_{index}",
        )
    horizontal_bar_positions = [-0.70, -0.48, -0.26, -0.04, 0.18, 0.40, 0.62]
    for index, z in enumerate(horizontal_bar_positions, start=1):
        panel.visual(
            Cylinder(radius=0.006, length=inner_width),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bright_steel,
            name=f"mesh_horizontal_{index}",
        )

    counterweight = model.part("counterweight")
    counterweight.meta["role"] = "balancing_mass"
    counterweight_size = (0.11, 0.09, 0.95)
    counterweight.inertial = Inertial.from_geometry(
        Box(counterweight_size),
        mass=85.0,
        origin=Origin(),
    )
    counterweight.visual(
        Box(counterweight_size),
        origin=Origin(),
        material=weight_finish,
        name="mass_block",
    )
    counterweight.visual(
        Box((0.04, 0.02, 0.08)),
        origin=Origin(xyz=(-0.03, 0.04, 0.435)),
        material=bright_steel,
        name="hanger_block",
    )
    for side_name, x in (("left", -0.055), ("right", 0.055)):
        for level_name, z in (("lower", -0.28), ("upper", 0.28)):
            counterweight.visual(
                Box((0.01, 0.04, 0.16)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=bright_steel,
                name=f"{side_name}_{level_name}_guide_shoe",
            )

    model.articulation(
        "threshold_to_left_guide",
        ArticulationType.FIXED,
        parent=threshold,
        child=left_guide,
        origin=Origin(xyz=(-1.68, 0.0, sill_height)),
    )
    model.articulation(
        "threshold_to_right_guide",
        ArticulationType.FIXED,
        parent=threshold,
        child=right_guide,
        origin=Origin(xyz=(1.68, 0.0, sill_height)),
    )
    model.articulation(
        "right_guide_to_header",
        ArticulationType.FIXED,
        parent=right_guide,
        child=header,
        origin=Origin(xyz=(-1.68, 0.0, column_height)),
    )
    model.articulation(
        "header_to_rear_column",
        ArticulationType.FIXED,
        parent=header,
        child=rear_column,
        origin=Origin(xyz=(1.68, -0.34, -column_height)),
    )
    model.articulation(
        "header_to_pulley_head",
        ArticulationType.FIXED,
        parent=header,
        child=pulley_head,
        origin=Origin(xyz=(1.68, -0.17, -0.26)),
    )

    panel_slide = model.articulation(
        "threshold_to_panel",
        ArticulationType.PRISMATIC,
        parent=threshold,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, panel_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35, lower=0.0, upper=panel_travel),
    )
    counterweight_slide = model.articulation(
        "threshold_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=threshold,
        child=counterweight,
        origin=Origin(xyz=(1.68, -0.325, 2.85)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35, lower=0.0, upper=panel_travel),
    )

    threshold.meta["panel_joint"] = panel_slide.name
    threshold.meta["counterweight_joint"] = counterweight_slide.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    threshold = object_model.get_part("threshold")
    header = object_model.get_part("header")
    rear_column = object_model.get_part("rear_column")
    panel = object_model.get_part("panel")
    counterweight = object_model.get_part("counterweight")
    panel_slide = object_model.get_articulation("threshold_to_panel")
    counterweight_slide = object_model.get_articulation("threshold_to_counterweight")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_contact(
        panel,
        threshold,
        elem_a="bottom_rail",
        elem_b="sill",
        name="panel rests on the sill when closed",
    )
    ctx.expect_within(
        counterweight,
        rear_column,
        axes="xy",
        inner_elem="mass_block",
        margin=0.0,
        name="counterweight block stays inside the rear guide column",
    )

    panel_rest = ctx.part_world_position(panel)
    counterweight_rest = ctx.part_world_position(counterweight)
    with ctx.pose({panel_slide: 1.40, counterweight_slide: 1.40}):
        panel_raised = ctx.part_world_position(panel)
        counterweight_lowered = ctx.part_world_position(counterweight)
        ctx.expect_gap(
            header,
            panel,
            axis="z",
            positive_elem="crossbeam",
            negative_elem="top_rail",
            min_gap=0.20,
            name="raised panel remains below the header beam",
        )
        ctx.expect_gap(
            panel,
            threshold,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="sill",
            min_gap=1.30,
            name="opened panel clears the threshold opening",
        )
        ctx.expect_within(
            counterweight,
            rear_column,
            axes="xy",
            inner_elem="mass_block",
            margin=0.0,
            name="lowered counterweight stays inside the rear guide column",
        )

    ctx.check(
        "panel rises while counterweight descends in synchronized travel",
        panel_rest is not None
        and counterweight_rest is not None
        and panel_raised is not None
        and counterweight_lowered is not None
        and panel_raised[2] > panel_rest[2] + 1.0
        and counterweight_lowered[2] < counterweight_rest[2] - 1.0,
        details=(
            f"panel_rest={panel_rest}, panel_raised={panel_raised}, "
            f"counterweight_rest={counterweight_rest}, "
            f"counterweight_lowered={counterweight_lowered}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
