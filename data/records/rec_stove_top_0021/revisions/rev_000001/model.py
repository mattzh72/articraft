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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_cooktop_in_counter", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    burner_black = model.material("burner_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.13, 0.14, 1.0))
    stone = model.material("counter_stone", rgba=(0.73, 0.72, 0.69, 1.0))

    counter_width = 0.90
    counter_depth = 0.65
    counter_thickness = 0.038
    cutout_width = 0.60
    cutout_depth = 0.52

    deck_width = cutout_width
    deck_depth = cutout_depth
    deck_thickness = 0.004

    countertop = model.part("countertop")
    countertop.inertial = Inertial.from_geometry(
        Box((counter_width, counter_depth, counter_thickness)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -counter_thickness * 0.5)),
    )
    front_back_depth = (counter_depth - cutout_depth) * 0.5
    side_width = (counter_width - cutout_width) * 0.5
    countertop.visual(
        Box((counter_width, front_back_depth, counter_thickness)),
        origin=Origin(
            xyz=(0.0, -(cutout_depth * 0.5 + front_back_depth * 0.5), -counter_thickness * 0.5)
        ),
        material=stone,
        name="counter_front",
    )
    countertop.visual(
        Box((counter_width, front_back_depth, counter_thickness)),
        origin=Origin(
            xyz=(0.0, cutout_depth * 0.5 + front_back_depth * 0.5, -counter_thickness * 0.5)
        ),
        material=stone,
        name="counter_back",
    )
    countertop.visual(
        Box((side_width, cutout_depth, counter_thickness)),
        origin=Origin(
            xyz=(-(cutout_width * 0.5 + side_width * 0.5), 0.0, -counter_thickness * 0.5)
        ),
        material=stone,
        name="counter_left",
    )
    countertop.visual(
        Box((side_width, cutout_depth, counter_thickness)),
        origin=Origin(
            xyz=(cutout_width * 0.5 + side_width * 0.5, 0.0, -counter_thickness * 0.5)
        ),
        material=stone,
        name="counter_right",
    )

    cooktop = model.part("cooktop")
    cooktop.inertial = Inertial.from_geometry(
        Box((deck_width, deck_depth, 0.060)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )
    deck_profile = rounded_rect_profile(deck_width, deck_depth, radius=0.026, corner_segments=8)
    cooktop.visual(
        _save_mesh("cooktop_deck.obj", ExtrudeGeometry(deck_profile, deck_thickness, center=True)),
        origin=Origin(xyz=(0.0, 0.0, -deck_thickness * 0.5)),
        material=stainless,
        name="deck",
    )

    skirt_height = 0.028
    skirt_thickness = 0.016
    inner_open_width = deck_width - 2.0 * skirt_thickness
    inner_open_depth = deck_depth - 2.0 * skirt_thickness
    skirt_center_z = -(deck_thickness + skirt_height * 0.5)
    cooktop.visual(
        Box((deck_width, skirt_thickness, skirt_height)),
        origin=Origin(xyz=(0.0, -(deck_depth * 0.5 - skirt_thickness * 0.5), skirt_center_z)),
        material=brushed_steel,
        name="skirt_front",
    )
    cooktop.visual(
        Box((deck_width, skirt_thickness, skirt_height)),
        origin=Origin(xyz=(0.0, deck_depth * 0.5 - skirt_thickness * 0.5, skirt_center_z)),
        material=brushed_steel,
        name="skirt_back",
    )
    cooktop.visual(
        Box((skirt_thickness, inner_open_depth, skirt_height)),
        origin=Origin(xyz=(-(deck_width * 0.5 - skirt_thickness * 0.5), 0.0, skirt_center_z)),
        material=brushed_steel,
        name="skirt_left",
    )
    cooktop.visual(
        Box((skirt_thickness, inner_open_depth, skirt_height)),
        origin=Origin(xyz=(deck_width * 0.5 - skirt_thickness * 0.5, 0.0, skirt_center_z)),
        material=brushed_steel,
        name="skirt_right",
    )

    control_pod_size = (0.220, 0.048, 0.118)
    control_pod_center = (0.0, -(deck_depth * 0.5 - control_pod_size[1] * 0.5), -0.072)
    control_pod_profile = rounded_rect_profile(
        control_pod_size[0],
        control_pod_size[1],
        radius=0.010,
        corner_segments=6,
    )
    cooktop.visual(
        _save_mesh(
            "control_pod.obj",
            ExtrudeGeometry(control_pod_profile, control_pod_size[2], center=True),
        ),
        origin=Origin(xyz=control_pod_center),
        material=stainless,
        name="control_pod",
    )

    def add_burner(
        name: str,
        *,
        xy: tuple[float, float],
        base_radius: float,
        grate_span: float,
    ) -> None:
        x, y = xy
        cooktop.visual(
            Cylinder(radius=base_radius, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=burner_black,
            name=f"{name}_base",
        )
        cooktop.visual(
            Cylinder(radius=base_radius * 0.78, length=0.006),
            origin=Origin(xyz=(x, y, 0.013)),
            material=cast_iron,
            name=f"{name}_head",
        )
        cooktop.visual(
            Cylinder(radius=base_radius * 0.58, length=0.008),
            origin=Origin(xyz=(x, y, 0.020)),
            material=burner_black,
            name=f"{name}_cap",
        )
        cooktop.visual(
            Box((0.008, 0.020, 0.010)),
            origin=Origin(xyz=(x + base_radius * 0.72, y - base_radius * 0.18, 0.005)),
            material=brushed_steel,
            name=f"{name}_igniter",
        )

        foot_size = 0.016
        foot_height = 0.022
        foot_offset = grate_span * 0.34
        for index, (dx, dy) in enumerate(
            (
                (-foot_offset, -foot_offset),
                (-foot_offset, foot_offset),
                (foot_offset, -foot_offset),
                (foot_offset, foot_offset),
            ),
            start=1,
        ):
            cooktop.visual(
                Box((foot_size, foot_size, foot_height)),
                origin=Origin(xyz=(x + dx, y + dy, foot_height * 0.5)),
                material=cast_iron,
                name=f"{name}_grate_foot_{index}",
            )
        grate_bar_height = 0.010
        grate_bar_z = foot_height + grate_bar_height * 0.5
        cooktop.visual(
            Box((grate_span, 0.014, grate_bar_height)),
            origin=Origin(xyz=(x, y, grate_bar_z)),
            material=cast_iron,
            name=f"{name}_grate_x",
        )
        cooktop.visual(
            Box((0.014, grate_span, grate_bar_height)),
            origin=Origin(xyz=(x, y, grate_bar_z)),
            material=cast_iron,
            name=f"{name}_grate_y",
        )
        frame_span = grate_span * 0.92
        frame_offset = frame_span * 0.5 - 0.010
        for suffix, size, origin in (
            (
                "grate_frame_front",
                (frame_span, 0.012, grate_bar_height),
                (x, y - frame_offset, grate_bar_z),
            ),
            (
                "grate_frame_back",
                (frame_span, 0.012, grate_bar_height),
                (x, y + frame_offset, grate_bar_z),
            ),
            (
                "grate_frame_left",
                (0.012, frame_span, grate_bar_height),
                (x - frame_offset, y, grate_bar_z),
            ),
            (
                "grate_frame_right",
                (0.012, frame_span, grate_bar_height),
                (x + frame_offset, y, grate_bar_z),
            ),
        ):
            cooktop.visual(
                Box(size),
                origin=Origin(xyz=origin),
                material=cast_iron,
                name=f"{name}_{suffix}",
            )

    add_burner("burner_rear_left", xy=(-0.170, 0.118), base_radius=0.050, grate_span=0.162)
    add_burner("burner_front_left", xy=(-0.170, -0.086), base_radius=0.044, grate_span=0.154)
    add_burner("burner_rear_right", xy=(0.170, 0.118), base_radius=0.042, grate_span=0.150)
    add_burner("burner_front_right", xy=(0.170, -0.086), base_radius=0.038, grate_span=0.146)

    def make_knob(name: str) -> None:
        knob = model.part(name)
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.028),
            mass=0.12,
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-pi * 0.5, 0.0, 0.0)),
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.028),
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(-pi * 0.5, 0.0, 0.0)),
            material=burner_black,
            name="hub",
        )
        knob.visual(
            Box((0.005, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, -0.025, 0.009)),
            material=brushed_steel,
            name="pointer",
        )

    make_knob("knob_top")
    make_knob("knob_left")
    make_knob("knob_right")
    make_knob("knob_bottom")

    model.articulation(
        "counter_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(),
    )
    model.articulation(
        "knob_top_turn",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child="knob_top",
        origin=Origin(xyz=(0.0, -deck_depth * 0.5, -0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    model.articulation(
        "knob_left_turn",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child="knob_left",
        origin=Origin(xyz=(-0.045, -deck_depth * 0.5, -0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    model.articulation(
        "knob_right_turn",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child="knob_right",
        origin=Origin(xyz=(0.045, -deck_depth * 0.5, -0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    model.articulation(
        "knob_bottom_turn",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child="knob_bottom",
        origin=Origin(xyz=(0.0, -deck_depth * 0.5, -0.098)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    knob_top = object_model.get_part("knob_top")
    knob_left = object_model.get_part("knob_left")
    knob_right = object_model.get_part("knob_right")
    knob_bottom = object_model.get_part("knob_bottom")

    knob_top_turn = object_model.get_articulation("knob_top_turn")
    knob_left_turn = object_model.get_articulation("knob_left_turn")
    knob_right_turn = object_model.get_articulation("knob_right_turn")
    knob_bottom_turn = object_model.get_articulation("knob_bottom_turn")

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

    ctx.expect_contact(countertop, cooktop, name="cooktop_seated_in_counter_cutout")

    for knob in (knob_top, knob_left, knob_right, knob_bottom):
        ctx.expect_contact(knob, cooktop, name=f"{knob.name}_mounted_to_control_pod")
        ctx.expect_origin_gap(
            cooktop,
            knob,
            axis="y",
            min_gap=0.24,
            max_gap=0.28,
            name=f"{knob.name}_at_front_center_cluster",
        )

    ctx.expect_origin_distance(
        knob_left,
        knob_right,
        axes="x",
        min_dist=0.085,
        max_dist=0.095,
        name="left_right_knob_spacing",
    )
    ctx.expect_origin_distance(
        knob_top,
        knob_bottom,
        axes="z",
        min_dist=0.039,
        max_dist=0.041,
        name="top_bottom_knob_spacing",
    )

    knob_positions = {
        "top": ctx.part_world_position(knob_top),
        "left": ctx.part_world_position(knob_left),
        "right": ctx.part_world_position(knob_right),
        "bottom": ctx.part_world_position(knob_bottom),
    }
    ctx.check(
        "knob_positions_resolve",
        all(position is not None for position in knob_positions.values()),
        details=str(knob_positions),
    )
    if all(position is not None for position in knob_positions.values()):
        top_pos = knob_positions["top"]
        left_pos = knob_positions["left"]
        right_pos = knob_positions["right"]
        bottom_pos = knob_positions["bottom"]
        assert top_pos is not None
        assert left_pos is not None
        assert right_pos is not None
        assert bottom_pos is not None

        ctx.check(
            "front_knob_cluster_centered",
            all(-0.275 <= pos[1] <= -0.245 for pos in (top_pos, left_pos, right_pos, bottom_pos))
            and abs((top_pos[0] + left_pos[0] + right_pos[0] + bottom_pos[0]) * 0.25) <= 0.002,
            details=str(knob_positions),
        )
        ctx.check(
            "diamond_knob_pattern",
            abs(top_pos[0]) <= 0.003
            and abs(bottom_pos[0]) <= 0.003
            and left_pos[0] < -0.035
            and right_pos[0] > 0.035
            and abs(left_pos[2] - right_pos[2]) <= 0.002
            and top_pos[2] > left_pos[2] > bottom_pos[2]
            and top_pos[2] > right_pos[2] > bottom_pos[2],
            details=str(knob_positions),
        )

    for joint in (knob_top_turn, knob_left_turn, knob_right_turn, knob_bottom_turn):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_front_to_back_axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_knob_poses_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=8, name="sampled_knob_poses_no_overlap")

    with ctx.pose(
        {
            knob_top_turn: pi * 0.5,
            knob_left_turn: pi,
            knob_right_turn: -pi * 0.5,
            knob_bottom_turn: pi * 0.25,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knob_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knob_pose_no_floating")
        for knob in (knob_top, knob_left, knob_right, knob_bottom):
            ctx.expect_contact(knob, cooktop, name=f"{knob.name}_stays_mounted_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
