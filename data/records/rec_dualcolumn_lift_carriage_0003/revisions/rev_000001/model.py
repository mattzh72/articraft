from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage", assets=ASSETS)

    base_width = 0.310
    base_depth = 0.100
    crosshead_thickness = 0.030
    column_x = 0.030
    column_y = 0.032
    column_spacing = 0.220
    column_length = 0.440
    top_crosshead_z = crosshead_thickness + column_length + crosshead_thickness / 2.0

    carriage_plate_x = 0.166
    carriage_plate_y = 0.018
    carriage_plate_z = 0.180
    side_plate_x = 0.022
    side_plate_y = 0.032
    side_plate_z = 0.180
    side_plate_center_x = 0.060
    side_plate_center_z = 0.122
    center_plate_x = 0.058
    center_plate_y = 0.036
    center_plate_z = 0.150
    center_plate_center_z = 0.122
    block_x = 0.024
    block_y = 0.034
    block_z = 0.040
    left_block_x = -column_spacing / 2.0 + column_x / 2.0 + block_x / 2.0
    right_block_x = column_spacing / 2.0 - column_x / 2.0 - block_x / 2.0
    upper_block_z = 0.174
    lower_block_z = 0.070
    lift_travel = 0.250

    steel = model.material("painted_steel", rgba=(0.24, 0.27, 0.31, 1.0))
    shaft = model.material("shaft_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    guide_block_color = model.material("guide_block", rgba=(0.39, 0.43, 0.47, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((base_width, base_depth, crosshead_thickness)),
        origin=Origin(xyz=(0.0, 0.0, crosshead_thickness / 2.0)),
        material=steel,
        name="base_crosshead",
    )
    frame.visual(
        Box((base_width, base_depth, crosshead_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_crosshead_z)),
        material=steel,
        name="top_crosshead",
    )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((column_x, column_y, column_length)),
            origin=Origin(
                xyz=(
                    x_sign * column_spacing / 2.0,
                    0.0,
                    crosshead_thickness + column_length / 2.0,
                )
            ),
            material=shaft,
            name=f"{side}_column",
        )

    frame.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, top_crosshead_z + crosshead_thickness / 2.0)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (top_crosshead_z + crosshead_thickness / 2.0) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((carriage_plate_x, carriage_plate_y, carriage_plate_z)),
        origin=Origin(xyz=(0.0, 0.0, side_plate_center_z)),
        material=carriage_gray,
        name="carriage_plate",
    )
    carriage.visual(
        Box((side_plate_x, side_plate_y, side_plate_z)),
        origin=Origin(xyz=(-side_plate_center_x, 0.0, side_plate_center_z)),
        material=carriage_gray,
        name="left_side_plate",
    )
    carriage.visual(
        Box((side_plate_x, side_plate_y, side_plate_z)),
        origin=Origin(xyz=(side_plate_center_x, 0.0, side_plate_center_z)),
        material=carriage_gray,
        name="right_side_plate",
    )
    carriage.visual(
        Box((center_plate_x, center_plate_y, center_plate_z)),
        origin=Origin(xyz=(0.0, 0.0, center_plate_center_z)),
        material=carriage_gray,
        name="center_plate",
    )

    block_positions = {
        "left_lower_block": (left_block_x, 0.0, lower_block_z),
        "left_upper_block": (left_block_x, 0.0, upper_block_z),
        "right_lower_block": (right_block_x, 0.0, lower_block_z),
        "right_upper_block": (right_block_x, 0.0, upper_block_z),
    }
    for block_name, xyz in block_positions.items():
        carriage.visual(
            Box((block_x, block_y, block_z)),
            origin=Origin(xyz=xyz),
            material=guide_block_color,
            name=block_name,
        )

    carriage.inertial = Inertial.from_geometry(
        Box((carriage_plate_x, center_plate_y, carriage_plate_z)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, side_plate_center_z)),
    )

    model.articulation(
        "lift_axis",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.25,
            lower=0.0,
            upper=lift_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift_axis = object_model.get_articulation("lift_axis")

    base_crosshead = frame.get_visual("base_crosshead")
    top_crosshead = frame.get_visual("top_crosshead")
    left_column = frame.get_visual("left_column")
    right_column = frame.get_visual("right_column")

    carriage_plate = carriage.get_visual("carriage_plate")
    left_side_plate = carriage.get_visual("left_side_plate")
    right_side_plate = carriage.get_visual("right_side_plate")
    center_plate = carriage.get_visual("center_plate")
    left_lower_block = carriage.get_visual("left_lower_block")
    left_upper_block = carriage.get_visual("left_upper_block")
    right_lower_block = carriage.get_visual("right_lower_block")
    right_upper_block = carriage.get_visual("right_upper_block")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "frame visuals present",
        all(v is not None for v in (base_crosshead, top_crosshead, left_column, right_column)),
        "expected base, top crosshead, and both guide columns",
    )
    ctx.check(
        "carriage visuals present",
        all(
            v is not None
            for v in (
                carriage_plate,
                left_side_plate,
                right_side_plate,
                center_plate,
                left_lower_block,
                left_upper_block,
                right_lower_block,
                right_upper_block,
            )
        ),
        "expected carriage plate, side plates, center plate, and four guide blocks",
    )
    ctx.check(
        "lift joint configuration",
        lift_axis.axis == (0.0, 0.0, 1.0)
        and lift_axis.motion_limits is not None
        and abs(lift_axis.motion_limits.lower - 0.0) < 1e-9
        and abs(lift_axis.motion_limits.upper - 0.25) < 1e-9,
        "lift axis should be a vertical 250 mm prismatic joint",
    )

    ctx.expect_overlap(carriage, frame, axes="x", min_overlap=0.16, elem_a=carriage_plate, elem_b=base_crosshead)
    ctx.expect_gap(carriage, frame, axis="z", min_gap=0.018, positive_elem=left_lower_block, negative_elem=base_crosshead)
    ctx.expect_gap(carriage, frame, axis="z", min_gap=0.018, positive_elem=right_lower_block, negative_elem=base_crosshead)
    ctx.expect_contact(carriage, frame, elem_a=left_lower_block, elem_b=left_column)
    ctx.expect_contact(carriage, frame, elem_a=left_upper_block, elem_b=left_column)
    ctx.expect_contact(carriage, frame, elem_a=right_lower_block, elem_b=right_column)
    ctx.expect_contact(carriage, frame, elem_a=right_upper_block, elem_b=right_column)
    ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=left_lower_block, elem_b=left_column)
    ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=left_upper_block, elem_b=left_column)
    ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=right_lower_block, elem_b=right_column)
    ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=right_upper_block, elem_b=right_column)

    def aabb_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    def check_block_centering(block_visual, column_visual, label: str) -> None:
        block_center = aabb_center(carriage, block_visual)
        column_center = aabb_center(frame, column_visual)
        ok = (
            block_center is not None
            and column_center is not None
            and abs(block_center[1] - column_center[1]) <= 0.0015
        )
        ctx.check(
            f"{label} aligned with column slot",
            ok,
            f"expected {label} to stay laterally aligned with its guide column in Y",
        )

    check_block_centering(left_lower_block, left_column, "left lower block")
    check_block_centering(left_upper_block, left_column, "left upper block")
    check_block_centering(right_lower_block, right_column, "right lower block")
    check_block_centering(right_upper_block, right_column, "right upper block")

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift_axis: 0.25}):
        upper_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(frame, carriage, axis="z", min_gap=0.024, positive_elem=top_crosshead, negative_elem=left_upper_block)
        ctx.expect_gap(frame, carriage, axis="z", min_gap=0.024, positive_elem=top_crosshead, negative_elem=right_upper_block)
        ctx.expect_contact(carriage, frame, elem_a=left_lower_block, elem_b=left_column)
        ctx.expect_contact(carriage, frame, elem_a=left_upper_block, elem_b=left_column)
        ctx.expect_contact(carriage, frame, elem_a=right_lower_block, elem_b=right_column)
        ctx.expect_contact(carriage, frame, elem_a=right_upper_block, elem_b=right_column)
        ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=left_lower_block, elem_b=left_column)
        ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=left_upper_block, elem_b=left_column)
        ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=right_lower_block, elem_b=right_column)
        ctx.expect_overlap(carriage, frame, axes="z", min_overlap=0.025, elem_a=right_upper_block, elem_b=right_column)
        check_block_centering(left_lower_block, left_column, "left lower block at full lift")
        check_block_centering(left_upper_block, left_column, "left upper block at full lift")
        check_block_centering(right_lower_block, right_column, "right lower block at full lift")
        check_block_centering(right_upper_block, right_column, "right upper block at full lift")
        ctx.check(
            "carriage translates 250 mm vertically",
            rest_pos is not None
            and upper_pos is not None
            and abs((upper_pos[2] - rest_pos[2]) - 0.25) <= 1e-6
            and abs(upper_pos[0] - rest_pos[0]) <= 1e-9
            and abs(upper_pos[1] - rest_pos[1]) <= 1e-9,
            "carriage should move only in +Z by the commanded 250 mm stroke",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
