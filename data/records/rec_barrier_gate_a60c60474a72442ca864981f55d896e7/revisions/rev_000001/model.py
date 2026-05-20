from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_leaf_visuals(
    part,
    *,
    width: float,
    height: float,
    thickness: float,
    frame_width: float,
    slat_count: int,
    direction: float,
    panel_center_y: float,
    frame_material,
    accent_material,
    add_post_hinge_barrels: bool = False,
    add_interleaf_hardware: bool = False,
) -> None:
    sign = 1.0 if direction >= 0.0 else -1.0
    rail_span = width - frame_width + 0.006
    slat_height = height - 2.0 * frame_width + 0.006
    inner_clear_span = width - 2.0 * frame_width
    slat_spacing = inner_clear_span / (slat_count + 1)
    slat_width = 0.024
    slat_thickness = thickness * 0.42

    part.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(sign * frame_width * 0.5, panel_center_y, height * 0.5)),
        material=frame_material,
        name="hinge_stile",
    )
    part.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(sign * (width - frame_width * 0.5), panel_center_y, height * 0.5)),
        material=frame_material,
        name="leading_stile",
    )
    part.visual(
        Box((rail_span, thickness, frame_width)),
        origin=Origin(xyz=(sign * width * 0.5, panel_center_y, frame_width * 0.5)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((rail_span, thickness, frame_width)),
        origin=Origin(xyz=(sign * width * 0.5, panel_center_y, height - frame_width * 0.5)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((rail_span, thickness * 0.88, frame_width * 0.72)),
        origin=Origin(xyz=(sign * width * 0.5, panel_center_y, height * 0.54)),
        material=frame_material,
        name="mid_rail",
    )

    for index in range(slat_count):
        x_pos = sign * (frame_width + slat_spacing * (index + 1))
        part.visual(
            Box((slat_width, slat_thickness, slat_height)),
            origin=Origin(xyz=(x_pos, panel_center_y, height * 0.5)),
            material=accent_material,
            name=f"slat_{index + 1}",
        )

    if add_post_hinge_barrels:
        for index, z_pos in enumerate((0.18, height * 0.5, height - 0.18), start=1):
            part.visual(
                Cylinder(radius=0.018, length=0.13),
                origin=Origin(xyz=(0.0, 0.0, z_pos)),
                material=accent_material,
                name=f"post_hinge_barrel_{index}",
            )

    if add_interleaf_hardware:
        edge_x = sign * width
        for index, z_pos in enumerate((0.20, height * 0.5, height - 0.20), start=1):
            part.visual(
                Box((0.012, 0.088, 0.10)),
                origin=Origin(xyz=(edge_x - sign * 0.004, -0.034, z_pos)),
                material=frame_material,
                name=f"interleaf_bracket_{index}",
            )

    latch_box_x = sign * (width - frame_width * 0.5)
    part.visual(
        Box((0.010, thickness * 0.40, 0.28)),
        origin=Origin(xyz=(latch_box_x, panel_center_y + thickness * 0.18, height * 0.56)),
        material=accent_material,
        name="latch_plate",
    )
def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_bifold_driveway_gate")

    powder_black = model.material("powder_black", rgba=(0.17, 0.18, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    footing_concrete = model.material("footing_concrete", rgba=(0.52, 0.52, 0.50, 1.0))

    post_width = 0.16
    post_depth = 0.16
    buried_depth = 0.20
    post_above_grade = 1.90
    post_height = buried_depth + post_above_grade

    leaf_width = 1.16
    leaf_height = 1.60
    leaf_thickness = 0.04
    frame_width = 0.05
    gate_bottom = 0.10
    hinge_y_offset = -0.03
    leaf_plane_center_y = 0.03

    left_post_center_x = -2.46
    right_post_center_x = 2.46
    left_outer_axis_x = -2.36
    right_outer_axis_x = 2.36
    interleaf_axis_offset = leaf_width + 0.02

    frame = model.part("frame")
    frame.visual(
        Box((5.08, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=footing_concrete,
        name="buried_tie_beam",
    )
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(left_post_center_x, 0.0, post_height * 0.5 - buried_depth)),
        material=dark_steel,
        name="left_post",
    )
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(right_post_center_x, 0.0, post_height * 0.5 - buried_depth)),
        material=dark_steel,
        name="right_post",
    )
    frame.visual(
        Box((0.18, 0.18, 0.022)),
        origin=Origin(xyz=(left_post_center_x, 0.0, post_above_grade + 0.011)),
        material=dark_steel,
        name="left_cap",
    )
    frame.visual(
        Box((0.18, 0.18, 0.022)),
        origin=Origin(xyz=(right_post_center_x, 0.0, post_above_grade + 0.011)),
        material=dark_steel,
        name="right_cap",
    )
    for side, x_pos in (("left", left_outer_axis_x), ("right", right_outer_axis_x)):
        for index, z_pos in enumerate((gate_bottom + 0.18, gate_bottom + leaf_height * 0.5, gate_bottom + leaf_height - 0.18), start=1):
            frame.visual(
                Box((0.040, 0.038, 0.11)),
                origin=Origin(xyz=(x_pos, -0.067, z_pos)),
                material=hinge_metal,
                name=f"{side}_post_bracket_{index}",
            )

    left_outer_leaf = model.part("left_outer_leaf")
    _add_leaf_visuals(
        left_outer_leaf,
        width=leaf_width,
        height=leaf_height,
        thickness=leaf_thickness,
        frame_width=frame_width,
        slat_count=5,
        direction=1.0,
        panel_center_y=leaf_plane_center_y,
        frame_material=powder_black,
        accent_material=dark_steel,
        add_post_hinge_barrels=True,
        add_interleaf_hardware=True,
    )

    left_inner_leaf = model.part("left_inner_leaf")
    _add_leaf_visuals(
        left_inner_leaf,
        width=leaf_width,
        height=leaf_height,
        thickness=leaf_thickness,
        frame_width=frame_width,
        slat_count=5,
        direction=1.0,
        panel_center_y=leaf_plane_center_y,
        frame_material=powder_black,
        accent_material=hinge_metal,
        add_post_hinge_barrels=True,
        add_interleaf_hardware=False,
    )

    right_outer_leaf = model.part("right_outer_leaf")
    _add_leaf_visuals(
        right_outer_leaf,
        width=leaf_width,
        height=leaf_height,
        thickness=leaf_thickness,
        frame_width=frame_width,
        slat_count=5,
        direction=-1.0,
        panel_center_y=leaf_plane_center_y,
        frame_material=powder_black,
        accent_material=dark_steel,
        add_post_hinge_barrels=True,
        add_interleaf_hardware=True,
    )

    right_inner_leaf = model.part("right_inner_leaf")
    _add_leaf_visuals(
        right_inner_leaf,
        width=leaf_width,
        height=leaf_height,
        thickness=leaf_thickness,
        frame_width=frame_width,
        slat_count=5,
        direction=-1.0,
        panel_center_y=leaf_plane_center_y,
        frame_material=powder_black,
        accent_material=hinge_metal,
        add_post_hinge_barrels=True,
        add_interleaf_hardware=False,
    )

    model.articulation(
        "left_post_to_outer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_outer_leaf,
        origin=Origin(xyz=(left_outer_axis_x, hinge_y_offset, gate_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.7, lower=0.0, upper=1.65),
    )
    model.articulation(
        "left_outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=left_outer_leaf,
        child=left_inner_leaf,
        origin=Origin(xyz=(interleaf_axis_offset, hinge_y_offset, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=2.95),
    )
    model.articulation(
        "right_post_to_outer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_outer_leaf,
        origin=Origin(xyz=(right_outer_axis_x, hinge_y_offset, gate_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.7, lower=0.0, upper=1.65),
    )
    model.articulation(
        "right_outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=right_outer_leaf,
        child=right_inner_leaf,
        origin=Origin(xyz=(-interleaf_axis_offset, hinge_y_offset, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=2.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    frame = object_model.get_part("frame")
    left_outer_leaf = object_model.get_part("left_outer_leaf")
    left_inner_leaf = object_model.get_part("left_inner_leaf")
    right_outer_leaf = object_model.get_part("right_outer_leaf")
    right_inner_leaf = object_model.get_part("right_inner_leaf")

    left_post_to_outer = object_model.get_articulation("left_post_to_outer")
    left_outer_to_inner = object_model.get_articulation("left_outer_to_inner")
    right_post_to_outer = object_model.get_articulation("right_post_to_outer")
    right_outer_to_inner = object_model.get_articulation("right_outer_to_inner")

    left_center_stile = left_inner_leaf.get_visual("leading_stile")
    right_center_stile = right_inner_leaf.get_visual("leading_stile")
    left_outer_hinge = left_outer_leaf.get_visual("hinge_stile")
    right_outer_hinge = right_outer_leaf.get_visual("hinge_stile")
    left_outer_leading = left_outer_leaf.get_visual("leading_stile")
    right_outer_leading = right_outer_leaf.get_visual("leading_stile")
    left_inner_hinge = left_inner_leaf.get_visual("hinge_stile")
    right_inner_hinge = right_inner_leaf.get_visual("hinge_stile")

    ctx.expect_gap(
        right_inner_leaf,
        left_inner_leaf,
        axis="x",
        positive_elem=right_center_stile,
        negative_elem=left_center_stile,
        min_gap=0.02,
        max_gap=0.06,
        name="center leaves meet with a narrow latch gap",
    )
    ctx.expect_overlap(
        left_inner_leaf,
        right_inner_leaf,
        axes="z",
        elem_a=left_center_stile,
        elem_b=right_center_stile,
        min_overlap=1.45,
        name="center meeting stiles align through most of the height",
    )
    ctx.expect_gap(
        left_outer_leaf,
        frame,
        axis="x",
        positive_elem=left_outer_hinge,
        negative_elem="left_post",
        min_gap=0.0,
        max_gap=0.03,
        name="left outer leaf hangs just clear of the left post",
    )
    ctx.expect_gap(
        frame,
        right_outer_leaf,
        axis="x",
        positive_elem="right_post",
        negative_elem=right_outer_hinge,
        min_gap=0.0,
        max_gap=0.03,
        name="right outer leaf hangs just clear of the right post",
    )

    closed_left_inner_center = _aabb_center(ctx.part_world_aabb(left_inner_leaf))
    closed_right_inner_center = _aabb_center(ctx.part_world_aabb(right_inner_leaf))

    with ctx.pose(
        {
            left_post_to_outer: 1.25,
            left_outer_to_inner: 2.75,
            right_post_to_outer: 1.25,
            right_outer_to_inner: 2.75,
        }
    ):
        open_left_inner_center = _aabb_center(ctx.part_world_aabb(left_inner_leaf))
        open_right_inner_center = _aabb_center(ctx.part_world_aabb(right_inner_leaf))
        open_left_outer_center = _aabb_center(ctx.part_world_aabb(left_outer_leaf))
        open_right_outer_center = _aabb_center(ctx.part_world_aabb(right_outer_leaf))

        ctx.expect_gap(
            left_outer_leaf,
            left_inner_leaf,
            axis="x",
            positive_elem=left_outer_leading,
            negative_elem=left_inner_hinge,
            min_gap=0.01,
            name="left folded pair keeps its frame members distinct",
        )
        ctx.expect_gap(
            right_inner_leaf,
            right_outer_leaf,
            axis="x",
            positive_elem=right_inner_hinge,
            negative_elem=right_outer_leading,
            min_gap=0.01,
            name="right folded pair keeps its frame members distinct",
        )

        ctx.check(
            "left gate pack folds back toward the left post",
            closed_left_inner_center is not None
            and open_left_inner_center is not None
            and open_left_outer_center is not None
            and open_left_inner_center[0] < closed_left_inner_center[0] - 1.15
            and open_left_inner_center[1] < -0.55
            and open_left_inner_center[1] < open_left_outer_center[1] - 0.02,
            details=(
                f"closed_left_inner_center={closed_left_inner_center}, "
                f"open_left_inner_center={open_left_inner_center}, "
                f"open_left_outer_center={open_left_outer_center}"
            ),
        )
        ctx.check(
            "right gate pack folds back toward the right post",
            closed_right_inner_center is not None
            and open_right_inner_center is not None
            and open_right_outer_center is not None
            and open_right_inner_center[0] > closed_right_inner_center[0] + 1.15
            and open_right_inner_center[1] < -0.55
            and open_right_inner_center[1] < open_right_outer_center[1] - 0.02,
            details=(
                f"closed_right_inner_center={closed_right_inner_center}, "
                f"open_right_inner_center={open_right_inner_center}, "
                f"open_right_outer_center={open_right_outer_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
