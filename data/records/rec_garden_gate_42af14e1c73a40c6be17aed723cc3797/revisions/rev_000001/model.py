from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_picket_garden_gate")

    post_mat = model.material("cedar_post", rgba=(0.48, 0.33, 0.21, 1.0))
    sill_mat = model.material("stone_sill", rgba=(0.58, 0.58, 0.56, 1.0))
    gate_mat = model.material("painted_gate", rgba=(0.13, 0.18, 0.14, 1.0))
    hardware_mat = model.material("hardware", rgba=(0.34, 0.36, 0.39, 1.0))

    opening_width = 1.52
    post_width = 0.14
    post_height = 1.50
    post_cap_height = 0.04
    threshold_height = 0.06
    threshold_width = 0.20
    threshold_length = opening_width + 2.0 * post_width + 0.10
    post_center_x = opening_width / 2.0 + post_width / 2.0

    leaf_width = 0.725
    leaf_thickness = 0.038
    leaf_height = 1.08
    leaf_bottom_z = threshold_height + 0.025
    hinge_axis_inset = 0.012
    hinge_axis_left_x = -opening_width / 2.0 + hinge_axis_inset
    hinge_axis_right_x = opening_width / 2.0 - hinge_axis_inset

    stile_width = 0.055
    rail_height = 0.07
    hinge_stile_center = hinge_axis_inset + stile_width / 2.0
    free_stile_center = leaf_width - stile_width / 2.0
    rail_length = leaf_width - hinge_axis_inset - 2.0 * stile_width
    rail_center = hinge_axis_inset + stile_width + rail_length / 2.0
    bottom_rail_center_z = 0.16
    top_rail_center_z = 0.82

    picket_width = 0.032
    picket_height = 0.94
    picket_center_z = 0.55
    picket_tip_height = 0.050
    picket_positions = (0.13, 0.25, 0.37, 0.49, 0.61)

    hinge_barrel_radius = 0.012
    hinge_barrel_length = 0.15
    hinge_barrel_zs = (0.18, 0.54, 0.90)

    latch_height_z = 0.62
    latch_pivot_x = 0.628
    latch_pivot_radius = 0.013
    latch_bar_length = 0.118

    bolt_x = -(leaf_width - stile_width / 2.0 - 0.006)
    bolt_axis_y = leaf_thickness / 2.0 + 0.009
    bolt_lower_z = 0.01
    bolt_travel = 0.16
    guide_loop_major = 0.010
    guide_loop_tube = 0.0025
    guide_loop_zs = (0.24, 0.76)

    guide_loop_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=guide_loop_major,
            tube=guide_loop_tube,
            radial_segments=18,
            tubular_segments=28,
        ),
        "cane_bolt_loop",
    )
    picket_finial_mesh = mesh_from_geometry(
        ConeGeometry(
            radius=picket_width * 0.62,
            height=picket_tip_height,
            radial_segments=4,
            closed=True,
        ),
        "picket_finial",
    )
    post_finial_mesh = mesh_from_geometry(
        ConeGeometry(
            radius=(post_width + 0.03) * 0.36,
            height=0.070,
            radial_segments=4,
            closed=True,
        ),
        "post_finial",
    )

    opening_frame = model.part("opening_frame")
    opening_frame.visual(
        Box((threshold_length, threshold_width, threshold_height)),
        origin=Origin(xyz=(0.0, 0.0, threshold_height / 2.0)),
        material=sill_mat,
        name="threshold_sill",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        post_x = x_sign * post_center_x
        opening_frame.visual(
            Box((post_width, threshold_width, post_height)),
            origin=Origin(
                xyz=(post_x, 0.0, threshold_height + post_height / 2.0)
            ),
            material=post_mat,
            name=f"{side}_post",
        )
        opening_frame.visual(
            Box((post_width + 0.03, threshold_width + 0.03, post_cap_height)),
            origin=Origin(
                xyz=(
                    post_x,
                    0.0,
                    threshold_height + post_height + post_cap_height / 2.0,
                )
            ),
            material=post_mat,
            name=f"{side}_post_cap",
        )
        opening_frame.visual(
            post_finial_mesh,
            origin=Origin(
                xyz=(
                    post_x,
                    0.0,
                    threshold_height
                    + post_height
                    + post_cap_height
                    + 0.070 / 2.0
                    - 0.001,
                ),
                rpy=(0.0, 0.0, math.pi / 4.0),
            ),
            material=post_mat,
            name=f"{side}_post_finial",
        )

    def add_leaf(
        part_name: str, sign: float, *, with_keeper: bool, with_latch_mount: bool
    ) -> None:
        leaf = model.part(part_name)
        hinge_name = f"{part_name.split('_')[0]}_hinge_stile"
        free_name = f"{part_name.split('_')[0]}_free_stile"
        leaf.visual(
            Box((stile_width, leaf_thickness, leaf_height)),
            origin=Origin(xyz=(sign * hinge_stile_center, 0.0, leaf_height / 2.0)),
            material=gate_mat,
            name=hinge_name,
        )
        leaf.visual(
            Box((stile_width, leaf_thickness, leaf_height)),
            origin=Origin(xyz=(sign * free_stile_center, 0.0, leaf_height / 2.0)),
            material=gate_mat,
            name=free_name,
        )
        leaf.visual(
            Box((rail_length, leaf_thickness, rail_height)),
            origin=Origin(xyz=(sign * rail_center, 0.0, bottom_rail_center_z)),
            material=gate_mat,
            name=f"{part_name}_bottom_rail",
        )
        leaf.visual(
            Box((rail_length, leaf_thickness, rail_height)),
            origin=Origin(xyz=(sign * rail_center, 0.0, top_rail_center_z)),
            material=gate_mat,
            name=f"{part_name}_top_rail",
        )
        for idx, x in enumerate(picket_positions, start=1):
            leaf.visual(
                Box((picket_width, leaf_thickness * 0.55, picket_height)),
                origin=Origin(xyz=(sign * x, 0.0, picket_center_z)),
                material=gate_mat,
                name=f"{part_name}_picket_{idx}",
            )
            leaf.visual(
                picket_finial_mesh,
                origin=Origin(
                    xyz=(
                        sign * x,
                        0.0,
                        picket_center_z + picket_height / 2.0 + picket_tip_height / 2.0 - 0.001,
                    ),
                    rpy=(0.0, 0.0, math.pi / 4.0),
                ),
                material=gate_mat,
                name=f"{part_name}_picket_tip_{idx}",
            )
        for idx, z in enumerate(hinge_barrel_zs, start=1):
            leaf.visual(
                Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=hardware_mat,
                name=f"{part_name}_hinge_barrel_{idx}",
            )
        if with_latch_mount:
            leaf.visual(
                Box((0.060, leaf_thickness, 0.060)),
                origin=Origin(
                    xyz=(latch_pivot_x, 0.0, latch_height_z)
                ),
                material=hardware_mat,
                name="latch_mount_pad",
            )
        if with_keeper:
            leaf.visual(
                Box((0.030, 0.014, 0.050)),
                origin=Origin(
                    xyz=(-leaf_width + 0.002, leaf_thickness / 2.0 + 0.007, latch_height_z)
                ),
                material=hardware_mat,
                name="latch_keeper",
            )
            for idx, z in enumerate(guide_loop_zs):
                leaf.visual(
                    guide_loop_mesh,
                    origin=Origin(xyz=(bolt_x, bolt_axis_y, z)),
                    material=hardware_mat,
                    name="guide_loop_lower" if idx == 0 else "guide_loop_upper",
                )

    add_leaf("left_leaf", 1.0, with_keeper=False, with_latch_mount=True)
    add_leaf("right_leaf", -1.0, with_keeper=True, with_latch_mount=False)

    latch = model.part("center_latch")
    latch.visual(
        Cylinder(radius=latch_pivot_radius, length=0.018),
        material=hardware_mat,
        name="latch_pivot",
    )
    latch.visual(
        Box((latch_bar_length, 0.014, 0.018)),
        origin=Origin(xyz=(latch_bar_length / 2.0, 0.0, 0.0)),
        material=hardware_mat,
        name="latch_bar",
    )
    latch.visual(
        Box((0.024, 0.030, 0.012)),
        origin=Origin(xyz=(0.020, 0.013, 0.0)),
        material=hardware_mat,
        name="latch_thumb",
    )

    cane_bolt = model.part("cane_bolt")
    cane_bolt.visual(
        Cylinder(radius=0.004, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=hardware_mat,
        name="cane_bolt_rod",
    )
    cane_bolt.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.7685)),
        material=hardware_mat,
        name="cane_bolt_stop",
    )
    cane_bolt.visual(
        Box((0.040, 0.010, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.70)),
        material=hardware_mat,
        name="cane_bolt_handle",
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child="left_leaf",
        origin=Origin(xyz=(hinge_axis_left_x, 0.0, leaf_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=0.0,
            upper=1.6,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child="right_leaf",
        origin=Origin(xyz=(hinge_axis_right_x, 0.0, leaf_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=0.0,
            upper=1.6,
        ),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent="left_leaf",
        child=latch,
        origin=Origin(
            xyz=(
                latch_pivot_x,
                leaf_thickness / 2.0 + latch_pivot_radius,
                latch_height_z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "cane_bolt_slide",
        ArticulationType.PRISMATIC,
        parent="right_leaf",
        child=cane_bolt,
        origin=Origin(xyz=(bolt_x, bolt_axis_y, bolt_lower_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.25,
            lower=0.0,
            upper=bolt_travel,
        ),
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

    opening_frame = object_model.get_part("opening_frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    latch = object_model.get_part("center_latch")
    cane_bolt = object_model.get_part("cane_bolt")

    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")
    cane_bolt_slide = object_model.get_articulation("cane_bolt_slide")

    threshold_sill = opening_frame.get_visual("threshold_sill")
    left_free_stile = left_leaf.get_visual("left_free_stile")
    latch_mount_pad = left_leaf.get_visual("latch_mount_pad")
    right_free_stile = right_leaf.get_visual("right_free_stile")
    latch_pivot_visual = latch.get_visual("latch_pivot")
    latch_bar = latch.get_visual("latch_bar")
    latch_keeper = right_leaf.get_visual("latch_keeper")
    cane_bolt_rod = cane_bolt.get_visual("cane_bolt_rod")
    guide_loop_lower = right_leaf.get_visual("guide_loop_lower")
    guide_loop_upper = right_leaf.get_visual("guide_loop_upper")

    ctx.check(
        "all gate parts exist",
        all(part is not None for part in (opening_frame, left_leaf, right_leaf, latch, cane_bolt)),
        details="Expected frame, two leaves, latch, and cane bolt parts.",
    )
    ctx.check(
        "all gate articulations exist",
        all(
            joint is not None
            for joint in (left_hinge, right_hinge, latch_pivot, cane_bolt_slide)
        ),
        details="Expected two leaf hinges, one latch pivot, and one cane bolt slide.",
    )

    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            latch_pivot: 0.0,
            cane_bolt_slide: 0.0,
        }
    ):
        ctx.expect_gap(
            right_leaf,
            left_leaf,
            axis="x",
            positive_elem=right_free_stile,
            negative_elem=left_free_stile,
            min_gap=0.035,
            max_gap=0.055,
            name="closed leaves keep a narrow center seam",
        )
        ctx.expect_gap(
            left_leaf,
            opening_frame,
            axis="z",
            positive_elem=left_free_stile,
            negative_elem=threshold_sill,
            min_gap=0.020,
            max_gap=0.030,
            name="leaf frame clears the threshold sill",
        )
        ctx.expect_contact(
            latch,
            left_leaf,
            elem_a=latch_pivot_visual,
            elem_b=latch_mount_pad,
            contact_tol=0.0015,
            name="latch pivot bears on the active leaf mount pad",
        )
        ctx.expect_gap(
            right_leaf,
            latch,
            axis="x",
            positive_elem=latch_keeper,
            negative_elem=latch_bar,
            min_gap=0.004,
            max_gap=0.018,
            name="closed latch bar sits just short of the keeper",
        )
        ctx.expect_overlap(
            cane_bolt,
            right_leaf,
            axes="xy",
            elem_a=cane_bolt_rod,
            elem_b=guide_loop_lower,
            min_overlap=0.006,
            name="cane bolt stays centered in the lower guide",
        )
        ctx.expect_overlap(
            cane_bolt,
            right_leaf,
            axes="xy",
            elem_a=cane_bolt_rod,
            elem_b=guide_loop_upper,
            min_overlap=0.006,
            name="cane bolt stays centered in the upper guide",
        )

        left_closed = ctx.part_element_world_aabb(left_leaf, elem=left_free_stile)
        right_closed = ctx.part_element_world_aabb(right_leaf, elem=right_free_stile)
        latch_closed = ctx.part_element_world_aabb(latch, elem=latch_bar)
        bolt_closed = ctx.part_element_world_aabb(cane_bolt, elem=cane_bolt_rod)

    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0, cane_bolt_slide: 0.16}):
        left_open = ctx.part_element_world_aabb(left_leaf, elem=left_free_stile)
        right_open = ctx.part_element_world_aabb(right_leaf, elem=right_free_stile)
        ctx.check(
            "both leaves swing inward about their post hinges",
            all(aabb is not None for aabb in (left_closed, right_closed, left_open, right_open))
            and left_open[0][1] > left_closed[0][1] + 0.25
            and right_open[0][1] > right_closed[0][1] + 0.25,
            details=(
                f"left_closed={left_closed}, left_open={left_open}, "
                f"right_closed={right_closed}, right_open={right_open}"
            ),
        )
        ctx.expect_overlap(
            cane_bolt,
            right_leaf,
            axes="xy",
            elem_a=cane_bolt_rod,
            elem_b=guide_loop_upper,
            min_overlap=0.006,
            name="raised cane bolt remains captured by the upper guide",
        )
        bolt_open = ctx.part_element_world_aabb(cane_bolt, elem=cane_bolt_rod)

    with ctx.pose({latch_pivot: 0.8}):
        latch_open = ctx.part_element_world_aabb(latch, elem=latch_bar)
        ctx.check(
            "latch bar swings away from the seam on its pivot",
            latch_closed is not None
            and latch_open is not None
            and latch_open[1][1] > latch_closed[1][1] + 0.05,
            details=f"latch_closed={latch_closed}, latch_open={latch_open}",
        )

    ctx.check(
        "cane bolt lifts vertically in its guides",
        bolt_closed is not None
        and bolt_open is not None
        and bolt_open[0][2] > bolt_closed[0][2] + 0.12,
        details=f"bolt_closed={bolt_closed}, bolt_open={bolt_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
