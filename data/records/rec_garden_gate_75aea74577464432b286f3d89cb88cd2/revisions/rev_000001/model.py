from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians, pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_garden_gate")

    cedar = model.material("cedar", rgba=(0.58, 0.39, 0.22, 1.0))
    painted_wood = model.material("painted_wood", rgba=(0.50, 0.32, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.23, 0.24, 0.25, 1.0))
    galvanized = model.material("galvanized", rgba=(0.69, 0.71, 0.73, 1.0))
    concrete = model.material("concrete", rgba=(0.58, 0.58, 0.56, 1.0))

    post_size = 0.10
    post_height = 1.65
    post_cap = 0.02
    opening = 1.00
    right_post_x = opening + post_size

    gate_bottom = 0.08
    gate_height = 1.20
    gate_depth = 0.04
    gate_y_offset = 0.02
    hinge_axis_x = 0.092
    hinge_axis_y = -0.09
    lower_hinge_z = 0.20
    upper_hinge_z = 1.00

    hinge_stile_width = 0.06
    latch_stile_width = 0.04
    hinge_stile_center_x = 0.03
    latch_stile_center_x = 0.924
    rail_width = 0.844
    rail_center_x = 0.482
    rail_height = 0.03

    footing = model.part("footing")
    footing.visual(
        Box((1.34, 0.18, 0.14)),
        origin=Origin(xyz=(0.55, 0.0, -0.07)),
        material=concrete,
        name="footing_block",
    )
    footing.inertial = Inertial.from_geometry(
        Box((1.34, 0.18, 0.14)),
        mass=140.0,
        origin=Origin(xyz=(0.55, 0.0, -0.07)),
    )

    hinge_post = model.part("hinge_post")
    hinge_post.visual(
        Box((post_size, post_size, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height * 0.5)),
        material=painted_wood,
        name="post_shaft",
    )
    hinge_post.visual(
        Box((0.12, 0.12, post_cap)),
        origin=Origin(xyz=(0.0, 0.0, post_height + post_cap * 0.5)),
        material=painted_wood,
        name="post_cap",
    )
    for hinge_name, hinge_z in (("lower", lower_hinge_z), ("upper", upper_hinge_z)):
        hinge_post.visual(
            Box((0.042, 0.04, 0.10)),
            origin=Origin(xyz=(0.071, -0.07, gate_bottom + hinge_z)),
            material=steel,
            name=f"{hinge_name}_hinge_knuckle",
        )
    hinge_post.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, post_height + post_cap)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + post_cap) * 0.5)),
    )

    latch_post = model.part("latch_post")
    latch_post.visual(
        Box((post_size, post_size, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height * 0.5)),
        material=painted_wood,
        name="post_shaft",
    )
    latch_post.visual(
        Box((0.12, 0.12, post_cap)),
        origin=Origin(xyz=(0.0, 0.0, post_height + post_cap * 0.5)),
        material=painted_wood,
        name="post_cap",
    )
    latch_post.visual(
        Box((0.006, 0.04, 0.18)),
        origin=Origin(xyz=(-0.053, -0.07, 0.76)),
        material=steel,
        name="strike_plate",
    )
    latch_post.visual(
        Box((0.01, 0.04, 0.05)),
        origin=Origin(xyz=(-0.045, -0.07, 0.76)),
        material=steel,
        name="keeper_block",
    )
    latch_post.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, post_height + post_cap)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + post_cap) * 0.5)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((hinge_stile_width, gate_depth, gate_height)),
        origin=Origin(xyz=(hinge_stile_center_x, gate_y_offset, gate_height * 0.5)),
        material=painted_wood,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((latch_stile_width, gate_depth, gate_height)),
        origin=Origin(xyz=(latch_stile_center_x, gate_y_offset, gate_height * 0.5)),
        material=painted_wood,
        name="latch_stile",
    )
    gate_leaf.visual(
        Box((rail_width, gate_depth, rail_height)),
        origin=Origin(xyz=(rail_center_x, gate_y_offset, rail_height * 0.5)),
        material=painted_wood,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((rail_width, gate_depth, rail_height)),
        origin=Origin(xyz=(rail_center_x, gate_y_offset, gate_height - rail_height * 0.5)),
        material=painted_wood,
        name="top_rail",
    )
    for hinge_name, hinge_z in (("lower", lower_hinge_z), ("upper", upper_hinge_z)):
        gate_leaf.visual(
            Cylinder(radius=0.009, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=galvanized,
            name=f"{hinge_name}_barrel",
        )
        gate_leaf.visual(
            Box((0.16, 0.012, 0.08)),
            origin=Origin(xyz=(0.08, 0.006, hinge_z)),
            material=steel,
            name=f"{hinge_name}_strap",
        )

    picket_width = 0.05
    picket_depth = 0.018
    picket_height = 1.14
    picket_centers = [0.14 + 0.115 * index for index in range(7)]
    for index, picket_x in enumerate(picket_centers, start=1):
        gate_leaf.visual(
            Box((picket_width, picket_depth, picket_height)),
            origin=Origin(xyz=(picket_x, gate_y_offset, 0.60)),
            material=cedar,
            name=f"picket_{index}",
        )

    gate_leaf.visual(
        Box((0.04, 0.006, 0.12)),
        origin=Origin(xyz=(0.889, -0.003, 0.80)),
        material=steel,
        name="latch_housing_backplate",
    )
    gate_leaf.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(0.87, 0.008, 0.83)),
        material=steel,
        name="latch_housing_left_ear",
    )
    gate_leaf.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(0.908, 0.008, 0.83)),
        material=steel,
        name="latch_housing_right_ear",
    )
    gate_leaf.visual(
        Cylinder(radius=0.003, length=0.034),
        origin=Origin(xyz=(0.889, 0.022, 0.83), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="thumb_pin",
    )
    gate_leaf.visual(
        Box((0.008, 0.012, 0.018)),
        origin=Origin(xyz=(0.948, gate_y_offset, 0.76)),
        material=steel,
        name="latch_bolt",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((0.95, 0.06, gate_height)),
        mass=16.0,
        origin=Origin(xyz=(0.475, gate_y_offset, gate_height * 0.5)),
    )

    thumb_piece = model.part("thumb_piece")
    thumb_piece.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="thumb_knuckle",
    )
    thumb_piece.visual(
        Box((0.01, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=steel,
        name="thumb_lever",
    )
    thumb_piece.visual(
        Box((0.022, 0.01, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.0345)),
        material=steel,
        name="thumb_pad",
    )
    thumb_piece.inertial = Inertial.from_geometry(
        Box((0.022, 0.012, 0.047)),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    model.articulation(
        "footing_to_hinge_post",
        ArticulationType.FIXED,
        parent=footing,
        child=hinge_post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "footing_to_latch_post",
        ArticulationType.FIXED,
        parent=footing,
        child=latch_post,
        origin=Origin(xyz=(right_post_x, 0.0, 0.0)),
    )
    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_post,
        child=gate_leaf,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, gate_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-radians(110.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "thumb_hinge",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=thumb_piece,
        origin=Origin(xyz=(0.889, 0.022, 0.83)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    footing = object_model.get_part("footing")
    hinge_post = object_model.get_part("hinge_post")
    latch_post = object_model.get_part("latch_post")
    gate_leaf = object_model.get_part("gate_leaf")
    thumb_piece = object_model.get_part("thumb_piece")

    gate_hinge = object_model.get_articulation("gate_hinge")
    thumb_hinge = object_model.get_articulation("thumb_hinge")

    footing_block = footing.get_visual("footing_block")
    hinge_post_shaft = hinge_post.get_visual("post_shaft")
    latch_post_shaft = latch_post.get_visual("post_shaft")
    lower_knuckle = hinge_post.get_visual("lower_hinge_knuckle")
    upper_knuckle = hinge_post.get_visual("upper_hinge_knuckle")
    strike_plate = latch_post.get_visual("strike_plate")
    hinge_stile = gate_leaf.get_visual("hinge_stile")
    latch_stile = gate_leaf.get_visual("latch_stile")
    bottom_rail = gate_leaf.get_visual("bottom_rail")
    lower_barrel = gate_leaf.get_visual("lower_barrel")
    upper_barrel = gate_leaf.get_visual("upper_barrel")
    latch_bolt = gate_leaf.get_visual("latch_bolt")
    thumb_pin = gate_leaf.get_visual("thumb_pin")
    thumb_knuckle = thumb_piece.get_visual("thumb_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        gate_leaf,
        hinge_post,
        elem_a=lower_barrel,
        elem_b=lower_knuckle,
        reason="Lower hinge barrel wraps the post-side lower knuckle at the pivot.",
    )
    ctx.allow_overlap(
        gate_leaf,
        hinge_post,
        elem_a=upper_barrel,
        elem_b=upper_knuckle,
        reason="Upper hinge barrel wraps the post-side upper knuckle at the pivot.",
    )
    ctx.allow_overlap(
        thumb_piece,
        gate_leaf,
        elem_a=thumb_knuckle,
        elem_b=thumb_pin,
        reason="Thumb latch pivots on the transverse pin carried by the housing ears.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=True)

    ctx.expect_gap(
        hinge_post,
        footing,
        axis="z",
        positive_elem=hinge_post_shaft,
        negative_elem=footing_block,
        max_gap=0.0,
        max_penetration=0.0,
        name="hinge_post_seated_on_footing",
    )
    ctx.expect_gap(
        latch_post,
        footing,
        axis="z",
        positive_elem=latch_post_shaft,
        negative_elem=footing_block,
        max_gap=0.0,
        max_penetration=0.0,
        name="latch_post_seated_on_footing",
    )
    ctx.expect_gap(
        gate_leaf,
        footing,
        axis="z",
        positive_elem=bottom_rail,
        negative_elem=footing_block,
        min_gap=0.08,
        max_gap=0.08,
        name="gate_bottom_clearance",
    )
    ctx.expect_gap(
        gate_leaf,
        hinge_post,
        axis="x",
        positive_elem=hinge_stile,
        negative_elem=hinge_post_shaft,
        min_gap=0.04,
        max_gap=0.05,
        name="hinge_stile_clears_post_body",
    )
    ctx.expect_overlap(
        gate_leaf,
        hinge_post,
        axes="yz",
        elem_a=lower_barrel,
        elem_b=lower_knuckle,
        min_overlap=0.008,
        name="lower_hinge_barrel_aligned",
    )
    ctx.expect_overlap(
        gate_leaf,
        hinge_post,
        axes="yz",
        elem_a=upper_barrel,
        elem_b=upper_knuckle,
        min_overlap=0.008,
        name="upper_hinge_barrel_aligned",
    )
    ctx.expect_overlap(
        thumb_piece,
        gate_leaf,
        axes="yz",
        elem_a=thumb_knuckle,
        elem_b=thumb_pin,
        min_overlap=0.005,
        name="thumb_knuckle_aligned_to_pin",
    )

    with ctx.pose({gate_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="gate_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="gate_closed_no_floating")
        ctx.expect_gap(
            latch_post,
            gate_leaf,
            axis="x",
            positive_elem=strike_plate,
            negative_elem=latch_bolt,
            max_gap=0.002,
            max_penetration=0.0,
            name="latch_gap_to_strike",
        )
        ctx.expect_overlap(
            gate_leaf,
            latch_post,
            axes="yz",
            elem_a=latch_bolt,
            elem_b=strike_plate,
            min_overlap=0.01,
            name="latch_bolt_aligned_with_strike_plate",
        )
        ctx.expect_gap(
            latch_post,
            gate_leaf,
            axis="x",
            positive_elem=latch_post_shaft,
            negative_elem=latch_stile,
            min_gap=0.012,
            max_gap=0.016,
            name="main_leaf_clears_latch_post",
        )

    gate_limits = gate_hinge.motion_limits
    if gate_limits is not None and gate_limits.lower is not None and gate_limits.upper is not None:
        closed_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
        with ctx.pose({gate_hinge: gate_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="gate_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="gate_hinge_lower_no_floating")
            open_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
            if closed_latch_aabb is not None and open_latch_aabb is not None:
                ctx.check(
                    "gate_leaf_swings_open",
                    open_latch_aabb[1][1] < closed_latch_aabb[0][1] - 0.75,
                    details=(
                        f"Expected open gate latch stile to move far into -Y; "
                        f"closed min y={closed_latch_aabb[0][1]:.3f}, "
                        f"open max y={open_latch_aabb[1][1]:.3f}"
                    ),
                )
        with ctx.pose({gate_hinge: gate_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="gate_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="gate_hinge_upper_no_floating")

    thumb_limits = thumb_hinge.motion_limits
    if thumb_limits is not None and thumb_limits.lower is not None and thumb_limits.upper is not None:
        rest_thumb_aabb = ctx.part_element_world_aabb(thumb_piece, elem="thumb_pad")
        with ctx.pose({thumb_hinge: thumb_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="thumb_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="thumb_hinge_lower_no_floating")
            lower_thumb_aabb = ctx.part_element_world_aabb(thumb_piece, elem="thumb_pad")
            if rest_thumb_aabb is not None and lower_thumb_aabb is not None:
                ctx.check(
                    "thumb_piece_rotates_about_pin",
                    lower_thumb_aabb[0][1] < rest_thumb_aabb[0][1] - 0.015,
                    details=(
                        f"Expected thumb pad to move into -Y when depressed; "
                        f"rest min y={rest_thumb_aabb[0][1]:.3f}, "
                        f"pressed min y={lower_thumb_aabb[0][1]:.3f}"
                    ),
                )
        with ctx.pose({thumb_hinge: thumb_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="thumb_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="thumb_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
