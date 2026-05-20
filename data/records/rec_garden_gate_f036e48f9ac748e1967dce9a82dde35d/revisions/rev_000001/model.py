from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_yard_gate")

    galvanized = model.material("galvanized_steel", color=(0.66, 0.68, 0.70))
    rubber = model.material("wheel_rubber", color=(0.10, 0.10, 0.10))
    concrete = model.material("concrete_pad", color=(0.58, 0.58, 0.58))

    post_radius = 0.04
    post_height = 1.95
    hinge_sleeve_radius = 0.022
    hinge_pin_radius = 0.010
    hinge_axis_x = post_radius + hinge_sleeve_radius + 0.003

    tube_radius = 0.02
    left_stile_x = 0.10
    right_stile_x = 1.07
    bottom_rail_z = 0.26
    top_rail_z = 1.60
    mid_rail_zs = (0.74, 1.18)
    frame_center_x = 0.5 * (left_stile_x + right_stile_x)
    frame_span_x = right_stile_x - left_stile_x
    frame_center_z = 0.5 * (bottom_rail_z + top_rail_z)
    frame_span_z = top_rail_z - bottom_rail_z
    hinge_zs = (0.40, 0.93, 1.46)

    caster_pivot_z = bottom_rail_z - tube_radius - 0.012
    wheel_radius = 0.09
    caster_trail_y = 0.09
    wheel_axle_drop = 0.128

    post = model.part("post")
    post.visual(
        Cylinder(radius=post_radius, length=post_height),
        origin=Origin(xyz=(0.0, 0.0, post_height / 2.0)),
        material=galvanized,
        name="post_shaft",
    )
    post.visual(
        Box((0.18, 0.18, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=concrete,
        name="post_base_pad",
    )
    hinge_rod_bottom_z = hinge_zs[0] - 0.11
    hinge_rod_top_z = hinge_zs[2] + 0.11
    post.visual(
        Cylinder(radius=hinge_pin_radius, length=hinge_rod_top_z - hinge_rod_bottom_z),
        origin=Origin(
            xyz=(
                hinge_axis_x,
                0.0,
                0.5 * (hinge_rod_bottom_z + hinge_rod_top_z),
            )
        ),
        material=galvanized,
        name="post_hinge_rod",
    )
    post.visual(
        Box((0.022, 0.016, 0.028)),
        origin=Origin(xyz=(0.047, 0.0, hinge_rod_bottom_z + 0.014)),
        material=galvanized,
        name="post_hinge_brace_lower",
    )
    post.visual(
        Box((0.022, 0.016, 0.028)),
        origin=Origin(xyz=(0.047, 0.0, hinge_rod_top_z - 0.014)),
        material=galvanized,
        name="post_hinge_brace_upper",
    )

    leaf = model.part("leaf")
    leaf.visual(
        Cylinder(radius=tube_radius, length=frame_span_z),
        origin=Origin(xyz=(left_stile_x, 0.0, frame_center_z)),
        material=galvanized,
        name="left_stile",
    )
    leaf.visual(
        Cylinder(radius=tube_radius, length=frame_span_z),
        origin=Origin(xyz=(right_stile_x, 0.0, frame_center_z)),
        material=galvanized,
        name="right_stile",
    )
    leaf.visual(
        Cylinder(radius=tube_radius, length=frame_span_x),
        origin=Origin(xyz=(frame_center_x, 0.0, bottom_rail_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="bottom_rail",
    )
    leaf.visual(
        Cylinder(radius=tube_radius, length=frame_span_x),
        origin=Origin(xyz=(frame_center_x, 0.0, top_rail_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_rail",
    )
    leaf.visual(
        Cylinder(radius=0.016, length=frame_span_x - 0.04),
        origin=Origin(
            xyz=(frame_center_x, 0.0, mid_rail_zs[0]),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=galvanized,
        name="mid_rail_lower",
    )
    leaf.visual(
        Cylinder(radius=0.016, length=frame_span_x - 0.04),
        origin=Origin(
            xyz=(frame_center_x, 0.0, mid_rail_zs[1]),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=galvanized,
        name="mid_rail_upper",
    )

    leaf.visual(
        Cylinder(radius=hinge_sleeve_radius, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, hinge_zs[0])),
        material=galvanized,
        name="hinge_sleeve_lower",
    )
    leaf.visual(
        Box((left_stile_x - hinge_sleeve_radius, 0.012, 0.028)),
        origin=Origin(
            xyz=((left_stile_x + hinge_sleeve_radius) / 2.0, 0.0, hinge_zs[0])
        ),
        material=galvanized,
        name="hinge_strap_lower",
    )
    leaf.visual(
        Cylinder(radius=hinge_sleeve_radius, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, hinge_zs[1])),
        material=galvanized,
        name="hinge_sleeve_middle",
    )
    leaf.visual(
        Box((left_stile_x - hinge_sleeve_radius, 0.012, 0.028)),
        origin=Origin(
            xyz=((left_stile_x + hinge_sleeve_radius) / 2.0, 0.0, hinge_zs[1])
        ),
        material=galvanized,
        name="hinge_strap_middle",
    )
    leaf.visual(
        Cylinder(radius=hinge_sleeve_radius, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, hinge_zs[2])),
        material=galvanized,
        name="hinge_sleeve_upper",
    )
    leaf.visual(
        Box((left_stile_x - hinge_sleeve_radius, 0.012, 0.028)),
        origin=Origin(
            xyz=((left_stile_x + hinge_sleeve_radius) / 2.0, 0.0, hinge_zs[2])
        ),
        material=galvanized,
        name="hinge_strap_upper",
    )

    leaf.visual(
        Box((0.090, 0.050, 0.012)),
        origin=Origin(xyz=(right_stile_x - 0.010, 0.0, caster_pivot_z + 0.006)),
        material=galvanized,
        name="caster_mount",
    )

    fork = model.part("wheel_fork")
    fork.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=galvanized,
        name="swivel_stem",
    )
    fork.visual(
        Box((0.050, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=galvanized,
        name="fork_head",
    )
    fork.visual(
        Box((0.008, caster_trail_y, 0.008)),
        origin=Origin(xyz=(-0.022, caster_trail_y / 2.0, -0.028)),
        material=galvanized,
        name="fork_trail_left",
    )
    fork.visual(
        Box((0.008, caster_trail_y, 0.008)),
        origin=Origin(xyz=(0.022, caster_trail_y / 2.0, -0.028)),
        material=galvanized,
        name="fork_trail_right",
    )
    fork.visual(
        Box((0.060, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, caster_trail_y, -0.019)),
        material=galvanized,
        name="fork_crown",
    )
    fork.visual(
        Box((0.008, 0.014, wheel_axle_drop - 0.019)),
        origin=Origin(
            xyz=(-0.023, caster_trail_y, -0.019 - (wheel_axle_drop - 0.019) / 2.0)
        ),
        material=galvanized,
        name="fork_arm_left",
    )
    fork.visual(
        Box((0.008, 0.014, wheel_axle_drop - 0.019)),
        origin=Origin(
            xyz=(0.023, caster_trail_y, -0.019 - (wheel_axle_drop - 0.019) / 2.0)
        ),
        material=galvanized,
        name="fork_arm_right",
    )

    wheel = model.part("support_wheel")
    wheel.visual(
        Cylinder(radius=wheel_radius, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    wheel.visual(
        Cylinder(radius=0.035, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="wheel_hub",
    )
    wheel.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="axle_stub_left",
    )
    wheel.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="axle_stub_right",
    )

    model.articulation(
        "post_to_leaf",
        ArticulationType.REVOLUTE,
        parent=post,
        child=leaf,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "leaf_to_fork_swivel",
        ArticulationType.CONTINUOUS,
        parent=leaf,
        child=fork,
        origin=Origin(xyz=(right_stile_x, 0.0, caster_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, caster_trail_y, -wheel_axle_drop)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=25.0),
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

    post = object_model.get_part("post")
    leaf = object_model.get_part("leaf")
    fork = object_model.get_part("wheel_fork")
    wheel = object_model.get_part("support_wheel")
    hinge_sleeve_lower = leaf.get_visual("hinge_sleeve_lower")
    hinge_sleeve_middle = leaf.get_visual("hinge_sleeve_middle")
    hinge_sleeve_upper = leaf.get_visual("hinge_sleeve_upper")
    post_shaft = post.get_visual("post_shaft")
    post_hinge_rod = post.get_visual("post_hinge_rod")
    swivel_stem = fork.get_visual("swivel_stem")
    caster_mount = leaf.get_visual("caster_mount")
    axle_stub_right = wheel.get_visual("axle_stub_right")
    fork_arm_right = fork.get_visual("fork_arm_right")

    gate_hinge = object_model.get_articulation("post_to_leaf")
    caster_swivel = object_model.get_articulation("leaf_to_fork_swivel")
    wheel_spin = object_model.get_articulation("fork_to_wheel")

    ctx.allow_overlap(
        post,
        leaf,
        elem_a=post_hinge_rod,
        elem_b=hinge_sleeve_lower,
        reason="The fixed hinge rod intentionally passes through the lower welded hinge sleeve.",
    )
    ctx.allow_overlap(
        post,
        leaf,
        elem_a=post_hinge_rod,
        elem_b=hinge_sleeve_middle,
        reason="The fixed hinge rod intentionally passes through the middle welded hinge sleeve.",
    )
    ctx.allow_overlap(
        post,
        leaf,
        elem_a=post_hinge_rod,
        elem_b=hinge_sleeve_upper,
        reason="The fixed hinge rod intentionally passes through the upper welded hinge sleeve.",
    )

    ctx.expect_gap(
        leaf,
        post,
        axis="x",
        min_gap=0.0,
        max_gap=0.01,
        positive_elem=hinge_sleeve_lower,
        negative_elem=post_shaft,
        name="lower hinge sleeve clears the hinge post with a tight realistic gap",
    )
    ctx.expect_contact(
        fork,
        leaf,
        elem_a=swivel_stem,
        elem_b=caster_mount,
        name="caster fork is mounted under the free corner plate",
    )
    ctx.expect_contact(
        wheel,
        fork,
        elem_a=axle_stub_right,
        elem_b=fork_arm_right,
        name="support wheel axle is carried by the fork arm",
    )

    hinge_limits = gate_hinge.motion_limits
    ctx.check(
        "gate hinge uses an outward-swinging vertical revolute joint",
        gate_hinge.articulation_type == ArticulationType.REVOLUTE
        and gate_hinge.axis == (0.0, 0.0, 1.0)
        and hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and hinge_limits.upper >= 1.4,
        details=f"axis={gate_hinge.axis}, limits={hinge_limits}",
    )

    swivel_limits = caster_swivel.motion_limits
    ctx.check(
        "caster pivot swivels continuously about a vertical axis",
        caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and caster_swivel.axis == (0.0, 0.0, 1.0)
        and swivel_limits is not None
        and swivel_limits.lower is None
        and swivel_limits.upper is None,
        details=f"axis={caster_swivel.axis}, limits={swivel_limits}",
    )

    spin_limits = wheel_spin.motion_limits
    ctx.check(
        "support wheel spins continuously on its axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0)
        and spin_limits is not None
        and spin_limits.lower is None
        and spin_limits.upper is None,
        details=f"axis={wheel_spin.axis}, limits={spin_limits}",
    )

    closed_leaf_box = ctx.part_world_aabb(leaf)
    with ctx.pose({gate_hinge: 1.0}):
        open_leaf_box = ctx.part_world_aabb(leaf)
    ctx.check(
        "leaf swings away from the post when opened",
        closed_leaf_box is not None
        and open_leaf_box is not None
        and open_leaf_box[1][1] > closed_leaf_box[1][1] + 0.45,
        details=f"closed={closed_leaf_box}, open={open_leaf_box}",
    )

    with ctx.pose({gate_hinge: 0.0, caster_swivel: 0.0}):
        wheel_rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({gate_hinge: 0.0, caster_swivel: 1.1}):
        wheel_swiveled_pos = ctx.part_world_position(wheel)
    ctx.check(
        "caster swivel moves the trailing wheel around the vertical pivot",
        wheel_rest_pos is not None
        and wheel_swiveled_pos is not None
        and abs(wheel_swiveled_pos[0] - wheel_rest_pos[0]) > 0.03
        and abs(wheel_swiveled_pos[1] - wheel_rest_pos[1]) > 0.01,
        details=f"rest={wheel_rest_pos}, swiveled={wheel_swiveled_pos}",
    )

    with ctx.pose({gate_hinge: 0.0, caster_swivel: 0.0}):
        leaf_box = ctx.part_world_aabb(leaf)
        wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "support wheel sits under the free end of the gate leaf",
        leaf_box is not None
        and wheel_pos is not None
        and wheel_pos[0] > leaf_box[1][0] - 0.12
        and wheel_pos[2] < leaf_box[0][2] - 0.02,
        details=f"leaf={leaf_box}, wheel={wheel_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
