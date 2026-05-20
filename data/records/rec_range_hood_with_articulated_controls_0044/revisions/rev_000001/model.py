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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.83, 0.85, 0.87, 1.0))
    trim = model.material("painted_trim", rgba=(0.72, 0.74, 0.76, 1.0))
    filter_mat = model.material("charcoal_filter", rgba=(0.30, 0.31, 0.33, 1.0))
    knob_mat = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    marker_mat = model.material("knob_marker", rgba=(0.86, 0.82, 0.70, 1.0))

    hood_body = model.part("hood_body")

    width = 0.76
    depth = 0.50
    height = 0.12
    shell_t = 0.008
    fascia_depth = 0.032
    fascia_height = 0.070
    filter_t = 0.006

    hood_body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2.0)),
        material=stainless,
        name="top_panel",
    )
    hood_body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(-(width - shell_t) / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="left_side",
    )
    hood_body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=((width - shell_t) / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="right_side",
    )
    hood_body.visual(
        Box((width - 2.0 * shell_t, shell_t, height)),
        origin=Origin(xyz=(0.0, (depth - shell_t) / 2.0, height / 2.0)),
        material=stainless,
        name="back_panel",
    )
    hood_body.visual(
        Box((width - 2.0 * shell_t, fascia_depth, fascia_height)),
        origin=Origin(
            xyz=(0.0, -(depth - fascia_depth) / 2.0, height - fascia_height / 2.0)
        ),
        material=trim,
        name="front_fascia",
    )
    hood_body.visual(
        Box((width - 2.0 * shell_t + 0.004, depth - 2.0 * shell_t + 0.004, filter_t)),
        origin=Origin(xyz=(0.0, 0.0, filter_t / 2.0)),
        material=filter_mat,
        name="filter_panel",
    )

    knob_radius = 0.017
    knob_length = 0.028
    knob_row_y = -(depth / 2.0) - knob_length / 2.0
    knob_row_z = 0.082

    def add_knob(part_name: str, joint_name: str, x_pos: float) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=knob_radius, length=knob_length),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.019, length=0.006),
            origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="knob_bezel",
        )
        knob.visual(
            Box((0.003, 0.002, 0.012)),
            origin=Origin(xyz=(0.011, -0.013, 0.0)),
            material=marker_mat,
            name="indicator_mark",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, knob_row_y, knob_row_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=10.0),
        )

    add_knob("left_knob", "left_knob_spin", -0.115)
    add_knob("center_knob", "center_knob_spin", 0.0)
    add_knob("right_knob", "right_knob_spin", 0.115)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    hood_body = object_model.get_part("hood_body")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")

    left_joint = object_model.get_articulation("left_knob_spin")
    center_joint = object_model.get_articulation("center_knob_spin")
    right_joint = object_model.get_articulation("right_knob_spin")

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

    ctx.check(
        "expected_part_set",
        {part.name for part in object_model.parts}
        == {"hood_body", "left_knob", "center_knob", "right_knob"},
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "only_three_knob_articulations",
        len(object_model.articulations) == 3,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    body_aabb = ctx.part_world_aabb(hood_body)
    if body_aabb is None:
        ctx.fail("hood_body_aabb", "hood body has no measurable geometry")
    else:
        width = body_aabb[1][0] - body_aabb[0][0]
        depth = body_aabb[1][1] - body_aabb[0][1]
        height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "hood_body_realistic_size",
            0.72 <= width <= 0.80 and 0.46 <= depth <= 0.52 and 0.10 <= height <= 0.14,
            details=f"width={width:.4f}, depth={depth:.4f}, height={height:.4f}",
        )

    for knob, joint in (
        (left_knob, left_joint),
        (center_knob, center_joint),
        (right_knob, right_joint),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis_front_to_back",
            tuple(float(value) for value in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name}_has_unbounded_limits",
            limits is not None
            and limits.lower is None
            and limits.upper is None
            and limits.effort > 0.0
            and limits.velocity > 0.0,
            details=(
                "expected positive effort/velocity and no lower/upper bounds; "
                f"limits={limits}"
            ),
        )
        ctx.expect_contact(
            knob,
            hood_body,
            contact_tol=1e-6,
            name=f"{knob.name}_mounted_to_fascia",
        )
        ctx.expect_gap(
            hood_body,
            knob,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"{knob.name}_flush_to_front_face",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            min_overlap=0.02,
            name=f"{knob.name}_overlaps_front_lip_area",
        )

    left_pos = ctx.part_world_position(left_knob)
    center_pos = ctx.part_world_position(center_knob)
    right_pos = ctx.part_world_position(right_knob)
    if left_pos is None or center_pos is None or right_pos is None:
        ctx.fail("knob_positions_available", "one or more knob world positions are unavailable")
    else:
        left_to_center = center_pos[0] - left_pos[0]
        center_to_right = right_pos[0] - center_pos[0]
        ctx.check(
            "knobs_evenly_spaced",
            abs(left_to_center - center_to_right) <= 0.002
            and 0.10 <= left_to_center <= 0.13,
            details=(
                f"left_to_center={left_to_center:.4f}, "
                f"center_to_right={center_to_right:.4f}"
            ),
        )
        ctx.check(
            "knob_row_centered_on_hood",
            abs(center_pos[0]) <= 0.002 and abs((left_pos[0] + right_pos[0]) / 2.0) <= 0.002,
            details=f"left={left_pos}, center={center_pos}, right={right_pos}",
        )
        ctx.check(
            "knob_row_is_level",
            abs(left_pos[2] - center_pos[2]) <= 0.001
            and abs(center_pos[2] - right_pos[2]) <= 0.001
            and abs(left_pos[1] - center_pos[1]) <= 0.001
            and abs(center_pos[1] - right_pos[1]) <= 0.001,
            details=f"left={left_pos}, center={center_pos}, right={right_pos}",
        )

    with ctx.pose(
        {
            left_joint: 0.5 * pi,
            center_joint: pi,
            right_joint: 1.5 * pi,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="posed_knobs_no_floating")
        for knob in (left_knob, center_knob, right_knob):
            ctx.expect_contact(
                knob,
                hood_body,
                contact_tol=1e-6,
                name=f"{knob.name}_mounted_in_rotated_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
