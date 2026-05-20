from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_wall_hood")

    brushed_steel = model.material(
        "brushed_steel",
        rgba=(0.77, 0.79, 0.80, 1.0),
    )
    satin_steel = model.material(
        "satin_steel",
        rgba=(0.68, 0.70, 0.72, 1.0),
    )
    dark_filter = model.material(
        "dark_filter",
        rgba=(0.27, 0.29, 0.31, 1.0),
    )
    knob_black = model.material(
        "knob_black",
        rgba=(0.11, 0.11, 0.12, 1.0),
    )

    hood_width = 1.80
    hood_depth = 1.00
    hood_height = 0.55
    shell_t = 0.025

    panel_width = 1.62
    panel_depth = 0.86
    panel_frame_t = 0.03
    rail_w = 0.045
    divider_w = 0.038
    panel_closed_angle = 0.48
    hinge_y = 0.14
    hinge_z = 0.46

    opening_width = (panel_width - 2.0 * rail_w - 2.0 * divider_w) / 3.0
    opening_depth = panel_depth - 2.0 * rail_w
    filter_frame_w = 0.018
    filter_thickness = 0.028
    filter_width = opening_width - 0.010
    filter_depth = opening_depth - 0.020

    body = model.part("canopy_body")
    body.visual(
        Box((hood_width, shell_t, hood_height)),
        origin=Origin(xyz=(0.0, shell_t / 2.0, hood_height / 2.0)),
        material=brushed_steel,
        name="body_back_panel",
    )
    body.visual(
        Box((hood_width, hood_depth, shell_t)),
        origin=Origin(xyz=(0.0, hood_depth / 2.0, hood_height - shell_t / 2.0)),
        material=brushed_steel,
        name="body_top_panel",
    )
    body.visual(
        Box((shell_t, hood_depth, hood_height)),
        origin=Origin(
            xyz=(-(hood_width / 2.0) + shell_t / 2.0, hood_depth / 2.0, hood_height / 2.0)
        ),
        material=brushed_steel,
        name="body_left_side",
    )
    body.visual(
        Box((shell_t, hood_depth, hood_height)),
        origin=Origin(
            xyz=((hood_width / 2.0) - shell_t / 2.0, hood_depth / 2.0, hood_height / 2.0)
        ),
        material=brushed_steel,
        name="body_right_side",
    )
    body.visual(
        Box((hood_width, shell_t, 0.18)),
        origin=Origin(xyz=(0.0, hood_depth - shell_t / 2.0, hood_height - 0.09)),
        material=brushed_steel,
        name="body_front_fascia",
    )
    body.visual(
        Box((hood_width, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, hood_depth - 0.03, 0.025)),
        material=satin_steel,
        name="body_grease_trough",
    )
    body.visual(
        Cylinder(radius=0.008, length=panel_width - 0.12),
        origin=Origin(
            xyz=(
                0.0,
                hinge_y + 0.023 * sin(panel_closed_angle),
                hinge_z + 0.023 * cos(panel_closed_angle),
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=satin_steel,
        name="body_hinge_pin",
    )
    body.visual(
        Box((panel_width - 0.16, 0.06, 0.038)),
        origin=Origin(xyz=(0.0, 0.13, 0.506)),
        material=satin_steel,
        name="body_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=0.14, length=0.16),
        origin=Origin(xyz=(0.0, 0.24, hood_height + 0.08)),
        material=satin_steel,
        name="body_exhaust_collar",
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((panel_width, rail_w, panel_frame_t)),
        origin=Origin(xyz=(0.0, rail_w / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_top_rail",
    )
    access_panel.visual(
        Box((panel_width, rail_w, panel_frame_t)),
        origin=Origin(xyz=(0.0, panel_depth - rail_w / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_front_rail",
    )
    access_panel.visual(
        Box((rail_w, panel_depth, panel_frame_t)),
        origin=Origin(xyz=(-(panel_width / 2.0) + rail_w / 2.0, panel_depth / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_left_rail",
    )
    access_panel.visual(
        Box((rail_w, panel_depth, panel_frame_t)),
        origin=Origin(xyz=((panel_width / 2.0) - rail_w / 2.0, panel_depth / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_right_rail",
    )
    divider_offset = (opening_width / 2.0) + (divider_w / 2.0)
    access_panel.visual(
        Box((divider_w, opening_depth, panel_frame_t)),
        origin=Origin(xyz=(-divider_offset, panel_depth / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_divider_left",
    )
    access_panel.visual(
        Box((divider_w, opening_depth, panel_frame_t)),
        origin=Origin(xyz=(divider_offset, panel_depth / 2.0, 0.0)),
        material=brushed_steel,
        name="panel_divider_right",
    )
    access_panel.visual(
        Box((panel_width * 0.60, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, panel_depth - 0.012, -0.020)),
        material=satin_steel,
        name="panel_pull_lip",
    )

    model.articulation(
        "body_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_panel,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(-panel_closed_angle, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.0,
        ),
    )

    filter_names = ("left", "center", "right")
    filter_centers = (
        -(panel_width / 2.0) + rail_w + opening_width / 2.0,
        0.0,
        (panel_width / 2.0) - rail_w - opening_width / 2.0,
    )

    for index, (label, x_center) in enumerate(zip(filter_names, filter_centers), start=1):
        filter_part = model.part(f"baffle_filter_{label}")
        filter_part.visual(
            Box((filter_width, filter_frame_w, filter_thickness)),
            origin=Origin(xyz=(0.0, (filter_depth / 2.0) - (filter_frame_w / 2.0), 0.0)),
            material=dark_filter,
            name="filter_top_rail",
        )
        filter_part.visual(
            Box((filter_width, filter_frame_w, filter_thickness)),
            origin=Origin(xyz=(0.0, -(filter_depth / 2.0) + (filter_frame_w / 2.0), 0.0)),
            material=dark_filter,
            name="filter_bottom_rail",
        )
        filter_part.visual(
            Box((filter_frame_w, filter_depth, filter_thickness)),
            origin=Origin(xyz=(-(filter_width / 2.0) + (filter_frame_w / 2.0), 0.0, 0.0)),
            material=dark_filter,
            name="filter_left_rail",
        )
        filter_part.visual(
            Box((filter_frame_w, filter_depth, filter_thickness)),
            origin=Origin(xyz=((filter_width / 2.0) - (filter_frame_w / 2.0), 0.0, 0.0)),
            material=dark_filter,
            name="filter_right_rail",
        )
        slat_y_positions = (-0.22, -0.12, -0.02, 0.08, 0.18, 0.28)
        for slat_idx, slat_y in enumerate(slat_y_positions, start=1):
            filter_part.visual(
                Box((filter_width - 0.010, 0.040, 0.008)),
                origin=Origin(xyz=(0.0, slat_y, 0.0), rpy=(0.58, 0.0, 0.0)),
                material=satin_steel,
                name=f"filter_slat_{slat_idx}",
            )
        filter_part.visual(
            Box((0.10, 0.018, 0.020)),
            origin=Origin(xyz=(0.0, -(filter_depth / 2.0) + 0.010, -0.018)),
            material=satin_steel,
            name="filter_pull_tab",
        )

        model.articulation(
            f"panel_to_filter_{label}",
            ArticulationType.FIXED,
            parent=access_panel,
            child=filter_part,
            origin=Origin(
                xyz=(
                    x_center,
                    rail_w + opening_depth / 2.0 + 0.010,
                    -0.012,
                )
            ),
        )

    knob_x_positions = (-0.42, -0.14, 0.14, 0.42)
    knob_z = hood_height - 0.095
    for idx, x_pos in enumerate(knob_x_positions, start=1):
        knob = model.part(f"control_knob_{idx}")
        knob.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name="knob_shaft",
        )
        knob.visual(
            Cylinder(radius=0.032, length=0.010),
            origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_collar",
        )
        knob.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name="knob_trim_ring",
        )
        knob.visual(
            Box((0.007, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.030, 0.019)),
            material=satin_steel,
            name="knob_indicator",
        )

        model.articulation(
            f"body_to_knob_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, hood_depth, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=4.0),
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
    body = object_model.get_part("canopy_body")
    access_panel = object_model.get_part("access_panel")
    panel_hinge = object_model.get_articulation("body_to_access_panel")
    filter_left = object_model.get_part("baffle_filter_left")
    filter_center = object_model.get_part("baffle_filter_center")
    filter_right = object_model.get_part("baffle_filter_right")
    knob_1 = object_model.get_part("control_knob_1")
    knob_2 = object_model.get_part("control_knob_2")
    knob_3 = object_model.get_part("control_knob_3")
    knob_4 = object_model.get_part("control_knob_4")
    knob_joint_1 = object_model.get_articulation("body_to_knob_1")
    knob_joint_2 = object_model.get_articulation("body_to_knob_2")
    knob_joint_3 = object_model.get_articulation("body_to_knob_3")
    knob_joint_4 = object_model.get_articulation("body_to_knob_4")

    with ctx.pose({panel_hinge: 0.0}):
        ctx.expect_gap(
            body,
            access_panel,
            axis="y",
            positive_elem="body_grease_trough",
            negative_elem="panel_front_rail",
            min_gap=0.03,
            max_gap=0.09,
            name="closed panel sits just behind the grease trough",
        )
        ctx.expect_gap(
            access_panel,
            body,
            axis="z",
            positive_elem="panel_front_rail",
            negative_elem="body_grease_trough",
            min_gap=-0.001,
            max_gap=0.03,
            name="closed panel front rail stays just above the trough lip",
        )
        for filter_part, label in (
            (filter_left, "left"),
            (filter_center, "center"),
            (filter_right, "right"),
        ):
            ctx.expect_contact(
                filter_part,
                access_panel,
                contact_tol=1e-6,
                name=f"{label} baffle filter seats in the access frame",
            )
        for knob_part, idx in (
            (knob_1, 1),
            (knob_2, 2),
            (knob_3, 3),
            (knob_4, 4),
        ):
            ctx.expect_contact(
                knob_part,
                body,
                elem_a="knob_collar",
                elem_b="body_front_fascia",
                contact_tol=1e-6,
                name=f"knob {idx} collar bears on the front fascia",
            )

    closed_panel_front = ctx.part_element_world_aabb(access_panel, elem="panel_front_rail")
    closed_filter_left = ctx.part_world_aabb(filter_left)
    closed_indicator = ctx.part_element_world_aabb(knob_1, elem="knob_indicator")
    with ctx.pose({panel_hinge: 0.92}):
        open_panel_front = ctx.part_element_world_aabb(access_panel, elem="panel_front_rail")
        open_filter_left = ctx.part_world_aabb(filter_left)

    with ctx.pose({knob_joint_1: pi / 2.0}):
        rotated_indicator = ctx.part_element_world_aabb(knob_1, elem="knob_indicator")

    if closed_panel_front is not None and open_panel_front is not None:
        closed_panel_front_center_z = (closed_panel_front[0][2] + closed_panel_front[1][2]) / 2.0
        open_panel_front_center_z = (open_panel_front[0][2] + open_panel_front[1][2]) / 2.0
        ctx.check(
            "access panel swings downward when opened",
            open_panel_front_center_z < closed_panel_front_center_z - 0.30,
            details=(
                f"closed_front_z={closed_panel_front_center_z:.4f}, "
                f"open_front_z={open_panel_front_center_z:.4f}"
            ),
        )

    if closed_filter_left is not None and open_filter_left is not None:
        closed_filter_center_z = (closed_filter_left[0][2] + closed_filter_left[1][2]) / 2.0
        open_filter_center_z = (open_filter_left[0][2] + open_filter_left[1][2]) / 2.0
        ctx.check(
            "filters are carried with the hinged access panel",
            open_filter_center_z < closed_filter_center_z - 0.20,
            details=(
                f"closed_filter_z={closed_filter_center_z:.4f}, "
                f"open_filter_z={open_filter_center_z:.4f}"
            ),
        )

    if closed_indicator is not None and rotated_indicator is not None:
        closed_indicator_x = (closed_indicator[0][0] + closed_indicator[1][0]) / 2.0
        closed_indicator_z = (closed_indicator[0][2] + closed_indicator[1][2]) / 2.0
        rotated_indicator_x = (rotated_indicator[0][0] + rotated_indicator[1][0]) / 2.0
        rotated_indicator_z = (rotated_indicator[0][2] + rotated_indicator[1][2]) / 2.0
        ctx.check(
            "control knob indicator rotates around the knob shaft",
            abs(rotated_indicator_x - closed_indicator_x) > 0.010
            and abs(rotated_indicator_z - closed_indicator_z) > 0.010,
            details=(
                f"closed_indicator=({closed_indicator_x:.4f}, {closed_indicator_z:.4f}), "
                f"rotated_indicator=({rotated_indicator_x:.4f}, {rotated_indicator_z:.4f})"
            ),
        )

    for knob_joint, idx in (
        (knob_joint_1, 1),
        (knob_joint_2, 2),
        (knob_joint_3, 3),
        (knob_joint_4, 4),
    ):
        limits = knob_joint.motion_limits
        ctx.check(
            f"knob {idx} uses a continuous rotation articulation",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(knob_joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
                f"lower={None if limits is None else limits.lower}, "
                f"upper={None if limits is None else limits.upper}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
