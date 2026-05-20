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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangle_panel_mesh(name: str, span: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (0.0, 0.0),
                (span, 0.0),
                (0.0, span),
            ],
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def _rotate_xy(x: float, y: float, yaw: float):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        (x * cos_yaw) - (y * sin_yaw),
        (x * sin_yaw) + (y * cos_yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_bifold_door")

    cabinet_span = 0.58
    cabinet_height = 0.90
    panel_thickness = 0.018
    door_thickness = 0.020
    door_side_gap = 0.004
    door_center_gap = 0.004
    door_top_gap = 0.015
    door_bottom_gap = 0.018
    door_height = cabinet_height - door_top_gap - door_bottom_gap
    opening_width = math.sqrt(2.0) * (cabinet_span - panel_thickness)
    leaf_width = (opening_width - (2.0 * door_side_gap) - door_center_gap) / 2.0
    door_closed_yaw = 3.0 * math.pi / 4.0
    hinge_leaf_width = door_side_gap / 2.0
    fold_leaf_width = door_center_gap / 2.0
    hinge_plate_depth = 0.012
    hinge_plate_offset_y = -(door_thickness + 0.006)
    hinge_plate_center_x = door_side_gap - (hinge_leaf_width / 2.0)
    cabinet_leaf_local_y = hinge_plate_offset_y - hinge_plate_depth
    cabinet_leaf_xy = _rotate_xy(
        hinge_plate_center_x,
        cabinet_leaf_local_y,
        door_closed_yaw,
    )
    hinge_support_start = (cabinet_span, panel_thickness)
    hinge_support_end = (
        cabinet_span + cabinet_leaf_xy[0],
        panel_thickness + cabinet_leaf_xy[1],
    )
    hinge_support_dx = hinge_support_end[0] - hinge_support_start[0]
    hinge_support_dy = hinge_support_end[1] - hinge_support_start[1]
    hinge_support_length = math.hypot(hinge_support_dx, hinge_support_dy)
    hinge_support_yaw = math.atan2(hinge_support_dy, hinge_support_dx)
    hinge_support_mid = (
        (hinge_support_start[0] + hinge_support_end[0]) / 2.0,
        (hinge_support_start[1] + hinge_support_end[1]) / 2.0,
    )

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.93, 0.90, 1.0))
    painted_front = model.material("painted_front", rgba=(0.82, 0.84, 0.79, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.62, 0.48, 0.33, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    interior_shadow = model.material("interior_shadow", rgba=(0.78, 0.76, 0.72, 1.0))

    top_panel_mesh = _triangle_panel_mesh("corner_cabinet_top_panel", cabinet_span, panel_thickness)
    bottom_panel_mesh = _triangle_panel_mesh(
        "corner_cabinet_bottom_panel",
        cabinet_span,
        panel_thickness,
    )
    shelf_mesh = _triangle_panel_mesh("corner_cabinet_shelf_panel", cabinet_span - 0.040, panel_thickness)

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        bottom_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_wood,
        name="bottom_panel",
    )
    cabinet_body.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - panel_thickness)),
        material=warm_wood,
        name="top_panel",
    )
    cabinet_body.visual(
        Box((panel_thickness, cabinet_span, cabinet_height - (2.0 * panel_thickness))),
        origin=Origin(
            xyz=(
                panel_thickness / 2.0,
                cabinet_span / 2.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet_body.visual(
        Box((cabinet_span, panel_thickness, cabinet_height - (2.0 * panel_thickness))),
        origin=Origin(
            xyz=(
                cabinet_span / 2.0,
                panel_thickness / 2.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet_body.visual(
        shelf_mesh,
        origin=Origin(xyz=(panel_thickness, panel_thickness, 0.44)),
        material=interior_shadow,
        name="fixed_shelf",
    )
    cabinet_body.visual(
        Box((hinge_leaf_width, hinge_plate_depth, 0.090)),
        origin=Origin(
            xyz=(
                cabinet_span + cabinet_leaf_xy[0],
                panel_thickness + cabinet_leaf_xy[1],
                door_bottom_gap + 0.12,
            ),
            rpy=(0.0, 0.0, door_closed_yaw),
        ),
        material=handle_metal,
        name="cabinet_hinge_leaf_lower",
    )
    cabinet_body.visual(
        Box((hinge_leaf_width, hinge_plate_depth, 0.090)),
        origin=Origin(
            xyz=(
                cabinet_span + cabinet_leaf_xy[0],
                panel_thickness + cabinet_leaf_xy[1],
                door_bottom_gap + door_height - 0.12,
            ),
            rpy=(0.0, 0.0, door_closed_yaw),
        ),
        material=handle_metal,
        name="cabinet_hinge_leaf_upper",
    )
    cabinet_body.visual(
        Cylinder(radius=0.0015, length=hinge_support_length),
        origin=Origin(
            xyz=(hinge_support_mid[0], hinge_support_mid[1], door_bottom_gap + 0.12),
            rpy=(0.0, math.pi / 2.0, hinge_support_yaw),
        ),
        material=handle_metal,
        name="cabinet_hinge_link_lower",
    )
    cabinet_body.visual(
        Cylinder(radius=0.0015, length=hinge_support_length),
        origin=Origin(
            xyz=(
                hinge_support_mid[0],
                hinge_support_mid[1],
                door_bottom_gap + door_height - 0.12,
            ),
            rpy=(0.0, math.pi / 2.0, hinge_support_yaw),
        ),
        material=handle_metal,
        name="cabinet_hinge_link_upper",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((cabinet_span, cabinet_span, cabinet_height)),
        mass=28.0,
        origin=Origin(
            xyz=(
                cabinet_span / 2.0,
                cabinet_span / 2.0,
                cabinet_height / 2.0,
            )
        ),
    )

    outer_door_leaf = model.part("outer_door_leaf")
    outer_door_leaf.visual(
        Box((leaf_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                door_side_gap + (leaf_width / 2.0),
                -(door_thickness / 2.0),
                door_height / 2.0,
            ),
        ),
        material=painted_front,
        name="outer_panel",
    )
    outer_door_leaf.visual(
        Box((hinge_leaf_width, hinge_plate_depth, 0.090)),
        origin=Origin(
            xyz=(
                door_side_gap - (hinge_leaf_width / 2.0),
                -(door_thickness + 0.006),
                0.12,
            ),
        ),
        material=handle_metal,
        name="outer_hinge_plate_lower",
    )
    outer_door_leaf.visual(
        Box((hinge_leaf_width, hinge_plate_depth, 0.090)),
        origin=Origin(
            xyz=(
                door_side_gap - (hinge_leaf_width / 2.0),
                -(door_thickness + 0.006),
                door_height - 0.12,
            ),
        ),
        material=handle_metal,
        name="outer_hinge_plate_upper",
    )
    outer_door_leaf.visual(
        Box((fold_leaf_width, hinge_plate_depth, door_height - 0.18)),
        origin=Origin(
            xyz=(
                door_side_gap + leaf_width + (fold_leaf_width / 2.0),
                -(door_thickness + 0.006),
                door_height / 2.0,
            ),
        ),
        material=handle_metal,
        name="center_knuckle_outer",
    )
    outer_door_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, door_thickness, door_height)),
        mass=5.5,
        origin=Origin(
            xyz=(
                door_side_gap + (leaf_width / 2.0),
                -(door_thickness / 2.0),
                door_height / 2.0,
            )
        ),
    )

    inner_door_leaf = model.part("inner_door_leaf")
    inner_door_leaf.visual(
        Box((leaf_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                leaf_width / 2.0,
                -(door_thickness / 2.0),
                door_height / 2.0,
            ),
        ),
        material=painted_front,
        name="inner_panel",
    )
    inner_door_leaf.visual(
        Box((0.020, 0.014, 0.48)),
        origin=Origin(
            xyz=(
                leaf_width - 0.050,
                -(door_thickness + 0.007),
                door_height / 2.0,
            ),
        ),
        material=handle_metal,
        name="pull_handle",
    )
    inner_door_leaf.visual(
        Box((fold_leaf_width, hinge_plate_depth, door_height - 0.18)),
        origin=Origin(
            xyz=(
                -(fold_leaf_width / 2.0),
                -(door_thickness + 0.006),
                door_height / 2.0,
            ),
        ),
        material=handle_metal,
        name="center_knuckle_inner",
    )
    inner_door_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, door_thickness, door_height)),
        mass=5.0,
        origin=Origin(
            xyz=(
                leaf_width / 2.0,
                -(door_thickness / 2.0),
                door_height / 2.0,
            )
        ),
    )

    model.articulation(
        "cabinet_to_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=outer_door_leaf,
        origin=Origin(
            xyz=(cabinet_span, panel_thickness, door_bottom_gap),
            rpy=(0.0, 0.0, door_closed_yaw),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=1.80,
        ),
    )
    model.articulation(
        "outer_leaf_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=outer_door_leaf,
        child=inner_door_leaf,
        origin=Origin(
            xyz=(
                door_side_gap + leaf_width + door_center_gap,
                0.0,
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.4,
            lower=0.0,
            upper=3.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_body = object_model.get_part("cabinet_body")
    outer_door_leaf = object_model.get_part("outer_door_leaf")
    inner_door_leaf = object_model.get_part("inner_door_leaf")
    cabinet_to_outer_leaf = object_model.get_articulation("cabinet_to_outer_leaf")
    outer_leaf_to_inner_leaf = object_model.get_articulation("outer_leaf_to_inner_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "outer hinge uses vertical axis",
        tuple(cabinet_to_outer_leaf.axis) == (0.0, 0.0, 1.0),
        details=f"Expected (0, 0, 1), got {cabinet_to_outer_leaf.axis!r}",
    )
    ctx.check(
        "center hinge uses vertical axis",
        tuple(outer_leaf_to_inner_leaf.axis) == (0.0, 0.0, 1.0),
        details=f"Expected (0, 0, 1), got {outer_leaf_to_inner_leaf.axis!r}",
    )

    cabinet_span = 0.58
    panel_thickness = 0.018
    door_bottom_gap = 0.018
    door_side_gap = 0.004
    door_center_gap = 0.004
    opening_width = math.sqrt(2.0) * (cabinet_span - panel_thickness)
    leaf_width = (opening_width - (2.0 * door_side_gap) - door_center_gap) / 2.0
    closed_yaw = 3.0 * math.pi / 4.0

    outer_origin = ctx.part_world_position(outer_door_leaf)
    inner_origin = ctx.part_world_position(inner_door_leaf)
    cabinet_origin = ctx.part_world_position(cabinet_body)
    expected_inner_xy = (
        cabinet_span + (math.cos(closed_yaw) * (door_side_gap + leaf_width + door_center_gap)),
        panel_thickness + (math.sin(closed_yaw) * (door_side_gap + leaf_width + door_center_gap)),
    )
    ctx.check(
        "cabinet body remains rooted at world origin",
        cabinet_origin == (0.0, 0.0, 0.0),
        details=f"Unexpected cabinet origin: {cabinet_origin!r}",
    )
    ctx.check(
        "outer leaf hinge sits at right front cabinet edge",
        outer_origin is not None
        and abs(outer_origin[0] - cabinet_span) < 1e-6
        and abs(outer_origin[1] - panel_thickness) < 1e-6
        and abs(outer_origin[2] - door_bottom_gap) < 1e-6,
        details=f"Unexpected outer leaf origin: {outer_origin!r}",
    )
    ctx.check(
        "inner leaf hinge lands on diagonal fold line when closed",
        inner_origin is not None
        and abs(inner_origin[0] - expected_inner_xy[0]) < 1e-6
        and abs(inner_origin[1] - expected_inner_xy[1]) < 1e-6
        and abs(inner_origin[2] - door_bottom_gap) < 1e-6,
        details=f"Unexpected inner leaf origin: {inner_origin!r}",
    )

    ctx.expect_contact(outer_door_leaf, cabinet_body)
    ctx.expect_contact(inner_door_leaf, outer_door_leaf)
    ctx.expect_overlap(outer_door_leaf, cabinet_body, axes="z", min_overlap=0.70)
    ctx.expect_overlap(inner_door_leaf, cabinet_body, axes="z", min_overlap=0.70)

    with ctx.pose({cabinet_to_outer_leaf: 1.20}):
        opened_inner_origin = ctx.part_world_position(inner_door_leaf)
        ctx.check(
            "outer hinge swings bifold assembly outward",
            opened_inner_origin is not None
            and opened_inner_origin[0] < inner_origin[0] - 0.05
            and opened_inner_origin[1] < inner_origin[1] - 0.15
            and opened_inner_origin[2] > 0.0,
            details=f"Unexpected opened inner hinge position: {opened_inner_origin!r}",
        )

    with ctx.pose({cabinet_to_outer_leaf: 1.10, outer_leaf_to_inner_leaf: 0.0}):
        outer_open_aabb = ctx.part_world_aabb(outer_door_leaf)
        inner_unfolded_aabb = ctx.part_world_aabb(inner_door_leaf)

    with ctx.pose({cabinet_to_outer_leaf: 1.10, outer_leaf_to_inner_leaf: 2.70}):
        outer_aabb = ctx.part_world_aabb(outer_door_leaf)
        inner_aabb = ctx.part_world_aabb(inner_door_leaf)
        outer_center_xy = (
            (outer_aabb[0][0] + outer_aabb[1][0]) / 2.0,
            (outer_aabb[0][1] + outer_aabb[1][1]) / 2.0,
        )
        inner_unfolded_center_xy = (
            (inner_unfolded_aabb[0][0] + inner_unfolded_aabb[1][0]) / 2.0,
            (inner_unfolded_aabb[0][1] + inner_unfolded_aabb[1][1]) / 2.0,
        )
        inner_folded_center_xy = (
            (inner_aabb[0][0] + inner_aabb[1][0]) / 2.0,
            (inner_aabb[0][1] + inner_aabb[1][1]) / 2.0,
        )
        unfolded_dist = math.hypot(
            inner_unfolded_center_xy[0] - outer_center_xy[0],
            inner_unfolded_center_xy[1] - outer_center_xy[1],
        )
        folded_dist = math.hypot(
            inner_folded_center_xy[0] - outer_center_xy[0],
            inner_folded_center_xy[1] - outer_center_xy[1],
        )
        ctx.check(
            "center hinge folds the inner leaf back beside the outer leaf",
            outer_open_aabb is not None
            and inner_unfolded_aabb is not None
            and outer_aabb is not None
            and inner_aabb is not None
            and folded_dist < unfolded_dist - 0.10
            and inner_folded_center_xy[1] < -0.05,
            details=f"Unexpected folded leaf bounds: outer={outer_aabb!r}, inner={inner_aabb!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
