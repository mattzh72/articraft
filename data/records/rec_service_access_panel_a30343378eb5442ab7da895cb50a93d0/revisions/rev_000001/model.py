from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="service_access_panel")

    body_paint = model.material("body_paint", rgba=(0.78, 0.79, 0.80, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.58, 0.60, 0.62, 1.0))
    door_paint = model.material("door_paint", rgba=(0.69, 0.71, 0.73, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    body_width = 0.80
    body_height = 1.00
    body_depth = 0.16
    skin_thickness = 0.006

    pocket_width = 0.53
    pocket_height = 0.79
    pocket_depth = 0.022

    frame_width = 0.055
    opening_width = pocket_width - 2.0 * frame_width
    opening_height = pocket_height - 2.0 * frame_width
    opening_depth = 0.050

    door_gap = 0.004
    door_width = opening_width - 2.0 * door_gap
    door_height = opening_height - 2.0 * door_gap
    hinge_radius = 0.0045
    door_thickness = 0.010
    door_plane_y = -pocket_depth + skin_thickness * 0.5
    hinge_side_gap = door_gap
    hinge_outboard_clearance = 0.002
    hinge_axis_x = -(opening_width * 0.5 + frame_width + hinge_radius + hinge_outboard_clearance)
    axis_to_leaf_center = -hinge_axis_x
    hinge_leaf_thickness = 0.003
    hinge_leaf_width = axis_to_leaf_center - door_width * 0.5 + 0.006
    frame_left_outer_x = -opening_width * 0.5 - frame_width
    hinge_mount_width = frame_left_outer_x - hinge_axis_x + 0.008
    hinge_parent_len = 0.19
    hinge_child_len = door_height - 2.0 * hinge_parent_len

    body = model.part("equipment_body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, -body_depth * 0.5, 0.0)),
    )

    face_side_width = (body_width - pocket_width) * 0.5
    face_top_height = (body_height - pocket_height) * 0.5

    body.visual(
        Box((face_side_width, skin_thickness, body_height)),
        origin=Origin(xyz=(-(pocket_width + face_side_width) * 0.5, skin_thickness * 0.5, 0.0)),
        material=body_paint,
        name="face_left",
    )
    body.visual(
        Box((face_side_width, skin_thickness, body_height)),
        origin=Origin(xyz=((pocket_width + face_side_width) * 0.5, skin_thickness * 0.5, 0.0)),
        material=body_paint,
        name="face_right",
    )
    body.visual(
        Box((pocket_width, skin_thickness, face_top_height)),
        origin=Origin(xyz=(0.0, skin_thickness * 0.5, (pocket_height + face_top_height) * 0.5)),
        material=body_paint,
        name="face_top",
    )
    body.visual(
        Box((pocket_width, skin_thickness, face_top_height)),
        origin=Origin(xyz=(0.0, skin_thickness * 0.5, -(pocket_height + face_top_height) * 0.5)),
        material=body_paint,
        name="face_bottom",
    )

    body.visual(
        Box((skin_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + skin_thickness * 0.5, -body_depth * 0.5, 0.0)),
        material=body_paint,
        name="side_left",
    )
    body.visual(
        Box((skin_thickness, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - skin_thickness * 0.5, -body_depth * 0.5, 0.0)),
        material=body_paint,
        name="side_right",
    )
    body.visual(
        Box((body_width - 2.0 * skin_thickness, body_depth, skin_thickness)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5, body_height * 0.5 - skin_thickness * 0.5)),
        material=body_paint,
        name="shell_top",
    )
    body.visual(
        Box((body_width - 2.0 * skin_thickness, body_depth, skin_thickness)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5, -body_height * 0.5 + skin_thickness * 0.5)),
        material=body_paint,
        name="shell_bottom",
    )
    body.visual(
        Box((body_width - 2.0 * skin_thickness, skin_thickness, body_height - 2.0 * skin_thickness)),
        origin=Origin(xyz=(0.0, -body_depth + skin_thickness * 0.5, 0.0)),
        material=body_paint,
        name="back_panel",
    )

    body.visual(
        Box((skin_thickness, pocket_depth, pocket_height)),
        origin=Origin(xyz=(-pocket_width * 0.5 + skin_thickness * 0.5, -pocket_depth * 0.5, 0.0)),
        material=frame_paint,
        name="reveal_left",
    )
    body.visual(
        Box((skin_thickness, pocket_depth, pocket_height)),
        origin=Origin(xyz=(pocket_width * 0.5 - skin_thickness * 0.5, -pocket_depth * 0.5, 0.0)),
        material=frame_paint,
        name="reveal_right",
    )
    body.visual(
        Box((pocket_width - 2.0 * skin_thickness, pocket_depth, skin_thickness)),
        origin=Origin(xyz=(0.0, -pocket_depth * 0.5, pocket_height * 0.5 - skin_thickness * 0.5)),
        material=frame_paint,
        name="reveal_top",
    )
    body.visual(
        Box((pocket_width - 2.0 * skin_thickness, pocket_depth, skin_thickness)),
        origin=Origin(xyz=(0.0, -pocket_depth * 0.5, -pocket_height * 0.5 + skin_thickness * 0.5)),
        material=frame_paint,
        name="reveal_bottom",
    )

    body.visual(
        Box((frame_width, skin_thickness, pocket_height)),
        origin=Origin(xyz=(-(opening_width + frame_width) * 0.5, door_plane_y, 0.0)),
        material=frame_paint,
        name="frame_left",
    )
    body.visual(
        Box((frame_width, skin_thickness, pocket_height)),
        origin=Origin(xyz=((opening_width + frame_width) * 0.5, door_plane_y, 0.0)),
        material=frame_paint,
        name="frame_right",
    )
    body.visual(
        Box((opening_width, skin_thickness, frame_width)),
        origin=Origin(xyz=(0.0, door_plane_y, (opening_height + frame_width) * 0.5)),
        material=frame_paint,
        name="frame_top",
    )
    body.visual(
        Box((opening_width, skin_thickness, frame_width)),
        origin=Origin(xyz=(0.0, door_plane_y, -(opening_height + frame_width) * 0.5)),
        material=frame_paint,
        name="frame_bottom",
    )

    body.visual(
        Box((skin_thickness, opening_depth, opening_height)),
        origin=Origin(
            xyz=(-opening_width * 0.5 - skin_thickness * 0.5, -pocket_depth - opening_depth * 0.5, 0.0)
        ),
        material=cavity_dark,
        name="opening_wall_left",
    )
    body.visual(
        Box((skin_thickness, opening_depth, opening_height)),
        origin=Origin(
            xyz=(opening_width * 0.5 + skin_thickness * 0.5, -pocket_depth - opening_depth * 0.5, 0.0)
        ),
        material=cavity_dark,
        name="opening_wall_right",
    )
    body.visual(
        Box((opening_width, opening_depth, skin_thickness)),
        origin=Origin(
            xyz=(0.0, -pocket_depth - opening_depth * 0.5, opening_height * 0.5 + skin_thickness * 0.5)
        ),
        material=cavity_dark,
        name="opening_wall_top",
    )
    body.visual(
        Box((opening_width, opening_depth, skin_thickness)),
        origin=Origin(
            xyz=(0.0, -pocket_depth - opening_depth * 0.5, -opening_height * 0.5 - skin_thickness * 0.5)
        ),
        material=cavity_dark,
        name="opening_wall_bottom",
    )
    body.visual(
        Box((opening_width, skin_thickness, opening_height)),
        origin=Origin(xyz=(0.0, -pocket_depth - opening_depth + skin_thickness * 0.5, 0.0)),
        material=cavity_dark,
        name="cavity_back",
    )

    door = model.part("door")
    door.visual(
        Box((hinge_leaf_width, hinge_leaf_thickness, door_height * 0.88)),
        origin=Origin(xyz=(hinge_leaf_width * 0.5, 0.0, 0.0)),
        material=door_paint,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=hinge_child_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_paint,
        name="hinge_knuckle",
    )
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(axis_to_leaf_center, 0.0, 0.0)),
        material=door_paint,
        name="door_leaf",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=6.5,
        origin=Origin(xyz=(axis_to_leaf_center, 0.0, 0.0)),
    )

    body.visual(
        Box((hinge_mount_width, hinge_leaf_thickness, door_height * 0.90)),
        origin=Origin(xyz=(hinge_axis_x + hinge_mount_width * 0.5 - 0.004, door_plane_y, 0.0)),
        material=frame_paint,
        name="hinge_mount",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_parent_len),
        origin=Origin(xyz=(hinge_axis_x, door_plane_y, door_height * 0.5 - hinge_parent_len * 0.5)),
        material=frame_paint,
        name="hinge_knuckle_top",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_parent_len),
        origin=Origin(xyz=(hinge_axis_x, door_plane_y, -door_height * 0.5 + hinge_parent_len * 0.5)),
        material=frame_paint,
        name="hinge_knuckle_bottom",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_plane_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.95),
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

    body = object_model.get_part("equipment_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_leaf",
            negative_elem="frame_left",
            min_gap=0.003,
            max_gap=0.0055,
            name="door has left reveal gap in closed pose",
        )
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="frame_right",
            negative_elem="door_leaf",
            min_gap=0.003,
            max_gap=0.0055,
            name="door has right reveal gap in closed pose",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="frame_top",
            negative_elem="door_leaf",
            min_gap=0.003,
            max_gap=0.0055,
            name="door has top reveal gap in closed pose",
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem="door_leaf",
            negative_elem="frame_bottom",
            min_gap=0.003,
            max_gap=0.0055,
            name="door has bottom reveal gap in closed pose",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_leaf",
            elem_b="cavity_back",
            min_overlap=0.30,
            name="door remains centered over the service opening",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")

    closed_max_y = None if closed_aabb is None else closed_aabb[1][1]
    open_max_y = None if open_aabb is None else open_aabb[1][1]
    ctx.check(
        "door opens outward from the equipment face",
        closed_max_y is not None and open_max_y is not None and open_max_y > closed_max_y + 0.18,
        details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
