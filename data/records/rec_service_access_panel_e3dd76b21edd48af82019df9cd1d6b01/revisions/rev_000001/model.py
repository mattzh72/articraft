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
    model = ArticulatedObject(name="equipment_service_access_panel")

    body_paint = model.material("body_paint", rgba=(0.69, 0.71, 0.74, 1.0))
    door_paint = model.material("door_paint", rgba=(0.52, 0.54, 0.58, 1.0))
    hardware = model.material("hardware", rgba=(0.71, 0.73, 0.76, 1.0))
    gasket = model.material("gasket", rgba=(0.16, 0.17, 0.18, 1.0))

    body_width = 0.92
    body_height = 1.18
    body_depth = 0.38
    wall_thickness = 0.03
    back_thickness = 0.02
    front_face_thickness = 0.018

    opening_width = 0.42
    opening_height = 0.64
    opening_bottom = 0.39
    opening_top = opening_bottom + opening_height
    opening_left = -opening_width / 2.0
    opening_right = opening_width / 2.0
    opening_center_z = opening_bottom + opening_height / 2.0

    left_field_width = opening_left - (-body_width / 2.0)
    right_field_width = body_width / 2.0 - opening_right
    top_field_height = body_height - opening_top
    bottom_field_height = opening_bottom

    body = model.part("equipment_body")
    body.visual(
        Box((body_width - 2.0 * wall_thickness, back_thickness, body_height - 2.0 * wall_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -body_depth + back_thickness / 2.0,
                wall_thickness + (body_height - 2.0 * wall_thickness) / 2.0,
            )
        ),
        material=body_paint,
        name="back_panel",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + wall_thickness / 2.0, -body_depth / 2.0, body_height / 2.0)),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - wall_thickness / 2.0, -body_depth / 2.0, body_height / 2.0)),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, body_depth, wall_thickness)),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0, body_height - wall_thickness / 2.0),
        ),
        material=body_paint,
        name="top_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, wall_thickness / 2.0)),
        material=body_paint,
        name="bottom_wall",
    )

    body.visual(
        Box((left_field_width, front_face_thickness, opening_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + left_field_width / 2.0,
                -front_face_thickness / 2.0,
                opening_center_z,
            )
        ),
        material=body_paint,
        name="frame_left",
    )
    body.visual(
        Box((right_field_width, front_face_thickness, opening_height)),
        origin=Origin(
            xyz=(
                opening_right + right_field_width / 2.0,
                -front_face_thickness / 2.0,
                opening_center_z,
            )
        ),
        material=body_paint,
        name="frame_right",
    )
    body.visual(
        Box((body_width, front_face_thickness, top_field_height)),
        origin=Origin(
            xyz=(0.0, -front_face_thickness / 2.0, opening_top + top_field_height / 2.0),
        ),
        material=body_paint,
        name="frame_top",
    )
    body.visual(
        Box((body_width, front_face_thickness, bottom_field_height)),
        origin=Origin(xyz=(0.0, -front_face_thickness / 2.0, bottom_field_height / 2.0)),
        material=body_paint,
        name="frame_bottom",
    )

    jamb_depth = 0.05
    jamb_thickness = 0.012
    body.visual(
        Box((jamb_thickness, jamb_depth, opening_height)),
        origin=Origin(
            xyz=(
                opening_left + jamb_thickness / 2.0,
                -front_face_thickness - jamb_depth / 2.0,
                opening_center_z,
            )
        ),
        material=gasket,
        name="jamb_left",
    )
    body.visual(
        Box((jamb_thickness, jamb_depth, opening_height)),
        origin=Origin(
            xyz=(
                opening_right - jamb_thickness / 2.0,
                -front_face_thickness - jamb_depth / 2.0,
                opening_center_z,
            )
        ),
        material=gasket,
        name="jamb_right",
    )
    body.visual(
        Box((opening_width, jamb_depth, jamb_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -front_face_thickness - jamb_depth / 2.0,
                opening_top - jamb_thickness / 2.0,
            )
        ),
        material=gasket,
        name="jamb_top",
    )
    body.visual(
        Box((opening_width, jamb_depth, jamb_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -front_face_thickness - jamb_depth / 2.0,
                opening_bottom + jamb_thickness / 2.0,
            )
        ),
        material=gasket,
        name="jamb_bottom",
    )

    hinge_mount_width = 0.028
    hinge_mount_depth = 0.024
    hinge_mount_length = 0.12
    hinge_mount_x = opening_left - 0.024
    hinge_mount_y = 0.012
    for index, center_z in enumerate((opening_bottom + 0.09, opening_center_z, opening_top - 0.09), start=1):
        body.visual(
            Box((hinge_mount_width, hinge_mount_depth, hinge_mount_length)),
            origin=Origin(xyz=(hinge_mount_x, hinge_mount_y, center_z)),
            material=hardware,
            name=f"hinge_mount_{index}",
        )

    body.visual(
        Box((0.012, 0.010, 0.18)),
        origin=Origin(xyz=(opening_right + 0.006, 0.005, opening_center_z)),
        material=hardware,
        name="strike_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=56.0,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height / 2.0)),
    )

    door_gap = 0.004
    door_width = opening_width - 2.0 * door_gap
    door_height = opening_height - 2.0 * door_gap
    door_thickness = 0.018
    hinge_axis_x = opening_left - 0.010
    hinge_axis_y = 0.036
    hinge_axis_z = opening_bottom + door_gap
    panel_start_x = opening_left + door_gap - hinge_axis_x
    panel_center_x = panel_start_x + door_width / 2.0
    panel_center_y = -0.024

    door = model.part("service_door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(panel_center_x, panel_center_y, door_height / 2.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((door_width - 0.05, 0.008, door_height - 0.05)),
        origin=Origin(xyz=(panel_center_x, panel_center_y - 0.004, door_height / 2.0)),
        material=gasket,
        name="door_inner_plate",
    )

    knuckle_radius = 0.010
    knuckle_length = 0.10
    leaf_width = 0.030
    leaf_depth = 0.020
    leaf_center_x = 0.015
    leaf_center_y = -0.016
    for index, center_z in enumerate((0.09, door_height / 2.0, door_height - 0.09), start=1):
        door.visual(
            Cylinder(radius=knuckle_radius, length=knuckle_length),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=hardware,
            name=f"hinge_knuckle_{index}",
        )
        door.visual(
            Box((leaf_width, leaf_depth, knuckle_length)),
            origin=Origin(xyz=(leaf_center_x, leaf_center_y, center_z)),
            material=hardware,
            name=f"hinge_leaf_{index}",
        )

    handle_x = panel_start_x + door_width - 0.038
    handle_y = -0.004
    handle_span = 0.14
    handle_radius = 0.006
    door.visual(
        Cylinder(radius=handle_radius, length=handle_span),
        origin=Origin(xyz=(handle_x, handle_y, door_height / 2.0)),
        material=hardware,
        name="pull_handle",
    )
    door.visual(
        Box((0.012, 0.022, 0.018)),
        origin=Origin(xyz=(handle_x, -0.008, door_height / 2.0 - 0.07)),
        material=hardware,
        name="handle_lower_standoff",
    )
    door.visual(
        Box((0.012, 0.022, 0.018)),
        origin=Origin(xyz=(handle_x, -0.008, door_height / 2.0 + 0.07)),
        material=hardware,
        name="handle_upper_standoff",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.05, door_height)),
        mass=5.8,
        origin=Origin(xyz=(panel_center_x, -0.012, door_height / 2.0)),
    )

    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=2.1,
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

    body = object_model.get_part("equipment_body")
    door = object_model.get_part("service_door")
    hinge = object_model.get_articulation("body_to_service_door")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="frame_left",
            max_gap=0.006,
            max_penetration=0.0,
            name="hinge side panel gap stays narrow",
        )
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="frame_right",
            negative_elem="door_panel",
            max_gap=0.006,
            max_penetration=0.0,
            name="latch side panel gap stays narrow",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="frame_top",
            negative_elem="door_panel",
            max_gap=0.006,
            max_penetration=0.0,
            name="top reveal stays narrow",
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem="door_panel",
            negative_elem="frame_bottom",
            max_gap=0.006,
            max_penetration=0.0,
            name="bottom reveal stays narrow",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="frame_left",
            min_gap=0.002,
            max_gap=0.025,
            name="closed door sits slightly proud of the equipment face",
        )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.15}):
        opened_panel = ctx.part_element_world_aabb(door, elem="door_panel")

    opened_farther_out = (
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][1] > closed_panel[1][1] + 0.20
    )
    ctx.check(
        "door opens outward from the face",
        opened_farther_out,
        details=f"closed_panel={closed_panel}, opened_panel={opened_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
