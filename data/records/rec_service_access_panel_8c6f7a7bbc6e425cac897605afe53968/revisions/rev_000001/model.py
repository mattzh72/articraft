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

    body_color = model.material("body_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    frame_color = model.material("frame_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    door_color = model.material("door_gray", rgba=(0.60, 0.62, 0.65, 1.0))
    hinge_color = model.material("hinge_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    latch_color = model.material("latch_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body_width = 0.82
    body_depth = 0.46
    body_height = 0.62
    wall_thickness = 0.024
    front_face_thickness = 0.020

    opening_width = 0.34
    opening_height = 0.22
    opening_center_z = 0.0
    opening_return_depth = 0.038
    opening_return_thickness = 0.018

    door_width = 0.37
    door_height = 0.25
    door_thickness = 0.018
    door_gap = 0.002
    door_cover = (door_width - opening_width) / 2.0

    front_y = body_depth / 2.0 - front_face_thickness / 2.0
    side_depth = body_depth - 2.0 * front_face_thickness

    left_frame_width = (body_width - opening_width) / 2.0
    top_frame_height = (body_height - opening_height) / 2.0

    body = model.part("equipment_body")
    body.visual(
        Box((body_width, front_face_thickness, body_height)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + front_face_thickness / 2.0, 0.0)),
        material=body_color,
        name="rear_panel",
    )
    body.visual(
        Box((wall_thickness, side_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + wall_thickness / 2.0, 0.0, 0.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, side_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - wall_thickness / 2.0, 0.0, 0.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, side_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0 - wall_thickness / 2.0)),
        material=body_color,
        name="top_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, side_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -body_height / 2.0 + wall_thickness / 2.0)),
        material=body_color,
        name="bottom_wall",
    )
    body.visual(
        Box((left_frame_width, front_face_thickness, body_height)),
        origin=Origin(
            xyz=(-body_width / 2.0 + left_frame_width / 2.0, front_y, 0.0),
        ),
        material=frame_color,
        name="front_frame_left",
    )
    body.visual(
        Box((0.016, 0.014, door_height)),
        origin=Origin(
            xyz=(
                -opening_width / 2.0 - door_cover - 0.016,
                body_depth / 2.0 + 0.005,
                opening_center_z,
            ),
        ),
        material=hinge_color,
        name="hinge_mount",
    )
    body.visual(
        Box((left_frame_width, front_face_thickness, body_height)),
        origin=Origin(
            xyz=(body_width / 2.0 - left_frame_width / 2.0, front_y, 0.0),
        ),
        material=frame_color,
        name="front_frame_right",
    )
    body.visual(
        Box((opening_width, front_face_thickness, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                opening_center_z + opening_height / 2.0 + top_frame_height / 2.0,
            ),
        ),
        material=frame_color,
        name="front_frame_top",
    )
    body.visual(
        Box((opening_width, front_face_thickness, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                opening_center_z - opening_height / 2.0 - top_frame_height / 2.0,
            ),
        ),
        material=frame_color,
        name="front_frame_bottom",
    )
    body.visual(
        Box((opening_return_thickness, opening_return_depth, opening_height)),
        origin=Origin(
            xyz=(
                -opening_width / 2.0 - opening_return_thickness / 2.0,
                body_depth / 2.0 - opening_return_depth / 2.0,
                opening_center_z,
            ),
        ),
        material=frame_color,
        name="opening_jamb_left",
    )
    body.visual(
        Box((opening_return_thickness, opening_return_depth, opening_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + opening_return_thickness / 2.0,
                body_depth / 2.0 - opening_return_depth / 2.0,
                opening_center_z,
            ),
        ),
        material=frame_color,
        name="opening_jamb_right",
    )
    body.visual(
        Box((opening_width, opening_return_depth, opening_return_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - opening_return_depth / 2.0,
                opening_center_z + opening_height / 2.0 + opening_return_thickness / 2.0,
            ),
        ),
        material=frame_color,
        name="opening_jamb_top",
    )
    body.visual(
        Box((opening_width, opening_return_depth, opening_return_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - opening_return_depth / 2.0,
                opening_center_z - opening_height / 2.0 - opening_return_thickness / 2.0,
            ),
        ),
        material=frame_color,
        name="opening_jamb_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=34.0,
        origin=Origin(),
    )

    door = model.part("access_door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
        material=door_color,
        name="door_skin",
    )
    door.visual(
        Box((door_width - 0.070, 0.018, door_height - 0.070)),
        origin=Origin(xyz=(door_width / 2.0, -0.017, 0.0)),
        material=frame_color,
        name="inner_stiffener",
    )
    door.visual(
        Cylinder(radius=0.008, length=door_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_color,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.058, 0.006, 0.090)),
        origin=Origin(xyz=(door_width - 0.036, 0.011, 0.0)),
        material=hinge_color,
        name="latch_plate",
    )
    door.visual(
        Box((0.036, 0.016, 0.048)),
        origin=Origin(xyz=(door_width - 0.028, 0.016, 0.0)),
        material=latch_color,
        name="latch_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness + 0.020, door_height)),
        mass=3.2,
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                -opening_width / 2.0 - door_cover,
                body_depth / 2.0 + door_thickness / 2.0 + door_gap,
                opening_center_z,
            ),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("equipment_body")
    door = object_model.get_part("access_door")
    hinge = object_model.get_articulation("door_hinge")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.0015,
        max_gap=0.0035,
        positive_elem="door_skin",
        negative_elem="front_frame_top",
        name="closed door sits just proud of the equipment face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.012,
        elem_a="door_skin",
        elem_b="front_frame_top",
        name="door overlays the top frame lip",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.012,
        elem_a="door_skin",
        elem_b="front_frame_right",
        name="door overlays the latch-side frame lip",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_skin")

    moved_outward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.10
    )
    ctx.check(
        "door swings outward about its vertical side hinge",
        moved_outward,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
