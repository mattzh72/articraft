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

    body_color = model.material("body_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    frame_color = model.material("frame_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    door_color = model.material("door_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    metal_color = model.material("hardware", rgba=(0.68, 0.70, 0.72, 1.0))

    body_width = 0.46
    body_depth = 0.14
    body_height = 0.36
    skin_t = 0.01

    opening_width = 0.30
    opening_height = 0.24

    door_thickness = 0.008
    door_height = 0.236
    door_width = 0.289
    hinge_radius = 0.006
    hinge_axis_x = -(opening_width / 2.0) + 0.001
    hinge_axis_y = (body_depth / 2.0) + 0.005
    door_leaf_offset = 0.008

    body = model.part("body")
    body.visual(
        Box((body_width, skin_t, body_height)),
        origin=Origin(xyz=(0.0, -(body_depth / 2.0) + (skin_t / 2.0), 0.0)),
        material=body_color,
        name="back_plate",
    )
    body.visual(
        Box((skin_t, body_depth - 2.0 * skin_t, body_height)),
        origin=Origin(xyz=(-(body_width / 2.0) + (skin_t / 2.0), 0.0, 0.0)),
        material=body_color,
        name="left_side",
    )
    body.visual(
        Box((skin_t, body_depth - 2.0 * skin_t, body_height)),
        origin=Origin(xyz=((body_width / 2.0) - (skin_t / 2.0), 0.0, 0.0)),
        material=body_color,
        name="right_side",
    )
    body.visual(
        Box((body_width - 2.0 * skin_t, body_depth - 2.0 * skin_t, skin_t)),
        origin=Origin(xyz=(0.0, 0.0, (body_height / 2.0) - (skin_t / 2.0))),
        material=body_color,
        name="top_shell",
    )
    body.visual(
        Box((body_width - 2.0 * skin_t, body_depth - 2.0 * skin_t, skin_t)),
        origin=Origin(xyz=(0.0, 0.0, -(body_height / 2.0) + (skin_t / 2.0))),
        material=body_color,
        name="bottom_shell",
    )

    side_frame_w = (body_width - opening_width) / 2.0
    top_frame_h = (body_height - opening_height) / 2.0
    front_y = (body_depth / 2.0) - (skin_t / 2.0)

    body.visual(
        Box((side_frame_w, skin_t, body_height)),
        origin=Origin(xyz=(-(opening_width / 2.0) - (side_frame_w / 2.0), front_y, 0.0)),
        material=frame_color,
        name="frame_left",
    )
    body.visual(
        Box((side_frame_w, skin_t, body_height)),
        origin=Origin(xyz=((opening_width / 2.0) + (side_frame_w / 2.0), front_y, 0.0)),
        material=frame_color,
        name="frame_right",
    )
    body.visual(
        Box((opening_width, skin_t, top_frame_h)),
        origin=Origin(xyz=(0.0, front_y, (opening_height / 2.0) + (top_frame_h / 2.0))),
        material=frame_color,
        name="frame_top",
    )
    body.visual(
        Box((opening_width, skin_t, top_frame_h)),
        origin=Origin(xyz=(0.0, front_y, -(opening_height / 2.0) - (top_frame_h / 2.0))),
        material=frame_color,
        name="frame_bottom",
    )

    for name, z_pos in (
        ("hinge_body_upper", -0.092),
        ("hinge_body_mid", 0.0),
        ("hinge_body_lower", 0.092),
    ):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.042),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z_pos)),
            material=metal_color,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.0,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_leaf_offset + (door_width / 2.0), 0.0, 0.0)),
        material=door_color,
        name="door_skin",
    )
    door.visual(
        Box((0.022, 0.014, 0.048)),
        origin=Origin(xyz=(door_leaf_offset + door_width - 0.017, 0.011, 0.0)),
        material=metal_color,
        name="latch_boss",
    )
    door.visual(
        Box((0.005, 0.018, 0.085)),
        origin=Origin(xyz=(door_leaf_offset + door_width - 0.017, 0.025, 0.0)),
        material=metal_color,
        name="pull_handle",
    )
    for name, z_pos in (("hinge_leaf_lower", -0.046), ("hinge_leaf_upper", 0.046)):
        door.visual(
            Box((0.012, 0.006, 0.042)),
            origin=Origin(xyz=(0.010, 0.0, z_pos)),
            material=metal_color,
            name=name,
        )
    for name, z_pos in (("hinge_door_lower", -0.046), ("hinge_door_upper", 0.046)):
        door.visual(
            Cylinder(radius=hinge_radius, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=metal_color,
            name=name,
        )

    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.2,
        origin=Origin(xyz=(door_leaf_offset + (door_width / 2.0), 0.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_skin",
        negative_elem="frame_top",
        min_gap=0.0005,
        max_gap=0.0025,
        name="door sits just proud of the front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_skin",
        min_overlap=0.22,
        name="closed door covers the framed opening area",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({hinge: 1.3}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_skin")

    open_outward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.10
    )
    ctx.check(
        "door swings outward from the equipment face",
        open_outward,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
