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
    model = ArticulatedObject(name="communal_mailbox")

    frame_color = model.material("frame_paint", rgba=(0.33, 0.29, 0.24, 1.0))
    housing_color = model.material("housing_metal", rgba=(0.70, 0.73, 0.75, 1.0))
    door_color = model.material("door_paint", rgba=(0.52, 0.48, 0.41, 1.0))
    handle_color = model.material("handle_dark", rgba=(0.11, 0.11, 0.12, 1.0))

    frame_w = 0.40
    frame_h = 0.30
    frame_d = 0.48
    frame_post = 0.03
    frame_rail = 0.04

    housing_w = 0.34
    housing_h = 0.22
    housing_d = 0.42
    housing_wall = 0.012

    door_w = 0.332
    door_h = 0.212
    door_t = 0.016

    handle_inset = 0.054

    frame = model.part("frame")
    frame.visual(
        Box((frame_d, frame_post, frame_h)),
        origin=Origin(xyz=(0.0, -(frame_w - frame_post) / 2.0, 0.0)),
        material=frame_color,
        name="left_post",
    )
    frame.visual(
        Box((frame_d, frame_post, frame_h)),
        origin=Origin(xyz=(0.0, (frame_w - frame_post) / 2.0, 0.0)),
        material=frame_color,
        name="right_post",
    )
    frame.visual(
        Box((frame_d, frame_w - 2.0 * frame_post, frame_rail)),
        origin=Origin(xyz=(0.0, 0.0, (frame_h - frame_rail) / 2.0)),
        material=frame_color,
        name="top_rail",
    )
    frame.visual(
        Box((frame_d, frame_w - 2.0 * frame_post, frame_rail)),
        origin=Origin(xyz=(0.0, 0.0, -(frame_h - frame_rail) / 2.0)),
        material=frame_color,
        name="bottom_rail",
    )

    housing = model.part("housing")
    housing.visual(
        Box((housing_d, housing_wall, housing_h)),
        origin=Origin(xyz=(0.0, -(housing_w - housing_wall) / 2.0, 0.0)),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((housing_d, housing_wall, housing_h)),
        origin=Origin(xyz=(0.0, (housing_w - housing_wall) / 2.0, 0.0)),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((housing_d, housing_w - 2.0 * housing_wall, housing_wall)),
        origin=Origin(xyz=(0.0, 0.0, (housing_h - housing_wall) / 2.0)),
        material=housing_color,
        name="top_wall",
    )
    housing.visual(
        Box((housing_d, housing_w - 2.0 * housing_wall, housing_wall)),
        origin=Origin(xyz=(0.0, 0.0, -(housing_h - housing_wall) / 2.0)),
        material=housing_color,
        name="bottom_wall",
    )
    housing.visual(
        Box((housing_wall, housing_w - 2.0 * housing_wall, housing_h - 2.0 * housing_wall)),
        origin=Origin(xyz=(-(housing_d - housing_wall) / 2.0, 0.0, 0.0)),
        material=housing_color,
        name="back_panel",
    )

    door = model.part("door")
    door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(door_t / 2.0, door_w / 2.0, 0.0)),
        material=door_color,
        name="door_panel",
    )
    door.visual(
        Box((0.006, door_w - 0.056, door_h - 0.060)),
        origin=Origin(xyz=(door_t - 0.003, door_w / 2.0, 0.0)),
        material=door_color,
        name="door_stiffener",
    )
    for idx, z_pos in enumerate((-0.068, 0.0, 0.068), start=1):
        door.visual(
            Cylinder(radius=0.008, length=0.046),
            origin=Origin(xyz=(door_t / 2.0, 0.0, z_pos)),
            material=door_color,
            name=f"hinge_barrel_{idx}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_color,
        name="pull_rosette",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_color,
        name="pull_spindle",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_color,
        name="pull_knob",
    )
    handle.visual(
        Box((0.006, 0.010, 0.020)),
        origin=Origin(xyz=(0.036, 0.0, 0.012)),
        material=handle_color,
        name="pull_fin",
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(housing_d / 2.0, -door_w / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.8,
        ),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_t, door_w - handle_inset, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
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

    frame = object_model.get_part("frame")
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("housing_to_door")
    handle_pivot = object_model.get_articulation("door_to_handle")

    ctx.expect_contact(
        housing,
        frame,
        name="housing is captured by the outer frame",
    )
    ctx.expect_within(
        housing,
        frame,
        axes="yz",
        margin=0.0,
        name="housing stays within the larger frame opening",
    )

    with ctx.pose({door_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="door_panel",
            name="closed door seats on the housing front plane",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            min_overlap=0.18,
            elem_a="door_panel",
            name="door covers the mailbox opening",
        )

        handle_rest = ctx.part_world_position(handle)
        fin_rest_aabb = ctx.part_element_world_aabb(handle, elem="pull_fin")

    with ctx.pose({door_hinge: 1.2, handle_pivot: 0.0}):
        handle_open = ctx.part_world_position(handle)

    ctx.check(
        "door swings outward from the hinge side",
        handle_rest is not None
        and handle_open is not None
        and handle_open[0] > handle_rest[0] + 0.08,
        details=f"rest={handle_rest}, open={handle_open}",
    )

    with ctx.pose({door_hinge: 0.0, handle_pivot: 0.28}):
        fin_rot_aabb = ctx.part_element_world_aabb(handle, elem="pull_fin")

    fin_rest_center = None
    fin_rot_center = None
    if fin_rest_aabb is not None:
        fin_rest_center = tuple(
            (lo + hi) / 2.0 for lo, hi in zip(fin_rest_aabb[0], fin_rest_aabb[1])
        )
    if fin_rot_aabb is not None:
        fin_rot_center = tuple(
            (lo + hi) / 2.0 for lo, hi in zip(fin_rot_aabb[0], fin_rot_aabb[1])
        )

    ctx.check(
        "pull handle rotates about its local pivot",
        fin_rest_center is not None
        and fin_rot_center is not None
        and abs(fin_rot_center[1] - fin_rest_center[1]) > 0.003,
        details=f"rest_center={fin_rest_center}, rotated_center={fin_rot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
