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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(
    width: float,
    height: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _offset_rect_loop(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> list[tuple[float, float]]:
    return [
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
    ]


def _circle_loop(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 24,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    body_finish = model.material("body_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    lock_finish = model.material("lock_finish", rgba=(0.74, 0.76, 0.79, 1.0))

    base_size = (0.34, 0.34, 0.025)
    post_size = (0.12, 0.12, 0.84)
    collar_size = (0.17, 0.17, 0.09)

    body_depth = 0.36
    body_width = 0.42
    body_height = 0.56
    wall = 0.018
    roof_size = (0.46, 0.40, 0.028)

    door_width = 0.40
    door_height = 0.54
    door_thickness = 0.02
    lock_radius = 0.0095
    lock_clear_radius = 0.0105
    hinge_radius = 0.007
    hinge_segment = 0.09

    base_top_z = base_size[2]
    post_top_z = base_top_z + post_size[2]
    collar_center_z = post_top_z + (collar_size[2] * 0.5) - 0.005
    body_bottom_z = post_top_z + collar_size[2] - 0.01
    body_center_z = body_bottom_z + (body_height * 0.5)
    roof_center_z = body_bottom_z + body_height + (roof_size[2] * 0.5) - 0.004
    door_center_z = body_center_z
    shell_front_x = body_depth * 0.5
    hinge_x = shell_front_x + 0.030
    hinge_y = -door_width * 0.5
    leaf_offset_z = -0.018

    shell_profile = _rect_loop(body_width, body_height)
    shell_hole = _rect_loop(body_width - (2.0 * wall), body_height - (2.0 * wall))
    shell_geom = ExtrudeWithHolesGeometry(
        shell_profile,
        [shell_hole],
        body_depth,
        cap=True,
        center=True,
    )
    shell_mesh = mesh_from_geometry(shell_geom, "mailbox_shell")

    door_profile = _offset_rect_loop(0.0, door_width, -door_height * 0.5, door_height * 0.5)
    lock_center = (door_width - 0.052, 0.0)
    door_hole = _circle_loop(lock_clear_radius, center=lock_center, segments=28)
    door_geom = ExtrudeWithHolesGeometry(
        door_profile,
        [door_hole],
        door_thickness,
        cap=True,
        center=True,
    )
    door_mesh = mesh_from_geometry(door_geom, "retrieval_door")

    escutcheon_geom = ExtrudeWithHolesGeometry(
        _rect_loop(0.052, 0.072, center=lock_center),
        [door_hole],
        0.004,
        cap=True,
        center=True,
    )
    escutcheon_mesh = mesh_from_geometry(escutcheon_geom, "lock_housing")

    pedestal_body = model.part("pedestal_body")
    pedestal_body.visual(
        Box(base_size),
        origin=Origin(xyz=(0.0, 0.0, base_size[2] * 0.5)),
        material=trim_finish,
        name="base_plate",
    )
    pedestal_body.visual(
        Box(post_size),
        origin=Origin(xyz=(0.0, 0.0, base_top_z + (post_size[2] * 0.5))),
        material=trim_finish,
        name="support_post",
    )
    pedestal_body.visual(
        Box(collar_size),
        origin=Origin(xyz=(0.0, 0.0, collar_center_z)),
        material=body_finish,
        name="support_collar",
    )
    pedestal_body.visual(
        shell_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, body_center_z),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        material=body_finish,
        name="mailbox_shell",
    )
    pedestal_body.visual(
        Box((wall, body_width - (2.0 * wall), body_height - (2.0 * wall))),
        origin=Origin(
            xyz=(-body_depth * 0.5 + (wall * 0.5), 0.0, body_center_z),
        ),
        material=body_finish,
        name="rear_panel",
    )
    pedestal_body.visual(
        Box(roof_size),
        origin=Origin(xyz=(0.0, 0.0, roof_center_z)),
        material=trim_finish,
        name="roof_cap",
    )
    for index, z_offset in enumerate((-0.185, 0.0, 0.185)):
        pedestal_body.visual(
            Cylinder(radius=hinge_radius, length=hinge_segment),
            origin=Origin(xyz=(hinge_x, hinge_y, door_center_z + z_offset)),
            material=trim_finish,
            name=f"body_hinge_knuckle_{index}",
        )
    pedestal_body.visual(
        Box((0.032, 0.012, door_height)),
        origin=Origin(xyz=(0.195, hinge_y - 0.008, door_center_z)),
        material=trim_finish,
        name="body_hinge_leaf",
    )
    pedestal_body.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, roof_center_z + (roof_size[2] * 0.5))),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (roof_center_z + (roof_size[2] * 0.5)))),
    )

    door = model.part("door")
    door.visual(
        door_mesh,
        origin=Origin(xyz=(0.0, 0.0, leaf_offset_z)),
        material=body_finish,
        name="door_panel",
    )
    door.visual(
        escutcheon_mesh,
        origin=Origin(xyz=(0.0, 0.0, leaf_offset_z + (door_thickness * 0.5) + 0.002)),
        material=lock_finish,
        name="lock_housing",
    )
    door.visual(
        Box((0.040, 0.012, 0.020)),
        origin=Origin(
            xyz=(
                door_width - 0.060,
                -0.085,
                leaf_offset_z + (door_thickness * 0.5) + 0.010,
            )
        ),
        material=lock_finish,
        name="pull_handle",
    )
    for index, y_offset in enumerate((-0.0925, 0.0925)):
        door.visual(
            Box((0.014, hinge_segment, 0.018)),
            origin=Origin(xyz=(0.007, y_offset, -0.009)),
            material=trim_finish,
            name=f"door_hinge_leaf_{index}",
        )
        door.visual(
            Cylinder(radius=hinge_radius * 0.95, length=hinge_segment),
            origin=Origin(
                xyz=(0.0, y_offset, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_finish,
            name=f"door_hinge_knuckle_{index}",
        )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_height, 0.035)),
        mass=4.8,
        origin=Origin(xyz=(door_width * 0.5, 0.0, -0.010)),
    )

    lock_cylinder = model.part("lock_cylinder")
    lock_cylinder.visual(
        Cylinder(radius=lock_radius, length=0.034),
        material=lock_finish,
        name="key_plug",
    )
    lock_cylinder.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=lock_finish,
        name="front_collar",
    )
    lock_cylinder.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=lock_finish,
        name="rear_collar",
    )
    lock_cylinder.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=trim_finish,
        name="rear_spindle",
    )
    lock_cylinder.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, -0.038)),
        material=trim_finish,
        name="lock_cam",
    )
    lock_cylinder.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, 0.050)),
        mass=0.15,
        origin=Origin(xyz=(0.006, 0.0, -0.010)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=pedestal_body,
        child=door,
        origin=Origin(
            xyz=(hinge_x, hinge_y, door_center_z),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "door_to_lock",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock_cylinder,
        origin=Origin(xyz=(lock_center[0], lock_center[1], leaf_offset_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.7,
            upper=0.7,
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

    pedestal_body = object_model.get_part("pedestal_body")
    door = object_model.get_part("door")
    lock_cylinder = object_model.get_part("lock_cylinder")
    door_hinge = object_model.get_articulation("body_to_door")
    lock_joint = object_model.get_articulation("door_to_lock")

    ctx.check(
        "door hinge joint axis is authored for outward opening",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "lock cylinder rotates about its local plug axis",
        tuple(round(v, 6) for v in lock_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lock_joint.axis}",
    )

    with ctx.pose({door_hinge: 0.0, lock_joint: 0.0}):
        ctx.expect_overlap(
            door,
            pedestal_body,
            axes="yz",
            elem_a="door_panel",
            elem_b="mailbox_shell",
            min_overlap=0.34,
            name="door panel broadly covers the front opening",
        )
        ctx.expect_gap(
            door,
            pedestal_body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="mailbox_shell",
            min_gap=0.0,
            max_gap=0.006,
            name="closed door sits just proud of the mailbox face",
        )
        closed_door_center = _aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))
        rest_cam_center = _aabb_center(ctx.part_element_world_aabb(lock_cylinder, elem="lock_cam"))

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper, lock_joint: 0.0}):
        open_door_center = _aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))

    ctx.check(
        "front retrieval door swings outward",
        closed_door_center is not None
        and open_door_center is not None
        and open_door_center[0] > closed_door_center[0] + 0.10,
        details=f"closed_center={closed_door_center}, open_center={open_door_center}",
    )

    with ctx.pose({door_hinge: 0.0, lock_joint: 0.5}):
        turned_cam_center = _aabb_center(ctx.part_element_world_aabb(lock_cylinder, elem="lock_cam"))

    ctx.check(
        "key turn rotates the latch cam",
        rest_cam_center is not None
        and turned_cam_center is not None
        and (
            abs(turned_cam_center[1] - rest_cam_center[1]) > 0.003
            or abs(turned_cam_center[2] - rest_cam_center[2]) > 0.003
        ),
        details=f"rest_cam={rest_cam_center}, turned_cam={turned_cam_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
