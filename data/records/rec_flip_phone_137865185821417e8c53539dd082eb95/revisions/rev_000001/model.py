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
    rounded_rect_profile,
)


LOWER_WIDTH = 0.074
LOWER_LENGTH = 0.086
LOWER_THICKNESS = 0.0165
UPPER_WIDTH = 0.072
UPPER_LENGTH = 0.087
UPPER_THICKNESS = 0.0074
BODY_CORNER_RADIUS = 0.011
HINGE_RADIUS = 0.0046
HINGE_CENTER_Y = LOWER_LENGTH * 0.5 - 0.0020

UPPER_SHELL_CENTER_CLOSED = (0.0, -0.0390, LOWER_THICKNESS * 0.5 + UPPER_THICKNESS * 0.5)
UPPER_INNER_FACE_Z = UPPER_SHELL_CENTER_CLOSED[2] - UPPER_THICKNESS * 0.5
UPPER_REAR_FACE_Z = UPPER_SHELL_CENTER_CLOSED[2] + UPPER_THICKNESS * 0.5

CAMERA_PIVOT_X = -0.0285
CAMERA_PIVOT_Y = -0.0640
CAMERA_PIVOT_RADIUS = 0.00135
CAMERA_PIVOT_Z = UPPER_REAR_FACE_Z + 0.00495
CAMERA_MOUNT_RAIL_Z = UPPER_REAR_FACE_Z + 0.0018
CAMERA_POD_WIDTH = 0.024
CAMERA_POD_HEIGHT = 0.034
CAMERA_POD_THICKNESS = 0.005


def _rounded_slab_mesh(name: str, width: float, length: float, thickness: float, radius: float):
    profile = rounded_rect_profile(width, length, radius, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True, cap=True, closed=True),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    aabb_min, aabb_max = aabb
    return tuple((aabb_min[index] + aabb_max[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_screen_flip_phone")

    frame_graphite = model.material("frame_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.57, 0.59, 0.62, 1.0))
    rear_glass = model.material("rear_glass", rgba=(0.04, 0.05, 0.07, 1.0))
    display_black = model.material("display_black", rgba=(0.02, 0.03, 0.04, 1.0))
    main_display = model.material("main_display", rgba=(0.11, 0.24, 0.36, 1.0))
    lens_black = model.material("lens_black", rgba=(0.05, 0.05, 0.06, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.31, 0.40, 0.65))
    flash_white = model.material("flash_white", rgba=(0.88, 0.87, 0.79, 1.0))

    lower_shell_mesh = _rounded_slab_mesh(
        "lower_shell",
        LOWER_WIDTH,
        LOWER_LENGTH,
        LOWER_THICKNESS,
        BODY_CORNER_RADIUS,
    )
    upper_shell_mesh = _rounded_slab_mesh(
        "upper_shell",
        UPPER_WIDTH,
        UPPER_LENGTH,
        UPPER_THICKNESS,
        BODY_CORNER_RADIUS,
    )
    camera_pod_mesh = _rounded_slab_mesh(
        "camera_pod",
        CAMERA_POD_WIDTH,
        CAMERA_POD_HEIGHT,
        CAMERA_POD_THICKNESS,
        0.004,
    )

    lower_body = model.part("lower_body")
    lower_body.visual(lower_shell_mesh, material=frame_graphite, name="lower_shell")
    lower_body.visual(
        Box((LOWER_WIDTH - 0.006, LOWER_LENGTH - 0.012, 0.0013)),
        origin=Origin(xyz=(0.0, 0.0, -LOWER_THICKNESS * 0.5 + 0.00065)),
        material=rear_glass,
        name="secondary_display",
    )
    lower_body.visual(
        Box((LOWER_WIDTH - 0.002, 0.0050, 0.0050)),
        origin=Origin(xyz=(0.0, HINGE_CENTER_Y - 0.0015, 0.0)),
        material=hinge_metal,
        name="hinge_bridge",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=LOWER_WIDTH - 0.003),
        origin=Origin(xyz=(0.0, HINGE_CENTER_Y, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="hinge_barrel",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((LOWER_WIDTH, LOWER_LENGTH, LOWER_THICKNESS + 0.010)),
        mass=0.115,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=UPPER_SHELL_CENTER_CLOSED),
        material=frame_graphite,
        name="upper_shell",
    )
    upper_body.visual(
        Box((UPPER_WIDTH - 0.008, UPPER_LENGTH - 0.010, 0.0010)),
        origin=Origin(xyz=(0.0, -0.0400, UPPER_INNER_FACE_Z + 0.0005)),
        material=main_display,
        name="main_display",
    )
    upper_body.visual(
        Box((0.013, 0.0022, 0.0007)),
        origin=Origin(xyz=(0.0, -0.0715, UPPER_INNER_FACE_Z + 0.00035)),
        material=display_black,
        name="earpiece_slot",
    )
    upper_body.visual(
        Box((0.028, 0.036, 0.0016)),
        origin=Origin(xyz=(CAMERA_PIVOT_X + 0.013, CAMERA_PIVOT_Y, UPPER_REAR_FACE_Z + 0.0008)),
        material=rear_glass,
        name="camera_mount_plate",
    )
    upper_body.visual(
        Box((0.0022, 0.034, 0.0036)),
        origin=Origin(xyz=(CAMERA_PIVOT_X, CAMERA_PIVOT_Y, CAMERA_MOUNT_RAIL_Z)),
        material=hinge_metal,
        name="camera_mount_rail",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((UPPER_WIDTH, UPPER_LENGTH, 0.022)),
        mass=0.082,
        origin=Origin(xyz=(0.0, -0.0390, 0.0115)),
    )

    camera_pod = model.part("camera_pod")
    camera_pod.visual(
        Cylinder(radius=CAMERA_PIVOT_RADIUS, length=CAMERA_POD_HEIGHT - 0.002),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_metal,
        name="pivot_barrel",
    )
    camera_pod.visual(
        Box((0.0050, CAMERA_POD_HEIGHT - 0.002, 0.0032)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0016)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    camera_pod.visual(
        camera_pod_mesh,
        origin=Origin(xyz=(0.0140, 0.0, CAMERA_POD_THICKNESS * 0.5)),
        material=lens_black,
        name="pod_body",
    )
    camera_pod.visual(
        Box((0.0195, 0.028, 0.0012)),
        origin=Origin(xyz=(0.0150, 0.0, CAMERA_POD_THICKNESS + 0.0006)),
        material=display_black,
        name="lens_glass_panel",
    )
    for index, y_pos in enumerate((0.008, -0.008)):
        lens_center_x = 0.012 if index == 0 else 0.018
        camera_pod.visual(
            Cylinder(radius=0.0048, length=0.0026),
            origin=Origin(xyz=(lens_center_x, y_pos, CAMERA_POD_THICKNESS + 0.0013)),
            material=lens_black,
            name=f"lens_housing_{index}",
        )
        camera_pod.visual(
            Cylinder(radius=0.0037, length=0.0012),
            origin=Origin(xyz=(lens_center_x, y_pos, CAMERA_POD_THICKNESS + 0.0024)),
            material=lens_glass,
            name=f"lens_glass_{index}",
        )
    camera_pod.visual(
        Cylinder(radius=0.0024, length=0.0011),
        origin=Origin(xyz=(0.0205, 0.0005, CAMERA_POD_THICKNESS + 0.0012)),
        material=flash_white,
        name="flash",
    )
    camera_pod.inertial = Inertial.from_geometry(
        Box((0.030, 0.038, 0.010)),
        mass=0.022,
        origin=Origin(xyz=(0.014, 0.0, 0.005)),
    )

    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, HINGE_CENTER_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.9,
            velocity=4.0,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "upper_to_camera_pod",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=camera_pod,
        origin=Origin(xyz=(CAMERA_PIVOT_X, CAMERA_PIVOT_Y, CAMERA_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=6.0,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    camera_pod = object_model.get_part("camera_pod")
    phone_hinge = object_model.get_articulation("lower_to_upper")
    pod_swivel = object_model.get_articulation("upper_to_camera_pod")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
        "phone hinge rotates about the body width",
        tuple(round(value, 6) for value in phone_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={phone_hinge.axis}",
    )
    ctx.check(
        "camera pod uses a vertical swivel axis",
        tuple(round(value, 6) for value in pod_swivel.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pod_swivel.axis}",
    )

    ctx.expect_contact(
        camera_pod,
        upper_body,
        elem_a="pivot_barrel",
        elem_b="camera_mount_rail",
        contact_tol=0.0008,
        name="camera pod remains captured by the upper-body mount rail",
    )

    secondary_display_aabb = ctx.part_element_world_aabb(lower_body, elem="secondary_display")
    lower_shell_aabb = ctx.part_element_world_aabb(lower_body, elem="lower_shell")
    main_display_aabb = ctx.part_element_world_aabb(upper_body, elem="main_display")
    upper_shell_aabb = ctx.part_element_world_aabb(upper_body, elem="upper_shell")

    secondary_width = None
    lower_shell_width = None
    if secondary_display_aabb is not None:
        secondary_width = secondary_display_aabb[1][0] - secondary_display_aabb[0][0]
    if lower_shell_aabb is not None:
        lower_shell_width = lower_shell_aabb[1][0] - lower_shell_aabb[0][0]
    ctx.check(
        "secondary display spans nearly the full lower-body width",
        secondary_width is not None
        and lower_shell_width is not None
        and secondary_width >= lower_shell_width - 0.010,
        details=f"display_width={secondary_width}, shell_width={lower_shell_width}",
    )

    main_display_size_ok = False
    if main_display_aabb is not None and upper_shell_aabb is not None:
        main_display_size_ok = (
            main_display_aabb[0][0] >= upper_shell_aabb[0][0] + 0.002
            and main_display_aabb[1][0] <= upper_shell_aabb[1][0] - 0.002
            and main_display_aabb[0][1] >= upper_shell_aabb[0][1] + 0.002
            and main_display_aabb[1][1] <= upper_shell_aabb[1][1] - 0.002
        )
    ctx.check(
        "main display stays inset within the upper shell bezel",
        main_display_size_ok,
        details=f"main_display={main_display_aabb}, upper_shell={upper_shell_aabb}",
    )

    with ctx.pose({phone_hinge: phone_hinge.motion_limits.lower}):
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="z",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            max_gap=0.0015,
            max_penetration=0.0006,
            name="closed halves nearly meet without interpenetration",
        )
        ctx.expect_overlap(
            upper_body,
            lower_body,
            axes="xy",
            elem_a="upper_shell",
            elem_b="lower_shell",
            min_overlap=0.060,
            name="closed halves align into a clamshell stack",
        )

    upper_rest_aabb = ctx.part_element_world_aabb(upper_body, elem="upper_shell")
    with ctx.pose({phone_hinge: 0.75}):
        upper_open_aabb = ctx.part_element_world_aabb(upper_body, elem="upper_shell")
    ctx.check(
        "positive phone hinge motion lifts the upper body",
        upper_rest_aabb is not None
        and upper_open_aabb is not None
        and upper_open_aabb[1][2] > upper_rest_aabb[1][2] + 0.020,
        details=f"rest={upper_rest_aabb}, opened={upper_open_aabb}",
    )

    pod_rest_aabb = ctx.part_element_world_aabb(camera_pod, elem="pod_body")
    with ctx.pose({pod_swivel: 1.0}):
        pod_swung_aabb = ctx.part_element_world_aabb(camera_pod, elem="pod_body")
        ctx.expect_contact(
            camera_pod,
            upper_body,
            elem_a="pivot_barrel",
            elem_b="camera_mount_rail",
            contact_tol=0.0008,
            name="camera pod stays supported while swiveled",
        )
    pod_rest_center = _aabb_center(pod_rest_aabb)
    pod_swung_center = _aabb_center(pod_swung_aabb)
    ctx.check(
        "camera pod swings outward from its edge pivot",
        pod_rest_center is not None
        and pod_swung_center is not None
        and pod_swung_center[0] < pod_rest_center[0] - 0.003,
        details=f"rest_center={pod_rest_center}, swung_center={pod_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
