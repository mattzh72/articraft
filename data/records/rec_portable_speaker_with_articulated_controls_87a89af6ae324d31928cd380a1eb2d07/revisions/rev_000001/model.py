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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_speaker")

    width = 0.34
    depth = 0.12
    height = 0.20

    mount_thickness = 0.016
    mount_y = 0.100
    mount_z = 0.140
    handle_span = 0.086
    handle_bar = 0.010
    handle_arm_y = 0.012
    handle_drop = 0.060
    hinge_radius = 0.005

    wheel_radius = 0.018
    wheel_length = 0.072
    wheel_center = (0.0, -(depth / 2.0) + 0.017, 0.185)

    shell = model.material("shell", rgba=(0.18, 0.19, 0.21, 1.0))
    grille = model.material("grille", rgba=(0.08, 0.08, 0.09, 1.0))
    trim = model.material("trim", rgba=(0.28, 0.29, 0.31, 1.0))
    handle_mat = model.material("handle", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_mat = model.material("wheel", rgba=(0.70, 0.72, 0.75, 1.0))
    wheel_hub_mat = model.material("wheel_hub", rgba=(0.20, 0.21, 0.22, 1.0))

    body = model.part("speaker_body")
    body.visual(
        Box((width, depth, 0.164)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box((width, 0.084, 0.036)),
        origin=Origin(xyz=(0.0, 0.018, 0.182)),
        material=shell,
        name="upper_rear_shell",
    )
    body.visual(
        Box((0.110, 0.036, 0.036)),
        origin=Origin(xyz=(-0.115, -0.042, 0.182)),
        material=shell,
        name="upper_left_shoulder",
    )
    body.visual(
        Box((0.110, 0.036, 0.036)),
        origin=Origin(xyz=(0.115, -0.042, 0.182)),
        material=shell,
        name="upper_right_shoulder",
    )
    body.visual(
        Box((0.292, 0.008, 0.138)),
        origin=Origin(xyz=(0.0, -(depth / 2.0) + 0.004, 0.087)),
        material=grille,
        name="front_grille",
    )
    body.visual(
        Box((0.104, 0.008, 0.020)),
        origin=Origin(xyz=(-0.094, -(depth / 2.0) + 0.004, 0.166)),
        material=grille,
        name="front_grille_upper_left",
    )
    body.visual(
        Box((0.104, 0.008, 0.020)),
        origin=Origin(xyz=(0.094, -(depth / 2.0) + 0.004, 0.166)),
        material=grille,
        name="front_grille_upper_right",
    )
    body.visual(
        Box((0.176, 0.058, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.007)),
        material=trim,
        name="top_cap",
    )
    body.visual(
        Box((mount_thickness, mount_y, mount_z)),
        origin=Origin(
            xyz=(
                -(width / 2.0) - (mount_thickness / 2.0),
                0.0,
                0.105,
            )
        ),
        material=trim,
        name="left_side_mount",
    )
    body.visual(
        Box((mount_thickness, mount_y, mount_z)),
        origin=Origin(
            xyz=(
                (width / 2.0) + (mount_thickness / 2.0),
                0.0,
                0.105,
            )
        ),
        material=trim,
        name="right_side_mount",
    )
    body.visual(
        Box((0.012, 0.094, 0.014)),
        origin=Origin(xyz=(-(width / 2.0) - 0.010, 0.0, 0.176)),
        material=trim,
        name="left_hinge_ridge",
    )
    body.visual(
        Box((0.012, 0.094, 0.014)),
        origin=Origin(xyz=((width / 2.0) + 0.010, 0.0, 0.176)),
        material=trim,
        name="right_hinge_ridge",
    )
    body.visual(
        Box((0.020, 0.032, 0.030)),
        origin=Origin(xyz=(-0.050, wheel_center[1], wheel_center[2])),
        material=trim,
        name="wheel_left_cheek",
    )
    body.visual(
        Box((0.020, 0.032, 0.030)),
        origin=Origin(xyz=(0.050, wheel_center[1], wheel_center[2])),
        material=trim,
        name="wheel_right_cheek",
    )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(
        Cylinder(radius=hinge_radius, length=0.016),
        origin=Origin(
            xyz=(0.0, -(handle_span / 2.0), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="left_handle_rear_barrel",
    )
    left_handle.visual(
        Cylinder(radius=hinge_radius, length=0.016),
        origin=Origin(
            xyz=(0.0, handle_span / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="left_handle_front_barrel",
    )
    left_handle.visual(
        Box((handle_bar, handle_arm_y, handle_drop)),
        origin=Origin(xyz=(0.0, -(handle_span / 2.0), -(handle_drop / 2.0))),
        material=handle_mat,
        name="left_handle_rear_arm",
    )
    left_handle.visual(
        Box((handle_bar, handle_arm_y, handle_drop)),
        origin=Origin(xyz=(0.0, handle_span / 2.0, -(handle_drop / 2.0))),
        material=handle_mat,
        name="left_handle_front_arm",
    )
    left_handle.visual(
        Box((handle_bar, handle_span, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -handle_drop)),
        material=handle_mat,
        name="left_handle_grip",
    )
    left_handle.inertial = Inertial.from_geometry(
        Box((0.014, handle_span + 0.010, handle_drop + 0.010)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -(handle_drop / 2.0))),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(
        Cylinder(radius=hinge_radius, length=0.016),
        origin=Origin(
            xyz=(0.0, -(handle_span / 2.0), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="right_handle_rear_barrel",
    )
    right_handle.visual(
        Cylinder(radius=hinge_radius, length=0.016),
        origin=Origin(
            xyz=(0.0, handle_span / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="right_handle_front_barrel",
    )
    right_handle.visual(
        Box((handle_bar, handle_arm_y, handle_drop)),
        origin=Origin(xyz=(0.0, -(handle_span / 2.0), -(handle_drop / 2.0))),
        material=handle_mat,
        name="right_handle_rear_arm",
    )
    right_handle.visual(
        Box((handle_bar, handle_arm_y, handle_drop)),
        origin=Origin(xyz=(0.0, handle_span / 2.0, -(handle_drop / 2.0))),
        material=handle_mat,
        name="right_handle_front_arm",
    )
    right_handle.visual(
        Box((handle_bar, handle_span, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -handle_drop)),
        material=handle_mat,
        name="right_handle_grip",
    )
    right_handle.inertial = Inertial.from_geometry(
        Box((0.014, handle_span + 0.010, handle_drop + 0.010)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -(handle_drop / 2.0))),
    )

    selector_wheel = model.part("selector_wheel")
    selector_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_mat,
        name="wheel_tire",
    )
    selector_wheel.visual(
        Cylinder(radius=0.0115, length=wheel_length + 0.004),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_hub_mat,
        name="wheel_core",
    )
    selector_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_length),
        mass=0.06,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_handle,
        origin=Origin(
            xyz=(
                -(width / 2.0) - mount_thickness - (handle_bar / 2.0),
                0.0,
                0.176,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_handle,
        origin=Origin(
            xyz=(
                (width / 2.0) + mount_thickness + (handle_bar / 2.0),
                0.0,
                0.176,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_wheel,
        origin=Origin(xyz=wheel_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
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
    body = object_model.get_part("speaker_body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    selector_wheel = object_model.get_part("selector_wheel")

    left_hinge = object_model.get_articulation("body_to_left_handle")
    right_hinge = object_model.get_articulation("body_to_right_handle")
    wheel_joint = object_model.get_articulation("body_to_selector_wheel")

    left_mount = body.get_visual("left_side_mount")
    right_mount = body.get_visual("right_side_mount")

    ctx.check(
        "portable speaker parts exist",
        all(part is not None for part in (body, left_handle, right_handle, selector_wheel)),
        details="Expected body, two side handles, and selector wheel parts.",
    )
    ctx.check(
        "handle hinge axes lift outward symmetrically",
        left_hinge.axis == (0.0, 1.0, 0.0) and right_hinge.axis == (0.0, -1.0, 0.0),
        details=f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "selector wheel spins across the speaker width",
        wheel_joint.axis == (1.0, 0.0, 0.0),
        details=f"wheel axis={wheel_joint.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            body,
            left_handle,
            axis="x",
            positive_elem=left_mount,
            max_gap=0.0015,
            max_penetration=1e-5,
            name="left handle folds flat against its side mount",
        )
        ctx.expect_gap(
            right_handle,
            body,
            axis="x",
            negative_elem=right_mount,
            max_gap=0.0015,
            max_penetration=1e-5,
            name="right handle folds flat against its side mount",
        )

        body_aabb = ctx.part_world_aabb(body)
        wheel_aabb = ctx.part_world_aabb(selector_wheel)
        wheel_pos = ctx.part_world_position(selector_wheel)
        body_pos = ctx.part_world_position(body)

        wheel_near_front_top = False
        if body_aabb is not None and wheel_aabb is not None and wheel_pos is not None and body_pos is not None:
            body_front = body_aabb[0][1]
            body_top = body_aabb[1][2]
            front_offset = wheel_pos[1] - body_front
            top_offset = body_top - wheel_pos[2]
            wheel_near_front_top = (
                abs(wheel_pos[0] - body_pos[0]) < 0.01
                and 0.0 <= front_offset <= 0.030
                and 0.0 <= top_offset <= 0.030
            )
        ctx.check(
            "selector wheel sits near the top front edge",
            wheel_near_front_top,
            details=(
                f"body_aabb={body_aabb}, wheel_aabb={wheel_aabb}, "
                f"wheel_pos={wheel_pos}, body_pos={body_pos}"
            ),
        )

        left_closed = ctx.part_world_aabb(left_handle)
        right_closed = ctx.part_world_aabb(right_handle)

    with ctx.pose({left_hinge: 1.1, right_hinge: 1.1}):
        left_open = ctx.part_world_aabb(left_handle)
        right_open = ctx.part_world_aabb(right_handle)

    left_lifts = (
        left_closed is not None
        and left_open is not None
        and left_open[0][2] > left_closed[0][2] + 0.028
        and left_open[0][0] < left_closed[0][0] - 0.040
    )
    right_lifts = (
        right_closed is not None
        and right_open is not None
        and right_open[0][2] > right_closed[0][2] + 0.028
        and right_open[1][0] > right_closed[1][0] + 0.040
    )
    ctx.check(
        "left handle swings up and away from the cabinet",
        left_lifts,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right handle swings up and away from the cabinet",
        right_lifts,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
