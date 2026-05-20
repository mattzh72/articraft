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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xz_section(
    y: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + height * 0.5)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _yz_section(
    x: float,
    *,
    depth: float,
    height: float,
    radius: float,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + center_y, z + base_z + height * 0.5)
        for y, z in rounded_rect_profile(depth, height, radius, corner_segments=8)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_speaker")

    housing = model.material("housing_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    grille = model.material("grille_black", rgba=(0.08, 0.09, 0.10, 1.0))
    trim = model.material("trim_graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    handle_mat = model.material("handle_graphite", rgba=(0.34, 0.36, 0.38, 1.0))
    knob_mat = model.material("knob_black", rgba=(0.12, 0.13, 0.14, 1.0))
    indicator_mat = model.material("indicator_orange", rgba=(0.93, 0.50, 0.16, 1.0))

    body = model.part("body")

    body_sections = [
        _xz_section(-0.078, width=0.298, height=0.198, radius=0.026),
        _xz_section(0.000, width=0.304, height=0.208, radius=0.030),
        _xz_section(0.078, width=0.294, height=0.194, radius=0.024),
    ]
    shell_mesh = mesh_from_geometry(section_loft(body_sections), "speaker_body_shell")
    body.visual(shell_mesh, material=housing, name="main_shell")
    body.visual(
        Box((0.262, 0.006, 0.142)),
        origin=Origin(xyz=(0.0, -0.081, 0.102)),
        material=grille,
        name="front_grille",
    )
    pod_roll = 0.0
    body.visual(
        Box((0.138, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, -0.044, 0.202), rpy=(pod_roll, 0.0, 0.0)),
        material=trim,
        name="control_pod",
    )
    body.visual(
        Box((0.012, 0.022, 0.050)),
        origin=Origin(xyz=(-0.153, -0.003, 0.178)),
        material=trim,
        name="left_pivot_mount",
    )
    body.visual(
        Box((0.012, 0.022, 0.050)),
        origin=Origin(xyz=(0.153, -0.003, 0.178)),
        material=trim,
        name="right_pivot_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.304, 0.160, 0.208)),
        mass=3.9,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
    )

    top_bar = model.part("top_bar")
    top_bar.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(-0.160, depth=0.028, height=0.018, radius=0.006, center_y=0.070, base_z=0.033),
                    _yz_section(0.000, depth=0.032, height=0.018, radius=0.007, center_y=0.070, base_z=0.033),
                    _yz_section(0.160, depth=0.028, height=0.018, radius=0.006, center_y=0.070, base_z=0.033),
                ]
            ),
            "speaker_handle_grip",
        ),
        material=handle_mat,
        name="bar_grip",
    )
    top_bar.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _xz_section(0.000, width=0.014, height=0.036, radius=0.004),
                    _xz_section(0.050, width=0.015, height=0.040, radius=0.004),
                    _xz_section(0.094, width=0.018, height=0.046, radius=0.005),
                ]
            ),
            "speaker_handle_left_arm",
        ),
        origin=Origin(xyz=(-0.166, 0.0, 0.0)),
        material=handle_mat,
        name="left_arm",
    )
    top_bar.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _xz_section(0.000, width=0.014, height=0.036, radius=0.004),
                    _xz_section(0.050, width=0.015, height=0.040, radius=0.004),
                    _xz_section(0.094, width=0.018, height=0.046, radius=0.005),
                ]
            ),
            "speaker_handle_right_arm",
        ),
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
        material=handle_mat,
        name="right_arm",
    )
    top_bar.inertial = Inertial.from_geometry(
        Box((0.334, 0.096, 0.062)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.048, 0.031)),
    )

    model.articulation(
        "body_to_top_bar",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_bar,
        origin=Origin(xyz=(0.0, -0.010, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    knob_x_positions = (-0.040, 0.0, 0.040)
    pod_center_y = -0.044
    pod_center_z = 0.202
    pod_top_local_y = 0.0
    pod_half_height = 0.010
    knob_mount_y = pod_center_y + pod_top_local_y * math.cos(pod_roll) - pod_half_height * math.sin(pod_roll)
    knob_mount_z = pod_center_z + pod_top_local_y * math.sin(pod_roll) + pod_half_height * math.cos(pod_roll)

    for index, knob_x in enumerate(knob_x_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.017, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=trim,
            name="base_collar",
        )
        knob.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=knob_mat,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=trim,
            name="knob_cap",
        )
        knob.visual(
            Box((0.003, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, 0.0075, 0.021)),
            material=indicator_mat,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.017, length=0.023),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        )

        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_mount_y, knob_mount_z), rpy=(pod_roll, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=8.0,
                lower=-2.35,
                upper=2.35,
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

    body = object_model.get_part("body")
    top_bar = object_model.get_part("top_bar")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 4)]

    top_bar_joint = object_model.get_articulation("body_to_top_bar")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(1, 4)]

    ctx.check(
        "speaker has one handle joint and three knob joints",
        len(knob_joints) == 3 and top_bar_joint.parent == "body" and top_bar_joint.child == "top_bar",
        details=f"handle_parent={top_bar_joint.parent}, handle_child={top_bar_joint.child}, knob_count={len(knob_joints)}",
    )

    for index, knob in enumerate(knobs, start=1):
        knob_joint = knob_joints[index - 1]
        limits = knob_joint.motion_limits
        ctx.check(
            f"knob {index} has a rotary joint with travel",
            knob_joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.upper - limits.lower > 4.0,
            details=f"type={knob_joint.articulation_type}, limits={limits}",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a="base_collar",
            elem_b="control_pod",
            contact_tol=5e-4,
            name=f"knob {index} seats on the control pod",
        )

    ctx.expect_contact(
        top_bar,
        body,
        elem_a="left_arm",
        elem_b="left_pivot_mount",
        contact_tol=5e-5,
        name="left handle arm bears on the left pivot mount",
    )
    ctx.expect_contact(
        top_bar,
        body,
        elem_a="right_arm",
        elem_b="right_pivot_mount",
        contact_tol=5e-5,
        name="right handle arm bears on the right pivot mount",
    )

    rest_grip_center = _aabb_center(ctx.part_element_world_aabb(top_bar, elem="bar_grip"))
    with ctx.pose({top_bar_joint: top_bar_joint.motion_limits.upper}):
        open_grip_center = _aabb_center(ctx.part_element_world_aabb(top_bar, elem="bar_grip"))

    ctx.check(
        "top bar lifts when opened",
        rest_grip_center is not None
        and open_grip_center is not None
        and open_grip_center[2] > rest_grip_center[2] + 0.03,
        details=f"rest={rest_grip_center}, open={open_grip_center}",
    )

    rest_indicator_centers = [
        _aabb_center(ctx.part_element_world_aabb(knob, elem="indicator")) for knob in knobs
    ]
    turned_position = 1.1

    for index, knob in enumerate(knobs):
        neighbor_index = (index + 1) % len(knobs)
        with ctx.pose({knob_joints[index]: turned_position}):
            moved_indicator = _aabb_center(ctx.part_element_world_aabb(knob, elem="indicator"))
            neighbor_indicator = _aabb_center(
                ctx.part_element_world_aabb(knobs[neighbor_index], elem="indicator")
            )

        moved_distance = None
        neighbor_distance = None
        if rest_indicator_centers[index] is not None and moved_indicator is not None:
            moved_distance = math.hypot(
                moved_indicator[0] - rest_indicator_centers[index][0],
                moved_indicator[1] - rest_indicator_centers[index][1],
            )
        if rest_indicator_centers[neighbor_index] is not None and neighbor_indicator is not None:
            neighbor_distance = math.hypot(
                neighbor_indicator[0] - rest_indicator_centers[neighbor_index][0],
                neighbor_indicator[1] - rest_indicator_centers[neighbor_index][1],
            )

        ctx.check(
            f"knob {index + 1} indicator turns with its shaft",
            moved_distance is not None and moved_distance > 0.005,
            details=f"rest={rest_indicator_centers[index]}, moved={moved_indicator}, delta={moved_distance}",
        )
        ctx.check(
            f"knob {index + 1} turns independently",
            neighbor_distance is not None and neighbor_distance < 5e-4,
            details=(
                f"neighbor_rest={rest_indicator_centers[neighbor_index]}, "
                f"neighbor_pose={neighbor_indicator}, delta={neighbor_distance}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
