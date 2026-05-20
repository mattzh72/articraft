from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    attempts = (
        ((), {"name": name, "color": rgba}),
        ((), {"name": name, "rgba": rgba}),
        ((name,), {"color": rgba}),
        ((name,), {"rgba": rgba}),
        ((name, rgba), {}),
    )
    for args, kwargs in attempts:
        try:
            return Material(*args, **kwargs)
        except TypeError:
            continue
    return None


def _arched_profile(width: float, side_height: float, arc_segments: int = 28):
    radius = width * 0.5
    points = [(-radius, 0.0), (radius, 0.0), (radius, side_height)]
    for step in range(1, arc_segments):
        theta = math.pi * step / arc_segments
        points.append((radius * math.cos(theta), side_height + radius * math.sin(theta)))
    points.append((-radius, side_height))
    return points


def _profile_at_y(profile, y: float):
    return [(x, y, z) for x, z in profile]


def _aabb_bounds(aabb):
    scalar_layouts = (
        ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax"),
        ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z"),
    )
    for names in scalar_layouts:
        if all(hasattr(aabb, name) for name in names):
            return tuple(float(getattr(aabb, name)) for name in names)

    for min_name, max_name in (("min", "max"), ("minimum", "maximum")):
        if hasattr(aabb, min_name) and hasattr(aabb, max_name):
            mn = getattr(aabb, min_name)
            mx = getattr(aabb, max_name)
            if len(mn) == 3 and len(mx) == 3:
                return (
                    float(mn[0]),
                    float(mx[0]),
                    float(mn[1]),
                    float(mx[1]),
                    float(mn[2]),
                    float(mx[2]),
                )

    if isinstance(aabb, dict):
        keys = ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")
        if all(key in aabb for key in keys):
            return tuple(float(aabb[key]) for key in keys)

    if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
        mn, mx = aabb
        if len(mn) == 3 and len(mx) == 3:
            return (
                float(mn[0]),
                float(mx[0]),
                float(mn[1]),
                float(mx[1]),
                float(mn[2]),
                float(mx[2]),
            )

    raise TypeError(f"Unsupported AABB layout: {type(aabb)!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_mailbox", assets=ASSETS)

    painted_green = _make_material("painted_green", (0.19, 0.30, 0.20, 1.0))
    trim_black = _make_material("trim_black", (0.10, 0.10, 0.11, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.72, 0.74, 0.76, 1.0))
    cedar_post = _make_material("cedar_post", (0.50, 0.35, 0.22, 1.0))
    signal_red = _make_material("signal_red", (0.73, 0.10, 0.10, 1.0))

    body_width = 0.23
    body_length = 0.48
    side_height = 0.074
    frame_thickness = 0.012
    rear_cap_thickness = 0.014
    opening_width = 0.216
    opening_side_height = 0.068
    door_width = 0.210
    door_side_height = 0.066
    door_thickness = 0.018

    outer_profile = _arched_profile(body_width, side_height)
    opening_profile = _arched_profile(opening_width, opening_side_height)
    door_profile = _arched_profile(door_width, door_side_height)

    shell_geom = ExtrudeGeometry(
        outer_profile,
        height=body_length,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi * 0.5)
    shell_mesh = mesh_from_geometry(shell_geom, ASSETS.mesh_path("mailbox_shell.obj"))

    rear_cap_geom = ExtrudeGeometry(
        outer_profile,
        height=rear_cap_thickness,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi * 0.5)
    rear_cap_mesh = mesh_from_geometry(rear_cap_geom, ASSETS.mesh_path("mailbox_rear_cap.obj"))

    frame_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [opening_profile],
        height=frame_thickness,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi * 0.5)
    frame_mesh = mesh_from_geometry(frame_geom, ASSETS.mesh_path("mailbox_front_frame.obj"))

    door_geom = ExtrudeGeometry(
        door_profile,
        height=door_thickness,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi * 0.5)
    door_mesh = mesh_from_geometry(door_geom, ASSETS.mesh_path("mailbox_door.obj"))

    support = model.part("support")
    support.visual(
        Box((0.09, 0.09, 0.824)),
        origin=Origin(xyz=(0.0, -0.06, 0.412)),
        material=cedar_post,
    )
    support.visual(
        Box((0.28, 0.26, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.831)),
        material=cedar_post,
    )
    support.visual(
        Box((0.07, 0.30, 0.05)),
        origin=Origin(xyz=(0.0, 0.02, 0.797)),
        material=cedar_post,
    )
    support.visual(
        Box((0.022, 0.19, 0.03)),
        origin=Origin(xyz=(0.028, -0.025, 0.725), rpy=(-0.82, 0.0, 0.0)),
        material=cedar_post,
    )
    support.visual(
        Box((0.022, 0.19, 0.03)),
        origin=Origin(xyz=(-0.028, -0.025, 0.725), rpy=(-0.82, 0.0, 0.0)),
        material=cedar_post,
    )
    support.inertial = Inertial.from_geometry(
        Box((0.30, 0.36, 0.86)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -0.01, 0.43)),
    )

    body = model.part("body")
    body.visual(shell_mesh, material=painted_green)
    body.visual(
        rear_cap_mesh,
        origin=Origin(xyz=(0.0, -body_length * 0.5 + rear_cap_thickness * 0.5, 0.0)),
        material=painted_green,
    )
    body.visual(
        frame_mesh,
        origin=Origin(xyz=(0.0, body_length * 0.5 - frame_thickness * 0.5, 0.0)),
        material=trim_black,
    )
    body.visual(
        Box((0.18, 0.35, 0.018)),
        origin=Origin(xyz=(0.0, -0.01, 0.009)),
        material=trim_black,
    )
    body.visual(
        Box((0.16, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.220, 0.178)),
        material=trim_black,
    )
    body.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(
            xyz=(body_width * 0.5 + 0.006, 0.070, 0.078),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_black,
    )
    body.visual(
        Box((0.004, 0.036, 0.074)),
        origin=Origin(xyz=(body_width * 0.5 + 0.011, 0.069, 0.053)),
        material=signal_red,
    )
    body.visual(
        Box((0.004, 0.082, 0.032)),
        origin=Origin(xyz=(body_width * 0.5 + 0.013, 0.102, 0.091)),
        material=signal_red,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.25, 0.50, 0.20)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    door = model.part("door")
    door.visual(
        door_mesh,
        origin=Origin(xyz=(0.0, door_thickness * 0.5, 0.0)),
        material=painted_green,
    )
    door.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(
            xyz=(0.0, door_thickness + 0.002, 0.080),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=trim_black,
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(
            xyz=(0.0, door_thickness + 0.010, 0.080),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brushed_steel,
    )
    door.visual(
        Box((0.030, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, door_thickness + 0.007, 0.065)),
        material=brushed_steel,
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_side_height + door_width * 0.5)),
        mass=0.45,
        origin=Origin(xyz=(0.0, door_thickness * 0.5, 0.085)),
    )

    model.articulation(
        "support_to_body",
        ArticulationType.FIXED,
        parent="support",
        child="body",
        origin=Origin(xyz=(0.0, 0.0, 0.843)),
    )
    model.articulation(
        "front_access_door",
        ArticulationType.REVOLUTE,
        parent="body",
        child="door",
        origin=Origin(xyz=(0.0, body_length * 0.5, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "body",
        "door",
        reason="The open shell and front frame generate conservative collision hulls around the access opening.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("body", "support", axes="xy", min_overlap=0.08)
    ctx.expect_origin_distance("body", "support", axes="xy", max_dist=0.09)
    ctx.expect_aabb_gap("body", "support", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "front_access_door",
        "door",
        world_axis="z",
        direction="negative",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "front_access_door",
        "door",
        world_axis="y",
        direction="positive",
        min_delta=0.05,
    )

    body_bounds = _aabb_bounds(ctx.part_world_aabb("body"))
    support_bounds = _aabb_bounds(ctx.part_world_aabb("support"))
    door_closed_bounds = _aabb_bounds(ctx.part_world_aabb("door"))

    body_width = body_bounds[1] - body_bounds[0]
    body_length = body_bounds[3] - body_bounds[2]
    body_height = body_bounds[5] - body_bounds[4]
    support_height = support_bounds[5] - support_bounds[4]

    assert body_length > body_width + 0.16, (
        "Mailbox body should read as a longitudinal roadside box."
    )
    assert body_height > 0.17, "Mailbox body should have a tall arched weather hood."
    assert support_height > 0.80, "Support should provide believable curbside mounting presence."
    assert abs(body_bounds[4] - support_bounds[5]) < 0.01, (
        "Mailbox body should sit directly on the support shelf."
    )
    assert abs(door_closed_bounds[2] - body_bounds[3]) < 0.025, (
        "Closed door should sit at the front opening."
    )
    assert door_closed_bounds[5] > body_bounds[4] + 0.16, (
        "Closed door should cover most of the front face."
    )

    with ctx.pose(front_access_door=0.75):
        door_mid_bounds = _aabb_bounds(ctx.part_world_aabb("door"))
        assert door_mid_bounds[5] < door_closed_bounds[5] - 0.03, (
            "Partially open door should drop from the closed position."
        )
        assert door_mid_bounds[3] > door_closed_bounds[3] + 0.035, (
            "Partially open door should project forward."
        )

    with ctx.pose(front_access_door=1.45):
        door_open_bounds = _aabb_bounds(ctx.part_world_aabb("door"))
        assert door_open_bounds[5] < door_mid_bounds[5] - 0.02, (
            "Fully open door should swing farther downward than the mid pose."
        )
        assert door_open_bounds[3] > door_mid_bounds[3] + 0.04, (
            "Fully open door should extend farther forward than the mid pose."
        )
        assert door_open_bounds[2] > support_bounds[3] + 0.045, (
            "Open door should clear the support by swinging out in front of the mounting shelf."
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
