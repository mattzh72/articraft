from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arch_loop(
    x_pos: float,
    width: float,
    height: float,
    *,
    base_z: float,
    arch_samples: int = 18,
    floor_samples: int = 8,
) -> list[tuple[float, float, float]]:
    half_width = 0.5 * width
    loop: list[tuple[float, float, float]] = []
    for index in range(arch_samples + 1):
        theta = math.pi - (math.pi * index / arch_samples)
        y_pos = half_width * math.cos(theta)
        z_pos = base_z + height * math.sin(theta)
        loop.append((x_pos, y_pos, z_pos))
    for index in range(1, floor_samples):
        frac = index / floor_samples
        y_pos = half_width - (width * frac)
        loop.append((x_pos, y_pos, base_z))
    return loop


def _append_loop(
    geom: MeshGeometry,
    loop: list[tuple[float, float, float]],
) -> list[int]:
    return [geom.add_vertex(*point) for point in loop]


def _bridge_loops(
    geom: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(loop_a)
    if count != len(loop_b):
        raise ValueError("Loop sizes must match for bridging.")
    for index in range(count):
        next_index = (index + 1) % count
        a0 = loop_a[index]
        a1 = loop_a[next_index]
        b0 = loop_b[index]
        b1 = loop_b[next_index]
        if reverse:
            geom.add_face(a0, b1, a1)
            geom.add_face(a0, b0, b1)
        else:
            geom.add_face(a0, a1, b1)
            geom.add_face(a0, b1, b0)


def _cap_loop(
    geom: MeshGeometry,
    loop_points: list[tuple[float, float, float]],
    loop_ids: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(loop_points)
    center = (
        sum(point[0] for point in loop_points) / count,
        sum(point[1] for point in loop_points) / count,
        sum(point[2] for point in loop_points) / count,
    )
    center_id = geom.add_vertex(*center)
    for index in range(count):
        next_index = (index + 1) % count
        a0 = loop_ids[index]
        a1 = loop_ids[next_index]
        if reverse:
            geom.add_face(center_id, a1, a0)
        else:
            geom.add_face(center_id, a0, a1)


def _build_oven_shell_geometry() -> MeshGeometry:
    floor_thickness = 0.06
    outer_specs = (
        (0.38, 0.84, 0.60),
        (0.22, 0.94, 0.70),
        (-0.05, 0.98, 0.74),
        (-0.28, 0.92, 0.68),
        (-0.50, 0.80, 0.56),
    )
    inner_specs = (
        (0.32, 0.60, 0.38),
        (0.16, 0.72, 0.46),
        (-0.08, 0.76, 0.48),
        (-0.28, 0.68, 0.42),
        (-0.42, 0.54, 0.24),
    )

    outer_loops = [
        _arch_loop(x_pos, width, height, base_z=0.0)
        for x_pos, width, height in outer_specs
    ]
    inner_loops = [
        _arch_loop(x_pos, width, height, base_z=floor_thickness)
        for x_pos, width, height in inner_specs
    ]

    shell = MeshGeometry()
    outer_ids = [_append_loop(shell, loop) for loop in outer_loops]
    inner_ids = [_append_loop(shell, loop) for loop in inner_loops]

    for index in range(len(outer_ids) - 1):
        _bridge_loops(shell, outer_ids[index], outer_ids[index + 1])

    for index in range(len(inner_ids) - 1):
        _bridge_loops(shell, inner_ids[index + 1], inner_ids[index])

    _bridge_loops(shell, inner_ids[-1], outer_ids[-1], reverse=True)

    _cap_loop(shell, outer_loops[-1], outer_ids[-1], reverse=True)
    _cap_loop(shell, inner_loops[-1], inner_ids[-1], reverse=False)

    shell.merge(BoxGeometry((0.02, 0.055, 0.04)).translate(0.50, -0.22, 0.06))
    shell.merge(BoxGeometry((0.02, 0.055, 0.04)).translate(0.50, 0.22, 0.06))
    shell.merge(
        CylinderGeometry(radius=0.011, height=0.055)
        .rotate_x(math.pi / 2.0)
        .translate(0.509, -0.22, 0.06)
    )
    shell.merge(
        CylinderGeometry(radius=0.011, height=0.055)
        .rotate_x(math.pi / 2.0)
        .translate(0.509, 0.22, 0.06)
    )

    return shell


def _arched_profile_2d(
    width: float,
    height: float,
    *,
    arch_samples: int = 18,
    floor_samples: int = 8,
) -> list[tuple[float, float]]:
    half_width = 0.5 * width
    profile: list[tuple[float, float]] = []
    for index in range(arch_samples + 1):
        theta = math.pi - (math.pi * index / arch_samples)
        y_pos = half_width * math.cos(theta)
        z_pos = height * math.sin(theta)
        profile.append((-z_pos, y_pos))
    for index in range(1, floor_samples):
        frac = index / floor_samples
        y_pos = half_width - (width * frac)
        profile.append((0.0, y_pos))
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_fired_bread_oven")

    stone = model.material("stone", rgba=(0.66, 0.61, 0.56, 1.0))
    slab_stone = model.material("slab_stone", rgba=(0.74, 0.70, 0.66, 1.0))
    firebrick = model.material("firebrick", rgba=(0.74, 0.56, 0.42, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.35, 0.22, 0.12, 1.0))

    hearth_platform = model.part("hearth_platform")
    hearth_platform.visual(
        Box((1.04, 1.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=stone,
        name="plinth",
    )
    hearth_platform.visual(
        Box((0.88, 0.20, 0.60)),
        origin=Origin(xyz=(-0.04, -0.55, 0.40)),
        material=stone,
        name="left_pier",
    )
    hearth_platform.visual(
        Box((0.88, 0.20, 0.60)),
        origin=Origin(xyz=(-0.04, 0.55, 0.40)),
        material=stone,
        name="right_pier",
    )
    hearth_platform.visual(
        Box((0.16, 0.90, 0.60)),
        origin=Origin(xyz=(-0.44, 0.0, 0.40)),
        material=stone,
        name="rear_wall",
    )
    hearth_platform.visual(
        Box((1.12, 1.38, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=slab_stone,
        name="slab",
    )
    hearth_platform.visual(
        Box((0.38, 0.03, 0.03)),
        origin=Origin(xyz=(0.22, -0.30, 0.685)),
        material=stone,
        name="guide_left",
    )
    hearth_platform.visual(
        Box((0.38, 0.03, 0.03)),
        origin=Origin(xyz=(0.22, 0.30, 0.685)),
        material=stone,
        name="guide_right",
    )
    hearth_platform.inertial = Inertial.from_geometry(
        Box((1.12, 1.38, 0.78)),
        mass=430.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    hearth_deck = model.part("hearth_deck")
    hearth_deck.visual(
        Box((1.00, 1.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=firebrick,
        name="deck_surface",
    )
    hearth_deck.visual(
        Box((0.10, 1.10, 0.03)),
        origin=Origin(xyz=(0.45, 0.0, 0.015)),
        material=firebrick,
        name="landing_nose",
    )
    hearth_deck.inertial = Inertial.from_geometry(
        Box((1.00, 1.10, 0.06)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    model.articulation(
        "platform_to_deck",
        ArticulationType.FIXED,
        parent=hearth_platform,
        child=hearth_deck,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    oven_body = model.part("oven_body")
    oven_body.visual(
        mesh_from_geometry(_build_oven_shell_geometry(), "oven_shell_with_hinges_v4"),
        material=stone,
        name="shell",
    )
    oven_body.inertial = Inertial.from_geometry(
        Box((0.99, 0.98, 0.74)),
        mass=340.0,
        origin=Origin(xyz=(-0.005, 0.0, 0.37)),
    )

    model.articulation(
        "deck_to_oven",
        ArticulationType.FIXED,
        parent=hearth_deck,
        child=oven_body,
        origin=Origin(xyz=(-0.03, 0.0, 0.06)),
    )

    door_assembly = model.part("door_assembly")
    door_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_arched_profile_2d(0.58, 0.36), 0.015).rotate_y(math.pi / 2.0),
        "door_leaf_panel_v5",
    )
    door_assembly.visual(
        door_mesh,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=cast_iron,
        name="door_leaf",
    )
    door_assembly.visual(
        Cylinder(radius=0.008, length=0.045),
        origin=Origin(xyz=(0.022, -0.22, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pin_left",
    )
    door_assembly.visual(
        Cylinder(radius=0.008, length=0.045),
        origin=Origin(xyz=(0.022, 0.22, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pin_right",
    )
    door_assembly.visual(
        Box((0.012, 0.045, 0.016)),
        origin=Origin(xyz=(0.036, -0.22, 0.008)),
        material=cast_iron,
        name="pin_bridge_left",
    )
    door_assembly.visual(
        Box((0.012, 0.045, 0.016)),
        origin=Origin(xyz=(0.036, 0.22, 0.008)),
        material=cast_iron,
        name="pin_bridge_right",
    )
    door_assembly.visual(
        Cylinder(radius=0.009, length=0.042),
        origin=Origin(xyz=(0.050, -0.09, 0.17), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="handle_post_left",
    )
    door_assembly.visual(
        Cylinder(radius=0.009, length=0.042),
        origin=Origin(xyz=(0.050, 0.09, 0.17), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="handle_post_right",
    )
    door_assembly.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.071, 0.0, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="handle_grip",
    )
    door_assembly.inertial = Inertial.from_geometry(
        Box((0.12, 0.58, 0.36)),
        mass=24.0,
        origin=Origin(xyz=(0.06, 0.0, 0.18)),
    )

    model.articulation(
        "oven_to_door",
        ArticulationType.REVOLUTE,
        parent=oven_body,
        child=door_assembly,
        origin=Origin(xyz=(0.49, 0.0, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(90.0),
        ),
    )

    ash_drawer = model.part("ash_drawer")
    ash_drawer.visual(
        Box((0.42, 0.59, 0.015)),
        origin=Origin(xyz=(-0.21, 0.0, 0.0075)),
        material=cast_iron,
        name="tray_bottom",
    )
    ash_drawer.visual(
        Box((0.42, 0.015, 0.11)),
        origin=Origin(xyz=(-0.21, -0.3025, 0.055)),
        material=cast_iron,
        name="tray_side_left",
    )
    ash_drawer.visual(
        Box((0.42, 0.015, 0.11)),
        origin=Origin(xyz=(-0.21, 0.3025, 0.055)),
        material=cast_iron,
        name="tray_side_right",
    )
    ash_drawer.visual(
        Box((0.015, 0.59, 0.11)),
        origin=Origin(xyz=(-0.4125, 0.0, 0.055)),
        material=cast_iron,
        name="tray_back",
    )
    ash_drawer.visual(
        Box((0.02, 0.62, 0.10)),
        origin=Origin(xyz=(-0.01, 0.0, 0.05)),
        material=cast_iron,
        name="drawer_front",
    )
    ash_drawer.visual(
        Box((0.36, 0.025, 0.03)),
        origin=Origin(xyz=(-0.18, -0.29, 0.105)),
        material=cast_iron,
        name="runner_left",
    )
    ash_drawer.visual(
        Box((0.36, 0.025, 0.03)),
        origin=Origin(xyz=(-0.18, 0.29, 0.105)),
        material=cast_iron,
        name="runner_right",
    )
    ash_drawer.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(xyz=(0.012, -0.07, 0.06), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="pull_post_left",
    )
    ash_drawer.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(xyz=(0.012, 0.07, 0.06), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="pull_post_right",
    )
    ash_drawer.visual(
        Cylinder(radius=0.010, length=0.16),
        origin=Origin(xyz=(0.03, 0.0, 0.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="pull_grip",
    )
    ash_drawer.inertial = Inertial.from_geometry(
        Box((0.42, 0.62, 0.12)),
        mass=13.0,
        origin=Origin(xyz=(-0.20, 0.0, 0.06)),
    )

    model.articulation(
        "platform_to_ash_drawer",
        ArticulationType.PRISMATIC,
        parent=hearth_platform,
        child=ash_drawer,
        origin=Origin(xyz=(0.41, 0.0, 0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.12,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hearth_platform = object_model.get_part("hearth_platform")
    hearth_deck = object_model.get_part("hearth_deck")
    oven_body = object_model.get_part("oven_body")
    door_assembly = object_model.get_part("door_assembly")
    ash_drawer = object_model.get_part("ash_drawer")

    oven_to_door = object_model.get_articulation("oven_to_door")
    platform_to_ash_drawer = object_model.get_articulation("platform_to_ash_drawer")

    shell = oven_body.get_visual("shell")
    door_leaf = door_assembly.get_visual("door_leaf")
    pin_left = door_assembly.get_visual("pin_left")
    pin_right = door_assembly.get_visual("pin_right")
    guide_left = hearth_platform.get_visual("guide_left")
    guide_right = hearth_platform.get_visual("guide_right")
    runner_left = ash_drawer.get_visual("runner_left")
    runner_right = ash_drawer.get_visual("runner_right")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        door_assembly,
        oven_body,
        elem_a=pin_left,
        elem_b=shell,
        reason="The left hinge pin rotates inside a cast-iron hinge boss on the oven face.",
    )
    ctx.allow_overlap(
        door_assembly,
        oven_body,
        elem_a=pin_right,
        elem_b=shell,
        reason="The right hinge pin rotates inside a cast-iron hinge boss on the oven face.",
    )
    ctx.allow_overlap(
        ash_drawer,
        hearth_platform,
        elem_a=runner_left,
        elem_b=guide_left,
        reason="The left runner remains captured inside its slide guide across drawer travel.",
    )
    ctx.allow_overlap(
        ash_drawer,
        hearth_platform,
        elem_a=runner_right,
        elem_b=guide_right,
        reason="The right runner remains captured inside its slide guide across drawer travel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(hearth_deck, hearth_platform)
    ctx.expect_contact(oven_body, hearth_deck)
    ctx.expect_contact(door_assembly, oven_body, elem_a=pin_left, elem_b=shell)
    ctx.expect_contact(door_assembly, oven_body, elem_a=pin_right, elem_b=shell)
    ctx.expect_contact(ash_drawer, hearth_platform, elem_a=runner_left, elem_b=guide_left)
    ctx.expect_contact(ash_drawer, hearth_platform, elem_a=runner_right, elem_b=guide_right)
    ctx.expect_within(oven_body, hearth_deck, axes="xy", margin=0.08)

    ctx.check(
        "door_hinge_axis_is_horizontal",
        tuple(oven_to_door.axis) == (0.0, 1.0, 0.0),
        details=f"Expected (0.0, 1.0, 0.0), got {oven_to_door.axis}",
    )
    ctx.check(
        "ash_drawer_axis_is_forward",
        tuple(platform_to_ash_drawer.axis) == (1.0, 0.0, 0.0),
        details=f"Expected (1.0, 0.0, 0.0), got {platform_to_ash_drawer.axis}",
    )

    with ctx.pose({oven_to_door: 0.0}):
        ctx.expect_overlap(
            door_assembly,
            oven_body,
            axes="yz",
            min_overlap=0.30,
            elem_a=door_leaf,
            elem_b=shell,
            name="door_closed_covers_arch",
        )

    door_limits = oven_to_door.motion_limits
    assert door_limits is not None and door_limits.lower is not None and door_limits.upper is not None
    with ctx.pose({oven_to_door: door_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_lower_pose_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="door_lower_pose_no_floating")
    with ctx.pose({oven_to_door: door_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_upper_pose_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="door_upper_pose_no_floating")
        ctx.expect_gap(
            door_assembly,
            hearth_deck,
            axis="z",
            min_gap=0.0,
            positive_elem=door_leaf,
            negative_elem="deck_surface",
            name="door_open_clears_hearth_landing",
        )

    drawer_limits = platform_to_ash_drawer.motion_limits
    assert drawer_limits is not None and drawer_limits.lower is not None and drawer_limits.upper is not None
    with ctx.pose({platform_to_ash_drawer: drawer_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_lower_pose_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="drawer_lower_pose_no_floating")
    with ctx.pose({platform_to_ash_drawer: drawer_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_upper_pose_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="drawer_upper_pose_no_floating")
        ctx.expect_contact(ash_drawer, hearth_platform, elem_a=runner_left, elem_b=guide_left)
        ctx.expect_contact(ash_drawer, hearth_platform, elem_a=runner_right, elem_b=guide_right)

    closed_door_aabb = ctx.part_world_aabb(door_assembly)
    assert closed_door_aabb is not None
    with ctx.pose({oven_to_door: door_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(door_assembly)
        assert open_door_aabb is not None
        ctx.check(
            "door_swings_forward_on_open",
            open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.20,
            details=f"Closed max x {closed_door_aabb[1][0]:.3f}, open max x {open_door_aabb[1][0]:.3f}",
        )
        ctx.check(
            "door_lowers_when_open",
            open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.20,
            details=f"Closed max z {closed_door_aabb[1][2]:.3f}, open max z {open_door_aabb[1][2]:.3f}",
        )

    closed_drawer_aabb = ctx.part_world_aabb(ash_drawer)
    assert closed_drawer_aabb is not None
    with ctx.pose({platform_to_ash_drawer: drawer_limits.upper}):
        open_drawer_aabb = ctx.part_world_aabb(ash_drawer)
        assert open_drawer_aabb is not None
        ctx.check(
            "ash_drawer_slides_outward",
            open_drawer_aabb[1][0] > closed_drawer_aabb[1][0] + 0.20,
            details=f"Closed max x {closed_drawer_aabb[1][0]:.3f}, open max x {open_drawer_aabb[1][0]:.3f}",
        )

    with ctx.pose({oven_to_door: door_limits.upper, platform_to_ash_drawer: drawer_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_open_pose_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
