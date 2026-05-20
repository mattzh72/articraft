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
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _ring_loop(
    *,
    x: float,
    radius_y: float,
    radius_z: float,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius_y * math.cos((2.0 * math.pi * index) / segments),
            radius_z * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _append_loop(
    geometry: MeshGeometry,
    loop: list[tuple[float, float, float]],
) -> list[int]:
    return [geometry.add_vertex(*point) for point in loop]


def _connect_loops(
    geometry: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        a0 = loop_a[index]
        a1 = loop_a[next_index]
        b0 = loop_b[index]
        b1 = loop_b[next_index]
        if reverse:
            geometry.add_face(a0, b1, b0)
            geometry.add_face(a0, a1, b1)
        else:
            geometry.add_face(a0, b0, b1)
            geometry.add_face(a0, b1, a1)


def _shade_shell_mesh(
    *,
    length: float,
    outer_back: tuple[float, float],
    outer_front: tuple[float, float],
    inner_back: tuple[float, float],
    inner_front: tuple[float, float],
    segments: int = 40,
) -> MeshGeometry:
    geometry = MeshGeometry()

    outer_back_ids = _append_loop(
        geometry,
        _ring_loop(
            x=0.0,
            radius_y=outer_back[0],
            radius_z=outer_back[1],
            segments=segments,
        ),
    )
    outer_front_ids = _append_loop(
        geometry,
        _ring_loop(
            x=length,
            radius_y=outer_front[0],
            radius_z=outer_front[1],
            segments=segments,
        ),
    )
    inner_front_ids = _append_loop(
        geometry,
        _ring_loop(
            x=length,
            radius_y=inner_front[0],
            radius_z=inner_front[1],
            segments=segments,
        ),
    )
    inner_back_ids = _append_loop(
        geometry,
        _ring_loop(
            x=0.0,
            radius_y=inner_back[0],
            radius_z=inner_back[1],
            segments=segments,
        ),
    )

    _connect_loops(geometry, outer_back_ids, outer_front_ids, reverse=False)
    _connect_loops(geometry, inner_front_ids, inner_back_ids, reverse=False)
    _connect_loops(geometry, outer_back_ids, inner_back_ids, reverse=True)
    return geometry


def _tube_shell_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 32,
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_back_ids = _append_loop(
        geometry,
        _ring_loop(x=0.0, radius_y=outer_radius, radius_z=outer_radius, segments=segments),
    )
    outer_front_ids = _append_loop(
        geometry,
        _ring_loop(x=length, radius_y=outer_radius, radius_z=outer_radius, segments=segments),
    )
    inner_front_ids = _append_loop(
        geometry,
        _ring_loop(x=length, radius_y=inner_radius, radius_z=inner_radius, segments=segments),
    )
    inner_back_ids = _append_loop(
        geometry,
        _ring_loop(x=0.0, radius_y=inner_radius, radius_z=inner_radius, segments=segments),
    )
    _connect_loops(geometry, outer_back_ids, outer_front_ids, reverse=False)
    _connect_loops(geometry, inner_front_ids, inner_back_ids, reverse=False)
    return geometry


def _spring_mesh(
    *,
    length: float,
    coil_radius: float,
    wire_radius: float,
    turns: float,
) -> MeshGeometry:
    helix_points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0), (0.012, 0.0, 0.0)]
    coil_start = 0.018
    coil_end = max(coil_start + 0.012, length - 0.018)
    samples = max(28, int(turns * 16))
    for index in range(samples + 1):
        t = index / samples
        angle = turns * 2.0 * math.pi * t
        x = coil_start + (coil_end - coil_start) * t
        helix_points.append(
            (
                x,
                coil_radius * math.cos(angle),
                coil_radius * math.sin(angle),
            )
        )
    helix_points.extend([(length - 0.012, 0.0, 0.0), (length, 0.0, 0.0)])
    return tube_from_spline_points(
        helix_points,
        radius=wire_radius,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )


def _add_arm_link(
    part,
    *,
    length: float,
    rail_offset: float,
    rail_width: float,
    rail_height: float,
    arm_material,
    spring_material,
    mesh_prefix: str,
) -> None:
    bar_start_x = 0.030
    bar_length = length - bar_start_x - 0.008
    bar_center_x = bar_start_x + bar_length * 0.5
    bridge_width = rail_offset * 2.0 + rail_width + 0.004

    part.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_material,
        name="proximal_pivot",
    )
    part.visual(
        Box((length - 0.018, 0.004, 0.004)),
        origin=Origin(xyz=(0.009 + (length - 0.018) * 0.5, 0.0, -0.001)),
        material=arm_material,
        name="center_tie",
    )
    part.visual(
        Box((0.018, bridge_width, rail_height + 0.002)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=arm_material,
        name="proximal_bridge",
    )
    part.visual(
        Box((bar_length, rail_width, rail_height)),
        origin=Origin(xyz=(bar_center_x, rail_offset, 0.0)),
        material=arm_material,
        name="left_rail",
    )
    part.visual(
        Box((bar_length, rail_width, rail_height)),
        origin=Origin(xyz=(bar_center_x, -rail_offset, 0.0)),
        material=arm_material,
        name="right_rail",
    )
    part.visual(
        Box((0.016, bridge_width - 0.010, 0.006)),
        origin=Origin(xyz=(length - 0.026, 0.0, -0.001)),
        material=arm_material,
        name="distal_saddle",
    )
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(
                xyz=(length, side_sign * rail_offset, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=arm_material,
            name=f"distal_lug_{side_index}",
        )

    spring_length = max(0.030, length - 0.060)
    spring = _spring_mesh(
        length=spring_length,
        coil_radius=0.004,
        wire_radius=0.0016,
        turns=max(4.0, length * 32.0),
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_spring", spring),
        origin=Origin(xyz=(0.030, 0.0, 0.015)),
        material=spring_material,
        name="counterbalance_spring",
    )
    for anchor_x, anchor_name in ((0.030, "spring_anchor_inner"), (length - 0.030, "spring_anchor_outer")):
        part.visual(
            Cylinder(radius=0.0026, length=0.020),
            origin=Origin(xyz=(anchor_x, 0.0, 0.009)),
            material=arm_material,
            name=anchor_name,
        )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_spring_arm_desk_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.14, 0.15, 0.16, 1.0))
    arm_steel = model.material("arm_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.73, 0.76, 0.79, 1.0))
    shade_enamel = model.material("shade_enamel", rgba=(0.33, 0.47, 0.39, 1.0))
    socket_black = model.material("socket_black", rgba=(0.09, 0.09, 0.10, 1.0))
    bulb_glass = model.material("bulb_glass", rgba=(0.97, 0.92, 0.76, 0.92))
    felt = model.material("felt", rgba=(0.18, 0.12, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh(
            "lamp_base_shell",
            ExtrudeGeometry(
                rounded_rect_profile(0.230, 0.170, 0.022, corner_segments=10),
                0.022,
                cap=True,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cast_iron,
        name="base_shell",
    )
    base.visual(
        Box((0.062, 0.084, 0.022)),
        origin=Origin(xyz=(-0.075, 0.0, 0.022)),
        material=cast_iron,
        name="pedestal_block",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(-0.075, 0.0, 0.043)),
        material=cast_iron,
        name="pedestal_post",
    )
    base.visual(
        Box((0.022, 0.014, 0.052)),
        origin=Origin(xyz=(-0.075, -0.025, 0.056)),
        material=cast_iron,
        name="clevis_left",
    )
    base.visual(
        Box((0.022, 0.014, 0.052)),
        origin=Origin(xyz=(-0.075, 0.025, 0.056)),
        material=cast_iron,
        name="clevis_right",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(-0.075, -0.014, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_steel,
        name="pivot_pin_left",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(-0.075, 0.014, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_steel,
        name="pivot_pin_right",
    )
    for foot_x, foot_y, foot_name in (
        (-0.080, -0.055, "rear_left_foot"),
        (-0.080, 0.055, "rear_right_foot"),
        (0.080, -0.055, "front_left_foot"),
        (0.080, 0.055, "front_right_foot"),
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(foot_x, foot_y, 0.0015)),
            material=felt,
            name=foot_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.230, 0.170, 0.086)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )

    link_1 = model.part("link_1")
    _add_arm_link(
        link_1,
        length=0.170,
        rail_offset=0.024,
        rail_width=0.010,
        rail_height=0.008,
        arm_material=arm_steel,
        spring_material=spring_steel,
        mesh_prefix="link_1",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((0.170, 0.070, 0.038)),
        mass=0.42,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    _add_arm_link(
        link_2,
        length=0.160,
        rail_offset=0.023,
        rail_width=0.010,
        rail_height=0.008,
        arm_material=arm_steel,
        spring_material=spring_steel,
        mesh_prefix="link_2",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((0.160, 0.068, 0.038)),
        mass=0.39,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    _add_arm_link(
        link_3,
        length=0.150,
        rail_offset=0.022,
        rail_width=0.009,
        rail_height=0.007,
        arm_material=arm_steel,
        spring_material=spring_steel,
        mesh_prefix="link_3",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((0.150, 0.064, 0.034)),
        mass=0.34,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    link_4 = model.part("link_4")
    _add_arm_link(
        link_4,
        length=0.120,
        rail_offset=0.020,
        rail_width=0.009,
        rail_height=0.007,
        arm_material=arm_steel,
        spring_material=spring_steel,
        mesh_prefix="link_4",
    )
    link_4.inertial = Inertial.from_geometry(
        Box((0.120, 0.060, 0.032)),
        mass=0.26,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_steel,
        name="pivot_barrel",
    )
    shade.visual(
        Box((0.032, 0.024, 0.022)),
        origin=Origin(xyz=(0.016, 0.0, -0.003)),
        material=arm_steel,
        name="yoke_block",
    )
    shade.visual(
        _save_mesh(
            "shade_shell",
            _shade_shell_mesh(
                length=0.125,
                outer_back=(0.028, 0.023),
                outer_front=(0.078, 0.064),
                inner_back=(0.020, 0.016),
                inner_front=(0.070, 0.056),
            ),
        ),
        origin=Origin(xyz=(0.028, 0.0, -0.012)),
        material=shade_enamel,
        name="shade_shell",
    )
    shade.visual(
        _save_mesh("shade_rear_collar", _tube_shell_mesh(length=0.028, outer_radius=0.018, inner_radius=0.0145)),
        origin=Origin(xyz=(0.028, 0.0, -0.012)),
        material=arm_steel,
        name="shade_rear_collar",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.036, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_steel,
        name="socket_mount",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.170, 0.160, 0.120)),
        mass=0.48,
        origin=Origin(xyz=(0.070, 0.0, -0.010)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_black,
        name="socket",
    )
    bulb.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_black,
        name="bulb_neck",
    )
    bulb.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=bulb_glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Sphere(radius=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(-0.075, 0.0, 0.070), rpy=(0.0, -0.95, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.80, upper=0.45),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(0.0, -0.30, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-1.00, upper=0.75),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, 0.42, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.75, upper=0.85),
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, 0.10, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "link_4_to_shade",
        ArticulationType.REVOLUTE,
        parent=link_4,
        child=shade,
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, 1.12, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.8, lower=-0.75, upper=1.10),
    )
    model.articulation(
        "shade_to_bulb",
        ArticulationType.FIXED,
        parent=shade,
        child=bulb,
        origin=Origin(xyz=(0.040, 0.0, -0.012)),
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
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    shade = object_model.get_part("shade")
    bulb = object_model.get_part("bulb")

    joint_names = (
        "base_to_link_1",
        "link_1_to_link_2",
        "link_2_to_link_3",
        "link_3_to_link_4",
        "link_4_to_shade",
    )
    joints = [object_model.get_articulation(name) for name in joint_names]

    ctx.check(
        "lamp assembly parts exist",
        all(part is not None for part in (base, link_1, link_2, link_3, link_4, shade, bulb)),
        details="Expected base, four arm links, shade, and bulb parts.",
    )
    ctx.check(
        "all arm pivots are revolute",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=str([(joint.name, joint.articulation_type) for joint in joints]),
    )
    ctx.expect_contact(
        link_1,
        base,
        elem_a="proximal_pivot",
        elem_b="pivot_pin_left",
        name="base fork pin contacts the first arm pivot barrel",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="z",
        min_gap=0.38,
        name="shade pivot sits well above the cast-iron base",
    )
    ctx.expect_within(
        bulb,
        shade,
        axes="yz",
        inner_elem="glass_envelope",
        outer_elem="shade_shell",
        margin=0.012,
        name="bulb remains centered within the shade profile",
    )
    ctx.expect_overlap(
        bulb,
        shade,
        axes="x",
        elem_a="glass_envelope",
        elem_b="shade_shell",
        min_overlap=0.035,
        name="bulb sits inside the shade depth",
    )
    ctx.expect_contact(
        bulb,
        shade,
        elem_a="socket",
        elem_b="socket_mount",
        name="bulb socket seats against the shade mount",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({joints[0]: 0.25}):
        raised_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "base joint raises the lamp arm",
        rest_shade_pos is not None
        and raised_shade_pos is not None
        and raised_shade_pos[2] > rest_shade_pos[2] + 0.03,
        details=f"rest={rest_shade_pos}, raised={raised_shade_pos}",
    )

    with ctx.pose({joints[1]: 0.25}):
        elbow_1_pos = ctx.part_world_position(shade)
    ctx.check(
        "second arm joint lifts the distal assembly",
        rest_shade_pos is not None and elbow_1_pos is not None and elbow_1_pos[2] > rest_shade_pos[2] + 0.018,
        details=f"rest={rest_shade_pos}, lifted={elbow_1_pos}",
    )

    with ctx.pose({joints[2]: 0.25}):
        elbow_2_pos = ctx.part_world_position(shade)
    ctx.check(
        "third arm joint lifts the distal assembly",
        rest_shade_pos is not None and elbow_2_pos is not None and elbow_2_pos[2] > rest_shade_pos[2] + 0.014,
        details=f"rest={rest_shade_pos}, lifted={elbow_2_pos}",
    )

    with ctx.pose({joints[3]: 0.30}):
        wrist_link_pos = ctx.part_world_position(shade)
    ctx.check(
        "terminal arm link pitches upward",
        rest_shade_pos is not None
        and wrist_link_pos is not None
        and wrist_link_pos[2] > rest_shade_pos[2] + 0.010,
        details=f"rest={rest_shade_pos}, lifted={wrist_link_pos}",
    )

    shade_joint = joints[4]
    shade_rest_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({shade_joint: 0.45}):
        shade_raised_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade tilt joint raises the lamp head",
        shade_rest_center is not None
        and shade_raised_center is not None
        and shade_raised_center[2] > shade_rest_center[2] + 0.018,
        details=f"rest={shade_rest_center}, raised={shade_raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
