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
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_bar_x(
    *,
    length: float,
    width: float,
    height: float,
    corner_radius: float,
    mesh_name: str,
):
    profile = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    geom = ExtrudeGeometry.centered(profile, length, cap=True, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _rounded_tube_x(
    *,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    outer_radius: float,
    inner_radius: float,
    mesh_name: str,
):
    outer = rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=8)
    inner = rounded_rect_profile(
        outer_width - 2.0 * wall,
        outer_height - 2.0 * wall,
        inner_radius,
        corner_segments=8,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        length,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _build_shade_shell_mesh(
    *,
    start_x: float,
    length: float,
    outer_back_radius: float,
    outer_front_radius: float,
    inner_back_radius: float,
    inner_front_radius: float,
    center_z: float,
    segments: int = 36,
) -> MeshGeometry:
    geom = MeshGeometry()

    def ring(x_pos: float, radius: float) -> list[int]:
        ids: list[int] = []
        for index in range(segments):
            theta = (2.0 * math.pi * index) / segments
            y = radius * math.cos(theta)
            z = center_z + radius * math.sin(theta)
            ids.append(geom.add_vertex(x_pos, y, z))
        return ids

    outer_back = ring(start_x, outer_back_radius)
    outer_front = ring(start_x + length, outer_front_radius)
    inner_back = ring(start_x, inner_back_radius)
    inner_front = ring(start_x + length, inner_front_radius)

    for index in range(segments):
        nxt = (index + 1) % segments

        geom.add_face(outer_back[index], outer_back[nxt], outer_front[nxt])
        geom.add_face(outer_back[index], outer_front[nxt], outer_front[index])

        geom.add_face(inner_back[index], inner_front[nxt], inner_back[nxt])
        geom.add_face(inner_back[index], inner_front[index], inner_front[nxt])

        geom.add_face(outer_back[index], inner_back[nxt], outer_back[nxt])
        geom.add_face(outer_back[index], inner_back[index], inner_back[nxt])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_task_lamp")

    plate_paint = model.material("plate_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    shade_paint = model.material("shade_paint", rgba=(0.13, 0.14, 0.15, 1.0))
    reflector = model.material("reflector", rgba=(0.86, 0.84, 0.78, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.008, 0.060, 0.120)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=plate_paint,
        name="plate",
    )
    wall_plate.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plate_paint,
        name="pivot_boss",
    )
    wall_plate.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.006, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="upper_fastener",
    )
    wall_plate.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.006, 0.0, -0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="lower_fastener",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.028, 0.060, 0.120)),
        mass=1.2,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    outer_arm = model.part("outer_arm")
    outer_sleeve_mesh = _rounded_tube_x(
        length=0.240,
        outer_width=0.032,
        outer_height=0.024,
        wall=0.004,
        outer_radius=0.005,
        inner_radius=0.003,
        mesh_name="outer_arm_sleeve",
    )
    outer_arm.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="swing_pivot_pad",
    )
    outer_arm.visual(
        Box((0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=arm_finish,
        name="arm_bridge",
    )
    outer_arm.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(0.168, 0.0, 0.0)),
        material=arm_finish,
        name="outer_sleeve",
    )
    outer_arm.inertial = Inertial.from_geometry(
        Box((0.288, 0.032, 0.030)),
        mass=1.1,
        origin=Origin(xyz=(0.144, 0.0, 0.0)),
    )

    inner_arm = model.part("inner_arm")
    inner_slider_mesh = _rounded_bar_x(
        length=0.240,
        width=0.020,
        height=0.012,
        corner_radius=0.003,
        mesh_name="inner_arm_slider",
    )
    inner_arm.visual(
        inner_slider_mesh,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=arm_finish,
        name="inner_slider",
    )
    inner_arm.visual(
        Box((0.012, 0.020, 0.006)),
        origin=Origin(xyz=(0.246, 0.0, -0.008)),
        material=arm_finish,
        name="tip_block",
    )
    inner_arm.visual(
        Box((0.020, 0.006, 0.022)),
        origin=Origin(xyz=(0.258, 0.012, 0.0)),
        material=arm_finish,
        name="left_tilt_ear",
    )
    inner_arm.visual(
        Box((0.020, 0.006, 0.022)),
        origin=Origin(xyz=(0.258, -0.012, 0.0)),
        material=arm_finish,
        name="right_tilt_ear",
    )
    inner_arm.inertial = Inertial.from_geometry(
        Box((0.268, 0.032, 0.022)),
        mass=0.8,
        origin=Origin(xyz=(0.134, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade_shell_geom = _build_shade_shell_mesh(
        start_x=0.014,
        length=0.090,
        outer_back_radius=0.032,
        outer_front_radius=0.038,
        inner_back_radius=0.010,
        inner_front_radius=0.034,
        center_z=-0.018,
        segments=40,
    )
    shade_shell_mesh = mesh_from_geometry(shade_shell_geom, "task_lamp_shade_shell")
    shade.visual(
        shade_shell_mesh,
        material=shade_paint,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.006, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="pivot_barrel",
    )
    shade.visual(
        Box((0.030, 0.008, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, -0.010)),
        material=arm_finish,
        name="pivot_bracket",
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.036, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector,
        name="socket_collar",
    )
    shade.visual(
        Box((0.018, 0.016, 0.018)),
        origin=Origin(xyz=(0.024, 0.0, -0.018)),
        material=reflector,
        name="socket_mount",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.108, 0.076, 0.070)),
        mass=0.55,
        origin=Origin(xyz=(0.054, 0.0, -0.010)),
    )

    wall_to_outer = model.articulation(
        "wall_to_outer_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=outer_arm,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-math.radians(95.0),
            upper=math.radians(95.0),
        ),
    )

    outer_to_inner = model.articulation(
        "outer_to_inner_arm",
        ArticulationType.PRISMATIC,
        parent=outer_arm,
        child=inner_arm,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.18,
            lower=0.0,
            upper=0.145,
        ),
    )

    inner_to_shade = model.articulation(
        "inner_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=shade,
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-math.radians(70.0),
            upper=math.radians(45.0),
        ),
    )

    model.meta["primary_articulations"] = (
        wall_to_outer.name,
        outer_to_inner.name,
        inner_to_shade.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    outer_arm = object_model.get_part("outer_arm")
    inner_arm = object_model.get_part("inner_arm")
    shade = object_model.get_part("shade")
    wall_to_outer = object_model.get_articulation("wall_to_outer_arm")
    outer_to_inner = object_model.get_articulation("outer_to_inner_arm")
    inner_to_shade = object_model.get_articulation("inner_arm_to_shade")

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

    ctx.expect_contact(
        outer_arm,
        wall_plate,
        elem_a="swing_pivot_pad",
        elem_b="pivot_boss",
        name="outer arm seats against wall boss",
    )
    ctx.expect_within(
        inner_arm,
        outer_arm,
        axes="yz",
        inner_elem="inner_slider",
        outer_elem="outer_sleeve",
        margin=0.0005,
        name="inner slider stays centered inside outer sleeve at rest",
    )
    ctx.expect_overlap(
        inner_arm,
        outer_arm,
        axes="x",
        elem_a="inner_slider",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="inner slider remains deeply inserted at rest",
    )
    ctx.expect_contact(
        shade,
        inner_arm,
        elem_a="pivot_bracket",
        elem_b="tip_block",
        name="shade bracket seats on arm tip block",
    )

    rest_inner_pos = ctx.part_world_position(inner_arm)
    with ctx.pose({wall_to_outer: math.radians(65.0)}):
        swung_inner_pos = ctx.part_world_position(inner_arm)
        ctx.check(
            "wall joint swings arm laterally",
            rest_inner_pos is not None
            and swung_inner_pos is not None
            and swung_inner_pos[1] > rest_inner_pos[1] + 0.03,
            details=f"rest={rest_inner_pos}, swung={swung_inner_pos}",
        )

    rest_slider_pos = ctx.part_world_position(inner_arm)
    with ctx.pose({outer_to_inner: 0.145}):
        extended_slider_pos = ctx.part_world_position(inner_arm)
        ctx.expect_within(
            inner_arm,
            outer_arm,
            axes="yz",
            inner_elem="inner_slider",
            outer_elem="outer_sleeve",
            margin=0.0005,
            name="inner slider stays centered inside outer sleeve when extended",
        )
        ctx.expect_overlap(
            inner_arm,
            outer_arm,
            axes="x",
            elem_a="inner_slider",
            elem_b="outer_sleeve",
            min_overlap=0.09,
            name="inner slider retains insertion at full extension",
        )
        ctx.check(
            "prismatic arm extends outward",
            rest_slider_pos is not None
            and extended_slider_pos is not None
            and extended_slider_pos[0] > rest_slider_pos[0] + 0.10,
            details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
        )

    rest_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({inner_to_shade: math.radians(35.0)}):
        tilted_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        ctx.check(
            "shade tilts upward on horizontal hinge",
            rest_shade_aabb is not None
            and tilted_shade_aabb is not None
            and tilted_shade_aabb[1][2] > rest_shade_aabb[1][2] + 0.02,
            details=f"rest={rest_shade_aabb}, tilted={tilted_shade_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
