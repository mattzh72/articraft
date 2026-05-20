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
    section_loft,
)


CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_TOP_Z = 0.141
CANOPY_FRONT_TOP_Y = 0.190
CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.21
CHIMNEY_HEIGHT = 0.54
FILTER_OPENING_WIDTH = 0.57
FILTER_OPENING_DEPTH = 0.31


def _yz_panel_mesh(
    profile_yz: list[tuple[float, float]],
    *,
    x_min: float,
    x_max: float,
    name: str,
):
    return mesh_from_geometry(
        section_loft(
            [
                [(x_min, y, z) for y, z in profile_yz],
                [(x_max, y, z) for y, z in profile_yz],
            ]
        ),
        name,
    )


def _front_strip_point(
    x: float,
    y_local: float,
    z_local: float,
    *,
    strip_center: tuple[float, float, float],
    pitch: float,
) -> tuple[float, float, float]:
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    return (
        strip_center[0] + x,
        strip_center[1] + y_local * cos_pitch - z_local * sin_pitch,
        strip_center[2] + y_local * sin_pitch + z_local * cos_pitch,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_chimney_range_hood")

    steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_filter", rgba=(0.20, 0.21, 0.22, 1.0))

    hood_body = model.part("hood_body")

    canopy_side_profile = [
        (-0.250, 0.000),
        (0.250, 0.015),
        (CANOPY_FRONT_TOP_Y, CANOPY_TOP_Z),
        (-0.250, CANOPY_TOP_Z),
    ]
    left_side_shell = _yz_panel_mesh(
        canopy_side_profile,
        x_min=-(CANOPY_WIDTH * 0.5),
        x_max=-(CANOPY_WIDTH * 0.5) + 0.012,
        name="left_canopy_side_shell",
    )
    right_side_shell = _yz_panel_mesh(
        canopy_side_profile,
        x_min=(CANOPY_WIDTH * 0.5) - 0.012,
        x_max=(CANOPY_WIDTH * 0.5),
        name="right_canopy_side_shell",
    )
    hood_body.visual(left_side_shell, material=steel, name="left_side_shell")
    hood_body.visual(right_side_shell, material=steel, name="right_side_shell")
    hood_body.visual(
        Box((0.876, 0.440, 0.012)),
        origin=Origin(xyz=(0.000, -0.030, 0.135)),
        material=steel,
        name="top_shell",
    )
    hood_body.visual(
        Box((CANOPY_WIDTH, 0.012, CANOPY_TOP_Z)),
        origin=Origin(xyz=(0.000, -0.244, CANOPY_TOP_Z * 0.5)),
        material=steel,
        name="rear_shell",
    )

    front_strip_pitch = math.radians(25.0)
    front_strip_center = (0.000, 0.220, 0.078)

    def add_front_strip_box(
        *,
        name: str,
        x_center: float,
        z_center: float,
        width: float,
        height: float,
        thickness: float = 0.012,
        material=steel,
    ) -> None:
        hood_body.visual(
            Box((width, thickness, height)),
            origin=Origin(
                xyz=_front_strip_point(
                    x_center,
                    0.0,
                    z_center,
                    strip_center=front_strip_center,
                    pitch=front_strip_pitch,
                ),
                rpy=(front_strip_pitch, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )

    add_front_strip_box(
        name="front_left_panel",
        x_center=-0.160,
        z_center=0.000,
        width=0.570,
        height=0.140,
    )
    add_front_strip_box(
        name="front_right_panel",
        x_center=0.372,
        z_center=0.000,
        width=0.156,
        height=0.140,
    )
    add_front_strip_box(
        name="switch_cluster_top_bar",
        x_center=0.210,
        z_center=0.032,
        width=0.170,
        height=0.076,
    )
    add_front_strip_box(
        name="switch_cluster_bottom_bar",
        x_center=0.210,
        z_center=-0.052,
        width=0.170,
        height=0.036,
    )
    for name, x_center in (
        ("switch_cluster_left_bar", 0.129),
        ("switch_cluster_sep_left", 0.183),
        ("switch_cluster_sep_right", 0.237),
        ("switch_cluster_right_bar", 0.291),
    ):
        add_front_strip_box(
            name=name,
            x_center=x_center,
            z_center=-0.018,
            width=0.008,
            height=0.030,
            material=dark_trim,
        )

    hood_body.visual(
        Box((0.610, 0.140, 0.018)),
        origin=Origin(xyz=(0.000, -0.185, 0.009)),
        material=dark_trim,
        name="rear_intake_bridge",
    )
    hood_body.visual(
        Box((0.610, 0.050, 0.018)),
        origin=Origin(xyz=(0.000, 0.225, 0.009)),
        material=dark_trim,
        name="front_intake_bridge",
    )
    hood_body.visual(
        Box((0.133, 0.360, 0.018)),
        origin=Origin(xyz=(-0.3715, 0.040, 0.009)),
        material=dark_trim,
        name="left_intake_bridge",
    )
    hood_body.visual(
        Box((0.133, 0.360, 0.018)),
        origin=Origin(xyz=(0.3715, 0.040, 0.009)),
        material=dark_trim,
        name="right_intake_bridge",
    )
    hood_body.visual(
        Box((0.610, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, -0.125, 0.009)),
        material=matte_black,
        name="intake_rear_rim",
    )
    hood_body.visual(
        Box((0.610, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, 0.205, 0.009)),
        material=matte_black,
        name="intake_front_rim",
    )
    hood_body.visual(
        Box((0.020, FILTER_OPENING_DEPTH, 0.018)),
        origin=Origin(xyz=(-0.295, 0.040, 0.009)),
        material=matte_black,
        name="intake_left_rim",
    )
    hood_body.visual(
        Box((0.020, FILTER_OPENING_DEPTH, 0.018)),
        origin=Origin(xyz=(0.295, 0.040, 0.009)),
        material=matte_black,
        name="intake_right_rim",
    )

    hood_body.visual(
        Box((CHIMNEY_WIDTH, 0.012, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.000, -0.244, CANOPY_TOP_Z + CHIMNEY_HEIGHT * 0.5)),
        material=steel,
        name="chimney_back_shell",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, 0.012, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.000, -0.046, CANOPY_TOP_Z + CHIMNEY_HEIGHT * 0.5)),
        material=steel,
        name="chimney_front_shell",
    )
    hood_body.visual(
        Box((0.012, CHIMNEY_DEPTH - 0.012, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CHIMNEY_WIDTH * 0.5) + 0.006,
                -0.145,
                CANOPY_TOP_Z + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=steel,
        name="chimney_left_shell",
    )
    hood_body.visual(
        Box((0.012, CHIMNEY_DEPTH - 0.012, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                (CHIMNEY_WIDTH * 0.5) - 0.006,
                -0.145,
                CANOPY_TOP_Z + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=steel,
        name="chimney_right_shell",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH - 0.024, CHIMNEY_DEPTH - 0.024, 0.012)),
        origin=Origin(xyz=(0.000, -0.145, CANOPY_TOP_Z + CHIMNEY_HEIGHT)),
        material=steel,
        name="chimney_cap",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_TOP_Z + CHIMNEY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.000, 0.000, (CANOPY_TOP_Z + CHIMNEY_HEIGHT) * 0.5)),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((0.560, 0.305, 0.012)),
        origin=Origin(xyz=(0.000, 0.1525, -0.006)),
        material=steel,
        name="door_panel",
    )
    filter_door.visual(
        Box((0.500, 0.245, 0.004)),
        origin=Origin(xyz=(0.000, 0.1525, -0.010)),
        material=charcoal,
        name="filter_media",
    )
    filter_door.visual(
        Box((0.150, 0.016, 0.010)),
        origin=Origin(xyz=(0.000, 0.292, -0.010)),
        material=dark_trim,
        name="door_pull",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.560, 0.305, 0.014)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.1525, -0.007)),
    )
    model.articulation(
        "hood_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=hood_body,
        child=filter_door,
        origin=Origin(xyz=(0.000, -0.115, 0.000)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    switch_centers = (
        ("switch_left", 0.156),
        ("switch_center", 0.210),
        ("switch_right", 0.264),
    )
    for part_name, switch_x in switch_centers:
        switch_part = model.part(part_name)
        switch_part.visual(
            Cylinder(radius=0.003, length=0.046),
            origin=Origin(xyz=(0.000, -0.001, 0.000), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=matte_black,
            name="pivot_barrel",
        )
        switch_part.visual(
            Box((0.042, 0.008, 0.022)),
            origin=Origin(xyz=(0.000, 0.006, 0.000)),
            material=matte_black,
            name="rocker_face",
        )
        switch_part.inertial = Inertial.from_geometry(
            Box((0.042, 0.010, 0.024)),
            mass=0.03,
            origin=Origin(xyz=(0.000, 0.005, 0.000)),
        )
        model.articulation(
            f"hood_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=hood_body,
            child=switch_part,
            origin=Origin(
                xyz=_front_strip_point(
                    switch_x,
                    0.0,
                    -0.020,
                    strip_center=front_strip_center,
                    pitch=front_strip_pitch,
                ),
                rpy=(front_strip_pitch, 0.0, 0.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=3.0,
                lower=-0.18,
                upper=0.18,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood_body = object_model.get_part("hood_body")
    filter_door = object_model.get_part("filter_door")
    filter_door_joint = object_model.get_articulation("hood_to_filter_door")
    switch_parts = [
        object_model.get_part("switch_left"),
        object_model.get_part("switch_center"),
        object_model.get_part("switch_right"),
    ]
    switch_joints = [
        object_model.get_articulation("hood_to_switch_left"),
        object_model.get_articulation("hood_to_switch_center"),
        object_model.get_articulation("hood_to_switch_right"),
    ]

    ctx.check(
        "filter door hinge opens downward from the intake rear edge",
        filter_door_joint.axis[0] < -0.99
        and abs(filter_door_joint.axis[1]) < 1e-9
        and abs(filter_door_joint.axis[2]) < 1e-9
        and filter_door_joint.motion_limits is not None
        and filter_door_joint.motion_limits.lower == 0.0
        and filter_door_joint.motion_limits.upper is not None
        and filter_door_joint.motion_limits.upper > 1.0,
        details=f"axis={filter_door_joint.axis}, limits={filter_door_joint.motion_limits}",
    )

    ctx.check(
        "rocker switches use short transverse pivots on the front strip",
        all(
            joint.motion_limits is not None
            and abs(joint.axis[0]) > 0.99
            and abs(joint.axis[1]) < 1e-9
            and abs(joint.axis[2]) < 1e-9
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
            and joint.motion_limits.upper <= 0.25
            for joint in switch_joints
        ),
        details=", ".join(f"{joint.name}: axis={joint.axis}, limits={joint.motion_limits}" for joint in switch_joints),
    )

    ctx.expect_contact(
        hood_body,
        filter_door,
        elem_a="intake_rear_rim",
        elem_b="door_panel",
        contact_tol=0.001,
        name="filter door seats against the rear intake rim when closed",
    )

    switch_positions = [ctx.part_world_position(part) for part in switch_parts]
    ctx.check(
        "switch cluster sits on the right side of the front control strip",
        all(position is not None for position in switch_positions)
        and switch_positions[0][0] < switch_positions[1][0] < switch_positions[2][0]
        and switch_positions[0][0] > 0.12
        and all(position[1] > 0.19 for position in switch_positions)
        and all(0.03 < position[2] < 0.09 for position in switch_positions),
        details=f"positions={switch_positions}",
    )

    closed_pull_aabb = ctx.part_element_world_aabb(filter_door, elem="door_pull")
    with ctx.pose({filter_door_joint: math.radians(72.0)}):
        open_pull_aabb = ctx.part_element_world_aabb(filter_door, elem="door_pull")
        ctx.expect_gap(
            hood_body,
            filter_door,
            axis="z",
            min_gap=0.08,
            max_gap=None,
            negative_elem="door_pull",
            name="opened filter door drops below the hood body for access",
        )
    closed_pull_center = _aabb_center(closed_pull_aabb)
    open_pull_center = _aabb_center(open_pull_aabb)
    ctx.check(
        "filter door swings forward and downward",
        closed_pull_center is not None
        and open_pull_center is not None
        and open_pull_center[1] < closed_pull_center[1] - 0.15
        and open_pull_center[2] < closed_pull_center[2] - 0.14,
        details=f"closed_pull={closed_pull_center}, open_pull={open_pull_center}",
    )

    left_switch = switch_parts[0]
    left_joint = switch_joints[0]
    with ctx.pose({left_joint: -0.18}):
        left_switch_back = ctx.part_element_world_aabb(left_switch, elem="rocker_face")
    with ctx.pose({left_joint: 0.18}):
        left_switch_forward = ctx.part_element_world_aabb(left_switch, elem="rocker_face")
    left_switch_back_center = _aabb_center(left_switch_back)
    left_switch_forward_center = _aabb_center(left_switch_forward)
    ctx.check(
        "rocker switch visibly tips through its travel",
        left_switch_back_center is not None
        and left_switch_forward_center is not None
        and abs(left_switch_forward_center[1] - left_switch_back_center[1]) > 0.0005
        and abs(left_switch_forward_center[2] - left_switch_back_center[2]) > 0.001,
        details=f"back={left_switch_back_center}, forward={left_switch_forward_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
