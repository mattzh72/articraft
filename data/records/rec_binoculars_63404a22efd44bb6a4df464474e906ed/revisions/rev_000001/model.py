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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _tube_shell_mesh(
    name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
        ),
        name,
    )


def _xz_rounded_section(
    *,
    center_x: float,
    y: float,
    center_z: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, y, center_z + pz)
        for px, pz in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _binocular_body_shell(name: str, side_sign: float):
    sections = [
        _xz_rounded_section(
            center_x=side_sign * 0.088,
            y=-0.058,
            center_z=-0.016,
            width=0.078,
            height=0.066,
            radius=0.014,
        ),
        _xz_rounded_section(
            center_x=side_sign * 0.072,
            y=-0.002,
            center_z=-0.004,
            width=0.092,
            height=0.078,
            radius=0.016,
        ),
        _xz_rounded_section(
            center_x=side_sign * 0.061,
            y=0.056,
            center_z=0.022,
            width=0.068,
            height=0.070,
            radius=0.014,
        ),
        _xz_rounded_section(
            center_x=side_sign * 0.044,
            y=0.108,
            center_z=0.040,
            width=0.046,
            height=0.050,
            radius=0.012,
        ),
    ]
    return mesh_from_geometry(section_loft(sections), name)


def _add_binocular_half(
    model: ArticulatedObject,
    *,
    name: str,
    side_sign: float,
    body_material,
    armor_material,
    glass_material,
    trim_material,
    objective_shell_mesh,
) -> None:
    body = model.part(name)
    body.visual(
        Box((0.020, 0.052, 0.058)),
        origin=Origin(xyz=(side_sign * 0.022, 0.000, 0.002)),
        material=trim_material,
        name="hinge_block",
    )
    body.visual(
        _binocular_body_shell(f"{name}_shell", side_sign),
        material=body_material,
        name="body_shell",
    )
    body.visual(
        Box((0.082, 0.118, 0.010)),
        origin=Origin(xyz=(side_sign * 0.071, 0.006, -0.040)),
        material=armor_material,
        name="underside_armor",
    )
    body.visual(
        objective_shell_mesh,
        origin=Origin(
            xyz=(side_sign * 0.103, -0.104, -0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_material,
        name="objective_shell",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.0032),
        origin=Origin(
            xyz=(side_sign * 0.103, -0.186, -0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_material,
        name="objective_glass",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(
            xyz=(side_sign * 0.032, 0.115, 0.041),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_material,
        name="eyepiece_shoulder",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.003),
        origin=Origin(
            xyz=(side_sign * 0.032, 0.122, 0.041),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_material,
        name="eyepiece_glass",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.160, 0.300, 0.110)),
        mass=1.18,
        origin=Origin(xyz=(side_sign * 0.070, -0.018, 0.000)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astronomy_porro_binocular")

    body_metal = model.material("body_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    armor_black = model.material("armor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.25, 0.26, 0.28, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.28, 0.42, 0.52, 0.55))
    ring_silver = model.material("ring_silver", rgba=(0.56, 0.58, 0.61, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    objective_shell_mesh = _tube_shell_mesh(
        "objective_barrel_shell",
        outer_profile=[
            (0.0415, -0.094),
            (0.0415, 0.068),
            (0.0430, 0.088),
            (0.0435, 0.094),
        ],
        inner_profile=[
            (0.0360, -0.090),
            (0.0360, 0.080),
            (0.0370, 0.091),
        ],
    )
    eyecup_shell_mesh = _tube_shell_mesh(
        "eyecup_shell",
        outer_profile=[
            (0.0210, -0.052),
            (0.0215, -0.018),
            (0.0245, -0.004),
            (0.0245, 0.000),
        ],
        inner_profile=[
            (0.0172, -0.048),
            (0.0176, -0.010),
            (0.0178, 0.000),
        ],
    )

    center_bridge = model.part("center_bridge")
    center_bridge.visual(
        Box((0.024, 0.050, 0.060)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=trim_graphite,
        name="hinge_tower",
    )
    center_bridge.visual(
        Cylinder(radius=0.010, length=0.066),
        origin=Origin(xyz=(0.000, 0.000, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ring_silver,
        name="hinge_axle",
    )
    center_bridge.visual(
        Box((0.028, 0.086, 0.040)),
        origin=Origin(xyz=(0.000, 0.052, 0.020)),
        material=trim_graphite,
        name="bridge_neck",
    )
    center_bridge.visual(
        Box((0.024, 0.022, 0.020)),
        origin=Origin(xyz=(0.000, 0.094, 0.026)),
        material=trim_graphite,
        name="focus_pedestal",
    )
    center_bridge.visual(
        Box((0.024, 0.014, 0.036)),
        origin=Origin(xyz=(0.000, 0.076, 0.054)),
        material=ring_silver,
        name="focus_rear_support",
    )
    center_bridge.visual(
        Box((0.004, 0.028, 0.036)),
        origin=Origin(xyz=(-0.014, 0.097, 0.054)),
        material=ring_silver,
        name="focus_left_fork",
    )
    center_bridge.visual(
        Box((0.004, 0.028, 0.036)),
        origin=Origin(xyz=(0.014, 0.097, 0.054)),
        material=ring_silver,
        name="focus_right_fork",
    )
    center_bridge.inertial = Inertial.from_geometry(
        Box((0.060, 0.150, 0.080)),
        mass=0.65,
        origin=Origin(xyz=(0.000, 0.055, 0.020)),
    )

    _add_binocular_half(
        model,
        name="left_body",
        side_sign=-1.0,
        body_material=body_metal,
        armor_material=armor_black,
        glass_material=glass_blue,
        trim_material=trim_graphite,
        objective_shell_mesh=objective_shell_mesh,
    )
    _add_binocular_half(
        model,
        name="right_body",
        side_sign=1.0,
        body_material=body_metal,
        armor_material=armor_black,
        glass_material=glass_blue,
        trim_material=trim_graphite,
        objective_shell_mesh=objective_shell_mesh,
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="focus_tire",
    )
    focus_wheel.visual(
        Cylinder(radius=0.021, length=0.005),
        origin=Origin(xyz=(-0.0095, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ring_silver,
        name="focus_left_flange",
    )
    focus_wheel.visual(
        Cylinder(radius=0.021, length=0.005),
        origin=Origin(xyz=(0.0095, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ring_silver,
        name="focus_right_flange",
    )
    focus_wheel.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.019)),
        material=ring_silver,
        name="focus_pointer",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.024),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    for side_name in ("left", "right"):
        eyecup = model.part(f"{side_name}_eyecup")
        eyecup.visual(
            eyecup_shell_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name="eyecup_shell",
        )
        eyecup.visual(
            Box((0.006, 0.014, 0.010)),
            origin=Origin(xyz=(0.021, 0.032, 0.000)),
            material=ring_silver,
            name="twist_rib",
        )
        eyecup.inertial = Inertial.from_geometry(
            Box((0.050, 0.056, 0.050)),
            mass=0.055,
            origin=Origin(xyz=(0.000, 0.026, 0.000)),
        )

    hinge_limits = MotionLimits(
        effort=4.0,
        velocity=1.0,
        lower=-0.17,
        upper=0.17,
    )
    model.articulation(
        "bridge_to_left_body",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child="left_body",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "bridge_to_right_body",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child="right_body",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-0.17,
            upper=0.17,
        ),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=focus_wheel,
        origin=Origin(xyz=(0.000, 0.102, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "left_body_to_left_eyecup",
        ArticulationType.REVOLUTE,
        parent="left_body",
        child="left_eyecup",
        origin=Origin(xyz=(-0.032, 0.122, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "right_body_to_right_eyecup",
        ArticulationType.REVOLUTE,
        parent="right_body",
        child="right_eyecup",
        origin=Origin(xyz=(0.032, 0.122, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_bridge = object_model.get_part("center_bridge")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    left_eyecup = object_model.get_part("left_eyecup")
    right_eyecup = object_model.get_part("right_eyecup")

    left_hinge = object_model.get_articulation("bridge_to_left_body")
    right_hinge = object_model.get_articulation("bridge_to_right_body")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    left_eyecup_joint = object_model.get_articulation("left_body_to_left_eyecup")
    right_eyecup_joint = object_model.get_articulation("right_body_to_right_eyecup")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "center_bridge",
        "left_body",
        "right_body",
        "focus_wheel",
        "left_eyecup",
        "right_eyecup",
    ):
        ctx.check(
            f"{part_name}_present",
            object_model.get_part(part_name) is not None,
            f"Expected part {part_name} to exist.",
        )

    ctx.check(
        "central_hinge_axes",
        left_hinge.axis == (0.0, 1.0, 0.0) and right_hinge.axis == (0.0, 1.0, 0.0),
        f"Expected both body hinges on world y, got {left_hinge.axis} and {right_hinge.axis}.",
    )
    ctx.check(
        "focus_wheel_axis",
        focus_joint.axis == (1.0, 0.0, 0.0),
        f"Expected focus wheel on world x, got {focus_joint.axis}.",
    )
    ctx.check(
        "eyecup_axes",
        left_eyecup_joint.axis == (0.0, 1.0, 0.0)
        and right_eyecup_joint.axis == (0.0, 1.0, 0.0),
        f"Expected eyecups to twist on world y, got {left_eyecup_joint.axis} and {right_eyecup_joint.axis}.",
    )

    ctx.expect_contact(left_body, center_bridge, name="left_body_mounted_to_bridge")
    ctx.expect_contact(right_body, center_bridge, name="right_body_mounted_to_bridge")
    ctx.expect_contact(focus_wheel, center_bridge, name="focus_wheel_seated_in_bridge")
    ctx.expect_contact(left_eyecup, left_body, name="left_eyecup_mounted")
    ctx.expect_contact(right_eyecup, right_body, name="right_eyecup_mounted")
    ctx.expect_gap(
        right_body,
        left_body,
        axis="x",
        positive_elem="objective_shell",
        negative_elem="objective_shell",
        min_gap=0.110,
        name="wide_objective_spacing",
    )

    rest_left_objective = ctx.part_element_world_aabb(left_body, elem="objective_shell")
    rest_focus_pointer = ctx.part_element_world_aabb(focus_wheel, elem="focus_pointer")
    rest_left_rib = ctx.part_element_world_aabb(left_eyecup, elem="twist_rib")

    with ctx.pose({left_hinge: 0.10, right_hinge: -0.10}):
        posed_left_objective = ctx.part_element_world_aabb(left_body, elem="objective_shell")
        ctx.expect_contact(left_body, center_bridge, name="left_body_contact_in_hinge_pose")
        ctx.expect_contact(right_body, center_bridge, name="right_body_contact_in_hinge_pose")
        ctx.expect_gap(
            right_body,
            left_body,
            axis="x",
            positive_elem="objective_shell",
            negative_elem="objective_shell",
            min_gap=0.095,
            name="objective_spacing_retained_in_hinge_pose",
        )
        hinge_motion_ok = (
            rest_left_objective is not None
            and posed_left_objective is not None
            and abs(posed_left_objective[0][2] - rest_left_objective[0][2]) > 0.004
        )
        ctx.check(
            "central_hinge_moves_objective_barrel",
            hinge_motion_ok,
            "Expected the articulated objective barrel AABB to shift in z when the hinge is posed.",
        )

    with ctx.pose({focus_joint: 1.0}):
        posed_focus_pointer = ctx.part_element_world_aabb(focus_wheel, elem="focus_pointer")
        focus_motion_ok = (
            rest_focus_pointer is not None
            and posed_focus_pointer is not None
            and abs(posed_focus_pointer[1][1] - rest_focus_pointer[1][1]) > 0.006
        )
        ctx.expect_contact(focus_wheel, center_bridge, name="focus_wheel_contact_when_rotated")
        ctx.check(
            "focus_wheel_rotates",
            focus_motion_ok,
            "Expected the focus pointer to move when the wheel articulation is posed.",
        )

    with ctx.pose({left_eyecup_joint: 0.9}):
        posed_left_rib = ctx.part_element_world_aabb(left_eyecup, elem="twist_rib")
        eyecup_motion_ok = (
            rest_left_rib is not None
            and posed_left_rib is not None
            and abs(posed_left_rib[1][2] - rest_left_rib[1][2]) > 0.006
        )
        ctx.expect_contact(left_eyecup, left_body, name="left_eyecup_contact_when_twisted")
        ctx.check(
            "left_eyecup_twist_motion",
            eyecup_motion_ok,
            "Expected the eyecup rib to move when the twist-lock eyecup is posed.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
