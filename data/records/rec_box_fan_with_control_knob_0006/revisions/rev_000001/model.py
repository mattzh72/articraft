from __future__ import annotations

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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_rotor_blade_mesh():
    blade_profile = [
        (0.008, -0.006),
        (0.016, -0.010),
        (0.032, -0.011),
        (0.046, -0.006),
        (0.054, 0.002),
        (0.046, 0.010),
        (0.028, 0.012),
        (0.010, 0.006),
    ]
    blade = ExtrudeGeometry.centered(blade_profile, 0.004, cap=True, closed=True)
    blade.rotate_x((math.pi / 2.0) + 0.18)
    return _save_mesh(blade, "desk_box_fan_blade.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_box_fan", assets=ASSETS)

    white_plastic = model.material("white_plastic", rgba=(0.95, 0.95, 0.96, 1.0))
    warm_white = model.material("warm_white", rgba=(0.90, 0.91, 0.92, 1.0))
    light_grey = model.material("light_grey", rgba=(0.74, 0.76, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.34, 0.36, 0.39, 1.0))

    body = model.part("body")
    body.visual(Box((0.160, 0.078, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=white_plastic, name="top_shell")
    body.visual(Box((0.160, 0.078, 0.016)), origin=Origin(xyz=(0.0, 0.0, -0.080)), material=white_plastic, name="bottom_shell")
    body.visual(Box((0.016, 0.078, 0.144)), origin=Origin(xyz=(0.080, 0.0, 0.0)), material=white_plastic, name="right_shell")
    body.visual(Box((0.016, 0.078, 0.144)), origin=Origin(xyz=(-0.080, 0.0, 0.0)), material=white_plastic, name="left_shell")

    body.visual(Box((0.148, 0.004, 0.010)), origin=Origin(xyz=(0.0, 0.038, 0.069)), material=white_plastic, name="front_top_bar")
    body.visual(Box((0.148, 0.004, 0.010)), origin=Origin(xyz=(0.0, 0.038, -0.069)), material=white_plastic, name="front_bottom_bar")
    body.visual(Box((0.010, 0.004, 0.148)), origin=Origin(xyz=(0.069, 0.038, 0.0)), material=white_plastic, name="front_right_bar")
    body.visual(Box((0.010, 0.004, 0.148)), origin=Origin(xyz=(-0.069, 0.038, 0.0)), material=white_plastic, name="front_left_bar")
    for index, z_pos in enumerate((-0.044, -0.022, 0.0, 0.022, 0.044), start=1):
        body.visual(
            Box((0.132, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, 0.038, z_pos)),
            material=warm_white,
            name=f"front_grille_h_{index}",
        )
    for index, x_pos in enumerate((-0.044, -0.022, 0.0, 0.022, 0.044), start=1):
        body.visual(
            Box((0.006, 0.004, 0.132)),
            origin=Origin(xyz=(x_pos, 0.038, 0.0)),
            material=warm_white,
            name=f"front_grille_v_{index}",
        )
    body.visual(Box((0.022, 0.004, 0.022)), origin=Origin(xyz=(0.0, 0.038, 0.0)), material=light_grey, name="front_badge")

    body.visual(Box((0.148, 0.004, 0.010)), origin=Origin(xyz=(0.0, -0.038, 0.069)), material=white_plastic, name="rear_top_bar")
    body.visual(Box((0.148, 0.004, 0.010)), origin=Origin(xyz=(0.0, -0.038, -0.069)), material=white_plastic, name="rear_bottom_bar")
    body.visual(Box((0.010, 0.004, 0.148)), origin=Origin(xyz=(0.069, -0.038, 0.0)), material=white_plastic, name="rear_right_bar")
    body.visual(Box((0.010, 0.004, 0.148)), origin=Origin(xyz=(-0.069, -0.038, 0.0)), material=white_plastic, name="rear_left_bar")
    for index, z_pos in enumerate((-0.044, -0.022, 0.0, 0.022, 0.044), start=1):
        body.visual(
            Box((0.132, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, -0.038, z_pos)),
            material=warm_white,
            name=f"rear_grille_h_{index}",
        )
    for index, x_pos in enumerate((-0.044, -0.022, 0.0, 0.022, 0.044), start=1):
        body.visual(
            Box((0.006, 0.004, 0.132)),
            origin=Origin(xyz=(x_pos, -0.038, 0.0)),
            material=warm_white,
            name=f"rear_grille_v_{index}",
        )
    body.visual(Box((0.096, 0.036, 0.010)), origin=Origin(xyz=(0.0, -0.018, 0.0)), material=warm_white, name="rear_support_h")
    body.visual(Box((0.010, 0.036, 0.096)), origin=Origin(xyz=(0.0, -0.018, 0.0)), material=warm_white, name="rear_support_v")
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="bearing_boss",
    )

    body.visual(Box((0.050, 0.056, 0.024)), origin=Origin(xyz=(-0.042, 0.0, -0.100)), material=warm_white, name="left_foot")
    body.visual(Box((0.050, 0.056, 0.024)), origin=Origin(xyz=(0.042, 0.0, -0.100)), material=warm_white, name="right_foot")

    body.visual(Box((0.010, 0.050, 0.050)), origin=Origin(xyz=(0.093, 0.0, -0.004)), material=white_plastic, name="speed_panel")
    body.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.104, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="knob_mount",
    )
    body.visual(Box((0.002, 0.014, 0.014)), origin=Origin(xyz=(0.105, 0.0055, 0.0070)), material=charcoal, name="speed_tick_low")
    body.visual(Box((0.002, 0.014, 0.014)), origin=Origin(xyz=(0.105, 0.0000, 0.0090)), material=charcoal, name="speed_tick_mid")
    body.visual(Box((0.002, 0.014, 0.014)), origin=Origin(xyz=(0.105, -0.0055, 0.0070)), material=charcoal, name="speed_tick_high")
    body.inertial = Inertial.from_geometry(Box((0.176, 0.090, 0.224)), mass=1.2, origin=Origin())

    rotor = model.part("rotor")
    blade_mesh = _build_rotor_blade_mesh()
    rotor.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=light_grey,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="center_nut",
    )
    for index in range(5):
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(0.0, index * math.tau / 5.0, 0.0)),
            material=warm_white,
            name=f"blade_{index + 1}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.026),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_grey,
        name="knob_base",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="knob_cap",
    )
    knob.visual(
        Box((0.004, 0.008, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, 0.009)),
        material=warm_white,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.022),
        mass=0.03,
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.110, 0.0, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=2.0, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("speed_knob")
    rotor_joint = object_model.get_articulation("body_to_rotor")
    knob_joint = object_model.get_articulation("body_to_speed_knob")

    bearing_boss = body.get_visual("bearing_boss")
    bottom_shell = body.get_visual("bottom_shell")
    front_badge = body.get_visual("front_badge")
    left_foot = body.get_visual("left_foot")
    right_foot = body.get_visual("right_foot")
    knob_mount = body.get_visual("knob_mount")
    tick_low = body.get_visual("speed_tick_low")
    tick_mid = body.get_visual("speed_tick_mid")
    tick_high = body.get_visual("speed_tick_high")

    hub = rotor.get_visual("hub")
    hub_cap = rotor.get_visual("hub_cap")
    center_nut = rotor.get_visual("center_nut")
    blades = [rotor.get_visual(f"blade_{index}") for index in range(1, 6)]

    knob_base = knob.get_visual("knob_base")
    pointer = knob.get_visual("pointer")

    def _axis_overlap(interval_a: tuple[float, float], interval_b: tuple[float, float]) -> float:
        return max(0.0, min(interval_a[1], interval_b[1]) - max(interval_a[0], interval_b[0]))

    def _aabb_min_overlap(aabb_a, aabb_b, axes: tuple[int, ...]) -> float | None:
        if aabb_a is None or aabb_b is None:
            return None
        return min(
            _axis_overlap((aabb_a[0][axis], aabb_a[1][axis]), (aabb_b[0][axis], aabb_b[1][axis]))
            for axis in axes
        )

    def _foot_is_seated(foot_name: str) -> bool:
        foot_aabb = ctx.part_element_world_aabb(body, elem=foot_name)
        shell_aabb = ctx.part_element_world_aabb(body, elem=bottom_shell.name)
        if foot_aabb is None or shell_aabb is None:
            return False
        z_gap = shell_aabb[0][2] - foot_aabb[1][2]
        xy_overlap = _aabb_min_overlap(foot_aabb, shell_aabb, (0, 1))
        return abs(z_gap) <= 1e-6 and xy_overlap is not None and xy_overlap > 0.02

    def _pointer_target_check(label: str, expected_tick: str) -> None:
        pointer_aabb = ctx.part_element_world_aabb(knob, elem=pointer.name)
        tick_names = (tick_low.name, tick_mid.name, tick_high.name)
        overlaps: dict[str, float] = {}
        for tick_name in tick_names:
            tick_aabb = ctx.part_element_world_aabb(body, elem=tick_name)
            overlap = _aabb_min_overlap(pointer_aabb, tick_aabb, (1, 2))
            overlaps[tick_name] = -1.0 if overlap is None else overlap
        target_overlap = overlaps[expected_tick]
        other_overlaps = [overlaps[tick_name] for tick_name in tick_names if tick_name != expected_tick]
        ctx.check(
            f"knob_{label}_points_to_expected_tick",
            target_overlap >= 0.0075 and all(target_overlap >= other + 0.0015 for other in other_overlaps),
            details=f"target={target_overlap:.4f}, others={[round(value, 4) for value in other_overlaps]}",
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32, overlap_tol=0.0005, overlap_volume_tol=0.0)

    ctx.check(
        "left_foot_attached_to_bottom_shell",
        _foot_is_seated(left_foot.name),
        details="left foot should seat directly against the bottom shell",
    )
    ctx.check(
        "right_foot_attached_to_bottom_shell",
        _foot_is_seated(right_foot.name),
        details="right foot should seat directly against the bottom shell",
    )

    ctx.expect_origin_distance(rotor, body, axes="xz", max_dist=0.002, name="rotor_centered_in_housing")
    ctx.expect_within(rotor, body, axes="xz", name="rotor_within_housing_opening")
    ctx.expect_contact(body, rotor, elem_a=bearing_boss, elem_b=hub, name="rotor_hub_contacts_bearing")
    ctx.expect_gap(
        body,
        rotor,
        axis="y",
        min_gap=0.010,
        max_gap=0.018,
        positive_elem=front_badge,
        negative_elem=hub_cap,
        name="hub_cap_stays_behind_front_badge",
    )
    ctx.expect_gap(
        body,
        rotor,
        axis="y",
        min_gap=0.006,
        max_gap=0.012,
        positive_elem=front_badge,
        negative_elem=center_nut,
        name="center_nut_stays_behind_front_badge",
    )
    for blade in blades:
        ctx.expect_within(rotor, body, axes="xz", inner_elem=blade, name=f"{blade.name}_within_front_bezel")

    ctx.expect_contact(knob, body, elem_a=knob_base, elem_b=knob_mount, name="knob_base_contacts_mount")
    ctx.expect_gap(
        knob,
        body,
        axis="x",
        min_gap=0.012,
        max_gap=0.022,
        positive_elem=pointer,
        negative_elem=knob_mount,
        name="pointer_projects_proud_of_side_panel",
    )
    ctx.expect_origin_distance(knob, body, axes="yz", max_dist=0.020, name="knob_stays_on_right_face")

    with ctx.pose({rotor_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotor_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="rotor_quarter_turn_no_floating")
        ctx.expect_gap(
            body,
            rotor,
            axis="y",
            min_gap=0.010,
            max_gap=0.018,
            positive_elem=front_badge,
            negative_elem=hub_cap,
            name="rotor_quarter_turn_hub_cap_clearance",
        )
        for blade in blades:
            ctx.expect_within(rotor, body, axes="xz", inner_elem=blade, name=f"{blade.name}_within_front_bezel_quarter_turn")

    knob_limits = knob_joint.motion_limits
    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        with ctx.pose({knob_joint: knob_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="knob_low_no_overlap")
            ctx.fail_if_isolated_parts(name="knob_low_no_floating")
            ctx.expect_contact(knob, body, elem_a=knob_base, elem_b=knob_mount, name="knob_low_contact")
            ctx.expect_overlap(knob, body, axes="yz", min_overlap=0.006, elem_a=pointer, elem_b=tick_low, name="knob_low_tick_overlap")
            _pointer_target_check("low", tick_low.name)

    with ctx.pose({knob_joint: 0.0}):
        ctx.expect_contact(knob, body, elem_a=knob_base, elem_b=knob_mount, name="knob_mid_contact")
        ctx.expect_overlap(knob, body, axes="yz", min_overlap=0.006, elem_a=pointer, elem_b=tick_mid, name="knob_mid_tick_overlap")
        _pointer_target_check("mid", tick_mid.name)

    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        with ctx.pose({knob_joint: knob_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="knob_high_no_overlap")
            ctx.fail_if_isolated_parts(name="knob_high_no_floating")
            ctx.expect_contact(knob, body, elem_a=knob_base, elem_b=knob_mount, name="knob_high_contact")
            ctx.expect_overlap(knob, body, axes="yz", min_overlap=0.006, elem_a=pointer, elem_b=tick_high, name="knob_high_tick_overlap")
            _pointer_target_check("high", tick_high.name)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
