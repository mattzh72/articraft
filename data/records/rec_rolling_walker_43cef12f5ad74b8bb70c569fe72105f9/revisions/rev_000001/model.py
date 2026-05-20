from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merged(*geometries):
    base = geometries[0].copy()
    for geometry in geometries[1:]:
        base.merge(geometry)
    return base


def _tube_segment(points, *, radius: float, radial_segments: int = 18):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=8,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _shift_points(points, *, dz: float = 0.0):
    return [(x, y, z + dz) for x, y, z in points]


def _build_frame_half_geometry(side_sign: float, *, z_offset: float):
    tube_r = 0.014
    outer_x = side_sign * 0.295
    rear_x = side_sign * 0.287

    outer_loop = wire_from_points(
        _shift_points(
            [
            (outer_x, 0.215, 0.180),
            (outer_x, 0.215, 0.870),
            (outer_x, 0.120, 0.930),
            (side_sign * 0.292, -0.135, 0.930),
            (rear_x, -0.220, 0.885),
            (rear_x, -0.235, 0.060),
            ],
            dz=z_offset,
        ),
        radius=tube_r,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.050,
        corner_segments=8,
    )

    front_upper = _tube_segment(
        _shift_points([(side_sign * 0.028, 0.105, 0.800), (side_sign * 0.286, 0.185, 0.882)], dz=z_offset),
        radius=tube_r,
    )
    rear_upper = _tube_segment(
        _shift_points([(side_sign * 0.028, -0.105, 0.780), (side_sign * 0.283, -0.182, 0.875)], dz=z_offset),
        radius=tube_r,
    )
    front_lower = _tube_segment(
        _shift_points([(side_sign * 0.028, 0.018, 0.490), (side_sign * 0.286, 0.198, 0.440)], dz=z_offset),
        radius=tube_r,
    )
    rear_lower = _tube_segment(
        _shift_points([(side_sign * 0.028, -0.040, 0.470), (side_sign * 0.281, -0.212, 0.420)], dz=z_offset),
        radius=tube_r,
    )

    center_mount = BoxGeometry((0.028, 0.160, 0.074)).translate(side_sign * 0.030, 0.000, z_offset + 0.002)
    center_gusset = BoxGeometry((0.046, 0.082, 0.110)).translate(side_sign * 0.044, -0.010, z_offset - 0.205)
    front_socket = CylinderGeometry(radius=0.017, height=0.016, radial_segments=24).translate(outer_x, 0.215, z_offset - 0.732)
    rear_socket = CylinderGeometry(radius=0.017, height=0.010, radial_segments=24).translate(rear_x, -0.235, z_offset - 0.855)
    handle_boss = BoxGeometry((0.030, 0.032, 0.068)).translate(side_sign * 0.294, 0.115, z_offset + 0.042)
    lever_perch = CylinderGeometry(radius=0.009, height=0.036, radial_segments=18).rotate_y(pi / 2.0).translate(
        side_sign * 0.287, 0.118, z_offset + 0.052
    )

    geometry = _merged(
        outer_loop,
        front_upper,
        rear_upper,
        front_lower,
        rear_lower,
        center_mount,
        center_gusset,
        front_socket,
        rear_socket,
        handle_boss,
        lever_perch,
    )

    return geometry


def _build_caster_fork_geometry():
    stem = CylinderGeometry(radius=0.012, height=0.080, radial_segments=24).translate(0.000, 0.000, -0.040)
    crown = BoxGeometry((0.058, 0.032, 0.014)).translate(0.000, 0.000, -0.078)
    left_plate = BoxGeometry((0.006, 0.034, 0.090)).translate(-0.022, 0.000, -0.120)
    right_plate = BoxGeometry((0.006, 0.034, 0.090)).translate(0.022, 0.000, -0.120)
    left_boss = CylinderGeometry(radius=0.016, height=0.009, radial_segments=20).rotate_y(pi / 2.0).translate(
        -0.019, 0.000, -0.112
    )
    right_boss = CylinderGeometry(radius=0.016, height=0.009, radial_segments=20).rotate_y(pi / 2.0).translate(
        0.019, 0.000, -0.112
    )
    thrust_collar = CylinderGeometry(radius=0.018, height=0.008, radial_segments=24).translate(0.000, 0.000, -0.004)
    return _merged(stem, crown, left_plate, right_plate, left_boss, right_boss, thrust_collar)


def _build_tip_part(model: ArticulatedObject, name: str, rubber, steel, *, side_sign: float):
    tip = model.part(name)
    tip.visual(
        Cylinder(radius=0.022, length=0.052),
        origin=Origin(xyz=(0.000, 0.000, -0.038)),
        material=rubber,
        name="boot",
    )
    tip.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, -0.016)),
        material=steel,
        name="insert",
    )
    tip.visual(
        Box((0.032, 0.040, 0.012)),
        origin=Origin(xyz=(side_sign * 0.002, 0.006, -0.066)),
        material=rubber,
        name="pad",
    )
    tip.inertial = Inertial.from_geometry(
        Box((0.045, 0.045, 0.070)),
        mass=0.18,
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
    )
    return tip


def _build_caster_wheel_part(model: ArticulatedObject, name: str, rubber, alloy, steel):
    wheel = model.part(name)
    spin_origin = Origin(rpy=(0.000, pi / 2.0, 0.000))
    wheel.visual(
        _mesh(
            f"{name}_tire",
            TorusGeometry(radius=0.040, tube=0.015, radial_segments=18, tubular_segments=32).rotate_y(pi / 2.0),
        ),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=spin_origin,
        material=alloy,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=spin_origin,
        material=steel,
        name="hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.026),
        mass=0.55,
        origin=spin_origin,
    )
    return wheel


def _build_caster_fork_part(model: ArticulatedObject, name: str, frame_paint, steel):
    fork = model.part(name)
    fork.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
        material=frame_paint,
        name="stem",
    )
    fork.visual(
        Box((0.036, 0.036, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=frame_paint,
        name="thrust_collar",
    )
    fork.visual(
        Box((0.050, 0.030, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, -0.058)),
        material=frame_paint,
        name="crown",
    )
    fork.visual(
        Box((0.006, 0.028, 0.070)),
        origin=Origin(xyz=(-0.020, 0.000, -0.095)),
        material=frame_paint,
        name="left_yoke",
    )
    fork.visual(
        Box((0.006, 0.028, 0.070)),
        origin=Origin(xyz=(0.020, 0.000, -0.095)),
        material=frame_paint,
        name="right_yoke",
    )
    fork.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, -0.125), rpy=(0.000, pi / 2.0, 0.000)),
        material=steel,
        name="axle",
    )
    fork.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.000, 0.014, -0.055)),
        material=steel,
        name="grease_nipple",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.150)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, -0.075)),
    )
    return fork


def _add_side_frame_visuals(part, *, side_sign: float, frame_paint, zinc, grip_rubber, folding_side: bool):
    outer_x = side_sign * 0.285
    inner_mount_x = side_sign * 0.039
    inner_block_x = side_sign * 0.060
    bridge_x = side_sign * 0.172
    part.visual(Box((0.050, 0.180, 0.220)), origin=Origin(xyz=(inner_block_x, -0.010, -0.110)), material=zinc, name="center_block")
    part.visual(Box((0.060, 0.300, 0.220)), origin=Origin(xyz=(inner_block_x, -0.010, -0.320)), material=frame_paint, name="lower_spine")
    part.visual(Box((0.250, 0.100, 0.040)), origin=Origin(xyz=(bridge_x, -0.010, -0.020)), material=frame_paint, name="upper_bridge")
    part.visual(Box((0.036, 0.036, 0.700)), origin=Origin(xyz=(outer_x, 0.215, -0.350)), material=frame_paint, name="front_leg")
    part.visual(Box((0.036, 0.036, 0.860)), origin=Origin(xyz=(outer_x, -0.235, -0.430)), material=frame_paint, name="rear_leg")
    part.visual(Box((0.036, 0.490, 0.036)), origin=Origin(xyz=(outer_x, -0.010, -0.010)), material=frame_paint, name="top_rail")
    part.visual(Box((0.250, 0.080, 0.040)), origin=Origin(xyz=(bridge_x, 0.155, -0.330)), material=frame_paint, name="front_brace")
    part.visual(Box((0.250, 0.080, 0.040)), origin=Origin(xyz=(bridge_x, -0.175, -0.380)), material=frame_paint, name="rear_brace")
    part.visual(Box((0.050, 0.050, 0.028)), origin=Origin(xyz=(outer_x, 0.215, -0.714)), material=zinc, name="front_socket")
    part.visual(Box((0.042, 0.042, 0.022)), origin=Origin(xyz=(outer_x, -0.235, -0.859)), material=zinc, name="rear_socket")
    mount_name = "right_mount" if folding_side else "left_mount"
    part.visual(Box((0.022, 0.180, 0.090)), origin=Origin(xyz=(inner_mount_x, -0.010, -0.040)), material=zinc, name=mount_name)
    part.visual(Box((0.042, 0.130, 0.042)), origin=Origin(xyz=(outer_x, 0.090, 0.010)), material=zinc, name="handle_block")
    part.visual(Box((0.020, 0.030, 0.026)), origin=Origin(xyz=(outer_x, 0.155, 0.012)), material=zinc, name="brake_perch")
    part.visual(
        Cylinder(radius=0.019, length=0.145),
        origin=Origin(xyz=(outer_x, 0.020, 0.012), rpy=(pi / 2.0, 0.000, 0.000)),
        material=grip_rubber,
        name="grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_rolling_walker")

    frame_paint = model.material("frame_paint", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    zinc = model.material("zinc", rgba=(0.76, 0.78, 0.80, 1.0))
    service_steel = model.material("service_steel", rgba=(0.46, 0.49, 0.52, 1.0))
    wheel_alloy = model.material("wheel_alloy", rgba=(0.72, 0.74, 0.77, 1.0))

    hinge_carrier = model.part("hinge_carrier")
    hinge_carrier.visual(
        Box((0.032, 0.180, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.880)),
        material=service_steel,
        name="carrier_body",
    )
    hinge_carrier.visual(
        Box((0.012, 0.180, 0.090)),
        origin=Origin(xyz=(-0.022, 0.000, 0.880)),
        material=service_steel,
        name="left_cheek",
    )
    hinge_carrier.visual(
        Box((0.012, 0.180, 0.090)),
        origin=Origin(xyz=(0.022, 0.000, 0.880)),
        material=service_steel,
        name="right_cheek",
    )
    hinge_carrier.visual(
        Box((0.028, 0.110, 0.140)),
        origin=Origin(xyz=(0.000, 0.000, 0.770)),
        material=service_steel,
        name="service_spine",
    )
    hinge_carrier.inertial = Inertial.from_geometry(
        Box((0.070, 0.220, 0.260)),
        mass=0.95,
        origin=Origin(xyz=(0.000, 0.000, 0.820)),
    )

    left_frame = model.part("left_frame")
    _add_side_frame_visuals(
        left_frame,
        side_sign=-1.0,
        frame_paint=frame_paint,
        zinc=zinc,
        grip_rubber=grip_rubber,
        folding_side=False,
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((0.330, 0.560, 0.900)),
        mass=3.8,
        origin=Origin(xyz=(-0.145, -0.010, -0.410)),
    )

    right_frame = model.part("right_frame")
    _add_side_frame_visuals(
        right_frame,
        side_sign=1.0,
        frame_paint=frame_paint,
        zinc=zinc,
        grip_rubber=grip_rubber,
        folding_side=True,
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((0.330, 0.560, 0.900)),
        mass=3.8,
        origin=Origin(xyz=(0.145, -0.010, -0.410)),
    )

    left_tip = _build_tip_part(model, "left_rear_tip", dark_rubber, zinc, side_sign=-1.0)
    right_tip = _build_tip_part(model, "right_rear_tip", dark_rubber, zinc, side_sign=1.0)

    left_caster_fork = _build_caster_fork_part(model, "left_caster_fork", frame_paint, service_steel)
    right_caster_fork = _build_caster_fork_part(model, "right_caster_fork", frame_paint, service_steel)
    left_caster_wheel = _build_caster_wheel_part(model, "left_caster_wheel", dark_rubber, wheel_alloy, service_steel)
    right_caster_wheel = _build_caster_wheel_part(model, "right_caster_wheel", dark_rubber, wheel_alloy, service_steel)

    model.articulation(
        "carrier_to_left_frame",
        ArticulationType.FIXED,
        parent=hinge_carrier,
        child=left_frame,
        origin=Origin(xyz=(0.000, 0.000, 0.920)),
    )
    model.articulation(
        "center_fold",
        ArticulationType.REVOLUTE,
        parent=hinge_carrier,
        child=right_frame,
        origin=Origin(xyz=(0.000, 0.000, 0.920)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=0.000, upper=1.100),
    )
    model.articulation(
        "left_tip_mount",
        ArticulationType.FIXED,
        parent=left_frame,
        child=left_tip,
        origin=Origin(xyz=(-0.285, -0.235, -0.844)),
    )
    model.articulation(
        "right_tip_mount",
        ArticulationType.FIXED,
        parent=right_frame,
        child=right_tip,
        origin=Origin(xyz=(0.285, -0.235, -0.844)),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=left_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(-0.285, 0.215, -0.714)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.285, 0.215, -0.714)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.000, 0.000, -0.125)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.000, 0.000, -0.125)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge_carrier = object_model.get_part("hinge_carrier")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    left_tip = object_model.get_part("left_rear_tip")
    right_tip = object_model.get_part("right_rear_tip")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_wheel = object_model.get_part("left_caster_wheel")
    right_wheel = object_model.get_part("right_caster_wheel")
    fold = object_model.get_articulation("center_fold")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_roll = object_model.get_articulation("left_caster_roll")
    right_roll = object_model.get_articulation("right_caster_roll")

    ctx.allow_overlap(
        left_frame,
        left_tip,
        elem_a="rear_socket",
        elem_b="insert",
        reason="Rear leg insert is modeled as a service-replaceable press-fit ferrule inside the tip socket.",
    )
    ctx.allow_overlap(
        right_frame,
        right_tip,
        elem_a="rear_socket",
        elem_b="insert",
        reason="Rear leg insert is modeled as a service-replaceable press-fit ferrule inside the tip socket.",
    )
    ctx.allow_overlap(
        left_fork,
        left_frame,
        elem_a="stem",
        elem_b="front_socket",
        reason="Caster swivel stem intentionally nests inside the front socket housing.",
    )
    ctx.allow_overlap(
        right_fork,
        right_frame,
        elem_a="stem",
        elem_b="front_socket",
        reason="Caster swivel stem intentionally nests inside the front socket housing.",
    )
    ctx.allow_overlap(
        left_fork,
        left_wheel,
        elem_a="axle",
        elem_b="tire",
        reason="Stub axle passes through the wheel core; the simplified wheel omits a separate bearing bore.",
    )
    ctx.allow_overlap(
        left_fork,
        left_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The axle runs through the caster hub centerline as a real supported shaft.",
    )
    ctx.allow_overlap(
        left_fork,
        left_wheel,
        elem_a="axle",
        elem_b="rim",
        reason="The simplified caster rim is solid-backed, so the real axle path is represented as an intentional nested overlap.",
    )
    ctx.allow_overlap(
        right_fork,
        right_wheel,
        elem_a="axle",
        elem_b="tire",
        reason="Stub axle passes through the wheel core; the simplified wheel omits a separate bearing bore.",
    )
    ctx.allow_overlap(
        right_fork,
        right_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The axle runs through the caster hub centerline as a real supported shaft.",
    )
    ctx.allow_overlap(
        right_fork,
        right_wheel,
        elem_a="axle",
        elem_b="rim",
        reason="The simplified caster rim is solid-backed, so the real axle path is represented as an intentional nested overlap.",
    )
    ctx.allow_overlap(
        left_fork,
        left_frame,
        elem_a="thrust_collar",
        elem_b="front_socket",
        reason="The swivel thrust collar is captured inside the socket block to retain the caster.",
    )
    ctx.allow_overlap(
        right_fork,
        right_frame,
        elem_a="thrust_collar",
        elem_b="front_socket",
        reason="The swivel thrust collar is captured inside the socket block to retain the caster.",
    )
    ctx.allow_overlap(
        left_frame,
        left_tip,
        elem_a="rear_socket",
        elem_b="boot",
        reason="The rubber rear tip boot telescopes slightly into the socket as a replaceable wear part.",
    )
    ctx.allow_overlap(
        left_frame,
        left_tip,
        elem_a="rear_leg",
        elem_b="insert",
        reason="The rear leg tube seats over the tip insert as a replaceable service ferrule.",
    )
    ctx.allow_overlap(
        right_frame,
        right_tip,
        elem_a="rear_socket",
        elem_b="boot",
        reason="The rubber rear tip boot telescopes slightly into the socket as a replaceable wear part.",
    )
    ctx.allow_overlap(
        right_frame,
        right_tip,
        elem_a="rear_leg",
        elem_b="insert",
        reason="The rear leg tube seats over the tip insert as a replaceable service ferrule.",
    )

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

    ctx.expect_contact(hinge_carrier, left_frame, name="left_frame_is_bolted_to_carrier")
    ctx.expect_contact(hinge_carrier, right_frame, name="right_frame_is_hinged_on_carrier")
    ctx.expect_contact(left_tip, left_frame, name="left_tip_is_socketed_onto_frame")
    ctx.expect_contact(right_tip, right_frame, name="right_tip_is_socketed_onto_frame")
    ctx.expect_contact(left_fork, left_frame, name="left_caster_fork_is_carried_by_socket")
    ctx.expect_contact(right_fork, right_frame, name="right_caster_fork_is_carried_by_socket")
    ctx.expect_contact(left_wheel, left_fork, name="left_wheel_is_supported_on_stub_axle")
    ctx.expect_contact(right_wheel, right_fork, name="right_wheel_is_supported_on_stub_axle")

    ctx.expect_origin_gap(left_fork, left_wheel, axis="z", min_gap=0.115, max_gap=0.135, name="left_wheel_hangs_below_swivel")
    ctx.expect_origin_gap(right_fork, right_wheel, axis="z", min_gap=0.115, max_gap=0.135, name="right_wheel_hangs_below_swivel")
    ctx.expect_origin_gap(left_fork, left_tip, axis="y", min_gap=0.40, name="left_caster_is_forward_of_rear_tip")
    ctx.expect_origin_gap(right_fork, right_tip, axis="y", min_gap=0.40, name="right_caster_is_forward_of_rear_tip")

    ctx.check(
        "fold_axis_runs_fore_aft_at_top_carrier",
        tuple(fold.axis) == (0.0, 1.0, 0.0) and fold.motion_limits is not None and fold.motion_limits.upper == 1.1,
        details=f"fold axis/limits = {fold.axis}, {fold.motion_limits}",
    )
    ctx.check(
        "caster_joint_axes_match_real_hardware",
        tuple(left_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(right_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(left_roll.axis) == (1.0, 0.0, 0.0)
        and tuple(right_roll.axis) == (1.0, 0.0, 0.0),
        details=(
            f"swivels=({left_swivel.axis}, {right_swivel.axis}), "
            f"rolls=({left_roll.axis}, {right_roll.axis})"
        ),
    )

    open_left_aabb = ctx.part_world_aabb(left_frame)
    open_right_aabb = ctx.part_world_aabb(right_frame)
    if open_left_aabb is not None and open_right_aabb is not None:
        open_width = open_right_aabb[1][0] - open_left_aabb[0][0]
        with ctx.pose({fold: 0.72, left_swivel: -0.35, right_swivel: 0.35}):
            folded_right_aabb = ctx.part_world_aabb(right_frame)
            if folded_right_aabb is not None:
                folded_width = folded_right_aabb[1][0] - open_left_aabb[0][0]
                ctx.check(
                    "frame_reduces_transport_width_when_folded",
                    folded_width < open_width - 0.03,
                    details=f"open width={open_width:.3f}, folded width={folded_width:.3f}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
