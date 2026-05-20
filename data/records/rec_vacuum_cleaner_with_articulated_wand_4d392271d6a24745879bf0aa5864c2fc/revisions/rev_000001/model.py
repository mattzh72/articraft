from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_articulated_vacuum")

    pearl = model.material("warm_white_plastic", rgba=(0.86, 0.86, 0.82, 1.0))
    charcoal = model.material("charcoal_rubber", rgba=(0.035, 0.038, 0.042, 1.0))
    dark = model.material("dark_gray_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    graphite = model.material("graphite_collars", rgba=(0.24, 0.25, 0.26, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.62, 0.65, 0.68, 1.0))
    accent = model.material("blue_release_buttons", rgba=(0.08, 0.38, 0.82, 1.0))
    brush = model.material("soft_brush_strip", rgba=(0.12, 0.17, 0.23, 1.0))

    body = model.part("body")
    body_shell = superellipse_side_loft(
        [
            (-0.185, 0.112, 0.135, 0.155),
            (-0.120, 0.124, 0.205, 0.205),
            (-0.020, 0.132, 0.230, 0.230),
            (0.090, 0.128, 0.215, 0.215),
            (0.185, 0.118, 0.155, 0.165),
        ],
        exponents=2.45,
        segments=56,
    )
    body.visual(_mesh(body_shell, "rounded_body_shell"), material=pearl, name="rounded_body_shell")

    # Embedded bumpers and rear exhaust details are slightly sunk into the shell so
    # they read as one manufactured canister, not separate floating decorations.
    body.visual(Box((0.355, 0.022, 0.036)), origin=Origin(xyz=(0.000, -0.122, 0.085)), material=charcoal, name="side_bumper_0")
    body.visual(Box((0.355, 0.022, 0.036)), origin=Origin(xyz=(0.000, 0.122, 0.085)), material=charcoal, name="side_bumper_1")
    body.visual(Box((0.020, 0.105, 0.060)), origin=Origin(xyz=(-0.188, 0.000, 0.128)), material=dark, name="rear_vent_panel")
    for z in (0.106, 0.124, 0.142, 0.160):
        body.visual(Box((0.024, 0.088, 0.004)), origin=Origin(xyz=(-0.198, 0.000, z)), material=graphite, name=f"rear_vent_louver_{z:.3f}")

    # A reinforced hose port, stow clips, wheels, and a top carry handle all tie
    # into the shell, keeping the root body a single coherent physical part.
    body.visual(Cylinder(radius=0.050, length=0.026), origin=Origin(xyz=(0.204, 0.0, 0.170), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="front_port_ring")
    body.visual(Cylinder(radius=0.035, length=0.020), origin=Origin(xyz=(0.192, 0.0, 0.170), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="front_socket_shadow")
    body.visual(Box((0.080, 0.020, 0.036)), origin=Origin(xyz=(-0.040, -0.128, 0.205)), material=graphite, name="wand_clip_0")
    body.visual(Box((0.080, 0.020, 0.036)), origin=Origin(xyz=(0.095, -0.128, 0.205)), material=graphite, name="wand_clip_1")
    body.visual(Box((0.020, 0.020, 0.045)), origin=Origin(xyz=(-0.040, -0.142, 0.220)), material=graphite, name="clip_lip_0")
    body.visual(Box((0.020, 0.020, 0.045)), origin=Origin(xyz=(0.095, -0.142, 0.220)), material=graphite, name="clip_lip_1")
    handle_geom = tube_from_spline_points(
        [(-0.125, 0.0, 0.232), (-0.065, 0.0, 0.270), (0.040, 0.0, 0.276), (0.125, 0.0, 0.232)],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
    )
    body.visual(_mesh(handle_geom, "top_carry_handle"), material=charcoal, name="top_carry_handle")
    for x in (-0.125, 0.125):
        body.visual(Cylinder(radius=0.033, length=0.022), origin=Origin(xyz=(x, -0.119, 0.035), rpy=(pi / 2.0, 0.0, 0.0)), material=charcoal, name=f"side_wheel_{x}_0")
        body.visual(Cylinder(radius=0.033, length=0.022), origin=Origin(xyz=(x, 0.119, 0.035), rpy=(pi / 2.0, 0.0, 0.0)), material=charcoal, name=f"side_wheel_{x}_1")
        body.visual(Cylinder(radius=0.014, length=0.270), origin=Origin(xyz=(x, 0.0, 0.035), rpy=(pi / 2.0, 0.0, 0.0)), material=graphite, name=f"wheel_axle_{x}")

    socket = model.part("socket")
    socket.visual(Cylinder(radius=0.038, length=0.018), origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="swivel_collar")
    socket.visual(Cylinder(radius=0.026, length=0.064), origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="short_hose_stub")
    socket.visual(Box((0.052, 0.016, 0.072)), origin=Origin(xyz=(0.082, -0.039, 0.0)), material=graphite, name="pitch_fork_0")
    socket.visual(Box((0.052, 0.016, 0.072)), origin=Origin(xyz=(0.082, 0.039, 0.0)), material=graphite, name="pitch_fork_1")
    socket.visual(Box((0.028, 0.094, 0.022)), origin=Origin(xyz=(0.060, 0.0, -0.030)), material=graphite, name="fork_bridge")

    lower_wand = model.part("lower_wand")
    lower_wand.visual(Cylinder(radius=0.026, length=0.062), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=metal, name="base_pivot_pin")
    lower_wand.visual(Cylinder(radius=0.016, length=0.310), origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=metal, name="lower_tube")
    lower_wand.visual(Cylinder(radius=0.023, length=0.036), origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="base_lock_collar")
    lower_wand.visual(Cylinder(radius=0.024, length=0.030), origin=Origin(xyz=(0.322, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="elbow_lock_collar")
    lower_wand.visual(Box((0.042, 0.014, 0.064)), origin=Origin(xyz=(0.357, -0.038, 0.0)), material=graphite, name="elbow_fork_0")
    lower_wand.visual(Box((0.042, 0.014, 0.064)), origin=Origin(xyz=(0.357, 0.038, 0.0)), material=graphite, name="elbow_fork_1")
    lower_wand.visual(Box((0.026, 0.090, 0.016)), origin=Origin(xyz=(0.342, 0.0, -0.038)), material=graphite, name="elbow_fork_bridge")
    lower_wand.visual(Box((0.020, 0.036, 0.012)), origin=Origin(xyz=(0.265, 0.0, 0.021)), material=accent, name="release_button")

    upper_wand = model.part("upper_wand")
    upper_wand.visual(Cylinder(radius=0.026, length=0.062), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=metal, name="elbow_pivot_pin")
    upper_wand.visual(Cylinder(radius=0.014, length=0.430), origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=metal, name="upper_tube")
    upper_wand.visual(Cylinder(radius=0.022, length=0.042), origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="fold_lock_collar")
    upper_wand.visual(Cylinder(radius=0.023, length=0.035), origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="nozzle_lock_collar")
    upper_wand.visual(Box((0.040, 0.014, 0.058)), origin=Origin(xyz=(0.468, -0.038, 0.0)), material=graphite, name="nozzle_fork_0")
    upper_wand.visual(Box((0.040, 0.014, 0.058)), origin=Origin(xyz=(0.468, 0.038, 0.0)), material=graphite, name="nozzle_fork_1")
    upper_wand.visual(Box((0.024, 0.090, 0.016)), origin=Origin(xyz=(0.452, 0.0, -0.036)), material=graphite, name="nozzle_fork_bridge")
    upper_wand.visual(Box((0.030, 0.036, 0.012)), origin=Origin(xyz=(0.390, 0.0, 0.020)), material=accent, name="length_release_button")

    nozzle_neck = model.part("nozzle_neck")
    nozzle_neck.visual(Cylinder(radius=0.024, length=0.062), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=metal, name="wand_pivot_pin")
    neck_tube = tube_from_spline_points(
        [(0.000, 0.0, 0.0), (0.048, 0.0, -0.026), (0.086, 0.0, -0.060)],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=16,
    )
    nozzle_neck.visual(_mesh(neck_tube, "curved_neck_tube"), material=metal, name="curved_neck_tube")
    nozzle_neck.visual(Box((0.022, 0.100, 0.022)), origin=Origin(xyz=(0.082, 0.0, -0.066)), material=graphite, name="neck_to_fork_web")
    nozzle_neck.visual(Box((0.032, 0.014, 0.050)), origin=Origin(xyz=(0.105, -0.049, -0.066)), material=graphite, name="nozzle_fork_0")
    nozzle_neck.visual(Box((0.032, 0.014, 0.050)), origin=Origin(xyz=(0.105, 0.049, -0.066)), material=graphite, name="nozzle_fork_1")
    nozzle_neck.visual(Box((0.024, 0.092, 0.012)), origin=Origin(xyz=(0.059, 0.0, -0.080)), material=graphite, name="nozzle_fork_bridge")

    nozzle = model.part("floor_nozzle")
    nozzle.visual(Cylinder(radius=0.018, length=0.084), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=metal, name="nozzle_pivot_bar")
    nozzle.visual(Box((0.195, 0.305, 0.045)), origin=Origin(xyz=(0.078, 0.0, -0.026)), material=dark, name="low_profile_nozzle_shell")
    nozzle.visual(Box((0.150, 0.245, 0.006)), origin=Origin(xyz=(0.080, 0.0, -0.050)), material=charcoal, name="suction_slot")
    nozzle.visual(Box((0.018, 0.260, 0.014)), origin=Origin(xyz=(0.162, 0.0, -0.047)), material=brush, name="front_brush_strip")
    nozzle.visual(Box((0.020, 0.300, 0.010)), origin=Origin(xyz=(0.040, 0.0, -0.019)), material=graphite, name="rear_hinge_rail")
    for y in (-0.135, 0.135):
        nozzle.visual(Cylinder(radius=0.018, length=0.018), origin=Origin(xyz=(0.125, y, -0.055), rpy=(pi / 2.0, 0.0, 0.0)), material=charcoal, name=f"edge_roller_{y}")

    body_to_socket = model.articulation(
        "body_to_socket",
        ArticulationType.REVOLUTE,
        parent=body,
        child=socket,
        origin=Origin(xyz=(0.230, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=1.05),
    )
    socket_to_lower = model.articulation(
        "socket_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=lower_wand,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.95, upper=0.42),
    )
    lower_to_upper = model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=-2.50, upper=0.20),
    )
    upper_to_neck = model.articulation(
        "upper_wand_to_neck",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=nozzle_neck,
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.5, lower=-1.05, upper=0.75),
    )
    neck_to_nozzle = model.articulation(
        "neck_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=nozzle_neck,
        child=nozzle,
        origin=Origin(xyz=(0.105, 0.0, -0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.65, upper=0.65),
    )

    # Preserve names for run_tests() while letting the SDK own the final graph.
    assert body_to_socket and socket_to_lower and lower_to_upper and upper_to_neck and neck_to_nozzle
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    socket = object_model.get_part("socket")
    lower = object_model.get_part("lower_wand")
    upper = object_model.get_part("upper_wand")
    neck = object_model.get_part("nozzle_neck")
    nozzle = object_model.get_part("floor_nozzle")

    ctx.expect_overlap(socket, body, axes="yz", min_overlap=0.045, name="swivel overlaps body port footprint")
    ctx.expect_gap(socket, body, axis="x", min_gap=0.0, max_gap=0.010, name="swivel collar seats on port without penetration")
    ctx.expect_origin_gap(nozzle, body, axis="x", min_gap=0.78, max_gap=1.35, name="operating wand reaches beyond compact body")

    aabb = ctx.part_world_aabb(nozzle)
    if aabb is not None:
        nozzle_bottom = aabb[0][2]
        nozzle_top = aabb[1][2]
        ctx.check(
            "nozzle rides close to floor",
            0.0 <= nozzle_bottom <= 0.025 and nozzle_top < 0.105,
            details=f"nozzle z bounds: bottom={nozzle_bottom:.4f}, top={nozzle_top:.4f}",
        )

    fold = object_model.get_articulation("lower_to_upper_wand")
    neck_pitch = object_model.get_articulation("upper_wand_to_neck")
    nozzle_pitch = object_model.get_articulation("neck_to_nozzle")
    with ctx.pose({fold: -2.45, neck_pitch: 0.45, nozzle_pitch: -0.25}):
        upper_box = ctx.part_world_aabb(upper)
        neck_box = ctx.part_world_aabb(neck)
        body_box = ctx.part_world_aabb(body)
        if upper_box is not None and neck_box is not None and body_box is not None:
            stowed_reach = max(upper_box[1][0], neck_box[1][0]) - body_box[1][0]
            ctx.check(
                "folded wand stays near body envelope",
                stowed_reach < 0.55,
                details=f"folded wand projects {stowed_reach:.3f} m past front body",
            )

    ctx.check(
        "five user-facing wand joints are articulated",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "body_to_socket",
                "socket_to_lower_wand",
                "lower_to_upper_wand",
                "upper_wand_to_neck",
                "neck_to_nozzle",
            )
        ),
    )

    return ctx.report()


object_model = build_object_model()
