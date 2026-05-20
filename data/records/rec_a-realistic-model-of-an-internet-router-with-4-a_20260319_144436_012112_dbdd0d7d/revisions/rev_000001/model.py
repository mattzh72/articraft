from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_WIDTH = 0.312
BODY_DEPTH = 0.202
BODY_HEIGHT = 0.028

ANTENNA_SPECS = (
    ("antenna_left_outer", "body_to_antenna_left_outer", -0.118, -0.09),
    ("antenna_left_inner", "body_to_antenna_left_inner", -0.040, -0.03),
    ("antenna_right_inner", "body_to_antenna_right_inner", 0.040, 0.03),
    ("antenna_right_outer", "body_to_antenna_right_outer", 0.118, 0.09),
)


def _rounded_profile(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    outline = rounded_rect_profile(width, depth, radius=min(width, depth) * 0.11, corner_segments=10)
    return [(x, y, z) for x, y in outline]


def _build_body_shell():
    shell_geometry = LoftGeometry(
        [
            _rounded_profile(BODY_WIDTH, BODY_DEPTH, 0.000),
            _rounded_profile(BODY_WIDTH - 0.006, BODY_DEPTH - 0.004, 0.008),
            _rounded_profile(BODY_WIDTH - 0.016, BODY_DEPTH - 0.016, 0.024),
            _rounded_profile(BODY_WIDTH - 0.024, BODY_DEPTH - 0.022, BODY_HEIGHT),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(shell_geometry, ASSETS.mesh_path("router_body_shell.obj"))


def _axis_offset(distance: float, tilt: float) -> tuple[float, float, float]:
    return (distance * sin(tilt), 0.0, distance * cos(tilt))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="internet_router_with_4_antennas", assets=ASSETS)

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.23, 0.24, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    led_glass = model.material("led_glass", rgba=(0.40, 0.86, 0.90, 0.45))
    io_black = model.material("io_black", rgba=(0.08, 0.08, 0.09, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(_build_body_shell(), material=body_black, name="shell")
    body.visual(
        Box((0.286, 0.168, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="underside_plate",
    )
    body.visual(
        Box((0.176, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.097, 0.011)),
        material=led_glass,
        name="front_led_strip",
    )
    body.visual(
        Box((0.218, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.097, 0.010)),
        material=io_black,
        name="rear_io_panel",
    )
    for port_x in (-0.068, -0.038, -0.008, 0.022):
        body.visual(
            Box((0.022, 0.010, 0.013)),
            origin=Origin(xyz=(port_x, 0.101, 0.012)),
            material=trim_gray,
            name=f"lan_port_{port_x:+.3f}",
        )
    body.visual(
        Box((0.022, 0.010, 0.013)),
        origin=Origin(xyz=(0.056, 0.101, 0.012)),
        material=led_glass,
        name="wan_port",
    )
    body.visual(
        Box((0.016, 0.010, 0.007)),
        origin=Origin(xyz=(0.088, 0.101, 0.010)),
        material=trim_gray,
        name="usb_port",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.118, 0.101, 0.011), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="power_jack",
    )

    for foot_x in (-0.108, 0.108):
        for foot_y in (-0.065, 0.065):
            body.visual(
                Box((0.032, 0.018, 0.004)),
                origin=Origin(xyz=(foot_x, foot_y, 0.002)),
                material=rubber,
                name=f"foot_{'l' if foot_x < 0 else 'r'}_{'f' if foot_y < 0 else 'b'}",
            )

    for rib_x in (-0.054, -0.036, -0.018, 0.000, 0.018, 0.036, 0.054):
        body.visual(
            Box((0.010, 0.092, 0.002)),
            origin=Origin(xyz=(rib_x, -0.004, BODY_HEIGHT + 0.001)),
            material=trim_gray,
            name=f"vent_rib_{rib_x:+.3f}",
        )

    for antenna_name, _, x_pos, _ in ANTENNA_SPECS:
        body.visual(
            Box((0.028, 0.018, 0.010)),
            origin=Origin(xyz=(x_pos, 0.078, 0.029)),
            material=trim_gray,
            name=f"{antenna_name}_pedestal",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    for part_name, joint_name, x_pos, lateral_tilt in ANTENNA_SPECS:
        mast_center = _axis_offset(0.062, lateral_tilt)
        tip_center = _axis_offset(0.130, lateral_tilt)
        cap_center = _axis_offset(0.1456, lateral_tilt)

        antenna = model.part(part_name)
        antenna.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=trim_gray,
            name="hinge_barrel",
        )
        antenna.visual(
            Box((0.016, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=trim_gray,
            name="hinge_collar",
        )
        antenna.visual(
            Cylinder(radius=0.0048, length=0.112),
            origin=Origin(xyz=mast_center, rpy=(0.0, lateral_tilt, 0.0)),
            material=antenna_black,
            name="main_mast",
        )
        antenna.visual(
            Cylinder(radius=0.0034, length=0.024),
            origin=Origin(xyz=tip_center, rpy=(0.0, lateral_tilt, 0.0)),
            material=antenna_black,
            name="upper_tip",
        )
        antenna.visual(
            Sphere(radius=0.0036),
            origin=Origin(xyz=cap_center),
            material=antenna_black,
            name="end_cap",
        )
        antenna.inertial = Inertial.from_geometry(
            Cylinder(radius=0.005, length=0.140),
            mass=0.06,
            origin=Origin(xyz=(0.0, 0.0, 0.070)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent="body",
            child=part_name,
            origin=Origin(xyz=(x_pos, 0.078, 0.032)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.5,
                lower=-1.15,
                upper=0.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for antenna_name, _, _, _ in ANTENNA_SPECS:
        ctx.expect_aabb_contact("body", antenna_name)
        ctx.expect_aabb_overlap("body", antenna_name, axes="xy", min_overlap=0.008)

    ctx.expect_origin_distance("antenna_left_outer", "antenna_left_inner", axes="y", max_dist=0.005)
    ctx.expect_origin_distance("antenna_left_outer", "antenna_left_inner", axes="z", max_dist=0.005)
    ctx.expect_origin_distance("antenna_right_outer", "antenna_right_inner", axes="y", max_dist=0.005)
    ctx.expect_origin_distance("antenna_right_outer", "antenna_right_inner", axes="z", max_dist=0.005)
    ctx.expect_origin_distance("antenna_left_outer", "antenna_right_outer", axes="y", max_dist=0.005)
    ctx.expect_origin_distance("antenna_left_outer", "antenna_right_outer", axes="z", max_dist=0.005)

    raised_pose = {joint_name: 0.30 for _, joint_name, _, _ in ANTENNA_SPECS}
    with ctx.pose(raised_pose):
        for antenna_name, _, _, _ in ANTENNA_SPECS:
            ctx.expect_aabb_contact("body", antenna_name)
            ctx.expect_aabb_overlap("body", antenna_name, axes="y", min_overlap=0.012)

    for antenna_name, joint_name, _, _ in ANTENNA_SPECS:
        ctx.expect_joint_motion_axis(
            joint_name,
            antenna_name,
            world_axis="z",
            direction="positive",
            min_delta=0.02,
        )

    ctx.expect_joint_motion_axis(
        "body_to_antenna_left_inner",
        "antenna_left_inner",
        world_axis="y",
        direction="negative",
        min_delta=0.03,
    )

    mount_positions = [ctx.part_world_position(name) for name, _, _, _ in ANTENNA_SPECS]
    xs = [position[0] for position in mount_positions]
    ys = [position[1] for position in mount_positions]
    assert xs[0] < xs[1] < xs[2] < xs[3], "Router antennas should read left-to-right across the chassis."
    assert xs[0] < -0.10 and xs[3] > 0.10, "Outer antennas should sit near the router shoulders."
    assert min(ys) > 0.06 and max(ys) - min(ys) < 0.01, "All four antennas should mount in a rear row."

    folded_pose = {joint_name: -1.0 for _, joint_name, _, _ in ANTENNA_SPECS}
    with ctx.pose(folded_pose):
        for antenna_name, _, _, _ in ANTENNA_SPECS:
            ctx.expect_aabb_contact("body", antenna_name)
            ctx.expect_aabb_overlap("body", antenna_name, axes="x", min_overlap=0.008)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
