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
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _rounded_rect_loop(
    width: float,
    depth: float,
    z: float,
    radius: float,
    *,
    y_shift: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _rounded_loft_mesh(
    filename: str,
    sections: list[tuple[float, float, float, float, float]],
    *,
    cap: bool,
) -> object:
    profiles = [
        _rounded_rect_loop(width, depth, z, radius, y_shift=y_shift)
        for z, width, depth, radius, y_shift in sections
    ]
    return mesh_from_geometry(
        LoftGeometry(profiles, cap=cap, closed=True),
        ASSETS.mesh_path(filename),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin", assets=ASSETS)

    body_green = Material(name="body_green", rgba=(0.196, 0.314, 0.220, 1.0))
    lid_green = Material(name="lid_green", rgba=(0.176, 0.286, 0.204, 1.0))
    wheel_black = Material(name="wheel_black", rgba=(0.085, 0.085, 0.085, 1.0))
    hub_gray = Material(name="hub_gray", rgba=(0.585, 0.595, 0.620, 1.0))
    model.materials.extend([body_green, lid_green, wheel_black, hub_gray])

    body_shell = _rounded_loft_mesh(
        "wheelie_bin_body_shell.obj",
        [
            (0.020, 0.420, 0.480, 0.040, 0.000),
            (0.180, 0.465, 0.585, 0.044, 0.000),
            (0.520, 0.545, 0.690, 0.056, 0.000),
            (0.860, 0.585, 0.740, 0.068, 0.000),
        ],
        cap=False,
    )
    inner_collar = _rounded_loft_mesh(
        "wheelie_bin_inner_collar.obj",
        [
            (0.735, 0.500, 0.640, 0.050, 0.000),
            (0.855, 0.535, 0.690, 0.056, 0.000),
        ],
        cap=False,
    )
    lid_shell = _rounded_loft_mesh(
        "wheelie_bin_lid_shell.obj",
        [
            (0.000, 0.600, 0.730, 0.050, 0.000),
            (0.028, 0.586, 0.718, 0.060, 0.000),
            (0.055, 0.560, 0.688, 0.072, 0.000),
        ],
        cap=True,
    )

    body = model.part("body")
    body.visual(body_shell, material=body_green, name="body_shell")
    body.visual(inner_collar, material=body_green, name="inner_collar")
    body.visual(
        Box((0.360, 0.420, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=body_green,
        name="bottom_plate",
    )
    body.visual(
        Box((0.520, 0.034, 0.028)),
        origin=Origin(xyz=(0.0, 0.356, 0.872)),
        material=body_green,
        name="front_rim",
    )
    body.visual(
        Box((0.500, 0.028, 0.032)),
        origin=Origin(xyz=(0.0, -0.347, 0.872)),
        material=body_green,
        name="rear_rim",
    )
    body.visual(
        Box((0.032, 0.660, 0.028)),
        origin=Origin(xyz=(0.278, 0.0, 0.872)),
        material=body_green,
        name="right_rim",
    )
    body.visual(
        Box((0.032, 0.660, 0.028)),
        origin=Origin(xyz=(-0.278, 0.0, 0.872)),
        material=body_green,
        name="left_rim",
    )
    body.visual(
        Box((0.440, 0.018, 0.240)),
        origin=Origin(xyz=(0.0, 0.338, 0.520)),
        material=body_green,
        name="front_panel",
    )
    body.visual(
        Box((0.045, 0.022, 0.460)),
        origin=Origin(xyz=(0.125, 0.338, 0.390)),
        material=body_green,
        name="front_rib_right",
    )
    body.visual(
        Box((0.045, 0.022, 0.460)),
        origin=Origin(xyz=(-0.125, 0.338, 0.390)),
        material=body_green,
        name="front_rib_left",
    )
    body.visual(
        Box((0.032, 0.340, 0.520)),
        origin=Origin(xyz=(0.244, 0.050, 0.460)),
        material=body_green,
        name="right_side_rib",
    )
    body.visual(
        Box((0.032, 0.340, 0.520)),
        origin=Origin(xyz=(-0.244, 0.050, 0.460)),
        material=body_green,
        name="left_side_rib",
    )
    body.visual(
        Box((0.048, 0.110, 0.190)),
        origin=Origin(xyz=(0.176, -0.228, 0.155)),
        material=body_green,
        name="right_wheel_housing",
    )
    body.visual(
        Box((0.048, 0.110, 0.190)),
        origin=Origin(xyz=(-0.176, -0.228, 0.155)),
        material=body_green,
        name="left_wheel_housing",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(xyz=(0.236, -0.225, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="right_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(xyz=(-0.236, -0.225, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="left_axle_stub",
    )
    body.visual(
        Box((0.300, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, -0.236, 0.060)),
        material=body_green,
        name="rear_step_bar",
    )
    body.visual(
        Box((0.055, 0.070, 0.090)),
        origin=Origin(xyz=(0.162, -0.385, 0.895)),
        material=body_green,
        name="right_handle_tower",
    )
    body.visual(
        Box((0.055, 0.070, 0.090)),
        origin=Origin(xyz=(-0.162, -0.385, 0.895)),
        material=body_green,
        name="left_handle_tower",
    )
    body.visual(
        Box((0.340, 0.036, 0.030)),
        origin=Origin(xyz=(0.0, -0.410, 0.922)),
        material=body_green,
        name="handle_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.590, 0.760, 0.920)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.0, 0.358, 0.016)),
        material=lid_green,
        name="lid_shell",
    )
    lid.visual(
        Box((0.550, 0.032, 0.026)),
        origin=Origin(xyz=(0.0, 0.010, 0.024)),
        material=lid_green,
        name="rear_flange",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(0.0, 0.000, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_green,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.260, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.712, 0.020)),
        material=lid_green,
        name="front_grab_lip",
    )
    lid.visual(
        Box((0.220, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.565, 0.056)),
        material=lid_green,
        name="top_grip_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.610, 0.740, 0.085)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.360, 0.040)),
    )

    for wheel_name in ("left_wheel", "right_wheel"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.135, length=0.042),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_black,
            name=f"{wheel_name}_tire",
        )
        wheel.visual(
            Cylinder(radius=0.052, length=0.050),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_gray,
            name=f"{wheel_name}_hub",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=0.058),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_gray,
            name=f"{wheel_name}_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.135, length=0.042),
            mass=1.1,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.0, -0.334, 0.872)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "left_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="left_wheel",
        origin=Origin(xyz=(-0.274, -0.225, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "right_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="right_wheel",
        origin=Origin(xyz=(0.274, -0.225, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=18.0,
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
        "lid",
        reason="Molded lid seal and hinge knuckles sit in very close contact and conservative collision hulls can overestimate the overlap.",
    )
    ctx.allow_overlap(
        "body",
        "left_wheel",
        reason="The wheel hub nests over a short molded axle stub, and the generated collision hulls are conservative around that hub interface.",
    )
    ctx.allow_overlap(
        "body",
        "right_wheel",
        reason="The wheel hub nests over a short molded axle stub, and the generated collision hulls are conservative around that hub interface.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.18)
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )

    body_pos = ctx.part_world_position("body")
    lid_pos = ctx.part_world_position("lid")
    left_wheel_pos = ctx.part_world_position("left_wheel")
    right_wheel_pos = ctx.part_world_position("right_wheel")

    assert lid_pos[1] < body_pos[1] - 0.25, "lid hinge should sit at the rear top of the bin body"
    assert lid_pos[2] > 0.84, "lid hinge should sit near the top opening"
    assert left_wheel_pos[0] < body_pos[0] - 0.15, (
        "left wheel should sit clearly outboard of the body"
    )
    assert right_wheel_pos[0] > body_pos[0] + 0.15, (
        "right wheel should sit clearly outboard of the body"
    )
    assert left_wheel_pos[1] < body_pos[1] - 0.16, (
        "left wheel should sit behind the body centerline"
    )
    assert right_wheel_pos[1] < body_pos[1] - 0.16, (
        "right wheel should sit behind the body centerline"
    )
    assert abs(left_wheel_pos[1] - right_wheel_pos[1]) < 1e-6, (
        "wheel axle centers should line up front-to-back"
    )
    assert right_wheel_pos[0] - left_wheel_pos[0] > 0.50, (
        "wheel track should be wide enough to stabilize the bin"
    )
    assert 0.10 < left_wheel_pos[2] < 0.18, "left wheel should sit low enough to roll the bin"
    assert 0.10 < right_wheel_pos[2] < 0.18, "right wheel should sit low enough to roll the bin"

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
