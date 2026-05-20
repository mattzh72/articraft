from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            return Material(name, rgba)


def _mesh_path(name: str):
    try:
        return ASSETS.mesh_path(name)
    except AttributeError:
        return ASSETS.mesh_dir / name


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * cos((tau * i) / segments), radius * sin((tau * i) / segments))
        for i in range(segments)
    ]


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, _mesh_path(name))


def _selector_knob_mesh(name: str):
    body = LatheGeometry(
        [
            (0.0, 0.000),
            (0.028, 0.000),
            (0.038, 0.003),
            (0.041, 0.012),
            (0.040, 0.024),
            (0.034, 0.033),
            (0.022, 0.038),
            (0.0, 0.038),
        ],
        segments=40,
    )
    indicator = BoxGeometry((0.010, 0.012, 0.012)).translate(0.0, 0.031, 0.024)
    body.merge(indicator)
    body.rotate_x(pi / 2.0)
    return mesh_from_geometry(body, _mesh_path(name))


def _door_handle_mesh(name: str):
    geom = tube_from_spline_points(
        [
            (0.432, -0.044, -0.102),
            (0.466, -0.084, -0.055),
            (0.474, -0.094, 0.000),
            (0.466, -0.084, 0.055),
            (0.432, -0.044, 0.102),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, _mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dryer", assets=ASSETS)

    materials = {
        "white_enamel": _material("white_enamel", (0.95, 0.95, 0.96, 1.0)),
        "graphite_plastic": _material("graphite_plastic", (0.15, 0.16, 0.18, 1.0)),
        "dark_glass": _material("dark_glass", (0.10, 0.12, 0.14, 0.45)),
        "display_black": _material("display_black", (0.05, 0.06, 0.07, 0.88)),
        "brushed_steel": _material("brushed_steel", (0.75, 0.77, 0.80, 1.0)),
        "rubber": _material("rubber", (0.19, 0.20, 0.21, 1.0)),
        "chrome": _material("chrome", (0.84, 0.85, 0.87, 1.0)),
    }
    model.materials.extend(materials.values())

    cabinet_bezel_mesh = _annulus_mesh("cabinet_bezel.obj", 0.293, 0.222, 0.018)
    gasket_mesh = _annulus_mesh("door_gasket.obj", 0.245, 0.198, 0.022)
    drum_lip_mesh = _annulus_mesh("drum_lip.obj", 0.279, 0.196, 0.080)
    door_outer_mesh = _annulus_mesh("door_outer_ring.obj", 0.288, 0.205, 0.040)
    door_inner_mesh = _annulus_mesh("door_inner_ring.obj", 0.242, 0.178, 0.024)
    selector_mesh = _selector_knob_mesh("selector_knob.obj")
    handle_mesh = _door_handle_mesh("door_handle.obj")

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.02, 0.72, 0.85)),
        origin=Origin(xyz=(-0.325, 0.0, 0.455)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.02, 0.72, 0.85)),
        origin=Origin(xyz=(0.325, 0.0, 0.455)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.63, 0.72, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.63, 0.72, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.870)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.63, 0.02, 0.81)),
        origin=Origin(xyz=(0.0, 0.350, 0.445)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.11, 0.04, 0.70)),
        origin=Origin(xyz=(-0.280, -0.340, 0.380)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.11, 0.04, 0.70)),
        origin=Origin(xyz=(0.280, -0.340, 0.380)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.45, 0.04, 0.11)),
        origin=Origin(xyz=(0.0, -0.340, 0.085)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.45, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, -0.340, 0.670)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.63, 0.04, 0.11)),
        origin=Origin(xyz=(0.0, -0.340, 0.085)),
        material=materials["graphite_plastic"],
    )
    cabinet.visual(
        Box((0.63, 0.20, 0.15)),
        origin=Origin(xyz=(0.0, -0.240, 0.790)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.63, 0.02, 0.10)),
        origin=Origin(xyz=(0.0, -0.350, 0.790)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        Box((0.21, 0.008, 0.090)),
        origin=Origin(xyz=(-0.170, -0.356, 0.790)),
        material=materials["graphite_plastic"],
    )
    cabinet.visual(
        Box((0.25, 0.008, 0.068)),
        origin=Origin(xyz=(0.150, -0.356, 0.824)),
        material=materials["display_black"],
    )
    cabinet.visual(
        Box((0.24, 0.008, 0.040)),
        origin=Origin(xyz=(0.160, -0.356, 0.770)),
        material=materials["graphite_plastic"],
    )
    cabinet.visual(
        cabinet_bezel_mesh,
        origin=Origin(xyz=(0.0, -0.329, 0.400)),
        material=materials["white_enamel"],
    )
    cabinet.visual(
        gasket_mesh,
        origin=Origin(xyz=(0.0, -0.349, 0.400)),
        material=materials["rubber"],
    )
    cabinet.visual(
        Cylinder(radius=0.245, length=0.44),
        origin=Origin(xyz=(0.0, 0.080, 0.400), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
    )
    cabinet.visual(
        drum_lip_mesh,
        origin=Origin(xyz=(0.0, -0.180, 0.400)),
        material=materials["brushed_steel"],
    )
    cabinet.visual(
        Cylinder(radius=0.230, length=0.040),
        origin=Origin(xyz=(0.0, 0.320, 0.400), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
    )
    cabinet.visual(
        Cylinder(radius=0.040, length=0.080),
        origin=Origin(xyz=(0.0, 0.350, 0.400), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["graphite_plastic"],
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(-0.240, -0.270, 0.015)),
        material=materials["rubber"],
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.240, -0.270, 0.015)),
        material=materials["rubber"],
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(-0.240, 0.270, 0.015)),
        material=materials["rubber"],
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.240, 0.270, 0.015)),
        material=materials["rubber"],
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.67, 0.72, 0.88)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
    )

    door = model.part("door")
    door.visual(
        door_outer_mesh,
        origin=Origin(xyz=(0.235, -0.043, 0.0)),
        material=materials["graphite_plastic"],
    )
    door.visual(
        door_inner_mesh,
        origin=Origin(xyz=(0.235, -0.056, 0.0)),
        material=materials["graphite_plastic"],
    )
    door.visual(
        Cylinder(radius=0.206, length=0.010),
        origin=Origin(xyz=(0.235, -0.049, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["dark_glass"],
    )
    door.visual(handle_mesh, material=materials["chrome"])
    door.visual(
        Box((0.050, 0.028, 0.220)),
        origin=Origin(xyz=(0.015, -0.030, 0.0)),
        material=materials["graphite_plastic"],
    )
    door.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(0.000, -0.016, 0.0)),
        material=materials["chrome"],
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=0.285, length=0.045),
        mass=4.8,
        origin=Origin(xyz=(0.235, -0.043, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        selector_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.038),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.036, 0.014, 0.017)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material=materials["graphite_plastic"],
    )
    power_button.inertial = Inertial.from_geometry(
        Box((0.036, 0.014, 0.017)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.050, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material=materials["graphite_plastic"],
    )
    start_button.inertial = Inertial.from_geometry(
        Box((0.050, 0.014, 0.018)),
        mass=0.04,
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="door",
        origin=Origin(xyz=(-0.235, -0.349, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-2.35,
            upper=0.0,
        ),
    )
    model.articulation(
        "selector_dial",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="selector_knob",
        origin=Origin(xyz=(-0.170, -0.360, 0.790)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
            lower=-2.40,
            upper=2.40,
        ),
    )
    model.articulation(
        "power_button_press",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="power_button",
        origin=Origin(xyz=(0.090, -0.350, 0.770)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "start_button_press",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="start_button",
        origin=Origin(xyz=(0.220, -0.350, 0.770)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
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
        "door",
        "cabinet",
        reason="The closed door compresses against the front gasket and conservative hulls overfill the circular seal.",
    )
    ctx.allow_overlap(
        "power_button",
        "cabinet",
        reason="The power button travels into a shallow switch pocket in the front fascia.",
    )
    ctx.allow_overlap(
        "start_button",
        "cabinet",
        reason="The start button travels into a shallow switch pocket in the front fascia.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "door_hinge",
        "door",
        world_axis="y",
        direction="positive",
        min_delta=0.07,
    )
    ctx.expect_joint_motion_axis(
        "power_button_press",
        "power_button",
        world_axis="y",
        direction="positive",
        min_delta=0.002,
    )
    ctx.expect_joint_motion_axis(
        "start_button_press",
        "start_button",
        world_axis="y",
        direction="positive",
        min_delta=0.002,
    )
    ctx.expect_aabb_overlap("door", "cabinet", axes="xy", min_overlap=0.003)
    ctx.expect_aabb_overlap("power_button", "cabinet", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_overlap("start_button", "cabinet", axes="xy", min_overlap=0.010)
    ctx.expect_origin_distance("selector_knob", "power_button", axes="xy", max_dist=0.27)
    ctx.expect_origin_distance("selector_knob", "start_button", axes="xy", max_dist=0.40)

    with ctx.pose(door_hinge=-2.10):
        ctx.expect_origin_distance("door", "cabinet", axes="xy", max_dist=0.70)

    with ctx.pose(selector_dial=2.20):
        ctx.expect_origin_distance("selector_knob", "power_button", axes="xy", max_dist=0.27)
        ctx.expect_origin_distance("selector_knob", "start_button", axes="xy", max_dist=0.40)

    with ctx.pose(selector_dial=-2.20):
        ctx.expect_origin_distance("selector_knob", "power_button", axes="xy", max_dist=0.27)
        ctx.expect_origin_distance("selector_knob", "start_button", axes="xy", max_dist=0.40)

    with ctx.pose(power_button_press=0.004, start_button_press=0.004):
        ctx.expect_aabb_overlap("power_button", "cabinet", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_overlap("start_button", "cabinet", axes="xy", min_overlap=0.010)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
