from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, center: tuple[float, float] = (0.0, 0.0), segments: int = 48):
    cx, cy = center
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _tube_shell(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, name)


def _shade_shell():
    shade = LatheGeometry.from_shell_profiles(
        [(0.034, 0.000), (0.060, 0.085), (0.095, 0.180)],
        [(0.023, 0.006), (0.049, 0.085), (0.082, 0.165)],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    # Lathe geometry is built along local +Z.  Rotate it so the shade axis
    # points along the shade part's local +X from its tilt trunnion.
    shade.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shade, "shade_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_gooseneck_desk_lamp")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.24, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    shade_blue = model.material("shade_blue", rgba=(0.03, 0.16, 0.22, 1.0))
    warm_white = model.material("warm_white", rgba=(1.0, 0.86, 0.52, 1.0))

    post_x = -0.105

    base = model.part("base")
    base_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.34, 0.20, 0.030, corner_segments=10),
        [_circle_profile(0.034, center=(post_x, 0.0), segments=56)],
        0.045,
        center=False,
    )
    base.visual(mesh_from_geometry(base_plate, "base_plate"), material=matte_black, name="base_plate")
    base.visual(
        _tube_shell("base_socket", outer_radius=0.049, inner_radius=0.027, z_min=0.040, z_max=0.145),
        material=dark_metal,
        origin=Origin(xyz=(post_x, 0.0, 0.0)),
        name="base_socket",
    )
    # Four low rubber feet make the weighted base read as a real desktop object.
    for idx, (x, y) in enumerate(((-0.130, -0.070), (-0.130, 0.070), (0.130, -0.070), (0.130, 0.070))):
        base.visual(
            Box((0.045, 0.030, 0.006)),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )
    cord = tube_from_spline_points(
        [(-0.170, 0.0, 0.018), (-0.225, -0.020, 0.015), (-0.285, -0.055, 0.014), (-0.345, -0.050, 0.014)],
        radius=0.004,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    base.visual(mesh_from_geometry(cord, "power_cord"), material=rubber, name="power_cord")

    stage_0 = model.part("post_stage_0")
    stage_0.visual(
        _tube_shell("stage_0_tube", outer_radius=0.027, inner_radius=0.020, z_min=-0.095, z_max=0.300),
        material=dark_metal,
        name="stage_0_tube",
    )
    stage_0.visual(
        _tube_shell("stage_0_collar", outer_radius=0.033, inner_radius=0.022, z_min=0.268, z_max=0.306),
        material=matte_black,
        name="stage_0_collar",
    )
    stage_0.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.0, -0.046, 0.287), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_metal,
        name="stage_0_lock_screw",
    )
    stage_0.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.068, 0.287), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="stage_0_lock_knob",
    )

    stage_1 = model.part("post_stage_1")
    stage_1.visual(
        _tube_shell("stage_1_tube", outer_radius=0.020, inner_radius=0.014, z_min=-0.140, z_max=0.240),
        material=polished_metal,
        name="stage_1_tube",
    )
    stage_1.visual(
        _tube_shell("stage_1_collar", outer_radius=0.025, inner_radius=0.016, z_min=0.212, z_max=0.248),
        material=matte_black,
        name="stage_1_collar",
    )
    stage_1.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, -0.036, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_metal,
        name="stage_1_lock_screw",
    )
    stage_1.visual(
        Cylinder(radius=0.011, length=0.007),
        origin=Origin(xyz=(0.0, -0.054, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="stage_1_lock_knob",
    )

    stage_2 = model.part("post_stage_2")
    stage_2.visual(
        _tube_shell("stage_2_tube", outer_radius=0.014, inner_radius=0.010, z_min=-0.120, z_max=0.230),
        material=polished_metal,
        name="stage_2_tube",
    )
    stage_2.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=matte_black,
        name="top_cap",
    )
    # Shoulder yoke: two cheeks joined by a bridge, mounted to the top cap.
    stage_2.visual(
        Box((0.036, 0.092, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=matte_black,
        name="shoulder_bridge",
    )
    for side, y in enumerate((-0.0335, 0.0335)):
        stage_2.visual(
            Box((0.042, 0.012, 0.064)),
            origin=Origin(xyz=(0.0, y, 0.270)),
            material=matte_black,
            name=f"shoulder_cheek_{side}",
        )

    arm_0 = model.part("arm_0")
    arm_0_len = 0.350
    arm_0_rod_len = arm_0_len - 0.035
    for side, y in enumerate((-0.018, 0.018)):
        arm_0.visual(
            Cylinder(radius=0.0055, length=arm_0_rod_len),
            origin=Origin(xyz=(arm_0_rod_len / 2.0, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_metal,
            name=f"arm_0_rod_{side}",
        )
    arm_0.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="arm_0_shoulder_hub",
    )
    for side, y in enumerate((-0.0275, 0.0275)):
        arm_0.visual(
            Box((0.055, 0.010, 0.048)),
            origin=Origin(xyz=(arm_0_len, y, 0.0)),
            material=matte_black,
            name=f"arm_0_elbow_cheek_{side}",
        )
    arm_0.visual(
        Box((0.018, 0.074, 0.020)),
        origin=Origin(xyz=(arm_0_len - 0.032, 0.0, 0.0)),
        material=matte_black,
        name="arm_0_elbow_bridge",
    )

    arm_1 = model.part("arm_1")
    arm_1_len = 0.300
    arm_1.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="arm_1_elbow_hub",
    )
    arm_1_rod_len = arm_1_len - 0.035
    for side, y in enumerate((-0.015, 0.015)):
        arm_1.visual(
            Cylinder(radius=0.005, length=arm_1_rod_len),
            origin=Origin(xyz=(arm_1_rod_len / 2.0, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_metal,
            name=f"arm_1_rod_{side}",
        )
    for side, y in enumerate((-0.0275, 0.0275)):
        arm_1.visual(
            Box((0.050, 0.010, 0.044)),
            origin=Origin(xyz=(arm_1_len, y, 0.0)),
            material=matte_black,
            name=f"arm_1_shade_cheek_{side}",
        )
    arm_1.visual(
        Box((0.018, 0.068, 0.018)),
        origin=Origin(xyz=(arm_1_len - 0.030, 0.0, 0.0)),
        material=matte_black,
        name="arm_1_shade_bridge",
    )

    shade = model.part("shade")
    shade.visual(
        _shade_shell(),
        origin=Origin(xyz=(0.035, 0.0, -0.040)),
        material=shade_blue,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.020, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.045),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="shade_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.052, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="bulb_socket",
    )
    shade.visual(
        Sphere(radius=0.027),
        origin=Origin(xyz=(0.092, 0.0, -0.040)),
        material=warm_white,
        name="bulb",
    )

    model.articulation(
        "base_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_0,
        origin=Origin(xyz=(post_x, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.090),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.16, lower=0.0, upper=0.100),
    )
    model.articulation(
        "stage_2_to_arm_0",
        ArticulationType.REVOLUTE,
        parent=stage_2,
        child=arm_0,
        origin=Origin(xyz=(0.0, 0.0, 0.270), rpy=(0.0, -0.35, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.75, upper=0.95),
    )
    model.articulation(
        "arm_0_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=arm_1,
        origin=Origin(xyz=(arm_0_len, 0.0, 0.0), rpy=(0.0, 0.72, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "arm_1_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=shade,
        origin=Origin(xyz=(arm_1_len, 0.0, 0.0), rpy=(0.0, -0.15, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=-0.75, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stage_0 = object_model.get_part("post_stage_0")
    stage_1 = object_model.get_part("post_stage_1")
    stage_2 = object_model.get_part("post_stage_2")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    shade = object_model.get_part("shade")

    j0 = object_model.get_articulation("base_to_stage_0")
    j1 = object_model.get_articulation("stage_0_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")
    shoulder = object_model.get_articulation("stage_2_to_arm_0")
    elbow = object_model.get_articulation("arm_0_to_arm_1")
    shade_tilt = object_model.get_articulation("arm_1_to_shade")

    ctx.allow_overlap(
        base,
        stage_0,
        elem_a="base_socket",
        elem_b="stage_0_tube",
        reason=(
            "The first telescoping tube is intentionally represented as sliding inside the "
            "hollow base socket; exact collision treats the socket shell as a proxy solid."
        ),
    )
    ctx.allow_overlap(
        stage_0,
        stage_1,
        elem_a="stage_0_tube",
        elem_b="stage_1_tube",
        reason=(
            "The second telescoping tube is intentionally nested inside the first stage with "
            "sliding bushing contact; the shell collision proxy reports their retained insertion."
        ),
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_1_tube",
        elem_b="stage_2_tube",
        reason=(
            "The third telescoping tube is intentionally nested inside the second stage with "
            "sliding bushing contact; the shell collision proxy reports their retained insertion."
        ),
    )

    for joint in (j0, j1, j2):
        ctx.check(
            f"{joint.name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"{joint.name} type={joint.articulation_type}",
        )
    for joint in (shoulder, elbow, shade_tilt):
        ctx.check(
            f"{joint.name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.expect_within(stage_0, base, axes="xy", inner_elem="stage_0_tube", outer_elem="base_socket", margin=0.004)
    ctx.expect_overlap(stage_0, base, axes="z", elem_a="stage_0_tube", elem_b="base_socket", min_overlap=0.090)
    ctx.expect_within(stage_1, stage_0, axes="xy", inner_elem="stage_1_tube", outer_elem="stage_0_tube", margin=0.0)
    ctx.expect_overlap(stage_1, stage_0, axes="z", elem_a="stage_1_tube", elem_b="stage_0_tube", min_overlap=0.10)
    ctx.expect_within(stage_2, stage_1, axes="xy", inner_elem="stage_2_tube", outer_elem="stage_1_tube", margin=0.0)
    ctx.expect_overlap(stage_2, stage_1, axes="z", elem_a="stage_2_tube", elem_b="stage_1_tube", min_overlap=0.10)

    rest_upper_pos = ctx.part_world_position(stage_2)
    with ctx.pose({j0: 0.090, j1: 0.100, j2: 0.100}):
        ctx.expect_overlap(stage_0, base, axes="z", elem_a="stage_0_tube", elem_b="base_socket", min_overlap=0.004)
        ctx.expect_overlap(stage_1, stage_0, axes="z", elem_a="stage_1_tube", elem_b="stage_0_tube", min_overlap=0.045)
        ctx.expect_overlap(stage_2, stage_1, axes="z", elem_a="stage_2_tube", elem_b="stage_1_tube", min_overlap=0.035)
        extended_upper_pos = ctx.part_world_position(stage_2)

    ctx.check(
        "three post stages extend upward together",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.27,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    shade_rest = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.55, elbow: -0.45, shade_tilt: 0.55}):
        shade_moved = ctx.part_world_position(shade)
    ctx.check(
        "articulated arm and shade move in the vertical plane",
        shade_rest is not None
        and shade_moved is not None
        and abs(shade_moved[2] - shade_rest[2]) > 0.050,
        details=f"rest={shade_rest}, moved={shade_moved}",
    )

    return ctx.report()


object_model = build_object_model()
