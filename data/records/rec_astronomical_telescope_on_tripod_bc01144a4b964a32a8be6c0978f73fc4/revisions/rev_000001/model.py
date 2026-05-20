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
)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _shell_along_x(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
    segments: int = 56,
):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spotting_scope_alt_az")

    tripod_black = model.material("tripod_black", rgba=(0.12, 0.12, 0.13, 1.0))
    head_gray = model.material("head_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    scope_green = model.material("scope_green", rgba=(0.31, 0.36, 0.28, 1.0))
    anodized_dark = model.material("anodized_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.42, 0.52, 0.58, 0.45))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.693)),
        material=head_gray,
        name="apex_plate",
    )
    tripod.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=head_gray,
        name="leg_shoulder_collar",
    )
    tripod.visual(
        Cylinder(radius=0.024, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.649)),
        material=head_gray,
        name="center_hub",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        top = (0.028 * math.cos(angle), 0.028 * math.sin(angle), 0.605)
        knee = (0.170 * math.cos(angle), 0.170 * math.sin(angle), 0.340)
        foot_top = (0.300 * math.cos(angle), 0.300 * math.sin(angle), 0.022)

        upper_origin, upper_length = _segment_origin(top, knee)
        lower_origin, lower_length = _segment_origin(knee, foot_top)

        tripod.visual(
            Cylinder(radius=0.010, length=upper_length),
            origin=upper_origin,
            material=tripod_black,
            name=f"upper_leg_{index}",
        )
        tripod.visual(
            Cylinder(radius=0.008, length=lower_length),
            origin=lower_origin,
            material=tripod_black,
            name=f"lower_leg_{index}",
        )
        tripod.visual(
            Cylinder(radius=0.013, length=0.022),
            origin=Origin(xyz=(foot_top[0], foot_top[1], 0.011)),
            material=rubber_black,
            name=f"foot_{index}",
        )

    tripod.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 0.72)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.044, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=head_gray,
        name="bearing_base",
    )
    azimuth_head.visual(
        Cylinder(radius=0.020, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=head_gray,
        name="bearing_column",
    )
    azimuth_head.visual(
        Box((0.065, 0.140, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=head_gray,
        name="yoke_block",
    )
    azimuth_head.visual(
        Box((0.070, 0.012, 0.068)),
        origin=Origin(xyz=(0.010, 0.072, 0.156)),
        material=head_gray,
        name="left_yoke_arm",
    )
    azimuth_head.visual(
        Box((0.070, 0.012, 0.068)),
        origin=Origin(xyz=(0.010, -0.072, 0.156)),
        material=head_gray,
        name="right_yoke_arm",
    )

    pan_rod_origin, pan_rod_length = _segment_origin(
        (-0.010, -0.018, 0.114),
        (-0.220, -0.018, 0.072),
    )
    azimuth_head.visual(
        Cylinder(radius=0.005, length=pan_rod_length),
        origin=pan_rod_origin,
        material=anodized_dark,
        name="pan_handle_rod",
    )
    pan_grip_origin, pan_grip_length = _segment_origin(
        (-0.220, -0.018, 0.072),
        (-0.305, -0.018, 0.050),
    )
    azimuth_head.visual(
        Cylinder(radius=0.011, length=pan_grip_length),
        origin=pan_grip_origin,
        material=rubber_black,
        name="pan_handle_grip",
    )
    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.32, 0.16, 0.22)),
        mass=1.1,
        origin=Origin(xyz=(-0.08, 0.0, 0.11)),
    )

    objective_body = model.part("objective_body")
    objective_body.visual(
        _shell_along_x(
            outer_profile=[(0.036, -0.045), (0.036, 0.050), (0.035, 0.080)],
            inner_profile=[(0.030, -0.045), (0.030, 0.050), (0.029, 0.080)],
            name="focus_sleeve",
        ),
        material=scope_green,
        name="focus_sleeve_shell",
    )
    objective_body.visual(
        _shell_along_x(
            outer_profile=[
                (0.035, 0.065),
                (0.037, 0.100),
                (0.040, 0.135),
                (0.043, 0.165),
                (0.042, 0.180),
            ],
            inner_profile=[
                (0.029, 0.065),
                (0.031, 0.105),
                (0.034, 0.140),
                (0.037, 0.173),
            ],
            name="objective_bell",
        ),
        material=scope_green,
        name="objective_bell_shell",
    )
    objective_body.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="armor_band",
    )
    objective_body.visual(
        Cylinder(radius=0.037, length=0.002),
        origin=Origin(xyz=(0.173, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    objective_body.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=Origin(xyz=(0.008, 0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_dark,
        name="left_trunnion",
    )
    objective_body.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=Origin(xyz=(0.008, -0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_dark,
        name="right_trunnion",
    )
    objective_body.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.008, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=scope_green,
        name="left_trunnion_boss",
    )
    objective_body.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.008, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=scope_green,
        name="right_trunnion_boss",
    )
    objective_body.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    eyepiece_assembly = model.part("eyepiece_assembly")
    eyepiece_assembly.visual(
        Cylinder(radius=0.027, length=0.115),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_dark,
        name="drawtube",
    )
    eyepiece_assembly.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=scope_green,
        name="eyepiece_barrel",
    )
    eyepiece_assembly.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_dark,
        name="focus_collar",
    )
    eyepiece_assembly.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(-0.106, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_dark,
        name="ocular_housing",
    )
    eyepiece_assembly.visual(
        Cylinder(radius=0.029, length=0.026),
        origin=Origin(xyz=(-0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="eyecup",
    )
    eyepiece_assembly.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(-0.147, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="eyelens",
    )
    eyepiece_assembly.inertial = Inertial.from_geometry(
        Box((0.18, 0.07, 0.07)),
        mass=0.35,
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
    )

    model.articulation(
        "tripod_to_azimuth_head",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0),
    )
    model.articulation(
        "head_to_objective_body",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=objective_body,
        origin=Origin(xyz=(0.008, 0.0, 0.164)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=math.radians(-20.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "objective_body_to_eyepiece_assembly",
        ArticulationType.PRISMATIC,
        parent=objective_body,
        child=eyepiece_assembly,
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.10,
            lower=0.0,
            upper=0.035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    azimuth_head = object_model.get_part("azimuth_head")
    objective_body = object_model.get_part("objective_body")
    eyepiece_assembly = object_model.get_part("eyepiece_assembly")

    azimuth = object_model.get_articulation("tripod_to_azimuth_head")
    altitude = object_model.get_articulation("head_to_objective_body")
    focus = object_model.get_articulation("objective_body_to_eyepiece_assembly")

    ctx.expect_gap(
        azimuth_head,
        tripod,
        axis="z",
        positive_elem="bearing_base",
        negative_elem="apex_plate",
        max_gap=0.001,
        max_penetration=0.0005,
        name="azimuth bearing base seats on tripod apex plate",
    )
    ctx.expect_overlap(
        azimuth_head,
        tripod,
        axes="xy",
        elem_a="bearing_base",
        elem_b="apex_plate",
        min_overlap=0.080,
        name="azimuth bearing stays centered on tripod apex",
    )

    ctx.expect_contact(
        objective_body,
        azimuth_head,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        contact_tol=0.0005,
        name="left trunnion bears on left yoke arm",
    )
    ctx.expect_contact(
        objective_body,
        azimuth_head,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        contact_tol=0.0005,
        name="right trunnion bears on right yoke arm",
    )

    with ctx.pose({focus: 0.0}):
        ctx.expect_contact(
            eyepiece_assembly,
            objective_body,
            elem_a="focus_collar",
            elem_b="focus_sleeve_shell",
            contact_tol=0.0005,
            name="collapsed focus collar seats against the sleeve rim",
        )
        ctx.expect_within(
            eyepiece_assembly,
            objective_body,
            axes="yz",
            inner_elem="drawtube",
            outer_elem="focus_sleeve_shell",
            margin=0.0,
            name="collapsed drawtube stays centered in focus sleeve",
        )
        ctx.expect_overlap(
            eyepiece_assembly,
            objective_body,
            axes="x",
            elem_a="drawtube",
            elem_b="focus_sleeve_shell",
            min_overlap=0.080,
            name="collapsed drawtube remains deeply inserted",
        )

    focus_upper = focus.motion_limits.upper if focus.motion_limits is not None else 0.035
    eyecup_rest = ctx.part_element_world_aabb(eyepiece_assembly, elem="eyecup")
    with ctx.pose({focus: focus_upper}):
        ctx.expect_within(
            eyepiece_assembly,
            objective_body,
            axes="yz",
            inner_elem="drawtube",
            outer_elem="focus_sleeve_shell",
            margin=0.0,
            name="extended drawtube stays centered in focus sleeve",
        )
        ctx.expect_overlap(
            eyepiece_assembly,
            objective_body,
            axes="x",
            elem_a="drawtube",
            elem_b="focus_sleeve_shell",
            min_overlap=0.045,
            name="extended drawtube retains insertion",
        )
        eyecup_extended = ctx.part_element_world_aabb(eyepiece_assembly, elem="eyecup")
    ctx.check(
        "focus drawtube extends rearward under positive travel",
        eyecup_rest is not None
        and eyecup_extended is not None
        and 0.5 * (eyecup_extended[0][0] + eyecup_extended[1][0])
        < 0.5 * (eyecup_rest[0][0] + eyecup_rest[1][0]) - 0.020,
        details=f"rest={eyecup_rest}, extended={eyecup_extended}",
    )

    bell_rest = ctx.part_element_world_aabb(objective_body, elem="objective_bell_shell")
    altitude_upper = altitude.motion_limits.upper if altitude.motion_limits is not None else math.radians(65.0)
    with ctx.pose({altitude: altitude_upper}):
        bell_raised = ctx.part_element_world_aabb(objective_body, elem="objective_bell_shell")
    ctx.check(
        "positive altitude raises the front of the scope",
        bell_rest is not None
        and bell_raised is not None
        and 0.5 * (bell_raised[0][2] + bell_raised[1][2])
        > 0.5 * (bell_rest[0][2] + bell_rest[1][2]) + 0.040,
        details=f"rest={bell_rest}, raised={bell_raised}",
    )

    bell_forward = ctx.part_element_world_aabb(objective_body, elem="objective_bell_shell")
    with ctx.pose({azimuth: math.pi / 2.0}):
        bell_left = ctx.part_element_world_aabb(objective_body, elem="objective_bell_shell")
    ctx.check(
        "azimuth rotation swings the scope around the tripod",
        bell_forward is not None
        and bell_left is not None
        and 0.5 * (bell_forward[0][0] + bell_forward[1][0]) > 0.060
        and abs(0.5 * (bell_forward[0][1] + bell_forward[1][1])) < 0.030
        and 0.5 * (bell_left[0][1] + bell_left[1][1]) > 0.060,
        details=f"forward={bell_forward}, turned={bell_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
