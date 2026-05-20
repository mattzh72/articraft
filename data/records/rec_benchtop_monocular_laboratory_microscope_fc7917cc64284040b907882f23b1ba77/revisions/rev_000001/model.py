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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _xy_section(
    width: float,
    depth: float,
    corner_radius: float,
    z: float,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    cx, cy = center_xy
    return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(width, depth, corner_radius)]


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_microscope")

    cast_gray = model.material("cast_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.17, 0.18, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.87, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")

    base_geom = section_loft(
        [
            _xy_section(0.176, 0.136, 0.028, 0.0, center_xy=(0.0, 0.0)),
            _xy_section(0.166, 0.126, 0.026, 0.010, center_xy=(0.0, -0.002)),
            _xy_section(0.130, 0.094, 0.020, 0.024, center_xy=(0.0, -0.012)),
        ]
    )
    body.visual(
        mesh_from_geometry(base_geom, "microscope_base_shell"),
        material=cast_gray,
        name="base_shell",
    )

    arm_geom = section_loft(
        [
            _xy_section(0.064, 0.052, 0.016, 0.024, center_xy=(0.0, -0.020)),
            _xy_section(0.052, 0.044, 0.014, 0.105, center_xy=(0.0, -0.018)),
            _xy_section(0.041, 0.032, 0.012, 0.180, center_xy=(0.0, -0.016)),
            _xy_section(0.035, 0.026, 0.010, 0.235, center_xy=(0.0, -0.010)),
            _xy_section(0.034, 0.038, 0.010, 0.255, center_xy=(0.0, 0.002)),
        ]
    )
    body.visual(
        mesh_from_geometry(arm_geom, "microscope_arm_shell"),
        material=cast_gray,
        name="arm_shell",
    )

    body.visual(
        Box((0.012, 0.040, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=cast_gray,
        name="stage_post",
    )
    body.visual(
        Box((0.022, 0.082, 0.012)),
        origin=Origin(xyz=(0.0, 0.055, 0.138)),
        material=dark_gray,
        name="stage_guide",
    )
    body.visual(
        Box((0.026, 0.048, 0.026)),
        origin=Origin(xyz=(0.0, 0.030, 0.246)),
        material=cast_gray,
        name="head_bridge",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.055, 0.246)),
        material=dark_gray,
        name="head_socket",
    )

    eyepiece_angle = math.radians(36.0)
    eyepiece_axis = (0.0, -math.sin(eyepiece_angle), math.cos(eyepiece_angle))
    eyepiece_base = (0.0, 0.055, 0.271)
    eyepiece_half = 0.095 * 0.5
    eyepiece_center = (
        eyepiece_base[0] + eyepiece_axis[0] * eyepiece_half,
        eyepiece_base[1] + eyepiece_axis[1] * eyepiece_half,
        eyepiece_base[2] + eyepiece_axis[2] * eyepiece_half,
    )
    eyecup_center = (
        eyepiece_base[0] + eyepiece_axis[0] * (0.095 + 0.015),
        eyepiece_base[1] + eyepiece_axis[1] * (0.095 + 0.015),
        eyepiece_base[2] + eyepiece_axis[2] * (0.095 + 0.015),
    )
    body.visual(
        Cylinder(radius=0.012, length=0.095),
        origin=Origin(xyz=eyepiece_center, rpy=(eyepiece_angle, 0.0, 0.0)),
        material=chrome,
        name="eyepiece_tube",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=eyecup_center, rpy=(eyepiece_angle, 0.0, 0.0)),
        material=rubber_black,
        name="eyecup",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.024, -0.018, 0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="focus_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.180, 0.150, 0.335)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.004, 0.168)),
    )

    stage = model.part("stage")
    stage_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.046, segments=48),
            [_circle_profile(0.010, segments=24)],
            height=0.006,
            center=True,
        ),
        "microscope_stage_plate",
    )
    stage.visual(
        stage_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_gray,
        name="stage_plate",
    )
    stage.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_gray,
        name="stage_pedestal",
    )
    stage.visual(
        Box((0.044, 0.056, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_gray,
        name="carriage_bridge",
    )
    stage.visual(
        Box((0.010, 0.054, 0.022)),
        origin=Origin(xyz=(0.016, 0.0, 0.000)),
        material=cast_gray,
        name="carriage_cheek_right",
    )
    stage.visual(
        Box((0.010, 0.054, 0.022)),
        origin=Origin(xyz=(-0.016, 0.0, 0.000)),
        material=cast_gray,
        name="carriage_cheek_left",
    )
    stage.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.050)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_gray,
        name="turret_disk",
    )
    turret.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_gray,
        name="turret_hub",
    )
    objective_specs = [
        ("objective_long", 0.016, 0.028, 0.0065, 0.0),
        ("objective_mid", 0.016, 0.023, 0.0060, 2.0 * math.pi / 3.0),
        ("objective_short", 0.016, 0.020, 0.0055, -2.0 * math.pi / 3.0),
    ]
    for name, radius_offset, length, radius, angle in objective_specs:
        turret.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(
                    radius_offset * math.cos(angle),
                    radius_offset * math.sin(angle),
                    -0.012 - length * 0.5,
                )
            ),
            material=chrome,
            name=name,
        )
    turret.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.048),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="focus_shaft",
    )
    focus_knob.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="focus_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="fine_focus_cap",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.022),
        mass=0.12,
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_stage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage,
        origin=Origin(xyz=(0.0, 0.055, 0.138)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=-0.028,
            upper=0.028,
        ),
    )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turret,
        origin=Origin(xyz=(0.0, 0.055, 0.221)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "body_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_knob,
        origin=Origin(xyz=(0.033, -0.018, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-1.6,
            upper=1.6,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    stage = object_model.get_part("stage")
    turret = object_model.get_part("turret")
    focus_knob = object_model.get_part("focus_knob")

    stage_slide = object_model.get_articulation("body_to_stage")
    turret_spin = object_model.get_articulation("body_to_turret")
    focus_joint = object_model.get_articulation("body_to_focus_knob")

    ctx.check(
        "stage slide axis is fore-aft",
        tuple(stage_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={stage_slide.axis}",
    )
    ctx.check(
        "turret spins on vertical optical axis",
        tuple(turret_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={turret_spin.axis}",
    )
    ctx.check(
        "focus knob rotates on horizontal axis",
        tuple(focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={focus_joint.axis}",
    )

    with ctx.pose({stage_slide: 0.0}):
        ctx.expect_contact(
            stage,
            body,
            elem_a="carriage_cheek_right",
            elem_b="stage_guide",
            contact_tol=1e-6,
            name="stage carriage bears on the central guide",
        )
        ctx.expect_within(
            body,
            stage,
            axes="x",
            inner_elem="stage_guide",
            outer_elem="carriage_bridge",
            margin=0.0,
            name="guide stays centered beneath the carriage",
        )
        ctx.expect_contact(
            focus_knob,
            body,
            elem_b="focus_boss",
            contact_tol=1e-5,
            name="focus knob seats against the arm boss",
        )

    rest_stage_pos = ctx.part_world_position(stage)
    rest_obj_aabb = ctx.part_element_world_aabb(turret, elem="objective_long")

    upper_stage = stage_slide.motion_limits.upper if stage_slide.motion_limits else None
    if upper_stage is not None:
        with ctx.pose({stage_slide: upper_stage}):
            ctx.expect_overlap(
                body,
                stage,
                axes="y",
                elem_a="stage_guide",
                elem_b="carriage_bridge",
                min_overlap=0.030,
                name="stage carriage retains guide engagement at full travel",
            )
            upper_stage_pos = ctx.part_world_position(stage)
        ctx.check(
            "stage advances forward at positive travel",
            rest_stage_pos is not None
            and upper_stage_pos is not None
            and upper_stage_pos[1] > rest_stage_pos[1] + 0.020,
            details=f"rest={rest_stage_pos}, upper={upper_stage_pos}",
        )

    with ctx.pose({turret_spin: math.pi / 2.0}):
        spun_obj_aabb = ctx.part_element_world_aabb(turret, elem="objective_long")

    def _center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    ctx.check(
        "turret rotation moves an objective around the optical axis",
        rest_obj_aabb is not None
        and spun_obj_aabb is not None
        and (
            abs(_center_x(rest_obj_aabb) - _center_x(spun_obj_aabb)) > 0.008
            or abs(_center_y(rest_obj_aabb) - _center_y(spun_obj_aabb)) > 0.008
        ),
        details=f"rest={rest_obj_aabb}, spun={spun_obj_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
