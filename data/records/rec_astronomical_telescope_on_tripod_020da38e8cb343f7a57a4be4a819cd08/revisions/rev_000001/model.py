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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


POLAR_TILT = math.radians(50.0)


def _beam_mesh(
    points: list[tuple[float, float, float]],
    *,
    width: float,
    height: float,
    name: str,
):
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.18,
        corner_segments=5,
    )
    return mesh_from_geometry(
        sweep_profile_along_spline(
            points,
            profile=profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classical_refractor_on_equatorial_mount")

    varnished_wood = model.material("varnished_wood", rgba=(0.55, 0.38, 0.19, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.37, 0.23, 0.11, 1.0))
    ivory_paint = model.material("ivory_paint", rgba=(0.91, 0.89, 0.82, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.72, 0.75, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.64, 0.29, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.10, 0.10, 0.11, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.105, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=dark_wood,
        name="head_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.048, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=dark_wood,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.062, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=dark_wood,
        name="tray_hub",
    )
    tripod_base.visual(
        Box((0.24, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=graphite,
        name="equatorial_head_base",
    )

    polar_support = _beam_mesh(
        [
            (0.00, 0.0, 1.18),
            (0.06, 0.0, 1.23),
            (0.10, 0.0, 1.24),
        ],
        width=0.15,
        height=0.12,
        name="polar_support_brace_mesh",
    )
    tripod_base.visual(
        polar_support,
        material=graphite,
        name="polar_support_brace",
    )
    tripod_base.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(
            xyz=(
                0.14 - 0.03 * math.sin(POLAR_TILT),
                0.0,
                1.30 - 0.03 * math.cos(POLAR_TILT),
            ),
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=blackened_steel,
        name="polar_support_beam",
    )
    tripod_base.visual(
        Box((0.08, 0.18, 0.22)),
        origin=Origin(xyz=(-0.02, 0.0, 1.20)),
        material=graphite,
        name="latitude_brace",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg = _beam_mesh(
            [
                (0.085 * c, 0.085 * s, 1.05),
                (0.27 * c, 0.27 * s, 0.62),
                (0.76 * c, 0.76 * s, 0.04),
            ],
            width=0.056,
            height=0.030,
            name=f"tripod_leg_mesh_{index}",
        )
        tripod_base.visual(
            leg,
            material=varnished_wood,
            name=f"tripod_leg_{index}",
        )
        spreader = _beam_mesh(
            [
                (0.035 * c, 0.035 * s, 0.56),
                (0.20 * c, 0.20 * s, 0.55),
                (0.39 * c, 0.39 * s, 0.49),
            ],
            width=0.026,
            height=0.014,
            name=f"spreader_arm_mesh_{index}",
        )
        tripod_base.visual(
            spreader,
            material=brass,
            name=f"spreader_arm_{index}",
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((1.65, 1.65, 1.32)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
    )

    ra_head = model.part("ra_head")
    ra_head.visual(
        Cylinder(radius=0.075, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=blackened_steel,
        name="lower_ra_collar",
    )
    ra_head.visual(
        Cylinder(radius=0.092, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=brass,
        name="ra_setting_circle",
    )
    ra_head.visual(
        Cylinder(radius=0.056, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=blackened_steel,
        name="polar_axis_housing",
    )
    ra_head.visual(
        Box((0.15, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=graphite,
        name="upper_ra_block",
    )
    ra_head.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=graphite,
        name="declination_neck",
    )
    ra_head.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.11, 0.64)),
        material=graphite,
        name="left_bearing_cheek",
    )
    ra_head.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.11, 0.64)),
        material=graphite,
        name="right_bearing_cheek",
    )
    ra_head.visual(
        Cylinder(radius=0.022, length=0.18),
        origin=Origin(xyz=(0.0, -0.12, 0.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="slow_motion_knob",
    )
    ra_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.32, 0.74)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    declination_assembly = model.part("declination_assembly")
    declination_assembly.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="declination_shaft",
    )
    declination_assembly.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=graphite,
        name="saddle_block",
    )
    declination_assembly.visual(
        Box((0.10, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=graphite,
        name="saddle_pier",
    )
    declination_assembly.visual(
        Box((0.76, 0.08, 0.02)),
        origin=Origin(xyz=(0.33, 0.0, 0.26)),
        material=brass,
        name="saddle_plate",
    )
    declination_assembly.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.20, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_tube_ring",
    )
    declination_assembly.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.58, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="front_tube_ring",
    )
    declination_assembly.visual(
        Cylinder(radius=0.075, length=1.30),
        origin=Origin(xyz=(0.63, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ivory_paint,
        name="main_tube",
    )
    declination_assembly.visual(
        Cylinder(radius=0.058, length=0.22),
        origin=Origin(xyz=(-0.13, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="focuser_body",
    )
    declination_assembly.visual(
        Cylinder(radius=0.034, length=0.15),
        origin=Origin(xyz=(-0.315, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drawtube",
    )
    declination_assembly.visual(
        Cylinder(radius=0.020, length=0.08),
        origin=Origin(xyz=(-0.43, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_steel,
        name="eyepiece",
    )
    declination_assembly.visual(
        Cylinder(radius=0.082, length=0.12),
        origin=Origin(xyz=(1.34, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="objective_cell",
    )
    declination_assembly.visual(
        Cylinder(radius=0.092, length=0.28),
        origin=Origin(xyz=(1.54, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ivory_paint,
        name="dew_shield",
    )
    declination_assembly.visual(
        Cylinder(radius=0.018, length=0.36),
        origin=Origin(xyz=(0.82, 0.0, 0.47), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_steel,
        name="finder_scope",
    )
    declination_assembly.visual(
        Box((0.020, 0.030, 0.040)),
        origin=Origin(xyz=(0.69, 0.0, 0.434)),
        material=brass,
        name="finder_bracket_rear",
    )
    declination_assembly.visual(
        Box((0.020, 0.030, 0.040)),
        origin=Origin(xyz=(0.95, 0.0, 0.434)),
        material=brass,
        name="finder_bracket_front",
    )
    declination_assembly.visual(
        Cylinder(radius=0.012, length=0.68),
        origin=Origin(xyz=(-0.34, 0.0, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="counterweight_shaft",
    )
    declination_assembly.visual(
        Cylinder(radius=0.080, length=0.07),
        origin=Origin(xyz=(-0.24, 0.0, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="counterweight_upper",
    )
    declination_assembly.visual(
        Cylinder(radius=0.080, length=0.07),
        origin=Origin(xyz=(-0.46, 0.0, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="counterweight_lower",
    )
    declination_assembly.inertial = Inertial.from_geometry(
        Box((2.15, 0.28, 1.02)),
        mass=11.0,
        origin=Origin(xyz=(0.60, 0.0, 0.18)),
    )

    model.articulation(
        "polar_axis",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=ra_head,
        origin=Origin(xyz=(0.14, 0.0, 1.30), rpy=(0.0, POLAR_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.6,
        ),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=ra_head,
        child=declination_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.64), rpy=(0.0, -POLAR_TILT, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.5,
            lower=-0.35,
            upper=0.85,
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
    tripod_base = object_model.get_part("tripod_base")
    ra_head = object_model.get_part("ra_head")
    declination_assembly = object_model.get_part("declination_assembly")
    polar_axis = object_model.get_articulation("polar_axis")
    declination_axis = object_model.get_articulation("declination_axis")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "all principal parts exist",
        tripod_base is not None and ra_head is not None and declination_assembly is not None,
        details="Tripod, right ascension head, and declination assembly must all be authored.",
    )
    ctx.check(
        "articulation types match equatorial mount",
        polar_axis.articulation_type == ArticulationType.CONTINUOUS
        and declination_axis.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"polar_axis={polar_axis.articulation_type}, "
            f"declination_axis={declination_axis.articulation_type}"
        ),
    )
    ctx.check(
        "polar axis is tilted toward the pole",
        abs(polar_axis.origin.rpy[1] - POLAR_TILT) < 1e-6
        and polar_axis.axis == (0.0, 0.0, 1.0),
        details=f"origin_rpy={polar_axis.origin.rpy}, axis={polar_axis.axis}",
    )
    ctx.check(
        "declination axis uses east-west trunnion direction",
        abs(abs(declination_axis.origin.rpy[1]) - POLAR_TILT) < 1e-6
        and declination_axis.axis == (0.0, -1.0, 0.0),
        details=f"origin_rpy={declination_axis.origin.rpy}, axis={declination_axis.axis}",
    )

    ctx.expect_contact(
        ra_head,
        tripod_base,
        elem_a="lower_ra_collar",
        elem_b="polar_support_beam",
        name="right ascension collar seats on polar support",
    )
    ctx.expect_contact(
        declination_assembly,
        ra_head,
        elem_a="declination_shaft",
        elem_b="left_bearing_cheek",
        name="declination shaft seats in left bearing cheek",
    )
    ctx.expect_contact(
        declination_assembly,
        ra_head,
        elem_a="declination_shaft",
        elem_b="right_bearing_cheek",
        name="declination shaft seats in right bearing cheek",
    )

    objective_rest = _aabb_center(
        ctx.part_element_world_aabb(declination_assembly, elem="objective_cell")
    )
    with ctx.pose({declination_axis: 0.60}):
        objective_raised = _aabb_center(
            ctx.part_element_world_aabb(declination_assembly, elem="objective_cell")
        )
    ctx.check(
        "declination motion raises the objective",
        objective_rest is not None
        and objective_raised is not None
        and objective_raised[2] > objective_rest[2] + 0.30,
        details=f"rest={objective_rest}, raised={objective_raised}",
    )

    with ctx.pose({polar_axis: math.pi / 2.0}):
        objective_rotated = _aabb_center(
            ctx.part_element_world_aabb(declination_assembly, elem="objective_cell")
        )
    ctx.check(
        "right ascension motion swings the tube around the polar axis",
        objective_rest is not None
        and objective_rotated is not None
        and abs(objective_rotated[1] - objective_rest[1]) > 0.50,
        details=f"rest={objective_rest}, rotated={objective_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
