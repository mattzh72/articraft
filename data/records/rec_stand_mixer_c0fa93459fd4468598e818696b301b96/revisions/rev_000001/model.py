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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _section_at_x(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _build_base_body_mesh():
    return section_loft(
        [
            _section_at_x(-0.135, width=0.102, height=0.172, radius=0.028, z_center=0.096),
            _section_at_x(-0.090, width=0.116, height=0.228, radius=0.034, z_center=0.142),
            _section_at_x(-0.045, width=0.138, height=0.246, radius=0.040, z_center=0.162),
            _section_at_x(0.005, width=0.154, height=0.148, radius=0.042, z_center=0.094),
            _section_at_x(0.050, width=0.170, height=0.074, radius=0.032, z_center=0.046),
            _section_at_x(0.082, width=0.136, height=0.034, radius=0.015, z_center=0.020),
        ]
    )


def _build_head_shell_mesh():
    return section_loft(
        [
            _section_at_x(0.000, width=0.112, height=0.118, radius=0.038, z_center=0.060),
            _section_at_x(0.070, width=0.152, height=0.164, radius=0.058, z_center=0.075),
            _section_at_x(0.165, width=0.186, height=0.170, radius=0.064, z_center=0.068),
            _section_at_x(0.250, width=0.158, height=0.132, radius=0.052, z_center=0.045),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_stand_mixer")

    enamel = model.material("enamel_cream", rgba=(0.93, 0.88, 0.78, 1.0))
    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.310, 0.205, 0.046), 0.026),
            "mixer_foot_plinth",
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=enamel,
        name="foot_plinth",
    )
    base.visual(
        mesh_from_geometry(_build_base_body_mesh(), "mixer_base_body"),
        material=enamel,
        name="base_body",
    )
    base.visual(
        Box((0.062, 0.014, 0.080)),
        origin=Origin(xyz=(0.112, 0.032, 0.066)),
        material=enamel,
        name="left_slide_cheek",
    )
    base.visual(
        Box((0.062, 0.014, 0.080)),
        origin=Origin(xyz=(0.112, -0.032, 0.066)),
        material=enamel,
        name="right_slide_cheek",
    )
    base.visual(
        Box((0.080, 0.094, 0.008)),
        origin=Origin(xyz=(0.096, 0.000, 0.022)),
        material=enamel,
        name="slide_bridge",
    )
    base.visual(
        Box((0.028, 0.022, 0.056)),
        origin=Origin(xyz=(-0.008, -0.068, 0.218)),
        material=enamel,
        name="speed_control_pod",
    )
    base.visual(
        Box((0.024, 0.020, 0.036)),
        origin=Origin(xyz=(-0.098, -0.060, 0.266)),
        material=enamel,
        name="lock_guide_pad",
    )
    base.visual(
        Box((0.050, 0.118, 0.020)),
        origin=Origin(xyz=(-0.080, 0.000, 0.284)),
        material=enamel,
        name="hinge_saddle",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(
            xyz=(-0.008, -0.075, 0.220),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=dark_trim,
        name="speed_knob_bezel",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(
            xyz=(-0.098, -0.061, 0.266),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=dark_trim,
        name="lock_guide_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.210, 0.310)),
        mass=8.0,
        origin=Origin(xyz=(0.000, 0.000, 0.155)),
    )

    bowl_slide = model.part("bowl_slide")
    bowl_slide.visual(
        Box((0.032, 0.050, 0.082)),
        origin=Origin(xyz=(0.000, 0.000, 0.043)),
        material=enamel,
        name="slide_column",
    )
    bowl_slide.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.010, 0.000, 0.088)),
        material=steel,
        name="bowl_platform",
    )
    bowl_slide.visual(
        Box((0.042, 0.052, 0.018)),
        origin=Origin(xyz=(-0.008, 0.000, 0.078)),
        material=enamel,
        name="platform_brace",
    )
    bowl_slide.inertial = Inertial.from_geometry(
        Box((0.150, 0.100, 0.105)),
        mass=1.3,
        origin=Origin(xyz=(0.005, 0.000, 0.055)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.025, 0.000),
                    (0.082, 0.014),
                    (0.108, 0.056),
                    (0.114, 0.140),
                    (0.120, 0.156),
                ],
                [
                    (0.000, 0.004),
                    (0.034, 0.010),
                    (0.085, 0.020),
                    (0.103, 0.060),
                    (0.109, 0.149),
                ],
                segments=64,
                end_cap="round",
                lip_samples=8,
            ),
            "mixing_bowl_shell",
        ),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=stainless,
        name="bowl_foot",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.156),
        mass=1.0,
        origin=Origin(xyz=(0.000, 0.000, 0.078)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_shell_mesh(), "mixer_head_shell"),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.254, 0.000, 0.016), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=steel,
        name="front_nose_band",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.195, 0.000, 0.000)),
        material=steel,
        name="planetary_socket",
    )
    head.visual(
        Box((0.074, 0.052, 0.028)),
        origin=Origin(xyz=(0.176, 0.000, 0.020)),
        material=enamel,
        name="planetary_fairing",
    )
    head.visual(
        Box((0.100, 0.040, 0.050)),
        origin=Origin(xyz=(0.145, 0.000, 0.045)),
        material=enamel,
        name="planetary_bridge",
    )
    head.visual(
        Box((0.060, 0.012, 0.050)),
        origin=Origin(xyz=(0.166, 0.028, 0.010)),
        material=enamel,
        name="right_planetary_strut",
    )
    head.visual(
        Box((0.060, 0.012, 0.050)),
        origin=Origin(xyz=(0.166, -0.028, 0.010)),
        material=enamel,
        name="left_planetary_strut",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.116),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=enamel,
        name="hinge_barrel",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.285, 0.190, 0.180)),
        mass=4.7,
        origin=Origin(xyz=(0.142, 0.000, 0.028)),
    )

    beater_shaft = model.part("beater_shaft")
    beater_shaft.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.014)),
        material=steel,
        name="hub",
    )
    beater_shaft.visual(
        Cylinder(radius=0.006, length=0.108),
        origin=Origin(xyz=(0.000, 0.000, -0.064)),
        material=steel,
        name="shaft",
    )
    beater_shaft.visual(
        Box((0.010, 0.006, 0.118)),
        origin=Origin(xyz=(0.000, 0.000, -0.113)),
        material=steel,
        name="center_blade",
    )
    beater_shaft.visual(
        Box((0.056, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.092)),
        material=steel,
        name="upper_crossbar",
    )
    beater_shaft.visual(
        Box((0.010, 0.006, 0.078)),
        origin=Origin(xyz=(-0.022, 0.000, -0.131)),
        material=steel,
        name="left_blade",
    )
    beater_shaft.visual(
        Box((0.010, 0.006, 0.078)),
        origin=Origin(xyz=(0.022, 0.000, -0.131)),
        material=steel,
        name="right_blade",
    )
    beater_shaft.visual(
        Box((0.048, 0.006, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, -0.166)),
        material=steel,
        name="lower_crossbar",
    )
    beater_shaft.inertial = Inertial.from_geometry(
        Box((0.080, 0.020, 0.200)),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, -0.100)),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(
            xyz=(0.000, -0.014, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=dark_trim,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.006, 0.004, 0.016)),
        origin=Origin(xyz=(0.012, -0.028, 0.010)),
        material=steel,
        name="indicator_fin",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.034, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(0.000, -0.017, 0.000)),
    )

    head_lock_plunger = model.part("head_lock_plunger")
    head_lock_plunger.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(0.000, -0.009, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=steel,
        name="plunger_stem",
    )
    head_lock_plunger.visual(
        Cylinder(radius=0.010, length=0.007),
        origin=Origin(
            xyz=(0.000, -0.0215, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=dark_trim,
        name="plunger_cap",
    )
    head_lock_plunger.inertial = Inertial.from_geometry(
        Box((0.022, 0.032, 0.022)),
        mass=0.05,
        origin=Origin(xyz=(0.000, -0.016, 0.000)),
    )

    model.articulation(
        "base_to_bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_slide,
        origin=Origin(xyz=(0.112, 0.000, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.06, lower=0.0, upper=0.026),
    )
    model.articulation(
        "bowl_slide_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_slide,
        child=bowl,
        origin=Origin(xyz=(0.010, 0.000, 0.093)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.080, 0.000, 0.322)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_beater_shaft",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater_shaft,
        origin=Origin(xyz=(0.195, 0.000, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=14.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.008, -0.080, 0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.3,
        ),
    )
    model.articulation(
        "base_to_head_lock_plunger",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock_plunger,
        origin=Origin(xyz=(-0.098, -0.069, 0.266)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.04,
            lower=0.0,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    bowl_slide = object_model.get_part("bowl_slide")

    bowl_lift = object_model.get_articulation("base_to_bowl_slide")
    head_tilt = object_model.get_articulation("base_to_head")

    ctx.expect_origin_gap(
        bowl,
        base,
        axis="x",
        min_gap=0.085,
        name="bowl sits ahead of the mixer base origin",
    )
    ctx.expect_gap(
        bowl,
        bowl_slide,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="bowl rests directly on the support platform",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.010,
        name="closed head clears the bowl rim",
    )

    rest_slide_pos = ctx.part_world_position(bowl_slide)
    with ctx.pose({bowl_lift: bowl_lift.motion_limits.upper}):
        raised_slide_pos = ctx.part_world_position(bowl_slide)
    ctx.check(
        "bowl support raises vertically",
        rest_slide_pos is not None
        and raised_slide_pos is not None
        and raised_slide_pos[2] > rest_slide_pos[2] + 0.020,
        details=f"rest={rest_slide_pos}, raised={raised_slide_pos}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(head, elem="front_nose_band")
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        open_head_aabb = ctx.part_element_world_aabb(head, elem="front_nose_band")
    ctx.check(
        "tilt head lifts upward at the front",
        rest_head_aabb is not None
        and open_head_aabb is not None
        and open_head_aabb[1][2] > rest_head_aabb[1][2] + 0.080,
        details=f"rest={rest_head_aabb}, open={open_head_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
