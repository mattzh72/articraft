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
    tube_from_spline_points,
)


def _xy_section(
    z: float,
    width: float,
    depth: float,
    radius: float,
    *,
    x_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_shift) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.72, 0.12, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.84, 0.86, 0.88, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))

    mixer_body = model.part("mixer_body")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.37, 0.23, 0.055), 0.045),
        "base_plate",
    )
    mixer_body.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.08, 0.0, 0.0225)),
        material=body_paint,
        name="base_plate",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.0, 0.13, 0.15, 0.040, x_shift=0.030),
                _xy_section(0.17, 0.11, 0.12, 0.034, x_shift=0.015),
                _xy_section(0.29, 0.09, 0.10, 0.028, x_shift=0.010),
            ]
        ),
        "pedestal_shell",
    )
    mixer_body.visual(
        pedestal_mesh,
        origin=Origin(xyz=(-0.145, 0.0, 0.045)),
        material=body_paint,
        name="pedestal_shell",
    )
    mixer_body.visual(
        Cylinder(radius=0.118, length=0.018),
        origin=Origin(xyz=(0.125, 0.0, 0.054)),
        material=dark_trim,
        name="bowl_turntable",
    )
    mixer_body.visual(
        Box((0.050, 0.100, 0.040)),
        origin=Origin(xyz=(-0.080, 0.0, 0.317)),
        material=body_paint,
        name="pivot_mount",
    )
    mixer_body.inertial = Inertial.from_geometry(
        Box((0.39, 0.23, 0.35)),
        mass=8.2,
        origin=Origin(xyz=(0.04, 0.0, 0.175)),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.028, 0.0),
            (0.055, 0.010),
            (0.096, 0.045),
            (0.118, 0.118),
            (0.128, 0.168),
            (0.132, 0.176),
        ],
        [
            (0.0, 0.004),
            (0.042, 0.012),
            (0.089, 0.050),
            (0.109, 0.120),
            (0.118, 0.168),
        ],
        segments=56,
        end_cap="round",
        lip_samples=8,
    )
    bowl_handle = tube_from_spline_points(
        [
            (0.0, 0.112, 0.070),
            (0.0, 0.155, 0.090),
            (0.0, 0.173, 0.123),
            (0.0, 0.162, 0.158),
            (0.0, 0.118, 0.170),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    bowl_shell.merge(bowl_handle)
    bowl_mesh = mesh_from_geometry(bowl_shell, "mixing_bowl")
    bowl.visual(bowl_mesh, material=stainless, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.132, length=0.176),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    head = model.part("head")
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.0, 0.086, 0.110, 0.028),
                _yz_section(0.10, 0.135, 0.170, 0.046),
                _yz_section(0.23, 0.145, 0.168, 0.046, z_shift=-0.004),
                _yz_section(0.35, 0.102, 0.136, 0.034, z_shift=-0.012),
            ]
        ),
        "head_shell",
    )
    head.visual(head_mesh, material=body_paint, name="head_shell")
    head.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.190, 0.0, -0.025)),
        material=dark_trim,
        name="hub_nose",
    )
    head.visual(
        Box((0.180, 0.110, 0.120)),
        origin=Origin(xyz=(0.185, 0.0, 0.005)),
        material=body_paint,
        name="hub_mount_web",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.36, 0.15, 0.18)),
        mass=4.0,
        origin=Origin(xyz=(0.18, 0.0, -0.004)),
    )

    beater_hub = model.part("beater_hub")
    beater_hub.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_trim,
        name="hub_cap",
    )
    beater_hub.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=stainless,
        name="beater_shaft",
    )
    beater_hub.visual(
        Box((0.038, 0.012, 0.012)),
        origin=Origin(xyz=(0.019, 0.0, -0.104)),
        material=stainless,
        name="beater_upper_bridge",
    )
    beater_hub.visual(
        Box((0.012, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=stainless,
        name="beater_inner_leg",
    )
    beater_hub.visual(
        Box((0.012, 0.012, 0.070)),
        origin=Origin(xyz=(0.038, 0.0, -0.132)),
        material=stainless,
        name="beater_outer_leg",
    )
    beater_hub.visual(
        Box((0.056, 0.012, 0.014)),
        origin=Origin(xyz=(0.019, 0.0, -0.160)),
        material=stainless,
        name="beater_lower_bridge",
    )
    beater_hub.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.174)),
        mass=0.35,
        origin=Origin(xyz=(0.019, 0.0, -0.087)),
    )

    model.articulation(
        "body_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=mixer_body,
        child=head,
        origin=Origin(xyz=(-0.055, 0.0, 0.334)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "body_to_bowl_lock",
        ArticulationType.REVOLUTE,
        parent=mixer_body,
        child=bowl,
        origin=Origin(xyz=(0.125, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "head_to_beater_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater_hub,
        origin=Origin(xyz=(0.180, 0.0, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=18.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mixer_body = object_model.get_part("mixer_body")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater_hub = object_model.get_part("beater_hub")

    head_tilt = object_model.get_articulation("body_to_head_tilt")
    bowl_lock = object_model.get_articulation("body_to_bowl_lock")
    beater_spin = object_model.get_articulation("head_to_beater_spin")

    ctx.expect_gap(
        bowl,
        mixer_body,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_turntable",
        max_gap=0.003,
        max_penetration=0.0,
        name="bowl sits on the lock turntable",
    )
    ctx.expect_overlap(
        bowl,
        mixer_body,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_turntable",
        min_overlap=0.18,
        name="bowl remains centered over the base turntable",
    )
    ctx.expect_contact(
        head,
        mixer_body,
        elem_a="head_shell",
        elem_b="pivot_mount",
        contact_tol=0.001,
        name="head is supported by the rear pivot mount",
    )
    ctx.expect_within(
        beater_hub,
        bowl,
        axes="xy",
        margin=0.012,
        name="beater stays within the bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="hub_nose",
        negative_elem="bowl_shell",
        min_gap=0.012,
        name="closed head clears the bowl rim",
    )

    rest_beater_pos = ctx.part_world_position(beater_hub)
    with ctx.pose({head_tilt: math.radians(58.0)}):
        raised_beater_pos = ctx.part_world_position(beater_hub)
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            positive_elem="hub_nose",
            negative_elem="bowl_shell",
            min_gap=0.18,
            name="tilted head lifts well clear of the bowl",
        )
    ctx.check(
        "tilt head raises the beater",
        rest_beater_pos is not None
        and raised_beater_pos is not None
        and raised_beater_pos[2] > rest_beater_pos[2] + 0.12,
        details=f"rest={rest_beater_pos}, raised={raised_beater_pos}",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_lock: math.radians(28.0)}):
        twisted_bowl_pos = ctx.part_world_position(bowl)
        ctx.expect_gap(
            bowl,
            mixer_body,
            axis="z",
            positive_elem="bowl_shell",
            negative_elem="bowl_turntable",
            max_gap=0.003,
            max_penetration=0.0,
            name="twist-locked bowl stays seated on the base",
        )
    ctx.check(
        "twist-lock rotates in place",
        rest_bowl_pos is not None
        and twisted_bowl_pos is not None
        and abs(rest_bowl_pos[0] - twisted_bowl_pos[0]) < 1e-6
        and abs(rest_bowl_pos[1] - twisted_bowl_pos[1]) < 1e-6
        and abs(rest_bowl_pos[2] - twisted_bowl_pos[2]) < 1e-6,
        details=f"rest={rest_bowl_pos}, twisted={twisted_bowl_pos}",
    )

    rest_spin_pos = ctx.part_world_position(beater_hub)
    with ctx.pose({beater_spin: 1.7}):
        spun_pos = ctx.part_world_position(beater_hub)
    ctx.check(
        "beater hub spins about a fixed axis",
        rest_spin_pos is not None
        and spun_pos is not None
        and abs(rest_spin_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_spin_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_spin_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_spin_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
