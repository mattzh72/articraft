from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_x(radius: float, length: float) -> Cylinder:
    """Return an SDK cylinder descriptor; callers rotate it onto the syringe axis."""
    return Cylinder(radius=radius, length=length)


def _x_axis_shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
    """Build a lathed hollow shell and rotate its local Z axis onto world X."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _axis_origin(xyz: tuple[float, float, float]) -> Origin:
    """Origin for a cylinder whose local Z axis should lie on syringe +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_syringe")

    clear_pc = model.material("clear_polycarbonate", rgba=(0.55, 0.82, 1.0, 0.38))
    black_rubber = model.material("black_epdm_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    stainless = model.material("passivated_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    marking = model.material("etched_black_marking", rgba=(0.01, 0.01, 0.012, 1.0))
    white = model.material("white_plastic", rgba=(0.90, 0.92, 0.88, 1.0))

    barrel = model.part("barrel")

    # One continuous, hollow, transparent pressure barrel.  The rear bore necks
    # down to a rod seal, while the front necks down through a practical luer-
    # style nozzle tip.  Wider collars form drip-shedding overhangs.
    barrel_shell = _x_axis_shell_mesh(
        "barrel_shell",
        outer_profile=[
            (0.030, -0.128),
            (0.030, -0.116),
            (0.024, -0.108),
            (0.024, 0.078),
            (0.030, 0.086),
            (0.030, 0.096),
            (0.014, 0.119),
            (0.006, 0.158),
        ],
        inner_profile=[
            (0.0065, -0.128),
            (0.0065, -0.116),
            (0.0190, -0.108),
            (0.0190, 0.078),
            (0.0110, 0.096),
            (0.0050, 0.119),
            (0.0020, 0.158),
        ],
    )
    barrel.visual(barrel_shell, material=clear_pc, name="barrel_shell")

    # Weather seals and stop geometry are separate visible materials but are
    # mounted into the shell with a small seated intersection.
    barrel.visual(
        _x_axis_shell_mesh(
            "rear_stop_lip",
            outer_profile=[(0.033, -0.004), (0.033, 0.004)],
            inner_profile=[(0.0062, -0.004), (0.0062, 0.004)],
            segments=64,
        ),
        origin=Origin(xyz=(-0.123, 0.0, 0.0)),
        material=black_rubber,
        name="rear_stop_lip",
    )
    barrel.visual(
        _x_axis_shell_mesh(
            "front_drip_seal",
            outer_profile=[(0.033, -0.004), (0.033, 0.004)],
            inner_profile=[(0.0060, -0.004), (0.0060, 0.004)],
            segments=64,
        ),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=black_rubber,
        name="front_drip_seal",
    )
    barrel.visual(
        _x_axis_shell_mesh(
            "rod_wiper_seal",
            outer_profile=[(0.015, -0.008), (0.015, 0.008)],
            inner_profile=[(0.0048, -0.008), (0.0048, 0.008)],
            segments=56,
        ),
        origin=Origin(xyz=(-0.133, 0.0, 0.0)),
        material=black_rubber,
        name="rod_wiper_seal",
    )

    # Corrosion-resistant clamp bands with protected screw heads hold the clear
    # tube and overmolded seals together.
    for i, x in enumerate((-0.048, 0.045)):
        barrel.visual(
            _x_axis_shell_mesh(
                f"stainless_band_{i}",
                outer_profile=[(0.0256, -0.003), (0.0256, 0.003)],
                inner_profile=[(0.0237, -0.003), (0.0237, 0.003)],
                segments=64,
            ),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=stainless,
            name=f"stainless_band_{i}",
        )
        barrel.visual(
            Cylinder(radius=0.0043, length=0.0022),
            origin=Origin(xyz=(x, 0.0, 0.0263)),
            material=stainless,
            name=f"cap_screw_{i}",
        )

    # Finger tabs are separate pads bonded into the rear overmold.  They overlap
    # the rear collar so the barrel remains one hard, supported assembly.
    for side, y in (("near", -0.043), ("far", 0.043)):
        barrel.visual(
            Box((0.014, 0.034, 0.014)),
            origin=Origin(xyz=(-0.124, y, 0.0)),
            material=black_rubber,
            name=f"{side}_finger_tab",
        )
        barrel.visual(
            Box((0.022, 0.010, 0.010)),
            origin=Origin(xyz=(-0.113, y * 0.68, 0.0), rpy=(0.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"{side}_tab_web",
        )

    # Raised graduation marks sit slightly proud/embedded on the outer surface.
    for i, x in enumerate([-0.080, -0.066, -0.052, -0.038, -0.024, -0.010, 0.004, 0.018, 0.032, 0.046, 0.060]):
        is_major = i % 5 == 0
        barrel.visual(
            Box((0.0012, 0.027 if is_major else 0.017, 0.0016)),
            origin=Origin(xyz=(x, 0.0, 0.0237)),
            material=marking,
            name=f"graduation_{i}",
        )

    # Dark outlet hole at the end of the molded nozzle tip.
    barrel.visual(
        Cylinder(radius=0.0021, length=0.0010),
        origin=_axis_origin((0.1585, 0.0, 0.0)),
        material=marking,
        name="nozzle_orifice",
    )

    plunger = model.part("plunger")
    # The child frame sits on the rear seal plane; positive prismatic motion is
    # a push along syringe +X.  Hidden rod length keeps the piston captured.
    plunger.visual(
        _cylinder_x(radius=0.0050, length=0.174),
        origin=_axis_origin((-0.007, 0.0, 0.0)),
        material=stainless,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0172, length=0.022),
        origin=_axis_origin((0.072, 0.0, 0.0)),
        material=black_rubber,
        name="piston_body",
    )
    for i, x in enumerate((0.062, 0.082)):
        plunger.visual(
            Cylinder(radius=0.0183, length=0.0035),
            origin=_axis_origin((x, 0.0, 0.0)),
            material=black_rubber,
            name=f"piston_seal_{i}",
        )
    plunger.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=_axis_origin((-0.102, 0.0, 0.0)),
        material=white,
        name="thumb_hub",
    )
    plunger.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=_axis_origin((-0.090, 0.0, 0.0)),
        material=white,
        name="thumb_plate",
    )
    plunger.visual(
        Cylinder(radius=0.034, length=0.0035),
        origin=_axis_origin((-0.097, 0.0, 0.0)),
        material=black_rubber,
        name="thumb_drip_lip",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=0.18, lower=0.0, upper=0.081),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="rod_wiper_seal",
        elem_b="plunger_rod",
        reason="The EPDM wiper seal is intentionally compressed around the stainless plunger rod for weatherproof sliding contact.",
    )

    ctx.check(
        "plunger joint is coaxial and bounded",
        tuple(slide.axis) == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.07 < slide.motion_limits.upper < 0.10,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="piston_body",
        outer_elem="barrel_shell",
        margin=0.0,
        name="piston remains coaxial within the barrel envelope",
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="rod_wiper_seal",
        margin=0.002,
        name="rod stays centered through the weather wiper",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_rod",
        elem_b="rod_wiper_seal",
        min_overlap=0.010,
        name="rod remains engaged through the compressed wiper",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="piston_body",
        elem_b="barrel_shell",
        min_overlap=0.015,
        name="piston is retained inside the barrel at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_body",
            outer_elem="barrel_shell",
            margin=0.0,
            name="piston stays coaxial at full stroke",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="piston_body",
            elem_b="barrel_shell",
            min_overlap=0.015,
            name="piston remains inserted at full stroke",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="rear_stop_lip",
            negative_elem="thumb_plate",
            min_gap=0.0,
            max_gap=0.003,
            name="thumb plate reaches the rear mechanical stop",
        )
        pushed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates forward along the syringe axis",
        rest_pos is not None and pushed_pos is not None and pushed_pos[0] > rest_pos[0] + 0.075,
        details=f"rest={rest_pos}, pushed={pushed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
