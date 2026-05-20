from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _ring_band(outer_radius: float, inner_radius: float, z_min: float, z_max: float, *, segments: int = 72):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _barrel_shell_geometry():
    # A clear, thin-wall reservoir with rounded lips; the bore remains open for
    # the plunger seal and rod, rather than being represented as a solid proxy.
    outer = [
        (0.0118, 0.000),
        (0.0128, 0.004),
        (0.0126, 0.018),
        (0.0122, 0.060),
        (0.0125, 0.098),
        (0.0135, 0.106),
        (0.0126, 0.114),
    ]
    inner = [
        (0.00945, 0.000),
        (0.01025, 0.006),
        (0.01040, 0.018),
        (0.01040, 0.098),
        (0.00995, 0.106),
        (0.00880, 0.114),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _nozzle_geometry():
    outer = [
        (0.00185, -0.044),
        (0.00245, -0.038),
        (0.00300, -0.026),
        (0.00580, -0.012),
        (0.00710, -0.002),
        (0.01000, 0.004),
    ]
    inner = [
        (0.00085, -0.044),
        (0.00100, -0.032),
        (0.00115, -0.018),
        (0.00150, -0.006),
        (0.00300, 0.004),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )


def _rounded_plate(width: float, height: float, thickness: float, radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        center=True,
        cap=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_syringe")

    clear_polymer = model.material("clear_polymer", rgba=(0.78, 0.92, 1.00, 0.36))
    frosted_polymer = model.material("frosted_polymer", rgba=(0.86, 0.93, 0.98, 0.64))
    printed_ink = model.material("printed_ink", rgba=(0.06, 0.09, 0.11, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    painted_metal = model.material("painted_metal", rgba=(0.18, 0.22, 0.27, 1.0))
    elastomer = model.material("elastomer", rgba=(0.03, 0.035, 0.04, 1.0))
    soft_polymer = model.material("soft_polymer", rgba=(0.23, 0.37, 0.48, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _mesh(_barrel_shell_geometry(), "barrel_shell"),
        material=clear_polymer,
        name="barrel_shell",
    )
    barrel.visual(
        _mesh(_nozzle_geometry(), "nozzle_tip"),
        material=frosted_polymer,
        name="nozzle_tip",
    )
    barrel.visual(
        _mesh(_ring_band(0.0083, 0.0030, -0.018, -0.006, segments=72), "luer_collar"),
        material=satin_metal,
        name="luer_collar",
    )
    barrel.visual(
        _mesh(_ring_band(0.0112, 0.0092, 0.010, 0.014, segments=72), "front_stop_lip"),
        material=frosted_polymer,
        name="front_stop_lip",
    )
    barrel.visual(
        _mesh(_ring_band(0.0145, 0.0038, 0.107, 0.116, segments=72), "rear_guide"),
        material=satin_metal,
        name="rear_guide",
    )
    barrel.visual(
        _mesh(_rounded_plate(0.032, 0.014, 0.0042, 0.004), "finger_tab_0"),
        origin=Origin(xyz=(0.027, 0.0, 0.112)),
        material=satin_metal,
        name="finger_tab_0",
    )
    barrel.visual(
        _mesh(_rounded_plate(0.032, 0.014, 0.0042, 0.004), "finger_tab_1"),
        origin=Origin(xyz=(-0.027, 0.0, 0.112)),
        material=satin_metal,
        name="finger_tab_1",
    )
    # Printed/etched graduation marks are shallow annular ink bands seated into
    # the clear barrel shell, giving a readable scale without floating decals.
    for index in range(11):
        z = 0.018 + index * 0.0072
        major = index % 2 == 0
        barrel.visual(
            _mesh(
                _ring_band(
                    0.01295 if major else 0.01278,
                    0.01205,
                    z - (0.00055 if major else 0.00034),
                    z + (0.00055 if major else 0.00034),
                    segments=72,
                ),
                f"graduation_{index}",
            ),
            material=printed_ink,
            name=f"graduation_{index}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00875, length=0.010),
        origin=Origin(),
        material=elastomer,
        name="piston_seal",
    )
    plunger.visual(
        Cylinder(radius=0.01055, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, -0.0038)),
        material=elastomer,
        name="front_wiper",
    )
    plunger.visual(
        Cylinder(radius=0.01055, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0038)),
        material=elastomer,
        name="rear_wiper",
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=painted_metal,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.0012, 0.0066, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=painted_metal,
        name="rod_web_0",
    )
    plunger.visual(
        Box((0.0066, 0.0012, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=painted_metal,
        name="rod_web_1",
    )
    plunger.visual(
        Cylinder(radius=0.0061, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=satin_metal,
        name="thumb_collar",
    )
    plunger.visual(
        _mesh(_rounded_plate(0.034, 0.021, 0.0060, 0.006), "thumb_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=soft_polymer,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.16, lower=0.0, upper=0.075),
        motion_properties=MotionProperties(damping=0.25, friction=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a="front_wiper",
        elem_b="barrel_shell",
        reason="The elastomer front wiper is intentionally shown under slight radial compression against the clear barrel bore.",
    )
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a="rear_wiper",
        elem_b="barrel_shell",
        reason="The elastomer rear wiper is intentionally shown under slight radial compression against the clear barrel bore.",
    )

    ctx.check(
        "plunger is a bounded coaxial linear slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.075,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.002,
            name="piston seal is coaxial inside the barrel bore at rest",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="piston_seal",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="resting piston is retained inside the barrel",
        )
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a="front_wiper",
            elem_b="barrel_shell",
            name="front elastomer wiper seals against the barrel wall",
        )
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a="rear_wiper",
            elem_b="barrel_shell",
            name="rear elastomer wiper seals against the barrel wall",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem="piston_seal",
            negative_elem="front_stop_lip",
            min_gap=0.0,
            max_gap=0.003,
            name="front stop lip bounds the depressed plunger",
        )
        rest_position = ctx.part_world_position(plunger)

    with ctx.pose({slide: 0.075}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.002,
            name="withdrawn piston remains coaxial in the barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="piston_seal",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="withdrawn piston remains retained inside the barrel",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="z",
            positive_elem="rear_guide",
            negative_elem="piston_seal",
            min_gap=0.0,
            max_gap=0.010,
            name="rear guide bounds maximum withdrawal",
        )
        extended_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger travels outward along the syringe axis",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.070,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
