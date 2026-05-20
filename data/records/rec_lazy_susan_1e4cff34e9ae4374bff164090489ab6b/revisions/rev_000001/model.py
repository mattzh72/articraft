from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_base_body():
    """Low stationary turntable base with a raised visible bearing race."""
    body = LatheGeometry(
        [
            (0.000, 0.000),
            (0.148, 0.000),
            (0.170, 0.006),
            (0.172, 0.018),
            (0.158, 0.034),
            (0.088, 0.039),
            (0.072, 0.045),
            (0.000, 0.045),
        ],
        segments=80,
    )
    return body


def _build_tray_shell():
    """Thin-walled shallow circular condiment tray with a rounded retaining lip."""
    tray = LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.006),
            (0.095, 0.006),
            (0.180, 0.008),
            (0.224, 0.012),
            (0.235, 0.026),
            (0.236, 0.056),
            (0.230, 0.066),
        ],
        [
            (0.000, 0.018),
            (0.095, 0.019),
            (0.178, 0.021),
            (0.211, 0.027),
            (0.218, 0.045),
            (0.219, 0.060),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    return tray


def _build_handle_arch():
    """Single raised tube arch that spans the tray diameter over the center."""
    return tube_from_spline_points(
        [
            (-0.178, 0.0, 0.064),
            (-0.145, 0.0, 0.150),
            (-0.070, 0.0, 0.245),
            (0.000, 0.0, 0.282),
            (0.070, 0.0, 0.245),
            (0.145, 0.0, 0.150),
            (0.178, 0.0, 0.064),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=24,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="condiment_lazy_susan")

    satin_black = model.material("satin_black", rgba=(0.025, 0.026, 0.028, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.010, 1.0))
    warm_ivory = model.material("warm_ivory", rgba=(0.88, 0.78, 0.60, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    shadow_gray = model.material("shadow_gray", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        _mesh("turntable_base", _build_base_body()),
        material=satin_black,
        name="turntable_base",
    )
    base.visual(
        _mesh(
            "bearing_race",
            TorusGeometry(radius=0.057, tube=0.0036, radial_segments=16, tubular_segments=80),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=brushed_steel,
        name="bearing_race",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0460)),
        material=brushed_steel,
        name="center_bearing_post",
    )
    base.visual(
        Cylinder(radius=0.162, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=black_rubber,
        name="rubber_foot_ring",
    )

    tray = model.part("tray")
    tray.visual(
        _mesh("tray_shell", _build_tray_shell()),
        material=warm_ivory,
        name="tray_shell",
    )
    tray.visual(
        _mesh(
            "rounded_rim",
            TorusGeometry(radius=0.223, tube=0.0065, radial_segments=18, tubular_segments=96),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=warm_ivory,
        name="rounded_rim",
    )
    tray.visual(
        Cylinder(radius=0.064, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shadow_gray,
        name="rotating_race",
    )
    tray.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=warm_ivory,
        name="center_hub",
    )

    # Three low ribs make the shallow tray read as a condiment organizer while
    # remaining fixed to the rotating tray.
    for index in range(3):
        angle = index * math.tau / 3.0
        tray.visual(
            Box((0.152, 0.010, 0.018)),
            origin=Origin(
                xyz=(0.076 * math.cos(angle), 0.076 * math.sin(angle), 0.036),
                rpy=(0.0, 0.0, angle),
            ),
            material=warm_ivory,
            name=f"divider_{index}",
        )

    tray.visual(
        _mesh("handle_arch", _build_handle_arch()),
        material=brushed_steel,
        name="handle_arch",
    )
    for index, x_pos in enumerate((-0.178, 0.178)):
        tray.visual(
            Cylinder(radius=0.020, length=0.052),
            origin=Origin(xyz=(x_pos, 0.0, 0.046)),
            material=brushed_steel,
            name=f"handle_socket_{index}",
        )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.0511)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    spin = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray uses continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.11,
        name="tray is centered over the low base",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="tray rides just above the base bearing",
    )

    rim_aabb = ctx.part_element_world_aabb(tray, elem="rounded_rim")
    handle_aabb = ctx.part_element_world_aabb(tray, elem="handle_arch")
    ctx.check(
        "handle arch rises well above the tray rim",
        rim_aabb is not None
        and handle_aabb is not None
        and handle_aabb[1][2] > rim_aabb[1][2] + 0.14,
        details=f"rim={rim_aabb}, handle={handle_aabb}",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            base,
            axes="xy",
            min_overlap=0.11,
            name="spun tray remains centered on the bearing",
        )
    ctx.check(
        "continuous spin keeps the tray on the central axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
