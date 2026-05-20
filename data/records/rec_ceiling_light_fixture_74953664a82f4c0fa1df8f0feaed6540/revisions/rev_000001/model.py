from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_gimbal_ceiling_light")

    antique_brass = Material("warm_antique_brass", rgba=(0.72, 0.52, 0.25, 1.0))
    dark_shadow = Material("shadowed_metal", rgba=(0.10, 0.09, 0.075, 1.0))
    frosted_glass = Material("warm_frosted_glass", rgba=(0.92, 0.86, 0.72, 0.58))

    fixture = model.part("fixture")

    # A shallow, round ceiling canopy with a rolled lower lip.  The whole fixture
    # hangs below z=0, so the flat top reads as semi-flush to a ceiling plane.
    fixture.visual(
        Cylinder(radius=0.180, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=antique_brass,
        name="canopy",
    )
    fixture.visual(
        mesh_from_geometry(TorusGeometry(radius=0.162, tube=0.010, radial_segments=16, tubular_segments=72), "canopy_rolled_lip"),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=antique_brass,
        name="canopy_rolled_lip",
    )
    fixture.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=antique_brass,
        name="stem_collar",
    )
    fixture.visual(
        Cylinder(radius=0.020, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=antique_brass,
        name="short_stem",
    )

    # The horizontal gimbal ring is physically carried by a cross yoke from the
    # short stem.  Slight visual intersections keep the fixture as one supported
    # manufactured assembly rather than separated decorative islands.
    fixture.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=antique_brass,
        name="lower_hub",
    )
    fixture.visual(
        Cylinder(radius=0.012, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, -0.270), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=antique_brass,
        name="yoke_bridge",
    )
    fixture.visual(
        Cylinder(radius=0.011, length=0.066),
        origin=Origin(xyz=(0.0, 0.340, -0.292)),
        material=antique_brass,
        name="ring_post_0",
    )
    fixture.visual(
        Cylinder(radius=0.011, length=0.066),
        origin=Origin(xyz=(0.0, -0.340, -0.292)),
        material=antique_brass,
        name="ring_post_1",
    )
    fixture.visual(
        mesh_from_geometry(TorusGeometry(radius=0.340, tube=0.013, radial_segments=18, tubular_segments=96), "gimbal_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=antique_brass,
        name="gimbal_ring",
    )
    fixture.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.347, 0.0, -0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="pivot_socket_0",
    )
    fixture.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(-0.347, 0.0, -0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="pivot_socket_1",
    )
    fixture.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.105, 0.0, -0.058)),
        material=dark_shadow,
        name="canopy_screw_0",
    )
    fixture.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.105, 0.0, -0.058)),
        material=dark_shadow,
        name="canopy_screw_1",
    )

    shade = model.part("shade")

    # Thin-walled lathed glass: a wide open rim at the top rolls down into a
    # shallow bowl.  Separate inner and outer profiles make the shade hollow
    # rather than a solid frosted blob.
    shade_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.050, -0.245),
            (0.145, -0.220),
            (0.235, -0.145),
            (0.292, -0.055),
            (0.307, 0.000),
        ],
        inner_profile=[
            (0.028, -0.224),
            (0.120, -0.202),
            (0.205, -0.132),
            (0.263, -0.047),
            (0.286, -0.006),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "shade_glass_shell"),
        origin=Origin(),
        material=frosted_glass,
        name="shade_shell",
    )
    shade.visual(
        mesh_from_geometry(TorusGeometry(radius=0.302, tube=0.008, radial_segments=14, tubular_segments=96), "shade_rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=antique_brass,
        name="shade_rim",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.072),
        origin=Origin(xyz=(0.329, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="pivot_pin_0",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.072),
        origin=Origin(xyz=(-0.329, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="pivot_pin_1",
    )
    shade.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.246)),
        material=antique_brass,
        name="bottom_finial_stem",
    )
    shade.visual(
        Sphere(radius=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=antique_brass,
        name="bottom_finial",
    )

    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixture = object_model.get_part("fixture")
    shade = object_model.get_part("shade")
    tilt = object_model.get_articulation("shade_tilt")

    for idx in (0, 1):
        ctx.allow_overlap(
            fixture,
            shade,
            elem_a="gimbal_ring",
            elem_b=f"pivot_pin_{idx}",
            reason="The pin passes through the gimbal ring wall at the trunnion bearing, represented as a local captured overlap.",
        )
        ctx.allow_overlap(
            fixture,
            shade,
            elem_a=f"pivot_socket_{idx}",
            elem_b=f"pivot_pin_{idx}",
            reason="Each shade trunnion pin is intentionally captured inside a metal gimbal socket.",
        )
        ctx.expect_overlap(
            fixture,
            shade,
            axes="xyz",
            min_overlap=0.008,
            elem_a=f"pivot_socket_{idx}",
            elem_b=f"pivot_pin_{idx}",
            name=f"pivot pin {idx} is retained by its socket",
        )
        ctx.expect_overlap(
            fixture,
            shade,
            axes="xyz",
            min_overlap=0.006,
            elem_a="gimbal_ring",
            elem_b=f"pivot_pin_{idx}",
            name=f"pivot pin {idx} passes through the gimbal ring wall",
        )

    ctx.expect_within(
        shade,
        fixture,
        axes="xy",
        inner_elem="shade_rim",
        outer_elem="gimbal_ring",
        margin=0.005,
        name="wide shade rim sits inside the gimbal ring",
    )
    ctx.expect_overlap(
        fixture,
        shade,
        axes="z",
        min_overlap=0.010,
        elem_a="gimbal_ring",
        elem_b="shade_rim",
        name="gimbal ring surrounds the shade rim elevation",
    )

    rest_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    with ctx.pose({tilt: 0.50}):
        tilted_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        tilted_center_y = None if tilted_aabb is None else (tilted_aabb[0][1] + tilted_aabb[1][1]) / 2.0
        ctx.expect_overlap(
            fixture,
            shade,
            axes="x",
            min_overlap=0.020,
            elem_a="pivot_socket_0",
            elem_b="pivot_pin_0",
            name="tilted shade remains on the pivot axis",
        )
    ctx.check(
        "positive tilt visibly angles the bowl shade",
        rest_center_y is not None
        and tilted_center_y is not None
        and tilted_center_y > rest_center_y + 0.045,
        details=f"rest_y={rest_center_y}, tilted_y={tilted_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
