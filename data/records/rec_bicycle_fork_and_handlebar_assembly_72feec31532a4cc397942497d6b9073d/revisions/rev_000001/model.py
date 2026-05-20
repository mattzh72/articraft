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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube(points, radius, name, *, segments=18, radial_segments=18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=segments,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _lever_mesh(name: str):
    """One connected brake lever mesh in a pivot-centered local frame."""
    geom = CylinderGeometry(0.007, 0.030, radial_segments=24, closed=True)
    geom.rotate_x(-math.pi / 2.0)
    blade = tube_from_spline_points(
        [
            (0.001, 0.0, -0.003),
            (0.012, 0.0, -0.055),
            (0.004, 0.0, -0.120),
            (-0.006, 0.0, -0.175),
        ],
        radius=0.0045,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    geom.merge(blade)
    paddle = BoxGeometry((0.012, 0.010, 0.042))
    paddle.translate(-0.008, 0.0, -0.177)
    geom.merge(paddle)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_fork_handlebar_assembly")

    frame_blue = model.material("painted_frame_blue", rgba=(0.06, 0.18, 0.34, 1.0))
    satin_black = model.material("satin_black", rgba=(0.005, 0.006, 0.007, 1.0))
    carbon_black = model.material("carbon_black", rgba=(0.018, 0.020, 0.022, 1.0))
    rubber = model.material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.61, 0.56, 1.0))
    dark_metal = model.material("anodized_dark_metal", rgba=(0.07, 0.075, 0.080, 1.0))

    head = model.part("head_tube")
    head_shell = LatheGeometry.from_shell_profiles(
        [(0.033, 0.0), (0.034, 0.018), (0.034, 0.182), (0.033, 0.200)],
        [(0.023, 0.0), (0.023, 0.200)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_tube_shell"),
        material=frame_blue,
        name="head_tube_shell",
    )
    for z, visual_name in ((0.0, "lower_bearing_cup"), (0.200, "upper_bearing_cup")):
        head.visual(
            mesh_from_geometry(TorusGeometry(0.029, 0.0045, radial_segments=24, tubular_segments=64), visual_name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_metal,
            name=visual_name,
        )
    head.visual(
        _tube([(-0.156, 0.0, 0.158), (-0.096, 0.0, 0.158), (-0.030, 0.0, 0.155)], 0.017, "top_tube_stub"),
        material=frame_blue,
        name="top_tube_stub",
    )
    head.visual(
        _tube([(-0.180, 0.0, -0.078), (-0.108, 0.0, -0.020), (-0.036, 0.0, 0.050)], 0.020, "down_tube_stub"),
        material=frame_blue,
        name="down_tube_stub",
    )

    front = model.part("front_end")
    front.visual(
        Cylinder(0.014, 0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=brushed_metal,
        name="steerer_tube",
    )
    front.visual(
        Cylinder(0.031, 0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=brushed_metal,
        name="upper_steerer_race",
    )
    front.visual(
        Cylinder(0.031, 0.007),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=brushed_metal,
        name="lower_steerer_race",
    )
    front.visual(
        Cylinder(0.024, 0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=dark_metal,
        name="steerer_clamp",
    )
    front.visual(
        Box((0.018, 0.018, 0.028)),
        origin=Origin(xyz=(0.024, 0.0, 0.286)),
        material=brushed_metal,
        name="steerer_clamp_gap",
    )
    for i, z in enumerate((0.252, 0.288)):
        front.visual(
            Cylinder(0.0042, 0.020),
            origin=Origin(xyz=(0.026, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"steerer_clamp_bolt_{i}",
        )

    front.visual(
        Box((0.094, 0.168, 0.046)),
        origin=Origin(xyz=(0.018, 0.0, -0.078)),
        material=dark_metal,
        name="crown_body",
    )
    front.visual(
        Cylinder(0.028, 0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_metal,
        name="crown_steerer_socket",
    )
    for side, y in ((0, -0.056), (1, 0.056)):
        front.visual(
            Cylinder(0.018, 0.056),
            origin=Origin(xyz=(0.018, y, -0.092)),
            material=dark_metal,
            name=f"blade_socket_{side}",
        )
        front.visual(
            _tube(
                [
                    (0.018, y, -0.104),
                    (0.006, y * 1.02, -0.205),
                    (0.022, y * 1.06, -0.330),
                    (0.058, y * 1.10, -0.455),
                ],
                0.0115,
                f"fork_blade_{side}",
                segments=20,
                radial_segments=20,
            ),
            material=carbon_black,
            name=f"fork_blade_{side}",
        )
        front.visual(
            Box((0.020, 0.030, 0.060)),
            origin=Origin(xyz=(0.060, y * 1.11, -0.462)),
            material=dark_metal,
            name=f"dropout_plate_{side}",
        )
        dropout_eye = TorusGeometry(0.012, 0.0032, radial_segments=18, tubular_segments=40)
        front.visual(
            mesh_from_geometry(dropout_eye, f"dropout_eye_{side}"),
            origin=Origin(xyz=(0.062, y * 1.13, -0.452), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"dropout_eye_{side}",
        )

    front.visual(
        _tube([(0.000, 0.0, 0.310), (0.064, 0.0, 0.324), (0.180, 0.0, 0.350)], 0.014, "stem_body"),
        material=dark_metal,
        name="stem_body",
    )
    front.visual(
        Cylinder(0.030, 0.072),
        origin=Origin(xyz=(0.180, 0.0, 0.350), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    front.visual(
        Box((0.010, 0.075, 0.060)),
        origin=Origin(xyz=(0.211, 0.0, 0.350)),
        material=dark_metal,
        name="stem_faceplate",
    )
    bolt_index = 0
    for y in (-0.026, 0.026):
        for z in (0.330, 0.370):
            front.visual(
                Cylinder(0.0042, 0.010),
                origin=Origin(xyz=(0.218, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brushed_metal,
                name=f"faceplate_bolt_{bolt_index}",
            )
            bolt_index += 1

    handlebar_points = [
        (0.246, -0.314, 0.078),
        (0.276, -0.314, 0.166),
        (0.288, -0.300, 0.248),
        (0.246, -0.270, 0.318),
        (0.184, -0.190, 0.350),
        (0.180, 0.000, 0.350),
        (0.184, 0.190, 0.350),
        (0.246, 0.270, 0.318),
        (0.288, 0.300, 0.248),
        (0.276, 0.314, 0.166),
        (0.246, 0.314, 0.078),
    ]
    front.visual(
        _tube(handlebar_points, 0.012, "handlebar_tube", segments=18, radial_segments=24),
        material=rubber,
        name="handlebar_tube",
    )

    for side, y in ((0, -0.286), (1, 0.286)):
        front.visual(
            Box((0.058, 0.038, 0.046)),
            origin=Origin(xyz=(0.260, y, 0.342)),
            material=rubber,
            name=f"brake_hood_{side}",
        )
        front.visual(
            Box((0.070, 0.030, 0.012)),
            origin=Origin(xyz=(0.286, y, 0.315)),
            material=dark_metal,
            name=f"brake_boss_bridge_{side}",
        )
        front.visual(
            Cylinder(0.011, 0.034),
            origin=Origin(xyz=(0.320, y, 0.300), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"brake_pivot_boss_{side}",
        )
        front.visual(
            _tube(
                [
                    (0.260, y, 0.352),
                    (0.205, y * 0.82, 0.390),
                    (0.118, y * 0.45, 0.405),
                    (0.030, y * 0.18, 0.365),
                ],
                0.0022,
                f"brake_cable_{side}",
                segments=14,
                radial_segments=10,
            ),
            material=rubber,
            name=f"brake_cable_{side}",
        )

    steering = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head,
        child=front,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-1.15, upper=1.15),
    )
    steering.meta["description"] = "Steering rotates the fork, steerer, stem, and handlebar together about the steerer axis."

    for side, y in ((0, -0.286), (1, 0.286)):
        lever = model.part(f"brake_lever_{side}")
        lever.visual(
            _lever_mesh(f"lever_hardware_{side}"),
            material=brushed_metal,
            name=f"lever_hardware_{side}",
        )
        model.articulation(
            f"brake_pivot_{side}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=lever,
            origin=Origin(xyz=(0.320, y, 0.300)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=0.0, upper=0.48),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head_tube")
    front = object_model.get_part("front_end")
    steering = object_model.get_articulation("steering")

    ctx.expect_overlap(
        front,
        head,
        axes="z",
        elem_a="steerer_tube",
        elem_b="head_tube_shell",
        min_overlap=0.185,
        name="steerer spans the head tube stub",
    )
    ctx.expect_within(
        front,
        head,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="head_tube_shell",
        margin=0.0,
        name="steerer stays concentric inside the head tube envelope",
    )
    ctx.expect_gap(
        front,
        head,
        axis="z",
        positive_elem="steerer_clamp",
        negative_elem="upper_bearing_cup",
        min_gap=0.006,
        max_gap=0.040,
        name="stem clamp clears the upper bearing cup",
    )

    def _center_of_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb else None

    rest_bar = _center_of_aabb(ctx.part_element_world_aabb(front, elem="handlebar_tube"))
    with ctx.pose({steering: 0.70}):
        turned_bar = _center_of_aabb(ctx.part_element_world_aabb(front, elem="handlebar_tube"))
    ctx.check(
        "steering turns the handlebar around the steerer",
        rest_bar is not None
        and turned_bar is not None
        and turned_bar[1] > rest_bar[1] + 0.08
        and abs(turned_bar[2] - rest_bar[2]) < 0.004,
        details=f"rest_bar={rest_bar}, turned_bar={turned_bar}",
    )

    for side in (0, 1):
        lever = object_model.get_part(f"brake_lever_{side}")
        pivot = object_model.get_articulation(f"brake_pivot_{side}")
        ctx.allow_overlap(
            front,
            lever,
            elem_a=f"brake_pivot_boss_{side}",
            elem_b=f"lever_hardware_{side}",
            reason="The lever pivot knuckle is intentionally nested in the compact brake body boss.",
        )
        ctx.expect_overlap(
            front,
            lever,
            axes="xyz",
            elem_a=f"brake_pivot_boss_{side}",
            elem_b=f"lever_hardware_{side}",
            min_overlap=0.006,
            name=f"brake lever {side} is captured in its pivot boss",
        )
        rest_lever = _center_of_aabb(ctx.part_element_world_aabb(lever, elem=f"lever_hardware_{side}"))
        with ctx.pose({pivot: 0.42}):
            squeezed_lever = _center_of_aabb(ctx.part_element_world_aabb(lever, elem=f"lever_hardware_{side}"))
        ctx.check(
            f"brake lever {side} squeezes toward the bar",
            rest_lever is not None
            and squeezed_lever is not None
            and squeezed_lever[0] < rest_lever[0] - 0.020,
            details=f"rest={rest_lever}, squeezed={squeezed_lever}",
        )

    return ctx.report()


object_model = build_object_model()
