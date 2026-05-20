from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_HEIGHT = 3.50
RIM_RADIUS = 2.00
PIVOT_RADIUS = 2.50
WHEEL_HALF_WIDTH = 0.50
GONDOLA_COUNT = 8


def _circle_points(radius: float, y: float, count: int = 48) -> list[tuple[float, float, float]]:
    return [
        (radius * cos(2.0 * pi * index / count), y, radius * sin(2.0 * pi * index / count))
        for index in range(count)
    ]


def _add_tube(part, points, *, radius: float, name: str, material: Material, closed: bool = False) -> None:
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=8,
                radial_segments=18,
                closed_spline=closed,
                cap_ends=not closed,
                up_hint=(0.0, 1.0, 0.0),
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _radial_cylinder_origin(angle: float, radius: float, y: float = 0.0) -> Origin:
    return Origin(
        xyz=(radius * cos(angle), y, radius * sin(angle)),
        rpy=(0.0, pi / 2.0 - angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_observation_wheel")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.22, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    concrete = model.material("concrete", rgba=(0.45, 0.46, 0.44, 1.0))
    cabin_red = model.material("cabin_red", rgba=(0.82, 0.14, 0.10, 1.0))
    cabin_yellow = model.material("cabin_yellow", rgba=(0.94, 0.70, 0.12, 1.0))
    cabin_blue = model.material("cabin_blue", rgba=(0.12, 0.34, 0.70, 1.0))
    window_glass = model.material("window_glass", rgba=(0.40, 0.72, 0.92, 0.58))

    support = model.part("support_frame")
    support.visual(
        Box((3.20, 1.90, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="foundation",
    )
    support.visual(
        Box((3.00, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.78, 0.16)),
        material=dark_steel,
        name="base_rail_0",
    )
    support.visual(
        Box((3.00, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, -0.78, 0.16)),
        material=dark_steel,
        name="base_rail_1",
    )
    support.visual(
        Cylinder(radius=0.080, length=1.56),
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="fixed_axle",
    )
    for side_index, y in enumerate((0.72, -0.72)):
        _add_tube(
            support,
            [(-1.38, y, 0.10), (0.0, y, AXLE_HEIGHT)],
            radius=0.045,
            name=f"tower_leg_{side_index}_0",
            material=painted_steel,
        )
        _add_tube(
            support,
            [(1.38, y, 0.10), (0.0, y, AXLE_HEIGHT)],
            radius=0.045,
            name=f"tower_leg_{side_index}_1",
            material=painted_steel,
        )
        _add_tube(
            support,
            [(-1.45, y, 0.18), (1.45, y, 0.18)],
            radius=0.035,
            name=f"tower_foot_{side_index}",
            material=painted_steel,
        )
        # Cross braces land directly on the sloping legs, giving the A-frame
        # the triangulated look of real observation-wheel supports.
        low_t = (0.95 - 0.10) / (AXLE_HEIGHT - 0.10)
        high_t = (2.10 - 0.10) / (AXLE_HEIGHT - 0.10)
        left_low = (-1.38 * (1.0 - low_t), y, 0.95)
        right_low = (1.38 * (1.0 - low_t), y, 0.95)
        left_high = (-1.38 * (1.0 - high_t), y, 2.10)
        right_high = (1.38 * (1.0 - high_t), y, 2.10)
        _add_tube(
            support,
            [left_low, right_high],
            radius=0.024,
            name=f"tower_brace_{side_index}_0",
            material=dark_steel,
        )
        _add_tube(
            support,
            [right_low, left_high],
            radius=0.024,
            name=f"tower_brace_{side_index}_1",
            material=dark_steel,
        )
        support.visual(
            Cylinder(radius=0.22, length=0.14),
            origin=Origin(xyz=(0.0, y, AXLE_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_{side_index}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.19, length=1.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hub_sleeve",
    )
    wheel.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, WHEEL_HALF_WIDTH + 0.01, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="hub_cap_0",
    )
    wheel.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, -WHEEL_HALF_WIDTH - 0.01, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="hub_cap_1",
    )
    for side_index, y in enumerate((WHEEL_HALF_WIDTH, -WHEEL_HALF_WIDTH)):
        _add_tube(
            wheel,
            _circle_points(RIM_RADIUS, y, 64),
            radius=0.045,
            name=f"outer_rim_{side_index}",
            material=painted_steel,
            closed=True,
        )
        _add_tube(
            wheel,
            _circle_points(RIM_RADIUS * 0.86, y, 64),
            radius=0.026,
            name=f"inner_rim_{side_index}",
            material=painted_steel,
            closed=True,
        )
        for spoke_index in range(12):
            angle = 2.0 * pi * spoke_index / 12.0
            start_r = 0.15
            end_r = RIM_RADIUS + 0.020
            spoke_len = end_r - start_r
            spoke_mid = (start_r + end_r) * 0.5
            wheel.visual(
                Cylinder(radius=0.023, length=spoke_len),
                origin=_radial_cylinder_origin(angle, spoke_mid, y),
                material=bright_steel,
                name=f"spoke_{side_index}_{spoke_index}",
            )
        for mount_index in range(GONDOLA_COUNT):
            angle = 2.0 * pi * mount_index / GONDOLA_COUNT
            mount_len = PIVOT_RADIUS - RIM_RADIUS + 0.04
            mount_mid = RIM_RADIUS + mount_len * 0.5 - 0.02
            lug_name = (
                "pivot_lug_0_0"
                if side_index == 0 and mount_index == 0
                else f"pivot_lug_{side_index}_{mount_index}"
            )
            wheel.visual(
                Cylinder(radius=0.025, length=mount_len),
                origin=_radial_cylinder_origin(angle, mount_mid, y),
                material=painted_steel,
                name=f"pivot_strut_{side_index}_{mount_index}",
            )
            wheel.visual(
                Box((0.14, 0.08, 0.14)),
                origin=Origin(xyz=(PIVOT_RADIUS * cos(angle), y, PIVOT_RADIUS * sin(angle))),
                material=dark_steel,
                name=lug_name,
            )
    for tie_index in range(8):
        angle = 2.0 * pi * (tie_index + 0.5) / 8.0
        wheel.visual(
            Cylinder(radius=0.022, length=2.0 * WHEEL_HALF_WIDTH),
            origin=Origin(
                xyz=(RIM_RADIUS * cos(angle), 0.0, RIM_RADIUS * sin(angle)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=bright_steel,
            name=f"rim_tie_{tie_index}",
        )

    wheel_joint = model.articulation(
        "wheel_axis",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.35),
    )
    wheel_joint.meta["qc_samples"] = [0.0, pi / 4.0, pi / 2.0]

    cabin_mats = (cabin_red, cabin_yellow, cabin_blue, cabin_yellow)
    for index in range(GONDOLA_COUNT):
        angle = 2.0 * pi * index / GONDOLA_COUNT
        cabin_mat = cabin_mats[index % len(cabin_mats)]
        gondola = model.part(f"gondola_{index}")
        gondola.visual(
            Cylinder(radius=0.030, length=0.92),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name="pivot_bar",
        )
        for y in (0.36, -0.36):
            gondola.visual(
                Box((0.58, 0.030, 0.035)),
                origin=Origin(xyz=(0.0, y, -0.020)),
                material=dark_steel,
                name=f"top_hanger_{'front' if y > 0.0 else 'rear'}",
            )
            for x in (-0.27, 0.27):
                gondola.visual(
                    Box((0.030, 0.030, 0.48)),
                    origin=Origin(xyz=(x, y, -0.24)),
                    material=dark_steel,
                    name=f"hanger_{'front' if y > 0.0 else 'rear'}_{0 if x < 0 else 1}",
                )
        gondola.visual(
            Box((0.72, 0.76, 0.08)),
            origin=Origin(xyz=(0.0, 0.0, -0.50)),
            material=dark_steel,
            name="roof",
        )
        gondola.visual(
            Box((0.66, 0.70, 0.34)),
            origin=Origin(xyz=(0.0, 0.0, -0.70)),
            material=cabin_mat,
            name="cabin",
        )
        gondola.visual(
            Box((0.46, 0.012, 0.16)),
            origin=Origin(xyz=(0.0, 0.356, -0.70)),
            material=window_glass,
            name="front_window",
        )
        gondola.visual(
            Box((0.46, 0.012, 0.16)),
            origin=Origin(xyz=(0.0, -0.356, -0.70)),
            material=window_glass,
            name="rear_window",
        )
        gondola.visual(
            Box((0.012, 0.42, 0.16)),
            origin=Origin(xyz=(0.336, 0.0, -0.70)),
            material=window_glass,
            name="side_window_0",
        )
        gondola.visual(
            Box((0.012, 0.42, 0.16)),
            origin=Origin(xyz=(-0.336, 0.0, -0.70)),
            material=window_glass,
            name="side_window_1",
        )
        model.articulation(
            f"gondola_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(PIVOT_RADIUS * cos(angle), 0.0, PIVOT_RADIUS * sin(angle))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-pi, upper=pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_axis = object_model.get_articulation("wheel_axis")

    ctx.allow_overlap(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_sleeve",
        reason="The fixed axle is intentionally captured inside the rotating wheel hub sleeve.",
    )
    for cap_name in ("hub_cap_0", "hub_cap_1"):
        ctx.allow_overlap(
            support,
            wheel,
            elem_a="fixed_axle",
            elem_b=cap_name,
            reason="The fixed axle intentionally passes through the rotating hub cap bearing proxy.",
        )
    ctx.expect_within(
        support,
        wheel,
        axes="xz",
        inner_elem="fixed_axle",
        outer_elem="hub_sleeve",
        margin=0.002,
        name="axle is centered inside hub sleeve",
    )
    ctx.expect_overlap(
        support,
        wheel,
        axes="y",
        elem_a="fixed_axle",
        elem_b="hub_sleeve",
        min_overlap=0.90,
        name="hub remains carried on the axle",
    )
    for cap_name in ("hub_cap_0", "hub_cap_1"):
        ctx.expect_within(
            support,
            wheel,
            axes="xz",
            inner_elem="fixed_axle",
            outer_elem=cap_name,
            margin=0.002,
            name=f"axle passes through {cap_name}",
        )
        ctx.expect_overlap(
            support,
            wheel,
            axes="y",
            elem_a="fixed_axle",
            elem_b=cap_name,
            min_overlap=0.12,
            name=f"axle retained by {cap_name}",
        )
    ctx.check(
        "main wheel has continuous horizontal axis",
        wheel_axis is not None
        and wheel_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in (wheel_axis.axis or ())) == (0.0, 1.0, 0.0),
        details=f"wheel_axis={wheel_axis}",
    )

    gondolas = [object_model.get_part(f"gondola_{index}") for index in range(GONDOLA_COUNT)]
    ctx.check(
        "eight passenger gondolas",
        all(gondola is not None for gondola in gondolas),
        details=f"gondolas={gondolas!r}",
    )
    ctx.check(
        "each gondola has a revolute pivot",
        all(
            object_model.get_articulation(f"gondola_pivot_{index}") is not None
            and object_model.get_articulation(f"gondola_pivot_{index}").articulation_type == ArticulationType.REVOLUTE
            for index in range(GONDOLA_COUNT)
        ),
        details="Expected one transverse revolute pivot per gondola.",
    )

    gondola_0 = object_model.get_part("gondola_0")
    pivot_0 = object_model.get_articulation("gondola_pivot_0")
    if gondola_0 is not None and pivot_0 is not None:
        ctx.expect_contact(
            gondola_0,
            wheel,
            elem_a="pivot_bar",
            elem_b="pivot_lug_0_0",
            contact_tol=0.015,
            name="gondola pivot bar reaches wheel lug",
        )
        with ctx.pose({wheel_axis: pi / 2.0, pivot_0: -pi / 2.0}):
            pivot_pos = ctx.part_world_position(gondola_0)
            cabin_aabb = ctx.part_element_world_aabb(gondola_0, elem="cabin")
            ok = pivot_pos is not None and cabin_aabb is not None and cabin_aabb[1][2] < pivot_pos[2] - 0.05
            ctx.check(
                "gondola can stay hanging below pivot while wheel turns",
                ok,
                details=f"pivot={pivot_pos}, cabin_aabb={cabin_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
