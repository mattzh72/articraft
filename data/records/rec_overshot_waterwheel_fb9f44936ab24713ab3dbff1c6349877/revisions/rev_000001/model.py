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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel_millrace")

    stone = model.material("stone", rgba=(0.60, 0.60, 0.58, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.47, 0.47, 0.45, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.48, 0.34, 0.22, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.34, 0.23, 0.15, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))

    wheel_outer_radius = 0.72
    rim_center_radius = 0.67
    rim_tube_radius = 0.05
    wheel_center_z = 0.86
    wheel_half_width = 0.09
    axle_radius = 0.045
    axle_length = 0.64
    hub_radius = 0.11
    hub_length = 0.24

    def cylinder_along_x(
        part,
        *,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        name: str,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    frame = model.part("frame")
    frame.visual(
        Box((2.20, 1.80, 0.12)),
        origin=Origin(xyz=(0.0, -0.40, 0.06)),
        material=dark_stone,
        name="base_plinth",
    )
    frame.visual(
        Box((1.30, 0.42, 1.48)),
        origin=Origin(xyz=(0.0, -1.00, 0.86)),
        material=stone,
        name="millrace_abutment",
    )
    frame.visual(
        Box((1.24, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, -1.00, 1.61)),
        material=dark_stone,
        name="millrace_floor",
    )
    frame.visual(
        Box((1.24, 0.05, 0.20)),
        origin=Origin(xyz=(0.0, -1.105, 1.69)),
        material=stone,
        name="millrace_outer_wall",
    )
    frame.visual(
        Box((1.24, 0.05, 0.20)),
        origin=Origin(xyz=(0.0, -0.895, 1.69)),
        material=stone,
        name="millrace_inner_wall",
    )
    frame.visual(
        Box((0.75, 0.72, 0.04)),
        origin=Origin(xyz=(0.0, -0.495, 1.61)),
        material=weathered_wood,
        name="chute_floor",
    )
    frame.visual(
        Box((0.04, 0.75, 0.18)),
        origin=Origin(xyz=(-0.34, -0.495, 1.69)),
        material=weathered_wood,
        name="chute_left_wall",
    )
    frame.visual(
        Box((0.04, 0.75, 0.18)),
        origin=Origin(xyz=(0.34, -0.495, 1.69)),
        material=weathered_wood,
        name="chute_right_wall",
    )
    frame.visual(
        Box((0.12, 0.42, 1.08)),
        origin=Origin(xyz=(-0.43, 0.00, 0.66)),
        material=stone,
        name="left_support",
    )
    frame.visual(
        Box((0.12, 0.42, 1.08)),
        origin=Origin(xyz=(0.43, 0.00, 0.66)),
        material=stone,
        name="right_support",
    )
    frame.visual(
        Box((0.12, 0.16, 0.10)),
        origin=Origin(xyz=(-0.32, 0.00, 0.765)),
        material=weathered_wood,
        name="bearing_left",
    )
    frame.visual(
        Box((0.12, 0.16, 0.10)),
        origin=Origin(xyz=(0.32, 0.00, 0.765)),
        material=weathered_wood,
        name="bearing_right",
    )
    frame.visual(
        Box((0.03, 0.06, 0.38)),
        origin=Origin(xyz=(-0.31, -0.20, 1.79)),
        material=weathered_wood,
        name="guide_left",
    )
    frame.visual(
        Box((0.03, 0.06, 0.38)),
        origin=Origin(xyz=(0.31, -0.20, 1.79)),
        material=weathered_wood,
        name="guide_right",
    )
    frame.visual(
        Box((0.66, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.20, 1.97)),
        material=weathered_wood,
        name="gate_header",
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.20, 1.80, 1.96)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, -0.40, 0.98)),
    )

    wheel = model.part("wheel")
    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=rim_center_radius, tube=rim_tube_radius).rotate_y(math.pi / 2.0),
        "waterwheel_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-wheel_half_width, 0.0, 0.0)),
        material=wet_wood,
        name="rim_left",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(wheel_half_width, 0.0, 0.0)),
        material=wet_wood,
        name="rim_right",
    )
    wheel.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle",
    )
    cylinder_along_x(
        wheel,
        radius=hub_radius,
        length=hub_length,
        xyz=(0.0, 0.0, 0.0),
        material=wet_wood,
        name="hub_barrel",
    )

    spoke_count = 10
    spoke_inner_radius = 0.09
    spoke_outer_radius = 0.63
    spoke_length = spoke_outer_radius - spoke_inner_radius
    spoke_mid_radius = 0.5 * (spoke_inner_radius + spoke_outer_radius)
    for side_index, side_x in enumerate((-wheel_half_width, wheel_half_width)):
        for spoke_index in range(spoke_count):
            theta = (2.0 * math.pi * spoke_index) / spoke_count
            spoke_pitch = theta - (math.pi / 2.0)
            wheel.visual(
                Cylinder(radius=0.018, length=spoke_length),
                origin=Origin(
                    xyz=(
                        side_x,
                        spoke_mid_radius * math.cos(theta),
                        spoke_mid_radius * math.sin(theta),
                    ),
                    rpy=(spoke_pitch, 0.0, 0.0),
                ),
                material=weathered_wood,
                name=f"spoke_{side_index}_{spoke_index:02d}",
            )

    bucket_count = 12
    for bucket_index in range(bucket_count):
        theta = (2.0 * math.pi * bucket_index) / bucket_count + (math.pi / bucket_count)
        bucket_pitch = theta - (math.pi / 2.0)
        wheel.visual(
            Box((0.18, 0.03, 0.18)),
            origin=Origin(
                xyz=(0.0, 0.62 * math.cos(theta), 0.62 * math.sin(theta)),
                rpy=(bucket_pitch, 0.0, 0.0),
            ),
            material=wet_wood,
            name=f"bucket_{bucket_index:02d}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_outer_radius, length=0.28),
        mass=85.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    gate = model.part("sluice_gate")
    gate.visual(
        Box((0.58, 0.03, 0.18)),
        material=weathered_wood,
        name="gate_panel",
    )
    gate.inertial = Inertial.from_geometry(Box((0.58, 0.03, 0.18)), mass=8.0)

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=3.0),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, -0.20, 1.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.20, lower=0.0, upper=0.12),
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

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("sluice_gate")
    wheel_spin = object_model.get_articulation("wheel_spin")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "wheel spins about a horizontal axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "sluice gate slides vertically",
        gate_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in gate_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_left",
        contact_tol=0.002,
        name="left axle end rests on left bearing",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_right",
        contact_tol=0.002,
        name="right axle end rests on right bearing",
    )

    with ctx.pose({gate_slide: 0.0}):
        ctx.expect_gap(
            gate,
            frame,
            axis="z",
            positive_elem="gate_panel",
            negative_elem="chute_floor",
            max_gap=0.003,
            max_penetration=0.001,
            name="closed gate panel reaches the chute floor",
        )
        ctx.expect_gap(
            gate,
            frame,
            axis="x",
            positive_elem="gate_panel",
            negative_elem="guide_left",
            min_gap=0.003,
            max_gap=0.020,
            name="closed gate clears the left guide",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="x",
            positive_elem="guide_right",
            negative_elem="gate_panel",
            min_gap=0.003,
            max_gap=0.020,
            name="closed gate clears the right guide",
        )
        closed_gate_pos = ctx.part_world_position(gate)

    with ctx.pose({gate_slide: 0.12}):
        ctx.expect_gap(
            gate,
            frame,
            axis="z",
            positive_elem="gate_panel",
            negative_elem="chute_floor",
            min_gap=0.11,
            name="opened gate lifts a clear flow opening",
        )
        open_gate_pos = ctx.part_world_position(gate)

    ctx.check(
        "positive gate travel raises the panel",
        closed_gate_pos is not None
        and open_gate_pos is not None
        and open_gate_pos[2] > closed_gate_pos[2] + 0.10,
        details=f"closed={closed_gate_pos}, open={open_gate_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
