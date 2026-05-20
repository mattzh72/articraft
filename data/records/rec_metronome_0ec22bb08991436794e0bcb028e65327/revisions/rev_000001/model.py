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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_front_plate_mesh(
    *,
    radius: float,
    thickness: float,
    slot_width: float,
    slot_height: float,
    slot_center_z: float,
):
    slot_profile = _offset_profile(
        rounded_rect_profile(
            slot_width,
            slot_height,
            radius=min(slot_width * 0.45, 0.008),
            corner_segments=8,
        ),
        dy=slot_center_z,
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(radius, segments=72),
            [slot_profile],
            thickness,
            center=True,
        ).rotate_x(-math.pi / 2.0),
        "metronome_front_plate",
    )


def _build_weight_ring_mesh(*, outer_radius: float, inner_radius: float, height: float):
    outer_profile = [
        (outer_radius, -height * 0.5),
        (outer_radius, height * 0.5),
    ]
    inner_profile = [
        (inner_radius, -height * 0.5),
        (inner_radius, height * 0.5),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
        ),
        "metronome_weight_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_metronome")

    body_finish = model.material("body_finish", rgba=(0.16, 0.16, 0.18, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.70, 0.63, 0.42, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.66, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.30, 1.0))

    housing_radius = 0.078
    shell_thickness = 0.006
    housing_depth = 0.056
    housing_center_z = 0.108
    front_plate_thickness = 0.004
    slot_width = 0.062
    slot_height = 0.142
    slot_center_z = 0.007
    pivot_y = -0.029
    pivot_z = -0.003

    front_plate_mesh = _build_front_plate_mesh(
        radius=housing_radius,
        thickness=front_plate_thickness,
        slot_width=slot_width,
        slot_height=slot_height,
        slot_center_z=slot_center_z,
    )
    shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (housing_radius, -housing_depth * 0.5),
                (housing_radius, housing_depth * 0.5),
            ],
            [
                (housing_radius - shell_thickness, -housing_depth * 0.5),
                (housing_radius - shell_thickness, housing_depth * 0.5),
            ],
            segments=72,
        ).rotate_x(-math.pi / 2.0),
        "metronome_housing_shell",
    )
    weight_ring_mesh = _build_weight_ring_mesh(
        outer_radius=0.017,
        inner_radius=0.0050,
        height=0.012,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.130, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_finish,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=trim_finish,
        name="base_pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.030),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    housing = model.part("housing")
    housing.visual(
        shell_mesh,
        material=body_finish,
        name="housing_shell",
    )
    housing.visual(
        front_plate_mesh,
        origin=Origin(xyz=(0.0, -(housing_depth * 0.5 + front_plate_thickness * 0.5), 0.0)),
        material=trim_finish,
        name="front_face",
    )
    housing.visual(
        Cylinder(radius=housing_radius, length=front_plate_thickness),
        origin=Origin(
            xyz=(0.0, housing_depth * 0.5 + front_plate_thickness * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_finish,
        name="rear_cover",
    )
    housing.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, housing_radius + 0.008)),
        material=trim_finish,
        name="crown_socket",
    )
    housing.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, housing_radius + 0.018)),
        material=trim_finish,
        name="crown_socket_cap",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.170, 0.068, 0.170)),
        mass=1.2,
        origin=Origin(),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=steel,
        name="upper_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0028, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=steel,
        name="lower_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0095, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=brass,
        name="lower_counterweight",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, 0.250)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    weight = model.part("weight")
    weight.visual(
        weight_ring_mesh,
        material=brass,
        name="ring_weight",
    )
    weight.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.012),
        mass=0.08,
        origin=Origin(),
    )

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="crown_stem",
    )
    crown.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=brass,
        name="crown_knob",
    )
    crown.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=trim_finish,
        name="crown_cap",
    )
    crown.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.028),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.FIXED,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, housing_center_z)),
    )
    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.4,
            lower=-0.38,
            upper=0.38,
        ),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "housing_to_crown",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crown,
        origin=Origin(xyz=(0.0, 0.0, housing_radius + 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    crown = object_model.get_part("crown")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_weight")
    crown_joint = object_model.get_articulation("housing_to_crown")

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="housing sits directly on the disc base",
    )
    ctx.expect_origin_distance(
        crown,
        housing,
        axes="xy",
        max_dist=0.001,
        name="crown stays centered on the top axis",
    )
    ctx.expect_origin_distance(
        weight,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="ring weight stays centered on the pendulum rod",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="xy",
        min_overlap=0.004,
        elem_a="ring_weight",
        elem_b="upper_rod",
        name="ring weight remains wrapped around the rod in plan",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: 0.055}):
        raised_weight_pos = ctx.part_world_position(weight)
        ctx.expect_origin_distance(
            weight,
            pendulum,
            axes="xy",
            max_dist=0.001,
            name="raised ring weight stays coaxial with the rod",
        )
    ctx.check(
        "ring weight slides upward along the rod",
        rest_weight_pos is not None
        and raised_weight_pos is not None
        and raised_weight_pos[2] > rest_weight_pos[2] + 0.04,
        details=f"rest={rest_weight_pos}, raised={raised_weight_pos}",
    )

    def _elem_center_x(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_tip_x = _elem_center_x("pendulum", "upper_rod")
    with ctx.pose({pendulum_joint: 0.30}):
        swung_tip_x = _elem_center_x("pendulum", "upper_rod")
    ctx.check(
        "positive pendulum motion swings the rod toward +x",
        rest_tip_x is not None and swung_tip_x is not None and swung_tip_x > rest_tip_x + 0.02,
        details=f"rest_tip_x={rest_tip_x}, swung_tip_x={swung_tip_x}",
    )

    with ctx.pose({crown_joint: 1.2}):
        ctx.expect_origin_distance(
            crown,
            housing,
            axes="xy",
            max_dist=0.001,
            name="crown rotates in place about the vertical axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
