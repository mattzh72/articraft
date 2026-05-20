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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    def ring_mesh(name: str, x_pos: float, radius: float, tube_radius: float):
        points = [
            (
                x_pos,
                radius * math.cos(2.0 * math.pi * i / 24.0),
                radius * math.sin(2.0 * math.pi * i / 24.0),
            )
            for i in range(24)
        ]
        return mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=tube_radius,
                samples_per_segment=6,
                radial_segments=12,
                closed_spline=True,
                cap_ends=False,
            ),
            name,
        )

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.52, 0.53, 0.56, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.60, 0.63, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="foot_disc",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=warm_gray,
        name="top_plinth",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=warm_gray,
        name="mast_socket",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.1485)),
        material=silver,
        name="upright_post",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.060, 0.0, 0.013)),
        material=rubber,
        name="speed_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.23, 0.23, 0.24)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=warm_gray,
        name="tilt_socket",
    )
    bearing_module.visual(
        Box((0.018, 0.170, 0.012)),
        origin=Origin(xyz=(-0.014, 0.0, 0.016)),
        material=warm_gray,
        name="lower_yoke_bar",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        bearing_module.visual(
            Box((0.018, 0.014, 0.100)),
            origin=Origin(xyz=(-0.006, sign * 0.085, 0.060)),
            material=warm_gray,
            name=f"{side}_bracket",
        )
        bearing_module.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(
                xyz=(0.002, sign * 0.091, 0.090),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=silver,
            name=f"{side}_pivot_cap",
        )
    bearing_module.inertial = Inertial.from_geometry(
        Box((0.060, 0.190, 0.130)),
        mass=0.8,
        origin=Origin(xyz=(-0.006, 0.0, 0.060)),
    )

    head_module = model.part("head_module")
    head_module.visual(
        Cylinder(radius=0.050, length=0.090),
        origin=Origin(
            xyz=(0.030, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="motor_barrel",
    )
    head_module.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(
            xyz=(-0.020, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_gray,
        name="rear_cap",
    )
    head_module.visual(
        Cylinder(radius=0.010, length=0.156),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="tilt_axle",
    )
    head_module.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(
            xyz=(0.060, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_gray,
        name="guard_spider",
    )
    head_module.visual(
        ring_mesh("rear_guard_ring", 0.060, 0.128, 0.003),
        material=silver,
        name="rear_guard_ring",
    )
    head_module.visual(
        ring_mesh("front_guard_ring", 0.140, 0.128, 0.003),
        material=silver,
        name="front_guard_ring",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        head_module.visual(
            Cylinder(radius=0.0026, length=0.106),
            origin=Origin(
                xyz=(0.060, sign * 0.075, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=silver,
            name=f"{side}_rear_stay",
        )
        head_module.visual(
            Cylinder(radius=0.0026, length=0.084),
            origin=Origin(
                xyz=(0.100, sign * 0.128, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=silver,
            name=f"{side}_cage_rib",
        )
    for side, sign in (("upper", 1.0), ("lower", -1.0)):
        head_module.visual(
            Cylinder(radius=0.0026, length=0.106),
            origin=Origin(
                xyz=(0.060, 0.0, sign * 0.075),
            ),
            material=silver,
            name=f"{side}_rear_stay",
        )
        head_module.visual(
            Cylinder(radius=0.0026, length=0.084),
            origin=Origin(
                xyz=(0.100, 0.0, sign * 0.128),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=silver,
            name=f"{side}_cage_rib",
        )
    head_module.visual(
        Cylinder(radius=0.0024, length=0.258),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=silver,
        name="front_vertical_guard",
    )
    for band, z_pos in (("upper", 0.048), ("lower", -0.048)):
        head_module.visual(
            Cylinder(radius=0.0024, length=0.240),
            origin=Origin(
                xyz=(0.140, 0.0, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=silver,
            name=f"front_{band}_guard",
        )
    head_module.inertial = Inertial.from_geometry(
        Box((0.210, 0.270, 0.270)),
        mass=1.4,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_gray,
        name="hub",
    )
    rotor.visual(
        Box((0.120, 0.034, 0.004)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=blade_gray,
        name="blade_0",
    )
    rotor.visual(
        Box((0.120, 0.034, 0.004)),
        origin=Origin(xyz=(-0.027, 0.048, 0.0), rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
        material=blade_gray,
        name="blade_1",
    )
    rotor.visual(
        Box((0.120, 0.034, 0.004)),
        origin=Origin(xyz=(-0.027, -0.048, 0.0), rpy=(0.0, 0.0, -2.0 * math.pi / 3.0)),
        material=blade_gray,
        name="blade_2",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.030),
        mass=0.18,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 0.241)),
    )
    model.articulation(
        "bearing_to_head",
        ArticulationType.REVOLUTE,
        parent=bearing_module,
        child=head_module,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head_module,
        child=rotor,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=24.0),
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

    base = object_model.get_part("base")
    bearing_module = object_model.get_part("bearing_module")
    head_module = object_model.get_part("head_module")
    rotor = object_model.get_part("rotor")
    tilt_joint = object_model.get_articulation("bearing_to_head")

    ctx.expect_contact(
        base,
        bearing_module,
        contact_tol=0.001,
        name="bearing module is seated on the base mast",
    )
    ctx.expect_gap(
        head_module,
        base,
        axis="z",
        positive_elem="motor_barrel",
        negative_elem="top_plinth",
        min_gap=0.20,
        name="motor housing stays well above the base plinth",
    )
    ctx.expect_origin_distance(
        rotor,
        head_module,
        axes="yz",
        max_dist=0.001,
        name="rotor stays centered on the head axle",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt_joint: math.radians(24.0)}):
        raised_pos = ctx.part_world_position(rotor)
    ctx.check(
        "tilt joint raises the fan head",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.03,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
