from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_air_fork")

    anodized_black = model.material("anodized_black", rgba=(0.02, 0.022, 0.025, 1.0))
    satin_black = model.material("satin_black", rgba=(0.004, 0.004, 0.004, 1.0))
    magnesium_gray = model.material("magnesium_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    polished = model.material("polished_stanchion", rgba=(0.78, 0.80, 0.82, 1.0))
    crown_red = model.material("anodized_crown", rgba=(0.72, 0.08, 0.035, 1.0))
    frame_teal = model.material("frame_teal", rgba=(0.02, 0.22, 0.24, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.0, 0.0, 0.0, 1.0))
    blue = model.material("blue_air_cap", rgba=(0.02, 0.18, 0.9, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.011, 1.0))

    # A short hollow head-tube section is the fixed reference for steering.
    # The fork/steerer/bars rotate around its central axis.
    head_tube = model.part("head_tube")
    head_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.040, -0.105), (0.040, 0.105)],
        inner_profile=[(0.024, -0.105), (0.024, 0.105)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    head_tube.visual(
        mesh_from_geometry(head_shell, "head_tube_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=anodized_black,
        name="head_tube_shell",
    )
    for z, name in ((0.835, "upper_bearing_cup"), (0.625, "lower_bearing_cup")):
        head_tube.visual(
            mesh_from_geometry(TorusGeometry(radius=0.033, tube=0.004, radial_segments=20, tubular_segments=64), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_black,
            name=name,
        )
    head_tube.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, -0.032, 0.790), (0.0, -0.145, 0.835), (0.0, -0.260, 0.860)],
                radius=0.017,
                samples_per_segment=10,
                radial_segments=18,
            ),
            "top_tube_stub",
        ),
        material=frame_teal,
        name="top_tube_stub",
    )
    head_tube.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, -0.034, 0.660), (0.0, -0.140, 0.585), (0.0, -0.245, 0.505)],
                radius=0.023,
                samples_per_segment=10,
                radial_segments=18,
            ),
            "down_tube_stub",
        ),
        material=frame_teal,
        name="down_tube_stub",
    )

    steering = model.part("steering")
    steerer = LatheGeometry(
        [
            (0.0, -0.275),
            (0.020, -0.275),
            (0.020, -0.205),
            (0.017, -0.125),
            (0.014, 0.290),
            (0.0, 0.290),
        ],
        segments=72,
    )
    steering.visual(
        mesh_from_geometry(steerer, "tapered_steerer"),
        material=polished,
        name="tapered_steerer",
    )
    for local_z, name in ((0.105, "upper_bearing_race"), (-0.105, "lower_bearing_race")):
        steering.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.023, tube=0.0080, radial_segments=20, tubular_segments=64),
                name,
            ),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=polished,
            name=name,
        )

    # Wide double-crown-like lower crown and clamp bosses.
    steering.visual(
        Box((0.285, 0.120, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=crown_red,
        name="wide_crown",
    )
    for x, name in ((-0.095, "crown_boss_0"), (0.095, "crown_boss_1")):
        steering.visual(
            Cylinder(radius=0.040, length=0.086),
            origin=Origin(xyz=(x, 0.0, -0.235)),
            material=crown_red,
            name=name,
        )
    steering.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=crown_red,
        name="steerer_crown_boss",
    )

    for x, name in ((-0.095, "stanchion_0"), (0.095, "stanchion_1")):
        steering.visual(
            Cylinder(radius=0.0175, length=0.500),
            origin=Origin(xyz=(x, 0.0, -0.480)),
            material=polished,
            name=name,
        )
    steering.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(-0.095, 0.0, -0.188)),
        material=blue,
        name="air_valve_cap",
    )
    steering.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.095, 0.0, -0.190)),
        material=bolt_black,
        name="compression_cap",
    )

    # Threadless steerer stem and a wide flat handlebar move with the steering.
    steering.visual(
        Box((0.090, 0.070, 0.100)),
        origin=Origin(xyz=(0.0, 0.010, 0.185)),
        material=anodized_black,
        name="steerer_clamp",
    )
    for z in (0.155, 0.215):
        steering.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(xyz=(-0.050, 0.036, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"stem_bolt_{0 if z < 0.18 else 1}",
        )
    steering.visual(
        Box((0.055, 0.175, 0.035)),
        origin=Origin(xyz=(0.0, 0.095, 0.200)),
        material=anodized_black,
        name="stem_body",
    )
    steering.visual(
        Cylinder(radius=0.035, length=0.090),
        origin=Origin(xyz=(0.0, 0.185, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="bar_clamp",
    )
    steering.visual(
        Cylinder(radius=0.011, length=0.790),
        origin=Origin(xyz=(0.0, 0.185, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="flat_handlebar",
    )
    for x, name in ((-0.345, "grip_0"), (0.345, "grip_1")):
        steering.visual(
            Cylinder(radius=0.015, length=0.115),
            origin=Origin(xyz=(x, 0.185, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=name,
        )
    for x, name in ((-0.025, "bar_clamp_bolt_0"), (0.025, "bar_clamp_bolt_1")):
        steering.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(x, 0.223, 0.215), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_black,
            name=name,
        )

    steer_joint = model.articulation(
        "head_tube_to_steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.90, upper=0.90),
    )

    # The two lower legs are separate telescoping slider links.  The second joint
    # mimics the first so the fork lowers move together while still exposing a
    # prismatic joint on each leg as requested.
    lower_legs = []
    for idx, x in enumerate((-0.095, 0.095)):
        lower = model.part(f"lower_leg_{idx}")
        slider_shell = LatheGeometry.from_shell_profiles(
            outer_profile=[(0.027, -0.092), (0.027, 0.086)],
            inner_profile=[(0.0195, -0.092), (0.0195, 0.086)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        )
        lower.visual(
            mesh_from_geometry(slider_shell, f"slider_shell_{idx}"),
            material=magnesium_gray,
            name="slider_shell",
        )
        lower.visual(
            mesh_from_geometry(TorusGeometry(radius=0.0235, tube=0.0038, radial_segments=16, tubular_segments=48), f"wiper_seal_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, 0.089)),
            material=rubber,
            name="wiper_seal",
        )
        lower.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.0188, tube=0.0024, radial_segments=16, tubular_segments=48),
                f"upper_bushing_{idx}",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.055)),
            material=bolt_black,
            name="upper_bushing",
        )
        lower.visual(
            Cylinder(radius=0.025, length=0.315),
            origin=Origin(xyz=(0.0, 0.0, -0.247)),
            material=magnesium_gray,
            name="lower_casting",
        )
        lower.visual(
            Box((0.050, 0.020, 0.085)),
            origin=Origin(xyz=(0.0, -0.024, -0.405)),
            material=magnesium_gray,
            name="dropout_front",
        )
        lower.visual(
            Box((0.050, 0.020, 0.085)),
            origin=Origin(xyz=(0.0, 0.024, -0.405)),
            material=magnesium_gray,
            name="dropout_rear",
        )
        lower_legs.append(lower)

        model.articulation(
            f"stanchion_to_lower_{idx}",
            ArticulationType.PRISMATIC,
            parent=steering,
            child=lower,
            origin=Origin(xyz=(x, 0.0, -0.640)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.140),
            mimic=Mimic("stanchion_to_lower_0") if idx == 1 else None,
        )

    # One molded arch is attached to lower_leg_0 and reaches into the mirrored
    # leg.  The paired prismatic joints keep the opposite end aligned.
    lower_legs[0].visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, -0.038, -0.090),
                    (0.040, -0.058, -0.040),
                    (0.095, -0.064, -0.015),
                    (0.150, -0.058, -0.040),
                    (0.190, -0.038, -0.090),
                ],
                radius=0.017,
                samples_per_segment=14,
                radial_segments=18,
            ),
            "fork_arch",
        ),
        material=magnesium_gray,
        name="fork_arch",
    )

    # Keep the source steering articulation live in the object so tests can refer
    # to its semantic name without relying on construction order.
    steering.meta["primary_steering_joint"] = steer_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering")
    lower_0 = object_model.get_part("lower_leg_0")
    lower_1 = object_model.get_part("lower_leg_1")
    steer_joint = object_model.get_articulation("head_tube_to_steering")
    travel_joint = object_model.get_articulation("stanchion_to_lower_0")

    for cup, race in (
        ("upper_bearing_cup", "upper_bearing_race"),
        ("lower_bearing_cup", "lower_bearing_race"),
    ):
        ctx.allow_overlap(
            head_tube,
            steering,
            elem_a=cup,
            elem_b=race,
            reason="The steerer bearing race is intentionally captured in the headset cup.",
        )
        ctx.expect_overlap(
            head_tube,
            steering,
            axes="xyz",
            elem_a=cup,
            elem_b=race,
            min_overlap=0.002,
            name=f"{race} is seated in the headset cup",
        )
        ctx.allow_overlap(
            head_tube,
            steering,
            elem_a="head_tube_shell",
            elem_b=race,
            reason=(
                "The headset race is intentionally represented as a captured "
                "press-fit at the end of the hollow head-tube shell."
            ),
        )
        ctx.expect_overlap(
            head_tube,
            steering,
            axes="xyz",
            elem_a="head_tube_shell",
            elem_b=race,
            min_overlap=0.002,
            name=f"{race} is retained by the head-tube shell",
        )

    for lower, stanchion in ((lower_0, "stanchion_0"), (lower_1, "stanchion_1")):
        ctx.allow_overlap(
            steering,
            lower,
            elem_a=stanchion,
            elem_b="upper_bushing",
            reason=(
                "The fork lower contains a sliding bushing that intentionally "
                "captures the polished stanchion tube."
            ),
        )
        ctx.expect_overlap(
            steering,
            lower,
            axes="xyz",
            elem_a=stanchion,
            elem_b="upper_bushing",
            min_overlap=0.002,
            name=f"{stanchion} is captured by its lower-leg bushing",
        )

    ctx.allow_overlap(
        lower_0,
        lower_1,
        elem_a="fork_arch",
        elem_b="slider_shell",
        reason=(
            "The molded fork arch is authored on lower_leg_0 and intentionally "
            "nests into the mirrored lower-leg socket while the second leg mimics "
            "the same telescoping travel."
        ),
    )
    ctx.expect_overlap(
        lower_0,
        lower_1,
        axes="xyz",
        elem_a="fork_arch",
        elem_b="slider_shell",
        min_overlap=0.004,
        name="molded arch reaches the mirrored lower leg",
    )

    ctx.expect_origin_distance(
        lower_0,
        lower_1,
        axes="x",
        min_dist=0.180,
        max_dist=0.205,
        name="fork legs have mountain-bike spacing",
    )
    ctx.expect_within(
        steering,
        lower_0,
        axes="xy",
        inner_elem="stanchion_0",
        outer_elem="slider_shell",
        margin=0.002,
        name="stanchion 0 is centered in the hollow slider",
    )
    ctx.expect_within(
        steering,
        lower_1,
        axes="xy",
        inner_elem="stanchion_1",
        outer_elem="slider_shell",
        margin=0.002,
        name="stanchion 1 is centered in the hollow slider",
    )
    ctx.expect_overlap(
        steering,
        lower_0,
        axes="z",
        elem_a="stanchion_0",
        elem_b="slider_shell",
        min_overlap=0.085,
        name="collapsed leg 0 has retained insertion",
    )
    ctx.expect_overlap(
        steering,
        lower_1,
        axes="z",
        elem_a="stanchion_1",
        elem_b="slider_shell",
        min_overlap=0.085,
        name="collapsed leg 1 has retained insertion",
    )

    handlebar_aabb = ctx.part_element_world_aabb(steering, elem="flat_handlebar")
    ctx.check(
        "handlebar is wide and flat",
        handlebar_aabb is not None and (handlebar_aabb[1][0] - handlebar_aabb[0][0]) > 0.75,
        details=f"flat_handlebar_aabb={handlebar_aabb}",
    )

    lower_0_rest = ctx.part_world_position(lower_0)
    lower_1_rest = ctx.part_world_position(lower_1)
    with ctx.pose({travel_joint: 0.120}):
        lower_0_extended = ctx.part_world_position(lower_0)
        lower_1_extended = ctx.part_world_position(lower_1)
        ctx.expect_overlap(
            steering,
            lower_0,
            axes="z",
            elem_a="stanchion_0",
            elem_b="slider_shell",
            min_overlap=0.020,
            name="extended leg 0 remains inserted",
        )
        ctx.expect_overlap(
            steering,
            lower_1,
            axes="z",
            elem_a="stanchion_1",
            elem_b="slider_shell",
            min_overlap=0.020,
            name="extended leg 1 remains inserted",
        )
    ctx.check(
        "both lower legs extend downward together",
        lower_0_rest is not None
        and lower_1_rest is not None
        and lower_0_extended is not None
        and lower_1_extended is not None
        and lower_0_extended[2] < lower_0_rest[2] - 0.10
        and lower_1_extended[2] < lower_1_rest[2] - 0.10,
        details=f"rest0={lower_0_rest}, ext0={lower_0_extended}, rest1={lower_1_rest}, ext1={lower_1_extended}",
    )

    lower_0_center = ctx.part_world_position(lower_0)
    with ctx.pose({steer_joint: 0.55}):
        lower_0_turned = ctx.part_world_position(lower_0)
    ctx.check(
        "fork steers about the head tube axis",
        lower_0_center is not None
        and lower_0_turned is not None
        and abs(lower_0_turned[1] - lower_0_center[1]) > 0.045,
        details=f"center={lower_0_center}, turned={lower_0_turned}",
    )

    return ctx.report()


object_model = build_object_model()
