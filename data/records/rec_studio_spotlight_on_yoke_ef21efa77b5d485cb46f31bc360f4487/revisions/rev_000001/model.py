from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
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


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_yoke_studio_spotlight")

    dark_paint = _mat("matte_graphite_powder_coat", (0.035, 0.039, 0.040, 1.0))
    black_paint = _mat("satin_black_painted_metal", (0.005, 0.006, 0.006, 1.0))
    rubber = _mat("black_molded_rubber", (0.010, 0.009, 0.008, 1.0))
    bare_steel = _mat("brushed_stainless_fasteners", (0.62, 0.60, 0.56, 1.0))
    warm_glass = _mat("slightly_warm_glass", (0.95, 0.82, 0.42, 0.34))
    reflector = _mat("pebbled_aluminum_reflector", (0.78, 0.73, 0.64, 1.0))
    amber = _mat("yellow_utility_marking", (0.95, 0.66, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.52, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_paint,
        name="weighted_base_plate",
    )
    base.visual(
        Box((0.58, 0.39, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=black_paint,
        name="raised_top_pad",
    )
    base.visual(
        Cylinder(radius=0.185, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=black_paint,
        name="fixed_bearing_housing",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.156, tube=0.009, radial_segments=24, tubular_segments=48),
            "exposed_bearing_race",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=bare_steel,
        name="exposed_bearing_race",
    )
    for i, (x, y) in enumerate(
        (
            (-0.295, -0.205),
            (0.295, -0.205),
            (-0.295, 0.205),
            (0.295, 0.205),
        )
    ):
        base.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(xyz=(x, y, 0.009)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.081)),
            material=bare_steel,
            name=f"base_bolt_{i}",
        )

    pan_lock_knob = model.part("pan_lock_knob")
    pan_lock_knob.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0), rpy=(0.0, -pi / 2.0, 0.0)),
        material=bare_steel,
        name="threaded_stem",
    )
    pan_lock_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.070,
                0.034,
                body_style="lobed",
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=12, depth=0.002),
                bore=KnobBore(style="round", diameter=0.010),
            ),
            "pan_lock_knob_cap",
        ),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, -pi / 2.0, 0.0)),
        material=black_paint,
        name="lobed_cap",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.166, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_paint,
        name="rotating_turntable",
    )
    yoke.visual(
        Cylinder(radius=0.085, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_paint,
        name="vertical_spindle",
    )
    yoke.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.073, tube=0.014, radial_segments=24, tubular_segments=48),
            "upper_bearing_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=bare_steel,
        name="upper_bearing_collar",
    )
    yoke.visual(
        Box((0.170, 0.440, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=black_paint,
        name="lower_yoke_bridge",
    )
    for side, y, cheek_name in (
        ("side_0", -0.220, "side_0_thick_cheek"),
        ("side_1", 0.220, "side_1_thick_cheek"),
    ):
        yoke.visual(
            Box((0.108, 0.064, 0.430)),
            origin=Origin(xyz=(0.0, y, 0.415)),
            material=dark_paint,
            name=cheek_name,
        )
        yoke.visual(
            Box((0.158, 0.070, 0.052)),
            origin=Origin(xyz=(0.0, y, 0.242)),
            material=black_paint,
            name=f"{side}_gusset_foot",
        )
        yoke.visual(
            Box((0.028, 0.073, 0.250)),
            origin=Origin(xyz=(-0.055, y, 0.345)),
            material=black_paint,
            name=f"{side}_rear_rib",
        )
        yoke.visual(
            Box((0.028, 0.073, 0.250)),
            origin=Origin(xyz=(0.055, y, 0.345)),
            material=black_paint,
            name=f"{side}_front_rib",
        )
        yoke.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.044, tube=0.011, radial_segments=24, tubular_segments=48),
                f"{side}_inner_bearing_ring",
            ),
            origin=Origin(xyz=(0.0, y * 0.875, 0.430), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"{side}_inner_bearing_ring",
        )
        yoke.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.052, tube=0.012, radial_segments=24, tubular_segments=48),
                f"{side}_outer_bearing_ring",
            ),
            origin=Origin(xyz=(0.0, y * 1.145, 0.430), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"{side}_outer_bearing_ring",
        )
        for j, z in enumerate((0.280, 0.545)):
            yoke.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(0.0, y * 1.155, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=bare_steel,
                name=f"{side}_cheek_bolt_{j}",
            )

    tilt_lock_knob = model.part("tilt_lock_knob")
    tilt_lock_knob.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="clamp_stem",
    )
    tilt_lock_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.080,
                0.040,
                body_style="lobed",
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=10, depth=0.0022),
                bore=KnobBore(style="round", diameter=0.011),
            ),
            "tilt_lock_knob_cap",
        ),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="lobed_cap",
    )

    spotlight = model.part("spotlight")
    can_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.116, -0.195),
            (0.132, -0.165),
            (0.150, 0.095),
            (0.166, 0.215),
            (0.174, 0.245),
        ],
        inner_profile=[
            (0.094, -0.178),
            (0.112, -0.135),
            (0.132, 0.120),
            (0.145, 0.205),
            (0.150, 0.228),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    spotlight.visual(
        mesh_from_geometry(can_shell, "spotlight_hollow_can"),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_paint,
        name="hollow_can",
    )
    reflector_bowl = LatheGeometry(
        [
            (0.030, -0.115),
            (0.070, -0.070),
            (0.112, 0.010),
            (0.136, 0.105),
            (0.145, 0.176),
            (0.010, 0.176),
            (0.010, -0.115),
        ],
        segments=72,
        closed=True,
    )
    spotlight.visual(
        mesh_from_geometry(reflector_bowl, "deep_reflector_bowl"),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=reflector,
        name="reflector_bowl",
    )
    spotlight.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=amber,
        name="bulb_dome",
    )
    spotlight.visual(
        Cylinder(radius=0.151, length=0.010),
        origin=Origin(xyz=(0.292, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_glass,
        name="tempered_lens",
    )
    spotlight.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.155, tube=0.014, radial_segments=24, tubular_segments=72),
            "front_protective_bezel",
        ),
        origin=Origin(xyz=(0.302, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_paint,
        name="front_protective_bezel",
    )
    for i, z in enumerate((-0.070, 0.0, 0.070)):
        spotlight.visual(
            Box((0.012, 0.285, 0.010)),
            origin=Origin(xyz=(0.314, 0.0, z)),
            material=dark_paint,
            name=f"horizontal_lens_guard_{i}",
        )
    for i, y in enumerate((-0.070, 0.0, 0.070)):
        spotlight.visual(
            Box((0.012, 0.010, 0.285)),
            origin=Origin(xyz=(0.318, y, 0.0)),
            material=dark_paint,
            name=f"vertical_lens_guard_{i}",
        )
    spotlight.visual(
        Cylinder(radius=0.104, length=0.020),
        origin=Origin(xyz=(-0.130, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_paint,
        name="rear_service_cap",
    )
    spotlight.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(-0.158, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bare_steel,
        name="rear_cable_gland",
    )
    for i, y in enumerate((-0.070, -0.035, 0.035, 0.070)):
        spotlight.visual(
            Box((0.090, 0.008, 0.020)),
            origin=Origin(xyz=(-0.090, y, 0.118)),
            material=bare_steel,
            name=f"top_heat_vent_{i}",
        )
    for side, y in (("side_0", -0.168), ("side_1", 0.168)):
        spotlight.visual(
            Cylinder(radius=0.026, length=0.035),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"{side}_trunnion_stub",
        )
        spotlight.visual(
            Cylinder(radius=0.042, length=0.014),
            origin=Origin(xyz=(0.0, y * 0.865, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_paint,
            name=f"{side}_can_boss",
        )
        for j, x in enumerate((-0.030, 0.030)):
            spotlight.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(xyz=(x, y * 0.860, 0.035), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=bare_steel,
                name=f"{side}_boss_bolt_{j}",
            )

    model.articulation(
        "base_to_pan_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pan_lock_knob,
        origin=Origin(xyz=(-0.185, 0.0, 0.140)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )
    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "yoke_to_spotlight",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=spotlight,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.9, lower=-0.85, upper=1.05),
    )
    model.articulation(
        "yoke_to_tilt_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=tilt_lock_knob,
        origin=Origin(xyz=(0.0, -0.250, 0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    spotlight = object_model.get_part("spotlight")
    pan_knob = object_model.get_part("pan_lock_knob")
    tilt_knob = object_model.get_part("tilt_lock_knob")
    pan = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_spotlight")

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        positive_elem="rotating_turntable",
        negative_elem="exposed_bearing_race",
        max_gap=0.001,
        max_penetration=0.001,
        name="turntable seats on bearing race",
    )
    ctx.expect_within(
        spotlight,
        yoke,
        axes="y",
        inner_elem="hollow_can",
        margin=0.001,
        name="spotlight can sits between yoke cheeks",
    )
    ctx.expect_gap(
        spotlight,
        yoke,
        axis="z",
        positive_elem="hollow_can",
        negative_elem="lower_yoke_bridge",
        min_gap=0.006,
        max_gap=0.020,
        name="tilting can clears lower yoke bridge",
    )
    ctx.expect_gap(
        base,
        pan_knob,
        axis="x",
        positive_elem="fixed_bearing_housing",
        negative_elem="threaded_stem",
        max_gap=0.001,
        max_penetration=0.001,
        name="pan lock stem reaches bearing housing",
    )
    ctx.expect_gap(
        yoke,
        tilt_knob,
        axis="y",
        positive_elem="side_0_thick_cheek",
        negative_elem="clamp_stem",
        max_gap=0.001,
        max_penetration=0.003,
        name="tilt lock stem seats in yoke cheek",
    )

    rest_front = ctx.part_element_world_aabb(spotlight, elem="front_protective_bezel")
    with ctx.pose({tilt: -0.60}):
        raised_front = ctx.part_element_world_aabb(spotlight, elem="front_protective_bezel")
    with ctx.pose({tilt: 0.80}):
        dipped_front = ctx.part_element_world_aabb(spotlight, elem="front_protective_bezel")
    ctx.check(
        "tilt joint aims can up and down",
        rest_front is not None
        and raised_front is not None
        and dipped_front is not None
        and raised_front[1][2] > rest_front[1][2] + 0.12
        and dipped_front[0][2] < rest_front[0][2] - 0.12,
        details=f"rest={rest_front}, raised={raised_front}, dipped={dipped_front}",
    )

    with ctx.pose({pan: pi / 2.0}):
        panned_front = ctx.part_element_world_aabb(spotlight, elem="front_protective_bezel")
    ctx.check(
        "pan joint swings aiming assembly around vertical spindle",
        rest_front is not None
        and panned_front is not None
        and (panned_front[0][1] + panned_front[1][1]) / 2.0
        > (rest_front[0][1] + rest_front[1][1]) / 2.0 + 0.20,
        details=f"rest={rest_front}, panned={panned_front}",
    )

    return ctx.report()


object_model = build_object_model()
