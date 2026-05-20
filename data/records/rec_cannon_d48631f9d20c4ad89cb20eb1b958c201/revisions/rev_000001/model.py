from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


AXLE_Z = 0.72
TRUNNION_X = 0.10
TRUNNION_Z = 1.08


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _barrel_body_mesh():
    # 3-inch ordnance rifles had a plain, wrought-iron tube with a slight muzzle
    # swell, reinforced breech, and rounded cascabel knob.  The profile is
    # authored about local Z and rotated onto the cannon's +X bore axis.
    profile = [
        (0.000, -0.760),
        (0.040, -0.755),
        (0.065, -0.720),
        (0.072, -0.675),
        (0.055, -0.635),
        (0.104, -0.625),
        (0.126, -0.580),
        (0.154, -0.500),
        (0.166, -0.300),
        (0.156, -0.060),
        (0.138, 0.260),
        (0.118, 0.700),
        (0.102, 1.060),
        (0.112, 1.175),
        (0.113, 1.300),
        (0.000, 1.300),
    ]
    return LatheGeometry(profile, segments=88)


def _trail_beam_mesh(y_sign: float):
    return sweep_profile_along_spline(
        [
            (-0.54, 0.30 * y_sign, 0.78),
            (-0.88, 0.26 * y_sign, 0.66),
            (-1.45, 0.18 * y_sign, 0.48),
            (-2.20, 0.075 * y_sign, 0.285),
            (-2.48, 0.045 * y_sign, 0.260),
        ],
        profile=rounded_rect_profile(0.115, 0.145, radius=0.018, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )


def _limber_hook_mesh():
    # A short iron shank ending in a vertical lunette/hook loop on the trail
    # plate.  The path is in the XZ plane so it reads as a rear hitch feature.
    return tube_from_spline_points(
        [
            (-2.505, 0.0, 0.305),
            (-2.600, 0.0, 0.305),
            (-2.675, 0.0, 0.360),
            (-2.735, 0.0, 0.305),
            (-2.675, 0.0, 0.250),
            (-2.600, 0.0, 0.285),
        ],
        radius=0.020,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_3_inch_ordnance_rifle")

    oak = model.material("oiled_oak", rgba=(0.55, 0.34, 0.16, 1.0))
    oak_dark = model.material("dark_oak_endgrain", rgba=(0.36, 0.20, 0.09, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.12, 0.13, 0.13, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.035, 0.038, 0.040, 1.0))
    bore_black = model.material("bore_black", rgba=(0.0, 0.0, 0.0, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.34, 0.12, 0.34)),
        origin=Origin(xyz=(0.10, 0.34, 0.858)),
        material=oak,
        name="cheek_0",
    )
    carriage.visual(
        Box((1.34, 0.12, 0.34)),
        origin=Origin(xyz=(0.10, -0.34, 0.858)),
        material=oak,
        name="cheek_1",
    )
    carriage.visual(
        Box((0.40, 0.86, 0.18)),
        origin=Origin(xyz=(0.00, 0.0, AXLE_Z)),
        material=oak,
        name="axle_bed",
    )
    carriage.visual(
        Cylinder(radius=0.045, length=1.72),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="axle",
    )
    carriage.visual(
        Box((0.26, 0.84, 0.10)),
        origin=Origin(xyz=(-0.43, 0.0, 0.74)),
        material=oak,
        name="trail_transom",
    )
    carriage.visual(_mesh("trail_beam_0", _trail_beam_mesh(1.0)), material=oak, name="trail_beam_0")
    carriage.visual(_mesh("trail_beam_1", _trail_beam_mesh(-1.0)), material=oak, name="trail_beam_1")
    carriage.visual(
        Box((0.22, 0.48, 0.12)),
        origin=Origin(xyz=(-1.25, 0.0, 0.54)),
        material=oak_dark,
        name="middle_crossbar",
    )
    carriage.visual(
        Box((0.26, 0.23, 0.11)),
        origin=Origin(xyz=(-2.42, 0.0, 0.265)),
        material=oak_dark,
        name="trail_end_block",
    )
    carriage.visual(
        Box((0.055, 0.24, 0.16)),
        origin=Origin(xyz=(-2.515, 0.0, 0.305)),
        material=wrought_iron,
        name="pintle_hook_plate",
    )
    carriage.visual(_mesh("limber_pintle_hook", _limber_hook_mesh()), material=wrought_iron, name="limber_hook")
    carriage.visual(
        Box((1.20, 0.020, 0.060)),
        origin=Origin(xyz=(0.08, 0.405, 0.995)),
        material=wrought_iron,
        name="cheek_strap_0",
    )
    carriage.visual(
        Box((1.20, 0.020, 0.060)),
        origin=Origin(xyz=(0.08, -0.405, 0.995)),
        material=wrought_iron,
        name="cheek_strap_1",
    )
    carriage.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(TRUNNION_X, 0.34, TRUNNION_Z - 0.112)),
        material=wrought_iron,
        name="trunnion_saddle_0",
    )
    carriage.visual(
        _mesh(
            "trunnion_bearing_0",
            TorusGeometry(radius=0.076, tube=0.018, radial_segments=18, tubular_segments=56),
        ),
        origin=Origin(xyz=(TRUNNION_X, 0.34, TRUNNION_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="trunnion_bearing_0",
    )
    carriage.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(TRUNNION_X, -0.34, TRUNNION_Z - 0.112)),
        material=wrought_iron,
        name="trunnion_saddle_1",
    )
    carriage.visual(
        _mesh(
            "trunnion_bearing_1",
            TorusGeometry(radius=0.076, tube=0.018, radial_segments=18, tubular_segments=56),
        ),
        origin=Origin(xyz=(TRUNNION_X, -0.34, TRUNNION_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="trunnion_bearing_1",
    )
    carriage.visual(
        Box((0.22, 0.22, 0.065)),
        origin=Origin(xyz=(-0.45, 0.0, 0.681)),
        material=wrought_iron,
        name="elevating_screw_foot",
    )
    carriage.visual(
        Cylinder(radius=0.024, length=0.20),
        origin=Origin(xyz=(-0.45, 0.0, 0.814)),
        material=wrought_iron,
        name="elevating_screw",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((3.25, 1.72, 1.15)),
        mass=520.0,
        origin=Origin(xyz=(-0.75, 0.0, 0.58)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _mesh("barrel_body", _barrel_body_mesh()),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wrought_iron,
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.052, length=0.96),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="trunnion_bar",
    )
    barrel.visual(
        Cylinder(radius=0.039, length=0.014),
        origin=Origin(xyz=(1.307, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore_black,
        name="muzzle_bore",
    )
    barrel.visual(
        Cylinder(radius=0.086, length=0.035),
        origin=Origin(xyz=(-0.665, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="cascabel_button",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=2.05),
        mass=370.0,
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    wheel_body = _mesh(
        "iron_spoked_wheel_body",
        WheelGeometry(
            0.655,
            0.120,
            rim=WheelRim(inner_radius=0.520, flange_height=0.014, flange_thickness=0.006),
            hub=WheelHub(radius=0.105, width=0.230, cap_style="domed"),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=14, thickness=0.010, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.120),
        ),
    )
    iron_tire = _mesh(
        "artillery_iron_tire",
        TireGeometry(
            0.720,
            0.125,
            inner_radius=0.660,
            sidewall=TireSidewall(style="square", bulge=0.01),
            shoulder=TireShoulder(width=0.010, radius=0.003),
        ),
    )

    wheel_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    for index in range(2):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(wheel_body, origin=wheel_origin, material=wrought_iron, name="spoked_wheel")
        wheel.visual(iron_tire, origin=wheel_origin, material=black_iron, name="iron_tire")
        wheel.visual(
            Cylinder(radius=0.070, length=0.160),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wrought_iron,
            name="hub_bushing",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.72, length=0.15),
            mass=92.0,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "barrel_trunnion",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.35, lower=-0.10, upper=0.35),
    )
    model.articulation(
        "wheel_axle_0",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="wheel_0",
        origin=Origin(xyz=(0.0, 0.72, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=12.0),
    )
    model.articulation(
        "wheel_axle_1",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="wheel_1",
        origin=Origin(xyz=(0.0, -0.72, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    trunnion = object_model.get_articulation("barrel_trunnion")
    axle_0 = object_model.get_articulation("wheel_axle_0")
    axle_1 = object_model.get_articulation("wheel_axle_1")

    ctx.allow_overlap(
        "carriage",
        "wheel_0",
        elem_a="axle",
        elem_b="hub_bushing",
        reason="The wheel hub bushing is intentionally captured around the simplified axle shaft.",
    )
    ctx.allow_overlap(
        "carriage",
        "wheel_1",
        elem_a="axle",
        elem_b="hub_bushing",
        reason="The wheel hub bushing is intentionally captured around the simplified axle shaft.",
    )
    ctx.allow_overlap(
        "carriage",
        "wheel_1",
        elem_a="axle",
        elem_b="spoked_wheel",
        reason="The axle passes through the simplified iron nave/bore region of the wheel.",
    )

    ctx.check("major_parts_present", all(p is not None for p in (carriage, barrel, wheel_0, wheel_1)))
    ctx.check(
        "trunnion_joint_is_limited_revolute",
        trunnion is not None
        and trunnion.articulation_type == ArticulationType.REVOLUTE
        and trunnion.motion_limits is not None
        and trunnion.motion_limits.lower < 0.0
        and trunnion.motion_limits.upper > 0.25,
    )
    ctx.check(
        "wheel_axles_are_continuous",
        axle_0 is not None
        and axle_1 is not None
        and axle_0.articulation_type == ArticulationType.CONTINUOUS
        and axle_1.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in axle_0.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in axle_1.axis) == (0.0, 1.0, 0.0),
    )
    ctx.expect_overlap(
        "barrel",
        "carriage",
        axes="y",
        elem_a="trunnion_bar",
        elem_b="trunnion_bearing_0",
        min_overlap=0.020,
        name="trunnion engages cheek bearing",
    )
    ctx.expect_overlap(
        "wheel_0",
        "carriage",
        axes="y",
        elem_a="spoked_wheel",
        elem_b="axle",
        min_overlap=0.050,
        name="wheel_0 rides on axle line",
    )
    ctx.expect_overlap(
        "wheel_0",
        "carriage",
        axes="y",
        elem_a="hub_bushing",
        elem_b="axle",
        min_overlap=0.060,
        name="wheel_0 bushing captures axle",
    )
    ctx.expect_overlap(
        "wheel_1",
        "carriage",
        axes="y",
        elem_a="spoked_wheel",
        elem_b="axle",
        min_overlap=0.050,
        name="wheel_1 rides on axle line",
    )
    ctx.expect_overlap(
        "wheel_1",
        "carriage",
        axes="y",
        elem_a="hub_bushing",
        elem_b="axle",
        min_overlap=0.060,
        name="wheel_1 bushing captures axle",
    )

    rest_aabb = ctx.part_element_world_aabb("barrel", elem="barrel_body")
    if trunnion is not None:
        with ctx.pose({trunnion: 0.35}):
            raised_aabb = ctx.part_element_world_aabb("barrel", elem="barrel_body")
        ctx.check(
            "positive_trunnion_elevates_muzzle",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.07,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
