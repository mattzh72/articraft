from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube(name: str, points, *, radius: float, samples: int = 16, radial: int = 18):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial,
        ),
    )


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    center_z: float,
    radius: float,
):
    return [(x, y, center_z + z) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_stationary_bike")

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    housing_enamel = model.material("housing_enamel", rgba=(0.25, 0.31, 0.25, 1.0))
    saddle_leather = model.material("saddle_leather", rgba=(0.39, 0.25, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.32, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bike_body = model.part("bike_body")
    bike_body.inertial = Inertial.from_geometry(
        Box((1.28, 0.58, 1.15)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    # Grounding and main frame.
    bike_body.visual(
        Box((0.10, 0.56, 0.06)),
        origin=Origin(xyz=(-0.43, 0.0, 0.03)),
        material=frame_paint,
        name="rear_stabilizer",
    )
    bike_body.visual(
        Box((0.10, 0.50, 0.06)),
        origin=Origin(xyz=(0.47, 0.0, 0.03)),
        material=frame_paint,
        name="front_stabilizer",
    )
    for name, x, y in [
        ("rear_left_foot", -0.43, 0.21),
        ("rear_right_foot", -0.43, -0.21),
        ("front_left_foot", 0.47, 0.18),
        ("front_right_foot", 0.47, -0.18),
    ]:
        bike_body.visual(
            Cylinder(radius=0.031, length=0.02),
            origin=Origin(xyz=(x, y, 0.01)),
            material=rubber,
            name=name,
        )

    bike_body.visual(
        _tube(
            "lower_spine_tube",
            [(-0.38, 0.0, 0.08), (-0.25, 0.0, 0.11), (-0.10, 0.0, 0.17), (-0.02, 0.0, 0.21)],
            radius=0.029,
        ),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=frame_paint,
        name="lower_spine",
    )
    bike_body.visual(
        _tube(
            "front_tie_tube",
            [(0.02, 0.0, 0.21), (0.16, 0.0, 0.18), (0.30, 0.0, 0.12), (0.43, 0.0, 0.08)],
            radius=0.025,
        ),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=frame_paint,
        name="front_tie",
    )
    bike_body.visual(
        _tube(
            "rear_mast_tube",
            [(-0.37, 0.0, 0.08), (-0.30, 0.0, 0.36), (-0.24, 0.0, 0.63), (-0.20, 0.0, 0.78)],
            radius=0.027,
        ),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=frame_paint,
        name="rear_mast",
    )
    bike_body.visual(
        _tube(
            "top_tube",
            [(-0.08, 0.0, 0.50), (-0.13, 0.0, 0.66), (-0.19, 0.0, 0.81)],
            radius=0.032,
        ),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=frame_paint,
        name="top_tube",
    )
    bike_body.visual(
        _tube(
            "front_mast_tube",
            [(0.12, 0.0, 0.49), (0.18, 0.0, 0.71), (0.26, 0.0, 0.92)],
            radius=0.030,
        ),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=frame_paint,
        name="front_mast",
    )
    bike_body.visual(
        _tube(
            "front_brace_tube",
            [(0.43, 0.0, 0.08), (0.35, 0.0, 0.36), (0.26, 0.0, 0.66)],
            radius=0.027,
        ),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=frame_paint,
        name="front_brace",
    )
    bike_body.visual(
        Box((0.08, 0.07, 0.08)),
        origin=Origin(xyz=(0.26, 0.070, 0.54)),
        material=frame_paint,
        name="front_brace_gusset",
    )

    bike_body.visual(
        Box((0.18, 0.07, 0.12)),
        origin=Origin(xyz=(-0.10, 0.070, 0.46)),
        material=frame_paint,
        name="rear_reinforcement",
    )
    bike_body.visual(
        Box((0.18, 0.07, 0.12)),
        origin=Origin(xyz=(0.16, 0.070, 0.49)),
        material=frame_paint,
        name="front_reinforcement",
    )
    bike_body.visual(
        Box((0.18, 0.07, 0.12)),
        origin=Origin(xyz=(-0.10, -0.070, 0.21)),
        material=frame_paint,
        name="lower_rear_reinforcement",
    )
    bike_body.visual(
        Box((0.18, 0.07, 0.12)),
        origin=Origin(xyz=(0.16, -0.070, 0.20)),
        material=frame_paint,
        name="lower_front_reinforcement",
    )
    bike_body.visual(
        Box((0.08, 0.14, 0.06)),
        origin=Origin(xyz=(-0.38, 0.0, 0.065)),
        material=frame_paint,
        name="rear_base_riser",
    )
    bike_body.visual(
        Box((0.062, 0.058, 0.14)),
        origin=Origin(xyz=(-0.071, 0.125, 0.23)),
        material=dark_steel,
        name="left_bearing_bridge",
    )
    bike_body.visual(
        Box((0.062, 0.058, 0.14)),
        origin=Origin(xyz=(-0.071, -0.125, 0.23)),
        material=dark_steel,
        name="right_bearing_bridge",
    )
    bike_body.visual(
        Box((0.14, 0.028, 0.02)),
        origin=Origin(xyz=(-0.11, 0.138, 0.19)),
        material=steel,
        name="left_housing_adapter_plate",
    )
    bike_body.visual(
        Box((0.14, 0.028, 0.02)),
        origin=Origin(xyz=(-0.11, -0.138, 0.19)),
        material=steel,
        name="right_housing_adapter_plate",
    )
    bike_body.visual(
        Box((0.09, 0.018, 0.14)),
        origin=Origin(xyz=(-0.12, 0.128, 0.25)),
        material=steel,
        name="left_housing_adapter",
    )
    bike_body.visual(
        Box((0.09, 0.018, 0.14)),
        origin=Origin(xyz=(-0.12, -0.128, 0.25)),
        material=steel,
        name="right_housing_adapter",
    )
    for name, y in [("left_adapter_bolt", 0.139), ("right_adapter_bolt", -0.139)]:
        bike_body.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(-0.11, y, 0.28), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    bike_body.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.089, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing_carrier",
    )
    bike_body.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, -0.089, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing_carrier",
    )
    bike_body.visual(
        Box((0.044, 0.022, 0.028)),
        origin=Origin(xyz=(-0.061, 0.096, 0.304)),
        material=dark_steel,
        name="left_carrier_rib",
    )
    bike_body.visual(
        Box((0.044, 0.022, 0.028)),
        origin=Origin(xyz=(-0.061, -0.096, 0.304)),
        material=dark_steel,
        name="right_carrier_rib",
    )

    # Flywheel housing and legacy service details.
    housing_shell = LatheGeometry.from_shell_profiles(
        [
            (0.11, -0.082),
            (0.17, -0.082),
            (0.22, -0.055),
            (0.252, -0.015),
            (0.252, 0.015),
            (0.22, 0.055),
            (0.17, 0.082),
            (0.11, 0.082),
        ],
        [
            (0.078, -0.070),
            (0.16, -0.070),
            (0.20, -0.045),
            (0.225, -0.012),
            (0.225, 0.012),
            (0.20, 0.045),
            (0.16, 0.070),
            (0.078, 0.070),
        ],
        segments=72,
    ).rotate_x(pi / 2.0)
    bike_body.visual(
        _mesh("housing_shell", housing_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=housing_enamel,
        name="housing_shell",
    )
    bike_body.visual(
        Box((0.34, 0.12, 0.03)),
        origin=Origin(xyz=(-0.03, 0.0, 0.08)),
        material=housing_enamel,
        name="housing_keel",
    )
    bike_body.visual(
        Box((0.12, 0.16, 0.04)),
        origin=Origin(xyz=(-0.02, 0.0, 0.61)),
        material=steel,
        name="resistance_bridge",
    )
    bike_body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(-0.02, 0.0, 0.632)),
        material=dark_steel,
        name="resistance_boss",
    )

    hatch_mesh = ExtrudeGeometry(rounded_rect_profile(0.12, 0.18, 0.016), 0.008).rotate_x(pi / 2.0)
    bike_body.visual(
        _mesh("left_service_hatch", hatch_mesh),
        origin=Origin(xyz=(-0.135, 0.086, 0.34)),
        material=steel,
        name="left_service_hatch",
    )
    bike_body.visual(
        _mesh("right_service_hatch", hatch_mesh),
        origin=Origin(xyz=(-0.135, -0.086, 0.34)),
        material=steel,
        name="right_service_hatch",
    )
    for name, x, y, z in [
        ("left_hatch_bolt_a", -0.175, 0.092, 0.28),
        ("left_hatch_bolt_b", -0.095, 0.092, 0.28),
        ("left_hatch_bolt_c", -0.175, 0.092, 0.40),
        ("left_hatch_bolt_d", -0.095, 0.092, 0.40),
        ("right_hatch_bolt_a", -0.175, -0.092, 0.28),
        ("right_hatch_bolt_b", -0.095, -0.092, 0.28),
        ("right_hatch_bolt_c", -0.175, -0.092, 0.40),
        ("right_hatch_bolt_d", -0.095, -0.092, 0.40),
    ]:
        bike_body.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    # Saddle post and clamp.
    bike_body.visual(
        Cylinder(radius=0.047, length=0.085),
        origin=Origin(xyz=(-0.20, 0.0, 0.7975)),
        material=dark_steel,
        name="seat_clamp_collar",
    )
    bike_body.visual(
        Box((0.06, 0.026, 0.04)),
        origin=Origin(xyz=(-0.20, 0.040, 0.82)),
        material=dark_steel,
        name="seat_clamp_ear_left",
    )
    bike_body.visual(
        Box((0.06, 0.026, 0.04)),
        origin=Origin(xyz=(-0.20, -0.040, 0.82)),
        material=dark_steel,
        name="seat_clamp_ear_right",
    )
    bike_body.visual(
        Cylinder(radius=0.006, length=0.092),
        origin=Origin(xyz=(-0.20, 0.0, 0.82), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="seat_clamp_bolt",
    )
    bike_body.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.20, 0.055, 0.82), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="seat_adjust_knob",
    )
    bike_body.visual(
        Cylinder(radius=0.025, length=0.33),
        origin=Origin(xyz=(-0.20, 0.0, 0.922)),
        material=steel,
        name="seat_post",
    )
    bike_body.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(-0.20, 0.0, 0.842)),
        material=steel,
        name="seat_post_stop",
    )
    bike_body.visual(
        Box((0.09, 0.06, 0.03)),
        origin=Origin(xyz=(-0.20, 0.0, 1.085)),
        material=dark_steel,
        name="saddle_yoke",
    )
    bike_body.visual(
        _tube(
            "left_saddle_rail",
            [(-0.22, 0.030, 1.09), (-0.16, 0.030, 1.10), (-0.08, 0.030, 1.09), (-0.03, 0.030, 1.06)],
            radius=0.007,
            samples=12,
            radial=14,
        ),
        material=steel,
        name="left_saddle_rail",
    )
    bike_body.visual(
        _tube(
            "right_saddle_rail",
            [(-0.22, -0.030, 1.09), (-0.16, -0.030, 1.10), (-0.08, -0.030, 1.09), (-0.03, -0.030, 1.06)],
            radius=0.007,
            samples=12,
            radial=14,
        ),
        material=steel,
        name="right_saddle_rail",
    )
    bike_body.visual(
        Box((0.15, 0.09, 0.02)),
        origin=Origin(xyz=(-0.13, 0.0, 1.075)),
        material=dark_steel,
        name="saddle_underpan",
    )
    saddle_shell = section_loft(
        [
            _yz_section(-0.28, width=0.10, height=0.04, center_z=1.102, radius=0.012),
            _yz_section(-0.19, width=0.16, height=0.06, center_z=1.118, radius=0.018),
            _yz_section(-0.09, width=0.19, height=0.068, center_z=1.120, radius=0.020),
            _yz_section(-0.01, width=0.15, height=0.05, center_z=1.104, radius=0.015),
        ]
    )
    bike_body.visual(
        _mesh("saddle_shell", saddle_shell),
        material=saddle_leather,
        name="saddle_shell",
    )

    # Handlebar stem and clamp.
    bike_body.visual(
        Cylinder(radius=0.044, length=0.090),
        origin=Origin(xyz=(0.26, 0.0, 0.932)),
        material=dark_steel,
        name="bar_clamp_collar",
    )
    bike_body.visual(
        Box((0.06, 0.026, 0.04)),
        origin=Origin(xyz=(0.26, 0.040, 0.95)),
        material=dark_steel,
        name="bar_clamp_ear_left",
    )
    bike_body.visual(
        Box((0.06, 0.026, 0.04)),
        origin=Origin(xyz=(0.26, -0.040, 0.95)),
        material=dark_steel,
        name="bar_clamp_ear_right",
    )
    bike_body.visual(
        Cylinder(radius=0.006, length=0.092),
        origin=Origin(xyz=(0.26, 0.0, 0.95), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bar_clamp_bolt",
    )
    bike_body.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.26, 0.055, 0.95), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="bar_adjust_knob",
    )
    bike_body.visual(
        Cylinder(radius=0.022, length=0.25),
        origin=Origin(xyz=(0.26, 0.0, 1.105)),
        material=steel,
        name="handlebar_stem",
    )
    bike_body.visual(
        Cylinder(radius=0.039, length=0.014),
        origin=Origin(xyz=(0.26, 0.0, 0.984)),
        material=steel,
        name="stem_stop",
    )
    bike_body.visual(
        Box((0.08, 0.06, 0.04)),
        origin=Origin(xyz=(0.31, 0.0, 1.225)),
        material=dark_steel,
        name="bar_adapter_block",
    )
    bike_body.visual(
        _tube(
            "bar_riser",
            [(0.26, 0.0, 1.23), (0.32, 0.0, 1.29), (0.40, 0.0, 1.35)],
            radius=0.015,
            samples=14,
            radial=16,
        ),
        material=frame_paint,
        name="bar_riser",
    )
    bike_body.visual(
        _tube(
            "handlebar_loop",
            [
                (0.32, -0.24, 1.25),
                (0.36, -0.18, 1.31),
                (0.39, -0.08, 1.34),
                (0.40, 0.0, 1.35),
                (0.39, 0.08, 1.34),
                (0.36, 0.18, 1.31),
                (0.32, 0.24, 1.25),
            ],
            radius=0.015,
            samples=14,
            radial=16,
        ),
        material=frame_paint,
        name="handlebar_loop",
    )
    bike_body.visual(
        Box((0.05, 0.18, 0.02)),
        origin=Origin(xyz=(0.35, 0.0, 1.28)),
        material=dark_steel,
        name="bar_cross_brace",
    )
    bike_body.visual(
        Cylinder(radius=0.019, length=0.10),
        origin=Origin(xyz=(0.31, 0.255, 1.24), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    bike_body.visual(
        Cylinder(radius=0.019, length=0.10),
        origin=Origin(xyz=(0.31, -0.255, 1.24), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Box((0.44, 0.38, 0.44)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crank_set.visual(
        Cylinder(radius=0.206, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flywheel_disc",
    )
    crank_set.visual(
        Cylinder(radius=0.222, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="flywheel_rim",
    )
    crank_set.visual(
        Cylinder(radius=0.020, length=0.154),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_axle",
    )
    crank_set.visual(
        Cylinder(radius=0.055, length=0.17),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="crank_hub",
    )
    crank_set.visual(
        Cylinder(radius=0.037, length=0.056),
        origin=Origin(xyz=(0.0, 0.082, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_bearing_collar",
    )
    crank_set.visual(
        Cylinder(radius=0.037, length=0.056),
        origin=Origin(xyz=(0.0, -0.082, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_bearing_collar",
    )
    crank_set.visual(
        Box((0.030, 0.012, 0.190)),
        origin=Origin(xyz=(0.0, 0.116, -0.095)),
        material=steel,
        name="left_crank_arm",
    )
    crank_set.visual(
        Box((0.030, 0.012, 0.190)),
        origin=Origin(xyz=(0.0, -0.116, 0.095)),
        material=steel,
        name="right_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, 0.140, -0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, -0.140, 0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Box((0.11, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, 0.150, -0.205)),
        material=rubber,
        name="left_pedal_body",
    )
    crank_set.visual(
        Box((0.11, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, -0.150, 0.205)),
        material=rubber,
        name="right_pedal_body",
    )

    resistance_knob = model.part("resistance_knob")
    resistance_knob.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 0.08)),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )
    resistance_knob.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="knob_washer",
    )
    resistance_knob.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=steel,
        name="knob_collar",
    )
    resistance_knob.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=knob_black,
        name="knob_wheel",
    )
    resistance_knob.visual(
        Box((0.050, 0.006, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.042)),
        material=steel,
        name="knob_pointer",
    )
    resistance_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=steel,
        name="knob_cap",
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=bike_body,
        child=crank_set,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "resistance_adjust",
        ArticulationType.CONTINUOUS,
        parent=bike_body,
        child=resistance_knob,
        origin=Origin(xyz=(-0.02, 0.0, 0.644)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bike_body")
    crank = object_model.get_part("crank_set")
    knob = object_model.get_part("resistance_knob")
    crank_joint = object_model.get_articulation("crank_spin")
    knob_joint = object_model.get_articulation("resistance_adjust")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        body,
        crank,
        elem_a="left_bearing_carrier",
        elem_b="left_bearing_collar",
        reason="Bearing collar intentionally nests into the left carrier cup as a supported press-fit style bearing seat.",
    )
    ctx.allow_overlap(
        body,
        crank,
        elem_a="right_bearing_carrier",
        elem_b="right_bearing_collar",
        reason="Bearing collar intentionally nests into the right carrier cup as a supported press-fit style bearing seat.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        body,
        crank,
        elem_a="left_bearing_carrier",
        elem_b="left_bearing_collar",
        contact_tol=0.001,
        name="left_bearing_support_contact",
    )
    ctx.expect_contact(
        body,
        crank,
        elem_a="right_bearing_carrier",
        elem_b="right_bearing_collar",
        contact_tol=0.001,
        name="right_bearing_support_contact",
    )
    ctx.expect_contact(
        body,
        knob,
        elem_a="resistance_boss",
        elem_b="knob_washer",
        contact_tol=0.001,
        name="resistance_knob_mount_contact",
    )
    ctx.expect_within(
        crank,
        body,
        axes="xz",
        inner_elem="flywheel_disc",
        outer_elem="housing_shell",
        margin=0.01,
        name="flywheel_within_housing_projection",
    )
    ctx.check(
        "crank_axis_is_lateral",
        crank_joint.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"Expected crank axis along ±Y, got {crank_joint.axis!r}",
    )
    ctx.check(
        "resistance_knob_axis_is_vertical",
        knob_joint.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical adjuster axis, got {knob_joint.axis!r}",
    )

    def elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({crank_joint: 0.0}):
        left_pedal_rest = elem_center(crank, "left_pedal_body")
        right_pedal_rest = elem_center(crank, "right_pedal_body")
    with ctx.pose({crank_joint: pi / 2.0}):
        left_pedal_quarter = elem_center(crank, "left_pedal_body")
        right_pedal_quarter = elem_center(crank, "right_pedal_body")

    left_ok = (
        left_pedal_rest is not None
        and left_pedal_quarter is not None
        and left_pedal_quarter[0] > left_pedal_rest[0] + 0.12
    )
    right_ok = (
        right_pedal_rest is not None
        and right_pedal_quarter is not None
        and right_pedal_quarter[0] < right_pedal_rest[0] - 0.12
    )
    ctx.check(
        "left_pedal_advances_on_positive_crank_rotation",
        left_ok,
        details=f"rest={left_pedal_rest}, quarter={left_pedal_quarter}",
    )
    ctx.check(
        "right_pedal_recedes_on_positive_crank_rotation",
        right_ok,
        details=f"rest={right_pedal_rest}, quarter={right_pedal_quarter}",
    )

    ctx.warn_if_articulation_overlaps(max_pose_samples=16)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
