from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="realistic_box_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.82, 0.80, 0.72, 1.0))
    grey = model.material("satin_grey_guard", rgba=(0.64, 0.66, 0.64, 1.0))
    dark = model.material("shadow_black", rgba=(0.02, 0.025, 0.025, 1.0))
    blade_blue = model.material("translucent_smoke_blades", rgba=(0.34, 0.45, 0.55, 0.72))
    knob_grey = model.material("dark_grey_knob", rgba=(0.12, 0.13, 0.14, 1.0))
    label_white = model.material("control_mark_white", rgba=(0.92, 0.92, 0.86, 1.0))

    housing = model.part("housing")

    # Real-world box-fan proportions: roughly a 20 inch square fan, about
    # 15 cm deep with a molded plastic frame.
    outer_w = 0.60
    outer_h = 0.60
    depth = 0.16
    rail = 0.065
    inner_h = outer_h - 2.0 * rail

    housing.visual(
        Box((depth, outer_w, rail)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - rail / 2.0)),
        material=warm_white,
        name="top_rail",
    )
    housing.visual(
        Box((depth, outer_w, rail)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + rail / 2.0)),
        material=warm_white,
        name="bottom_rail",
    )
    housing.visual(
        Box((depth, rail, inner_h)),
        origin=Origin(xyz=(0.0, -outer_w / 2.0 + rail / 2.0, 0.0)),
        material=warm_white,
        name="side_rail_0",
    )
    housing.visual(
        Box((depth, rail, inner_h)),
        origin=Origin(xyz=(0.0, outer_w / 2.0 - rail / 2.0, 0.0)),
        material=warm_white,
        name="side_rail_1",
    )

    # Raised front lip and a rear rim make the plastic frame read as a box, not
    # a single flat picture frame.
    for x, lip_name in ((-0.087, "front_lip"), (0.087, "rear_lip")):
        housing.visual(
            Box((0.014, outer_w, 0.018)),
            origin=Origin(xyz=(x, 0.0, outer_h / 2.0 - 0.009)),
            material=warm_white,
            name=f"{lip_name}_top",
        )
        housing.visual(
            Box((0.014, outer_w, 0.018)),
            origin=Origin(xyz=(x, 0.0, -outer_h / 2.0 + 0.009)),
            material=warm_white,
            name=f"{lip_name}_bottom",
        )
        housing.visual(
            Box((0.014, 0.018, inner_h)),
            origin=Origin(xyz=(x, -outer_w / 2.0 + 0.009, 0.0)),
            material=warm_white,
            name=f"{lip_name}_side_0",
        )
        housing.visual(
            Box((0.014, 0.018, inner_h)),
            origin=Origin(xyz=(x, outer_w / 2.0 - 0.009, 0.0)),
            material=warm_white,
            name=f"{lip_name}_side_1",
        )

    # Two molded feet widen the stance while remaining integral with the lower
    # rail of the housing.
    for i, y in enumerate((-0.18, 0.18)):
        housing.visual(
            Box((0.20, 0.16, 0.055)),
            origin=Origin(xyz=(0.01, y, -0.3275)),
            material=warm_white,
            name=f"foot_{i}",
        )
        housing.visual(
            Box((0.14, 0.11, 0.014)),
            origin=Origin(xyz=(-0.01, y, -0.361)),
            material=dark,
            name=f"rubber_pad_{i}",
        )

    # Integrated carry feature: a dark recessed slot plus a molded rounded grip
    # bridge on the top rail.
    housing.visual(
        Box((0.105, 0.245, 0.006)),
        origin=Origin(xyz=(-0.018, 0.0, 0.303)),
        material=dark,
        name="carry_recess",
    )
    housing.visual(
        Box((0.045, 0.055, 0.026)),
        origin=Origin(xyz=(-0.035, -0.143, 0.312)),
        material=warm_white,
        name="handle_socket_0",
    )
    housing.visual(
        Box((0.045, 0.055, 0.026)),
        origin=Origin(xyz=(-0.035, 0.143, 0.312)),
        material=warm_white,
        name="handle_socket_1",
    )
    handle_tube = tube_from_spline_points(
        [
            (-0.035, -0.143, 0.312),
            (-0.038, -0.090, 0.338),
            (-0.040, 0.000, 0.350),
            (-0.038, 0.090, 0.338),
            (-0.035, 0.143, 0.312),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    housing.visual(
        mesh_from_geometry(handle_tube, "carry_handle"),
        material=warm_white,
        name="carry_handle",
    )

    # Protective front wire grille: concentric rings plus radial spokes in the
    # YZ plane, standing proud of the frame like a real removable guard.
    grille_x = -0.094
    ring_radii = (0.070, 0.115, 0.160, 0.205)
    for i, radius in enumerate(ring_radii):
        ring = mesh_from_geometry(
            TorusGeometry(radius=radius, tube=0.0032, radial_segments=14, tubular_segments=96),
            f"front_ring_{i}",
        )
        housing.visual(
            ring,
            origin=Origin(xyz=(grille_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"front_ring_{i}",
        )
    outer_ring = mesh_from_geometry(
        TorusGeometry(radius=0.248, tube=0.0032, radial_segments=14, tubular_segments=96),
        "front_ring_4",
    )
    housing.visual(
        outer_ring,
        origin=Origin(xyz=(grille_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="front_ring_4",
    )

    spoke_count = 16
    for i in range(spoke_count):
        theta = 2.0 * math.pi * i / spoke_count
        housing.visual(
            Cylinder(radius=0.0024, length=0.510),
            origin=Origin(
                xyz=(grille_x, 0.0, 0.0),
                rpy=(theta - math.pi / 2.0, 0.0, 0.0),
            ),
            material=grey,
            name=f"front_spoke_{i}",
        )

    housing.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(grille_x - 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="grille_center_badge",
    )

    # Four visible screw bosses reinforce that the front guard is serviceable.
    for i, (y, z) in enumerate(((-0.266, -0.266), (-0.266, 0.266), (0.266, -0.266), (0.266, 0.266))):
        housing.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(grille_x - 0.003, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"guard_screw_boss_{i}",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.007),
            origin=Origin(xyz=(grille_x - 0.007, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"guard_screw_head_{i}",
        )

    rear_panel = PerforatedPanelGeometry(
        (0.536, 0.536),
        0.005,
        hole_diameter=0.018,
        pitch=(0.034, 0.034),
        frame=0.020,
        corner_radius=0.010,
        stagger=True,
    )
    housing.visual(
        mesh_from_geometry(rear_panel, "rear_perforated_guard"),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="rear_perforated_guard",
    )

    # Stationary motor housing and three broad struts behind the rotor.
    housing.visual(
        Cylinder(radius=0.088, length=0.052),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="motor_pod",
    )
    strut_inner_r = 0.060
    strut_outer_r = 0.278
    strut_mid_r = (strut_inner_r + strut_outer_r) / 2.0
    for i, theta in enumerate((math.pi / 2.0, math.pi * 7.0 / 6.0, math.pi * 11.0 / 6.0)):
        y = strut_mid_r * math.cos(theta)
        z = strut_mid_r * math.sin(theta)
        housing.visual(
            Cylinder(radius=0.010, length=strut_outer_r - strut_inner_r),
            origin=Origin(
                xyz=(0.034, y, z),
                rpy=(theta - math.pi / 2.0, 0.0, 0.0),
            ),
            material=warm_white,
            name=f"motor_strut_{i}",
        )

    # Front speed-control dial panel on the upper-right face.
    control_y = 0.210
    control_z = 0.225
    housing.visual(
        Cylinder(radius=0.058, length=0.006),
        origin=Origin(xyz=(-0.092, control_y, control_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="control_panel",
    )
    for i, theta in enumerate((-0.75, -0.25, 0.25, 0.75)):
        y = control_y + 0.041 * math.sin(theta)
        z = control_z + 0.041 * math.cos(theta)
        housing.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(-0.0965, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=label_white,
            name=f"speed_dot_{i}",
        )

    rotor = model.part("blade_rotor")
    rotor_geom = FanRotorGeometry(
        0.218,
        0.055,
        5,
        thickness=0.034,
        blade_pitch_deg=32.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18, tip_clearance=0.006),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.044),
    )
    rotor.visual(
        mesh_from_geometry(rotor_geom, "scimitar_blade_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_blue,
        name="scimitar_blades",
    )
    rotor.visual(
        Cylinder(radius=0.014, length=0.085),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="hub_shaft",
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=90.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.001),
    )

    knob = model.part("control_knob")
    knob_geom = KnobGeometry(
        0.046,
        0.027,
        body_style="skirted",
        top_diameter=0.035,
        edge_radius=0.001,
        skirt=KnobSkirt(0.052, 0.005, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "fluted_speed_knob"),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_grey,
        name="fluted_speed_knob",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_grey,
        name="knob_stem",
    )

    model.articulation(
        "housing_to_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(-0.0975, control_y, control_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0, lower=0.0, upper=math.radians(270.0)),
        motion_properties=MotionProperties(damping=0.02, friction=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("blade_rotor")
    knob = object_model.get_part("control_knob")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.allow_overlap(
        housing,
        rotor,
        elem_a="motor_pod",
        elem_b="hub_shaft",
        reason="The rotating hub shaft is intentionally captured inside the stationary motor bearing pod.",
    )
    ctx.allow_overlap(
        housing,
        knob,
        elem_a="control_panel",
        elem_b="knob_stem",
        reason="The rotary control stem intentionally passes through the front control bushing.",
    )

    ctx.check(
        "rotor is continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotor_joint.articulation_type}",
    )
    ctx.check(
        "knob has appliance dial range",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower == 0.0
        and knob_joint.motion_limits.upper is not None
        and math.radians(250.0) <= knob_joint.motion_limits.upper <= math.radians(290.0),
        details=f"limits={knob_joint.motion_limits}",
    )

    ctx.expect_gap(
        rotor,
        housing,
        axis="x",
        negative_elem="front_ring_4",
        min_gap=0.020,
        name="front guard stands ahead of blades",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="x",
        positive_elem="motor_pod",
        negative_elem="scimitar_blades",
        min_gap=0.010,
        name="stationary motor pod sits behind blades",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="yz",
        margin=0.0,
        inner_elem="scimitar_blades",
        outer_elem="front_ring_4",
        name="rotor diameter fits inside front guard ring",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="yz",
        margin=0.001,
        inner_elem="hub_shaft",
        outer_elem="motor_pod",
        name="hub shaft is centered in motor pod bearing",
    )
    ctx.expect_overlap(
        rotor,
        housing,
        axes="x",
        min_overlap=0.020,
        elem_a="hub_shaft",
        elem_b="motor_pod",
        name="hub shaft remains captured by motor pod",
    )
    ctx.expect_overlap(
        knob,
        housing,
        axes="x",
        min_overlap=0.006,
        elem_a="knob_stem",
        elem_b="control_panel",
        name="control stem passes through panel bushing",
    )
    ctx.expect_gap(
        housing,
        knob,
        axis="x",
        positive_elem="control_panel",
        negative_elem="fluted_speed_knob",
        max_gap=0.006,
        max_penetration=0.001,
        name="control knob seats on front control panel",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({rotor_joint: 1.2, knob_joint: math.radians(180.0)}):
        turned_pos = ctx.part_world_position(rotor)
        ctx.expect_gap(
            rotor,
            housing,
            axis="x",
            negative_elem="front_ring_4",
            min_gap=0.020,
            name="spinning blades stay behind front guard",
        )
    ctx.check(
        "rotor spins about fixed center",
        rest_pos is not None and turned_pos is not None and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
