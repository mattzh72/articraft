from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_model_plane")

    anodized = model.material("mat_anodized_gray", rgba=(0.42, 0.44, 0.45, 1.0))
    dark = model.material("mat_matte_black", rgba=(0.02, 0.022, 0.025, 1.0))
    light = model.material("mat_datum_white", rgba=(0.86, 0.88, 0.84, 1.0))
    blue = model.material("mat_wing_blue", rgba=(0.08, 0.22, 0.58, 1.0))
    orange = model.material("mat_index_orange", rgba=(1.0, 0.48, 0.08, 1.0))
    steel = model.material("mat_brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))

    # Root fixture: a datum plate, vertical standard, and calibrated yoke.
    stand = model.part("stand")
    stand.visual(
        Box((0.46, 0.30, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=anodized,
        name="datum_plate",
    )
    stand.visual(
        Box((0.40, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=light,
        name="centerline_y",
    )
    stand.visual(
        Box((0.010, 0.24, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=light,
        name="centerline_x",
    )
    for i, x in enumerate((-0.16, -0.08, 0.08, 0.16)):
        stand.visual(
            Box((0.004, 0.050, 0.0035)),
            origin=Origin(xyz=(x, -0.115, 0.03175)),
            material=dark,
            name=f"base_index_{i}",
        )
    stand.visual(
        Box((0.042, 0.042, 0.190)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=anodized,
        name="standard_post",
    )
    stand.visual(
        Box((0.074, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=anodized,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.030, 0.012, 0.066)),
        origin=Origin(xyz=(0.0, -0.037, 0.240)),
        material=anodized,
        name="yoke_cheek_0",
    )
    stand.visual(
        Box((0.030, 0.012, 0.066)),
        origin=Origin(xyz=(0.0, 0.037, 0.240)),
        material=anodized,
        name="yoke_cheek_1",
    )
    for i, z in enumerate((0.218, 0.229, 0.240, 0.251, 0.262)):
        stand.visual(
            Box((0.020 if i == 2 else 0.012, 0.002, 0.0022)),
            origin=Origin(xyz=(0.018, -0.044, z)),
            material=orange if i == 2 else dark,
            name=f"tilt_scale_{i}",
        )

    # Tilting cradle: a captive pivot shaft with a flat aircraft mounting pad.
    tilt = model.part("tilt_stage")
    tilt.visual(
        Cylinder(radius=0.014, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    tilt.visual(
        Box((0.032, 0.030, 0.044)),
        origin=Origin(xyz=(0.018, 0.0, 0.018)),
        material=steel,
        name="web_rib",
    )
    tilt.visual(
        Box((0.084, 0.048, 0.012)),
        origin=Origin(xyz=(0.045, 0.0, 0.030)),
        material=steel,
        name="mount_plate",
    )
    tilt.visual(
        Box((0.030, 0.006, 0.004)),
        origin=Origin(xyz=(0.002, -0.026, 0.012)),
        material=orange,
        name="pitch_pointer",
    )

    # One spin-able locking knob on the yoke side: visible but simple.
    knob = model.part("lock_knob")
    knob.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_wheel",
    )
    knob.visual(
        Box((0.004, 0.019, 0.030)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=light,
        name="knob_index",
    )

    # Calibration aircraft: intentionally flat datum surfaces with a clean
    # model-plane silhouette.  All wings and tail surfaces are part of the
    # fixed airframe, while the propeller is a separate rotating child.
    airframe = model.part("airframe")
    airframe.visual(
        Box((0.360, 0.048, 0.052)),
        origin=Origin(xyz=(-0.015, 0.0, 0.044)),
        material=light,
        name="fuselage",
    )
    airframe.visual(
        Box((0.052, 0.034, 0.036)),
        origin=Origin(xyz=(0.176, 0.0, 0.044)),
        material=light,
        name="nose_block",
    )
    airframe.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.217, 0.0, 0.044), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="nose_bearing",
    )
    airframe.visual(
        Box((0.050, 0.034, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0095)),
        material=steel,
        name="mount_boss",
    )
    airframe.visual(
        Box((0.126, 0.460, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.032)),
        material=blue,
        name="main_wing",
    )
    airframe.visual(
        Box((0.132, 0.026, 0.034)),
        origin=Origin(xyz=(-0.240, 0.0, 0.043)),
        material=light,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.080, 0.205, 0.008)),
        origin=Origin(xyz=(-0.292, 0.0, 0.055)),
        material=blue,
        name="tailplane",
    )
    airframe.visual(
        Box((0.074, 0.010, 0.066)),
        origin=Origin(xyz=(-0.295, 0.0, 0.088)),
        material=blue,
        name="fin",
    )
    airframe.visual(
        Box((0.250, 0.004, 0.003)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0715)),
        material=dark,
        name="fuselage_centerline",
    )
    for i, y in enumerate((-0.180, -0.090, 0.0, 0.090, 0.180)):
        airframe.visual(
            Box((0.018, 0.004, 0.003)),
            origin=Origin(xyz=(0.042, y, 0.0385)),
            material=orange if i == 2 else dark,
            name=f"wing_index_{i}",
        )
    for i, y in enumerate((-0.210, 0.210)):
        airframe.visual(
            Box((0.050, 0.026, 0.003)),
            origin=Origin(xyz=(-0.015, y, 0.0385)),
            material=light,
            name=f"wing_datum_pad_{i}",
        )

    propeller = model.part("propeller")
    rotor = FanRotorGeometry(
        outer_radius=0.066,
        hub_radius=0.014,
        blade_count=2,
        thickness=0.007,
        blade_pitch_deg=31.0,
        blade_sweep_deg=12.0,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=18.0, camber=0.08),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.004, rear_collar_radius=0.013),
    )
    propeller.visual(
        mesh_from_geometry(rotor, "two_blade_propeller"),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="propeller_blades",
    )
    propeller.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="prop_shaft",
    )

    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.6, lower=-0.26, upper=0.26),
    )
    model.articulation(
        "stand_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=knob,
        origin=Origin(xyz=(-0.022, 0.043, 0.240)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0),
    )
    model.articulation(
        "tilt_to_airframe",
        ArticulationType.FIXED,
        parent=tilt,
        child=airframe,
        origin=Origin(xyz=(0.060, 0.0, 0.073)),
    )
    model.articulation(
        "airframe_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.234, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    tilt = object_model.get_part("tilt_stage")
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    knob = object_model.get_part("lock_knob")
    tilt_joint = object_model.get_articulation("stand_to_tilt")
    prop_joint = object_model.get_articulation("airframe_to_propeller")

    ctx.allow_overlap(
        airframe,
        propeller,
        elem_a="nose_bearing",
        elem_b="prop_shaft",
        reason=(
            "The propeller shaft is intentionally captured inside the simplified "
            "nose bearing proxy so the spinning prop is mechanically supported."
        ),
    )
    ctx.expect_within(
        propeller,
        airframe,
        axes="yz",
        inner_elem="prop_shaft",
        outer_elem="nose_bearing",
        margin=0.002,
        name="prop shaft is centered in nose bearing",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="x",
        elem_a="prop_shaft",
        elem_b="nose_bearing",
        min_overlap=0.010,
        name="prop shaft remains inserted in nose bearing",
    )

    ctx.expect_gap(
        stand,
        tilt,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="yoke_cheek_1",
        negative_elem="pivot_shaft",
        name="right yoke cheek captures pivot shaft",
    )
    ctx.expect_gap(
        tilt,
        stand,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="pivot_shaft",
        negative_elem="yoke_cheek_0",
        name="left yoke cheek captures pivot shaft",
    )
    ctx.expect_contact(
        airframe,
        tilt,
        elem_a="mount_boss",
        elem_b="mount_plate",
        contact_tol=0.001,
        name="airframe mount boss seats on tilt plate",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem="propeller_blades",
        negative_elem="nose_bearing",
        name="propeller has visible nose-bearing running gap",
    )
    ctx.expect_contact(
        knob,
        stand,
        elem_a="knob_wheel",
        elem_b="yoke_cheek_1",
        contact_tol=0.0015,
        name="lock knob is mounted on yoke cheek",
    )

    rest_nose = ctx.part_world_aabb(propeller)
    with ctx.pose({tilt_joint: 0.20, prop_joint: 0.9}):
        raised_nose = ctx.part_world_aabb(propeller)
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            min_gap=0.001,
            max_gap=0.030,
            positive_elem="propeller_blades",
            negative_elem="nose_bearing",
            name="propeller gap remains controlled while spun and pitched",
        )

    ctx.check(
        "positive stand pitch raises the propeller",
        rest_nose is not None
        and raised_nose is not None
        and raised_nose[1][2] > rest_nose[1][2] + 0.010,
        details=f"rest_aabb={rest_nose}, pitched_aabb={raised_nose}",
    )

    return ctx.report()


object_model = build_object_model()
