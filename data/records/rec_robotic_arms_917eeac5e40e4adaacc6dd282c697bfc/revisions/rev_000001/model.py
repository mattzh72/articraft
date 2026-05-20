from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Centered rounded rectangular solid, authored in meters."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        safe_radius = min(radius, min(size) * 0.30)
        try:
            shape = shape.edges().fillet(safe_radius)
        except Exception:
            # Very thin trim strips can be smaller than CadQuery's robust
            # all-edge fillet envelope.  Keeping their crisp rectangular form
            # is preferable to failing the whole product model.
            pass
    return shape


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_arm")

    painted = _mat(model, "satin_warm_white_painted_metal", (0.86, 0.84, 0.78, 1.0))
    graphite = _mat(model, "graphite_engineering_polymer", (0.06, 0.065, 0.07, 1.0))
    anodized = _mat(model, "champagne_anodized_aluminum", (0.70, 0.62, 0.48, 1.0))
    elastomer = _mat(model, "matte_black_elastomer", (0.012, 0.012, 0.014, 1.0))
    dark_glass = _mat(model, "smoked_status_lens", (0.02, 0.04, 0.055, 1.0))

    # Root pedestal: broad, low, and visibly grounded.  The yaw joint sits on
    # the top bearing face, so the rotating shoulder carousel has an honest
    # support path into the pedestal.
    base = model.part("base_pedestal")
    base.visual(
        Cylinder(radius=0.205, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted,
        name="weighted_foot",
    )
    base.visual(
        Cylinder(radius=0.116, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=painted,
        name="tapered_column",
    )
    base.visual(
        Cylinder(radius=0.149, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=anodized,
        name="fixed_bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.154, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=elastomer,
        name="yaw_seal",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.082, 0.030, 0.040), 0.006), "base_cable_gland"),
        origin=Origin(xyz=(-0.102, 0.0, 0.090)),
        material=elastomer,
        name="cable_gland",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.050, 0.004, 0.012), 0.002), "base_status_lens"),
        origin=Origin(xyz=(0.0, -0.205, 0.043)),
        material=dark_glass,
        name="status_lens",
    )

    # Yawing shoulder carrier.  The side plates and the lower bridge form a
    # real clevis around the shoulder axis without using hidden overlaps.
    shoulder = model.part("shoulder_carousel")
    shoulder.visual(
        Cylinder(radius=0.148, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=painted,
        name="rotating_turntable",
    )
    shoulder.visual(
        Cylinder(radius=0.122, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=anodized,
        name="turntable_trim",
    )
    shoulder.visual(
        mesh_from_cadquery(_rounded_box((0.110, 0.142, 0.052), 0.012), "shoulder_low_spine"),
        origin=Origin(xyz=(0.055, 0.0, 0.074)),
        material=painted,
        name="low_spine",
    )
    shoulder.visual(
        mesh_from_cadquery(_rounded_box((0.135, 0.025, 0.160), 0.010), "shoulder_yoke_pos"),
        origin=Origin(xyz=(0.180, 0.094, 0.130)),
        material=painted,
        name="yoke_plate_0",
    )
    shoulder.visual(
        mesh_from_cadquery(_rounded_box((0.135, 0.025, 0.160), 0.010), "shoulder_yoke_neg"),
        origin=Origin(xyz=(0.180, -0.094, 0.130)),
        material=painted,
        name="yoke_plate_1",
    )
    shoulder.visual(
        mesh_from_cadquery(_rounded_box((0.120, 0.188, 0.026), 0.008), "shoulder_under_bridge"),
        origin=Origin(xyz=(0.163, 0.0, 0.050)),
        material=painted,
        name="under_bridge",
    )
    shoulder.visual(
        Cylinder(radius=0.071, length=0.010),
        origin=Origin(xyz=(0.180, 0.110, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="outer_bearing_0",
    )
    shoulder.visual(
        Cylinder(radius=0.071, length=0.010),
        origin=Origin(xyz=(0.180, -0.110, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="outer_bearing_1",
    )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(radius=0.064, length=0.152),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_cartridge",
    )
    upper.visual(
        Cylinder(radius=0.071, length=0.010),
        origin=Origin(xyz=(0.0, 0.081, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="shoulder_cap_0",
    )
    upper.visual(
        Cylinder(radius=0.071, length=0.010),
        origin=Origin(xyz=(0.0, -0.081, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="shoulder_cap_1",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.280, 0.070, 0.060), 0.018), "upper_sculpted_link"),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=painted,
        name="sculpted_link",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.206, 0.004, 0.024), 0.002), "upper_polymer_inset_pos"),
        origin=Origin(xyz=(0.188, 0.037, 0.010)),
        material=graphite,
        name="side_inset_0",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.206, 0.004, 0.024), 0.002), "upper_polymer_inset_neg"),
        origin=Origin(xyz=(0.188, -0.037, 0.010)),
        material=graphite,
        name="side_inset_1",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.055, 0.158, 0.056), 0.010), "upper_clevis_web"),
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        material=painted,
        name="clevis_web",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.170, 0.024, 0.110), 0.009), "upper_elbow_rail_pos"),
        origin=Origin(xyz=(0.375, 0.076, 0.0)),
        material=painted,
        name="elbow_rail_0",
    )
    upper.visual(
        mesh_from_cadquery(_rounded_box((0.170, 0.024, 0.110), 0.009), "upper_elbow_rail_neg"),
        origin=Origin(xyz=(0.375, -0.076, 0.0)),
        material=painted,
        name="elbow_rail_1",
    )
    upper.visual(
        Cylinder(radius=0.059, length=0.012),
        origin=Origin(xyz=(0.440, 0.094, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="elbow_bearing_0",
    )
    upper.visual(
        Cylinder(radius=0.059, length=0.012),
        origin=Origin(xyz=(0.440, -0.094, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="elbow_bearing_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.052, length=0.118),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_cartridge",
    )
    forearm.visual(
        Cylinder(radius=0.056, length=0.007),
        origin=Origin(xyz=(0.0, 0.0615, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="elbow_cap_0",
    )
    forearm.visual(
        Cylinder(radius=0.056, length=0.007),
        origin=Origin(xyz=(0.0, -0.0615, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="elbow_cap_1",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.245, 0.064, 0.052), 0.016), "forearm_sculpted_link"),
        origin=Origin(xyz=(0.168, 0.0, 0.0)),
        material=painted,
        name="sculpted_link",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.170, 0.004, 0.020), 0.002), "forearm_inset_pos"),
        origin=Origin(xyz=(0.170, 0.034, 0.010)),
        material=graphite,
        name="side_inset_0",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.170, 0.004, 0.020), 0.002), "forearm_inset_neg"),
        origin=Origin(xyz=(0.170, -0.034, 0.010)),
        material=graphite,
        name="side_inset_1",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.048, 0.128, 0.050), 0.009), "forearm_clevis_web"),
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        material=painted,
        name="clevis_web",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.125, 0.021, 0.092), 0.008), "forearm_wrist_rail_pos"),
        origin=Origin(xyz=(0.322, 0.061, 0.0)),
        material=painted,
        name="wrist_rail_0",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.125, 0.021, 0.092), 0.008), "forearm_wrist_rail_neg"),
        origin=Origin(xyz=(0.322, -0.061, 0.0)),
        material=painted,
        name="wrist_rail_1",
    )
    forearm.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.370, 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="wrist_bearing_0",
    )
    forearm.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.370, -0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="wrist_bearing_1",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.043, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pitch_cartridge",
    )
    wrist.visual(
        Cylinder(radius=0.047, length=0.004),
        origin=Origin(xyz=(0.0, 0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="pitch_cap_0",
    )
    wrist.visual(
        Cylinder(radius=0.047, length=0.004),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="pitch_cap_1",
    )
    wrist.visual(
        mesh_from_cadquery(_rounded_box((0.104, 0.072, 0.058), 0.014), "wrist_palm_body"),
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
        material=painted,
        name="palm_body",
    )
    wrist.visual(
        Cylinder(radius=0.041, length=0.018),
        origin=Origin(xyz=(0.121, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="roll_seal",
    )

    flange = model.part("tool_flange")
    flange.visual(
        Cylinder(radius=0.040, length=0.064),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="roll_cartridge",
    )
    flange.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="mounting_plate",
    )
    flange.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="seal_face",
    )
    for idx, (y, z) in enumerate(((0.026, 0.026), (0.026, -0.026), (-0.026, 0.026), (-0.026, -0.026))):
        flange.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(0.080, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"fastener_{idx}",
        )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-2.90, upper=2.90),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper,
        origin=Origin(xyz=(0.180, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=-0.95, upper=1.30),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.440, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.7, lower=-2.05, upper=2.20),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.370, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=flange,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_pedestal")
    shoulder = object_model.get_part("shoulder_carousel")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    flange = object_model.get_part("tool_flange")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check(
        "shoulder elbow wrist chain is fully articulated",
        len(object_model.articulations) == 5,
        details=f"articulation_count={len(object_model.articulations)}",
    )
    ctx.expect_contact(
        base,
        shoulder,
        elem_a="yaw_seal",
        elem_b="rotating_turntable",
        contact_tol=0.006,
        name="yaw cartridge sits on pedestal bearing",
    )
    ctx.expect_overlap(
        shoulder,
        upper,
        axes="xz",
        elem_a="yoke_plate_0",
        elem_b="shoulder_cartridge",
        min_overlap=0.070,
        name="shoulder yoke visibly captures cartridge",
    )
    ctx.expect_gap(
        shoulder,
        upper,
        axis="y",
        positive_elem="yoke_plate_0",
        negative_elem="shoulder_cartridge",
        min_gap=0.001,
        max_gap=0.010,
        name="shoulder bearing has small side clearance",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="xz",
        elem_a="elbow_rail_0",
        elem_b="elbow_cartridge",
        min_overlap=0.055,
        name="elbow fork visibly captures cartridge",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xz",
        elem_a="wrist_rail_0",
        elem_b="pitch_cartridge",
        min_overlap=0.045,
        name="wrist fork visibly captures cartridge",
    )
    ctx.expect_contact(
        wrist,
        flange,
        elem_a="roll_seal",
        elem_b="seal_face",
        contact_tol=0.010,
        name="roll flange is seated against elastomer seal",
    )

    rest_flange = ctx.part_world_position(flange)
    with ctx.pose({shoulder_pitch: 0.65, elbow_pitch: 0.75, wrist_pitch: -0.35}):
        posed_flange = ctx.part_world_position(flange)
    ctx.check(
        "pitch joints move the tool flange upward",
        rest_flange is not None
        and posed_flange is not None
        and posed_flange[2] > rest_flange[2] + 0.20,
        details=f"rest={rest_flange}, posed={posed_flange}",
    )

    return ctx.report()


object_model = build_object_model()
