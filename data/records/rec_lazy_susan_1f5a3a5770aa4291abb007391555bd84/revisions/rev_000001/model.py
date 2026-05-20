from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    LatheGeometry,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_lazy_susan")

    wood = Material("warm_oak", rgba=(0.76, 0.49, 0.25, 1.0))
    graphite = Material("dark_graphite", rgba=(0.04, 0.045, 0.05, 1.0))
    steel = Material("brushed_steel", rgba=(0.74, 0.72, 0.67, 1.0))

    # Real-world dining-table scale: roughly a 78 cm rotating tray on a low,
    # slightly smaller bearing ring.  Z=0 is the tabletop underside of the base.
    joint_z = 0.045

    base = model.part("base")
    base_profile = [
        (0.130, 0.000),
        (0.310, 0.000),
        (0.310, 0.014),
        (0.268, 0.014),
        (0.268, 0.031),
        (0.288, 0.031),
        (0.288, 0.039),
        (0.190, 0.039),
        (0.190, 0.031),
        (0.205, 0.031),
        (0.205, 0.014),
        (0.130, 0.014),
    ]
    base.visual(
        mesh_from_geometry(
            LatheGeometry(base_profile, segments=128, closed=True),
            "base_body",
        ),
        material=graphite,
        name="base_body",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(0.235, 0.008, radial_segments=24, tubular_segments=128),
            "bearing_race",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=steel,
        name="bearing_race",
    )

    tray = model.part("tray")
    tray_profile = [
        (0.000, 0.010),
        (0.000, 0.034),
        (0.360, 0.034),
        (0.360, 0.056),
        (0.392, 0.056),
        (0.392, 0.010),
    ]
    tray.visual(
        mesh_from_geometry(
            LatheGeometry(tray_profile, segments=160, closed=True),
            "tray_body",
        ),
        material=wood,
        name="tray_body",
    )

    # A hidden rotating retainer collar is attached to the underside of the tray.
    # Its inward hook sits below the fixed base lip with clearance, so the tray
    # reads as mechanically clipped/seated while still rotating about the center.
    clip_profile = [
        (0.296, 0.012),
        (0.318, 0.012),
        (0.318, -0.024),
        (0.276, -0.024),
        (0.276, -0.018),
        (0.296, -0.018),
    ]
    tray.visual(
        mesh_from_geometry(
            LatheGeometry(clip_profile, segments=128, closed=True),
            "clip_ring",
        ),
        material=steel,
        name="clip_ring",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    turntable = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray has continuous vertical rotation",
        getattr(turntable, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(getattr(turntable, "axis", ())) == (0.0, 0.0, 1.0),
        details=f"type={getattr(turntable, 'articulation_type', None)}, axis={getattr(turntable, 'axis', None)}",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_body",
        negative_elem="bearing_race",
        min_gap=0.0,
        max_gap=0.001,
        name="tray body is seated on the low bearing race",
    )
    ctx.expect_within(
        base,
        tray,
        axes="xy",
        inner_elem="base_body",
        outer_elem="tray_body",
        margin=0.0,
        name="fixed base ring is smaller than tray footprint",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({turntable: 1.75}):
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            positive_elem="tray_body",
            negative_elem="bearing_race",
            min_gap=0.0,
            max_gap=0.001,
            name="rotated tray remains seated on bearing race",
        )
        turned_pos = ctx.part_world_position(tray)

    ctx.check(
        "rotation keeps tray clipped on center axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
