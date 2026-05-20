from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_DEPTH = 0.44
CASE_WIDTH = 0.78
CASE_HEIGHT = 0.34
FRONT_X = CASE_DEPTH / 2.0
REAR_X = -CASE_DEPTH / 2.0
TOP_Z = CASE_HEIGHT
WHEEL_RADIUS = 0.125
WHEEL_WIDTH = 0.080


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Return a lightly filleted rectangular solid authored in meters."""

    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_ready_toolbox")

    charcoal = model.material("charcoal_molded_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite_lid", rgba=(0.18, 0.20, 0.21, 1.0))
    orange = model.material("jobsite_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    black = model.material("rubber_black", rgba=(0.02, 0.02, 0.02, 1.0))
    grey = model.material("dark_hardware", rgba=(0.32, 0.34, 0.35, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))

    case = model.part("case")

    # A broad portable jobsite case: lower tub, proud lid, front seam/lip,
    # reinforced rear channels, latch hinge brackets, hub stubs, and skid feet.
    case.visual(
        mesh_from_cadquery(_rounded_box((CASE_DEPTH, CASE_WIDTH, 0.255), 0.030), "case_tub"),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=charcoal,
        name="tub_shell",
    )
    case.visual(
        mesh_from_cadquery(_rounded_box((0.470, 0.810, 0.075), 0.026), "case_lid"),
        origin=Origin(xyz=(0.0, 0.0, 0.312)),
        material=graphite,
        name="lid_shell",
    )
    case.visual(
        Box((0.028, 0.760, 0.030)),
        origin=Origin(xyz=(FRONT_X + 0.008, 0.0, 0.280)),
        material=orange,
        name="front_lid_lip",
    )
    case.visual(
        Box((0.018, 0.720, 0.018)),
        origin=Origin(xyz=(FRONT_X + 0.001, 0.0, 0.245)),
        material=charcoal,
        name="front_seam_shadow",
    )

    for y in (-0.300, 0.300):
        case.visual(
            Box((0.360, 0.030, 0.018)),
            origin=Origin(xyz=(0.000, y, 0.355)),
            material=orange,
            name=f"top_rib_{0 if y < 0.0 else 1}",
        )
    for y in (-0.265, 0.265):
        # Rear guide channels are open U-shaped tracks.  They are wider than
        # the first telescoping handle stage and visibly fixed to the case back.
        idx = 0 if y < 0.0 else 1
        case.visual(
            Box((0.014, 0.078, 0.330)),
            origin=Origin(xyz=(REAR_X - 0.044, y, 0.224)),
            material=grey,
            name=f"rear_channel_{idx}_back",
        )
        case.visual(
            Box((0.038, 0.010, 0.330)),
            origin=Origin(xyz=(REAR_X - 0.024, y - 0.039, 0.224)),
            material=grey,
            name=f"rear_channel_{idx}_side_0",
        )
        case.visual(
            Box((0.038, 0.010, 0.330)),
            origin=Origin(xyz=(REAR_X - 0.024, y + 0.039, 0.224)),
            material=grey,
            name=f"rear_channel_{idx}_side_1",
        )
        case.visual(
            Box((0.014, 0.084, 0.026)),
            origin=Origin(xyz=(REAR_X - 0.046, y, TOP_Z + 0.010)),
            material=grey,
            name=f"guide_mouth_{idx}_back",
        )
        case.visual(
            Box((0.050, 0.010, 0.026)),
            origin=Origin(xyz=(REAR_X - 0.025, y - 0.042, TOP_Z + 0.010)),
            material=grey,
            name=f"guide_mouth_{idx}_side_0",
        )
        case.visual(
            Box((0.050, 0.010, 0.026)),
            origin=Origin(xyz=(REAR_X - 0.025, y + 0.042, TOP_Z + 0.010)),
            material=grey,
            name=f"guide_mouth_{idx}_side_1",
        )

    for y in (-0.215, 0.215):
        idx = 0 if y < 0.0 else 1
        case.visual(
            Box((0.052, 0.018, 0.044)),
            origin=Origin(xyz=(FRONT_X + 0.022, y - 0.065, 0.318)),
            material=grey,
            name=f"latch_bracket_{idx}_0",
        )
        case.visual(
            Box((0.052, 0.018, 0.044)),
            origin=Origin(xyz=(FRONT_X + 0.022, y + 0.065, 0.318)),
            material=grey,
            name=f"latch_bracket_{idx}_1",
        )
        case.visual(
            Cylinder(radius=0.010, length=0.135),
            origin=Origin(xyz=(FRONT_X + 0.030, y, 0.338), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"latch_pin_{idx}",
        )

    for y, side in ((-(CASE_WIDTH / 2.0 + 0.006), 0), (CASE_WIDTH / 2.0 + 0.006, 1)):
        # Short fixed hubs stop just inside the wheel inner faces.
        case.visual(
            Cylinder(radius=0.045, length=0.052),
            origin=Origin(
                xyz=(REAR_X + 0.040, y, WHEEL_RADIUS),
                rpy=(math.pi / 2.0 if y < 0.0 else -math.pi / 2.0, 0.0, 0.0),
            ),
            material=grey,
            name=f"axle_hub_{side}",
        )
    for y in (-0.250, 0.250):
        case.visual(
            Box((0.120, 0.110, 0.035)),
            origin=Origin(xyz=(FRONT_X - 0.075, y, 0.026)),
            material=black,
            name=f"front_foot_{0 if y < 0.0 else 1}",
        )

    # First pull-handle stage: an open metal bridge with two wide sliding tracks
    # that travel out of the rear guide channels.
    stage_0 = model.part("handle_stage_0")
    for y in (-0.265, 0.265):
        idx = 0 if y < 0.0 else 1
        stage_0.visual(
            Box((0.008, 0.052, 0.480)),
            origin=Origin(xyz=(-0.012, y, -0.055)),
            material=steel,
            name=f"post_back_{idx}",
        )
        stage_0.visual(
            Box((0.028, 0.006, 0.480)),
            origin=Origin(xyz=(-0.005, y - 0.026, -0.055)),
            material=steel,
            name=f"post_side_{idx}_0",
        )
        stage_0.visual(
            Box((0.028, 0.006, 0.480)),
            origin=Origin(xyz=(-0.005, y + 0.026, -0.055)),
            material=steel,
            name=f"post_side_{idx}_1",
        )
    stage_0.visual(
        Box((0.006, 0.590, 0.026)),
        origin=Origin(xyz=(-0.018, 0.0, 0.185)),
        material=steel,
        name="outer_bridge",
    )
    stage_0.visual(
        Box((0.046, 0.605, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, -0.296)),
        material=grey,
        name="lower_sync_bar",
    )

    stage_1 = model.part("handle_stage_1")
    for y in (-0.265, 0.265):
        idx = 0 if y < 0.0 else 1
        stage_1.visual(
            Box((0.018, 0.018, 0.440)),
            origin=Origin(xyz=(-0.003, y, -0.112)),
            material=steel,
            name=f"inner_rail_{idx}",
        )
    stage_1.visual(
        Cylinder(radius=0.028, length=0.560),
        origin=Origin(xyz=(-0.003, 0.0, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_sleeve",
    )
    stage_1.visual(
        Box((0.044, 0.590, 0.040)),
        origin=Origin(xyz=(-0.003, 0.0, 0.118)),
        material=orange,
        name="handle_crossbar_core",
    )

    model.articulation(
        "case_to_handle_stage_0",
        ArticulationType.PRISMATIC,
        parent=case,
        child=stage_0,
        origin=Origin(xyz=(REAR_X - 0.022, 0.0, TOP_Z + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.28, lower=0.0, upper=0.240),
    )
    model.articulation(
        "handle_stage_0_to_handle_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.220),
    )

    # Two independent latch tabs mounted on the front edge.  Their frames live
    # on the hinge pins; closed tabs hang down the front lip and rotate outward.
    for y in (-0.215, 0.215):
        idx = 0 if y < 0.0 else 1
        latch = model.part(f"latch_tab_{idx}")
        latch.visual(
            Cylinder(radius=0.013, length=0.105),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=orange,
            name="hinge_barrel",
        )
        latch.visual(
            Box((0.030, 0.100, 0.105)),
            origin=Origin(xyz=(0.018, 0.0, -0.062)),
            material=orange,
            name="release_tab",
        )
        latch.visual(
            Box((0.009, 0.065, 0.034)),
            origin=Origin(xyz=(0.036, 0.0, -0.101)),
            material=black,
            name="finger_pad",
        )
        model.articulation(
            f"case_to_latch_tab_{idx}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=latch,
            origin=Origin(xyz=(FRONT_X + 0.038, y, 0.338)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.25),
        )

    for y, idx in (-(CASE_WIDTH / 2.0 + WHEEL_WIDTH / 2.0 + 0.034), 0), (
        (CASE_WIDTH / 2.0 + WHEEL_WIDTH / 2.0 + 0.034),
        1,
    ):
        wheel = model.part(f"rear_wheel_{idx}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.081, length=0.084),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="rim_shell",
        )
        wheel.visual(
            Cylinder(radius=0.040, length=0.086),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=orange,
            name="hub_cap",
        )
        for lug_index in range(16):
            angle = lug_index * (2.0 * math.pi / 16.0)
            wheel.visual(
                Box((WHEEL_WIDTH + 0.010, 0.026, 0.014)),
                origin=Origin(
                    xyz=(0.0, -0.123 * math.sin(angle), 0.123 * math.cos(angle)),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=black,
                name=f"tread_lug_{lug_index}",
            )
        model.articulation(
            f"case_to_rear_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=case,
            child=wheel,
            origin=Origin(xyz=(REAR_X + 0.040, y, WHEEL_RADIUS), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    stage_0 = object_model.get_part("handle_stage_0")
    stage_1 = object_model.get_part("handle_stage_1")
    latch_0 = object_model.get_part("latch_tab_0")
    wheel_0 = object_model.get_part("rear_wheel_0")
    wheel_1 = object_model.get_part("rear_wheel_1")
    slide_0 = object_model.get_articulation("case_to_handle_stage_0")
    slide_1 = object_model.get_articulation("handle_stage_0_to_handle_stage_1")
    latch_joint = object_model.get_articulation("case_to_latch_tab_0")

    for idx in (0, 1):
        ctx.allow_overlap(
            "case",
            f"latch_tab_{idx}",
            elem_a=f"latch_pin_{idx}",
            elem_b="hinge_barrel",
            reason="The latch hinge barrel is intentionally captured around the fixed front hinge pin.",
        )
        ctx.expect_overlap(
            "case",
            f"latch_tab_{idx}",
            axes="y",
            elem_a=f"latch_pin_{idx}",
            elem_b="hinge_barrel",
            min_overlap=0.085,
            name=f"latch_{idx}_barrel_spans_pin",
        )
        ctx.expect_gap(
            f"latch_tab_{idx}",
            "case",
            axis="x",
            positive_elem="hinge_barrel",
            negative_elem=f"latch_pin_{idx}",
            max_penetration=0.030,
            name=f"latch_{idx}_pin_capture_is_local",
        )

    for idx in (0, 1):
        ctx.expect_contact(
            "case",
            f"rear_wheel_{idx}",
            elem_a=f"axle_hub_{idx}",
            elem_b="rim_shell",
            contact_tol=0.001,
            name=f"rear_wheel_{idx}_hub_contacts_rim",
        )

    ctx.check("broad_wheeled_case_present", case is not None, "Expected the root wheeled case.")
    ctx.check(
        "telescoping_handle_stages_present",
        stage_0 is not None and stage_1 is not None and slide_0 is not None and slide_1 is not None,
        "Expected a nested two-stage prismatic pull handle.",
    )
    ctx.check(
        "front_latch_tabs_present",
        latch_0 is not None and object_model.get_part("latch_tab_1") is not None,
        "Expected a pair of front latch tabs.",
    )
    ctx.check(
        "rear_wheels_present",
        wheel_0 is not None and wheel_1 is not None,
        "Expected two large rear wheels.",
    )
    if stage_0 is not None and stage_1 is not None and slide_0 is not None and slide_1 is not None:
        rest_stage_0 = ctx.part_world_position(stage_0)
        rest_stage_1 = ctx.part_world_position(stage_1)
        with ctx.pose({slide_0: 0.240, slide_1: 0.220}):
            high_stage_0 = ctx.part_world_position(stage_0)
            high_stage_1 = ctx.part_world_position(stage_1)
        ctx.check(
            "handle_stage_0_extends_up",
            rest_stage_0 is not None and high_stage_0 is not None and high_stage_0[2] > rest_stage_0[2] + 0.20,
            details=f"rest={rest_stage_0}, extended={high_stage_0}",
        )
        ctx.check(
            "handle_stage_1_extends_above_stage_0",
            rest_stage_1 is not None and high_stage_1 is not None and high_stage_1[2] > rest_stage_1[2] + 0.42,
            details=f"rest={rest_stage_1}, extended={high_stage_1}",
        )

    if latch_0 is not None and latch_joint is not None:
        closed = ctx.part_world_aabb(latch_0)
        with ctx.pose({latch_joint: 1.15}):
            released = ctx.part_world_aabb(latch_0)
        ok = False
        if closed is not None and released is not None:
            closed_min, closed_max = closed
            released_min, released_max = released
            ok = released_max[0] > closed_max[0] + 0.035 and released_min[2] > closed_min[2] + 0.030
        ctx.check("latch_tab_rotates_outward_to_release", ok, details=f"closed={closed}, released={released}")

    for idx in (0, 1):
        joint = object_model.get_articulation(f"case_to_rear_wheel_{idx}")
        ctx.check(
            f"rear_wheel_{idx}_continuous_rotation",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            "Wheel should be a continuous axle rotation joint.",
        )

    return ctx.report()


object_model = build_object_model()
