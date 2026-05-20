from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROLL_AXIS_Z = 0.245


def _make_pedestal() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .circle(0.075)
        .extrude(0.014)
        .faces(">Z")
        .circle(0.054)
        .extrude(0.008)
    )
    column = (
        cq.Workplane("XY")
        .box(0.032, 0.022, 0.106, centered=(True, True, False))
        .translate((0.0, 0.0, 0.022))
        .edges("|Z")
        .fillet(0.004)
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.026, 0.020, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, 0.128))
        .edges("|Z")
        .fillet(0.003)
    )
    front_post = cq.Workplane("XY").box(0.010, 0.014, 0.100).translate((0.0, 0.058, 0.190))
    rear_post = cq.Workplane("XY").box(0.010, 0.014, 0.100).translate((0.0, -0.058, 0.190))
    front_pod = cq.Workplane("XZ").circle(0.012).extrude(0.002, both=True).translate((0.0, 0.049, ROLL_AXIS_Z))
    rear_pod = cq.Workplane("XZ").circle(0.012).extrude(0.002, both=True).translate((0.0, -0.049, ROLL_AXIS_Z))

    return (
        base.union(column)
        .union(shoulder)
        .union(front_post)
        .union(rear_post)
        .union(front_pod)
        .union(rear_pod)
    )


def _make_roll_ring() -> cq.Workplane:
    rim = cq.Workplane("XZ").circle(0.094).circle(0.086).extrude(0.002, both=True)
    front_shaft = cq.Workplane("XZ").circle(0.010).extrude(0.002, both=True).translate((0.0, 0.049, 0.0))
    rear_shaft = cq.Workplane("XZ").circle(0.010).extrude(0.002, both=True).translate((0.0, -0.049, 0.0))
    left_pad = cq.Workplane("XY").box(0.004, 0.008, 0.014).translate((-0.078, 0.0, 0.0))
    right_pad = cq.Workplane("XY").box(0.004, 0.008, 0.014).translate((0.078, 0.0, 0.0))
    left_upper_rib = cq.Workplane("XY").box(0.004, 0.004, 0.022).translate((-0.078, 0.0, 0.057))
    left_lower_rib = cq.Workplane("XY").box(0.004, 0.004, 0.022).translate((-0.078, 0.0, -0.057))
    right_upper_rib = cq.Workplane("XY").box(0.004, 0.004, 0.022).translate((0.078, 0.0, 0.057))
    right_lower_rib = cq.Workplane("XY").box(0.004, 0.004, 0.022).translate((0.078, 0.0, -0.057))

    return (
        rim.union(front_shaft)
        .union(rear_shaft)
        .union(left_pad)
        .union(right_pad)
        .union(left_upper_rib)
        .union(left_lower_rib)
        .union(right_upper_rib)
        .union(right_lower_rib)
    )


def _make_cradle_frame() -> cq.Workplane:
    left_trunnion = cq.Workplane("XY").box(0.004, 0.008, 0.014).translate((-0.074, 0.0, 0.0))
    right_trunnion = cq.Workplane("XY").box(0.004, 0.008, 0.014).translate((0.074, 0.0, 0.0))
    left_cheek = cq.Workplane("XY").box(0.008, 0.006, 0.034).translate((-0.042, 0.010, 0.0))
    right_cheek = cq.Workplane("XY").box(0.008, 0.006, 0.034).translate((0.042, 0.010, 0.0))
    upper_rail = cq.Workplane("XY").box(0.068, 0.004, 0.004).translate((0.0, 0.014, 0.013))
    lower_rail = cq.Workplane("XY").box(0.068, 0.004, 0.004).translate((0.0, 0.014, -0.013))
    rear_brace = cq.Workplane("XY").box(0.042, 0.004, 0.018).translate((0.0, 0.008, 0.0))
    front_mount = cq.Workplane("XY").box(0.026, 0.004, 0.016).translate((0.0, 0.022, 0.0))

    return (
        left_trunnion.union(right_trunnion)
        .union(left_cheek)
        .union(right_cheek)
        .union(upper_rail)
        .union(lower_rail)
        .union(rear_brace)
        .union(front_mount)
    )


def _make_tool_plate() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .rect(0.046, 0.028)
        .extrude(0.004, both=True)
        .edges("|Y")
        .fillet(0.003)
        .faces(">Y")
        .workplane()
        .pushPoints(((-0.013, -0.008), (0.013, -0.008), (-0.013, 0.008), (0.013, 0.008)))
        .hole(0.005)
        .translate((0.0, 0.026, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_instrument_head")

    pedestal_color = model.material("pedestal_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    ring_color = model.material("ring_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    cradle_color = model.material("cradle_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    plate_color = model.material("tool_plate", rgba=(0.55, 0.57, 0.60, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal(), "pedestal_body"),
        material=pedestal_color,
        name="pedestal_body",
    )

    roll_ring = model.part("roll_ring")
    roll_ring.visual(
        mesh_from_cadquery(_make_roll_ring(), "roll_ring_body"),
        material=ring_color,
        name="ring_body",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_cradle_frame(), "pitch_cradle_frame"),
        material=cradle_color,
        name="cradle_frame",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_make_tool_plate(), "instrument_tool_plate"),
        material=plate_color,
        name="tool_plate",
    )

    model.articulation(
        "pedestal_to_roll_ring",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=roll_ring,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "roll_ring_to_pitch_cradle",
        ArticulationType.REVOLUTE,
        parent=roll_ring,
        child=pitch_cradle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.3, lower=-0.9, upper=0.9),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    roll_ring = object_model.get_part("roll_ring")
    pitch_cradle = object_model.get_part("pitch_cradle")
    ring_body = roll_ring.get_visual("ring_body")
    tool_plate = pitch_cradle.get_visual("tool_plate")
    roll_joint = object_model.get_articulation("pedestal_to_roll_ring")
    pitch_joint = object_model.get_articulation("roll_ring_to_pitch_cradle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        pedestal,
        roll_ring,
        reason="roll-axis shaft stubs are intentionally modeled as embedded in the pedestal bearing pods",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_ring,
        reason="pitch-axis trunnions are intentionally modeled as embedded in the roll-ring side bearings",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "roll axis is front-back",
        tuple(roll_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected roll axis (0, 1, 0), got {roll_joint.axis}",
    )
    ctx.check(
        "pitch axis is left-right",
        tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected pitch axis (1, 0, 0), got {pitch_joint.axis}",
    )
    ctx.check(
        "joint limits bracket neutral pose",
        (
            roll_joint.motion_limits is not None
            and pitch_joint.motion_limits is not None
            and roll_joint.motion_limits.lower is not None
            and roll_joint.motion_limits.upper is not None
            and pitch_joint.motion_limits.lower is not None
            and pitch_joint.motion_limits.upper is not None
            and roll_joint.motion_limits.lower < 0.0 < roll_joint.motion_limits.upper
            and pitch_joint.motion_limits.lower < 0.0 < pitch_joint.motion_limits.upper
        ),
        details="expected symmetric useful motion limits on both axes",
    )

    ctx.expect_contact(
        roll_ring,
        pedestal,
        contact_tol=0.0005,
        name="roll ring is borne by pedestal bearings",
    )
    ctx.expect_contact(
        pitch_cradle,
        roll_ring,
        contact_tol=0.0005,
        name="pitch cradle is borne by ring bearings",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_ring,
        axes="xz",
        inner_elem=tool_plate,
        outer_elem=ring_body,
        margin=0.002,
        name="tool plate stays within ring silhouette",
    )

    with ctx.pose({roll_joint: 0.75}):
        ctx.expect_contact(
            roll_ring,
            pedestal,
            contact_tol=0.0005,
            name="roll bearing contact holds at posed roll",
        )

    with ctx.pose({pitch_joint: -0.60}):
        ctx.expect_contact(
            pitch_cradle,
            roll_ring,
            contact_tol=0.0005,
            name="pitch bearing contact holds at posed pitch",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
