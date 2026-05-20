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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - (length * 0.5), z))
    )


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x - (length * 0.5), y, z))
    )


def _frame_shape() -> cq.Workplane:
    base = _box((0.18, 0.16, 0.02), (-0.10, 0.0, 0.01))
    mast = _box((0.05, 0.11, 0.21), (-0.125, 0.0, 0.115))
    head = _box((0.10, 0.13, 0.024), (-0.08, 0.0, 0.203))
    root = _box((0.09, 0.09, 0.06), (-0.02, 0.0, 0.06))
    nose = _box((0.028, 0.10, 0.026), (0.56, 0.0, 0.093))
    spine = _box((0.14, 0.06, 0.04), (-0.03, 0.0, 0.10))

    shape = base.union(mast).union(head).union(root).union(nose).union(spine)

    for y in (-0.04, 0.04):
        rail_root = _box((0.08, 0.018, 0.024), (0.01, y, 0.088))
        rail = _box((0.58, 0.018, 0.018), (0.29, y, 0.091))
        shape = shape.union(rail_root).union(rail)

    return shape.combine()


def _carriage_shape() -> cq.Workplane:
    left_shoe = _box((0.125, 0.024, 0.014), (0.0, -0.04, 0.007))
    right_shoe = _box((0.125, 0.024, 0.014), (0.0, 0.04, 0.007))
    bridge = _box((0.10, 0.06, 0.026), (-0.005, 0.0, 0.018))
    saddle = _box((0.07, 0.045, 0.03), (-0.012, 0.0, 0.043))
    pylon = _box((0.026, 0.03, 0.04), (0.025, 0.0, 0.068))

    left_lug = _box((0.022, 0.012, 0.036), (0.04, -0.015, 0.083))
    right_lug = _box((0.022, 0.012, 0.036), (0.04, 0.015, 0.083))
    lug_bridge = _box((0.014, 0.03, 0.02), (0.028, 0.0, 0.072))

    return (
        left_shoe.union(right_shoe)
        .union(bridge)
        .union(saddle)
        .union(pylon)
        .union(left_lug)
        .union(right_lug)
        .union(lug_bridge)
    ).combine()


def _upper_link_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.014)
        .lineTo(0.05, -0.018)
        .lineTo(0.135, -0.016)
        .lineTo(0.188, -0.010)
        .lineTo(0.188, 0.014)
        .lineTo(0.13, 0.019)
        .lineTo(0.05, 0.016)
        .lineTo(0.0, 0.014)
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )

    shape = _box((0.022, 0.022, 0.036), (0.011, 0.0, 0.0))
    shape = shape.union(beam)
    shape = shape.union(_box((0.026, 0.03, 0.022), (0.195, 0.0, 0.0)))

    for y in (-0.015, 0.015):
        shape = shape.union(_box((0.018, 0.01, 0.034), (0.199, y, 0.0)))

    return shape


def _tip_shape() -> cq.Workplane:
    shape = _box((0.018, 0.022, 0.03), (0.009, 0.0, 0.0))
    shape = shape.union(_box((0.058, 0.028, 0.022), (0.038, 0.0, 0.0)))
    shape = shape.union(_box((0.05, 0.028, 0.018), (0.09, 0.0, 0.0)))
    shape = shape.union(_box((0.10, 0.01, 0.014), (0.15, -0.01, 0.0)))
    shape = shape.union(_box((0.10, 0.01, 0.014), (0.15, 0.01, 0.0)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_slide")

    model.material("frame_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_gray", rgba=(0.52, 0.54, 0.57, 1.0))
    model.material("arm_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("tip_black", rgba=(0.10, 0.10, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame"),
        material="frame_gray",
        name="frame_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_gray",
        name="carriage_shell",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link"),
        material="arm_dark",
        name="upper_link_shell",
    )

    tip = model.part("tip")
    tip.visual(
        mesh_from_cadquery(_tip_shape(), "tip"),
        material="tip_black",
        name="tip_shell",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.22, 0.0, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.18,
            lower=0.0,
            upper=0.28,
        ),
    )

    model.articulation(
        "carriage_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(0.051, 0.0, 0.081)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.3,
            lower=-0.75,
            upper=1.10,
        ),
    )

    model.articulation(
        "upper_link_to_tip",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=tip,
        origin=Origin(xyz=(0.208, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.6,
            lower=-0.55,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    tip = object_model.get_part("tip")
    slide = object_model.get_articulation("frame_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_link")
    elbow = object_model.get_articulation("upper_link_to_tip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        carriage,
        frame,
        reason=(
            "The carriage models an enclosed linear-bearing shoe that wraps the frame rails; "
            "internal rolling clearance is omitted in this simplified visible geometry."
        ),
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

    ctx.expect_contact(carriage, frame, name="carriage_runs_on_frame")
    ctx.expect_contact(upper_link, carriage, name="shoulder_joint_is_supported")
    ctx.expect_contact(tip, upper_link, name="tip_joint_is_supported")

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.24}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(carriage, frame, name="carriage_stays_supported_when_extended")
    if rest_carriage_pos is not None and extended_carriage_pos is not None:
        ctx.check(
            "slide_extends_forward",
            extended_carriage_pos[0] > rest_carriage_pos[0] + 0.20,
            details=(
                f"expected carriage +x travel > 0.20 m, got "
                f"{extended_carriage_pos[0] - rest_carriage_pos[0]:.3f} m"
            ),
        )

    rest_tip_aabb = ctx.part_world_aabb(tip)
    with ctx.pose({shoulder: 0.70, elbow: 0.45}):
        posed_tip_aabb = ctx.part_world_aabb(tip)
    if rest_tip_aabb is not None and posed_tip_aabb is not None:
        rest_tip_top = rest_tip_aabb[1][2]
        posed_tip_top = posed_tip_aabb[1][2]
        ctx.check(
            "arm_raises_tip_upward",
            posed_tip_top > rest_tip_top + 0.08,
            details=(
                f"expected articulated arm to lift tip by > 0.08 m, got "
                f"{posed_tip_top - rest_tip_top:.3f} m"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
