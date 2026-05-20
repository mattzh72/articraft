from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_yaw_rotary_module")

    frame_dark = model.material("frame_dark", color=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", color=(0.73, 0.75, 0.78, 1.0))
    bearing_black = model.material("bearing_black", color=(0.10, 0.10, 0.11, 1.0))

    base_w = 0.170
    base_d = 0.120
    base_t = 0.012

    cheek_t = 0.018
    cheek_d = 0.084
    cheek_h = 0.082
    cheek_x = 0.049
    cheek_relief_r = 0.044

    pedestal_r = 0.017
    pedestal_h = 0.008

    collar_r = 0.017
    collar_h = 0.006
    drum_r = 0.022
    drum_h = 0.040
    top_cap_r = 0.019
    top_cap_h = 0.006

    neck_w = 0.016
    neck_d = 0.024
    neck_h = 0.028
    neck_y = 0.012
    neck_z = 0.012

    output_face_r = 0.020
    output_face_t = 0.006
    output_face_y = 0.034
    output_face_z = 0.026

    frame = model.part("frame")
    head = model.part("head")

    frame_body = (
        cq.Workplane("XY")
        .box(base_w, base_d, base_t, centered=(True, True, False))
        .union(
            cq.Workplane("XY")
            .box(cheek_t, cheek_d, cheek_h, centered=(True, True, False))
            .translate((-cheek_x, 0.0, base_t))
        )
        .union(
            cq.Workplane("XY")
            .box(cheek_t, cheek_d, cheek_h, centered=(True, True, False))
            .translate((cheek_x, 0.0, base_t))
        )
        .cut(
            cq.Workplane("XY")
            .circle(cheek_relief_r)
            .extrude(cheek_h + 0.002)
            .translate((0.0, 0.0, base_t))
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.055, -0.040),
                (-0.055, 0.040),
                (0.055, -0.040),
                (0.055, 0.040),
            ]
        )
        .hole(0.008)
        .edges("|Z")
        .fillet(0.003)
    )
    frame.visual(
        mesh_from_cadquery(frame_body, "frame_body"),
        material=frame_dark,
        name="frame_body",
    )
    frame.visual(
        Cylinder(radius=pedestal_r, length=pedestal_h),
        origin=Origin(xyz=(0.0, 0.0, base_t + pedestal_h / 2.0)),
        material=bearing_black,
        name="bearing_pedestal",
    )

    head_body = (
        cq.Workplane("XY")
        .circle(collar_r)
        .extrude(collar_h)
        .union(
            cq.Workplane("XY")
            .circle(drum_r)
            .extrude(drum_h)
            .translate((0.0, 0.0, collar_h))
        )
        .union(
            cq.Workplane("XY")
            .circle(top_cap_r)
            .extrude(top_cap_h)
            .translate((0.0, 0.0, collar_h + drum_h))
        )
        .union(
            cq.Workplane("XY")
            .box(neck_w, neck_d, neck_h, centered=(True, False, False))
            .translate((0.0, neck_y, neck_z))
        )
    )
    output_face = (
        cq.Workplane("XZ")
        .circle(output_face_r)
        .extrude(output_face_t)
        .translate((0.0, output_face_y, output_face_z))
    )

    head.visual(
        mesh_from_cadquery(head_body, "head_body"),
        material=steel,
        name="head_body",
    )
    head.visual(
        mesh_from_cadquery(output_face, "output_face"),
        material=steel,
        name="output_face",
    )

    model.articulation(
        "frame_to_head_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, base_t + pedestal_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.45,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    yaw = object_model.get_articulation("frame_to_head_yaw")
    limits = yaw.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.check(
        "yaw_axis_is_vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {yaw.axis}",
    )
    ctx.check(
        "yaw_motion_limits_are_symmetric",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0
        and limits.upper > 0.0
        and abs(abs(limits.lower) - limits.upper) < 1e-6,
        details=f"unexpected limits: {limits}",
    )

    neutral_face_center = None
    with ctx.pose({yaw: 0.0}):
        ctx.expect_contact(
            head,
            frame,
            elem_b="bearing_pedestal",
            name="head_supported_on_bearing_pedestal",
        )
        neutral_face_center = center_from_aabb(
            ctx.part_element_world_aabb(head, elem="output_face")
        )
        ctx.check(
            "output_face_neutral_and_forward",
            neutral_face_center is not None
            and abs(neutral_face_center[0]) < 0.004
            and neutral_face_center[1] > 0.028,
            details=f"neutral output face center: {neutral_face_center}",
        )

    if limits is not None and limits.upper is not None:
        with ctx.pose({yaw: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="no_overlap_at_positive_yaw_limit"
            )
            positive_face_center = center_from_aabb(
                ctx.part_element_world_aabb(head, elem="output_face")
            )
            ctx.check(
                "output_face_yaws_about_vertical_axis",
                neutral_face_center is not None
                and positive_face_center is not None
                and positive_face_center[0] < neutral_face_center[0] - 0.015
                and abs(positive_face_center[2] - neutral_face_center[2]) < 0.002,
                details=(
                    f"neutral center={neutral_face_center}, "
                    f"positive center={positive_face_center}"
                ),
            )

    if limits is not None and limits.lower is not None:
        with ctx.pose({yaw: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="no_overlap_at_negative_yaw_limit"
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
