from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SUPPORT_INNER_GAP = 0.100
UPPER_ARM_LENGTH = 0.340
ELBOW_INNER_GAP = 0.078
FOREARM_LENGTH = 0.270
WRIST_INNER_GAP = 0.062

def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _top_support_shape() -> cq.Workplane:
    top_plate = cq.Workplane("XY").box(0.280, 0.184, 0.018).translate((-0.140, 0.0, 0.126))
    mast = cq.Workplane("XY").box(0.090, 0.108, 0.084).translate((-0.190, 0.0, 0.083))
    spine = cq.Workplane("XY").box(0.150, 0.078, 0.038).translate((-0.105, 0.0, 0.062))
    shoulder_nose = cq.Workplane("XY").box(0.050, 0.108, 0.060).translate((-0.025, 0.0, 0.022))
    lower_lug = cq.Workplane("XY").box(0.020, 0.094, 0.070).translate((-0.010, 0.0, -0.022))

    return top_plate.union(mast).union(spine).union(shoulder_nose).union(lower_lug)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _cylinder_y(radius=0.034, length=0.086).translate((0.034, 0.0, 0.000))
    root_block = cq.Workplane("XY").box(0.050, 0.048, 0.040).translate((0.055, 0.0, -0.018))
    main_beam = cq.Workplane("XY").box(0.190, 0.044, 0.034).translate((0.170, 0.0, -0.022))
    lower_rib = cq.Workplane("XY").box(0.120, 0.028, 0.022).translate((0.145, 0.0, -0.045))
    elbow_hub = _cylinder_y(radius=0.026, length=0.074).translate((UPPER_ARM_LENGTH - 0.026, 0.0, -0.004))
    elbow_bridge = cq.Workplane("XY").box(0.034, 0.038, 0.028).translate((0.277, 0.0, -0.018))

    return shoulder_hub.union(root_block).union(main_beam).union(lower_rib).union(elbow_bridge).union(elbow_hub)


def _forearm_shape() -> cq.Workplane:
    elbow_hub = _cylinder_y(radius=0.028, length=0.072).translate((0.028, 0.0, -0.004))
    root_block = cq.Workplane("XY").box(0.042, 0.040, 0.032).translate((0.050, 0.0, -0.018))
    main_beam = cq.Workplane("XY").box(0.150, 0.038, 0.030).translate((0.135, 0.0, -0.022))
    lower_rib = cq.Workplane("XY").box(0.090, 0.024, 0.018).translate((0.132, 0.0, -0.042))
    wrist_hub = _cylinder_y(radius=0.022, length=0.056).translate((FOREARM_LENGTH - 0.022, 0.0, -0.010))
    wrist_bridge = cq.Workplane("XY").box(0.026, 0.032, 0.024).translate((0.218, 0.0, -0.020))

    return elbow_hub.union(root_block).union(main_beam).union(lower_rib).union(wrist_bridge).union(wrist_hub)


def _wrist_shape() -> cq.Workplane:
    wrist_hub = _cylinder_y(radius=0.022, length=0.056).translate((0.022, 0.0, -0.010))
    neck = cq.Workplane("XY").box(0.026, 0.032, 0.026).translate((0.042, 0.0, -0.020))
    housing = cq.Workplane("XY").box(0.058, 0.046, 0.036).translate((0.078, 0.0, -0.040))
    chin = cq.Workplane("XY").box(0.026, 0.046, 0.018).translate((0.102, 0.0, -0.058))
    tool_stem = cq.Workplane("XY").box(0.018, 0.026, 0.020).translate((0.106, 0.0, -0.074))
    tool_pad = cq.Workplane("XY").circle(0.028).extrude(0.010).translate((0.106, 0.0, -0.089))

    return wrist_hub.union(neck).union(housing).union(chin).union(tool_stem).union(tool_pad)


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_robot_arm")

    model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("machined_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("anodized_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("tool_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_top_support_shape(), "top_support"),
        material="graphite",
        name="support_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm"),
        material="machined_aluminum",
        name="upper_arm_body",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm"),
        material="machined_aluminum",
        name="forearm_body",
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_wrist_shape(), "wrist_housing"),
        material="anodized_dark",
        name="wrist_body",
    )
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.20, effort=140.0, velocity=1.4),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.10, upper=2.00, effort=95.0, velocity=1.8),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=50.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

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

    ctx.expect_contact(top_support, upper_arm, contact_tol=0.0015, name="shoulder assembly is supported")
    ctx.expect_contact(upper_arm, forearm, contact_tol=0.0015, name="elbow assembly is supported")
    ctx.expect_contact(forearm, wrist, contact_tol=0.0015, name="wrist assembly is supported")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_pitch: 0.0}):
        elbow_rest = ctx.part_world_position(forearm)
        wrist_rest = ctx.part_world_position(wrist)
        wrist_rest_aabb = ctx.part_world_aabb(wrist)
        tool_rest = wrist_rest_aabb[0][2] if wrist_rest_aabb is not None else None

    with ctx.pose({shoulder: 0.60, elbow: 0.0, wrist_pitch: 0.0}):
        elbow_up = ctx.part_world_position(forearm)

    with ctx.pose({shoulder: 0.0, elbow: 0.75, wrist_pitch: 0.0}):
        wrist_up = ctx.part_world_position(wrist)

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_pitch: 0.80}):
        wrist_up_aabb = ctx.part_world_aabb(wrist)
        tool_up = wrist_up_aabb[0][2] if wrist_up_aabb is not None else None

    shoulder_ok = elbow_rest is not None and elbow_up is not None and elbow_up[2] > elbow_rest[2] + 0.15
    ctx.check(
        "shoulder positive motion raises the elbow",
        shoulder_ok,
        details=f"rest={elbow_rest}, raised={elbow_up}",
    )

    elbow_ok = wrist_rest is not None and wrist_up is not None and wrist_up[2] > wrist_rest[2] + 0.12
    ctx.check(
        "elbow positive motion raises the wrist",
        elbow_ok,
        details=f"rest={wrist_rest}, raised={wrist_up}",
    )

    wrist_ok = tool_rest is not None and tool_up is not None and tool_up > tool_rest + 0.020
    ctx.check(
        "wrist positive motion lifts the underslung tool face",
        wrist_ok,
        details=f"rest_z={tool_rest}, raised_z={tool_up}",
    )

    wrist_drop_ok = tool_rest is not None and tool_rest < -0.060
    ctx.check(
        "tool face sits below the wrist joint in the rest pose",
        wrist_drop_ok,
        details=f"tool_face_center_z={tool_rest}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
