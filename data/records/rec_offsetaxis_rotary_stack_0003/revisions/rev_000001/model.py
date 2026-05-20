from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def _cyl_z(radius: float, height: float, z0: float, x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _cyl_y(radius: float, length: float, y0: float, *, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length).translate((0.0, y0, 0.0))


def _cyl_x(radius: float, length: float, x0: float, *, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def _add_xy_bolts(
    body: cq.Workplane,
    points: list[tuple[float, float]],
    *,
    z0: float,
    radius: float,
    height: float,
) -> cq.Workplane:
    for x, y in points:
        body = body.union(_cyl_z(radius, height, z0, x=x, y=y))
    return body


def _add_xz_bolts(
    body: cq.Workplane,
    points: list[tuple[float, float]],
    *,
    y0: float,
    radius: float,
    height: float,
) -> cq.Workplane:
    for x, z in points:
        body = body.union(_cyl_y(radius, height, y0, x=x, z=z))
    return body


def _add_yz_bolts(
    body: cq.Workplane,
    points: list[tuple[float, float]],
    *,
    x0: float,
    radius: float,
    height: float,
) -> cq.Workplane:
    for y, z in points:
        body = body.union(_cyl_x(radius, height, x0, y=y, z=z))
    return body


def _make_base_frame() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.36, 0.28, 0.02).translate((0.0, 0.0, 0.01))

    for x in (-0.105, 0.105):
        body = body.union(cq.Workplane("XY").box(0.07, 0.06, 0.014).translate((x, 0.0, 0.007)))

    body = body.union(cq.Workplane("XY").box(0.16, 0.12, 0.16).translate((0.0, 0.0, 0.10)))
    body = body.union(cq.Workplane("XY").box(0.11, 0.11, 0.03).translate((0.0, 0.0, 0.175)))
    body = body.union(_cyl_z(0.056, 0.025, 0.19))

    for x in (-0.055, 0.055):
        body = body.union(cq.Workplane("XY").box(0.05, 0.08, 0.08).translate((x, 0.0, 0.06)))

    body = body.cut(_cyl_z(0.03, 0.03, 0.02))
    body = body.cut(_cyl_z(0.036, 0.13, 0.05))
    body = body.cut(_cyl_z(0.03, 0.035, 0.18))
    body = body.cut(cq.Workplane("XY").box(0.10, 0.03, 0.08).translate((0.0, -0.075, 0.11)))

    bolt_circle = []
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = math.radians(angle_deg)
        bolt_circle.append((0.044 * math.cos(angle), 0.044 * math.sin(angle)))
    body = _add_xy_bolts(body, bolt_circle, z0=0.215, radius=0.0045, height=0.004)

    return body


def _make_base_cover() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.116, 0.008, 0.096)
    body = body.union(cq.Workplane("XY").box(0.072, 0.004, 0.052).translate((0.0, -0.006, 0.0)))
    body = _add_xz_bolts(
        body,
        [(-0.045, -0.034), (0.045, -0.034), (-0.045, 0.034), (0.045, 0.034)],
        y0=-0.008,
        radius=0.004,
        height=0.004,
    )
    return body


def _make_stage1_rotor() -> cq.Workplane:
    body = _cyl_z(0.03, 0.20, -0.12)
    body = body.union(_cyl_z(0.042, 0.008, 0.075))
    body = body.union(_cyl_z(0.055, 0.045, 0.085))
    body = body.union(cq.Workplane("XY").box(0.12, 0.05, 0.024).translate((0.08, 0.0, 0.10)))
    body = body.union(cq.Workplane("XY").box(0.026, 0.07, 0.14).translate((0.11, 0.0, 0.16)))
    body = body.union(cq.Workplane("XY").box(0.05, 0.10, 0.02).translate((0.12, 0.0, 0.145)))
    body = body.union(cq.Workplane("XY").box(0.06, 0.14, 0.02).translate((0.14, 0.0, 0.235)))
    body = body.union(cq.Workplane("XY").box(0.05, 0.03, 0.07).translate((0.14, 0.055, 0.19)))
    body = body.union(cq.Workplane("XY").box(0.05, 0.03, 0.07).translate((0.14, -0.055, 0.19)))
    body = body.union(cq.Workplane("XY").box(0.04, 0.06, 0.03).translate((0.11, 0.0, 0.18)))
    body = body.union(
        cq.Workplane("XZ")
        .polyline([(0.03, 0.085), (0.10, 0.085), (0.11, 0.18), (0.06, 0.18)])
        .close()
        .extrude(0.06)
        .translate((0.0, -0.03, 0.0))
    )

    body = body.cut(_cyl_y(0.022, 0.16, -0.08, x=0.14, z=0.19))
    body = body.cut(cq.Workplane("XY").box(0.035, 0.06, 0.06).translate((0.11, 0.0, 0.115)))

    bolt_circle = []
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = math.radians(angle_deg)
        bolt_circle.append((0.042 * math.cos(angle), 0.042 * math.sin(angle)))
    body = _add_xy_bolts(body, bolt_circle, z0=0.13, radius=0.004, height=0.004)

    return body


def _make_stage2_cover() -> cq.Workplane:
    body = _cyl_y(0.034, 0.008, -0.004)
    body = body.union(_cyl_y(0.022, 0.005, 0.004, x=0.0, z=0.0))
    cap_bolts: list[tuple[float, float]] = []
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = math.radians(angle_deg)
        cap_bolts.append((0.023 * math.cos(angle), 0.023 * math.sin(angle)))
    body = _add_xz_bolts(body, cap_bolts, y0=0.004, radius=0.0035, height=0.004)
    return body


def _make_stage2_rotor() -> cq.Workplane:
    body = _cyl_y(0.022, 0.16, -0.08)
    body = body.union(_cyl_y(0.03, 0.012, -0.036))
    body = body.union(_cyl_y(0.03, 0.012, 0.024))
    body = body.union(cq.Workplane("XY").box(0.065, 0.08, 0.08).translate((0.015, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.15, 0.045, 0.024).translate((0.08, 0.0, 0.045)))
    body = body.union(cq.Workplane("XY").box(0.06, 0.07, 0.03).translate((0.07, 0.0, 0.07)))
    body = body.union(cq.Workplane("XY").box(0.028, 0.05, 0.10).translate((0.14, 0.0, 0.095)))
    body = body.union(cq.Workplane("XY").box(0.09, 0.08, 0.072).translate((0.15, 0.0, 0.12)))
    body = body.union(cq.Workplane("XY").box(0.06, 0.09, 0.018).translate((0.15, 0.0, 0.165)))
    body = body.union(cq.Workplane("XY").box(0.04, 0.09, 0.03).translate((0.11, 0.0, 0.135)))
    body = body.union(
        cq.Workplane("XZ")
        .polyline([(0.04, 0.03), (0.13, 0.03), (0.15, 0.13), (0.08, 0.13)])
        .close()
        .extrude(0.05)
        .translate((0.0, -0.025, 0.0))
    )

    body = body.cut(_cyl_z(0.019, 0.02, 0.084, x=0.15, y=0.0))
    body = body.cut(_cyl_z(0.024, 0.032, 0.104, x=0.15, y=0.0))
    body = body.cut(_cyl_z(0.019, 0.03, 0.136, x=0.15, y=0.0))
    body = body.cut(cq.Workplane("XY").box(0.03, 0.028, 0.05).translate((0.195, 0.0, 0.12)))

    cap_bolts = []
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = math.radians(angle_deg)
        cap_bolts.append((0.15 + 0.026 * math.cos(angle), 0.026 * math.sin(angle)))
    body = _add_xy_bolts(body, cap_bolts, z0=0.174, radius=0.0035, height=0.003)

    return body


def _make_stage3_cover() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.008, 0.062, 0.078)
    body = body.union(cq.Workplane("XY").box(0.004, 0.038, 0.042).translate((0.006, 0.0, 0.0)))
    body = _add_yz_bolts(
        body,
        [(-0.022, -0.028), (0.022, -0.028), (-0.022, 0.028), (0.022, 0.028)],
        x0=0.004,
        radius=0.0035,
        height=0.004,
    )
    return body


def _make_stage3_core() -> cq.Workplane:
    body = _cyl_z(0.019, 0.12, -0.06)
    body = body.union(_cyl_z(0.032, 0.014, -0.045))
    body = body.union(_cyl_z(0.036, 0.05, -0.005))
    body = body.union(_cyl_z(0.052, 0.016, 0.032))
    body = body.union(cq.Workplane("XY").box(0.045, 0.04, 0.018).translate((0.022, 0.0, 0.045)))

    bolt_circle = []
    for angle_deg in (0.0, 120.0, 240.0):
        angle = math.radians(angle_deg)
        bolt_circle.append((0.036 * math.cos(angle), 0.036 * math.sin(angle)))
    body = _add_xy_bolts(body, bolt_circle, z0=0.048, radius=0.0035, height=0.003)

    return body


def _make_stage3_arm() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.105, 0.038, 0.018).translate((0.068, 0.0, 0.045))
    body = body.union(cq.Workplane("XY").box(0.026, 0.06, 0.04).translate((0.118, 0.0, 0.05)))
    body = body.cut(cq.Workplane("XY").box(0.012, 0.026, 0.022).translate((0.118, 0.0, 0.05)))
    body = _add_xy_bolts(body, [(0.118, -0.018), (0.118, 0.018)], z0=0.07, radius=0.003, height=0.003)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack", assets=ASSETS)

    iron = model.material("iron", rgba=(0.30, 0.32, 0.35, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    coated = model.material("coated_bracket", rgba=(0.37, 0.41, 0.46, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.18, 0.20, 0.22, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame.obj", assets=ASSETS),
        material=iron,
        name="structure",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 0.22)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    base_access_cover = model.part("base_access_cover")
    base_access_cover.visual(
        mesh_from_cadquery(_make_base_cover(), "base_access_cover.obj", assets=ASSETS),
        material=cover_finish,
        name="cover",
    )
    base_access_cover.inertial = Inertial.from_geometry(
        Box((0.116, 0.008, 0.096)),
        mass=0.6,
    )

    stage1_rotor = model.part("stage1_rotor")
    stage1_rotor.visual(
        mesh_from_cadquery(_make_stage1_rotor(), "stage1_rotor.obj", assets=ASSETS),
        material=coated,
        name="rotor",
    )
    stage1_rotor.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.38)),
        mass=6.0,
        origin=Origin(xyz=(0.10, 0.0, 0.14)),
    )

    stage2_access_cover = model.part("stage2_access_cover")
    stage2_access_cover.visual(
        mesh_from_cadquery(_make_stage2_cover(), "stage2_access_cover.obj", assets=ASSETS),
        material=cover_finish,
        name="cover",
    )
    stage2_access_cover.inertial = Inertial.from_geometry(
        Box((0.068, 0.012, 0.068)),
        mass=0.35,
    )

    stage2_rotor = model.part("stage2_rotor")
    stage2_rotor.visual(
        mesh_from_cadquery(_make_stage2_rotor(), "stage2_rotor.obj", assets=ASSETS),
        material=machined,
        name="rotor",
    )
    stage2_rotor.inertial = Inertial.from_geometry(
        Box((0.25, 0.16, 0.20)),
        mass=4.5,
        origin=Origin(xyz=(0.11, 0.0, 0.10)),
    )

    stage3_access_cover = model.part("stage3_access_cover")
    stage3_access_cover.visual(
        mesh_from_cadquery(_make_stage3_cover(), "stage3_access_cover.obj", assets=ASSETS),
        material=cover_finish,
        name="cover",
    )
    stage3_access_cover.inertial = Inertial.from_geometry(
        Box((0.012, 0.062, 0.078)),
        mass=0.25,
    )

    stage3_rotor = model.part("stage3_rotor")
    stage3_rotor.visual(
        mesh_from_cadquery(_make_stage3_core(), "stage3_rotor_core.obj", assets=ASSETS),
        material=machined,
        name="core",
    )
    stage3_rotor.visual(
        mesh_from_cadquery(_make_stage3_arm(), "stage3_rotor_arm.obj", assets=ASSETS),
        material=machined,
        name="output_arm",
    )
    stage3_rotor.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.12)),
        mass=2.7,
        origin=Origin(xyz=(0.05, 0.0, 0.04)),
    )

    model.articulation(
        "base_cover_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=base_access_cover,
        origin=Origin(xyz=(0.0, -0.064, 0.11)),
    )
    model.articulation(
        "stage1_axis",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=stage1_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.6, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "stage2_cover_mount",
        ArticulationType.FIXED,
        parent=stage1_rotor,
        child=stage2_access_cover,
        origin=Origin(xyz=(0.14, 0.074, 0.19)),
    )
    model.articulation(
        "stage2_axis",
        ArticulationType.REVOLUTE,
        parent=stage1_rotor,
        child=stage2_rotor,
        origin=Origin(xyz=(0.14, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "stage3_cover_mount",
        ArticulationType.FIXED,
        parent=stage2_rotor,
        child=stage3_access_cover,
        origin=Origin(xyz=(0.199, 0.0, 0.12)),
    )
    model.articulation(
        "stage3_axis",
        ArticulationType.REVOLUTE,
        parent=stage2_rotor,
        child=stage3_rotor,
        origin=Origin(xyz=(0.15, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.0, lower=-2.8, upper=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    base_access_cover = object_model.get_part("base_access_cover")
    stage1_rotor = object_model.get_part("stage1_rotor")
    stage2_access_cover = object_model.get_part("stage2_access_cover")
    stage2_rotor = object_model.get_part("stage2_rotor")
    stage3_access_cover = object_model.get_part("stage3_access_cover")
    stage3_rotor = object_model.get_part("stage3_rotor")

    stage1_axis = object_model.get_articulation("stage1_axis")
    stage2_axis = object_model.get_articulation("stage2_axis")
    stage3_axis = object_model.get_articulation("stage3_axis")

    stage3_output_arm = stage3_rotor.get_visual("output_arm")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    ctx.expect_contact(base_access_cover, base_frame, name="base access cover seats on housing face")
    ctx.expect_overlap(base_access_cover, base_frame, axes="xz", min_overlap=0.09, name="base cover footprint aligns with housing")
    ctx.expect_within(base_access_cover, base_frame, axes="xz", margin=0.02, name="base cover stays inside housing envelope")

    ctx.expect_contact(stage1_rotor, base_frame, name="stage1 rotor is supported by base journals")
    ctx.expect_contact(stage2_access_cover, stage1_rotor, name="stage2 bearing cap mounts to first transfer bracket")
    ctx.expect_overlap(stage2_access_cover, stage1_rotor, axes="xz", min_overlap=0.05, name="stage2 cap overlaps carrier block footprint")
    ctx.expect_contact(stage2_rotor, stage1_rotor, name="stage2 trunnion is supported by bracket carrier")

    ctx.expect_contact(stage3_access_cover, stage2_rotor, name="stage3 access cover mounts to second stage housing")
    ctx.expect_overlap(stage3_access_cover, stage2_rotor, axes="yz", min_overlap=0.05, name="stage3 cover aligns with side opening")
    ctx.expect_contact(stage3_rotor, stage2_rotor, name="stage3 rotor is supported by offset housing journals")

    with ctx.pose({stage1_axis: 1.0, stage2_axis: -0.4, stage3_axis: 1.6}):
        ctx.expect_contact(stage1_rotor, base_frame, name="stage1 support contact persists in posed assembly")
        ctx.expect_contact(stage2_rotor, stage1_rotor, name="stage2 support contact persists in posed assembly")
        ctx.expect_contact(stage3_rotor, stage2_rotor, name="stage3 support contact persists in posed assembly")
        ctx.expect_gap(stage2_rotor, base_frame, axis="z", min_gap=0.02, name="stage2 rotor clears the base in motion")
        ctx.expect_gap(stage3_rotor, base_frame, axis="z", min_gap=0.02, name="stage3 rotor clears the base in motion")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({stage1_axis: 0.0, stage2_axis: 0.0, stage3_axis: 0.0}):
        stage2_pos_0 = ctx.part_world_position(stage2_rotor)
        stage3_pos_0 = ctx.part_world_position(stage3_rotor)
        arm_center_0 = _aabb_center(ctx.part_element_world_aabb(stage3_rotor, elem=stage3_output_arm))

    with ctx.pose({stage1_axis: 1.0, stage2_axis: 0.0, stage3_axis: 0.0}):
        stage2_pos_1 = ctx.part_world_position(stage2_rotor)

    with ctx.pose({stage1_axis: 0.0, stage2_axis: 0.55, stage3_axis: 0.0}):
        stage2_pos_pitch = ctx.part_world_position(stage2_rotor)
        stage3_pos_pitch = ctx.part_world_position(stage3_rotor)

    with ctx.pose({stage1_axis: 0.0, stage2_axis: 0.0, stage3_axis: math.pi / 2.0}):
        stage3_axis_pos_1 = ctx.part_world_position(stage3_rotor)
        arm_center_1 = _aabb_center(ctx.part_element_world_aabb(stage3_rotor, elem=stage3_output_arm))

    if stage2_pos_0 and stage2_pos_1:
        radial_0 = math.hypot(stage2_pos_0[0], stage2_pos_0[1])
        radial_1 = math.hypot(stage2_pos_1[0], stage2_pos_1[1])
        ctx.check(
            "stage1 rotates offset second axis around base centerline",
            abs(radial_0 - radial_1) <= 0.005 and radial_0 >= 0.12 and abs(stage2_pos_1[1] - stage2_pos_0[1]) >= 0.10,
            details=f"stage2 positions at stage1 poses: {stage2_pos_0} -> {stage2_pos_1}",
        )
    else:
        ctx.fail("stage1 rotates offset second axis around base centerline", "could not resolve stage2 world position")

    if stage2_pos_0 and stage3_pos_0 and stage2_pos_pitch and stage3_pos_pitch:
        rel_0 = (stage3_pos_0[0] - stage2_pos_0[0], stage3_pos_0[2] - stage2_pos_0[2])
        rel_1 = (stage3_pos_pitch[0] - stage2_pos_pitch[0], stage3_pos_pitch[2] - stage2_pos_pitch[2])
        span_0 = math.hypot(rel_0[0], rel_0[1])
        span_1 = math.hypot(rel_1[0], rel_1[1])
        ctx.check(
            "stage2 pitches third stage along an xz arc",
            abs(span_0 - span_1) <= 0.005
            and abs(rel_1[0] - rel_0[0]) >= 0.03
            and abs(rel_1[1] - rel_0[1]) >= 0.05,
            details=f"stage3 offsets relative to stage2: {rel_0} -> {rel_1}",
        )
    else:
        ctx.fail("stage2 pitches third stage along an xz arc", "could not resolve stage2 or stage3 world positions")

    if stage3_pos_0 and stage3_axis_pos_1 and arm_center_0 and arm_center_1:
        ctx.check(
            "stage3 spins eccentric output arm about the final vertical axis",
            arm_center_0[0] - stage3_pos_0[0] >= 0.05
            and abs(arm_center_0[1] - stage3_pos_0[1]) <= 0.025
            and abs(arm_center_1[0] - stage3_axis_pos_1[0]) <= 0.04
            and abs(arm_center_1[1] - stage3_axis_pos_1[1]) >= 0.04,
            details=f"output arm centers: rest={arm_center_0}, quarter_turn={arm_center_1}",
        )
    else:
        ctx.fail("stage3 spins eccentric output arm about the final vertical axis", "could not resolve stage3 output arm bounds")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
