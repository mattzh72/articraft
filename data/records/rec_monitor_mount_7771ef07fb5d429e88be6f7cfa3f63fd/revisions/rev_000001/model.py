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


POST_RADIUS = 0.016
POST_HEIGHT = 0.40
ARM_HEIGHT = 0.185
COLLAR_LENGTH = 0.042
PRIMARY_LENGTH = 0.365
SECONDARY_LENGTH = 0.225
TILT_OFFSET_X = 0.045
TILT_OFFSET_Z = 0.030


def _annulus(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length + 0.002).translate((0.0, 0.0, -0.001))
    return outer.cut(inner)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def make_clamp_post_shape() -> cq.Workplane:
    clamp_body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.070, 0.018),
                (-0.010, 0.018),
                (-0.010, 0.005),
                (-0.028, 0.005),
                (-0.028, -0.054),
                (-0.010, -0.054),
                (-0.010, -0.066),
                (-0.070, -0.066),
            ]
        )
        .close()
        .extrude(0.056, both=True)
        .edges("|Y")
        .fillet(0.004)
    )

    socket = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.0))
    )
    gusset = (
        cq.Workplane("XY")
        .box(0.026, 0.042, 0.028)
        .edges("|Y")
        .fillet(0.005)
        .translate((-0.018, 0.0, 0.012))
    )
    post = (
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(POST_HEIGHT)
        .translate((0.0, 0.0, 0.016))
    )
    top_cap = (
        cq.Workplane("XY")
        .circle(POST_RADIUS * 1.08)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.016 + POST_HEIGHT))
    )
    stop_ring = _annulus(0.024, POST_RADIUS + 0.0016, 0.006).translate(
        (0.0, 0.0, ARM_HEIGHT - COLLAR_LENGTH / 2.0 - 0.006)
    )

    clamp_screw = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.040)
        .translate((-0.024, 0.0, -0.106))
    )
    pressure_pad = (
        cq.Workplane("XY")
        .circle(0.011)
        .extrude(0.004)
        .translate((-0.024, 0.0, -0.110))
    )
    handle_bar = (
        cq.Workplane("XZ")
        .circle(0.0035)
        .extrude(0.040)
        .translate((-0.024, -0.020, -0.104))
    )

    return (
        clamp_body.union(socket)
        .union(gusset)
        .union(post)
        .union(top_cap)
        .union(stop_ring)
        .union(clamp_screw)
        .union(pressure_pad)
        .union(handle_bar)
    )


def make_primary_link_shape() -> cq.Workplane:
    collar = _annulus(0.024, POST_RADIUS + 0.0022, COLLAR_LENGTH).translate(
        (0.0, 0.0, -COLLAR_LENGTH / 2.0)
    )
    collar_split = cq.Workplane("XY").box(0.028, 0.010, COLLAR_LENGTH + 0.004).translate((0.018, 0.0, 0.0))
    collar = collar.cut(collar_split)
    clamp_block = (
        cq.Workplane("XY")
        .box(0.024, 0.040, 0.024)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.036, 0.0, 0.0))
    )
    clamp_bolt = _cylinder_y(0.0030, 0.016, (0.036, 0.028, 0.0))
    clamp_knob = cq.Workplane("XY").box(0.010, 0.010, 0.010).translate((0.036, 0.040, 0.0))
    arm_body = (
        cq.Workplane("XY")
        .box(0.248, 0.024, 0.012)
        .edges("|X")
        .fillet(0.003)
        .translate((0.186, 0.0, -0.010))
    )
    elbow_pad = (
        cq.Workplane("XY")
        .box(0.034, 0.024, 0.012)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.335, 0.0, -0.010))
    )
    elbow_disk = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.008)
        .translate((PRIMARY_LENGTH, 0.0, -0.008))
    )

    return collar.union(clamp_block).union(clamp_bolt).union(clamp_knob).union(arm_body).union(elbow_pad).union(elbow_disk)


def make_secondary_link_shape() -> cq.Workplane:
    elbow_disk = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.0))
    )
    rear_pad = (
        cq.Workplane("XY")
        .box(0.028, 0.022, 0.010)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.018, 0.0, 0.009))
    )
    arm_body = (
        cq.Workplane("XY")
        .box(0.142, 0.022, 0.012)
        .edges("|X")
        .fillet(0.003)
        .translate((0.110, 0.0, 0.010))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(0.028, 0.020, 0.010)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.197, 0.0, -0.001))
    )
    front_disk = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.008)
        .translate((SECONDARY_LENGTH, 0.0, -0.008))
    )

    return elbow_disk.union(rear_pad).union(arm_body).union(front_pad).union(front_disk)


def make_head_swivel_shape() -> cq.Workplane:
    lower_disk = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.026, 0.014, 0.010)
        .edges("|X")
        .fillet(0.002)
        .translate((0.020, 0.0, 0.016))
    )
    ear_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.010)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.036, 0.0, 0.024))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.008, 0.006, 0.020)
        .edges("|X")
        .fillet(0.0015)
        .translate((TILT_OFFSET_X, -0.015, TILT_OFFSET_Z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.008, 0.006, 0.020)
        .edges("|X")
        .fillet(0.0015)
        .translate((TILT_OFFSET_X, 0.015, TILT_OFFSET_Z))
    )

    return lower_disk.union(neck).union(ear_bridge).union(left_ear).union(right_ear)


def make_vesa_head_shape() -> cq.Workplane:
    tilt_barrel = _cylinder_y(0.006, 0.024, (0.0, 0.0, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(0.016, 0.012, 0.014)
        .edges("|X")
        .fillet(0.002)
        .translate((0.018, 0.0, 0.0))
    )
    frame_plate = (
        cq.Workplane("XY")
        .box(0.006, 0.125, 0.115)
        .edges("|X")
        .fillet(0.004)
        .translate((0.064, 0.0, 0.0))
    )
    frame_window = cq.Workplane("XY").box(0.010, 0.072, 0.052).translate((0.064, 0.0, 0.0))
    vesa_holes = _cylinder_x(0.0032, 0.012, (0.064, -0.050, -0.050))
    for y_pos in (-0.050, 0.050):
        for z_pos in (-0.050, 0.050):
            if y_pos == -0.050 and z_pos == -0.050:
                continue
            vesa_holes = vesa_holes.union(_cylinder_x(0.0032, 0.012, (0.064, y_pos, z_pos)))

    return tilt_barrel.union(bridge).union(frame_plate.cut(frame_window).cut(vesa_holes))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_post_monitor_arm")

    graphite = model.material("graphite", color=(0.16, 0.17, 0.18))
    matte_black = model.material("matte_black", color=(0.10, 0.10, 0.11))

    clamp_post = model.part("clamp_post")
    clamp_post.visual(
        mesh_from_cadquery(make_clamp_post_shape(), "monitor_arm_clamp_post"),
        material=graphite,
        name="clamp_post_body",
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(make_primary_link_shape(), "monitor_arm_primary_link"),
        material=matte_black,
        name="primary_link_body",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(make_secondary_link_shape(), "monitor_arm_secondary_link"),
        material=matte_black,
        name="secondary_link_body",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(make_head_swivel_shape(), "monitor_arm_head_swivel"),
        material=graphite,
        name="head_swivel_body",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        mesh_from_cadquery(make_vesa_head_shape(), "monitor_arm_vesa_head"),
        material=matte_black,
        name="vesa_head_frame",
    )

    model.articulation(
        "post_to_primary",
        ArticulationType.REVOLUTE,
        parent=clamp_post,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, ARM_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.9, upper=2.9),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-2.5, upper=2.5),
    )
    model.articulation(
        "secondary_to_head_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_swivel,
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-3.0, upper=3.0),
    )
    model.articulation(
        "head_swivel_to_vesa",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=vesa_head,
        origin=Origin(xyz=(TILT_OFFSET_X, 0.0, TILT_OFFSET_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.65, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    articulation_names = {joint.name for joint in object_model.articulations}

    required_parts = (
        "clamp_post",
        "primary_link",
        "secondary_link",
        "head_swivel",
        "vesa_head",
    )
    required_joints = (
        "post_to_primary",
        "primary_to_secondary",
        "secondary_to_head_swivel",
        "head_swivel_to_vesa",
    )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        "clamp_post",
        "primary_link",
        reason="Split collar is represented as a captured sleeve around the desk post without separate bearing hardware.",
    )
    ctx.allow_overlap(
        "primary_link",
        "secondary_link",
        reason="Elbow pivot uses nested joint housings that intentionally share the same compact pivot envelope.",
    )
    ctx.allow_overlap(
        "secondary_link",
        "head_swivel",
        reason="Head swivel base is modeled as a compact captured vertical pivot nested into the front link clevis.",
    )
    ctx.allow_overlap(
        "head_swivel",
        "vesa_head",
        reason="Tilt pivot is represented as a clevis-and-barrel joint with shared pivot volume instead of separate pin geometry.",
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

    for name in required_parts:
        ctx.check(f"{name} present", name in part_names, f"missing part: {name}")
    for name in required_joints:
        ctx.check(f"{name} present", name in articulation_names, f"missing articulation: {name}")

    clamp_post = object_model.get_part("clamp_post")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    head_swivel = object_model.get_part("head_swivel")
    vesa_head = object_model.get_part("vesa_head")

    collar_joint = object_model.get_articulation("post_to_primary")
    elbow_joint = object_model.get_articulation("primary_to_secondary")
    head_swivel_joint = object_model.get_articulation("secondary_to_head_swivel")
    tilt_joint = object_model.get_articulation("head_swivel_to_vesa")

    ctx.check("collar axis vertical", tuple(collar_joint.axis) == (0.0, 0.0, 1.0), f"axis={collar_joint.axis}")
    ctx.check("elbow axis vertical", tuple(elbow_joint.axis) == (0.0, 0.0, 1.0), f"axis={elbow_joint.axis}")
    ctx.check(
        "head swivel axis vertical",
        tuple(head_swivel_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={head_swivel_joint.axis}",
    )
    ctx.check("head tilt axis horizontal", tuple(tilt_joint.axis) == (0.0, 1.0, 0.0), f"axis={tilt_joint.axis}")

    ctx.expect_contact(primary_link, clamp_post, name="primary link seated on collar stop")
    ctx.expect_contact(primary_link, secondary_link, name="elbow joint stack connected")
    ctx.expect_contact(secondary_link, head_swivel, name="head swivel stack connected")
    ctx.expect_contact(head_swivel, vesa_head, name="tilt head connected")

    primary_aabb = ctx.part_world_aabb(primary_link)
    secondary_aabb = ctx.part_world_aabb(secondary_link)
    if primary_aabb is not None and secondary_aabb is not None:
        primary_dx = primary_aabb[1][0] - primary_aabb[0][0]
        secondary_dx = secondary_aabb[1][0] - secondary_aabb[0][0]
        ctx.check(
            "primary link longer than secondary",
            primary_dx > secondary_dx + 0.08,
            f"primary_dx={primary_dx:.4f}, secondary_dx={secondary_dx:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
