from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _add_forked_link(
    part,
    *,
    mesh_name: str,
    length: float,
    body_radius: float,
    body_rise: float,
    arm_material,
    joint_material,
) -> None:
    part.visual(
        Cylinder(radius=0.0105, length=0.013),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_material,
        name="proximal_barrel",
    )
    part.visual(
        Box((0.024, 0.018, 0.020)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=joint_material,
        name="proximal_block",
    )
    part.visual(
        _tube_mesh(
            mesh_name,
            [
                (0.014, 0.0, 0.0),
                (length * 0.34, 0.0, body_rise * 0.70),
                (length * 0.70, 0.0, body_rise),
                (length - 0.020, 0.0, 0.0),
            ],
            body_radius,
        ),
        material=arm_material,
        name="arm_tube",
    )
    part.visual(
        Box((0.028, 0.018, 0.018)),
        origin=Origin(xyz=(length - 0.026, 0.0, 0.0)),
        material=joint_material,
        name="distal_bridge",
    )
    part.visual(
        Box((0.012, 0.006, 0.030)),
        origin=Origin(xyz=(length - 0.006, 0.0095, 0.0)),
        material=joint_material,
        name="distal_ear_pos",
    )
    part.visual(
        Box((0.012, 0.006, 0.030)),
        origin=Origin(xyz=(length - 0.006, -0.0095, 0.0)),
        material=joint_material,
        name="distal_ear_neg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_clamp_gooseneck_lamp")

    matte_black = model.material("matte_black", rgba=(0.13, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(0.94, 0.92, 0.84, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.18, 0.19, 0.20, 1.0))

    link_1_length = 0.18
    link_2_length = 0.16
    link_3_length = 0.14

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.028, 0.022, 0.028)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
        material=graphite,
        name="shoulder_block",
    )
    clamp_base.visual(
        Box((0.014, 0.006, 0.032)),
        origin=Origin(xyz=(-0.006, 0.0095, 0.0)),
        material=graphite,
        name="shoulder_ear_pos",
    )
    clamp_base.visual(
        Box((0.014, 0.006, 0.032)),
        origin=Origin(xyz=(-0.006, -0.0095, 0.0)),
        material=graphite,
        name="shoulder_ear_neg",
    )
    clamp_base.visual(
        Box((0.030, 0.030, 0.122)),
        origin=Origin(xyz=(-0.042, 0.0, -0.060)),
        material=matte_black,
        name="rear_spine",
    )
    clamp_base.visual(
        Box((0.048, 0.030, 0.018)),
        origin=Origin(xyz=(-0.036, 0.0, -0.012)),
        material=matte_black,
        name="upper_jaw",
    )
    clamp_base.visual(
        Box((0.054, 0.030, 0.020)),
        origin=Origin(xyz=(-0.028, 0.0, -0.095)),
        material=matte_black,
        name="lower_jaw",
    )
    clamp_base.visual(
        Box((0.026, 0.022, 0.004)),
        origin=Origin(xyz=(-0.024, 0.0, -0.023)),
        material=soft_pad,
        name="upper_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(-0.010, 0.0, -0.112)),
        material=graphite,
        name="screw_boss",
    )
    clamp_base.visual(
        Cylinder(radius=0.0055, length=0.106),
        origin=Origin(xyz=(-0.010, 0.0, -0.078)),
        material=satin_aluminum,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(-0.010, 0.0, -0.028)),
        material=soft_pad,
        name="screw_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.0045, length=0.048),
        origin=Origin(xyz=(-0.010, 0.0, -0.131), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="screw_handle",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.085, 0.045, 0.145)),
        mass=0.9,
        origin=Origin(xyz=(-0.028, 0.0, -0.060)),
    )

    link_1 = model.part("link_1")
    _add_forked_link(
        link_1,
        mesh_name="lamp_link_1_tube",
        length=link_1_length,
        body_radius=0.0075,
        body_rise=0.012,
        arm_material=satin_aluminum,
        joint_material=graphite,
    )
    link_1.inertial = Inertial.from_geometry(
        Box((link_1_length + 0.020, 0.050, 0.050)),
        mass=0.38,
        origin=Origin(xyz=(link_1_length * 0.50, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    _add_forked_link(
        link_2,
        mesh_name="lamp_link_2_tube",
        length=link_2_length,
        body_radius=0.0070,
        body_rise=0.010,
        arm_material=satin_aluminum,
        joint_material=graphite,
    )
    link_2.inertial = Inertial.from_geometry(
        Box((link_2_length + 0.020, 0.048, 0.048)),
        mass=0.32,
        origin=Origin(xyz=(link_2_length * 0.50, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    _add_forked_link(
        link_3,
        mesh_name="lamp_link_3_tube",
        length=link_3_length,
        body_radius=0.0063,
        body_rise=0.009,
        arm_material=satin_aluminum,
        joint_material=graphite,
    )
    link_3.inertial = Inertial.from_geometry(
        Box((link_3_length + 0.020, 0.046, 0.046)),
        mass=0.27,
        origin=Origin(xyz=(link_3_length * 0.50, 0.0, 0.0)),
    )

    led_head = model.part("led_head")
    led_head.visual(
        Cylinder(radius=0.010, length=0.013),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="head_barrel",
    )
    led_head.visual(
        Box((0.024, 0.018, 0.016)),
        origin=Origin(xyz=(0.011, 0.0, -0.004)),
        material=graphite,
        name="head_stem",
    )
    led_head.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    led_head.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.032, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="puck_body",
    )
    led_head.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.042, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_diffuser,
        name="puck_diffuser",
    )
    led_head.visual(
        Cylinder(radius=0.029, length=0.003),
        origin=Origin(xyz=(0.034, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="front_bezel",
    )
    led_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.060)),
        mass=0.24,
        origin=Origin(xyz=(0.030, 0.0, -0.004)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.65, upper=1.20),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_1_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(link_2_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=led_head,
        origin=Origin(xyz=(link_3_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-1.80, upper=1.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    clamp_base = object_model.get_part("clamp_base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    led_head = object_model.get_part("led_head")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

    ctx.expect_contact(clamp_base, link_1, name="clamp shoulder hinge stays in contact")
    ctx.expect_contact(link_1, link_2, name="first elbow hinge stays in contact")
    ctx.expect_contact(link_2, link_3, name="second elbow hinge stays in contact")
    ctx.expect_contact(link_3, led_head, name="head hinge stays in contact")

    head_rest = ctx.part_world_position(led_head)
    with ctx.pose({shoulder_joint: 0.75}):
        head_raised = ctx.part_world_position(led_head)
    ctx.check(
        "shoulder joint lifts the lamp upward",
        head_rest is not None and head_raised is not None and head_raised[2] > head_rest[2] + 0.20,
        details=f"rest={head_rest}, raised={head_raised}",
    )

    with ctx.pose({shoulder_joint: 0.35, elbow_joint: 0.85}):
        elbow_up = ctx.part_world_position(led_head)
    with ctx.pose({shoulder_joint: 0.35, elbow_joint: -0.75}):
        elbow_down = ctx.part_world_position(led_head)
    ctx.check(
        "elbow joint changes head height",
        elbow_up is not None and elbow_down is not None and elbow_up[2] > elbow_down[2] + 0.10,
        details=f"elbow_up={elbow_up}, elbow_down={elbow_down}",
    )

    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.25, wrist_joint: 0.80}):
        wrist_up = ctx.part_world_position(led_head)
    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.25, wrist_joint: -0.75}):
        wrist_down = ctx.part_world_position(led_head)
    ctx.check(
        "wrist joint redirects the final link",
        wrist_up is not None and wrist_down is not None and wrist_up[2] > wrist_down[2] + 0.05,
        details=f"wrist_up={wrist_up}, wrist_down={wrist_down}",
    )

    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.10, wrist_joint: -0.10, head_tilt_joint: 0.70}):
        tilted_up = ctx.part_element_world_aabb(led_head, elem="puck_diffuser")
    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.10, wrist_joint: -0.10, head_tilt_joint: -0.70}):
        tilted_down = ctx.part_element_world_aabb(led_head, elem="puck_diffuser")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return 0.5 * (mins[2] + maxs[2])

    diffuser_up = _aabb_center_z(tilted_up)
    diffuser_down = _aabb_center_z(tilted_down)
    ctx.check(
        "head tilt rotates the LED puck",
        diffuser_up is not None and diffuser_down is not None and diffuser_up > diffuser_down + 0.01,
        details=f"diffuser_up={diffuser_up}, diffuser_down={diffuser_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
