from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_bench_vise")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.24, 0.30, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    def x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))

    def tube_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
        geom = LatheGeometry.from_shell_profiles(
            [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
            [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    left_sleeve_mesh = tube_shell(
        "left_guide_sleeve",
        outer_radius=0.016,
        inner_radius=0.0103,
        length=0.092,
    )
    right_sleeve_mesh = tube_shell(
        "right_guide_sleeve",
        outer_radius=0.016,
        inner_radius=0.0103,
        length=0.092,
    )
    screw_nut_mesh = tube_shell(
        "screw_nut_housing",
        outer_radius=0.015,
        inner_radius=0.0086,
        length=0.100,
    )
    handle_mount_mesh = tube_shell(
        "handle_mount_boss_shell",
        outer_radius=0.017,
        inner_radius=0.0088,
        length=0.028,
    )
    handle_hub_mesh = tube_shell(
        "handle_hub_shell",
        outer_radius=0.018,
        inner_radius=0.0088,
        length=0.024,
    )

    body = model.part("body")
    body.visual(
        Box((0.28, 0.23, 0.02)),
        origin=Origin(xyz=(-0.13, 0.0, 0.01)),
        material=cast_iron,
        name="mount_base",
    )
    body.visual(
        Box((0.18, 0.19, 0.07)),
        origin=Origin(xyz=(-0.10, 0.0, 0.045)),
        material=cast_iron,
        name="pedestal",
    )
    body.visual(
        Box((0.06, 0.18, 0.105)),
        origin=Origin(xyz=(-0.03, 0.0, 0.1275)),
        material=cast_iron,
        name="fixed_jaw_block",
    )
    body.visual(
        Box((0.008, 0.15, 0.045)),
        origin=Origin(xyz=(-0.004, 0.0, 0.125)),
        material=jaw_steel,
        name="fixed_jaw_plate",
    )
    body.visual(
        Box((0.06, 0.08, 0.015)),
        origin=Origin(xyz=(-0.045, 0.045, 0.1825)),
        material=jaw_steel,
        name="anvil_pad",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=x_axis_origin((-0.010, -0.050, 0.048)),
        material=cast_iron,
        name="left_bar_bearing",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=x_axis_origin((-0.010, 0.050, 0.048)),
        material=cast_iron,
        name="right_bar_bearing",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=x_axis_origin((-0.014, 0.0, 0.024)),
        material=cast_iron,
        name="screw_bearing_boss",
    )
    body.visual(
        Box((0.035, 0.07, 0.10)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.092)),
        material=cast_iron,
        name="throat_web",
    )
    body.visual(
        Cylinder(radius=0.0098, length=0.180),
        origin=x_axis_origin((0.09, -0.050, 0.048)),
        material=blackened_steel,
        name="left_guide_bar",
    )
    body.visual(
        Cylinder(radius=0.0098, length=0.180),
        origin=x_axis_origin((0.09, 0.050, 0.048)),
        material=blackened_steel,
        name="right_guide_bar",
    )
    body.visual(
        Cylinder(radius=0.0080, length=0.112),
        origin=x_axis_origin((0.056, 0.0, 0.024)),
        material=blackened_steel,
        name="lead_screw",
    )
    body.visual(
        Box((0.034, 0.022, 0.040)),
        origin=Origin(xyz=(0.008, 0.091, 0.036)),
        material=cast_iron,
        name="lever_bracket",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.28, 0.23, 0.19)),
        mass=18.0,
        origin=Origin(xyz=(-0.13, 0.0, 0.095)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.052, 0.18, 0.085)),
        origin=Origin(xyz=(0.034, 0.0, 0.1275)),
        material=cast_iron,
        name="moving_jaw_block",
    )
    moving_jaw.visual(
        Box((0.008, 0.15, 0.045)),
        origin=Origin(xyz=(0.004, 0.0, 0.125)),
        material=jaw_steel,
        name="moving_jaw_plate",
    )
    moving_jaw.visual(
        Box((0.050, 0.12, 0.026)),
        origin=Origin(xyz=(0.060, 0.0, 0.074)),
        material=cast_iron,
        name="carriage_bridge",
    )
    moving_jaw.visual(
        Box((0.046, 0.016, 0.044)),
        origin=Origin(xyz=(0.056, -0.023, 0.040)),
        material=cast_iron,
        name="left_center_web",
    )
    moving_jaw.visual(
        Box((0.046, 0.016, 0.044)),
        origin=Origin(xyz=(0.056, 0.023, 0.040)),
        material=cast_iron,
        name="right_center_web",
    )
    moving_jaw.visual(
        left_sleeve_mesh,
        origin=Origin(xyz=(0.076, -0.050, 0.048)),
        material=cast_iron,
        name="left_guide_sleeve",
    )
    moving_jaw.visual(
        right_sleeve_mesh,
        origin=Origin(xyz=(0.076, 0.050, 0.048)),
        material=cast_iron,
        name="right_guide_sleeve",
    )
    moving_jaw.visual(
        screw_nut_mesh,
        origin=Origin(xyz=(0.072, 0.0, 0.024)),
        material=cast_iron,
        name="screw_nut_housing",
    )
    moving_jaw.visual(
        handle_mount_mesh,
        origin=Origin(xyz=(0.108, 0.0, 0.024)),
        material=cast_iron,
        name="handle_mount_boss",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.12, 0.18, 0.13)),
        mass=6.0,
        origin=Origin(xyz=(0.060, 0.0, 0.085)),
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=x_axis_origin((0.0, 0.0, 0.0)),
        material=blackened_steel,
        name="lever_pivot",
    )
    release_lever.visual(
        Box((0.012, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, 0.036, -0.012), rpy=(-0.28, 0.0, 0.0)),
        material=cast_iron,
        name="lever_arm",
    )
    release_lever.visual(
        Cylinder(radius=0.0075, length=0.028),
        origin=x_axis_origin((0.0, 0.070, -0.022)),
        material=grip_black,
        name="lever_grip",
    )
    release_lever.inertial = Inertial.from_geometry(
        Box((0.02, 0.085, 0.035)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.040, -0.012)),
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        handle_hub_mesh,
        origin=Origin(),
        material=blackened_steel,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.0055, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="tommy_bar",
    )
    screw_handle.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        material=grip_black,
        name="handle_knob_pos",
    )
    screw_handle.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=grip_black,
        name="handle_knob_neg",
    )
    screw_handle.inertial = Inertial.from_geometry(
        Box((0.03, 0.19, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.12,
            lower=0.0,
            upper=0.165,
        ),
    )
    model.articulation(
        "body_to_release_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=release_lever,
        origin=Origin(xyz=(0.008, 0.112, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.90,
        ),
    )
    model.articulation(
        "moving_jaw_to_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(0.134, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    moving_jaw = object_model.get_part("moving_jaw")
    release_lever = object_model.get_part("release_lever")
    screw_handle = object_model.get_part("screw_handle")

    slide = object_model.get_articulation("body_to_moving_jaw")
    lever_joint = object_model.get_articulation("body_to_release_lever")
    handle_joint = object_model.get_articulation("moving_jaw_to_screw_handle")

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            moving_jaw,
            body,
            axis="x",
            positive_elem="moving_jaw_plate",
            negative_elem="fixed_jaw_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="jaw plates meet in the closed pose",
        )
        ctx.expect_overlap(
            moving_jaw,
            body,
            axes="yz",
            elem_a="moving_jaw_plate",
            elem_b="fixed_jaw_plate",
            min_overlap=0.040,
            name="jaw plates stay aligned across height and width",
        )
        rest_jaw_pos = ctx.part_world_position(moving_jaw)
        rest_handle_pos = ctx.part_world_position(screw_handle)
        rest_lever_aabb = ctx.part_element_world_aabb(release_lever, elem="lever_grip")

    if slide_upper is not None:
        with ctx.pose({slide: slide_upper}):
            ctx.expect_gap(
                moving_jaw,
                body,
                axis="x",
                positive_elem="moving_jaw_plate",
                negative_elem="fixed_jaw_plate",
                min_gap=0.160,
                max_gap=0.166,
                name="quick release jaw opens to a wide gap",
            )
            ctx.expect_overlap(
                moving_jaw,
                body,
                axes="yz",
                elem_a="moving_jaw_plate",
                elem_b="fixed_jaw_plate",
                min_overlap=0.040,
                name="moving jaw remains co-planar with the fixed jaw when extended",
            )
            extended_jaw_pos = ctx.part_world_position(moving_jaw)
            extended_handle_pos = ctx.part_world_position(screw_handle)

        ctx.check(
            "moving jaw translates forward along the slide axis",
            rest_jaw_pos is not None
            and extended_jaw_pos is not None
            and extended_jaw_pos[0] > rest_jaw_pos[0] + 0.15,
            details=f"rest={rest_jaw_pos}, extended={extended_jaw_pos}",
        )
        ctx.check(
            "handle assembly travels with the moving jaw",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[0] > rest_handle_pos[0] + 0.15,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    with ctx.pose({lever_joint: 0.75}):
        raised_lever_aabb = ctx.part_element_world_aabb(release_lever, elem="lever_grip")

    if rest_lever_aabb is not None and raised_lever_aabb is not None:
        ctx.check(
            "release lever lifts upward when actuated",
            raised_lever_aabb[0][2] > rest_lever_aabb[0][2] + 0.015,
            details=f"rest={rest_lever_aabb}, raised={raised_lever_aabb}",
        )

    ctx.check(
        "lead screw handle uses continuous rotation",
        handle_joint.joint_type == ArticulationType.CONTINUOUS
        and handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower is None
        and handle_joint.motion_limits.upper is None,
        details=f"joint_type={handle_joint.joint_type}, limits={handle_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
