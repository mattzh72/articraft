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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _extrude_shell_profile_along_x(
    profile_yz: list[tuple[float, float]],
    *,
    width: float,
    mesh_name: str,
):
    profile_xy = [(-z, y) for y, z in profile_yz]
    geom = ExtrudeGeometry(profile_xy, width, cap=True, center=True, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_balcony_grill")

    stand_finish = model.material("stand_finish", rgba=(0.12, 0.12, 0.12, 1.0))
    shell_finish = model.material("shell_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.74, 0.76, 0.78, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.56, 0.57, 0.58, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.38, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_finish,
        name="base_crossbar_x",
    )
    stand.visual(
        Box((0.06, 0.28, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_finish,
        name="base_crossbar_y",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=stand_finish,
        name="center_post",
    )
    stand.visual(
        Box((0.12, 0.10, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material=stand_finish,
        name="top_saddle",
    )
    stand.visual(
        Box((0.30, 0.035, 0.020)),
        origin=Origin(xyz=(0.0, -0.045, 0.680)),
        material=stand_finish,
        name="rear_support_arm",
    )
    stand.visual(
        Box((0.30, 0.035, 0.020)),
        origin=Origin(xyz=(0.0, 0.045, 0.680)),
        material=stand_finish,
        name="front_support_arm",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.38, 0.28, 0.72)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    body = model.part("body")
    cookbox_profile = [
        (-0.175, 0.104),
        (-0.172, 0.090),
        (-0.164, 0.070),
        (-0.150, 0.045),
        (-0.125, 0.020),
        (-0.085, 0.006),
        (-0.030, 0.001),
        (0.030, 0.001),
        (0.085, 0.006),
        (0.125, 0.020),
        (0.150, 0.045),
        (0.164, 0.070),
        (0.172, 0.090),
        (0.175, 0.104),
        (0.165, 0.095),
        (0.161, 0.081),
        (0.152, 0.060),
        (0.136, 0.036),
        (0.110, 0.015),
        (0.075, 0.008),
        (0.000, 0.008),
        (-0.075, 0.008),
        (-0.110, 0.015),
        (-0.136, 0.036),
        (-0.152, 0.060),
        (-0.161, 0.081),
        (-0.165, 0.095),
    ]
    body.visual(
        _extrude_shell_profile_along_x(
            cookbox_profile,
            width=0.50,
            mesh_name="compact_balcony_grill_cookbox_shell",
        ),
        material=shell_finish,
        name="cookbox_shell",
    )
    body.visual(
        Box((0.14, 0.10, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=panel_finish,
        name="mount_block",
    )
    body.visual(
        Box((0.062, 0.082, 0.060)),
        origin=Origin(xyz=(0.279, 0.0, 0.070)),
        material=panel_finish,
        name="control_housing",
    )
    body.visual(
        Box((0.178, 0.040, 0.018)),
        origin=Origin(xyz=(0.159, 0.0, 0.018)),
        material=panel_finish,
        name="control_support_arm",
    )
    body.visual(
        Box((0.022, 0.040, 0.040)),
        origin=Origin(xyz=(0.248, 0.0, 0.034)),
        material=panel_finish,
        name="control_riser",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.56, 0.38, 0.13)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    lid = model.part("lid")
    lid_profile = [
        (0.000, 0.000),
        (0.010, 0.030),
        (0.030, 0.065),
        (0.065, 0.100),
        (0.115, 0.132),
        (0.175, 0.153),
        (0.240, 0.158),
        (0.292, 0.145),
        (0.325, 0.110),
        (0.342, 0.060),
        (0.350, 0.018),
        (0.341, 0.016),
        (0.326, 0.055),
        (0.298, 0.089),
        (0.255, 0.108),
        (0.200, 0.115),
        (0.140, 0.107),
        (0.090, 0.087),
        (0.050, 0.060),
        (0.022, 0.034),
        (0.009, 0.009),
    ]
    lid.visual(
        _extrude_shell_profile_along_x(
            lid_profile,
            width=0.50,
            mesh_name="compact_balcony_grill_lid_shell",
        ),
        material=shell_finish,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(xyz=(-0.078, 0.245, 0.133)),
        material=handle_finish,
        name="handle_post_left",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(xyz=(0.078, 0.245, 0.133)),
        material=handle_finish,
        name="handle_post_right",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.0, 0.245, 0.158), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_finish,
        name="handle_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.50, 0.36, 0.18)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.175, 0.090)),
    )

    knob = model.part("temperature_knob")
    knob.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_finish,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob_dial",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.048, 0.045, 0.045)),
        mass=0.08,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_body",
        ArticulationType.FIXED,
        parent=stand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.704)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.175, 0.102)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_temperature_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.310, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
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

    stand = object_model.get_part("stand")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("temperature_knob")

    lid_joint = object_model.get_articulation("body_to_lid")
    knob_joint = object_model.get_articulation("body_to_temperature_knob")

    body_shell = body.get_visual("cookbox_shell")
    mount_block = body.get_visual("mount_block")
    control_housing = body.get_visual("control_housing")
    lid_shell = lid.get_visual("lid_shell")
    knob_shaft = knob.get_visual("knob_shaft")

    lid_limits = lid_joint.motion_limits
    ctx.check(
        "rear hinge uses a realistic x-axis lid rotation",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and lid_joint.axis == (1.0, 0.0, 0.0)
        and lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper >= math.radians(95.0),
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}, limits={lid_limits}",
    )
    ctx.check(
        "temperature knob spins on a side-facing shaft",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            body,
            stand,
            axis="z",
            max_gap=0.0025,
            max_penetration=0.001,
            positive_elem=mount_block,
            negative_elem="top_saddle",
            name="cookbox sits on the narrow stand saddle",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.28,
            elem_a=lid_shell,
            elem_b=body_shell,
            name="closed lid covers the cookbox footprint",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a=knob_shaft,
            elem_b=control_housing,
            name="temperature knob shaft seats on the side control housing",
        )
        ctx.expect_origin_gap(
            knob,
            body,
            axis="x",
            min_gap=0.30,
            max_gap=0.33,
            name="temperature knob remains mounted on the right side panel",
        )

        closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip")

    with ctx.pose({lid_joint: lid_limits.upper if lid_limits is not None else math.radians(105.0)}):
        open_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip")

    ctx.check(
        "lid handle swings up and rearward when the grill opens",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.03
        and open_handle_aabb[1][1] < closed_handle_aabb[0][1] - 0.30,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
