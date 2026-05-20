from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _tube_shell(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 56,
):
    half_length = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half_length),
            (outer_radius, half_length),
        ],
        [
            (inner_radius, -half_length),
            (inner_radius, half_length),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _shade_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.012, 0.020),
            (0.026, 0.016),
            (0.050, 0.002),
            (0.076, -0.032),
            (0.097, -0.074),
            (0.110, -0.112),
        ],
        [
            (0.006, 0.014),
            (0.018, 0.010),
            (0.040, -0.002),
            (0.066, -0.034),
            (0.086, -0.074),
            (0.100, -0.106),
        ],
        segments=80,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.71, 0.73, 0.75, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.83, 0.84, 0.86, 1.0))
    enamel_white = model.material("enamel_white", rgba=(0.94, 0.94, 0.91, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    warm_black = model.material("warm_black", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=graphite,
        name="base_disc",
    )
    base.visual(
        _save_mesh("base_plinth_ring.obj", _tube_shell(0.055, 0.021, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=warm_black,
        name="base_plinth",
    )
    base.visual(
        _save_mesh("lower_receiver_shell.obj", _tube_shell(0.022, 0.0185, 0.900)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=brushed_steel,
        name="lower_receiver",
    )
    base.visual(
        _save_mesh("receiver_collar_shell.obj", _tube_shell(0.031, 0.021, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        material=satin_aluminum,
        name="receiver_collar",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.043, 0.0, 0.915), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_black,
        name="collar_knob",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.030, 0.0, 0.915), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_black,
        name="collar_boss",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.950),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.0185, length=1.050),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=brushed_steel,
        name="upper_pole",
    )
    upper_stage.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=satin_aluminum,
        name="stop_collar",
    )
    upper_stage.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
        material=satin_aluminum,
        name="arm_socket",
    )
    upper_stage.visual(
        Cylinder(radius=0.010, length=0.230),
        origin=Origin(xyz=(0.115, 0.0, 0.995), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="arm_tube",
    )
    upper_stage.visual(
        Box((0.018, 0.024, 0.028)),
        origin=Origin(xyz=(0.225, 0.0, 0.995)),
        material=satin_aluminum,
        name="hinge_mount",
    )
    upper_stage.visual(
        Box((0.020, 0.008, 0.036)),
        origin=Origin(xyz=(0.233, -0.014, 0.992)),
        material=warm_black,
        name="fork_left",
    )
    upper_stage.visual(
        Box((0.020, 0.008, 0.036)),
        origin=Origin(xyz=(0.233, 0.014, 0.992)),
        material=warm_black,
        name="fork_right",
    )
    upper_stage.visual(
        Cylinder(radius=0.0072, length=0.034),
        origin=Origin(xyz=(0.246, 0.0, 0.992), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_black,
        name="pivot_pin",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((0.310, 0.080, 1.100)),
        mass=2.3,
        origin=Origin(xyz=(0.120, 0.0, 0.550)),
    )

    shade = model.part("shade")
    shade.visual(
        _save_mesh("hinge_barrel_shell.obj", _tube_shell(0.011, 0.007, 0.030)),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_black,
        name="hinge_barrel",
    )
    shade.visual(
        Cylinder(radius=0.008, length=0.055),
        origin=Origin(xyz=(0.038, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_black,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.081, 0.0, 0.012)),
        material=warm_black,
        name="crown_cap",
    )
    shade.visual(
        _save_mesh("shade_shell.obj", _shade_shell()),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=enamel_white,
        name="dome_shell",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.118, 0.0, -0.012)),
        material=graphite,
        name="lamp_socket",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.240, 0.220, 0.170)),
        mass=0.95,
        origin=Origin(xyz=(0.118, 0.0, -0.040)),
    )

    model.articulation(
        "pole_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.320,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=shade,
        origin=Origin(xyz=(0.246, 0.0, 0.992)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.850,
            upper=0.550,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    upper_stage = object_model.get_part("upper_stage")
    shade = object_model.get_part("shade")
    pole_extension = object_model.get_articulation("pole_extension")
    shade_tilt = object_model.get_articulation("shade_tilt")

    base_disc = base.get_visual("base_disc")
    lower_receiver = base.get_visual("lower_receiver")
    receiver_collar = base.get_visual("receiver_collar")
    upper_pole = upper_stage.get_visual("upper_pole")
    stop_collar = upper_stage.get_visual("stop_collar")
    arm_tube = upper_stage.get_visual("arm_tube")
    pivot_pin = upper_stage.get_visual("pivot_pin")
    dome_shell = shade.get_visual("dome_shell")
    hinge_barrel = shade.get_visual("hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        base,
        upper_stage,
        elem_a=lower_receiver,
        elem_b=upper_pole,
        reason="The telescoping lamp uses a nested sliding sleeve, so the upper pole occupies the receiver bore by design.",
    )
    ctx.allow_overlap(
        shade,
        upper_stage,
        elem_a=hinge_barrel,
        elem_b=pivot_pin,
        reason="The hinge pin is modeled with negligible interference inside the barrel so the shade remains mechanically captured through the full tilt range.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        upper_stage,
        base,
        axes="xy",
        inner_elem=upper_pole,
        outer_elem=lower_receiver,
        name="upper_pole_stays_concentric_in_receiver",
    )
    ctx.expect_contact(
        upper_stage,
        base,
        elem_a=upper_pole,
        elem_b=lower_receiver,
        contact_tol=0.001,
        name="upper_pole_bears_in_receiver",
    )
    ctx.expect_gap(
        shade,
        upper_stage,
        axis="x",
        positive_elem=dome_shell,
        negative_elem=upper_pole,
        min_gap=0.040,
        name="shade_projects_forward_of_pole",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        positive_elem=dome_shell,
        negative_elem=base_disc,
        min_gap=0.750,
        name="shade_hangs_above_disc_base",
    )
    ctx.expect_gap(
        upper_stage,
        base,
        axis="z",
        positive_elem=stop_collar,
        negative_elem=receiver_collar,
        min_gap=0.020,
        name="upper_stage_trim_collar_stays_above_receiver_clamp",
    )
    ctx.expect_contact(
        shade,
        upper_stage,
        elem_a=hinge_barrel,
        elem_b=pivot_pin,
        contact_tol=0.001,
        name="shade_hinge_sleeve_contacts_pivot_pin",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    rest_dome_aabb = ctx.part_element_world_aabb(shade, elem="dome_shell")

    pole_limits = pole_extension.motion_limits
    assert pole_limits is not None
    assert pole_limits.lower is not None
    assert pole_limits.upper is not None

    with ctx.pose({pole_extension: pole_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pole_extension_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="pole_extension_lower_no_floating")
        ctx.expect_contact(
            upper_stage,
            base,
            elem_a=upper_pole,
            elem_b=lower_receiver,
            contact_tol=0.001,
            name="pole_extension_lower_contact",
        )

    with ctx.pose({pole_extension: pole_limits.upper}):
        extended_shade_pos = ctx.part_world_position(shade)
        ctx.check(
            "pole_extension_raises_shade",
            rest_shade_pos is not None
            and extended_shade_pos is not None
            and extended_shade_pos[2] > rest_shade_pos[2] + 0.25,
            "Prismatic extension should raise the lamp head by roughly the extension travel.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="pole_extension_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="pole_extension_upper_no_floating")
        ctx.expect_within(
            upper_stage,
            base,
            axes="xy",
            inner_elem=upper_pole,
            outer_elem=lower_receiver,
            name="pole_extension_upper_concentric",
        )
        ctx.expect_contact(
            upper_stage,
            base,
            elem_a=upper_pole,
            elem_b=lower_receiver,
            contact_tol=0.001,
            name="pole_extension_upper_contact",
        )
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            positive_elem=dome_shell,
            negative_elem=base_disc,
            min_gap=1.050,
            name="extended_lamp_lifts_shade_higher",
        )

    tilt_limits = shade_tilt.motion_limits
    assert tilt_limits is not None
    assert tilt_limits.lower is not None
    assert tilt_limits.upper is not None

    with ctx.pose({shade_tilt: tilt_limits.lower}):
        raised_dome_aabb = ctx.part_element_world_aabb(shade, elem="dome_shell")
        ctx.check(
            "shade_tilt_raises_shade_when_retracted",
            rest_dome_aabb is not None
            and raised_dome_aabb is not None
            and raised_dome_aabb[1][2] > rest_dome_aabb[1][2] + 0.04,
            "One tilt extreme should raise the dome crown above the rest pose.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="shade_tilt_lower_no_floating")
        ctx.expect_contact(
            shade,
            upper_stage,
            elem_a=hinge_barrel,
            elem_b=pivot_pin,
            contact_tol=0.001,
            name="shade_tilt_lower_hinge_contact",
        )
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            positive_elem=dome_shell,
            negative_elem=base_disc,
            min_gap=0.620,
            name="tilted_shade_still_clears_disc_base",
        )

    with ctx.pose({shade_tilt: tilt_limits.upper}):
        lowered_dome_aabb = ctx.part_element_world_aabb(shade, elem="dome_shell")
        ctx.check(
            "shade_tilt_moves_head_downward",
            rest_dome_aabb is not None
            and lowered_dome_aabb is not None
            and lowered_dome_aabb[0][2] < rest_dome_aabb[0][2] - 0.04,
            "The opposite tilt extreme should aim the shade downward.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="shade_tilt_upper_no_floating")
        ctx.expect_contact(
            shade,
            upper_stage,
            elem_a=hinge_barrel,
            elem_b=pivot_pin,
            contact_tol=0.001,
            name="shade_tilt_upper_hinge_contact",
        )
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            positive_elem=dome_shell,
            negative_elem=base_disc,
            min_gap=0.620,
            name="down_tilt_keeps_shade_above_base",
        )

    with ctx.pose({pole_extension: pole_limits.upper, shade_tilt: tilt_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            positive_elem=dome_shell,
            negative_elem=base_disc,
            min_gap=0.900,
            name="extended_and_tilted_shade_still_clears_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
